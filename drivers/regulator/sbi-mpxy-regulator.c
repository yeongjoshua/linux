// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RISC-V SBI MPXY Based Regulator Driver
 *
 * Copyright (C) 2023-2024 Shanghai StarFive Technology Co., Ltd.
 *
 * Implements a regulator driver on top of SBI RPMI Proxy Extension (MPXY)
 *
 * Each SBI MPXY regulator instance is associated, through the means of a proper DT
 * entry description, to a specific Transport ID.
 */

#define pr_fmt(fmt) "sbi-mpxy-regulator: " fmt

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/linear_range.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <asm/sbi.h>
#include <asm/rpmi.h>

#define ATTR_COUNT(t, f)	(t - f + 1)

#define SBI_MPXY_REG_NAME_LEN	16

#define VOLTAGE_FORMAT_MASK	GENMASK(3, 1)
#define ALWAYS_ON_MASK		BIT(0)

#define MULTI_LINEAR_RANGE_SIZE	4

static int rpmi_to_linux_error(int rpmi_error)
{
	switch(rpmi_error) {
	case RPMI_SUCCESS:
		return 0;
	case RPMI_ERROR_DENIED:
		return -EPERM;
	case RPMI_ERROR_BUSY:
		return -EBUSY;
	case RPMI_ERROR_HW_FAULT:
		return -EFAULT;
	case RPMI_ERROR_TIMEOUT:
		return -ETIMEDOUT;
	case RPMI_ERROR_COMMS:
		return -ECOMM;
	case RPMI_ERROR_ALREADY:
		return -EALREADY;
	case RPMI_ERROR_OUT_OF_RANGE:
	case RPMI_ERROR_OUT_OF_RESOURCE:
	case RPMI_ERROR_INVALID_PARAM:
		return -EINVAL;
	case RPMI_ERROR_EXTENSION:
	case RPMI_ERROR_NOT_FOUND:
	case RPMI_ERROR_NOT_SUPPORTED:
	case RPMI_ERROR_FAILED:
	default:
		return -EOPNOTSUPP;
	}
}

/* regulator control */
enum mpxy_domain_config {
	MPXY_VOL_DISABLE = 0,
	MPXY_VOL_ENABLE = 1,
};

/* regulator voltage types */
enum mpxy_voltage_type {
	MPXY_VOL_FIXED = 0,
	MPXY_VOL_SIMPLE_LINEAR = 1,
	MPXY_VOL_MULTI_LINEAR = 2,
	MPXY_VOL_DISCRETE = 3,
	MPXY_VOL_TYPE_MAX_IDX,
};

struct sbi_mpxy_ctx {
	/* transport id */
	u32 channel_id;
	u32 max_msg_len;
};

struct sbi_mpxy_voltage_config {
	/* simple_linear */
#define MPXY_SIMPLE_LINEAR_VOLTAGE_MIN		0
#define MPXY_SIMPLE_LINEAR_VOLTAGE_MAX		1
#define MPXY_SIMPLE_LINEAR_VOLTAGE_STEP		2
	/* multi_linear */
#define MPXY_MULTI_LINEAR_VOLTAGE_MIN		0
#define MPXY_MULTI_LINEAR_VOLTAGE_MIN_SEL	1
#define MPXY_MULTI_LINEAR_VOLTAGE_MAX_SEL	2
#define MPXY_MULTI_LINEAR_VOLTAGE_STEP		3
	int returned_levels;
	u32 *levels_uv;
};

/**
 * struct sbi_mpxy_reg - describe one available MPXY Voltage Domain
 *
 * @id: the domain ID as advertised by the PuC
 * @voltage_format: types of voltage regulator
 *		    - fixed voltage
 *		    - simple linear voltage containing a range of
 *		      voltages: <min_uV>, <max_uV>, <step_uV>
 *		    - multi-linear voltage containing multiple range of
 *		      voltages: <min_uV>, <min_sel>, <max_sel>, <step_uV>
 *		    - discrete voltage range
 * @always_on: regulator is always on.
 * @num_levels: number of available voltage levels for this domain advertised
 *              by the PuC. This value is dependent on @voltage_format.
 * @transition_latency: max time allowed for voltage change to stabilize
 * @name: voltage domain name assigned by the PuC
 * @dev: device associated with the regulator in this domain
 */
struct sbi_mpxy_reg {
	u32 id;
	u32 voltage_format:2;
	u32 always_on:1;
	u32 num_levels;
	u32 transition_latency;
	char name[SBI_MPXY_REG_NAME_LEN];
	struct sbi_mpxy_ctx *mpxy_ctx;
	struct sbi_mpxy_voltage_config vcfg;
	struct device *dev;
	struct regulator_dev *rdev;
	struct regulator_desc desc;
	struct regulator_config conf;
	struct regulator_init_data *reg_init_data;
};

/* number of voltage domains response data */
/* Service ID: RPMI_VOLTAGE_SRV_GET_NUM_DOMAINS */
struct rpmi_get_num_domain_rx {
	s32 status;
	u32 num_domains;
};

/* voltage domain attributes request data */
/* Service ID: RPMI_VOLTAGE_SRV_GET_ATTRIBUTES */
struct rpmi_get_domain_attrs_tx {
	u32 domain_id;
};

/* voltage domain attributes response data */
/* Service ID: RPMI_VOLTAGE_SRV_GET_ATTRIBUTES */
struct rpmi_get_domain_attrs_rx {
	s32 status;
	u32 flags;
#define REG_VOLTAGE_FORMAT(f)	(FIELD_GET(VOLTAGE_FORMAT_MASK, (f)))
#define REG_ALWAYS_ON(f)	(FIELD_GET(ALWAYS_ON_MASK, (f)))
	u32 num_levels;
	u32 transition_latency;
	char name[SBI_MPXY_REG_NAME_LEN];
};

/* number of voltage levels in a domain request data */
/* Service ID: RPMI_VOLTAGE_SRV_GET_SUPPORTED_LEVELS */
struct rpmi_get_domain_levels_tx {
	u32 domain_id;
	u32 level_index;
};

/* number of voltage levels in a domain response data */
/* Service ID: RPMI_VOLTAGE_SRV_GET_SUPPORTED_LEVELS */
struct rpmi_get_domain_levels_rx {
	s32 status;
	u32 flags;
	u32 remaining_items;
	u32 returned_items;
	s32 voltage[];
};

struct rpmi_set_domain_config_tx {
	u32 domain_id;
	u32 config;
};

struct rpmi_set_domain_config_rx {
	s32 status;
};

struct rpmi_get_domain_config_tx {
	u32 domain_id;
};

struct rpmi_get_domain_config_rx {
	s32 status;
	u32 config;
};

struct rpmi_set_voltage_level_tx {
	u32 domain_id;
	s32 voltage_level;
};

struct rpmi_set_voltage_level_rx {
	s32 status;
};

struct rpmi_get_voltage_level_tx {
	u32 domain_id;
};

struct rpmi_get_voltage_level_rx {
	s32 status;
	s32 voltage_level;
};

static int sbi_mpxy_reg_get_config(struct regulator_dev *rdev)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_get_domain_config_tx tx;
	struct rpmi_get_domain_config_rx rx;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_GET_DOMAIN_CONFIG,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret)
		return ret;
	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return rx.config;
}

static int sbi_mpxy_reg_set_config(struct regulator_dev *rdev,
				   unsigned int config)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_set_domain_config_tx tx;
	struct rpmi_set_domain_config_rx rx;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	tx.config = cpu_to_le32(config);
	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_SET_DOMAIN_CONFIG,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret)
		return ret;

	return rx.status;
}

static int sbi_mpxy_reg_is_enabled(struct regulator_dev *rdev)
{
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);
	int ret;

	ret = sbi_mpxy_reg_get_config(rdev);
	if (ret < 0)
		return ret;

	dev_dbg(mpxy_reg->dev,
		"check voltage domain #%d is enabled, returns config=%d",
		 mpxy_reg->id, ret);

	return ret & MPXY_VOL_ENABLE;
}

static int sbi_mpxy_reg_enable(struct regulator_dev *rdev)
{
	int config;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	if (mpxy_reg->always_on)
		return -EOPNOTSUPP;

	config = MPXY_VOL_ENABLE;

	return sbi_mpxy_reg_set_config(rdev, config);
}

static int sbi_mpxy_reg_disable(struct regulator_dev *rdev)
{
	int config;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	if (mpxy_reg->always_on)
		return -EOPNOTSUPP;

	config = MPXY_VOL_DISABLE;

	return sbi_mpxy_reg_set_config(rdev, config);
}

static int sbi_mpxy_reg_get_domain_levels(struct sbi_mpxy_reg *mpxy_reg)
{
	int ret;
	int val;
	int num_remaining;
	int write_index;
	unsigned long rxmsg_len;
	unsigned int num_levels = 0;
	struct rpmi_get_domain_levels_tx tx;
	struct rpmi_get_domain_levels_rx *rx;

	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	rx = devm_kcalloc(mpxy_reg->dev, mpxy_reg->mpxy_ctx->max_msg_len,
			       sizeof(u32), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	while (num_levels < mpxy_reg->num_levels) {
		num_remaining = mpxy_reg->num_levels - num_levels;
		tx.level_index = cpu_to_le32(num_levels);

		ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
						     RPMI_VOLTAGE_SRV_GET_SUPPORTED_LEVELS,
						     &tx, sizeof(tx),
						     rx, &rxmsg_len);

		if (ret) {
			dev_err(mpxy_reg->dev,
				"get domain #%d voltage levels failed with error: %d",
				 mpxy_reg->id, ret);
			return ret;
		}

		if (rx->status) {
			dev_err(mpxy_reg->dev,
				"get domain #%d voltage levels failed with RPMI error: %d",
				 mpxy_reg->id, rx->status);
			return rpmi_to_linux_error(rx->status);
		}

		write_index = num_levels;
		num_levels += rx->returned_items;
		if (num_levels > mpxy_reg->num_levels) {
			num_levels = mpxy_reg->num_levels;
			if (rx->remaining_items) {
				dev_err(mpxy_reg->dev,
					"remaining levels: %d but number of levels is met\n",
					 rx->remaining_items);
				return -EINVAL;
			}
		}

		for (int i = 0; i < rx->returned_items; i++) {
			val = le32_to_cpu(rx->voltage[i]);
			mpxy_reg->vcfg.levels_uv[write_index + i] = val;
		}
	}

	mpxy_reg->vcfg.returned_levels = num_levels;

	return rx->status;
}

static int sbi_mpxy_reg_get_voltage(struct regulator_dev *rdev)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_get_voltage_level_tx tx;
	struct rpmi_get_voltage_level_rx rx;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_GET_LEVEL,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret)
		return ret;
	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return rx.voltage_level;
}

static int sbi_mpxy_reg_set_voltage(struct regulator_dev *rdev,
				    unsigned int voltage_level)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_set_voltage_level_tx tx;
	struct rpmi_set_voltage_level_rx rx;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	dev_dbg(mpxy_reg->dev, "set domain #%d to voltage level: %u",
		mpxy_reg->id, voltage_level);
	tx.voltage_level = cpu_to_le32(voltage_level);
	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_SET_LEVEL,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret)
		return ret;

	return rx.status;
}

static int sbi_mpxy_reg_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret;
	unsigned long rxmsg_len;
	s32 volt_uV;
	struct rpmi_get_voltage_level_tx tx;
	struct rpmi_get_voltage_level_rx rx;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_GET_LEVEL,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret)
		return ret;
	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	volt_uV = rx.voltage_level;

	return mpxy_reg->desc.ops->map_voltage(rdev, volt_uV, volt_uV);
}

static int sbi_mpxy_reg_set_voltage_sel(struct regulator_dev *rdev,
					unsigned int selector)
{
	s32 volt_uV;
	struct sbi_mpxy_reg *mpxy_reg = rdev_get_drvdata(rdev);

	volt_uV = mpxy_reg->desc.ops->list_voltage(rdev, selector);
	if (volt_uV <= 0)
		return -EINVAL;

	return sbi_mpxy_reg_set_voltage(rdev, volt_uV);
}

static const struct regulator_ops sbi_mpxy_reg_fixed_ops = {
	.enable = sbi_mpxy_reg_enable,
	.disable = sbi_mpxy_reg_disable,
	.is_enabled = sbi_mpxy_reg_is_enabled,
	.get_voltage = sbi_mpxy_reg_get_voltage,
};

static const struct regulator_ops sbi_mpxy_reg_simple_linear_ops = {
	.enable = sbi_mpxy_reg_enable,
	.disable = sbi_mpxy_reg_disable,
	.is_enabled = sbi_mpxy_reg_is_enabled,
	.get_voltage_sel = sbi_mpxy_reg_get_voltage_sel,
	.set_voltage_sel = sbi_mpxy_reg_set_voltage_sel,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
};

static const struct regulator_ops sbi_mpxy_reg_multi_linear_ops = {
	.enable = sbi_mpxy_reg_enable,
	.disable = sbi_mpxy_reg_disable,
	.is_enabled = sbi_mpxy_reg_is_enabled,
	.get_voltage_sel = sbi_mpxy_reg_get_voltage_sel,
	.set_voltage_sel = sbi_mpxy_reg_set_voltage_sel,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
};

static const struct regulator_ops sbi_mpxy_reg_discrete_ops = {
	.enable = sbi_mpxy_reg_enable,
	.disable = sbi_mpxy_reg_disable,
	.is_enabled = sbi_mpxy_reg_is_enabled,
	.get_voltage_sel = sbi_mpxy_reg_get_voltage_sel,
	.set_voltage_sel = sbi_mpxy_reg_set_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
};

static int
mpxy_config_simple_linear_regulator_mappings(struct sbi_mpxy_reg *mpxy_reg)
{
	s32 delta_uV;
	struct regulator_init_data *init_data;
	struct sbi_mpxy_voltage_config *vcfg = &mpxy_reg->vcfg;

	/*
	 * Note that MPXY voltage domains describable by simple linear range
	 * {min, max, step} comes in one single triplet as defined in the
	 * MPXY Voltage Service Group protocol.
	 */

	delta_uV = (vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MAX] -
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MIN]);

	/* Rule out buggy voltage-intervals answers from PuC */
	if (delta_uV <= 0) {
		dev_err(mpxy_reg->dev,
			"Invalid volt-range %d-%duV for simple-linear domain %d\n",
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MIN],
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MAX],
			mpxy_reg->id);
		return -EINVAL;
	}

	init_data = devm_kzalloc(mpxy_reg->dev,
				 sizeof(struct regulator_init_data),
				 GFP_KERNEL);
	if (!init_data)
		return -ENOMEM;

	/* One simple linear mapping. */
	mpxy_reg->desc.min_uV =
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MIN];
	mpxy_reg->desc.uV_step =
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_STEP];
	mpxy_reg->desc.linear_min_sel = 0;
	mpxy_reg->desc.continuous_voltage_range = true;
	mpxy_reg->desc.n_voltages = (delta_uV / mpxy_reg->desc.uV_step) + 1;

	init_data->constraints.min_uV = mpxy_reg->desc.min_uV;
	init_data->constraints.max_uV =
			vcfg->levels_uv[MPXY_SIMPLE_LINEAR_VOLTAGE_MAX];
	init_data->constraints.uV_offset = mpxy_reg->desc.uV_step;
	init_data->constraints.always_on = mpxy_reg->always_on;

	mpxy_reg->conf.init_data = init_data;

	mpxy_reg->desc.ops = &sbi_mpxy_reg_simple_linear_ops;

	return 0;
}

static int
mpxy_config_multi_linear_regulator_mappings(struct sbi_mpxy_reg *mpxy_reg)
{
	int num_ranges;
	int num_levels;
	int range_start;
	struct linear_range *linear_ranges;
	struct sbi_mpxy_voltage_config *vcfg = &mpxy_reg->vcfg;

	/*
	 * Note that MPXY voltage domains describable by multiple linear range
	 * {min, min_sel, max_sel, step} comes with multiple 4-tuples as defined
	 * in the MPXY Voltage Service Group protocol.
	 */

	num_levels = mpxy_reg->vcfg.returned_levels;

	/* Rule out buggy multi-linear voltage levels answers from PuC */
	if (num_levels % MULTI_LINEAR_RANGE_SIZE) {
		dev_err(mpxy_reg->dev,
			"Invalid voltage levels returned: %d for multi-linear domain %d\n",
			num_levels,
			mpxy_reg->id);
		return -EINVAL;
	}

	/* multi linear mapping. */
	num_ranges = num_levels / MULTI_LINEAR_RANGE_SIZE;
	linear_ranges = devm_kcalloc(mpxy_reg->dev,
				     num_ranges,
				     sizeof(struct linear_range),
				     GFP_KERNEL);
	if (!linear_ranges)
		return -ENOMEM;

	for (int i = 0; i < num_ranges; i++) {
		range_start = i * MULTI_LINEAR_RANGE_SIZE;
		linear_ranges[i].min = vcfg->levels_uv[range_start];
		linear_ranges[i].min_sel = vcfg->levels_uv[range_start + 1];
		linear_ranges[i].max_sel = vcfg->levels_uv[range_start + 2];
		linear_ranges[i].step = vcfg->levels_uv[range_start + 3];
	}

	mpxy_reg->desc.continuous_voltage_range = true;
	mpxy_reg->desc.linear_ranges = linear_ranges;
	mpxy_reg->desc.n_linear_ranges = num_ranges;
	mpxy_reg->desc.n_voltages = linear_ranges[num_ranges - 1].max_sel;

	mpxy_reg->desc.ops = &sbi_mpxy_reg_multi_linear_ops;

	return 0;
}

static int
mpxy_config_discrete_regulator_mappings(struct sbi_mpxy_reg *mpxy_reg)
{
	struct sbi_mpxy_voltage_config *vcfg = &mpxy_reg->vcfg;

	/* Discrete non linear levels are mapped to volt_table */
	mpxy_reg->desc.n_voltages = vcfg->returned_levels;

	if (mpxy_reg->desc.n_voltages > 1) {
		mpxy_reg->desc.volt_table = (const unsigned int *)vcfg->levels_uv;
		mpxy_reg->desc.ops = &sbi_mpxy_reg_discrete_ops;
	} else {
		mpxy_reg->desc.fixed_uV = vcfg->levels_uv[0];
		mpxy_reg->desc.ops = &sbi_mpxy_reg_fixed_ops;
	}

	return 0;
}

static int sbi_mpxy_regulator_common_init(struct sbi_mpxy_reg *mpxy_reg)
{
	int ret;
	struct device *dev = mpxy_reg->dev;
	struct sbi_mpxy_voltage_config *vcfg = &mpxy_reg->vcfg;

	mpxy_reg->desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s", mpxy_reg->name);

	mpxy_reg->desc.id = mpxy_reg->id;
	mpxy_reg->desc.type = REGULATOR_VOLTAGE;
	mpxy_reg->desc.owner = THIS_MODULE;
	mpxy_reg->desc.regulators_node = "regulators";

	if (mpxy_reg->voltage_format == MPXY_VOL_FIXED) {
		mpxy_reg->desc.fixed_uV = vcfg->levels_uv[0];
		mpxy_reg->desc.n_voltages = 1;
		mpxy_reg->desc.ops = &sbi_mpxy_reg_fixed_ops;
	} else if (mpxy_reg->voltage_format == MPXY_VOL_SIMPLE_LINEAR) {
		ret = mpxy_config_simple_linear_regulator_mappings(mpxy_reg);
	} else if (mpxy_reg->voltage_format == MPXY_VOL_MULTI_LINEAR) {
		ret = mpxy_config_multi_linear_regulator_mappings(mpxy_reg);
	} else if (mpxy_reg->voltage_format == MPXY_VOL_DISCRETE) {
		ret = mpxy_config_discrete_regulator_mappings(mpxy_reg);
	}

	mpxy_reg->conf.dev = dev;
	/* Store for later retrieval via rdev_get_drvdata() */
	mpxy_reg->conf.driver_data = mpxy_reg;

	return ret;
}

/* obtain the MPXY voltage domain attributes */
static int sbi_mpxy_reg_get_attrs(u32 domain_id, struct sbi_mpxy_reg *mpxy_reg)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_get_domain_attrs_tx tx;
	struct rpmi_get_domain_attrs_rx rx;

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_reg->mpxy_ctx->channel_id,
					     RPMI_VOLTAGE_SRV_GET_ATTRIBUTES,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_reg->dev,
			"get atributes of voltage domain #%u failed with error: %d\n",
			 domain_id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_reg->dev,
			"get attributes of voltage domain #%u failed with RPMI error: %d\n",
			 domain_id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	mpxy_reg->voltage_format = REG_VOLTAGE_FORMAT(rx.flags);
	mpxy_reg->always_on = REG_ALWAYS_ON(rx.flags);
	mpxy_reg->num_levels = rx.num_levels;
	mpxy_reg->transition_latency = rx.transition_latency;
	strscpy(mpxy_reg->name, rx.name, SBI_MPXY_REG_NAME_LEN);

	if (mpxy_reg->voltage_format >= MPXY_VOL_TYPE_MAX_IDX) {
		dev_err(mpxy_reg->dev,
			"domain #%u - invalid voltage format: %u\n",
			 mpxy_reg->id, mpxy_reg->voltage_format);
		return -EINVAL;
	}

	switch (mpxy_reg->voltage_format) {
	case MPXY_VOL_FIXED:
		dev_dbg(mpxy_reg->dev,
			"domain #%u - fixed voltage regulator\n",
			 mpxy_reg->id);
		if (mpxy_reg->num_levels != 1) {
			dev_err(mpxy_reg->dev,
				"domain #%u - incorrect number of voltage levels: %u\n",
				 mpxy_reg->id, mpxy_reg->num_levels);
			return -ENODEV;
		}
		break;
	case MPXY_VOL_SIMPLE_LINEAR:
		dev_dbg(mpxy_reg->dev,
			"domain #%u - simple linear voltage regulator\n",
			 mpxy_reg->id);
		if (mpxy_reg->num_levels != 3) {
			dev_err(mpxy_reg->dev,
				"domain #%u - incorrect number of voltage levels: %u\n",
				 mpxy_reg->id, mpxy_reg->num_levels);
			return -ENODEV;
		}
		break;
	case MPXY_VOL_MULTI_LINEAR:
		dev_dbg(mpxy_reg->dev,
			"domain #%u - multi linear voltage regulator\n",
			 mpxy_reg->id);
		if (mpxy_reg->num_levels == 0 ||
		    !((mpxy_reg->num_levels & 0x3) == 0)) {
			dev_err(mpxy_reg->dev,
				"domain #%u - incorrect number of voltage levels: %u\n",
				 mpxy_reg->id, mpxy_reg->num_levels);
			return -ENODEV;
		}
		break;
	case MPXY_VOL_DISCRETE:
		dev_dbg(mpxy_reg->dev, "domain #%u - discrete voltage regulator\n",
			mpxy_reg->id);
		if (mpxy_reg->num_levels == 0) {
			dev_err(mpxy_reg->dev,
				"domain #%u - incorrect number of voltage levels: %u\n",
				 mpxy_reg->id, mpxy_reg->num_levels);
			return -ENODEV;
		}
		break;
	default:
		dev_err(mpxy_reg->dev,
			"domain #%u - invalid regulator voltage type\n",
			 mpxy_reg->id);
		return -EINVAL;
	}

	mpxy_reg->vcfg.levels_uv = devm_kzalloc(mpxy_reg->dev,
						sizeof(u32) * rx.num_levels,
						GFP_KERNEL);
	if (!mpxy_reg->vcfg.levels_uv)
		return -ENOMEM;

	ret = sbi_mpxy_reg_get_domain_levels(mpxy_reg);
	if (ret)
		return ret;

	dev_dbg(mpxy_reg->dev,
		"domain_id=%d voltage_format=%x num_levels=%d name=%s\n",
		 mpxy_reg->id, mpxy_reg->voltage_format, rx.num_levels, rx.name);

	return 0;
}

static int sbi_mpxy_reg_get_num_domains(u32 channel_id)
{
	int ret;
	struct rpmi_get_num_domain_rx rx;

	ret = sbi_mpxy_send_message_withresp(channel_id,
					     RPMI_VOLTAGE_SRV_GET_NUM_DOMAINS,
					     NULL, 0, &rx, NULL);
	if (ret)
		return ret;

	/* RPMI message status code */
	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return rx.num_domains;
}

static int sbi_mpxy_reg_enumerate(struct sbi_mpxy_reg *mpxy_reg, u32 domain_id)
{
	int ret;

	ret = sbi_mpxy_reg_get_attrs(domain_id, mpxy_reg);
	if (ret)
		return ret;

	return 0;
}

static int sbi_mpxy_reg_probe(struct platform_device *pdev)
{
	u32 channel_id, i, attr_count, version;
	int ret, num_domains;
	struct sbi_mpxy_reg *mpxy_reg;
	struct sbi_mpxy_ctx *mpxy_ctx;
	u32 *attr_buf;
	struct of_phandle_args args;

	if (sbi_spec_version < sbi_mk_version(1, 0) ||
	    sbi_probe_extension(SBI_EXT_MPXY) <= 0) {
		dev_err(&pdev->dev, "sbi mpxy extension not present\n");
		return -ENODEV;
	}

	ret = of_parse_phandle_with_args(pdev->dev.of_node,
					 "mboxes", "#mbox-cells", 0, &args);
	if (ret) {
		dev_err(&pdev->dev, "Missing mboxes phandle\n");
		return ret;
	}

	if (args.args_count < 1) {
		dev_err(&pdev->dev, "mboxes args missing channel-id\n");
		of_node_put(args.np);
		return -EINVAL;
	}

	channel_id = args.args[0];

	attr_count = ATTR_COUNT(SBI_MPXY_ATTR_MSG_SEND_TIMEOUT,
				SBI_MPXY_ATTR_MSG_PROT_ID);

	attr_buf = devm_kzalloc(&pdev->dev, sizeof(u32) * attr_count,
				GFP_KERNEL);
	if (!attr_buf)
		return -ENOMEM;

	/**
	 * Read MPXY channel attributes.
	 * Channel attributes are different from RPMI clock attributes
	 */
	ret = sbi_mpxy_read_attrs(channel_id, SBI_MPXY_ATTR_MSG_PROT_ID,
				  attr_count, attr_buf);
	if (ret == -ENOTSUPP) {
		dev_err(&pdev->dev, "%u mpxy channel not available\n", channel_id);
		return -EPROBE_DEFER;
	}

	if (ret) {
		dev_err(&pdev->dev, "channel-%u: read attributes - %d\n", channel_id, ret);
		return ret;
	}

	if (attr_buf[0] != SBI_MPXY_MSGPROTO_RPMI_ID) {
		dev_err(&pdev->dev,
			"channel-%u: msgproto mismatch, expect:%u, found:%u\n",
			channel_id, SBI_MPXY_MSGPROTO_RPMI_ID, attr_buf[0]);
		return -EINVAL;
	}

	version = RPMI_MSGPROTO_VERSION(RPMI_MAJOR_VER, RPMI_MINOR_VER);
	if (attr_buf[1] != version) {
		dev_err(&pdev->dev,
			"channel-%u: msgproto version mismatch, expect:%u, found:%u\n",
			channel_id, version, attr_buf[1]);
		return -EINVAL;
	}

	mpxy_reg = devm_kcalloc(&pdev->dev, num_domains, sizeof(*mpxy_reg), GFP_KERNEL);
	if (!mpxy_reg)
		return -ENOMEM;

	mpxy_ctx = devm_kzalloc(&pdev->dev, sizeof(*mpxy_ctx), GFP_KERNEL);
	if (!mpxy_ctx)
		return -ENOMEM;

	mpxy_ctx->channel_id = channel_id;
	mpxy_ctx->max_msg_len = attr_buf[2];

	ret = sbi_mpxy_read_attrs(channel_id, SBI_MPXY_ATTR_MSGPROTO_ATTR_START,
				  1, attr_buf);
	if (ret) {
		dev_err(&pdev->dev, "channel-%u: read attributes - %d\n",
			channel_id, ret);
		return ret;
	}

	if (attr_buf[0] != RPMI_SRVGRP_VOLTAGE) {
		dev_err(&pdev->dev,
		"channel-%u ServiceGroup match failed, expected %x, found %x\n",
		channel_id, RPMI_SRVGRP_VOLTAGE, attr_buf[0]);
		return -EINVAL;
	}

	num_domains = sbi_mpxy_reg_get_num_domains(channel_id);
	if (num_domains <= 0) {
		if (!num_domains) {
			dev_err(&pdev->dev, "No MPXY voltage domains found!\n");
			num_domains = -EINVAL;
		} else {
			dev_err(&pdev->dev,
				"failed to get number of MPXY voltage domains - err:%d\n",
				 num_domains);
		}

		return num_domains;
	}


	for (i = 0; i < num_domains; i++, mpxy_reg++) {
		mpxy_reg->dev = &pdev->dev;
		mpxy_reg->mpxy_ctx = mpxy_ctx;
		mpxy_reg->id = i;

		ret = sbi_mpxy_reg_enumerate(mpxy_reg, i);
		if (ret)
			return ret;

		ret = sbi_mpxy_regulator_common_init(mpxy_reg);
		if (ret) {
			dev_err_probe(mpxy_reg->dev, ret,
				      "voltage domain #%d initialization failed\n",
				       mpxy_reg->id);
			return ret;
		}

		mpxy_reg->rdev = devm_regulator_register(mpxy_reg->dev,
							 &mpxy_reg->desc,
							 &mpxy_reg->conf);
		if (IS_ERR(mpxy_reg->rdev)) {
			mpxy_reg->rdev = NULL;
			dev_info(mpxy_reg->dev,
				 "devm_regulator_register for voltage domain #%d failed\n",
				  mpxy_reg->id);
			continue;
		}

		dev_info(mpxy_reg->dev,
			 "Regulator %s is registered for voltage domain #%d\n",
			  mpxy_reg->desc.name, mpxy_reg->id);
	}

	u32 events = 1;
	ret = sbi_mpxy_write_attrs(channel_id, SBI_MPXY_ATTR_EVENTS_STATE_CONTROL,
				  1, &events);

	dev_set_drvdata(&pdev->dev, mpxy_reg);
	return ret;
}

static const struct of_device_id sbi_mpxy_reg_of_match[] = {
	{ .compatible = "riscv,rpmi-voltage" },
	{ },
};

MODULE_DEVICE_TABLE(of, sbi_mpxy_reg_of_match);

#define DRIVER_NAME    "regulator-sbi-mpxy"

static struct platform_driver sbi_mpxy_reg_platdrv = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = sbi_mpxy_reg_of_match,
	},
	.probe = sbi_mpxy_reg_probe,
};

module_platform_driver(sbi_mpxy_reg_platdrv);

MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Regulator Driver based on SBI MPXY extension");
MODULE_LICENSE("GPL v2");
