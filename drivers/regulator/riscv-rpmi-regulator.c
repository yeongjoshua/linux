// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RISC-V RPMI Based Regulator Driver through SBI MPXY
 *
 * Copyright (C) 2025 Shanghai StarFive Technology Co., Ltd.
 *
 * Implements a regulator driver on top of SBI RPMI Message Proxy Extension (MPXY)
 *
 * Each SBI MPXY regulator instance is associated, through the means of a proper DT
 * entry description, to a specific Transport ID.
 */

#define pr_fmt(fmt) "riscv-rpmi-regulator: " fmt

#include <linux/bitfield.h>
#include <linux/mailbox/riscv-rpmi-message.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

#define RPMI_REG_DOMAIN_NAME_LEN	16

#define VOLTAGE_FORMAT_MASK		GENMASK(3, 1)
#define ALWAYS_ON_MASK			BIT(0)

#define MULTI_LINEAR_RANGE_SIZE	4

struct rpmi_ctx {
	struct mbox_chan *chan;
	struct mbox_client client;
	u32 max_msg_size;
};

struct rpmi_reg_level_discrete {
	u32 uvolt;
};

struct rpmi_reg_level_linear {
	u32 uvolt_min;
	u32 uvolt_max;
	u32 uvolt_step;
};

struct rpmi_reg_domain {
	u32 id;
	struct rpmi_ctx *rpmi_ctx;
	struct device *dev;
	struct regulator_desc desc;
	u32 voltage_format:2;
	u32 always_on:1;
	u32 num_levels;
	u32 transition_latency;
	u32 *level;
	char name[RPMI_REG_DOMAIN_NAME_LEN];
};

/* Service ID: RPMI_VOLTAGE_SRV_GET_NUM_DOMAINS */
struct rpmi_get_num_domain_rx {
	s32 status;
	u32 num_domains;
};

/* Service ID: RPMI_VOLTAGE_SRV_GET_ATTRIBUTES */
struct rpmi_get_domain_attrs_tx {
	u32 domain_id;
};

/* Service ID: RPMI_VOLTAGE_SRV_GET_SUPPORTED_LEVELS */
struct rpmi_get_supp_levels_tx {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_get_supp_levels_rx {
	s32 status;
	u32 flags;
	u32 remaining_items;
	u32 returned_items;
	u32 level[];
};

/* Service ID: RPMI_VOLTAGE_SRV_SET_CONFIG */
struct rpmi_set_config_tx {
	u32 domain_id;
	u32 config;
};

struct rpmi_set_config_rx {
	s32 status;
};

/* Service ID: RPMI_VOLTAGE_SRV_GET_CONFIG */
struct rpmi_get_config_tx {
	u32 domain_id;
};

struct rpmi_get_config_rx {
	s32 status;
	u32 config;
};

/* Service ID: RPMI_VOLTAGE_SRV_SET_LEVEL */
struct rpmi_set_level_tx {
	u32 domain_id;
	s32 level;
};

struct rpmi_set_level_rx {
	s32 status;
};

/* Service ID: RPMI_VOLTAGE_SRV_GET_LEVEL */
struct rpmi_get_level_tx {
	u32 domain_id;
};

struct rpmi_get_level_rx {
	s32 status;
	s32 level;
};

/* regulator control */
enum rpmi_domain_config {
	RPMI_VOLT_DISABLE = 0,
	RPMI_VOLT_ENABLE = 1,
};

struct rpmi_get_domain_attrs_rx {
	s32 status;
	u32 flags;
#define REG_VOLTAGE_FORMAT(f)	(FIELD_GET(GENMASK(3, 1), (f)))
#define REG_FORMAT_DISCRETE	0
#define REG_FORMAT_LINEAR	1
#define REG_ALWAYS_ON(f)	(FIELD_GET(BIT(0), (f)))
	u32 num_levels;
	u32 transition_latency;
	char name[RPMI_REG_DOMAIN_NAME_LEN];
};

static int rpmi_reg_get_num_domains(struct rpmi_ctx *mpxy_ctx, u32 *domain)
{
	struct rpmi_get_num_domain_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_GET_NUM_DOMAINS,
					  NULL, 0, &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*domain = rx.num_domains;

	return 0;
}

static int rpmi_reg_get_attrs(struct rpmi_reg_domain *mpxy_reg)
{
	struct rpmi_get_domain_attrs_tx tx;
	struct rpmi_get_domain_attrs_rx rx;
	struct rpmi_mbox_message msg;
	int ret, size;

	tx.domain_id = mpxy_reg->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_GET_ATTRIBUTES,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	mpxy_reg->voltage_format = REG_VOLTAGE_FORMAT(rx.flags);
	mpxy_reg->always_on = REG_ALWAYS_ON(rx.flags);
	// pr_info("%s - L%d - always_on? %s\n", __func__, __LINE__, REG_ALWAYS_ON(rx.flags) ? "yes": "no");

	mpxy_reg->num_levels = rx.num_levels;
	mpxy_reg->transition_latency = rx.transition_latency;
	strscpy(mpxy_reg->name, rx.name, RPMI_REG_DOMAIN_NAME_LEN);

	switch (mpxy_reg->voltage_format) {
	case REG_FORMAT_DISCRETE:
		size =  sizeof(struct rpmi_reg_level_discrete) * rx.num_levels;
		break;
	case REG_FORMAT_LINEAR:
		size =  sizeof(struct rpmi_reg_level_linear) * rx.num_levels;
		break;
	default:
		return -EINVAL;
	}

	mpxy_reg->level = devm_kzalloc(mpxy_reg->dev, size, GFP_KERNEL);
	if (!mpxy_reg->level)
		return -ENOMEM;

	return 0;
}

static int rpmi_reg_get_supported_levels(struct rpmi_reg_domain *mpxy_reg)
{
	struct rpmi_get_supp_levels_tx tx;
	struct rpmi_get_supp_levels_rx *rx;
	int ret = 0, size = 0, index = 0;
	struct rpmi_mbox_message msg;
	u32 *level, *data;
	int i;

	rx = kmalloc(mpxy_reg->rpmi_ctx->max_msg_size, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	tx.domain_id = mpxy_reg->id;
	level = mpxy_reg->level;

	// pr_info("%s - L%d - domain %d\n", __func__, __LINE__, tx.domain_id);
	while (index < mpxy_reg->num_levels) {
		tx.level_index = index;

		rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_GET_SUPPORTED_LEVELS,
						  &tx, sizeof(tx), rx,
						  mpxy_reg->rpmi_ctx->max_msg_size);

		ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
		if (ret)
			break;

		if (rx->status) {
			ret = rpmi_to_linux_error(rx->status);
			break;
		}

		if (mpxy_reg->num_levels != (index + rx->remaining_items + rx->returned_items)) {
			dev_err(mpxy_reg->dev,
				"invalid number of levels received\n");
			ret = -EINVAL;
			break;
		}

		data = rx->level;

		for (i = 0; i < rx->returned_items; i++) {

			switch (mpxy_reg->voltage_format) {
			case REG_FORMAT_DISCRETE:
				// pr_info("%s - L%d - DISCRETE\n", __func__, __LINE__);
				size = sizeof(struct rpmi_reg_level_discrete) / sizeof(u32);
				level[0] = data[0];
				// pr_info("%s - L%d - level[%d] %d\n", __func__, __LINE__, index + i, level[0]);
				break;
			case REG_FORMAT_LINEAR:
				// pr_info("%s - L%d - LINEAR\n", __func__, __LINE__);
				size = sizeof(struct rpmi_reg_level_linear) / sizeof(u32);
				level[0] = data[0];
				level[1] = data[1];
				level[2] = data[2];
				// pr_info("%s - L%d - level[%d] %d\n", __func__, __LINE__, index + i + 0, level[0]);
				// pr_info("%s - L%d - level[%d] %d\n", __func__, __LINE__, index + i + 1, level[1]);
				// pr_info("%s - L%d - level[%d] %d\n", __func__, __LINE__, index + i + 2, level[2]);
				break;
			}
			level += size;
			data += size;
		}
		index += rx->returned_items;
	}

	kfree(rx);

	return ret;
}

static int rpmi_reg_set_config(struct rpmi_reg_domain *mpxy_reg, u32 config)
{
	struct rpmi_set_config_tx tx;
	struct rpmi_set_config_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = mpxy_reg->id;
	tx.config = config;

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_SET_CONFIG,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}

static int mpxy_reg_enable(struct regulator_dev *rdev)
{
	struct rpmi_reg_domain *mpxy_reg = rdev_get_drvdata(rdev);

	pr_info("%s - L%d\n", __func__, __LINE__);

	if (mpxy_reg->always_on)
		return -EOPNOTSUPP;

	return rpmi_reg_set_config(mpxy_reg, RPMI_VOLT_ENABLE);
}

static int mpxy_reg_disable(struct regulator_dev *rdev)
{
	struct rpmi_reg_domain *mpxy_reg = rdev_get_drvdata(rdev);

	pr_info("%s - L%d\n", __func__, __LINE__);

	if (mpxy_reg->always_on)
		return -EOPNOTSUPP;

	return rpmi_reg_set_config(mpxy_reg, RPMI_VOLT_DISABLE);
}

static int mpxy_reg_is_enabled(struct regulator_dev *rdev)
{
	struct rpmi_reg_domain *mpxy_reg = rdev_get_drvdata(rdev);
	struct rpmi_get_config_tx tx;
	struct rpmi_get_config_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = mpxy_reg->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_GET_CONFIG,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return !!(rx.config & RPMI_VOLT_ENABLE);
}

static int mpxy_reg_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	struct rpmi_reg_domain *mpxy_reg = rdev_get_drvdata(rdev);
	struct rpmi_set_level_tx tx;
	struct rpmi_set_level_rx rx;
	struct rpmi_mbox_message msg;
	s32 volt_uV;
	int ret;

	volt_uV = mpxy_reg->desc.ops->list_voltage(rdev, selector);
	if (volt_uV <= 0)
		return -EINVAL;

	tx.domain_id = cpu_to_le32(mpxy_reg->id);
	tx.level = volt_uV;

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_SET_CONFIG,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}

static int mpxy_reg_get_voltage_sel(struct regulator_dev *rdev)
{
	struct rpmi_reg_domain *mpxy_reg = rdev_get_drvdata(rdev);
	struct rpmi_get_level_tx tx;
	struct rpmi_get_level_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = cpu_to_le32(mpxy_reg->id);

	rpmi_mbox_init_send_with_response(&msg, RPMI_VOLT_SRV_GET_LEVEL,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_reg->rpmi_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return mpxy_reg->desc.ops->map_voltage(rdev, rx.level, rx.level);
}

static const struct regulator_ops mpxy_reg_discrete_ops = {
	.enable = mpxy_reg_enable,
	.disable = mpxy_reg_disable,
	.is_enabled = mpxy_reg_is_enabled,
	.set_voltage_sel = mpxy_reg_set_voltage_sel,
	.get_voltage_sel = mpxy_reg_get_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
};

static const struct regulator_ops mpxy_reg_multi_linear_ops = {
	.enable = mpxy_reg_enable,
	.disable = mpxy_reg_disable,
	.is_enabled = mpxy_reg_is_enabled,
	.set_voltage_sel = mpxy_reg_set_voltage_sel,
	.get_voltage_sel = mpxy_reg_get_voltage_sel,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
};

static int rpmi_reg_setup(struct rpmi_reg_domain *mpxy_reg)
{
	struct rpmi_reg_level_linear *linear_level;
	struct linear_range *linear_ranges;
	u32 i, linear_index, n_step;

	mpxy_reg->desc.name = devm_kasprintf(mpxy_reg->dev, GFP_KERNEL, "%s", mpxy_reg->name);
	mpxy_reg->desc.id = mpxy_reg->id;
	mpxy_reg->desc.type = REGULATOR_VOLTAGE;
	mpxy_reg->desc.owner = THIS_MODULE;
	mpxy_reg->desc.regulators_node = "regulators";

	switch (mpxy_reg->voltage_format) {
	case REG_FORMAT_DISCRETE:
		mpxy_reg->desc.n_voltages = mpxy_reg->num_levels;
		mpxy_reg->desc.volt_table = (const unsigned int *) mpxy_reg->level;
		mpxy_reg->desc.ops = &mpxy_reg_discrete_ops;
		break;

	case REG_FORMAT_LINEAR:
		linear_ranges = devm_kcalloc(mpxy_reg->dev, mpxy_reg->num_levels,
					     sizeof(struct linear_range), GFP_KERNEL);
		if (!linear_ranges)
			return -ENOMEM;

		linear_level = (struct rpmi_reg_level_linear *) mpxy_reg->level;
		for (i = 0, linear_index = 0; i < mpxy_reg->num_levels; i++) {
			n_step = (linear_level[i].uvolt_max - linear_level[i].uvolt_min) /
				 linear_level[i].uvolt_step;

			linear_ranges[i].min = linear_level[i].uvolt_min;
			linear_ranges[i].min_sel = linear_index;
			linear_ranges[i].max_sel = linear_index + n_step;
			linear_ranges[i].step = linear_level[i].uvolt_step;

			linear_index += n_step;
		}

		mpxy_reg->desc.continuous_voltage_range = true;
		mpxy_reg->desc.linear_ranges = linear_ranges;
		mpxy_reg->desc.n_linear_ranges = mpxy_reg->num_levels;
		mpxy_reg->desc.n_voltages = linear_ranges[i - 1].max_sel;
		mpxy_reg->desc.ops = &mpxy_reg_multi_linear_ops;

		break;
	}

	return 0;
}

static int rpmi_reg_attr_setup(struct device *dev, struct rpmi_ctx *mpxy_ctx)
{
	struct rpmi_mbox_message msg;
	int ret;

	/* Validate RPMI specification version */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SPEC_VERSION);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_err(dev, "Failed to get spec version\n");
		return ret;
	}

	if (msg.attr.value < RPMI_MKVER(1, 0)) {
		dev_err(dev,
			"msg protocol version mismatch, expected 0x%x, found 0x%x\n",
			RPMI_MKVER(1, 0), msg.attr.value);
		return -EINVAL;
	}

	/* Validate voltage service group ID */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SERVICEGROUP_ID);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_err(dev, "Failed to get service group ID\n");
		return ret;
	}

	if (msg.attr.value != RPMI_SRVGRP_VOLTAGE) {
		dev_err(dev,
			"service group match failed, expected 0x%x, found 0x%x\n",
			RPMI_SRVGRP_VOLTAGE, msg.attr.value);
		return -EINVAL;
	}

	/* Validate voltage service group version */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SERVICEGROUP_VERSION);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_err(dev, "Failed to get service group version\n");
		return ret;
	}

	if (msg.attr.value < RPMI_MKVER(1, 0)) {
		dev_err(dev,
			"service group version failed, expected 0x%x, found 0x%x\n",
			RPMI_MKVER(1, 0), msg.attr.value);
		return -EINVAL;
	}

	/* Get max message size */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_MAX_MSG_DATA_SIZE);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_err(dev, "Failed to get max message data size\n");
		return ret;
	}
	mpxy_ctx->max_msg_size = msg.attr.value;

	return 0;
}

static int rpmi_reg_probe(struct platform_device *pdev)
{
	struct regulator_config config = {};
	struct rpmi_reg_domain *rpmi_reg;
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct rpmi_ctx *mpxy_ctx;
	int num_domains = 0;
	int i, ret;

	mpxy_ctx = devm_kzalloc(&pdev->dev, sizeof(*mpxy_ctx), GFP_KERNEL);
	if (!mpxy_ctx)
		return -ENOMEM;

	/* Setup mailbox client */
	mpxy_ctx->client.dev		= dev;
	mpxy_ctx->client.rx_callback	= NULL;
	mpxy_ctx->client.tx_block	= false;
	mpxy_ctx->client.knows_txdone	= true;
	mpxy_ctx->client.tx_tout	= 0;

	/* Request mailbox channel */
	mpxy_ctx->chan = mbox_request_channel(&mpxy_ctx->client, 0);
	if (IS_ERR(mpxy_ctx->chan))
		return PTR_ERR(mpxy_ctx->chan);

	ret = rpmi_reg_attr_setup(dev, mpxy_ctx);
	if (ret) {
		dev_err(dev, "failed to verify RPMI attribute - err:%d\n", ret);
		goto fail_free_channel;
	}

	/* Get number of voltage domain */
	ret = rpmi_reg_get_num_domains(mpxy_ctx, &num_domains);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to get number of voltage domains - err:%d\n", ret);
		goto fail_free_channel;
	}

	if (!num_domains) {
		dev_err(&pdev->dev, "No voltage domains found!\n");
		ret = -EINVAL;
		goto fail_free_channel;
	}

	// dev_info(dev, "%d MPXY Voltage domains are found\n", num_domains);

	rpmi_reg = devm_kcalloc(dev, num_domains, sizeof(*rpmi_reg), GFP_KERNEL);
	if (!rpmi_reg)
		return -ENOMEM;

	for (i = 0; i < num_domains; i++, rpmi_reg++) {
		rpmi_reg->rpmi_ctx = mpxy_ctx;
		rpmi_reg->dev = dev;
		rpmi_reg->id = i;

		ret = rpmi_reg_get_attrs(rpmi_reg);
		if (ret) {
			dev_warn(rpmi_reg->dev,
				 "voltage domain %d initialization failed\n",
				 rpmi_reg->id);
			continue;
		}

		ret = rpmi_reg_get_supported_levels(rpmi_reg);
		if (ret) {
			dev_warn(rpmi_reg->dev,
				 "voltage domain %d initialization failed\n",
				 rpmi_reg->id);
			continue;
		}

		ret = rpmi_reg_setup(rpmi_reg);
		if (ret) {
			dev_warn(rpmi_reg->dev,
				 "voltage domain %d initialization failed\n",
				 rpmi_reg->id);
			continue;
		}

		config.dev = rpmi_reg->dev;
		config.driver_data = rpmi_reg;

		rdev = devm_regulator_register(rpmi_reg->dev, &rpmi_reg->desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(dev, "failed to register RPMI voltage domain %d\n", i);
			continue;
		}

		dev_info(dev, "MPXY Voltage domains %s registered\n", rpmi_reg->desc.name);
	}

	return 0;

fail_free_channel:
	mbox_free_channel(mpxy_ctx->chan);

	return ret;
}

static const struct of_device_id rpmi_reg_of_match[] = {
	{ .compatible = "riscv,rpmi-voltage" },
	{ },
};

MODULE_DEVICE_TABLE(of, rpmi_reg_of_match);

static struct platform_driver rpmi_reg_platdrv = {
	.driver = {
		.name = "riscv-rpmi-regulator",
		.of_match_table = rpmi_reg_of_match,
	},
	.probe = rpmi_reg_probe,
};

module_platform_driver(rpmi_reg_platdrv);

MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Regulator Driver based on SBI MPXY extension");
MODULE_LICENSE("GPL");
