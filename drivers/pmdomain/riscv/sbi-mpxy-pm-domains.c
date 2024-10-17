// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RISC-V SBI MPXY Based Device Power Driver
 *
 * Copyright (C) 2023-2024 Shanghai StarFive Technology Co., Ltd.
 *
 * Implements a Device Power driver on top of SBI RPMI Proxy Extension (MPXY)
 *
 * Each SBI MPXY Device Power instance is associated, through the means of a proper DT
 * entry description, to a specific Transport ID.
 */

#define pr_fmt(fmt) "sbi-mpxy-device-power-domains: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <asm/sbi.h>
#include <asm/rpmi.h>

#define ATTR_COUNT(t, f)	(t - f + 1)

#define SBI_MPXY_PM_DOMAIN_NAME_LEN		16

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

/* power state for device power domains */
#define SBI_MPXY_POWER_STATE_CONTEXT_SHIFT	BIT(16)
#define SBI_MPXY_POWER_STATE_CONTEXT_PRESERVED	0
#define SBI_MPXY_POWER_STATE_CONTEXT_LOST	1
#define SBI_MPXY_POWER_STATE_VALUE_MASK		GENMASK(15, 0)
#define SBI_MPXY_POWER_STATE_VALUE_ON		0
#define SBI_MPXY_POWER_STATE_VALUE_OFF		3

#define SBI_MPXY_POWER_STATE_PARAM(context_type, state_value) \
	((FIELD_GET(SBI_MPXY_POWER_STATE_CONTEXT_SHIFT, context_type)) | \
	(FIELD_GET(SBI_MPXY_POWER_STATE_VALUE_MASK, state_value)))

#define SBI_MPXY_POWER_STATE_GENERIC_ON  \
	SBI_MPXY_POWER_STATE_PARAM(SBI_MPXY_POWER_STATE_CONTEXT_PRESERVED, \
				   SBI_MPXY_POWER_STATE_VALUE_ON)

#define SBI_MPXY_POWER_STATE_GENERIC_OFF  \
	SBI_MPXY_POWER_STATE_PARAM(SBI_MPXY_POWER_STATE_CONTEXT_LOST, \
				   SBI_MPXY_POWER_STATE_VALUE_OFF)

struct sbi_mpxy_ctx {
	/* transport id */
	u32 channel_id;
	u32 max_msg_len;
};

/**
 * struct sbi_mpxy_device_power_domain - describe one available MPXY Device Power Domain
 *
 * @id: the power domain ID as advertised by PuC
 * @transition_latency: worst case transition latency of power domain from one state
 *			to another
 * @dev: device associated with this power domain
 * @name: device power domain name assigned by PuC
 */
struct sbi_mpxy_device_power_domain {
	u32 id;
	u32 transition_latency;
	struct device *dev;
	struct sbi_mpxy_ctx *mpxy_ctx;
	struct generic_pm_domain genpd;
	char name[SBI_MPXY_PM_DOMAIN_NAME_LEN];
};

#define to_sbi_mpxy_pd(gpd) container_of(gpd, struct sbi_mpxy_device_power_domain, genpd)

/* Service: ENABLE_NOTIFICATION */
struct rpmi_pm_enable_notification_tx {
	u32 event_id;
};

struct rpmi_pm_enable_notification_rx {
	s32 status;
};

/* Service: GET_POWER_DOMAINS */
struct rpmi_pm_get_num_domain_rx {
	s32 status;
	u32 num_domains;
};

/* Service: GET_POWER_DOMAIN_ATTRS */
struct rpmi_pm_get_domain_attrs_tx {
	u32 domain_id;
};

/* pm domain attributes response data */
struct rpmi_pm_get_domain_attrs_rx {
	s32 status;
	u32 flags;
	u32 transition_latency;
	char name[SBI_MPXY_PM_DOMAIN_NAME_LEN];
};

/* Service: SET_POWER_DOMAIN_STATE */
struct rpmi_pm_set_power_state_tx {
	u32 domain_id;
	u32 power_state;
};

struct rpmi_pm_set_power_state_rx {
	s32 status;
};

/* Service: GET_POWER_DOMAIN_STATE */
struct rpmi_pm_get_power_state_tx {
	u32 domain_id;
};

struct rpmi_pm_get_power_state_rx {
	s32 status;
	u32 power_state;
};

static int sbi_mpxy_power_state_get(struct sbi_mpxy_device_power_domain *mpxy_pm_domain,
				    u32 domain_id, u32 *state)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_pm_get_power_state_tx tx;
	struct rpmi_pm_get_power_state_rx rx;

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_pm_domain->mpxy_ctx->channel_id,
					     RPMI_DP_SRV_GET_STATE,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_pm_domain->dev,
			"get state of power domain #%u failed with error: %d\n",
			 domain_id, ret);
		return ret;
	}

	if (rx.status) {
		dev_err(mpxy_pm_domain->dev,
			"get state of power domain #%u failed with RPMI error: %d\n",
			 domain_id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	*state = rx.power_state;

	return ret;
}

static int sbi_mpxy_power_state_set(struct sbi_mpxy_device_power_domain *mpxy_pm_domain,
				    u32 domain_id,
				    u32 state)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_pm_set_power_state_tx tx;
	struct rpmi_pm_set_power_state_rx rx;

	tx.domain_id = cpu_to_le32(domain_id);
	tx.power_state = cpu_to_le32(state);
	ret = sbi_mpxy_send_message_withresp(mpxy_pm_domain->mpxy_ctx->channel_id,
					     RPMI_DP_SRV_SET_STATE,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_pm_domain->dev,
			"set power domain #%u to state %u failed with error: %d\n",
			 domain_id, state, ret);
		return ret;
	}

	if (rx.status) {
		dev_err(mpxy_pm_domain->dev,
			"set power domain #%u to state %u failed with RPMI error: %d\n",
			 domain_id, state, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	return ret;
}

static int sbi_mpxy_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	int ret;
	u32 state, ret_state;
	u32 domain_id;
	struct sbi_mpxy_device_power_domain *mpxy_pm_domain;

	if (power_on)
		state = SBI_MPXY_POWER_STATE_GENERIC_ON;
	else
		state = SBI_MPXY_POWER_STATE_GENERIC_OFF;

	mpxy_pm_domain = to_sbi_mpxy_pd(domain);
	domain_id = mpxy_pm_domain->id;

	ret = sbi_mpxy_power_state_set(mpxy_pm_domain, domain_id, state);
	if (!ret)
		ret = sbi_mpxy_power_state_get(mpxy_pm_domain, domain_id, &ret_state);
	if (!ret && state != ret_state)
		return -EIO;

	return ret;
}

static int sbi_mpxy_pd_power_on(struct generic_pm_domain *domain)
{
	return sbi_mpxy_pd_power(domain, true);
}

static int sbi_mpxy_pd_power_off(struct generic_pm_domain *domain)
{
	return sbi_mpxy_pd_power(domain, false);
}

static int sbi_mpxy_pm_get_num_domains(u32 channel_id)
{
	int ret;
	struct rpmi_pm_get_num_domain_rx rx;

	ret = sbi_mpxy_send_message_withresp(channel_id, RPMI_DP_SRV_GET_NUM_DOMAINS,
					     NULL, 0, &rx, NULL);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return rx.num_domains;
}

/* obtain the MPXY device power domain attributes */
static int sbi_mpxy_device_power_get_attrs(u32 domain_id,
					   struct sbi_mpxy_device_power_domain *mpxy_pm_domain)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_pm_get_domain_attrs_tx tx;
	struct rpmi_pm_get_domain_attrs_rx rx;

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_pm_domain->mpxy_ctx->channel_id,
					     RPMI_DP_SRV_GET_ATTRS,
					     &tx, sizeof(tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_pm_domain->dev,
			"get attributes of power domain #%u failed with error: %d\n",
			domain_id, ret);
		return ret;
	}

	if (rx.status) {
		dev_err(mpxy_pm_domain->dev,
			"get attributes of power domain #%u failed with RPMI error: %d\n",
			domain_id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	mpxy_pm_domain->transition_latency = rx.transition_latency;
	strscpy(mpxy_pm_domain->name, rx.name, SBI_MPXY_PM_DOMAIN_NAME_LEN);

	return 0;
}

static int sbi_mpxy_device_power_enumerate(struct sbi_mpxy_device_power_domain *mpxy_pm_domain,
					   u32 domain_id)
{
	int ret;

	ret = sbi_mpxy_device_power_get_attrs(domain_id, mpxy_pm_domain);
	if (ret)
		return ret;

	return 0;
}

static int sbi_mpxy_pm_domain_probe(struct platform_device *pdev)
{
	u32 channel_id, i, attr_count, version;
	int ret, num_domains;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct genpd_onecell_data *mpxy_pd_data;
	struct generic_pm_domain **domains;
	struct sbi_mpxy_device_power_domain *mpxy_pd;
	struct sbi_mpxy_ctx *mpxy_ctx;
	u32 *attr_buf;
	struct of_phandle_args args;

	if (sbi_spec_version < sbi_mk_version(1, 0) ||
	    sbi_probe_extension(SBI_EXT_MPXY) <= 0) {
		dev_err(&pdev->dev, "sbi mpxy extension not present!\n");
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

	ret = sbi_mpxy_read_attrs(channel_id, SBI_MPXY_ATTR_MSGPROTO_ATTR_START,
				  1, attr_buf);
	if (ret) {
		dev_err(&pdev->dev, "channel-%u: read attributes - %d\n",
			channel_id, ret);
		return ret;
	}

	if (attr_buf[0] != RPMI_SRVGRP_DEVICE_POWER) {
		dev_err(&pdev->dev,
		"channel-%u ServiceGroup match failed, expected %x, found %x\n",
		channel_id, RPMI_SRVGRP_DEVICE_POWER, attr_buf[0]);
		return -EINVAL;
	}

	num_domains = sbi_mpxy_pm_get_num_domains(channel_id);
	if (num_domains <= 0) {
		if (!num_domains) {
			if (!num_domains) {
				dev_err(&pdev->dev, "No PM domains found!\n");
				num_domains = -EINVAL;
			}
		} else {
			dev_err(&pdev->dev,
				"failed to get number of pm domains - err:%d\n",
				 num_domains);
		}

		return num_domains;
	}

	dev_info(&pdev->dev, "%d MPXY PM domains are found\n",
		 num_domains);

	mpxy_pd = devm_kcalloc(&pdev->dev, num_domains, sizeof(*mpxy_pd), GFP_KERNEL);
	if (!mpxy_pd)
		return -ENOMEM;

	mpxy_ctx = devm_kzalloc(&pdev->dev, sizeof(*mpxy_ctx), GFP_KERNEL);
	if (!mpxy_ctx)
		return -ENOMEM;

	mpxy_ctx->channel_id = channel_id;
	mpxy_ctx->max_msg_len = attr_buf[2];

	mpxy_pd_data = devm_kzalloc(&pdev->dev, sizeof(*mpxy_pd_data), GFP_KERNEL);
	if (!mpxy_pd_data)
		return -ENOMEM;

	domains = devm_kcalloc(&pdev->dev, num_domains, sizeof(*domains), GFP_KERNEL);
	if (!domains)
		return -ENOMEM;

	for (i = 0; i < num_domains; i++, mpxy_pd++) {
		u32 state;

		mpxy_pd->dev = &pdev->dev;
		mpxy_pd->mpxy_ctx = mpxy_ctx;
		mpxy_pd->id = i;

		ret = sbi_mpxy_power_state_get(mpxy_pd, i, &state);
		if (ret) {
			dev_err_probe(mpxy_pd->dev, ret,
				      "failed to get state for power domain %d\n",
				      mpxy_pd->id);
			return ret;
		}

		ret = sbi_mpxy_device_power_enumerate(mpxy_pd, i);
		if (ret) {
			dev_err_probe(mpxy_pd->dev, ret,
				      "power domain %d initialization failed\n",
				       mpxy_pd->id);
			return ret;
		}

		mpxy_pd->genpd.name = mpxy_pd->name;
		mpxy_pd->genpd.power_off = sbi_mpxy_pd_power_off;
		mpxy_pd->genpd.power_on = sbi_mpxy_pd_power_on;

		pm_genpd_init(&mpxy_pd->genpd, NULL,
			      state == SBI_MPXY_POWER_STATE_GENERIC_OFF);

		domains[i] = &mpxy_pd->genpd;
	}

	mpxy_pd_data->domains = domains;
	mpxy_pd_data->num_domains = num_domains;

	platform_set_drvdata(pdev, mpxy_pd_data);

	return of_genpd_add_provider_onecell(np, mpxy_pd_data);
}

static void sbi_mpxy_pm_domain_remove(struct platform_device *pdev)
{
	int i;
	struct genpd_onecell_data *mpxy_pd_data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	of_genpd_del_provider(np);

	mpxy_pd_data = platform_get_drvdata(pdev);
	for (i = 0; i < mpxy_pd_data->num_domains; i++) {
		if (!mpxy_pd_data->domains[i])
			continue;
		pm_genpd_remove(mpxy_pd_data->domains[i]);
	}
}

static const struct of_device_id sbi_mpxy_pm_domain_of_match[] = {
	{.compatible = "riscv,rpmi-device-power"},
	{},
};

MODULE_DEVICE_TABLE(of, sbi_mpxy_pm_domain_of_match);

static struct platform_driver sbi_mpxy_pm_domain_platdrv = {
	.driver = {
		.name = "sbi-mpxy-device-power",
		.of_match_table = sbi_mpxy_pm_domain_of_match,
	},
	.probe = sbi_mpxy_pm_domain_probe,
	.remove = sbi_mpxy_pm_domain_remove,
};

module_platform_driver(sbi_mpxy_pm_domain_platdrv);

MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("RISC-V SBI MPXY Based Power Domain Driver");
MODULE_LICENSE("GPL v2");
