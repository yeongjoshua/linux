// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RISC-V RPMI Based Device Power Driver through SBI MPXY
 *
 * Copyright (C) 2025 Shanghai StarFive Technology Co., Ltd.
 *
 * Implements a Device Power driver on top of SBI RPMI Message Proxy Extension (MPXY)
 *
 * Each SBI MPXY Device Power instance is associated, through the means of a proper DT
 * entry description, to a specific Transport ID.
 */

#define pr_fmt(fmt) "riscv-rpmi-device-power: " fmt

#include <linux/bitfield.h>
#include <linux/mailbox/riscv-rpmi-message.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

#define RPMI_PM_DOMAIN_NAME_LEN		16

/* power state for device power domains */
#define RPMI_POWER_STATE_CONTEXT_MASK		GENMASK(16, 16)
#define RPMI_POWER_STATE_CONTEXT_PRESERVED	0
#define RPMI_POWER_STATE_CONTEXT_LOST		1
#define RPMI_POWER_STATE_VALUE_MASK		GENMASK(15, 0)
#define RPMI_POWER_STATE_VALUE_ON		0
#define RPMI_POWER_STATE_VALUE_OFF		3

#define RPMI_POWER_STATE_PARAM(context_type, state_value) \
	((FIELD_PREP(RPMI_POWER_STATE_CONTEXT_MASK, context_type)) | \
	(FIELD_PREP(RPMI_POWER_STATE_VALUE_MASK, state_value)))

#define RPMI_POWER_STATE_GENERIC_ON  \
	RPMI_POWER_STATE_PARAM(RPMI_POWER_STATE_CONTEXT_PRESERVED, \
				   RPMI_POWER_STATE_VALUE_ON)

#define RPMI_POWER_STATE_GENERIC_OFF  \
	RPMI_POWER_STATE_PARAM(RPMI_POWER_STATE_CONTEXT_PRESERVED, \
				   RPMI_POWER_STATE_VALUE_OFF)

struct rpmi_ctx {
	struct mbox_chan *chan;
	struct mbox_client client;
};

/**
 * struct rpmi_device_power_domain - describe one available MPXY Device Power Domain
 *
 * @id: the power domain ID as advertised by PuC
 * @transition_latency: worst case transition latency of power domain from one state
 *			to another
 * @dev: device associated with this power domain
 * @name: device power domain name assigned by PuC
 */
struct rpmi_device_power_domain {
	u32 id;
	u32 transition_latency;
	struct device *dev;
	struct rpmi_ctx *mpxy_ctx;
	struct generic_pm_domain genpd;
	char name[RPMI_PM_DOMAIN_NAME_LEN];
};

#define to_rpmi_pd(gpd) container_of(gpd, struct rpmi_device_power_domain, genpd)

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
	char name[RPMI_PM_DOMAIN_NAME_LEN];
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

static int rpmi_power_state_get(struct rpmi_device_power_domain *mpxy_pm_domain,
				u32 domain_id, u32 *state)
{
	struct rpmi_pm_get_power_state_tx tx;
	struct rpmi_pm_get_power_state_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = cpu_to_le32(domain_id);

	rpmi_mbox_init_send_with_response(&msg, RPMI_DP_SRV_GET_STATE,
					  &tx, sizeof(tx), &rx, sizeof(rx));

	ret = rpmi_mbox_send_message(mpxy_pm_domain->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*state = rx.power_state;

	return ret;
}

static int rpmi_power_state_set(struct rpmi_device_power_domain *mpxy_pm_domain,
				u32 domain_id, u32 state)
{
	struct rpmi_pm_set_power_state_tx tx;
	struct rpmi_pm_set_power_state_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = cpu_to_le32(domain_id);
	tx.power_state = cpu_to_le32(state);

	rpmi_mbox_init_send_with_response(&msg, RPMI_DP_SRV_SET_STATE,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(mpxy_pm_domain->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}

static int rpmi_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct rpmi_device_power_domain *mpxy_pm_domain;
	u32 state, ret_state, domain_id;
	int ret;

	if (power_on)
		state = RPMI_POWER_STATE_GENERIC_ON;
	else
		state = RPMI_POWER_STATE_GENERIC_OFF;

	mpxy_pm_domain = to_rpmi_pd(domain);
	domain_id = mpxy_pm_domain->id;

	ret = rpmi_power_state_set(mpxy_pm_domain, domain_id, state);
	if (!ret)
		ret = rpmi_power_state_get(mpxy_pm_domain, domain_id, &ret_state);
	if (!ret && state != ret_state)
		return -EIO;

	return ret;
}

static int rpmi_pd_power_on(struct generic_pm_domain *domain)
{
	return rpmi_pd_power(domain, true);
}

static int rpmi_pd_power_off(struct generic_pm_domain *domain)
{
	return rpmi_pd_power(domain, false);
}

static int rpmi_pm_get_num_domains(struct rpmi_ctx *mpxy_ctx, u32 *domain)
{
	struct rpmi_pm_get_num_domain_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	rpmi_mbox_init_send_with_response(&msg, RPMI_DP_SRV_GET_NUM_DOMAINS,
					  NULL, 0, &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*domain = rx.num_domains;

	return 0;
}

/* obtain the MPXY device power domain attributes */
static int rpmi_device_power_get_attrs(u32 domain_id,
				       struct rpmi_device_power_domain *mpxy_pm_domain)
{
	struct rpmi_pm_get_domain_attrs_tx tx;
	struct rpmi_pm_get_domain_attrs_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = cpu_to_le32(domain_id);

	rpmi_mbox_init_send_with_response(&msg, RPMI_DP_SRV_GET_ATTRS,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(mpxy_pm_domain->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	mpxy_pm_domain->transition_latency = rx.transition_latency;
	strscpy(mpxy_pm_domain->name, rx.name, RPMI_PM_DOMAIN_NAME_LEN);

	return 0;
}

static int rpmi_pm_attr_setup(struct device *dev, struct rpmi_ctx *mpxy_ctx)
{
	struct rpmi_mbox_message msg;
	int ret;

	/* Validate RPMI specification version */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SPEC_VERSION);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_dbg(dev, "Failed to get spec version\n");
		return ret;
	}

	if (msg.attr.value < RPMI_MKVER(1, 0)) {
		dev_dbg(dev,
			"msg protocol version mismatch, expected 0x%x, found 0x%x\n",
			RPMI_MKVER(1, 0), msg.attr.value);
		return -EINVAL;
	}

	/* Validate device power service group ID */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SERVICEGROUP_ID);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_dbg(dev, "Failed to get service group ID\n");
		return ret;
	}

	if (msg.attr.value != RPMI_SRVGRP_DEVICE_POWER) {
		dev_dbg(dev,
			"service group match failed, expected 0x%x, found 0x%x\n",
			RPMI_SRVGRP_DEVICE_POWER, msg.attr.value);
		return -EINVAL;
	}

	/* Validate device power service group version */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SERVICEGROUP_VERSION);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_dbg(dev, "Failed to get service group version\n");
		return ret;
	}

	if (msg.attr.value < RPMI_MKVER(1, 0)) {
		dev_dbg(dev,
			"service group version failed, expected 0x%x, found 0x%x\n",
			RPMI_MKVER(1, 0), msg.attr.value);
		return -EINVAL;
	}

	return 0;
}

static int rpmi_pm_domain_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct rpmi_device_power_domain *mpxy_pd;
	struct genpd_onecell_data *mpxy_pd_data;
	struct generic_pm_domain **domains;
	struct device *dev = &pdev->dev;
	struct rpmi_ctx *mpxy_ctx;
	int num_domains = 0;
	int ret, i;

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

	ret = rpmi_pm_attr_setup(dev, mpxy_ctx);
	if (ret) {
		dev_err(dev, "failed to verify RPMI attribute - err:%d\n", ret);
		goto fail_free_channel;
	}

	/* Get number of device power domain */
	ret = rpmi_pm_get_num_domains(mpxy_ctx, &num_domains);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to get number of pm domains - err:%d\n", ret);
		goto fail_free_channel;
	}

	if (!num_domains) {
		dev_err(&pdev->dev, "No PM domains found!\n");
		ret = -EINVAL;
		goto fail_free_channel;
	}

	dev_info(&pdev->dev, "%d MPXY PM domains are found\n", num_domains);

	mpxy_pd = devm_kcalloc(&pdev->dev, num_domains, sizeof(*mpxy_pd), GFP_KERNEL);
	if (!mpxy_pd) {
		ret = -ENOMEM;
		goto fail_free_channel;
	}

	domains = devm_kcalloc(&pdev->dev, num_domains, sizeof(*domains), GFP_KERNEL);
	if (!domains) {
		ret = -ENOMEM;
		goto fail_free_channel;
	}

	mpxy_pd_data = devm_kzalloc(&pdev->dev, sizeof(*mpxy_pd_data), GFP_KERNEL);
	if (!mpxy_pd_data) {
		ret = -ENOMEM;
		goto fail_free_channel;
	}

	for (i = 0; i < num_domains; i++, mpxy_pd++) {
		u32 state;

		mpxy_pd->dev = &pdev->dev;
		mpxy_pd->mpxy_ctx = mpxy_ctx;
		mpxy_pd->id = i;

		ret = rpmi_device_power_get_attrs(i, mpxy_pd);
		if (ret) {
			dev_warn(mpxy_pd->dev,
				 "power domain %d initialization failed\n",
				 mpxy_pd->id);
			domains[i] = NULL;
			continue;
		}

		ret = rpmi_power_state_get(mpxy_pd, i, &state);
		if (ret || (state != RPMI_POWER_STATE_GENERIC_OFF &&
			    state != RPMI_POWER_STATE_GENERIC_ON)) {
			dev_warn(mpxy_pd->dev,
				 "failed to get state for power domain %d\n",
				 mpxy_pd->id);
			domains[i] = NULL;
			continue;
		}

		mpxy_pd->genpd.name = mpxy_pd->name;
		mpxy_pd->genpd.power_off = rpmi_pd_power_off;
		mpxy_pd->genpd.power_on = rpmi_pd_power_on;

		pm_genpd_init(&mpxy_pd->genpd, NULL,
			      state == RPMI_POWER_STATE_GENERIC_OFF);

		domains[i] = &mpxy_pd->genpd;
	}

	mpxy_pd_data->domains = domains;
	mpxy_pd_data->num_domains = num_domains;

	platform_set_drvdata(pdev, mpxy_pd_data);

	ret = of_genpd_add_provider_onecell(np, mpxy_pd_data);
	if (ret)
		goto fail_free_channel;

	return 0;

fail_free_channel:
	mbox_free_channel(mpxy_ctx->chan);

	return ret;
}

static void rpmi_pm_domain_remove(struct platform_device *pdev)
{
	struct rpmi_device_power_domain *mpxy_pm_domain;
	struct device_node *np = pdev->dev.of_node;
	struct genpd_onecell_data *mpxy_pd_data;
	int i;

	of_genpd_del_provider(np);
	mpxy_pd_data = platform_get_drvdata(pdev);
	for (i = 0; i < mpxy_pd_data->num_domains; i++) {
		if (!mpxy_pd_data->domains[i])
			continue;
		pm_genpd_remove(mpxy_pd_data->domains[i]);
	}

	mpxy_pm_domain = to_rpmi_pd(mpxy_pd_data->domains[0]);

	mbox_free_channel(mpxy_pm_domain->mpxy_ctx->chan);
}

static const struct of_device_id rpmi_pm_domain_of_match[] = {
	{ .compatible = "riscv,rpmi-device-power" },
	{},
};

MODULE_DEVICE_TABLE(of, rpmi_pm_domain_of_match);

static struct platform_driver rpmi_pm_domain_platdrv = {
	.driver = {
		.name = "riscv-rpmi-device-power",
		.of_match_table = rpmi_pm_domain_of_match,
	},
	.probe = rpmi_pm_domain_probe,
	.remove = rpmi_pm_domain_remove,
};

module_platform_driver(rpmi_pm_domain_platdrv);

MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Device Power Driver based on RPMI message protocol");
MODULE_LICENSE("GPL");
