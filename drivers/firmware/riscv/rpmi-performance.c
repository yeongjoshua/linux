// SPDX-License-Identifier: GPL-2.0
/*
 * RISC-V RPMI Performance Service Provider
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 */

#define pr_fmt(fmt) "riscv-rpmi-performance: " fmt

#include <linux/bitfield.h>
#include <linux/bsearch.h>
#include <linux/io.h>
#include <linux/mailbox/riscv-rpmi-message.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/riscv-rpmi-performance.h>
#include <linux/string.h>

struct rpmi_fc_db_info {
	int width;
	u64 set;
	void __iomem *addr;
};

struct rpmi_fc_info {
	void __iomem *set_addr;
	void __iomem *get_addr;
	struct rpmi_fc_db_info *set_db;
};

struct rpmi_ctx {
	struct mbox_chan *chan;
	struct mbox_client client;
	u32 max_msg_size;
};

struct rpmi_fast_channel {
	u64 addr;
	u64 size;
};

struct rpmi_perf {
	u32 num_domains;
	struct rpmi_fast_channel fast_channel;
	struct device *dev;
	struct rpmi_perf_domain *domain;
	struct rpmi_ctx *mpxy_ctx;
#if IS_ENABLED(CONFIG_RISCV_RPMI_CPUFREQ)
	struct platform_device *cpufreq_pdev;
#endif
#if IS_ENABLED(CONFIG_RISCV_RPMI_DEVFREQ)
	struct platform_device *devfreq_pdev;
#endif
};

static bool rpmi_perf_node_is_cpu(const struct device_node *np)
{
	const char *type;

	if (!np)
		return false;

	if (!of_property_read_string(np, "device_type", &type) &&
	    !strcmp(type, "cpu"))
		return true;

	return false;
}

static bool rpmi_perf_node_uses_provider(struct device *provider,
					 struct device_node *np)
{
	struct of_phandle_args args;
	int count, i;

	if (!provider || !np)
		return false;

	count = of_count_phandle_with_args(np, "performance-domains",
					   "#performance-domain-cells");
	if (count <= 0)
		return false;

	for (i = 0; i < count; i++) {
		if (of_parse_phandle_with_args(np, "performance-domains",
					       "#performance-domain-cells",
					       i, &args))
			continue;

		if (args.np == provider->of_node) {
			of_node_put(args.np);
			return true;
		}

		of_node_put(args.np);
	}

	return false;
}

static bool rpmi_perf_has_consumer(struct device *provider, bool want_cpu)
{
	struct device_node *np;

	for_each_of_allnodes(np) {
		if (want_cpu != rpmi_perf_node_is_cpu(np))
			continue;

		if (rpmi_perf_node_uses_provider(provider, np))
			return true;
	}

	return false;
}

enum {
	RPMI_PERF_FC_LEVEL,
	RPMI_PERF_FC_LIMIT,
	RPMI_PERF_FC_MAX,
};

/* Service: ENABLE_NOTIFICATION */
struct rpmi_perf_enable_notification_tx {
	u32 event_id;
};

struct rpmi_perf_enable_notification_rx {
	s32 status;
};

/* Service: GET_NUM_DOMAINS */
struct rpmi_perf_get_num_domain_rx {
	s32 status;
	u32 num_domains;
};

/* Service: GET_ATTRS */
struct rpmi_perf_get_attrs_tx {
	u32 domain_id;
};

struct rpmi_perf_get_attrs_rx {
	s32 status;
	u32 flags;
#define PERF_LIMIT_SETTING(f)		(FIELD_GET(BIT(2), (f)))
#define PERF_LEVEL_SETTING(f)		(FIELD_GET(BIT(1), (f)))
#define FAST_CHANNEL_SUPPORT(f)		(FIELD_GET(BIT(0), (f)))
	u32 num_levels;
	u32 trans_latency_us;
	char name[RPMI_PERF_DOMAIN_NAME_LEN];
};

/* Service: GET_SUPPORTED_LEVELS */
struct rpmi_perf_get_supported_level_tx {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_perf_get_supported_level_rx {
	s32 status;
	u32 flags;
	u32 remaining;
	u32 returned;
	struct rpmi_perf_opp opp[];
};

/* Service: GET_LEVELS */
struct rpmi_perf_get_level_tx {
	u32 domain_id;
};

struct rpmi_perf_get_level_rx {
	s32 status;
	u32 level_index;
};

/* Service: SET_LEVELS */
struct rpmi_perf_set_level_tx {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_perf_set_level_rx {
	s32 status;
};

/* Service: GET_FAST_CHANNEL_REGION */
struct rpmi_perf_get_fast_channel_region_rx {
	s32 status;
	u32 addr_low;
	u32 addr_high;
	u32 size_low;
	u32 size_high;
};

/* Service: GET_FAST_CHANNEL_ATTRIBUTES */
struct rpmi_perf_get_fast_channel_attributes_tx {
	u32 domain_id;
	u32 service_id;
};

struct rpmi_perf_get_fast_channel_attributes_rx {
	s32 status;
	u32 flags;
#define DOORBELL_REG_WIDTH(f)	(FIELD_GET(GENMASK(2, 1), (f)))
#define SUPPORTS_DOORBELL(f)	(FIELD_GET(BIT(0), (f)))
	u32 offset_low;
	u32 offset_high;
	u32 size;
	u32 db_addr_low;
	u32 db_addr_high;
	u32 db_id;
};

static void rpmi_perf_fastchannel_db_ring(struct rpmi_fc_db_info *db)
{
	if (!db || !db->addr)
		return;

	switch (db->width) {
	case 8:
		iowrite8((u8)db->set, db->addr);
		break;
	case 16:
		iowrite16((u16)db->set, db->addr);
		break;
	case 32:
		iowrite32((u32)db->set, db->addr);
		break;
	}
}

static int rpmi_perf_get_num_domains(struct rpmi_ctx *mpxy_ctx, u32 *domain)
{
	struct rpmi_perf_get_num_domain_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_NUM_DOMAINS,
					  NULL, 0, &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*domain = rx.num_domains;

	return 0;
}

static int rpmi_perf_get_attrs(struct rpmi_perf *perf, struct rpmi_perf_domain *domain)
{
	struct rpmi_perf_get_attrs_tx tx;
	struct rpmi_perf_get_attrs_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = domain->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_ATTRIBUTES,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(domain->perf->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	domain->set_limit = PERF_LIMIT_SETTING(rx.flags);
	domain->set_level = PERF_LEVEL_SETTING(rx.flags);
	domain->has_fastchannels = perf->fast_channel.size &&
				   (domain->set_limit || domain->set_level) ?
				   FAST_CHANNEL_SUPPORT(rx.flags) : false;
	domain->opp_count = rx.num_levels;
	domain->rate_limit_us = rx.trans_latency_us;
	strscpy(domain->name, rx.name, RPMI_PERF_DOMAIN_NAME_LEN);

	if (!domain->opp_count)
		return -EINVAL;

	domain->opp = devm_kcalloc(perf->dev, domain->opp_count,
				   sizeof(struct rpmi_perf_opp), GFP_KERNEL);
	if (!domain->opp)
		return -ENOMEM;

	return 0;
}

static int rpmi_perf_get_supported_levels(struct rpmi_perf_domain *domain)
{
	struct rpmi_perf_get_supported_level_tx tx;
	struct rpmi_perf_get_supported_level_rx *rx;
	struct rpmi_mbox_message msg;
	struct rpmi_perf_opp *opp;
	u32 index = 0;
	int i;
	int ret = 0;

	rx = devm_kcalloc(domain->perf->dev, domain->perf->mpxy_ctx->max_msg_size,
			  sizeof(u32), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	do {
		tx.domain_id = domain->id;
		tx.level_index = index;

		rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_SUPPORTED_LEVELS,
						  &tx, sizeof(tx), rx,
						  domain->perf->mpxy_ctx->max_msg_size);
		ret = rpmi_mbox_send_message(domain->perf->mpxy_ctx->chan, &msg);
		if (ret)
			goto exit;

		if (rx->status) {
			ret = rpmi_to_linux_error(rx->status);
			goto exit;
		}

		if ((index + rx->returned + rx->remaining) != domain->opp_count) {
			ret = -EINVAL;
			goto exit;
		}

		for (i = 0; i < rx->returned; i++) {
			opp = &domain->opp[index + i];
			opp->index = rx->opp[i].index;
			opp->clock_freq = rx->opp[i].clock_freq;
			opp->power_cost = rx->opp[i].power_cost;
			opp->trans_latency_us = rx->opp[i].trans_latency_us;
		}

		index += rx->returned;
	} while (rx->remaining);

exit:
	devm_kfree(domain->perf->dev, rx);

	return ret;
}

static int rpmi_perf_get_levels(struct rpmi_perf_domain *domain, u32 *level_index)
{
	struct rpmi_perf_get_level_tx tx;
	struct rpmi_perf_get_level_rx rx;
	struct rpmi_mbox_message msg;
	int ret = 0;

	tx.domain_id = domain->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_LEVEL,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(domain->perf->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*level_index = rx.level_index;

	return 0;
}

static int rpmi_perf_set_levels(struct rpmi_perf_domain *domain, u32 level_index)
{
	struct rpmi_perf_set_level_tx tx;
	struct rpmi_perf_set_level_rx rx;
	struct rpmi_mbox_message msg;
	int ret = 0;

	tx.domain_id = domain->id;
	tx.level_index = level_index;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_SET_LEVEL,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(domain->perf->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}

static int rpmi_perf_get_fast_channel_regions(struct rpmi_ctx *mpxy_ctx, struct rpmi_perf *perf)
{
	struct rpmi_perf_get_fast_channel_region_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_FAST_CHANNEL_REGION,
					  NULL, 0, &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	perf->fast_channel.addr = rx.addr_low | (((u64)rx.addr_high) << 32);
	perf->fast_channel.size = rx.size_low | (((u64)rx.size_high) << 32);

	return 0;
}

static int rpmi_perf_extract_attributes(struct rpmi_perf_get_fast_channel_attributes_rx *rx,
					struct rpmi_perf_domain *domain,
					struct rpmi_fc_info *fc, u32 service_id)
{
	u64 phys_offset, phys_addr, db_phys_addr;
	struct rpmi_fc_db_info *db = NULL;
	void __iomem *addr, *db_addr;
	u8 db_addr_width;
	u32 flags;

	flags = le32_to_cpu(rx->flags);
	phys_offset = le32_to_cpu(rx->offset_low);
	phys_offset |= (u64)le32_to_cpu(rx->offset_high) << 32;
	phys_addr = domain->perf->fast_channel.addr + phys_offset;

	addr = devm_ioremap(domain->perf->dev, phys_addr, 8);
	if (!addr) {
		dev_err(domain->perf->dev,
			"failed to get fastchannel virtual addr in domain: %d\n",
			domain->id);
		return -EADDRNOTAVAIL;
	}

	switch (service_id) {
	case RPMI_PERF_SRV_GET_LEVEL:
	case RPMI_PERF_SRV_GET_LIMIT:
		fc->get_addr = addr;
		break;
	case RPMI_PERF_SRV_SET_LEVEL:
	case RPMI_PERF_SRV_SET_LIMIT:
		if (SUPPORTS_DOORBELL(flags)) {
			db = devm_kzalloc(domain->perf->dev,
					  sizeof(struct rpmi_fc_db_info),
					  GFP_KERNEL);
			if (!db)
				break;

			db_addr_width = 1 << (DOORBELL_REG_WIDTH(flags) + 3);
			db_phys_addr = le32_to_cpu(rx->db_addr_low);
			db_phys_addr |= (u64)le32_to_cpu(rx->db_addr_high) << 32;

			db_addr = devm_ioremap(domain->perf->dev, db_phys_addr, 8);
			if (!db_addr)
				break;

			db->addr = db_addr;
			db->width = db_addr_width;
			db->set = le32_to_cpu(rx->db_id);
			fc->set_db = db;
		}
		fc->set_addr = addr;

		break;
	}

	return 0;
}

static int rpmi_perf_get_fast_channel_attributes(struct rpmi_perf_domain *domain,
						 struct rpmi_fc_info *fc,
						 u32 service_id)
{
	struct rpmi_perf_get_fast_channel_attributes_tx tx;
	struct rpmi_perf_get_fast_channel_attributes_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = domain->id;
	tx.service_id = service_id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_FAST_CHANNEL_ATTRS,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(domain->perf->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	ret = rpmi_perf_extract_attributes(&rx, domain, fc, service_id);

	return ret;
}

static int rpmi_perf_enumerate(struct rpmi_perf *perf, struct rpmi_perf_domain *domain)
{
	struct rpmi_fc_info *fc;
	int ret, id;

	id = domain->id;

	ret = rpmi_perf_get_attrs(perf, domain);
	if (ret) {
		dev_err(domain->perf->dev,
			"Failed to get attributes of perf domain: #%u\n", id);
		return ret;
	}

	ret = rpmi_perf_get_supported_levels(domain);
	if (ret) {
		dev_err(domain->perf->dev,
			"Failed to get supported level of perf domain: #%u\n", id);
		return ret;
	}

	if (domain->has_fastchannels) {
		fc = devm_kcalloc(domain->perf->dev, RPMI_PERF_FC_MAX, sizeof(*fc),
				  GFP_KERNEL);
		if (!fc)
			return -ENOMEM;

		if (domain->set_level) {
			rpmi_perf_get_fast_channel_attributes(domain,
							      &fc[RPMI_PERF_FC_LEVEL],
							      RPMI_PERF_SRV_SET_LEVEL);
		}

		rpmi_perf_get_fast_channel_attributes(domain,
						      &fc[RPMI_PERF_FC_LEVEL],
						      RPMI_PERF_SRV_GET_LEVEL);

		if (domain->set_limit) {
			rpmi_perf_get_fast_channel_attributes(domain,
							      &fc[RPMI_PERF_FC_LIMIT],
							      RPMI_PERF_SRV_SET_LIMIT);
		}

		rpmi_perf_get_fast_channel_attributes(domain,
						      &fc[RPMI_PERF_FC_LIMIT],
						      RPMI_PERF_SRV_GET_LIMIT);

		domain->fc_info = fc;
	}

	return 0;
}

struct rpmi_perf *rpmi_perf_get(struct device *dev)
{
	return dev_get_drvdata(dev);
}
EXPORT_SYMBOL_GPL(rpmi_perf_get);

struct device *rpmi_perf_get_dev(struct rpmi_perf *perf)
{
	if (!perf)
		return NULL;

	return perf->dev;
}
EXPORT_SYMBOL_GPL(rpmi_perf_get_dev);

int rpmi_perf_domain_count(struct rpmi_perf *perf)
{
	if (!perf)
		return 0;

	return perf->num_domains;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_count);

struct rpmi_perf_domain *rpmi_perf_get_domain(struct rpmi_perf *perf, int id)
{
	if (!perf || id < 0 || id >= perf->num_domains)
		return NULL;

	return &perf->domain[id];
}
EXPORT_SYMBOL_GPL(rpmi_perf_get_domain);

int rpmi_perf_read_level(struct rpmi_perf_domain *domain, u32 *level_index)
{
	if (!domain || !level_index)
		return -EINVAL;

	if (!domain->fc_info || !domain->fc_info[RPMI_PERF_FC_LEVEL].get_addr)
		return rpmi_perf_get_levels(domain, level_index);

	*level_index = ioread32(domain->fc_info[RPMI_PERF_FC_LEVEL].get_addr);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_read_level);

int rpmi_perf_set_level(struct rpmi_perf_domain *domain, u32 level_index)
{
	if (!domain)
		return -EINVAL;

	if (!domain->set_level)
		return -EOPNOTSUPP;

	if (!domain->fc_info || !domain->fc_info[RPMI_PERF_FC_LEVEL].set_addr)
		return rpmi_perf_set_levels(domain, level_index);

	iowrite32(level_index, domain->fc_info[RPMI_PERF_FC_LEVEL].set_addr);
	rpmi_perf_fastchannel_db_ring(domain->fc_info[RPMI_PERF_FC_LEVEL].set_db);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_set_level);

static int rpmi_perf_search_index(const void *id, const void *opp)
{
	return *((u32 *)id) - ((struct rpmi_perf_opp *)opp)->index;
}

static int rpmi_perf_search_freq(const void *id, const void *opp)
{
	return *((u32 *)id) - ((struct rpmi_perf_opp *)opp)->clock_freq;
}

int rpmi_perf_level_to_frequency(struct rpmi_perf_domain *domain, u32 index,
				 u32 *freq_khz)
{
	struct rpmi_perf_opp *found;

	if (!domain || !freq_khz)
		return -EINVAL;

	found = bsearch(&index, domain->opp, domain->opp_count,
			sizeof(domain->opp[0]), rpmi_perf_search_index);
	if (found) {
		*freq_khz = found->clock_freq;
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rpmi_perf_level_to_frequency);

int rpmi_perf_frequency_to_level(struct rpmi_perf_domain *domain, u32 freq_khz,
				 u32 *index)
{
	struct rpmi_perf_opp *found;

	if (!domain || !index)
		return -EINVAL;

	found = bsearch(&freq_khz, domain->opp, domain->opp_count,
			sizeof(domain->opp[0]), rpmi_perf_search_freq);
	if (found) {
		*index = found->index;
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rpmi_perf_frequency_to_level);

int rpmi_perf_dvfs_device_opps_add(const struct rpmi_perf *perf,
				   struct device *dev, int domain_id)
{
	struct rpmi_perf_opp *opp;
	struct rpmi_perf_domain *dom;
	unsigned long freq;
	int idx, ret;

	if (!perf || !dev)
		return -EINVAL;

	if (domain_id < 0 || domain_id >= perf->num_domains)
		return -EINVAL;

	dom = perf->domain + domain_id;

	for (opp = dom->opp, idx = 0; idx < dom->opp_count; idx++, opp++) {
		/* Frequency from RPMI is in kHz */
		freq = opp->clock_freq * 1000;

		ret = dev_pm_opp_add(dev, freq, 0);
		if (ret) {
			dev_warn(dev, "failed to add opp %luHz\n", freq);

			while (idx-- > 0) {
				/* Frequency from RPMI is in kHz */
				freq = (--opp)->clock_freq * 1000;
				dev_pm_opp_remove(dev, freq);
			}
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_dvfs_device_opps_add);

void rpmi_perf_dvfs_device_opps_remove(const struct rpmi_perf *perf,
				       struct device *dev, int domain_id)
{
	struct rpmi_perf_opp *opp;
	struct rpmi_perf_domain *dom;
	unsigned long freq;
	int idx;

	if (!perf || !dev)
		return;

	if (domain_id < 0 || domain_id >= perf->num_domains)
		return;

	dom = perf->domain + domain_id;

	for (opp = dom->opp, idx = 0; idx < dom->opp_count; idx++, opp++) {
		/* Frequency from RPMI is in kHz */
		freq = opp->clock_freq * 1000;
		dev_pm_opp_remove(dev, freq);
	}
}
EXPORT_SYMBOL_GPL(rpmi_perf_dvfs_device_opps_remove);

static int rpmi_perf_attr_setup(struct device *dev, struct rpmi_ctx *mpxy_ctx)
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

	/* Validate performance service group ID */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_SERVICEGROUP_ID);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_dbg(dev, "Failed to get service group ID\n");
		return ret;
	}

	if (msg.attr.value != RPMI_SRVGRP_PERFORMANCE) {
		dev_dbg(dev,
			"service group match failed, expected 0x%x, found 0x%x\n",
			RPMI_SRVGRP_PERFORMANCE, msg.attr.value);
		return -EINVAL;
	}

	/* Validate performance service group version */
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

	/* Get max message size */
	rpmi_mbox_init_get_attribute(&msg, RPMI_MBOX_ATTR_MAX_MSG_DATA_SIZE);
	ret = rpmi_mbox_send_message(mpxy_ctx->chan, &msg);
	if (ret) {
		dev_dbg(dev, "Failed to get max message data size\n");
		return ret;
	}

	mpxy_ctx->max_msg_size = msg.attr.value;

	return 0;
}

static int rpmi_perf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmi_perf *perf;
	struct rpmi_ctx *mpxy_ctx;
	int num_domains = 0;
	int ret, i;

	mpxy_ctx = devm_kzalloc(dev, sizeof(*mpxy_ctx), GFP_KERNEL);
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

	ret = rpmi_perf_attr_setup(dev, mpxy_ctx);
	if (ret) {
		dev_err(dev, "failed to verify RPMI attribute - err:%d\n", ret);
		goto fail_free_channel;
	}

	/* Get number of performance domain */
	ret = rpmi_perf_get_num_domains(mpxy_ctx, &num_domains);
	if (ret) {
		dev_err(dev, "invalid number of perf domains - err:%d\n", ret);
		goto fail_free_channel;
	}

	if (!num_domains) {
		dev_err(dev, "No PM domains found!\n");
		ret = -EINVAL;
		goto fail_free_channel;
	}

	perf = devm_kzalloc(dev, sizeof(*perf), GFP_KERNEL);
	if (!perf) {
		ret = -ENOMEM;
		goto fail_free_channel;
	}

	perf->domain = devm_kcalloc(dev, num_domains,
				    sizeof(struct rpmi_perf_domain),
				    GFP_KERNEL);
	if (!perf->domain) {
		ret = -ENOMEM;
		goto fail_free_channel;
	}

	perf->num_domains = num_domains;
	perf->dev = dev;
	perf->mpxy_ctx = mpxy_ctx;

	/* Get fast channel region for performance domain */
	ret = rpmi_perf_get_fast_channel_regions(mpxy_ctx, perf);
	if (ret)
		dev_err(dev, "invalid fast channel region\n");

	for (i = 0; i < num_domains; i++) {
		struct rpmi_perf_domain *domain = &perf->domain[i];

		domain->id = i;
		domain->perf = perf;

		rpmi_perf_enumerate(perf, domain);
	}

	dev_set_drvdata(dev, perf);

#if IS_ENABLED(CONFIG_RISCV_RPMI_CPUFREQ)
	if (rpmi_perf_has_consumer(dev, true)) {
		perf->cpufreq_pdev = platform_device_register_data(dev,
								   "riscv-rpmi-performance-cpufreq",
								   PLATFORM_DEVID_AUTO,
								   NULL, 0);
		if (IS_ERR(perf->cpufreq_pdev)) {
			dev_warn(dev, "failed to register cpufreq child: %ld\n",
				 PTR_ERR(perf->cpufreq_pdev));
			perf->cpufreq_pdev = NULL;
		}
	}
#endif

#if IS_ENABLED(CONFIG_RISCV_RPMI_DEVFREQ)
	if (rpmi_perf_has_consumer(dev, false)) {
		perf->devfreq_pdev = platform_device_register_data(dev,
								   "riscv-rpmi-performance-devfreq",
								   PLATFORM_DEVID_AUTO,
								   NULL, 0);
		if (IS_ERR(perf->devfreq_pdev)) {
			dev_warn(dev, "failed to register devfreq child: %ld\n",
				 PTR_ERR(perf->devfreq_pdev));
			perf->devfreq_pdev = NULL;
		}
	}
#endif

	dev_info(dev, "%d RPMI perf domains registered\n", num_domains);

	return 0;

fail_free_channel:
	mbox_free_channel(mpxy_ctx->chan);

	return ret;
}

static void rpmi_perf_remove(struct platform_device *pdev)
{
	struct rpmi_perf *perf = dev_get_drvdata(&pdev->dev);

	if (!perf || !perf->mpxy_ctx)
		return;

#if IS_ENABLED(CONFIG_RISCV_RPMI_DEVFREQ)
	if (perf->devfreq_pdev)
		platform_device_unregister(perf->devfreq_pdev);
#endif

#if IS_ENABLED(CONFIG_RISCV_RPMI_CPUFREQ)
	if (perf->cpufreq_pdev)
		platform_device_unregister(perf->cpufreq_pdev);
#endif

	mbox_free_channel(perf->mpxy_ctx->chan);
}

static const struct of_device_id rpmi_perf_of_match[] = {
	{ .compatible = "riscv,rpmi-performance" },
	{ },
};
MODULE_DEVICE_TABLE(of, rpmi_perf_of_match);

static struct platform_driver rpmi_perf_platdrv = {
	.driver = {
		.name = "riscv-rpmi-performance",
		.of_match_table = rpmi_perf_of_match,
	},
	.probe = rpmi_perf_probe,
	.remove = rpmi_perf_remove,
};

module_platform_driver(rpmi_perf_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("RPMI performance service provider");
MODULE_LICENSE("GPL");
