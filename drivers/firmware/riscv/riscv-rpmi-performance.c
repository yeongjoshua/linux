// SPDX-License-Identifier: GPL-2.0
/*
 * RISC-V RPMI performance service group core
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 *
 * Owns the MPXY mailbox channel of the RPMI performance service group and the
 * enumeration of the performance domains behind it, and hands both to whoever
 * drives a domain: cpufreq for a domain shared by CPUs, a device driver for
 * any other IP block. The channel cannot be shared, so it has a single owner
 * here and the front-ends sit on top of it.
 */

#define pr_fmt(fmt) "riscv-rpmi-performance: " fmt

#include <linux/bitfield.h>
#include <linux/bsearch.h>
#include <linux/firmware/riscv/riscv-rpmi-performance.h>
#include <linux/io.h>
#include <linux/mailbox/riscv-rpmi-message.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#define RPMI_PERF_DOMAIN_NAME_LEN	16

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

struct rpmi_perf_domain {
	u32 id;
	bool set_limit;
	bool set_level;
	bool has_fastchannels;
	u32 level_count;
	u32 rate_limit_us;
	char name[RPMI_PERF_DOMAIN_NAME_LEN];
	struct rpmi_perf_level *level;
	struct rpmi_fc_info *fc_info;
	struct rpmi_ctx *mpxy_ctx;
	struct device *dev;
	struct rpmi_perf *perf;
};

struct rpmi_perf {
	u32 num_domains;
	struct rpmi_fast_channel fast_channel;
	struct device *dev;
	struct rpmi_perf_domain *domain;
	struct list_head node;
};

enum {
	RPMI_PERF_FC_LEVEL,
	RPMI_PERF_FC_LIMIT,
	RPMI_PERF_FC_MAX,
};

/*
 * Providers register here as they probe, so that a consumer can resolve its
 * "performance-domains" phandle and defer until the provider shows up.
 */
static LIST_HEAD(rpmi_perf_providers);
static DEFINE_MUTEX(rpmi_perf_providers_lock);

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
	struct rpmi_perf_level level[];
};

/* Service: GET_LEVEL */
struct rpmi_perf_get_level_tx {
	u32 domain_id;
};

struct rpmi_perf_get_level_rx {
	s32 status;
	u32 level_index;
};

/* Service: SET_LEVEL */
struct rpmi_perf_set_level_tx {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_perf_set_level_rx {
	s32 status;
};

/* Service: GET_LIMIT */
struct rpmi_perf_get_limit_tx {
	u32 domain_id;
};

struct rpmi_perf_get_limit_rx {
	s32 status;
	u32 max_level;
	u32 min_level;
};

/* Service: SET_LIMIT */
struct rpmi_perf_set_limit_tx {
	u32 domain_id;
	u32 max_level;
	u32 min_level;
};

struct rpmi_perf_set_limit_rx {
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

static int rpmi_perf_get_attrs(struct rpmi_perf *mpxy_perf, struct rpmi_perf_domain *perf_domain)
{
	struct rpmi_perf_get_attrs_tx tx;
	struct rpmi_perf_get_attrs_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = perf_domain->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_ATTRIBUTES,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(perf_domain->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	perf_domain->set_limit = PERF_LIMIT_SETTING(rx.flags);
	perf_domain->set_level = PERF_LEVEL_SETTING(rx.flags);
	perf_domain->has_fastchannels = mpxy_perf->fast_channel.size &&
					(perf_domain->set_limit || perf_domain->set_level) ?
					FAST_CHANNEL_SUPPORT(rx.flags) : false;
	perf_domain->level_count = rx.num_levels;
	perf_domain->rate_limit_us = rx.trans_latency_us;
	strscpy(perf_domain->name, rx.name, RPMI_PERF_DOMAIN_NAME_LEN);

	if (!perf_domain->level_count)
		return -EINVAL;

	perf_domain->level = devm_kcalloc(mpxy_perf->dev, perf_domain->level_count,
					  sizeof(struct rpmi_perf_level), GFP_KERNEL);
	if (!perf_domain->level)
		return -ENOMEM;

	return 0;
}

static int rpmi_perf_get_supported_levels(struct rpmi_perf_domain *perf_domain)
{
	struct rpmi_perf_get_supported_level_tx tx;
	struct rpmi_perf_get_supported_level_rx *rx;
	struct rpmi_mbox_message msg;
	struct rpmi_perf_level *level;
	u32 index = 0;
	int ret = 0;

	rx = devm_kcalloc(perf_domain->dev, perf_domain->mpxy_ctx->max_msg_size,
			  sizeof(u32), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	do {
		tx.domain_id = perf_domain->id;
		tx.level_index = index;

		rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_SUPPORTED_LEVELS,
						  &tx, sizeof(tx), rx,
						  perf_domain->mpxy_ctx->max_msg_size);
		ret = rpmi_mbox_send_message(perf_domain->mpxy_ctx->chan, &msg);
		if (ret)
			goto exit;

		if (rx->status) {
			ret = rpmi_to_linux_error(rx->status);
			goto exit;
		}

		if ((index + rx->returned + rx->remaining) != perf_domain->level_count) {
			ret = -EINVAL;
			goto exit;
		}

		for (int i = 0; i < rx->returned; i++) {
			level = &perf_domain->level[index + i];
			level->index = rx->level[i].index;
			level->clock_freq = rx->level[i].clock_freq;
			level->power_cost = rx->level[i].power_cost;
			level->trans_latency_us = rx->level[i].trans_latency_us;
		}

		index += rx->returned;

	} while (rx->remaining);

exit:
	devm_kfree(perf_domain->dev, rx);

	return ret;
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

	addr = devm_ioremap(domain->dev, phys_addr, 8);
	if (!addr) {
		dev_err(domain->dev,
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
			db = devm_kzalloc(domain->dev,
					  sizeof(struct rpmi_fc_db_info),
					  GFP_KERNEL);
			if (!db)
				break;

			db_addr_width = 1 << (DOORBELL_REG_WIDTH(flags) + 3);
			db_phys_addr = le32_to_cpu(rx->db_addr_low);
			db_phys_addr |= (u64)le32_to_cpu(rx->db_addr_high) << 32;

			db_addr = devm_ioremap(domain->dev, db_phys_addr, 8);
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

static int rpmi_perf_get_fast_channel_attributes(struct rpmi_perf_domain *perf_domain,
						 struct rpmi_fc_info *fc,
						 u32 service_id)
{
	struct rpmi_perf_get_fast_channel_attributes_tx tx;
	struct rpmi_perf_get_fast_channel_attributes_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = perf_domain->id;
	tx.service_id = service_id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_FAST_CHANNEL_ATTRS,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(perf_domain->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	ret = rpmi_perf_extract_attributes(&rx, perf_domain, fc, service_id);

	return ret;
}

static int rpmi_perf_enumerate(struct rpmi_perf *mpxy_perf, struct rpmi_perf_domain *domain)
{
	struct rpmi_fc_info *fc;
	int ret, id;

	id = domain->id;

	ret = rpmi_perf_get_attrs(mpxy_perf, domain);
	if (ret) {
		dev_err(domain->dev,
			"Failed to get attributes of perf domain: #%u\n", id);
		return ret;
	}

	ret = rpmi_perf_get_supported_levels(domain);
	if (ret) {
		dev_err(domain->dev,
			"Failed to get supported level of perf domain: #%u\n", id);
		return ret;
	}

	if (domain->has_fastchannels) {
		fc = devm_kcalloc(domain->dev, RPMI_PERF_FC_MAX, sizeof(*fc), GFP_KERNEL);
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

/*
 * Domain accessors. A consumer only ever sees an opaque handle, so the
 * enumeration above stays private to this file.
 */

const char *rpmi_perf_domain_name(struct rpmi_perf_domain *pd)
{
	return pd->name;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_name);

u32 rpmi_perf_domain_level_count(struct rpmi_perf_domain *pd)
{
	return pd->level_count;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_level_count);

int rpmi_perf_domain_level_info(struct rpmi_perf_domain *pd, u32 idx,
				struct rpmi_perf_level *level)
{
	if (idx >= pd->level_count)
		return -EINVAL;

	*level = pd->level[idx];

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_level_info);

bool rpmi_perf_domain_can_set_level(struct rpmi_perf_domain *pd)
{
	return pd->set_level;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_can_set_level);

bool rpmi_perf_domain_has_fast_channel(struct rpmi_perf_domain *pd)
{
	return pd->fc_info && pd->fc_info[RPMI_PERF_FC_LEVEL].set_addr;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_has_fast_channel);

u32 rpmi_perf_domain_trans_latency_us(struct rpmi_perf_domain *pd)
{
	return pd->rate_limit_us;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_trans_latency_us);

struct rpmi_perf_domain *rpmi_perf_domain_by_id(struct rpmi_perf *perf, u32 id)
{
	if (!perf || id >= perf->num_domains)
		return NULL;

	return &perf->domain[id];
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_by_id);

u32 rpmi_perf_num_domains(struct rpmi_perf *perf)
{
	return perf ? perf->num_domains : 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_num_domains);

/**
 * rpmi_perf_domain_get_level() - read the level a domain is running at
 * @pd: performance domain
 * @level: where the level index is stored
 *
 * Reads through the fast channel when the domain has one, and over the
 * mailbox otherwise. Either way the value comes from the platform
 * microcontroller rather than from a cached copy.
 *
 * Return: 0 on success, a negative errno otherwise.
 */
int rpmi_perf_domain_get_level(struct rpmi_perf_domain *pd, u32 *level)
{
	struct rpmi_perf_get_level_tx tx;
	struct rpmi_perf_get_level_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	if (pd->fc_info && pd->fc_info[RPMI_PERF_FC_LEVEL].get_addr) {
		*level = ioread32(pd->fc_info[RPMI_PERF_FC_LEVEL].get_addr);
		return 0;
	}

	tx.domain_id = pd->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_LEVEL,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(pd->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*level = rx.level_index;

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_get_level);

/**
 * rpmi_perf_domain_set_level() - ask a domain to run at a level
 * @pd: performance domain
 * @level: level index, as advertised by the domain
 *
 * May sleep, because it goes over the mailbox. Use
 * rpmi_perf_domain_set_level_fast() from a context that must not.
 *
 * Return: 0 on success, a negative errno otherwise.
 */
int rpmi_perf_domain_set_level(struct rpmi_perf_domain *pd, u32 level)
{
	struct rpmi_perf_set_level_tx tx;
	struct rpmi_perf_set_level_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	if (!pd->set_level)
		return -EOPNOTSUPP;

	tx.domain_id = pd->id;
	tx.level_index = level;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_SET_LEVEL,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(pd->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_set_level);

/**
 * rpmi_perf_domain_set_level_fast() - set a level without sleeping
 * @pd: performance domain
 * @level: level index, as advertised by the domain
 *
 * Writes the level through the domain's fast channel and rings its doorbell.
 * The platform microcontroller applies it asynchronously and there is no
 * status to read back, so this only reports whether the request was posted.
 *
 * Return: 0 on success, -EOPNOTSUPP if the domain has no fast channel.
 */
int rpmi_perf_domain_set_level_fast(struct rpmi_perf_domain *pd, u32 level)
{
	if (!rpmi_perf_domain_has_fast_channel(pd))
		return -EOPNOTSUPP;

	iowrite32(level, pd->fc_info[RPMI_PERF_FC_LEVEL].set_addr);
	rpmi_perf_fastchannel_db_ring(pd->fc_info[RPMI_PERF_FC_LEVEL].set_db);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_set_level_fast);

int rpmi_perf_domain_get_limit(struct rpmi_perf_domain *pd, u32 *min, u32 *max)
{
	struct rpmi_perf_get_limit_tx tx;
	struct rpmi_perf_get_limit_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	tx.domain_id = pd->id;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_GET_LIMIT,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(pd->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	*max = rx.max_level;
	*min = rx.min_level;

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_get_limit);

int rpmi_perf_domain_set_limit(struct rpmi_perf_domain *pd, u32 min, u32 max)
{
	struct rpmi_perf_set_limit_tx tx;
	struct rpmi_perf_set_limit_rx rx;
	struct rpmi_mbox_message msg;
	int ret;

	if (!pd->set_limit)
		return -EOPNOTSUPP;

	tx.domain_id = pd->id;
	tx.max_level = max;
	tx.min_level = min;

	rpmi_mbox_init_send_with_response(&msg, RPMI_PERF_SRV_SET_LIMIT,
					  &tx, sizeof(tx), &rx, sizeof(rx));
	ret = rpmi_mbox_send_message(pd->mpxy_ctx->chan, &msg);
	if (ret)
		return ret;

	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_set_limit);

static int rpmi_perf_search_index(const void *id, const void *level)
{
	return *((u32 *)id) - ((struct rpmi_perf_level *)level)->index;
}

static int rpmi_perf_search_freq(const void *id, const void *level)
{
	return *((u32 *)id) - ((struct rpmi_perf_level *)level)->clock_freq;
}

int rpmi_perf_domain_level_to_freq(struct rpmi_perf_domain *pd, u32 level, u32 *khz)
{
	struct rpmi_perf_level *found;

	found = bsearch(&level, pd->level, pd->level_count,
			sizeof(pd->level[0]), rpmi_perf_search_index);
	if (!found)
		return -EINVAL;

	*khz = found->clock_freq;

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_level_to_freq);

int rpmi_perf_domain_freq_to_level(struct rpmi_perf_domain *pd, u32 khz, u32 *level)
{
	struct rpmi_perf_level *found;

	found = bsearch(&khz, pd->level, pd->level_count,
			sizeof(pd->level[0]), rpmi_perf_search_freq);
	if (!found)
		return -EINVAL;

	*level = found->index;

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_freq_to_level);

/**
 * rpmi_perf_domain_opps_add() - give @dev an OPP for every level of @pd
 * @pd: performance domain
 * @dev: device that runs in the domain
 *
 * The levels come from the platform microcontroller, so there is no
 * operating-points-v2 table in the device tree to parse. The OPP frequency is
 * the level's clock frequency in Hz and the OPP level is the RPMI level
 * index, which is what the set and get services take.
 *
 * Return: 0 on success, a negative errno otherwise.
 */
int rpmi_perf_domain_opps_add(struct rpmi_perf_domain *pd, struct device *dev)
{
	struct dev_pm_opp_data data = {};
	struct rpmi_perf_level *level;
	int idx, ret;

	for (level = pd->level, idx = 0; idx < pd->level_count; idx++, level++) {
		/* Frequency from RPMI is in kHz */
		data.freq = (unsigned long)level->clock_freq * 1000;
		data.level = level->index;

		ret = dev_pm_opp_add_dynamic(dev, &data);
		if (ret) {
			dev_warn(dev, "failed to add opp %luHz\n", data.freq);

			while (idx-- > 0) {
				/* Frequency from RPMI is in kHz */
				dev_pm_opp_remove(dev,
						  (unsigned long)(--level)->clock_freq * 1000);
			}
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rpmi_perf_domain_opps_add);

static struct rpmi_perf *rpmi_perf_find_provider(struct device_node *np)
{
	struct rpmi_perf *perf;

	guard(mutex)(&rpmi_perf_providers_lock);

	list_for_each_entry(perf, &rpmi_perf_providers, node) {
		if (dev_of_node(perf->dev) == np)
			return perf;
	}

	return NULL;
}

static void rpmi_perf_domain_release(void *data)
{
	put_device((struct device *)data);
}

/**
 * devm_rpmi_perf_domain_get() - resolve a "performance-domains" phandle
 * @dev: consumer device, whose node carries the phandle
 * @index: index into the "performance-domains" property
 *
 * Return: the performance domain on success, or an ERR_PTR. -EPROBE_DEFER
 * means the phandle resolves to an RPMI performance provider that has not
 * probed yet, so the caller should try again later.
 */
struct rpmi_perf_domain *devm_rpmi_perf_domain_get(struct device *dev, int index)
{
	struct of_phandle_args args;
	struct rpmi_perf_domain *pd;
	struct rpmi_perf *perf;
	int ret;

	if (!dev_of_node(dev))
		return ERR_PTR(-ENODEV);

	ret = of_parse_phandle_with_args(dev_of_node(dev), "performance-domains",
					 "#performance-domain-cells", index, &args);
	if (ret)
		return ERR_PTR(ret);

	perf = rpmi_perf_find_provider(args.np);
	of_node_put(args.np);
	if (!perf)
		return ERR_PTR(-EPROBE_DEFER);

	if (args.args_count < 1)
		return ERR_PTR(-EINVAL);

	pd = rpmi_perf_domain_by_id(perf, args.args[0]);
	if (!pd)
		return ERR_PTR(-EINVAL);

	get_device(perf->dev);
	ret = devm_add_action_or_reset(dev, rpmi_perf_domain_release, perf->dev);
	if (ret)
		return ERR_PTR(ret);

	return pd;
}
EXPORT_SYMBOL_GPL(devm_rpmi_perf_domain_get);

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

static void rpmi_perf_mbox_chan_release(void *data)
{
	mbox_free_channel((struct mbox_chan *)data);
}

static void rpmi_perf_provider_unregister(void *data)
{
	struct rpmi_perf *perf = data;

	guard(mutex)(&rpmi_perf_providers_lock);
	list_del(&perf->node);
}

/*
 * cpufreq has no device tree node of its own: a CPU names its domain through
 * "performance-domains" on the CPU node. Create the device it binds to, but
 * only when some CPU actually points back here.
 */
static bool rpmi_perf_cpus_present(struct device *dev)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int ret;

	for_each_of_cpu_node(cpu_np) {
		ret = of_parse_phandle_with_args(cpu_np, "performance-domains",
						 "#performance-domain-cells", 0,
						 &args);
		if (ret)
			continue;

		if (args.np == dev_of_node(dev)) {
			of_node_put(args.np);
			of_node_put(cpu_np);
			return true;
		}
		of_node_put(args.np);
	}

	return false;
}

static void rpmi_perf_cpufreq_unregister(void *data)
{
	platform_device_unregister((struct platform_device *)data);
}

static int rpmi_perf_cpufreq_register(struct device *dev, struct rpmi_perf *perf)
{
	struct platform_device *pdev;

	if (!IS_ENABLED(CONFIG_RISCV_RPMI_CPUFREQ) || !rpmi_perf_cpus_present(dev))
		return 0;

	pdev = platform_device_register_data(dev, "riscv-rpmi-cpufreq",
					     PLATFORM_DEVID_NONE, &perf,
					     sizeof(perf));
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	return devm_add_action_or_reset(dev, rpmi_perf_cpufreq_unregister, pdev);
}

static int rpmi_perf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmi_perf *mpxy_perf;
	struct rpmi_ctx *mpxy_ctx;
	u32 num_domains = 0;
	int ret;
	u32 i;

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

	ret = devm_add_action_or_reset(dev, rpmi_perf_mbox_chan_release,
				       mpxy_ctx->chan);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add rpmi mbox channel cleanup\n");

	ret = rpmi_perf_attr_setup(dev, mpxy_ctx);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to verify RPMI attribute\n");

	/* Get number of performance domain */
	ret = rpmi_perf_get_num_domains(mpxy_ctx, &num_domains);
	if (ret)
		return dev_err_probe(dev, ret,
				     "invalid number of perf domains\n");

	if (!num_domains)
		return dev_err_probe(dev, -EINVAL, "No perf domains found!\n");

	mpxy_perf = devm_kzalloc(dev, sizeof(*mpxy_perf), GFP_KERNEL);
	if (!mpxy_perf)
		return -ENOMEM;

	mpxy_perf->domain = devm_kcalloc(dev, num_domains,
					 sizeof(struct rpmi_perf_domain),
					 GFP_KERNEL);
	if (!mpxy_perf->domain)
		return -ENOMEM;

	mpxy_perf->num_domains = num_domains;
	mpxy_perf->dev = dev;
	INIT_LIST_HEAD(&mpxy_perf->node);

	/* Get fast channel region for performance domain */
	ret = rpmi_perf_get_fast_channel_regions(mpxy_ctx, mpxy_perf);
	if (ret)
		dev_err(dev, "invalid fast channel region\n");

	for (i = 0; i < num_domains; i++) {
		struct rpmi_perf_domain *domain = &mpxy_perf->domain[i];

		domain->id = i;
		domain->mpxy_ctx = mpxy_ctx;
		domain->dev = dev;
		domain->perf = mpxy_perf;

		rpmi_perf_enumerate(mpxy_perf, domain);
	}

	scoped_guard(mutex, &rpmi_perf_providers_lock)
		list_add_tail(&mpxy_perf->node, &rpmi_perf_providers);

	ret = devm_add_action_or_reset(dev, rpmi_perf_provider_unregister, mpxy_perf);
	if (ret)
		return ret;

	dev_set_drvdata(dev, mpxy_perf);

	ret = rpmi_perf_cpufreq_register(dev, mpxy_perf);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register cpufreq device\n");

	dev_info(dev, "%d MPXY performance domains registered\n", num_domains);

	return 0;
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
};

module_platform_driver(rpmi_perf_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("RISC-V RPMI performance service group core");
MODULE_LICENSE("GPL");
