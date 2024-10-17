// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RISC-V MPXY Based CPUFreq Interface Driver
 *
 * Copyright (C) 2023-2024 Shanghai StarFive Technology Co., Ltd.
 *
 * Implements a CPUFreq driver on top of SBI RPMI Proxy Extension (MPXY)
 *
 * Each SBI MPXY CPUFreq instance is associated, through the means of a proper DT
 * entry description, to a specific Transport ID.
 */

#define pr_fmt(fmt) "sbi-mpxy-cpufreq: " fmt

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/energy_model.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/units.h>
#include <asm/sbi.h>
#include <asm/rpmi.h>

#define ATTR_COUNT(t, f)	(t - f + 1)

#define SBI_MPXY_PERF_NAME_LEN	16
#define SBI_MPXY_PERF_MAX_OPPS	16

#define DOORBELL_REG_MASK	GENMASK(2, 1)
#define DOORBELL_SUPPORT_MASK	BIT(0)

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

/* event notification IDs */
enum sbi_mpxy_perf_notification_event_ids {
	SBI_MPXY_PERF_POWER_CHANGE = 1,	/* performance power change */
	SBI_MPXY_PERF_LIMIT_CHANGE = 2,	/* performance limit change */
	SBI_MPXY_PERF_LEVEL_CHANGE = 3,	/* performance level change */
	SBI_MPXY_PERF_EVENT_MAX_IDX,
};

/* fastchannel operation types
 *
 * @SBI_MPXY_PERF_FC_LEVEL - get or set domain perf level using fastchannel
 * @SBI_MPXY_PERF_FC_LIMIT - get or set domain perf limits using fastchannel
 * @SBI_MPXY_PERF_FC_MAX - maximum number of fastchannel operations
 *
 */
enum {
	SBI_MPXY_PERF_FC_LEVEL,
	SBI_MPXY_PERF_FC_LIMIT,
	SBI_MPXY_PERF_FC_MAX,
};

/* describes a fastchannel doorbell */
struct sbi_mpxy_fc_db_info {
	int width;
	u64 set;
	u64 mask;
	void __iomem *addr;
};

/* describes a fastchannel */
struct sbi_mpxy_fc_info {
	void __iomem *set_addr;
	void __iomem *get_addr;
	struct sbi_mpxy_fc_db_info *set_db;
};

/* Perf system */
struct sbi_mpxy_ctx {
	/* transport id */
	u32 channel_id;
	u32 max_msg_len;
};

static struct sbi_mpxy_ctx mpxy_ctx;

enum sbi_mpxy_power_scale {
	SBI_MPXY_POWER_BOGOWATTS,
	SBI_MPXY_POWER_MILLIWATTS,
	SBI_MPXY_POWER_MICROWATTS,
};

/**
 * struct sbi_mpxy_perf_opp - describe an operating performance point
 *
 * @perf_level: performance level value
 * @power_cost: power cost in microwatt. Value 0 means power cost not
 *              returned by PuC
 * @trans_latency_us: worst case latency time (uS) to switch from any supported
 *                    performance level to the level in this entry
 */
struct sbi_mpxy_perf_opp {
	u32 perf_level;
	u32 power_cost;
	u32 trans_latency_us;
};

/**
 * struct sbi_mpxy_perf - describe one available MPXY Perf Domain
 *
 * @id: the domain ID as advertised by PuC
 * @set_limits: capability to set performance limits
 * @set_perf: capability to set performance level
 * @perf_fastchannels: capability to support fastchannels
 * @opp_count: number of available performance levels for this domain
 *             advertised by PuC
 * @rate_limit_us: min time required between two consecutive requests
 * @name: domain name assigned by PuC
 * @opp: table of performance operating points
 * @fc_info: fastchannel information
 * @dev: device associated with this perf domain
 */
struct sbi_mpxy_perf {
	u32 id;
	bool set_limits;
	bool set_perf;
	bool perf_fastchannels;
	u32 opp_count;
	u32 rate_limit_us;
	char name[SBI_MPXY_PERF_NAME_LEN];
	struct sbi_mpxy_perf_opp *opp;
	struct sbi_mpxy_fc_info *fc_info;
	struct device *dev;
};

/**
 * struct sbi_mpxy_perf_info - describe the MPXY Perf Service System
 *
 * @num_domains: total number of perf domains available in the system
 *		 as advertised by PuC
 * @device: pointer to the MPXY Perf Service system
 * @sperf: array of MPXY Perf domain information
 *
 */
struct sbi_mpxy_perf_info {
	u32 num_domains;
	enum sbi_mpxy_power_scale power_scale;
	struct device *dev;
	struct sbi_mpxy_perf *sperf;
};

/* Service: ENABLE_NOTIFICATION */
struct rpmi_perf_enable_notification_tx {
	u32 event_id;
};

struct rpmi_perf_enable_notification_rx {
	s32 status;
};

/* Service: GET_PERF_DOMAINS */
struct rpmi_perf_get_num_domain_rx {
	s32 status;
	u32 num_domains;
};

/* Service: GET_DOMAIN_ATTRS */
struct rpmi_perf_get_domain_attrs_tx {
	u32 domain_id;
};

struct rpmi_perf_get_domain_attrs_rx {
	u32 status;
	u32 flags;
#define PERF_LIMIT_SETTING(f)		(FIELD_GET(BIT(10), (f)))
#define PERF_LEVEL_SETTING(f)		(FIELD_GET(BIT(9), (f)))
#define FAST_CHANNEL_SUPPORT(f)		(FIELD_GET(BIT(8), (f)))
#define TOTAL_NUM_PERF_LEVELS(f)	(FIELD_GET(GENMASK(7, 0), (f)))
	u32 rate_limit_us;
	char name[SBI_MPXY_PERF_NAME_LEN];
};

/* Service: GET_DOMAIN_LEVELS */
struct rpmi_perf_get_domain_levels_tx {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_perf_get_domain_levels_rx {
	s32 status;
	u32 flags;
	u32 remaining_items;
	u32 returned_items;
	u32 perf_levels[][3];
};

/* Service: GET_PERF_LEVEL */
struct rpmi_perf_get_perf_level_tx {
	u32 domain_id;
};

struct rpmi_perf_get_perf_level_rx {
	s32 status;
	u32 perf_level;
};

/* Service: SET_PERF_LEVEL */
struct rpmi_perf_set_perf_level_tx {
	u32 domain_id;
	u32 perf_level;
};

struct rpmi_perf_set_perf_level_rx {
	s32 status;
};

/* Service: GET_PERF_LIMIT */
struct rpmi_perf_get_perf_limit_tx {
	u32 domain_id;
};

struct rpmi_perf_get_perf_limit_rx {
	s32 status;
	u32 max_perf_level;
	u32 min_perf_level;
};

/* Service: SET_PERF_LIMIT */
struct rpmi_perf_set_perf_limit_tx {
	u32 domain_id;
	u32 max_perf_level;
	u32 min_perf_level;
};

struct rpmi_perf_set_perf_limit_rx {
	s32 status;
};

/* Service: GET_PERF_DOMAIN_FAST_CHANNEL */
struct rpmi_perf_get_fc_addr_tx {
	u32 domain_id;
	u32 service_id;
};

struct rpmi_perf_get_fc_addr_rx {
	s32 status;
	u32 flags;
#define DOORBELL_REG_WIDTH(f)	(FIELD_GET(DOORBELL_REG_MASK, (f)))
#define SUPPORTS_DOORBELL(f)	(FIELD_GET(DOORBELL_SUPPORT_MASK, (f)))
	u32 chan_addr_low;
	u32 chan_addr_high;
	u32 db_addr_low;
	u32 db_addr_high;
	u32 db_id_low;
	u32 db_id_high;
	u32 db_preserved_lmask;
	u32 db_preserved_hmask;
};

static struct sbi_mpxy_perf_info *pinfo;

struct sbi_mpxy_cpufreq_data {
	int cpufreq_domain_id;
	int nr_opp;
	struct device *cpu_dev;
};

#define SBI_MPXY_PROTO_FC_RING_DB(w)			\
do {							\
	u##w val = 0;					\
							\
	if (db->mask)					\
		val = ioread##w(db->addr) & db->mask;	\
	iowrite##w((u##w)db->set | val, db->addr);	\
} while (0)

static void sbi_mpxy_fastchannel_db_ring(struct sbi_mpxy_fc_db_info *db)
{
	if (!db || !db->addr)
		return;

	if (db->width == 1)
		SBI_MPXY_PROTO_FC_RING_DB(8);
	else if (db->width == 2)
		SBI_MPXY_PROTO_FC_RING_DB(16);
	else if (db->width == 4)
		SBI_MPXY_PROTO_FC_RING_DB(32);
	else /* db->width == 8 */
#ifdef CONFIG_64BIT
		SBI_MPXY_PROTO_FC_RING_DB(64);
#else
	{
		u64 val = 0;

		if (db->mask)
			val = ioread64_hi_lo(db->addr) & db->mask;
		iowrite64_hi_lo(db->set | val, db->addr);
	}
#endif
}

static __attribute__ ((unused))
int sbi_mpxy_perf_limits_set(struct sbi_mpxy_perf *mpxy_perf, u32 domain_id,
				 u32 max_perf, u32 min_perf)
{
	int ret = 0;
	unsigned long rxmsg_len;
	struct rpmi_perf_set_perf_limit_tx tx;
	struct rpmi_perf_set_perf_limit_rx rx;

	if (!mpxy_perf->set_limits) {
		dev_err(mpxy_perf->dev,
			"perf domain #%u - set perf limits is not supportedi\n",
			mpxy_perf->id);
		return -EOPNOTSUPP;
	}

	if (mpxy_perf->fc_info && mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LIMIT].set_addr) {
		struct sbi_mpxy_fc_info *fci = &mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LIMIT];

		iowrite32(max_perf, fci->set_addr);
		iowrite32(min_perf, fci->set_addr + 4);
		sbi_mpxy_fastchannel_db_ring(fci->set_db);
		return 0;
	}

	tx.domain_id = cpu_to_le32(domain_id);
	tx.max_perf_level = cpu_to_le32(max_perf);
	tx.min_perf_level = cpu_to_le32(min_perf);
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_SET_PERF_LIMIT,
					     &tx,
					     sizeof(struct rpmi_perf_set_perf_limit_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"set domain #%u perf limits failed with error: %d",
			mpxy_perf->id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"set domain #%u perf limits failed with RPMI error: %d",
			mpxy_perf->id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	return rx.status;
}

static __attribute__ ((unused))
int sbi_mpxy_perf_limits_get(struct sbi_mpxy_perf *mpxy_perf, u32 domain_id,
			     u32 *max_perf, u32 *min_perf)
{
	int ret = 0;
	unsigned long rxmsg_len;
	struct rpmi_perf_get_perf_limit_tx tx;
	struct rpmi_perf_get_perf_limit_rx rx;

	if (mpxy_perf->fc_info && mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LIMIT].get_addr) {
		struct sbi_mpxy_fc_info *fci = &mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LIMIT];

		*max_perf = ioread32(fci->get_addr);
		*min_perf = ioread32(fci->get_addr + 4);
		return 0;
	}

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_GET_PERF_LIMIT,
					     &tx,
					     sizeof(struct rpmi_perf_get_perf_limit_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"get domain #%u perf limits failed with error: %d",
			mpxy_perf->id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"get domain #%u perf limits failed with RPMI error: %d",
			mpxy_perf->id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	*max_perf = le32_to_cpu(rx.max_perf_level);
	*min_perf = le32_to_cpu(rx.min_perf_level);

	return rx.status;
}

static int sbi_mpxy_perf_level_set(struct sbi_mpxy_perf *mpxy_perf, u32 domain_id,
				   u32 level)
{
	int ret = 0;
	unsigned long rxmsg_len;
	struct rpmi_perf_set_perf_level_tx tx;
	struct rpmi_perf_set_perf_level_rx rx;

	if (!mpxy_perf->set_perf) {
		dev_err(mpxy_perf->dev,
			"perf domain #%u - set perf level is not supported\n",
			mpxy_perf->id);
		return -EOPNOTSUPP;
	}

	if (mpxy_perf->fc_info && mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LEVEL].set_addr) {
		struct sbi_mpxy_fc_info *fci = &mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LEVEL];

		iowrite32(level, fci->set_addr);
		sbi_mpxy_fastchannel_db_ring(fci->set_db);
		return 0;
	}

	tx.domain_id = cpu_to_le32(domain_id);
	tx.perf_level = cpu_to_le32(level);
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_SET_PERF_LEVEL,
					     &tx,
					     sizeof(struct rpmi_perf_set_perf_level_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"set domain #%u perf level failed with error: %d",
			mpxy_perf->id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"set domain #%u perf level failed with RPMI error: %d",
			mpxy_perf->id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	return rx.status;
}

static int sbi_mpxy_perf_level_get(struct sbi_mpxy_perf *mpxy_perf, u32 domain_id,
				   u32 *level)
{
	int ret = 0;
	unsigned long rxmsg_len;
	struct rpmi_perf_get_perf_level_tx tx;
	struct rpmi_perf_get_perf_level_rx rx;

	if (mpxy_perf->fc_info && mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LEVEL].get_addr) {
		struct sbi_mpxy_fc_info *fci = &mpxy_perf->fc_info[SBI_MPXY_PERF_FC_LEVEL];

		*level = ioread32(fci->get_addr);
		return 0;
	}

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_GET_PERF_LEVEL,
					     &tx, sizeof(struct rpmi_perf_get_perf_level_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"get domain #%u perf level failed with error: %d",
			mpxy_perf->id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"get domain #%u perf level failed with RPMI error: %d",
			mpxy_perf->id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	*level = le32_to_cpu(rx.perf_level);

	return rx.status;
}

static int sbi_mpxy_dvfs_device_opps_add(const struct sbi_mpxy_perf_info *pinfo,
					 struct device *dev, int domain)
{
	int idx, ret;
	unsigned long freq;
	struct sbi_mpxy_perf_opp *opp;
	struct sbi_mpxy_perf *dom;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;

	for (opp = dom->opp, idx = 0; idx < dom->opp_count; idx++, opp++) {
		/* Frequency from RPMI is in kHz*/
		freq = opp->perf_level * 1000;

		ret = dev_pm_opp_add(dev, freq, 0);
		if (ret) {
			dev_warn(dev, "failed to add opp %luHz\n", freq);

			while (idx-- > 0) {
				/* Frequency from RPMI is in kHz*/
				freq = (--opp)->perf_level * 1000;
				dev_pm_opp_remove(dev, freq);
			}
			return ret;
		}
	}
	return 0;
}

static int
sbi_mpxy_dvfs_transition_latency_get(const struct sbi_mpxy_perf_info *pinfo,
				     struct cpufreq_policy *policy)
{
	struct sbi_mpxy_perf *dom;
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;
	int domain = priv->cpufreq_domain_id;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;
	/* uS to nS */
	return dom->opp[dom->opp_count - 1].trans_latency_us * 1000;
}

static bool sbi_mpxy_fast_switch_possible(const struct sbi_mpxy_perf_info *pinfo,
					  struct cpufreq_policy *policy)
{
	struct sbi_mpxy_perf *dom;
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;
	int domain = priv->cpufreq_domain_id;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;

	return dom->fc_info && dom->fc_info[SBI_MPXY_PERF_FC_LEVEL].set_addr;
}

static int sbi_mpxy_dvfs_freq_set(const struct sbi_mpxy_perf_info *pinfo,
				  u32 domain, unsigned long freq)
{
	struct sbi_mpxy_perf *dom;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;

	return sbi_mpxy_perf_level_set(dom, domain, freq);
}

static int sbi_mpxy_dvfs_freq_get(const struct sbi_mpxy_perf_info *pinfo,
				  u32 domain, unsigned long *freq)
{
	int ret;
	u32 level;
	struct sbi_mpxy_perf *dom;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;

	ret = sbi_mpxy_perf_level_get(dom, domain, &level);

	if (!ret)
		*freq = (unsigned long) level;

	return ret;
}

static int sbi_mpxy_dvfs_est_power_get(const struct sbi_mpxy_perf_info *pinfo,
				       u32 domain, unsigned long *freq,
				       unsigned long *power)
{
	struct sbi_mpxy_perf *dom;
	unsigned long opp_freq;
	int idx, ret = -EINVAL;
	struct sbi_mpxy_perf_opp *opp;

	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	dom = pinfo->sperf + domain;

	for (opp = dom->opp, idx = 0; idx < dom->opp_count; idx++, opp++) {
		opp_freq = opp->perf_level;
		if (opp_freq < *freq)
			continue;

		*freq = opp_freq;
		*power = opp->power_cost;
		ret = 0;
		break;
	}

	return ret;
}

static enum sbi_mpxy_power_scale
sbi_mpxy_power_scale_get(const struct sbi_mpxy_perf_info *pinfo)
{
	return pinfo->power_scale;
}

static int sbi_mpxy_perf_get_domain_levels(struct sbi_mpxy_perf *mpxy_perf)
{
	int ret;
	unsigned int num_opps = 0;
	unsigned long rxmsg_len;
	struct rpmi_perf_get_domain_levels_tx tx;
	struct rpmi_perf_get_domain_levels_rx *rx;

	rx = devm_kcalloc(mpxy_perf->dev, mpxy_ctx.max_msg_len,
			       sizeof(u32), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	do {
		tx.domain_id = cpu_to_le32(mpxy_perf->id);
		tx.level_index = cpu_to_le32(num_opps);

		ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
						     RPMI_PERF_SRV_GET_DOMAIN_LEVELS,
						     &tx,
						     sizeof(struct rpmi_perf_get_domain_levels_tx),
						     rx, &rxmsg_len);
		if (ret) {
			dev_err(mpxy_perf->dev,
				"get domain #%u opp levels failed with error: %d",
				mpxy_perf->id, ret);
			return ret;
		}
		if (rx->status) {
			dev_err(mpxy_perf->dev,
				"get domain #%u opp levels failed with RPMI error: %d",
				mpxy_perf->id, rx->status);
			return rpmi_to_linux_error(rx->status);
		}

		if (num_opps + rx->returned_items + rx->remaining_items > mpxy_perf->opp_count) {
			dev_err(mpxy_perf->dev,
				"number of opp levels can't exceed %u\n",
				mpxy_perf->opp_count);
			return -EINVAL;
		}

		/* initialize opp table with values returned from PuC */
		for (int i = 0; i < rx->returned_items; i++) {
			mpxy_perf->opp[num_opps + i].perf_level = le32_to_cpu(rx->perf_levels[i][0]);
			mpxy_perf->opp[num_opps + i].power_cost = le32_to_cpu(rx->perf_levels[i][1]);
			mpxy_perf->opp[num_opps + i].trans_latency_us = le32_to_cpu(rx->perf_levels[i][2]);
		}

		num_opps += rx->returned_items;

	} while (rx->remaining_items > 0);

	return rx->status;
}

static int sbi_mpxy_perf_fastchannel_init(struct sbi_mpxy_perf *mpxy_perf,
					  struct sbi_mpxy_fc_info *fc,
					  unsigned int cmd)
{
	int ret = 0;
	u32 flags;
	u64 phys_addr;
	u8 size;
	void __iomem *addr, *db_addr;
	unsigned long rxmsg_len;
	struct sbi_mpxy_fc_db_info *db = NULL;
	struct rpmi_perf_get_fc_addr_tx tx;
	struct rpmi_perf_get_fc_addr_rx rx;

	tx.domain_id = cpu_to_le32(mpxy_perf->id);
	tx.service_id = cmd;
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_GET_FAST_CHANNEL_ADDR,
					     &tx,
					     sizeof(struct rpmi_perf_get_fc_addr_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"get fastchannel addrs in domain: %d failed with error: %d\n",
			mpxy_perf->id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"get fastchannel addrs in domain: %d failed with RPMI error: %d\n",
			mpxy_perf->id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	flags = le32_to_cpu(rx.flags);

	phys_addr = le32_to_cpu(rx.chan_addr_low);
	phys_addr |= (u64)le32_to_cpu(rx.chan_addr_high) << 32;
	addr = devm_ioremap(mpxy_perf->dev, phys_addr, 8);
	if (!addr) {
		dev_err(mpxy_perf->dev,
			"failed to get fastchannel virtual addr in domain: %d\n",
			mpxy_perf->id);
		ret = -EADDRNOTAVAIL;
	}

	if ((cmd == RPMI_PERF_SRV_SET_PERF_LEVEL ||
	     cmd == RPMI_PERF_SRV_SET_PERF_LIMIT) &&
	     SUPPORTS_DOORBELL(flags)) {
		db = devm_kzalloc(mpxy_perf->dev,
				  sizeof(struct sbi_mpxy_fc_db_info),
				  GFP_KERNEL);
		if (!db)
			return -ENOMEM;

		size = 1 << DOORBELL_REG_WIDTH(flags);
		phys_addr = le32_to_cpu(rx.db_addr_low);
		phys_addr |= (u64)le32_to_cpu(rx.db_addr_high) << 32;
		db_addr = devm_ioremap(mpxy_perf->dev, phys_addr, size);
		if (!db_addr) {
			dev_err(mpxy_perf->dev,
				"failed to get doorbell virtual addr in domain: %d\n",
				mpxy_perf->id);
			ret = -EADDRNOTAVAIL;
		}

		db->addr = db_addr;
		db->width = size;
		db->set = le32_to_cpu(rx.db_id_low);
		db->set |= (u64)le32_to_cpu(rx.db_id_high) << 32;
		db->mask = le32_to_cpu(rx.db_preserved_lmask);
		db->mask |= (u64)le32_to_cpu(rx.db_preserved_hmask) << 32;
	}

	switch (cmd) {
		case RPMI_PERF_SRV_GET_PERF_LEVEL:
		case RPMI_PERF_SRV_GET_PERF_LIMIT:
			fc->get_addr = addr;
			break;
		case RPMI_PERF_SRV_SET_PERF_LEVEL:
		case RPMI_PERF_SRV_SET_PERF_LIMIT:
			fc->set_addr = addr;
			fc->set_db = db;
			break;
	}

	return ret;
}

static int sbi_mpxy_perf_domain_init_fc(struct sbi_mpxy_perf *mpxy_perf)
{
	struct sbi_mpxy_fc_info *fc;

	fc = devm_kcalloc(mpxy_perf->dev, SBI_MPXY_PERF_FC_MAX, sizeof(*fc), GFP_KERNEL);
	if (!fc)
		return -ENOMEM;

	sbi_mpxy_perf_fastchannel_init(mpxy_perf,
				       &fc[SBI_MPXY_PERF_FC_LEVEL],
				       RPMI_PERF_SRV_GET_PERF_LEVEL);

	if (mpxy_perf->set_perf)
		sbi_mpxy_perf_fastchannel_init(mpxy_perf,
					       &fc[SBI_MPXY_PERF_FC_LEVEL],
					       RPMI_PERF_SRV_SET_PERF_LEVEL);

	sbi_mpxy_perf_fastchannel_init(mpxy_perf,
				       &fc[SBI_MPXY_PERF_FC_LIMIT],
				       RPMI_PERF_SRV_GET_PERF_LIMIT);

	if (mpxy_perf->set_limits)
		sbi_mpxy_perf_fastchannel_init(mpxy_perf,
					       &fc[SBI_MPXY_PERF_FC_LIMIT],
					       RPMI_PERF_SRV_SET_PERF_LIMIT);

	mpxy_perf->fc_info = fc;

	return 0;
}

/* obtains the MPXY perf domain attributes */
static int sbi_mpxy_perf_get_attrs(u32 domain_id, struct sbi_mpxy_perf *mpxy_perf)
{
	int ret;
	unsigned long rxmsg_len;
	struct rpmi_perf_get_domain_attrs_tx tx;
	struct rpmi_perf_get_domain_attrs_rx rx;

	tx.domain_id = cpu_to_le32(domain_id);
	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_GET_DOMAIN_ATTRIBUTES,
					     &tx,
					     sizeof(struct rpmi_perf_get_domain_attrs_tx),
					     &rx, &rxmsg_len);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"get perf domain %u attributes failed with error: %d\n",
			domain_id, ret);
		return ret;
	}
	if (rx.status) {
		dev_err(mpxy_perf->dev,
			"get perf domain %u attributes failed with RPMI error: %d\n",
			domain_id, rx.status);
		return rpmi_to_linux_error(rx.status);
	}

	mpxy_perf->id = domain_id;
	mpxy_perf->set_limits = PERF_LIMIT_SETTING(rx.flags);
	mpxy_perf->set_perf = PERF_LEVEL_SETTING(rx.flags);
	mpxy_perf->perf_fastchannels = FAST_CHANNEL_SUPPORT(rx.flags);
	mpxy_perf->opp_count = TOTAL_NUM_PERF_LEVELS(rx.flags);
	mpxy_perf->rate_limit_us = rx.rate_limit_us;
	strscpy(mpxy_perf->name, rx.name, SBI_MPXY_PERF_NAME_LEN);

	if (mpxy_perf->opp_count <= 0) {
		dev_err(mpxy_perf->dev,
			"invalid opps count: %u\n",
			mpxy_perf->opp_count);
		return -EINVAL;
	}

	mpxy_perf->opp = devm_kcalloc(mpxy_perf->dev, mpxy_perf->opp_count,
			       sizeof(struct sbi_mpxy_perf_opp), GFP_KERNEL);
	if (!mpxy_perf->opp)
		return -ENOMEM;

	return ret;
}

static int sbi_mpxy_perf_get_num_domains(void)
{
	int ret;
	struct rpmi_perf_get_num_domain_rx rx;

	ret = sbi_mpxy_send_message_withresp(mpxy_ctx.channel_id,
					     RPMI_PERF_SRV_GET_NUM_DOMAINS,
					     NULL, 0, &rx, NULL);
	if (ret)
		return ret;
	if (rx.status)
		return rpmi_to_linux_error(rx.status);

	return rx.num_domains;
}

static int sbi_mpxy_perf_enumerate(struct sbi_mpxy_perf *mpxy_perf, u32 domain_id)
{
	int ret;

	ret = sbi_mpxy_perf_get_attrs(domain_id, mpxy_perf);
	if (ret) {
		dev_err(mpxy_perf->dev,
			"Failed to get attributes of perf domain: #%u\n",
			domain_id);
		return ret;
	}

	ret = sbi_mpxy_perf_get_domain_levels(mpxy_perf);
	if (ret)
		return ret;

	if (mpxy_perf->perf_fastchannels) {
		ret = sbi_mpxy_perf_domain_init_fc(mpxy_perf);
		if (ret)
			return ret;
	}

	return 0;
}

static int
sbi_mpxy_cpufreq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;
	u64 freq = policy->freq_table[index].frequency;

	return sbi_mpxy_dvfs_freq_set(pinfo, priv->cpufreq_domain_id, freq);
}

static unsigned int sbi_mpxy_cpufreq_fast_switch(struct cpufreq_policy *policy,
						 unsigned int target_freq)
{
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;

	if (!sbi_mpxy_dvfs_freq_set(pinfo, priv->cpufreq_domain_id,
				    target_freq))
		return target_freq;

	return 0;
}

static unsigned int sbi_mpxy_cpufreq_get_rate(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;
	unsigned long rate;
	int ret;

	ret = sbi_mpxy_dvfs_freq_get(pinfo, priv->cpufreq_domain_id, &rate);
	if (ret)
		return 0;
	return rate;
}

static int sbi_mpxy_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret, nr_opp, domain_id;
	unsigned int latency;
	struct device *cpu_dev;
	struct sbi_mpxy_cpufreq_data *priv;
	struct cpufreq_frequency_table *freq_table;
	struct of_phandle_args args;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", policy->cpu);
		return -ENODEV;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_perf_domain_get_sharing_cpumask(policy->cpu,
						 "performance-domains",
						 "#performance-domain-cells",
						 policy->cpus, &args);
	if (ret) {
		dev_err(cpu_dev, "%s: failed to performance domain info: %d\n",
			__func__, ret);
		return ret;
	}
	domain_id = args.args[0];
	of_node_put(args.np);

	ret = sbi_mpxy_dvfs_device_opps_add(pinfo, cpu_dev, domain_id);
	if (ret) {
		dev_warn(cpu_dev, "failed to add opps to the device\n");
		goto out_free_priv;
	}

	nr_opp = dev_pm_opp_get_opp_count(cpu_dev);
	if (nr_opp <= 0) {
		dev_dbg(cpu_dev, "OPP table is not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out_free_priv;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	priv->cpu_dev = cpu_dev;
	priv->nr_opp = nr_opp;
	priv->cpufreq_domain_id = domain_id;

	/* Allow DVFS request for any domain from any CPU */
	policy->dvfs_possible_from_any_cpu = true;
	policy->driver_data = priv;
	policy->freq_table = freq_table;

	latency = sbi_mpxy_dvfs_transition_latency_get(pinfo, policy);
	if (!latency)
		latency = CPUFREQ_ETERNAL;

	policy->cpuinfo.transition_latency = latency;
	policy->fast_switch_possible = sbi_mpxy_fast_switch_possible(pinfo, policy);

	return 0;

out_free_opp:
	dev_pm_opp_remove_all_dynamic(cpu_dev);

out_free_priv:
	kfree(priv);

	return ret;
}

static int __maybe_unused
sbi_mpxy_get_cpu_power(struct device *cpu_dev, unsigned long *power,
		       unsigned long *KHz)
{
	enum sbi_mpxy_power_scale power_scale = sbi_mpxy_power_scale_get(pinfo);
	unsigned long Hz;
	int ret, domain;
	struct cpufreq_policy *policy;
	struct sbi_mpxy_cpufreq_data *priv;

	policy = cpufreq_cpu_get_raw(cpu_dev->id);
	if (!policy)
		return -EINVAL;

	priv = policy->driver_data;
	domain = priv->cpufreq_domain_id;
	if (domain < 0)
		return domain;
	else if (domain > pinfo->num_domains)
		return -EINVAL;

	/* Get the power cost of the performance domain. */
	Hz = *KHz * 1000;
	ret = sbi_mpxy_dvfs_est_power_get(pinfo, domain, &Hz, power);
	if (ret)
		return ret;

	/* Convert the power to uW if it is mW (ignore bogoW) */
	if (power_scale == SBI_MPXY_POWER_MILLIWATTS)
		*power *= MICROWATT_PER_MILLIWATT;

	/* The EM framework specifies the frequency in KHz. */
	*KHz = Hz / 1000;

	return 0;
}

static void sbi_mpxy_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &policy->freq_table);
	dev_pm_opp_remove_all_dynamic(priv->cpu_dev);
	kfree(priv);
}

static void sbi_mpxy_cpufreq_register_em(struct cpufreq_policy *policy)
{
	struct em_data_callback em_cb = EM_DATA_CB(sbi_mpxy_get_cpu_power);
	enum sbi_mpxy_power_scale power_scale = sbi_mpxy_power_scale_get(pinfo);
	struct sbi_mpxy_cpufreq_data *priv = policy->driver_data;
	bool em_power_scale = false;

	/*
	 * This callback will be called for each policy, but we don't need to
	 * register with EM every time. Despite not being part of the same
	 * policy, some CPUs may still share their perf-domains, and a CPU from
	 * another policy may already have registered with EM on behalf of CPUs
	 * of this policy.
	 */
	if (!priv->nr_opp)
		return;

	if (power_scale == SBI_MPXY_POWER_MILLIWATTS ||
	    power_scale == SBI_MPXY_POWER_MICROWATTS)
		em_power_scale = true;

	em_dev_register_perf_domain(get_cpu_device(policy->cpu), priv->nr_opp,
				    &em_cb, policy->cpus, em_power_scale);
}

static struct cpufreq_driver sbi_mpxy_cpufreq_driver = {
	.name   = "sbi-mpxy-cpufreq",
	.flags  = CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		  CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		  CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.attr   = cpufreq_generic_attr,
	.target_index   = sbi_mpxy_cpufreq_set_target,
	.fast_switch    = sbi_mpxy_cpufreq_fast_switch,
	.get    = sbi_mpxy_cpufreq_get_rate,
	.init   = sbi_mpxy_cpufreq_init,
	.exit   = sbi_mpxy_cpufreq_exit,
	.register_em    = sbi_mpxy_cpufreq_register_em,
};

static int sbi_mpxy_cpufreq_probe(struct platform_device *pdev)
{
	u32 channel_id, i, attr_count, version;
	int ret, num_domains;
	u32 *attr_buf;
	struct of_phandle_args args;

	if (sbi_spec_version < sbi_mk_version(1, 0) ||
	    sbi_probe_extension(SBI_EXT_MPXY) <= 0) {
		dev_err(&pdev->dev, "sbi mpxy extension is not present\n");
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

	attr_count = ATTR_COUNT(SBI_MPXY_ATTR_MSG_SEND_TIMEOUT,
				SBI_MPXY_ATTR_MSG_PROT_ID);

	attr_buf = devm_kzalloc(&pdev->dev, sizeof(u32) * attr_count,
				GFP_KERNEL);
	if (!attr_buf)
		return -ENOMEM;

	channel_id = args.args[0];

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

	mpxy_ctx.channel_id = channel_id;
	mpxy_ctx.max_msg_len = attr_buf[2];

	ret = sbi_mpxy_read_attrs(channel_id, SBI_MPXY_ATTR_MSGPROTO_ATTR_START,
				  1, attr_buf);
	if (ret) {
		dev_err(&pdev->dev, "channel-%u: read attributes - %d\n",
			channel_id, ret);
		return ret;
	}

	if (attr_buf[0] != RPMI_SRVGRP_PERF) {
		dev_err(&pdev->dev,
		"channel-%u ServiceGroup match failed, expected %x, found %x\n",
		channel_id, RPMI_SRVGRP_PERF, attr_buf[0]);
		return -EINVAL;
	}

	num_domains = sbi_mpxy_perf_get_num_domains();
	if (num_domains == 0) {
		dev_err(&pdev->dev, "No perf domains found!\n");
		return -EINVAL;
	} else if (num_domains < 0) {
		dev_err(&pdev->dev,
			"invalid number of perf domains - err:%d\n",
			num_domains);
		return -EINVAL;
	}

	pinfo = devm_kzalloc(&pdev->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pinfo->sperf = devm_kcalloc(&pdev->dev, num_domains,
				    sizeof(struct sbi_mpxy_perf),
				    GFP_KERNEL);
	if (!pinfo->sperf)
		return -ENOMEM;

	pinfo->num_domains = num_domains;
	pinfo->power_scale = SBI_MPXY_POWER_MICROWATTS;
	pinfo->dev = &pdev->dev;

	for (i = 0; i < num_domains; i++) {
		struct sbi_mpxy_perf *mpxy_perf = &pinfo->sperf[i];
		mpxy_perf->dev = pinfo->dev;
		sbi_mpxy_perf_enumerate(mpxy_perf, i);
	}

	ret = cpufreq_register_driver(&sbi_mpxy_cpufreq_driver);
	if (ret)
		dev_err(&pdev->dev, "registering cpufreq failed, err: %d\n", ret);

	dev_set_drvdata(&pdev->dev, pinfo);

	return ret;
}

static void sbi_mpxy_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&sbi_mpxy_cpufreq_driver);
}

static const struct of_device_id sbi_mpxy_cpufreq_of_match[] = {
	{ .compatible = "riscv,rpmi-performance" },
	{ },
};

MODULE_DEVICE_TABLE(of, sbi_mpxy_cpufreq_of_match);

#define DRIVER_NAME    "sbi-mpxy-cpufreq"

static struct platform_driver sbi_mpxy_cpufreq_platdrv = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = sbi_mpxy_cpufreq_of_match,
	},
	.probe = sbi_mpxy_cpufreq_probe,
	.remove = sbi_mpxy_cpufreq_remove,
};

module_platform_driver(sbi_mpxy_cpufreq_platdrv);

MODULE_AUTHOR("Alex Soo <yuklin.soo@starfivetech.com>");
MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("CPUFreq Driver based on SBI MPXY extension");
MODULE_LICENSE("GPL v2");
