/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RISC-V RPMI Performance Service API
 */

#ifndef __LINUX_RISCV_RPMI_PERFORMANCE_H__
#define __LINUX_RISCV_RPMI_PERFORMANCE_H__

#include <linux/device.h>
#include <linux/types.h>

#define RPMI_PERF_DOMAIN_NAME_LEN		16

struct rpmi_fc_info;
struct rpmi_perf;

struct rpmi_perf_opp {
	u32 index;
	u32 clock_freq;
	u32 power_cost;
	u32 trans_latency_us;
};

struct rpmi_perf_domain {
	u32 id;
	bool set_limit;
	bool set_level;
	bool has_fastchannels;
	u32 opp_count;
	u32 rate_limit_us;
	char name[RPMI_PERF_DOMAIN_NAME_LEN];
	struct rpmi_perf_opp *opp;
	struct rpmi_fc_info *fc_info;
	struct rpmi_perf *perf;
};

struct rpmi_perf *rpmi_perf_get(struct device *dev);
struct device *rpmi_perf_get_dev(struct rpmi_perf *perf);
int rpmi_perf_domain_count(struct rpmi_perf *perf);
struct rpmi_perf_domain *rpmi_perf_get_domain(struct rpmi_perf *perf, int id);
int rpmi_perf_read_level(struct rpmi_perf_domain *domain, u32 *level_index);
int rpmi_perf_set_level(struct rpmi_perf_domain *domain, u32 level_index);
int rpmi_perf_level_to_frequency(struct rpmi_perf_domain *domain, u32 index,
				 u32 *freq_khz);
int rpmi_perf_frequency_to_level(struct rpmi_perf_domain *domain, u32 freq_khz,
				 u32 *index);
int rpmi_perf_dvfs_device_opps_add(const struct rpmi_perf *perf,
				   struct device *dev, int domain_id);
void rpmi_perf_dvfs_device_opps_remove(const struct rpmi_perf *perf,
				       struct device *dev, int domain_id);

#endif /* __LINUX_RISCV_RPMI_PERFORMANCE_H__ */
