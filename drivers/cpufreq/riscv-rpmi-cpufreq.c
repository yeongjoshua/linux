// SPDX-License-Identifier: GPL-2.0
/*
 * RISC-V RPMI Based CPUFreq Driver
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 */

#define pr_fmt(fmt) "riscv-rpmi-cpufreq: " fmt

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/riscv-rpmi-performance.h>

struct rpmi_perf_cpufreq_data {
	int cpufreq_domain_id;
	int nr_opp;
	struct device *cpu_dev;
	struct rpmi_perf *perf;
};

static int rpmi_perf_set_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	struct rpmi_perf_cpufreq_data *data = policy->driver_data;
	struct rpmi_perf_domain *domain;

	domain = rpmi_perf_get_domain(data->perf, data->cpufreq_domain_id);
	if (!domain)
		return -EINVAL;

	return rpmi_perf_set_level(domain, index);
}

static unsigned int rpmi_perf_fast_switch(struct cpufreq_policy *policy,
					  unsigned int target_freq)
{
	struct rpmi_perf_cpufreq_data *data = policy->driver_data;
	struct rpmi_perf_domain *domain;
	u32 level_index = 0;

	domain = rpmi_perf_get_domain(data->perf, data->cpufreq_domain_id);
	if (!domain)
		return 0;

	if (!rpmi_perf_frequency_to_level(domain, target_freq, &level_index))
		return 0;

	if (!rpmi_perf_set_target_index(policy, level_index))
		return target_freq;

	return 0;
}

static unsigned int rpmi_perf_get_rate(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);
	struct rpmi_perf_cpufreq_data *data = policy->driver_data;
	struct rpmi_perf_domain *domain;
	u32 cpufreq, level_index;
	unsigned long rate;
	int ret;

	domain = rpmi_perf_get_domain(data->perf, data->cpufreq_domain_id);
	if (!domain)
		return 0;

	ret = rpmi_perf_read_level(domain, &level_index);
	if (ret)
		return 0;

	ret = rpmi_perf_level_to_frequency(domain, level_index, &cpufreq);
	if (ret)
		return 0;

	rate = (unsigned long)cpufreq;

	return rate;
}

static int rpmi_perf_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_table;
	struct rpmi_perf_cpufreq_data *data;
	struct rpmi_perf_domain *domain;
	struct of_phandle_args args;
	struct rpmi_perf *perf;
	int ret, nr_opp, domain_id;
	struct device *cpu_dev;

	perf = cpufreq_get_driver_data();
	if (!perf)
		return -ENODEV;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", policy->cpu);
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_perf_domain_get_sharing_cpumask(policy->cpu,
						 "performance-domains",
						 "#performance-domain-cells",
						 policy->cpus, &args);
	if (ret) {
		dev_err(cpu_dev, "%s: failed to performance domain info: %d\n",
			__func__, ret);
		goto out_free_priv;
	}

	domain_id = args.args[0];
	of_node_put(args.np);

	ret = rpmi_perf_dvfs_device_opps_add(perf, cpu_dev, domain_id);
	if (ret) {
		dev_warn(cpu_dev, "failed to add opps to the device\n");
		goto out_free_priv;
	}

	nr_opp = dev_pm_opp_get_opp_count(cpu_dev);
	if (nr_opp <= 0) {
		dev_dbg(cpu_dev, "OPP table is not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out_free_opp;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	data->cpu_dev = cpu_dev;
	data->nr_opp = nr_opp;
	data->cpufreq_domain_id = domain_id;
	data->perf = perf;

	/* Allow DVFS request for any domain from any CPU */
	policy->dvfs_possible_from_any_cpu = true;
	policy->driver_data = data;
	policy->freq_table = freq_table;

	domain = rpmi_perf_get_domain(perf, domain_id);
	if (!domain) {
		ret = -EINVAL;
		goto out_free_table;
	}

	policy->cpuinfo.transition_latency = domain->rate_limit_us * 1000;
	policy->fast_switch_possible = true;

	return 0;

out_free_table:
	dev_pm_opp_free_cpufreq_table(data->cpu_dev, &policy->freq_table);

out_free_opp:
	rpmi_perf_dvfs_device_opps_remove(perf, cpu_dev, domain_id);

out_free_priv:
	kfree(data);

	return ret;
}

static void rpmi_perf_exit(struct cpufreq_policy *policy)
{
	struct rpmi_perf_cpufreq_data *data = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(data->cpu_dev, &policy->freq_table);
	rpmi_perf_dvfs_device_opps_remove(data->perf, data->cpu_dev,
					  data->cpufreq_domain_id);
	kfree(data);
}

static int __maybe_unused
rpmi_perf_get_cpu_power(struct device *cpu_dev, unsigned long *uW,
			unsigned long *kHz)
{
	struct rpmi_perf_cpufreq_data *data;
	struct rpmi_perf_domain *domain;
	struct cpufreq_policy *policy;
	struct rpmi_perf_opp *opp;
	int idx;

	policy = cpufreq_cpu_get_raw(cpu_dev->id);
	if (!policy)
		return 0;

	data = policy->driver_data;
	domain = rpmi_perf_get_domain(data->perf, data->cpufreq_domain_id);
	if (!domain)
		return 0;

	for (opp = domain->opp, idx = 0; idx < domain->opp_count; idx++, opp++) {
		if (opp->clock_freq < *kHz)
			continue;

		*uW = opp->power_cost;
		*kHz = opp->clock_freq;
		break;
	}

	return 0;
}

static void rpmi_perf_register_em(struct cpufreq_policy *policy)
{
	struct em_data_callback em_cb = EM_DATA_CB(rpmi_perf_get_cpu_power);
	struct rpmi_perf_cpufreq_data *data = policy->driver_data;

	em_dev_register_perf_domain(get_cpu_device(policy->cpu), data->nr_opp,
				    &em_cb, policy->cpus, true);
}

static struct cpufreq_driver rpmi_perf_cpufreq_driver = {
	.name = "riscv-rpmi-cpufreq",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		 CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = rpmi_perf_set_target_index,
	.fast_switch = rpmi_perf_fast_switch,
	.get = rpmi_perf_get_rate,
	.init = rpmi_perf_init,
	.exit = rpmi_perf_exit,
	.register_em = rpmi_perf_register_em,
};

static int rpmi_cpufreq_probe(struct platform_device *pdev)
{
	struct rpmi_perf *perf = dev_get_drvdata(pdev->dev.parent);
	int ret;

	if (!perf)
		return -EPROBE_DEFER;

	rpmi_perf_cpufreq_driver.driver_data = perf;

	ret = cpufreq_register_driver(&rpmi_perf_cpufreq_driver);
	if (ret)
		dev_err(&pdev->dev, "registering cpufreq failed, err: %d\n", ret);

	return ret;
}

static void rpmi_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&rpmi_perf_cpufreq_driver);
}

static struct platform_driver rpmi_cpufreq_platdrv = {
	.driver = {
		.name = "riscv-rpmi-performance-cpufreq",
	},
	.probe = rpmi_cpufreq_probe,
	.remove = rpmi_cpufreq_remove,
};

module_platform_driver(rpmi_cpufreq_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("CPUFreq driver for RPMI performance domains");
MODULE_LICENSE("GPL");
