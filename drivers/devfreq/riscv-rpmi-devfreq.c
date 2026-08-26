// SPDX-License-Identifier: GPL-2.0
/*
 * RISC-V RPMI performance devfreq driver
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 *
 * A device that is not a CPU reaches an RPMI performance domain through
 * devfreq, the way a CPU reaches one through cpufreq. The device names its
 * domain with a "performance-domains" phandle, the levels the platform
 * microcontroller advertises become its operating points, and a governor
 * picks between them.
 */

#define pr_fmt(fmt) "riscv-rpmi-devfreq: " fmt

#include <linux/devfreq.h>
#include <linux/firmware/riscv/riscv-rpmi-performance.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/units.h>

struct rpmi_devfreq {
	struct device *dev;
	struct devfreq *devfreq;
	struct rpmi_perf_domain *domain;
};

static int rpmi_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct rpmi_devfreq *rd = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long rate;
	u32 level;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	rate = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	ret = rpmi_perf_domain_freq_to_level(rd->domain, rate / HZ_PER_KHZ, &level);
	if (ret)
		return ret;

	return rpmi_perf_domain_set_level(rd->domain, level);
}

static int rpmi_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct rpmi_devfreq *rd = dev_get_drvdata(dev);
	u32 khz, level;
	int ret;

	ret = rpmi_perf_domain_get_level(rd->domain, &level);
	if (ret)
		return ret;

	ret = rpmi_perf_domain_level_to_freq(rd->domain, level, &khz);
	if (ret)
		return ret;

	*freq = (unsigned long)khz * HZ_PER_KHZ;

	return 0;
}

static int rpmi_devfreq_get_dev_status(struct device *dev,
				       struct devfreq_dev_status *stat)
{
	/* There is no load counter behind an RPMI performance domain. */
	return 0;
}

static void rpmi_devfreq_remove_opps(void *dev)
{
	dev_pm_opp_remove_all_dynamic(dev);
}

static int rpmi_devfreq_probe(struct platform_device *pdev)
{
	struct devfreq_dev_profile *profile;
	struct device *dev = &pdev->dev;
	struct rpmi_devfreq *rd;
	unsigned long freq;
	int ret;

	rd = devm_kzalloc(dev, sizeof(*rd), GFP_KERNEL);
	if (!rd)
		return -ENOMEM;

	rd->dev = dev;

	rd->domain = devm_rpmi_perf_domain_get(dev, 0);
	if (IS_ERR(rd->domain))
		return dev_err_probe(dev, PTR_ERR(rd->domain),
				     "failed to get performance domain\n");

	if (!rpmi_perf_domain_can_set_level(rd->domain))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "performance domain %s cannot be set\n",
				     rpmi_perf_domain_name(rd->domain));

	platform_set_drvdata(pdev, rd);

	/*
	 * The levels come from the platform microcontroller, so there is no
	 * operating-points-v2 table in the device tree for devfreq to parse.
	 */
	ret = rpmi_perf_domain_opps_add(rd->domain, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add operating points\n");

	ret = devm_add_action_or_reset(dev, rpmi_devfreq_remove_opps, dev);
	if (ret)
		return ret;

	ret = rpmi_devfreq_get_cur_freq(dev, &freq);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the initial level\n");

	profile = devm_kzalloc(dev, sizeof(*profile), GFP_KERNEL);
	if (!profile)
		return -ENOMEM;

	*profile = (struct devfreq_dev_profile) {
		.initial_freq	= freq,
		.target		= rpmi_devfreq_target,
		.get_dev_status	= rpmi_devfreq_get_dev_status,
		.get_cur_freq	= rpmi_devfreq_get_cur_freq,
	};

	rd->devfreq = devm_devfreq_add_device(dev, profile,
					      DEVFREQ_GOV_USERSPACE, NULL);
	if (IS_ERR(rd->devfreq))
		return dev_err_probe(dev, PTR_ERR(rd->devfreq),
				     "failed to add devfreq device\n");

	dev_info(dev, "performance domain %s: %u level(s), running at %lu Hz\n",
		 rpmi_perf_domain_name(rd->domain),
		 rpmi_perf_domain_level_count(rd->domain), freq);

	return 0;
}

static const struct of_device_id rpmi_devfreq_of_match[] = {
	{ .compatible = "riscv,rpmi-performance-test" },
	{ },
};
MODULE_DEVICE_TABLE(of, rpmi_devfreq_of_match);

static struct platform_driver rpmi_devfreq_platdrv = {
	.driver = {
		.name = "riscv-rpmi-devfreq",
		.of_match_table = rpmi_devfreq_of_match,
	},
	.probe = rpmi_devfreq_probe,
};

module_platform_driver(rpmi_devfreq_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("devfreq driver for the RISC-V RPMI performance service");
MODULE_LICENSE("GPL");
