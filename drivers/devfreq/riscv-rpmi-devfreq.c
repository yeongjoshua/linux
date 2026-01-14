// SPDX-License-Identifier: GPL-2.0
/*
 * RISC-V RPMI Based Devfreq Driver
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 */

#define pr_fmt(fmt) "riscv-rpmi-devfreq: " fmt

#include <linux/device/bus.h>
#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/riscv-rpmi-performance.h>
#include <linux/string.h>

struct rpmi_perf_devfreq {
	struct list_head node;
	struct device *dev;
	struct devfreq *devfreq;
	struct devfreq_dev_profile profile;
	struct rpmi_perf *perf;
	int domain_id;
};

struct rpmi_perf_devfreq_ctx {
	struct rpmi_perf *perf;
	struct notifier_block nb;
	struct list_head devfreq_list;
	struct mutex lock;
};

static char rpmi_devfreq_governor[32] = DEVFREQ_GOV_USERSPACE;
module_param_string(devfreq_governor, rpmi_devfreq_governor,
		    sizeof(rpmi_devfreq_governor), 0644);
MODULE_PARM_DESC(devfreq_governor,
		 "Devfreq governor for RPMI performance domains (default: userspace)");

static struct rpmi_perf_devfreq *rpmi_perf_devfreq_from_dev(struct device *dev)
{
	struct devfreq *devfreq;

	if (!dev->of_node)
		return ERR_PTR(-ENODEV);

	devfreq = devfreq_get_devfreq_by_node(dev->of_node);
	if (IS_ERR(devfreq))
		return ERR_CAST(devfreq);

	return devfreq->data;
}

static int rpmi_perf_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct rpmi_perf_devfreq *df = rpmi_perf_devfreq_from_dev(dev);
	struct rpmi_perf_domain *domain;
	u32 level_index;
	u32 freq_khz;
	int ret;

	if (IS_ERR(df))
		return PTR_ERR(df);

	domain = rpmi_perf_get_domain(df->perf, df->domain_id);
	if (!domain)
		return -EINVAL;

	ret = rpmi_perf_read_level(domain, &level_index);
	if (ret)
		return ret;

	ret = rpmi_perf_level_to_frequency(domain, level_index, &freq_khz);
	if (ret)
		return ret;

	*freq = (unsigned long)freq_khz * 1000;

	return 0;
}

static int rpmi_perf_devfreq_target(struct device *dev, unsigned long *freq,
				    u32 flags)
{
	struct rpmi_perf_devfreq *df = rpmi_perf_devfreq_from_dev(dev);
	struct rpmi_perf_domain *domain;
	struct dev_pm_opp *opp;
	unsigned long req_khz;
	u32 level_index;
	int ret;

	(void)flags;

	if (IS_ERR(df))
		return PTR_ERR(df);

	domain = rpmi_perf_get_domain(df->perf, df->domain_id);
	if (!domain)
		return -EINVAL;

	opp = devfreq_recommended_opp(dev, freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	*freq = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	req_khz = DIV_ROUND_CLOSEST(*freq, 1000);

	ret = rpmi_perf_frequency_to_level(domain, (u32)req_khz, &level_index);
	if (ret)
		return ret;

	ret = rpmi_perf_set_level(domain, level_index);
	if (ret)
		return ret;

	*freq = req_khz * 1000;

	return 0;
}

static int rpmi_perf_devfreq_get_dev_status(struct device *dev,
					    struct devfreq_dev_status *stat)
{
	unsigned long freq;
	int ret;

	ret = rpmi_perf_devfreq_get_cur_freq(dev, &freq);
	if (ret)
		return ret;

	stat->current_frequency = freq;
	stat->total_time = 0;
	stat->busy_time = 0;

	return 0;
}

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

static int rpmi_perf_match_domain(struct rpmi_perf *perf,
				  struct device_node *np, int *domain_id)
{
	struct device *perf_dev = rpmi_perf_get_dev(perf);
	struct of_phandle_args args;
	int count, i;

	if (!np || !domain_id || !perf_dev)
		return -EINVAL;

	count = of_count_phandle_with_args(np, "performance-domains",
					   "#performance-domain-cells");
	if (count <= 0)
		return -ENOENT;

	for (i = 0; i < count; i++) {
		if (of_parse_phandle_with_args(np, "performance-domains",
					       "#performance-domain-cells",
					       i, &args))
			continue;

		if (args.np == perf_dev->of_node) {
			if (args.args_count >= 1)
				*domain_id = args.args[0];
			of_node_put(args.np);
			return 0;
		}

		of_node_put(args.np);
	}

	return -ENOENT;
}

static int rpmi_perf_register_devfreq(struct rpmi_perf_devfreq_ctx *ctx,
				      struct device *dev, int domain_id)
{
	struct rpmi_perf_devfreq *df;
	struct rpmi_perf_domain *domain;
	unsigned long init_freq;
	u32 level_index;
	u32 freq_khz;
	int ret;

	domain = rpmi_perf_get_domain(ctx->perf, domain_id);
	if (!domain)
		return -EINVAL;

	if (!domain->set_level)
		return -EOPNOTSUPP;

	mutex_lock(&ctx->lock);
	list_for_each_entry(df, &ctx->devfreq_list, node) {
		if (df->dev == dev) {
			mutex_unlock(&ctx->lock);
			return 0;
		}
	}
	mutex_unlock(&ctx->lock);

	ret = rpmi_perf_dvfs_device_opps_add(ctx->perf, dev, domain_id);
	if (ret)
		return ret;

	ret = rpmi_perf_read_level(domain, &level_index);
	if (!ret)
		ret = rpmi_perf_level_to_frequency(domain, level_index, &freq_khz);

	if (ret)
		init_freq = (unsigned long)domain->opp[0].clock_freq * 1000;
	else
		init_freq = (unsigned long)freq_khz * 1000;

	df = kzalloc(sizeof(*df), GFP_KERNEL);
	if (!df) {
		rpmi_perf_dvfs_device_opps_remove(ctx->perf, dev, domain_id);
		return -ENOMEM;
	}

	df->dev = dev;
	df->perf = ctx->perf;
	df->domain_id = domain_id;
	df->profile = (struct devfreq_dev_profile) {
		.initial_freq = init_freq,
		.polling_ms = 0,
		.timer = DEVFREQ_TIMER_DELAYED,
		.target = rpmi_perf_devfreq_target,
		.get_dev_status = rpmi_perf_devfreq_get_dev_status,
		.get_cur_freq = rpmi_perf_devfreq_get_cur_freq,
	};

	df->devfreq = devfreq_add_device(dev, &df->profile,
					 rpmi_devfreq_governor, df);
	if (IS_ERR(df->devfreq)) {
		ret = PTR_ERR(df->devfreq);
		kfree(df);
		rpmi_perf_dvfs_device_opps_remove(ctx->perf, dev, domain_id);
		return ret;
	}

	ret = devfreq_register_opp_notifier(dev, df->devfreq);
	if (ret) {
		devfreq_remove_device(df->devfreq);
		kfree(df);
		rpmi_perf_dvfs_device_opps_remove(ctx->perf, dev, domain_id);
		return ret;
	}

	mutex_lock(&ctx->lock);
	list_add(&df->node, &ctx->devfreq_list);
	mutex_unlock(&ctx->lock);

	return 0;
}

static void rpmi_perf_unregister_devfreq(struct rpmi_perf_devfreq_ctx *ctx,
					 struct rpmi_perf_devfreq *df)
{
	devfreq_unregister_opp_notifier(df->dev, df->devfreq);
	devfreq_remove_device(df->devfreq);
	rpmi_perf_dvfs_device_opps_remove(ctx->perf, df->dev, df->domain_id);
	kfree(df);
}

static int rpmi_perf_try_register_devfreq_node(struct rpmi_perf_devfreq_ctx *ctx,
					       struct device_node *np)
{
	struct device *dev;
	int domain_id;
	int ret;

	if (!np || rpmi_perf_node_is_cpu(np))
		return 0;

	ret = rpmi_perf_match_domain(ctx->perf, np, &domain_id);
	if (ret)
		return 0;

	dev = bus_find_device_by_of_node(&platform_bus_type, np);
	if (!dev)
		return 0;

	ret = rpmi_perf_register_devfreq(ctx, dev, domain_id);
	put_device(dev);

	return ret;
}

static int rpmi_perf_devfreq_notifier(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	struct rpmi_perf_devfreq_ctx *ctx =
		container_of(nb, struct rpmi_perf_devfreq_ctx, nb);
	struct device *dev = data;

	if (action != BUS_NOTIFY_ADD_DEVICE || !dev->of_node)
		return NOTIFY_DONE;

	rpmi_perf_try_register_devfreq_node(ctx, dev->of_node);

	return NOTIFY_DONE;
}

static int rpmi_perf_devfreq_probe(struct platform_device *pdev)
{
	struct rpmi_perf_devfreq_ctx *ctx;
	struct device_node *np;
	struct rpmi_perf *perf = dev_get_drvdata(pdev->dev.parent);
	int ret;

	if (!perf)
		return -EPROBE_DEFER;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->perf = perf;
	mutex_init(&ctx->lock);
	INIT_LIST_HEAD(&ctx->devfreq_list);
	ctx->nb.notifier_call = rpmi_perf_devfreq_notifier;

	ret = bus_register_notifier(&platform_bus_type, &ctx->nb);
	if (ret)
		return ret;

	for_each_of_allnodes(np)
		if (of_find_property(np, "performance-domains", NULL))
			rpmi_perf_try_register_devfreq_node(ctx, np);

	platform_set_drvdata(pdev, ctx);

	return 0;
}

static void rpmi_perf_devfreq_remove(struct platform_device *pdev)
{
	struct rpmi_perf_devfreq_ctx *ctx = platform_get_drvdata(pdev);
	LIST_HEAD(tmp);
	struct rpmi_perf_devfreq *df, *next;

	bus_unregister_notifier(&platform_bus_type, &ctx->nb);

	mutex_lock(&ctx->lock);
	list_splice_init(&ctx->devfreq_list, &tmp);
	mutex_unlock(&ctx->lock);

	list_for_each_entry_safe(df, next, &tmp, node)
		rpmi_perf_unregister_devfreq(ctx, df);
}

static struct platform_driver rpmi_perf_devfreq_driver = {
	.driver = {
		.name = "riscv-rpmi-performance-devfreq",
	},
	.probe = rpmi_perf_devfreq_probe,
	.remove = rpmi_perf_devfreq_remove,
};

module_platform_driver(rpmi_perf_devfreq_driver);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Devfreq driver for RPMI performance domains");
MODULE_LICENSE("GPL");
