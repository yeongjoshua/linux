// SPDX-License-Identifier: GPL-2.0
/*
 * Consumer side self test for the RISC-V RPMI voltage service group
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 *
 * The RPMI voltage service group describes the levels of a domain in one of
 * two formats: a table of discrete levels, or a linear range given as a
 * (min, max, step) tuple, where each range counts as a single level.
 *
 * Both are enumerated through selectors but reach the regulator core by
 * different helpers, so this driver walks every supply its node names and
 * checks that reading a voltage, enumerating the selectable levels and setting
 * a voltage each behave the way the format of that domain requires.
 *
 * Nothing here describes a domain. The levels and the constraints come from
 * the platform microcontroller by way of the provider, so the device tree only
 * says which domains to exercise, either through an ordinary "<name>-supply"
 * naming each domain's node in the provider's "regulators" container -- the
 * same way a consumer names an SCMI voltage domain -- or by DOMAIN_ID, through
 * "voltage-domains" and "voltage-domain-names".
 *
 * The voltages this driver asks for are its own choice, made through the
 * consumer API. There is no device tree property saying what to set: the
 * regulator bindings have none for a consumer, and the one that came closest,
 * regulator-suspend-microvolt, is deprecated in favour of exactly this.
 */

#define pr_fmt(fmt) "riscv-rpmi-regulator-test: " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/riscv-rpmi-regulator.h>

#define RPMI_REG_TEST_SUPPLY_SUFFIX	"-supply"
#define RPMI_REG_TEST_NAME_LEN		32

/*
 * A linear range with a fine step can span hundreds of thousands of
 * selectors, so sample them instead of walking every one.
 */
#define RPMI_REG_TEST_MAX_SAMPLES	64

struct rpmi_reg_test_result {
	unsigned int checks;
	unsigned int failures;
};

static void rpmi_reg_test_check(struct device *dev,
				struct rpmi_reg_test_result *res, bool ok,
				const char *supply, const char *what)
{
	res->checks++;

	if (ok) {
		dev_info(dev, "  PASS %-6s %s\n", supply, what);
		return;
	}

	res->failures++;
	dev_err(dev, "  FAIL %-6s %s\n", supply, what);
}

/*
 * Every domain has to enumerate at least one level, and every level it lists
 * has to sit inside the constraints the provider derived for it.
 */
static void rpmi_reg_test_levels(struct device *dev,
				 struct rpmi_reg_test_result *res,
				 struct regulator *reg, const char *supply,
				 int count)
{
	int i, uV, step, listed = 0, sampled = 0;
	bool in_range = true;

	step = DIV_ROUND_UP(count, RPMI_REG_TEST_MAX_SAMPLES);

	for (i = 0; i < count; i += step) {
		/* The last selector matters most, so always include it. */
		if (i + step >= count)
			i = count - 1;

		sampled++;
		uV = regulator_list_voltage(reg, i);
		if (uV <= 0)
			continue;

		listed++;
		/*
		 * Ask the core rather than comparing against bounds read back
		 * here: this is the same test it applies to a consumer, so it
		 * also proves the constraints were built at all.
		 */
		if (regulator_is_supported_voltage(reg, uV, uV) != 1)
			in_range = false;
	}

	dev_info(dev, "       %s: %d selector(s), %d of %d sampled listed\n",
		 supply, count, listed, sampled);

	rpmi_reg_test_check(dev, res, listed == sampled, supply,
			    "enumerates every sampled level");
	rpmi_reg_test_check(dev, res, in_range, supply,
			    "every listed level is within the constraints");
}

/*
 * Ask for the highest level the domain enumerates and require it to be
 * honoured exactly. The top selector is the interesting one, because an
 * off-by-one in the selector count hides it.
 */
static void rpmi_reg_test_set(struct device *dev,
			      struct rpmi_reg_test_result *res,
			      struct regulator *reg, const char *supply,
			      int count)
{
	int ret, target, readback;

	target = regulator_list_voltage(reg, count - 1);
	if (target <= 0) {
		rpmi_reg_test_check(dev, res, false, supply,
				    "has a level available to set");
		return;
	}

	ret = regulator_set_voltage(reg, target, target);
	if (ret) {
		dev_err(dev, "       %s: set to %d uV failed: %d\n",
			supply, target, ret);
		rpmi_reg_test_check(dev, res, false, supply,
				    "accepts an exact level");
		return;
	}

	rpmi_reg_test_check(dev, res, true, supply, "accepts an exact level");

	readback = regulator_get_voltage(reg);
	dev_info(dev, "       %s: asked %d uV, read back %d uV\n",
		 supply, target, readback);

	rpmi_reg_test_check(dev, res, readback == target, supply,
			    "reads back the level that was set");
}

/*
 * Drop the domain to the lowest level it offers, the way a driver that knows
 * what its hardware needs would pick a voltage: through the consumer API,
 * with no device tree property telling it what to ask for.
 *
 * Running after the highest-level test makes this a second transition, and a
 * downward one, which that test on its own cannot show.
 */
static void rpmi_reg_test_lowest(struct device *dev,
				 struct rpmi_reg_test_result *res,
				 struct regulator *reg, const char *supply,
				 int count)
{
	int i, uV, step, target = INT_MAX, ret, readback;

	step = DIV_ROUND_UP(count, RPMI_REG_TEST_MAX_SAMPLES);

	for (i = 0; i < count; i += step) {
		if (i + step >= count)
			i = count - 1;

		uV = regulator_list_voltage(reg, i);
		if (uV > 0 && uV < target)
			target = uV;
	}

	if (target == INT_MAX) {
		rpmi_reg_test_check(dev, res, false, supply,
				    "has a lowest level to fall back to");
		return;
	}

	ret = regulator_set_voltage(reg, target, target);
	if (ret) {
		dev_err(dev, "       %s: set to lowest %d uV failed: %d\n",
			supply, target, ret);
		rpmi_reg_test_check(dev, res, false, supply,
				    "accepts the lowest level");
		return;
	}

	rpmi_reg_test_check(dev, res, true, supply, "accepts the lowest level");

	readback = regulator_get_voltage(reg);
	dev_info(dev, "       %s: lowest %d uV, read back %d uV\n",
		 supply, target, readback);

	rpmi_reg_test_check(dev, res, readback == target, supply,
			    "runs at the lowest level");
}

/*
 * Bring the rail up the way a driver that needs it would.
 *
 * A plain regulator_enable() would prove very little: every domain starts out
 * supplying, so the core finds it already enabled, takes a reference and
 * returns without ever calling the driver -- see the ret == 0 || ret == -EINVAL
 * guard around _regulator_do_enable(). Dropping the rail first is what makes
 * the enable that follows a real RPMI SET_CONFIG.
 *
 * An always-on domain cannot be dropped, and the core knows it: with
 * REGULATOR_CHANGE_STATUS clear the disable never reaches the driver and the
 * rail correctly stays up. Both shapes end enabled, so both are checked the
 * same way.
 */
static void rpmi_reg_test_enable(struct device *dev,
				 struct rpmi_reg_test_result *res,
				 struct regulator *reg, const char *supply)
{
	int ret;

	ret = regulator_enable(reg);
	rpmi_reg_test_check(dev, res, ret == 0, supply, "enable is accepted");
	if (ret)
		return;

	rpmi_reg_test_check(dev, res, regulator_is_enabled(reg) == 1, supply,
			    "reads as enabled once enabled");

	ret = regulator_disable(reg);
	rpmi_reg_test_check(dev, res, ret == 0, supply, "disable is accepted");
	if (ret)
		return;

	if (regulator_is_enabled(reg) == 1) {
		dev_info(dev, "       %s: always-on, stays up across a disable\n",
			 supply);
	} else {
		dev_info(dev, "       %s: switched off, bringing it back\n",
			 supply);
		ret = regulator_enable(reg);
		if (ret)
			dev_err(dev, "       %s: re-enable failed: %d\n",
				supply, ret);
	}

	rpmi_reg_test_check(dev, res, regulator_is_enabled(reg) == 1, supply,
			    "ends up enabled");
}

static int rpmi_reg_test_one(struct device *dev,
			     struct rpmi_reg_test_result *res,
			     const char *supply)
{
	struct regulator *reg;
	int uV, count;

	/*
	 * Optional, so that a supply that cannot be resolved is reported as
	 * such instead of being quietly replaced by the dummy regulator.
	 */
	reg = devm_regulator_get_optional(dev, supply);
	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(dev, "  SKIP %-6s not available: %pe\n", supply, reg);
		return 0;
	}

	dev_info(dev, "testing domain %s\n", supply);

	uV = regulator_get_voltage(reg);
	dev_info(dev, "       %s: initial %d uV\n", supply, uV);
	rpmi_reg_test_check(dev, res, uV > 0, supply,
			    "initial voltage is readable");
	rpmi_reg_test_check(dev, res,
			    uV > 0 && regulator_is_supported_voltage(reg, uV, uV) == 1,
			    supply, "initial voltage is within the constraints");

	count = regulator_count_voltages(reg);
	if (count <= 0) {
		rpmi_reg_test_check(dev, res, false, supply,
				    "enumerates its selectable levels");
		return 0;
	}

	rpmi_reg_test_levels(dev, res, reg, supply, count);

	rpmi_reg_test_check(dev, res, regulator_is_enabled(reg) >= 0, supply,
			    "supply state is readable");

	rpmi_reg_test_set(dev, res, reg, supply, count);

	rpmi_reg_test_lowest(dev, res, reg, supply, count);

	rpmi_reg_test_enable(dev, res, reg, supply);

	return 0;
}

static int rpmi_reg_test_probe(struct platform_device *pdev)
{
	struct rpmi_reg_test_result res = {};
	struct device *dev = &pdev->dev;
	unsigned int count = 0;
	struct property *pp;
	const char *name;
	int ret;

	/*
	 * A domain named by DOMAIN_ID has to be resolved before any check runs.
	 * Nothing orders this probe after the provider's, and a deferral half
	 * way through would run the checks twice.
	 */
	of_property_for_each_string(dev_of_node(dev), "voltage-domain-names",
				    pp, name) {
		ret = devm_rpmi_voltage_supply_alias(dev, name);
		if (ret)
			return dev_err_probe(dev, ret, "cannot resolve %s\n",
					     name);
	}

	dev_info(dev, "RPMI voltage consumer self test\n");

	/* From here on a domain named by DOMAIN_ID is an ordinary supply. */
	of_property_for_each_string(dev_of_node(dev), "voltage-domain-names",
				    pp, name) {
		ret = rpmi_reg_test_one(dev, &res, name);
		if (ret)
			return ret;
		count++;
	}

	/*
	 * Every "<name>-supply" names a domain to exercise, so the set of
	 * domains follows the device tree instead of being listed here.
	 */
	for_each_property_of_node(dev_of_node(dev), pp) {
		char supply[RPMI_REG_TEST_NAME_LEN];
		size_t len, tail;

		len = strlen(pp->name);
		tail = strlen(RPMI_REG_TEST_SUPPLY_SUFFIX);
		if (len <= tail || len - tail >= sizeof(supply) ||
		    strcmp(pp->name + len - tail, RPMI_REG_TEST_SUPPLY_SUFFIX))
			continue;

		memcpy(supply, pp->name, len - tail);
		supply[len - tail] = '\0';

		ret = rpmi_reg_test_one(dev, &res, supply);
		if (ret)
			return ret;
		count++;
	}

	if (!count)
		return dev_err_probe(dev, -ENOENT, "no supplies to test\n");

	dev_info(dev, "self test done: %u check(s), %u failure(s)\n",
		 res.checks, res.failures);

	return 0;
}

static const struct of_device_id rpmi_reg_test_of_match[] = {
	{ .compatible = "riscv,rpmi-voltage-test" },
	{ },
};
MODULE_DEVICE_TABLE(of, rpmi_reg_test_of_match);

static struct platform_driver rpmi_reg_test_platdrv = {
	.driver = {
		.name = "riscv-rpmi-regulator-test",
		.of_match_table = rpmi_reg_test_of_match,
	},
	.probe = rpmi_reg_test_probe,
};

module_platform_driver(rpmi_reg_test_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Consumer self test for the RISC-V RPMI voltage service");
MODULE_LICENSE("GPL");
