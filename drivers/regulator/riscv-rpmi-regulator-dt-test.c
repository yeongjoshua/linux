// SPDX-License-Identifier: GPL-2.0
/*
 * Device tree voltage range self test for the RISC-V RPMI voltage service group
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 *
 * The companion test, riscv-rpmi-regulator-test.c, walks every level of a rail
 * to check the driver. This one models how real consumers share a rail whose
 * permitted range the device tree sets, the way consumers share an SCMI
 * voltage domain.
 *
 * What a board permits a rail to supply is not something RPMI can express, so
 * the device tree states it as a constraint on the rail's regulator node. What
 * each consumer needs is stated in the consumer's own node, the way the MMC
 * bindings' "voltage-ranges" states what a host's slot needs:
 *
 *   rpmi-voltage {
 *           regulators {
 *                   volt6 {
 *                           regulator-min-microvolt = <1800000>;
 *                           regulator-max-microvolt = <2500000>;
 *                   };
 *           };
 *   };
 *   rpmi-voltage-dt-test-b {
 *           compatible = "riscv,rpmi-voltage-dt-test";
 *           volt6-supply = <&volt6>;
 *           voltage-range-microvolt = <1800000 2700000>;
 *   };
 *
 * The driver reads the range and asks for it through the consumer API, with
 * regulator_set_voltage(). The core narrows the request to the rail's range,
 * refusing it if nothing is left, and then sets the rail to the lowest level
 * inside every request held, refusing it if that leaves nothing. The request
 * stays in force for as long as the consumer holds it, as a real device's
 * would.
 *
 * Each consumer predicts its outcome from the rail's range and the requests
 * held before it, asks, and checks the prediction, that the rail is inside the
 * range before and after, and that the rail sits at the lowest level every
 * held request allows. The checks hold whatever order the consumers probe in,
 * though which of them is refused does depend on it.
 */

#define pr_fmt(fmt) "riscv-rpmi-regulator-dt-test: " fmt

#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define RPMI_REG_DT_TEST_SUPPLY_SUFFIX	"-supply"

/* A listed level the model has to find inside a range, at most this many. */
#define RPMI_REG_DT_TEST_MAX_LEVELS	64

/*
 * The intersection of the requests every accepted consumer holds. All
 * consumers of the rail probe through this driver, so this models the
 * regulator core's view of it.
 */
static DEFINE_MUTEX(rpmi_reg_dt_test_lock);
static int rpmi_reg_dt_test_lo;
static int rpmi_reg_dt_test_hi = INT_MAX;

enum rpmi_reg_dt_test_outcome {
	RPMI_REG_DT_TEST_ACCEPTED,
	RPMI_REG_DT_TEST_OUTSIDE_RANGE,
	RPMI_REG_DT_TEST_CONFLICTS,
};

static const char * const rpmi_reg_dt_test_outcome_str[] = {
	[RPMI_REG_DT_TEST_ACCEPTED]		= "acceptance",
	[RPMI_REG_DT_TEST_OUTSIDE_RANGE]	= "refusal by the device tree range",
	[RPMI_REG_DT_TEST_CONFLICTS]		= "refusal by the requests held",
};

struct rpmi_reg_dt_test_result {
	unsigned int checks;
	unsigned int failures;
};

static void rpmi_reg_dt_test_check(struct device *dev,
				   struct rpmi_reg_dt_test_result *res,
				   bool ok, const char *supply,
				   const char *what)
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
 * The levels the rail can reach, lowest first. The core lists a level outside
 * the constraint as 0, so only levels inside the device tree range come back.
 */
static int rpmi_reg_dt_test_levels(struct regulator *reg, int *levels,
				   int *n_levels)
{
	int count, i, j, uV;

	count = regulator_count_voltages(reg);
	if (count <= 0)
		return count ? count : -EINVAL;

	*n_levels = 0;
	for (i = 0; i < count && *n_levels < RPMI_REG_DT_TEST_MAX_LEVELS; i++) {
		uV = regulator_list_voltage(reg, i);
		if (uV <= 0)
			continue;
		for (j = *n_levels; j > 0 && levels[j - 1] > uV; j--)
			levels[j] = levels[j - 1];
		levels[j] = uV;
		(*n_levels)++;
	}

	return *n_levels ? 0 : -EINVAL;
}

/* The lowest level inside [lo, hi], or 0 if there is none. */
static int rpmi_reg_dt_test_lowest_in(const int *levels, int n_levels,
				      int lo, int hi)
{
	int i;

	for (i = 0; i < n_levels; i++)
		if (levels[i] >= lo && levels[i] <= hi)
			return levels[i];

	return 0;
}

/*
 * What the core does with one regulator_set_voltage(min, max): narrow it to
 * the device tree range, intersect it with the requests held, and find a
 * level inside what is left. [*lo, *hi] is updated only on acceptance.
 */
static enum rpmi_reg_dt_test_outcome
rpmi_reg_dt_test_request(int min_uV, int max_uV, int range_min, int range_max,
			 const int *levels, int n_levels, int *lo, int *hi)
{
	int l, h;

	min_uV = max(min_uV, range_min);
	max_uV = min(max_uV, range_max);
	if (min_uV > max_uV)
		return RPMI_REG_DT_TEST_OUTSIDE_RANGE;

	l = max(*lo, min_uV);
	h = min(*hi, max_uV);
	if (l > h || !rpmi_reg_dt_test_lowest_in(levels, n_levels, l, h))
		return RPMI_REG_DT_TEST_CONFLICTS;

	*lo = l;
	*hi = h;

	return RPMI_REG_DT_TEST_ACCEPTED;
}

static int rpmi_reg_dt_test_one(struct device *dev,
				struct rpmi_reg_dt_test_result *res,
				const u32 need[2], struct device_node *np,
				const char *supply)
{
	int levels[RPMI_REG_DT_TEST_MAX_LEVELS], n_levels;
	int uV, range_min = 0, range_max = 0, lo, hi, ret;
	enum rpmi_reg_dt_test_outcome expect;
	struct regulator *reg;
	char what[96];

	/*
	 * Read the range back out of the device tree so the checks are against
	 * what the tree actually says, not against values repeated here that
	 * could drift from it.
	 */
	of_property_read_u32(np, "regulator-min-microvolt", &range_min);
	of_property_read_u32(np, "regulator-max-microvolt", &range_max);

	if (!range_min || range_min > range_max) {
		dev_err(dev, "  SKIP %-6s has no voltage range in the device tree\n",
			supply);
		return 0;
	}

	reg = devm_regulator_get_optional(dev, supply);
	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(dev, "  SKIP %-6s not available: %pe\n", supply, reg);
		return 0;
	}

	ret = rpmi_reg_dt_test_levels(reg, levels, &n_levels);
	if (ret) {
		dev_err(dev, "  SKIP %-6s lists no level inside its range: %d\n",
			supply, ret);
		return 0;
	}

	dev_info(dev, "       %s: node needs %u..%u uV\n", supply, need[0], need[1]);

	/*
	 * Predict, ask and read back under one lock, so that no other
	 * consumer's request can land in between.
	 */
	guard(mutex)(&rpmi_reg_dt_test_lock);

	uV = regulator_get_voltage(reg);
	lo = rpmi_reg_dt_test_lo;
	hi = rpmi_reg_dt_test_hi;
	if (hi == INT_MAX)
		dev_info(dev, "       %s: device tree allows %d..%d uV, rail at %d uV, no request held\n",
			 supply, range_min, range_max, uV);
	else
		dev_info(dev, "       %s: device tree allows %d..%d uV, rail at %d uV, held to %d..%d uV\n",
			 supply, range_min, range_max, uV, lo, hi);

	rpmi_reg_dt_test_check(dev, res, uV >= range_min && uV <= range_max,
			       supply, "is inside the device tree range before asking");

	expect = rpmi_reg_dt_test_request(need[0], need[1], range_min, range_max,
					  levels, n_levels, &lo, &hi);

	ret = regulator_set_voltage(reg, need[0], need[1]);
	uV = regulator_get_voltage(reg);
	dev_info(dev, "       %s: expected %s, got %d, rail at %d uV\n",
		 supply, rpmi_reg_dt_test_outcome_str[expect], ret, uV);

	snprintf(what, sizeof(what), "request meets the expected %s",
		 rpmi_reg_dt_test_outcome_str[expect]);
	rpmi_reg_dt_test_check(dev, res,
			       !ret == (expect == RPMI_REG_DT_TEST_ACCEPTED),
			       supply, what);

	if (!ret && expect == RPMI_REG_DT_TEST_ACCEPTED) {
		rpmi_reg_dt_test_lo = lo;
		rpmi_reg_dt_test_hi = hi;
	}

	rpmi_reg_dt_test_check(dev, res, uV >= range_min && uV <= range_max,
			       supply, "is inside the device tree range after asking");

	/*
	 * Whatever happened, the rail must be at the lowest level every request
	 * still held allows: a refusal leaves it where the held requests put it.
	 */
	rpmi_reg_dt_test_check(dev, res,
			       uV == rpmi_reg_dt_test_lowest_in(levels, n_levels,
								max(rpmi_reg_dt_test_lo, range_min),
								min(rpmi_reg_dt_test_hi, range_max)),
			       supply, "sits at the lowest level every held request allows");

	return 0;
}

static int rpmi_reg_dt_test_probe(struct platform_device *pdev)
{
	struct rpmi_reg_dt_test_result res = {};
	struct device *dev = &pdev->dev;
	struct property *pp;
	u32 need[2];
	int ret;

	/* What this consumer needs, from its own node. */
	ret = of_property_read_u32_array(dev_of_node(dev),
					 "voltage-range-microvolt", need, 2);
	if (ret)
		return dev_err_probe(dev, ret, "no voltage-range-microvolt\n");
	if (need[0] > need[1])
		return dev_err_probe(dev, -EINVAL,
				     "voltage-range-microvolt minimum above maximum\n");

	dev_info(dev, "RPMI voltage device tree range self test\n");

	/*
	 * Every "<name>-supply" names a rail to check, so the set of rails
	 * follows the device tree instead of being listed here.
	 */
	for_each_property_of_node(dev_of_node(dev), pp) {
		struct device_node *np __free(device_node) = NULL;
		const char *tail;
		char supply[32];
		size_t len;

		tail = strstr(pp->name, RPMI_REG_DT_TEST_SUPPLY_SUFFIX);
		if (!tail || tail[strlen(RPMI_REG_DT_TEST_SUPPLY_SUFFIX)])
			continue;

		len = tail - pp->name;
		if (!len || len >= sizeof(supply))
			continue;

		memcpy(supply, pp->name, len);
		supply[len] = '\0';

		np = of_parse_phandle(dev_of_node(dev), pp->name, 0);
		if (!np) {
			dev_err(dev, "  SKIP %-6s supply has no node\n", supply);
			continue;
		}

		ret = rpmi_reg_dt_test_one(dev, &res, need, np, supply);
		if (ret)
			return ret;
	}

	dev_info(dev, "self test done: %u check(s), %u failure(s)\n",
		 res.checks, res.failures);

	return 0;
}

static const struct of_device_id rpmi_reg_dt_test_of_match[] = {
	{ .compatible = "riscv,rpmi-voltage-dt-test" },
	{ },
};
MODULE_DEVICE_TABLE(of, rpmi_reg_dt_test_of_match);

static struct platform_driver rpmi_reg_dt_test_platdrv = {
	.driver = {
		.name = "riscv-rpmi-regulator-dt-test",
		.of_match_table = rpmi_reg_dt_test_of_match,
	},
	.probe = rpmi_reg_dt_test_probe,
};

module_platform_driver(rpmi_reg_dt_test_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Device tree voltage range self test for the RISC-V RPMI voltage service");
MODULE_LICENSE("GPL");
