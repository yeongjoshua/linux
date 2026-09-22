// SPDX-License-Identifier: GPL-2.0
/*
 * Device tree voltage request self test for the RISC-V RPMI voltage service
 *
 * Copyright (C) 2026 Shanghai StarFive Technology Co., Ltd.
 *
 * A consumer cannot state a voltage through the regulator bindings: they only
 * describe regulators, and a "<name>-supply" is a bare phandle. The one
 * standard way for a consumer's own node to ask for a voltage is an OPP table,
 * whose entries carry "opp-microvolt = <target min max>" and which the OPP
 * core applies to the supply with regulator_set_voltage_triplet():
 *
 *   rail: volt7 {
 *           regulator-min-microvolt = <1200000>;
 *           regulator-max-microvolt = <1800000>;
 *   };
 *   consumer {
 *           vdd-supply = <&rail>;
 *           operating-points-v2 = <&table>;
 *   };
 *   table: opp-table {
 *           compatible = "operating-points-v2";
 *           opp-1 {
 *                   opp-level = <1>;
 *                   opp-microvolt = <1800000 1500000 1800000>;
 *           };
 *   };
 *
 * Using an OPP table for one fixed voltage, with a level and no clock, is a
 * shape nothing in the kernel tree uses: OPP tables exist for devices that
 * scale frequency and voltage together. It is used here because it is the
 * only standard way to put a consumer's request in the device tree, and the
 * path it takes through the regulator core is the one every such device uses.
 *
 * Several consumers share one rail, whose node carries a range the board
 * permits, so each request meets the provider's range twice and the other
 * consumers once:
 *
 *  - When the table is parsed, the OPP core drops any entry whose min..max
 *    holds no level the regulator will allow (_opp_supported_by_regulators()),
 *    so an entry wholly outside the provider's range never reaches the rail.
 *  - When the OPP is applied, the regulator core narrows each request to the
 *    provider's range, refusing it if nothing is left, and then sets the rail
 *    to the lowest level inside every request held, refusing it if that leaves
 *    nothing. The triplet asks for target..max first and falls back to
 *    min..max, so what a consumer ends up holding depends on what the others
 *    already hold.
 *
 * A refused or dropped request never moves the rail.
 *
 * Each consumer predicts its outcome, and the reason for a refusal, from the
 * provider's range and the requests held before it, then applies its own OPP
 * and checks the prediction and that the rail sits inside the provider's range
 * and every request still held. The checks hold whatever order the consumers
 * probe in, though which of them is refused, and why, does depend on it.
 */

#define pr_fmt(fmt) "riscv-rpmi-regulator-opp-test: " fmt

#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/riscv-rpmi-regulator.h>

/*
 * The supply every consumer names, as "vdd-supply" or as the "voltage-domains"
 * entry "voltage-domain-names" calls "vdd".
 */
#define RPMI_REG_OPP_TEST_SUPPLY	"vdd"

/*
 * The intersection of the ranges every accepted consumer still holds. All
 * consumers of the rail probe through this driver, so it models the regulator
 * core's view of the rail.
 */
static DEFINE_MUTEX(rpmi_reg_opp_test_lock);
static int rpmi_reg_opp_test_lo;
static int rpmi_reg_opp_test_hi = INT_MAX;

enum rpmi_reg_opp_test_outcome {
	RPMI_REG_OPP_TEST_ACCEPTED,
	RPMI_REG_OPP_TEST_DROPPED,
	RPMI_REG_OPP_TEST_OUTSIDE_PROVIDER,
	RPMI_REG_OPP_TEST_CONFLICTS,
};

static const char * const rpmi_reg_opp_test_outcome_str[] = {
	[RPMI_REG_OPP_TEST_ACCEPTED]		= "acceptance",
	[RPMI_REG_OPP_TEST_DROPPED]		= "drop from the table",
	[RPMI_REG_OPP_TEST_OUTSIDE_PROVIDER]	= "refusal by the provider's range",
	[RPMI_REG_OPP_TEST_CONFLICTS]		= "refusal by the requests held",
};

struct rpmi_reg_opp_test_result {
	unsigned int checks;
	unsigned int failures;
};

static void rpmi_reg_opp_test_check(struct device *dev,
				    struct rpmi_reg_opp_test_result *res,
				    bool ok, const char *what)
{
	res->checks++;

	if (ok) {
		dev_info(dev, "  PASS %s\n", what);
		return;
	}

	res->failures++;
	dev_err(dev, "  FAIL %s\n", what);
}

/*
 * The range the provider permits, as the consumer API sees it: the core lists
 * a level outside the constraint as 0, so the lowest and highest levels it
 * does list bound the constraint. Whether any listed level falls inside
 * [in_min, in_max] is found on the way, which is what decides whether the OPP
 * core keeps an entry asking for that range.
 */
static int rpmi_reg_opp_test_provider_range(struct regulator *reg,
					    int in_min, int in_max,
					    int *min_uV, int *max_uV,
					    bool *has_level_in)
{
	int count, i, uV;

	count = regulator_count_voltages(reg);
	if (count <= 0)
		return count ? count : -EINVAL;

	*min_uV = INT_MAX;
	*max_uV = 0;
	*has_level_in = false;
	for (i = 0; i < count; i++) {
		uV = regulator_list_voltage(reg, i);
		if (uV <= 0)
			continue;
		*min_uV = min(*min_uV, uV);
		*max_uV = max(*max_uV, uV);
		if (uV >= in_min && uV <= in_max)
			*has_level_in = true;
	}

	return *min_uV <= *max_uV ? 0 : -EINVAL;
}

/*
 * What the core does with one regulator_set_voltage(min, max): narrow the
 * request to the provider's range, then intersect it with the requests held.
 * [*lo, *hi] is updated only when the request is accepted.
 */
static enum rpmi_reg_opp_test_outcome
rpmi_reg_opp_test_request(int min_uV, int max_uV, int prov_min, int prov_max,
			  int *lo, int *hi)
{
	int l, h;

	min_uV = max(min_uV, prov_min);
	max_uV = min(max_uV, prov_max);
	if (min_uV > max_uV)
		return RPMI_REG_OPP_TEST_OUTSIDE_PROVIDER;

	l = max(*lo, min_uV);
	h = min(*hi, max_uV);
	if (l > h)
		return RPMI_REG_OPP_TEST_CONFLICTS;

	*lo = l;
	*hi = h;

	return RPMI_REG_OPP_TEST_ACCEPTED;
}

static int rpmi_reg_opp_test_probe(struct platform_device *pdev)
{
	static const char * const supplies[] = { RPMI_REG_OPP_TEST_SUPPLY, NULL };
	enum rpmi_reg_opp_test_outcome expect;
	struct rpmi_reg_opp_test_result res = {};
	struct device *dev = &pdev->dev;
	struct device_node *table __free(device_node) = NULL;
	struct device_node *entry __free(device_node) = NULL;
	int prov_min, prov_max, lo, hi, uV, ret;
	u32 req[3]; /* target, min, max */
	unsigned int level = 0;
	struct dev_pm_opp *opp;
	struct regulator *reg;
	bool kept;
	char what[96];

	/*
	 * A rail named by DOMAIN_ID has to be resolved before anything looks
	 * for it, this driver or the OPP core, and before anything that cannot
	 * be repeated, since nothing orders this probe after the provider's.
	 */
	if (of_property_present(dev_of_node(dev), "voltage-domains")) {
		ret = devm_rpmi_voltage_supply_alias(dev, RPMI_REG_OPP_TEST_SUPPLY);
		if (ret)
			return dev_err_probe(dev, ret, "cannot resolve %s\n",
					     RPMI_REG_OPP_TEST_SUPPLY);
	}

	/*
	 * The OPP core holds its own handle to the supply, and that handle is
	 * the one carrying the request. This one only reads the rail back, and
	 * a handle that never asks for a voltage takes no part in arbitration.
	 */
	reg = devm_regulator_get_optional(dev, RPMI_REG_OPP_TEST_SUPPLY);
	if (IS_ERR(reg))
		return dev_err_probe(dev, PTR_ERR(reg), "no %s supply\n",
				     RPMI_REG_OPP_TEST_SUPPLY);

	/*
	 * Read the request out of the device tree rather than out of the OPP
	 * core, which may have dropped the entry by the time it is asked.
	 */
	table = of_parse_phandle(dev_of_node(dev), "operating-points-v2", 0);
	if (table)
		entry = of_get_next_available_child(table, NULL);
	if (!entry || of_property_read_u32_array(entry, "opp-microvolt", req,
						 ARRAY_SIZE(req)))
		return dev_err_probe(dev, -EINVAL,
				     "no <target min max> opp-microvolt entry\n");

	ret = rpmi_reg_opp_test_provider_range(reg, req[1], req[2],
					       &prov_min, &prov_max, &kept);
	if (ret)
		return dev_err_probe(dev, ret, "supply lists no usable level\n");

	dev_info(dev, "RPMI voltage OPP request self test\n");
	dev_info(dev, "       requests %u uV, accepting %u..%u uV\n",
		 req[0], req[1], req[2]);
	dev_info(dev, "       provider permits %d..%d uV\n", prov_min, prov_max);

	/* The regulators have to be known before the table is parsed. */
	ret = devm_pm_opp_set_regulators(dev, supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set OPP regulators\n");

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add OPP table\n");

	opp = dev_pm_opp_find_level_ceil(dev, &level);
	if (IS_ERR(opp) && PTR_ERR(opp) != -ERANGE)
		return dev_err_probe(dev, PTR_ERR(opp), "OPP lookup failed\n");

	/*
	 * An entry with no allowed level between its min and max is dropped
	 * while the table is parsed, so there is nothing to apply. An entry
	 * that has one must still be there.
	 */
	if (kept && IS_ERR(opp))
		return dev_err_probe(dev, PTR_ERR(opp),
				     "OPP table is empty, though its entry has an allowed level\n");

	/*
	 * Predict and apply under one lock, so that no other consumer's request
	 * can land in between, and read the rail back under it too.
	 */
	mutex_lock(&rpmi_reg_opp_test_lock);

	if (!kept) {
		dev_info(dev, "       expected drop from the table, table has %d OPP(s)\n",
			 dev_pm_opp_get_opp_count(dev));
		rpmi_reg_opp_test_check(dev, &res, IS_ERR(opp),
					"request meets the expected drop from the table");
		if (!IS_ERR(opp))
			dev_pm_opp_put(opp);
		goto check_rail;
	}

	lo = rpmi_reg_opp_test_lo;
	hi = rpmi_reg_opp_test_hi;
	if (hi == INT_MAX)
		dev_info(dev, "       no earlier consumer holds a request\n");
	else
		dev_info(dev, "       earlier consumers hold the rail to %d..%d uV\n",
			 lo, hi);

	/* As regulator_set_voltage_triplet(): target..max, then min..max. */
	expect = rpmi_reg_opp_test_request(req[0], req[2],
					   prov_min, prov_max, &lo, &hi);
	if (expect != RPMI_REG_OPP_TEST_ACCEPTED)
		expect = rpmi_reg_opp_test_request(req[1], req[2],
						   prov_min, prov_max, &lo, &hi);

	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);

	dev_info(dev, "       expected %s, dev_pm_opp_set_opp() returned %d\n",
		 rpmi_reg_opp_test_outcome_str[expect], ret);

	snprintf(what, sizeof(what), "request meets the expected %s",
		 rpmi_reg_opp_test_outcome_str[expect]);
	rpmi_reg_opp_test_check(dev, &res,
				!ret == (expect == RPMI_REG_OPP_TEST_ACCEPTED),
				what);

	if (!ret && expect == RPMI_REG_OPP_TEST_ACCEPTED) {
		rpmi_reg_opp_test_lo = lo;
		rpmi_reg_opp_test_hi = hi;
	}

check_rail:
	uV = regulator_get_voltage(reg);
	dev_info(dev, "       rail at %d uV, every held request allows %d..%d uV\n",
		 uV, rpmi_reg_opp_test_lo, rpmi_reg_opp_test_hi);

	rpmi_reg_opp_test_check(dev, &res, uV >= prov_min && uV <= prov_max,
				"rail is inside the provider's range");
	rpmi_reg_opp_test_check(dev, &res,
				uV >= rpmi_reg_opp_test_lo &&
				uV <= rpmi_reg_opp_test_hi,
				"rail is inside every request still held");

	mutex_unlock(&rpmi_reg_opp_test_lock);

	dev_info(dev, "self test done: %u check(s), %u failure(s)\n",
		 res.checks, res.failures);

	return 0;
}

static const struct of_device_id rpmi_reg_opp_test_of_match[] = {
	{ .compatible = "riscv,rpmi-voltage-opp-test" },
	{ },
};
MODULE_DEVICE_TABLE(of, rpmi_reg_opp_test_of_match);

static struct platform_driver rpmi_reg_opp_test_platdrv = {
	.driver = {
		.name = "riscv-rpmi-regulator-opp-test",
		.of_match_table = rpmi_reg_opp_test_of_match,
	},
	.probe = rpmi_reg_opp_test_probe,
};

module_platform_driver(rpmi_reg_opp_test_platdrv);

MODULE_AUTHOR("Joshua Yeong <joshua.yeong@starfivetech.com>");
MODULE_DESCRIPTION("Device tree voltage request self test for the RISC-V RPMI voltage service");
MODULE_LICENSE("GPL");
