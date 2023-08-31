/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/ztest.h>

#define PINCTRL_STATE_TEST 2

/* pin configuration for test device */
#define TEST_DEVICE DT_NODELABEL(test_device)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static const struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

ZTEST(p_rza2_suite, test_dt_extract)
{
	const struct pinctrl_state *scfg;

	zassert_equal(pcfg->state_cnt, 1U);

	scfg = &pcfg->states[0];
	zassert_equal(scfg->id, PINCTRL_STATE_DEFAULT);
}

/**
 * @brief Test that pinctrl_lookup_state() works as expected
 */
ZTEST(p_rza2_suite, test_lookup_state)
{
	int ret;
	const struct pinctrl_state *scfg;

	ret = pinctrl_lookup_state(pcfg, PINCTRL_STATE_DEFAULT, &scfg);
	zassert_equal(ret, 0);
	zassert_equal_ptr(scfg, &pcfg->states[0], NULL);

	/* There shouldn't be sleep state. It is ignored in power management is disabled */
	ret = pinctrl_lookup_state(pcfg, PINCTRL_STATE_TEST, &scfg);
	zassert_equal(ret, -ENOENT);
}

/**
 * @brief Test that pinctrl_apply_state() works as expected.
 */
ZTEST(p_rza2_suite, test_apply_state)
{
	zassert_ok(pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT));
}

ZTEST_SUITE(p_rza2_suite, NULL, NULL, NULL, NULL, NULL);
