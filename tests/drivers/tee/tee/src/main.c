/*
 * Copyright (c) 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_xen
 * @{
 * @defgroup t_xen_regions test_xen_regions_operations
 * @}
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>

#include <zephyr/drivers/tee.h>

ZTEST(tee_test_suite, test_basic_calls)
{
	int ret;
	uint32_t session_id;
	int addr;
	struct tee_shm *shm = NULL;
	const struct device *const dev = DEVICE_DT_GET_ONE(test_tee);
	zassert_not_null(dev, "Unable to get dev");

	ret = tee_get_version(dev, NULL);
	zassert_ok(ret, "tee_get_version failed with code %d", ret);

	ret = tee_open_session(dev, NULL, 0, NULL, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	ret = tee_close_session(dev, 0);
	zassert_ok(ret, "close_session failed with code %d", ret);

	ret = tee_cancel(dev, 0, 0);
	zassert_ok(ret, "tee_cancel failed with code %d", ret);

	ret = tee_invoke_func(dev, NULL, 0, NULL);
	zassert_ok(ret, "tee_invoke_func failed with code %d", ret);

	ret = tee_shm_register(dev, &addr, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_unregister(dev, shm);
	zassert_ok(ret, "tee_shm_unregister failed with code %d", ret);

	ret = tee_shm_alloc(dev, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_alloc failed with code %d", ret);

	ret = tee_shm_free(dev, shm);
	zassert_ok(ret, "tee_shm_free failed with code %d", ret);

	ret = tee_suppl_recv(dev, 0, 0, NULL);
	zassert_ok(ret, "tee_suppl_recv failed with code %d", ret);

	ret = tee_suppl_send(dev, 0, NULL);
	zassert_ok(ret, "tee_suppl_send failed with code %d", ret);
}

ZTEST(tee_test_suite, test_reg_unreg)
{
	int ret;
	int addr;
	struct tee_shm *shm = NULL;
	const struct device *const dev = DEVICE_DT_GET_ONE(test_tee);
	zassert_not_null(dev, "Unable to get dev");

	/* Fail pass */
	ret = tee_shm_register(dev, &addr, 1, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_register(dev, NULL, 1, 0, &shm);
	zassert_equal(ret, -ENOMEM, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_register(dev, &addr, 1, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_register(dev, &addr, 0, 0, &shm);
	zassert_equal(ret, -EINVAL, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_unregister(dev, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_unregister failed with code %d", ret);

	/* Happy pass */
	ret = tee_shm_register(dev, &addr, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_register failed with code %d", ret);

	ret = tee_shm_unregister(dev, shm);
	zassert_ok(ret, "tee_shm_unregister failed with code %d", ret);

	ret = tee_shm_alloc(dev, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_alloc failed with code %d", ret);

	ret = tee_shm_free(dev, shm);
	zassert_ok(ret, "tee_shm_free failed with code %d", ret);
}

ZTEST_SUITE(tee_test_suite, NULL, NULL, NULL, NULL, NULL);
