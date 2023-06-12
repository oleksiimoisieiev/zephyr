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

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>

#include <zephyr/drivers/tee.h>
#include "../../../drivers/tee/optee/optee_msg.h"

#define TEE_OPTEE_CAP_TZ (1 << 0)

void arm_smccc_smc(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		   unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		   struct arm_smccc_res *res)
{
	printk("TEST a0 = %ld a1 = %ld\n", a0, a1);
}

ZTEST(tee_test_suite, test_basic_calls)
{
	int ret;
	uint32_t session_id;
	struct tee_version_info info;
	int addr;
	struct tee_shm *shm = NULL;
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
	zassert_not_null(dev, "Unable to get dev");

	ret = tee_get_version(dev, NULL);
	zassert_equal(ret, -EINVAL, "tee_get_version failed with code %d", ret);

	ret = tee_get_version(dev, &info);
	zassert_ok(ret, "tee_get_version failed with code %d", ret);
	zassert_equal(info.impl_id, 1, "Wrong impl_id");
	zassert_equal(info.impl_caps, TEE_OPTEE_CAP_TZ, "Wrong impl_caps");
	zassert_equal(info.gen_caps, TEE_GEN_CAP_GP, "Wrong gen_caps");

	ret = tee_open_session(dev, NULL, 0, NULL, &session_id);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

	ret = tee_open_session(dev, NULL, 0, NULL, NULL);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

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
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
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
