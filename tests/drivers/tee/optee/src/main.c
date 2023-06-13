/*
 * Copyright (c) 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup optee_driver
 * @{
 * @defgroup t_tee_driver test_optee_driver
 * @}
 */

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>

#include <zephyr/drivers/tee.h>
#include "../../../drivers/tee/optee/optee_msg.h"
#include "../../../drivers/tee/optee/optee_smc.h"

#define TEE_OPTEE_CAP_TZ (1 << 0)

typedef void (*smc_cb_t)(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			 unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			 struct arm_smccc_res *res);

static struct test_call {
	int num;
	smc_cb_t smc_cb;
	uint32_t a0;
	uint32_t a1;
	uint32_t a2;
	uint32_t a3;
	uint32_t a4;
	uint32_t a5;
	uint32_t a6;
	uint32_t a7;
} t_call;

void arm_smccc_smc(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		   unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		   struct arm_smccc_res *res)
{
	if (t_call.smc_cb) {
		t_call.smc_cb(a0, a1, a2, a3, a4, a5, a6, a7, res);
	}
}

ZTEST(tee_test_suite, test_get_version)
{
	int ret;
	struct tee_version_info info;
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
	zassert_not_null(dev, "Unable to get dev");

	ret = tee_get_version(dev, NULL);
	zassert_equal(ret, -EINVAL, "tee_get_version failed with code %d", ret);

	ret = tee_get_version(dev, &info);
	zassert_ok(ret, "tee_get_version failed with code %d", ret);
	zassert_equal(info.impl_id, 1, "Wrong impl_id");
	zassert_equal(info.impl_caps, TEE_OPTEE_CAP_TZ, "Wrong impl_caps");
	zassert_equal(info.gen_caps, TEE_GEN_CAP_GP, "Wrong gen_caps");

	ret = tee_get_version(dev, NULL);
	zassert_equal(ret, -EINVAL, "tee_get_version failed with code %d", ret);
}

void fast_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		   unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		   struct arm_smccc_res *res)
{
	t_call.a0 = a0;
	t_call.a1 = a1;
	t_call.a2 = a2;
	t_call.a3 = a3;
	t_call.a4 = a4;
	t_call.a5 = a5;
	t_call.a6 = a6;
	t_call.a7 = a7;

	res->a0 = OPTEE_SMC_RETURN_OK;
}


ZTEST(tee_test_suite, test_fast_calls)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	// Fail pass
	ret = tee_open_session(dev, NULL, 0, NULL, &session_id);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

	ret = tee_open_session(dev, NULL, 0, NULL, NULL);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

	// Happy pass
	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

void normal_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		 unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		 struct arm_smccc_res *res)
{
	t_call.a0 = a0;
	t_call.a1 = a1;
	t_call.a2 = a2;
	t_call.a3 = a3;
	t_call.a4 = a4;
	t_call.a5 = a5;
	t_call.a6 = a6;
	t_call.a7 = a7;

	switch (t_call.num) {
	case 0:
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_ALLOC;
		break;
	case 1:
		zassert_equal(a0, 0x32000003, "normal_call failed with ret %lx", a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FREE ;
		res->a1 = a1;
		res->a2 = a2;
		res->a3 = a3;
		res->a4 = a4;
		res->a5 = a5;
		break;
	case 2:
		zassert_equal(a0, 0x32000003, "normal_call failed with ret %lx", a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FOREIGN_INTR;
		break;
	case 3:
		zassert_equal(a0, 0x32000003, "normal_call failed with ret %lx", a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_CMD;
		break;
	default:
		zassert_equal(a0, 0x32000003, "normal_call failed with ret %lx", a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
	}

	t_call.num++;
}
ZTEST(tee_test_suite, test_normal_calls)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = normal_call;

	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
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
