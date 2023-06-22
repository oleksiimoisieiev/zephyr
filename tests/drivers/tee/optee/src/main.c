/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_optee_driver
 * @{
 * @defgroup t_optee_driver test_optee_driver
 * @}
 */

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>

#include <zephyr/drivers/tee.h>
#include <optee_msg.h>
#include <optee_smc.h>
#include <optee_rpc_cmd.h>

/*
 * TODO: Test shm_register/shm_register API to work with huge
 * buffers (more than 512K) to ensure that optee_construct_page_list
 * builds correct page list.
 */

#define TEE_OPTEE_CAP_TZ BIT(0)

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

ZTEST(optee_test_suite, test_get_version)
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
	zassert_equal(info.gen_caps, TEE_GEN_CAP_GP | TEE_GEN_CAP_REG_MEM,
		      "Wrong gen_caps");

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

void fail_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
	       unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
	       struct arm_smccc_res *res)
{
	res->a0 = OPTEE_SMC_RETURN_EBUSY;
}

ZTEST(optee_test_suite, test_fast_calls)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	/* Fail pass */
	ret = tee_open_session(dev, NULL, 0, NULL, &session_id);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

	ret = tee_open_session(dev, NULL, 0, NULL, NULL);
	zassert_equal(ret, -EINVAL, "tee_open_session failed with code %d", ret);

	/* Happy pass */
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

ZTEST(optee_test_suite, test_invoke_fn)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_invoke_func_arg invoke_arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	/* Fail pass */
	ret = tee_invoke_func(dev, NULL, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_invoke_fn failed with code %d", ret);

	t_call.smc_cb = fail_call;

	invoke_arg.func = 12;
	invoke_arg.session = 1;
	ret = tee_invoke_func(dev, &invoke_arg, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_invoke_fn failed with code %d", ret);

	t_call.smc_cb = fast_call;

	/* Happy pass */
	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	invoke_arg.func = 12;
	invoke_arg.session = 1;
	ret = tee_invoke_func(dev, &invoke_arg, 1, &param);
	zassert_ok(ret, "tee_invoke_fn failed with code %d", ret);

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

ZTEST(optee_test_suite, test_cancel)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	ret = tee_cancel(dev, 1, 25);
	zassert_ok(ret, "tee_cancel failed with code %d", ret);

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
		res->a1 = a1;
		res->a2 = a2;
		res->a3 = a3;
		res->a4 = a4;
		res->a5 = a5;
		break;
	case 1:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FREE;
		res->a1 = a1;
		res->a2 = a2;
		res->a3 = a3;
		res->a4 = a4;
		res->a5 = a5;
		break;
	case 2:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FOREIGN_INTR;
		break;
	default:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
	}

	t_call.num++;
}

ZTEST(optee_test_suite, test_normal_calls)
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

	t_call.num = 0;

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

ZTEST(optee_test_suite, test_reg_unreg)
{
	int ret;
	int addr;
	struct tee_shm *shm = NULL;
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	t_call.num = 0;
	t_call.smc_cb = normal_call;
	zassert_not_null(dev, "Unable to get dev");

	/* Fail pass */
	ret = tee_shm_register(dev, &addr, 1, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_register failed with code %d", ret);
	t_call.num = 0;

	ret = tee_shm_register(dev, NULL, 1, 0, &shm);
	zassert_equal(ret, -ENOMEM, "tee_shm_register failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_register(dev, &addr, 1, 0, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_register failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_register(dev, &addr, 0, 0, &shm);
	zassert_equal(ret, 0, "tee_shm_register failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_unregister(dev, NULL);
	zassert_equal(ret, -EINVAL, "tee_shm_unregister failed with code %d", ret);

	/* Happy pass */
	t_call.num = 0;
	ret = tee_shm_register(dev, &addr, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_register failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_unregister(dev, shm);
	zassert_ok(ret, "tee_shm_unregister failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_alloc(dev, 1, 0, &shm);
	zassert_ok(ret, "tee_shm_alloc failed with code %d", ret);

	t_call.num = 0;
	ret = tee_shm_free(dev, shm);
	zassert_ok(ret, "tee_shm_free failed with code %d", ret);
}

static uint64_t regs_to_u64(uint32_t reg0, uint32_t reg1)
{
	return (uint64_t)(((uint64_t)reg0 << 32) | reg1);
}

static void u64_to_regs(uint64_t val, unsigned long *reg0, unsigned long *reg1)
{
	*reg0 = val >> 32;
	*reg1 = val;
}

uint64_t g_shm_ref;
uint64_t g_func_shm_ref;

void cmd_alloc_free_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			 unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			 struct arm_smccc_res *res)
{
	struct optee_msg_arg *arg;
	struct tee_shm *shm;

	t_call.a0 = a0;
	t_call.a1 = a1;
	t_call.a2 = a2;
	t_call.a3 = a3;
	t_call.a4 = a4;
	t_call.a5 = a5;
	t_call.a6 = a6;
	t_call.a7 = a7;

	res->a1 = a1;
	res->a2 = a2;
	res->a3 = a3;
	res->a4 = a4;
	res->a5 = a5;

	switch (t_call.num) {
	case 0:
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_ALLOC;
		res->a1 = 1;
		break;
	case 1:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_CMD;
		arg = (struct optee_msg_arg *)regs_to_u64(a1, a2);
		arg->cmd = OPTEE_RPC_CMD_SHM_ALLOC;
		arg->num_params = 1;
		arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
		arg->params[0].u.value.b = 4096;
		arg->params[0].u.value.a = OPTEE_RPC_SHM_TYPE_KERNEL;
		res->a1 = a4;
		res->a2 = a5;
		g_shm_ref = regs_to_u64(a4, a5);
		break;
	case 2:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_CMD;
		printk("a1 %lx, a2 %lx a4 %lx a5 %lx\n", a1, a2);
		shm = (struct tee_shm *)regs_to_u64(a1, a2);
		arg = (struct optee_msg_arg *)shm->addr;
		arg->cmd = OPTEE_RPC_CMD_SHM_FREE;
		arg->num_params = 1;
		arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
		arg->params[0].u.value.a = OPTEE_RPC_SHM_TYPE_KERNEL;
		arg->params[0].u.value.b = g_func_shm_ref;
		u64_to_regs(g_shm_ref, &res->a1, &res->a2);
		break;
	case 3:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		u64_to_regs(g_shm_ref, &res->a1, &res->a2);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FREE;
		break;
	case 4:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
		break;
	default:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
	}

	t_call.num++;
}

ZTEST(optee_test_suite, test_func_shm_alloc)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_invoke_func_arg invoke_arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	t_call.num = 0;
	t_call.smc_cb = cmd_alloc_free_call;

	invoke_arg.func = 12;
	invoke_arg.session = 1;
	ret = tee_invoke_func(dev, &invoke_arg, 1, &param);
	zassert_ok(ret, "tee_invoke_fn failed with code %d", ret);

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

void cmd_gettime_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		      unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		      struct arm_smccc_res *res)
{
	struct optee_msg_arg *arg;
	struct tee_shm *shm;

	res->a1 = a1;
	res->a2 = a2;
	res->a3 = a3;
	res->a4 = a4;
	res->a5 = a5;

	switch (t_call.num) {
	case 0:
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_ALLOC;
		res->a1 = 1;
		break;
	case 1:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_CMD;
		arg = (struct optee_msg_arg *)regs_to_u64(a1, a2);
		arg->cmd = OPTEE_RPC_CMD_GET_TIME;
		arg->num_params = 1;
		arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT;
		res->a1 = a4;
		res->a2 = a5;
		g_shm_ref = regs_to_u64(a4, a5);
		break;
	case 2:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FREE;
		shm = (struct tee_shm *)regs_to_u64(a1, a2);
		arg = (struct optee_msg_arg *)shm->addr;

		t_call.a6 = arg->params[0].u.value.a;
		t_call.a7 = arg->params[0].u.value.b;
		u64_to_regs(g_shm_ref, &res->a1, &res->a2);
		break;
	case 3:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
		break;
	default:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
	}

	t_call.num++;
}

#define TICKS 0xDEADBEEF
#define TEST_SEC 37359285
#define TEST_NSEC 590000000
extern void z_vrfy_sys_clock_tick_set(uint64_t tick);

ZTEST(optee_test_suite, test_gettime)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_invoke_func_arg invoke_arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	t_call.num = 0;
	t_call.smc_cb = cmd_gettime_call;

	z_vrfy_sys_clock_tick_set(TICKS);

	invoke_arg.func = 12;
	invoke_arg.session = 1;
	ret = tee_invoke_func(dev, &invoke_arg, 1, &param);
	zassert_ok(ret, "tee_invoke_fn failed with code %d", ret);

	zassert_equal(t_call.a6, TEST_SEC, "Unexpected secs");
	zassert_equal(t_call.a7, TEST_NSEC, "Unexpected nsecs");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

void cmd_suspend_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
		      unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
		      struct arm_smccc_res *res)
{
	struct optee_msg_arg *arg;
	struct tee_shm *shm;

	res->a1 = a1;
	res->a2 = a2;
	res->a3 = a3;
	res->a4 = a4;
	res->a5 = a5;

	switch (t_call.num) {
	case 0:
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_ALLOC;
		res->a1 = 1;
		break;
	case 1:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_CMD;
		arg = (struct optee_msg_arg *)regs_to_u64(a1, a2);
		arg->cmd = OPTEE_RPC_CMD_SUSPEND;
		arg->num_params = 1;
		arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
		arg->params[0].u.value.a = t_call.a0;
		res->a1 = a4;
		res->a2 = a5;
		g_shm_ref = regs_to_u64(a4, a5);
		break;
	case 2:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_RPC_PREFIX | OPTEE_SMC_RPC_FUNC_FREE;
		shm = (struct tee_shm *)regs_to_u64(a1, a2);
		arg = (struct optee_msg_arg *)shm->addr;

		t_call.a6 = arg->params[0].u.value.a;
		t_call.a7 = arg->params[0].u.value.b;
		u64_to_regs(g_shm_ref, &res->a1, &res->a2);
		break;
	case 3:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
		break;
	default:
		zassert_equal(a0, 0x32000003, "%s failed with ret %lx", __func__, a0);
		res->a0 = OPTEE_SMC_RETURN_OK;
	}

	t_call.num++;
}

ZTEST(optee_test_suite, test_suspend)
{
	int ret;
	uint32_t session_id;
	struct tee_open_session_arg arg = {};
	struct tee_invoke_func_arg invoke_arg = {};
	struct tee_param param = {};
	const struct device *const dev = DEVICE_DT_GET_ONE(linaro_optee_tz);

	zassert_not_null(dev, "Unable to get dev");

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	arg.uuid[0] = 111;
	arg.clnt_uuid[0] = 222;
	arg.clnt_login = TEEC_LOGIN_PUBLIC;
	param.attr = TEE_PARAM_ATTR_TYPE_NONE;
	param.a = 3333;
	ret = tee_open_session(dev, &arg, 1, &param, &session_id);
	zassert_ok(ret, "tee_open_session failed with code %d", ret);

	t_call.num = 0;
	t_call.a0 = 4000; /* Set timeout 4000 ms */
	t_call.smc_cb = cmd_suspend_call;

	invoke_arg.func = 12;
	invoke_arg.session = 1;
	ret = tee_invoke_func(dev, &invoke_arg, 1, &param);
	zassert_ok(ret, "tee_invoke_fn failed with code %d", ret);

	t_call.num = 0;
	t_call.smc_cb = fast_call;

	ret = tee_close_session(dev, session_id);
	zassert_ok(ret, "close_session failed with code %d", ret);
}

ZTEST_SUITE(optee_test_suite, NULL, NULL, NULL, NULL, NULL);
