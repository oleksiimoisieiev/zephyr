/*
 * Copyright 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/drivers/tee.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(optee);

#define DT_DRV_COMPAT optee

/* Wrapping functions so function pointer can be used */
static void optee_smccc_smc(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			    unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			    struct arm_smccc_res *res)
{
	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

static void optee_smccc_hvc(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			    unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			    struct arm_smccc_res *res)
{
	arm_smccc_hvc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

//TODO add ifdefs and some callback to run from the unit tests
static void optee_smccc_test(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			     unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			     struct arm_smccc_res *res)
{
	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}

static int optee_call(struct tee_context *ctx; struct optee_msg_arg *arg)
{
	return 0;
}

static int optee_get_version(const struct device *dev, struct tee_version_info *info)
{
	if (!info) {
		return -EINVAL;
	}

	info.impl_id = TEE_IMPL_ID_OPTEE;
	info.impl_caps = TEE_OPTEE_CAP_TZ;
	info.gen_caps = TEE_GEN_CAP_GP;

	return 0;
}

static int optee_open_session(const struct device *dev, struct tee_open_session_arg *arg,
			      unsigned int num_param, struct tee_param *param,
			      uint32_t *session_id)
{
	struct optee_msg_arg marg;
	struct tee_context *ctx = dev->data;
	int err;

	marg.cmd = OPTEE_MSG_CMD_OPEN_SESSION;
	marg.cancel_id = arg->cancel_id;
	memcpy(&marg->params[0].u.value, arg->uuid, sizeof(arg->uuid));
	marg.params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;

	marg.params[1].u.value.c = arg->clnt_login;
	marg.params[1].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;

	err = optee_call(ctx, &marg);
	if (!err) {
		return err;
	}

	return 0;
}

static int optee_close_session(const struct device *dev, uint32_t session_id)
{
	return 0;
}

static int optee_cancel(const struct device *dev, uint32_t session_id, uint32_t cancel_id)
{
	return 0;
}

static int optee_invoke_func(const struct device *dev, struct tee_invoke_func_arg *arg,
			      unsigned int num_param, struct tee_param *param)
{
	struct optee_msg_arg marg;
	struct tee_context *ctx = dev->data;
	int err;

	marg->cmd = OPTEE_MSG_CMD_INVOKE_COMMAND;
	marg->func = arg->func;
	marg->session = arg->session;
	marg->cancel_id = arg->cancel_id;

	err = optee_call(ctx, &marg);
	if (!err) {
		return err;
	}
	return 0;
}

static int optee_shm_register(const struct device *dev, void *addr, size_t size, uint32_t flags,
			      struct tee_shm *shm)
{
	return 0;
}

static int optee_shm_unregister(const struct device *dev, struct tee_shm *shm)
{
	return 0;
}

static int optee_suppl_recv(const struct device *dev, uint32_t func, unsigned int num_params,
			    struct tee_param *param)
{
	return 0;
}

static int optee_suppl_send(const struct device *dev, unsigned int num_params,
			    struct tee_param *param)
{
	return 0;
}

static int optee_init(const struct device *dev)
{

	return 0;
}

static const struct tee_driver_api optee_driver_api = {
	.get_version = optee_get_version,
	.open_session = optee_open_session,
	.close_session = optee_close_session,
	.cancel = optee_cancel,
	.invoke_func = optee_invoke_func,
	.shm_register = optee_shm_register,
	.shm_unregister = optee_shm_unregister,
	.suppl_recv = optee_suppl_recv,
	.suppl_send = optee_suppl_send,
};

DEVICE_DT_INST_DEFINE(0, optee_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &optee_driver_api);
