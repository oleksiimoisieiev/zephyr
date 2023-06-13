/*
 * Copyright 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/drivers/tee.h>
#include <zephyr/logging/log.h>

#include "optee_msg.h"
#include "optee_smc.h"
LOG_MODULE_REGISTER(optee);

#define DT_DRV_COMPAT linaro_optee_tz

// TODO move it to some private header

/*
 * TEE Implementation ID
 */
#define TEE_IMPL_ID_OPTEE 1

/*
 * OP-TEE specific capabilities
 */
#define TEE_OPTEE_CAP_TZ  (1 << 0)

struct optee_rpc_param {
	uint32_t a0;
	uint32_t a1;
	uint32_t a2;
	uint32_t a3;
	uint32_t a4;
	uint32_t a5;
	uint32_t a6;
	uint32_t a7;
};
typedef void (*smc_call_t)(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			   unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			   struct arm_smccc_res *res);

static struct optee_driver_data {
	smc_call_t smc_call;
} optee_data;

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

static int param_to_msg_param(const struct tee_param *param, unsigned int num_param,
			      struct optee_msg_param *msg_param)
{
	int i;
	const struct tee_param *tp = param;
	struct optee_msg_param *mtp = msg_param;

	if (!param || !msg_param) {
		return -EINVAL;
	}

	for (i = 0; i < num_param; i++, tp++, mtp++) {
		if (!tp || !mtp) {
			LOG_ERR("Wrong param on %d iteration", i);
			return -EINVAL;
		}

		switch (tp->attr) {
		case TEE_PARAM_ATTR_TYPE_NONE:
			mtp->attr = OPTEE_MSG_ATTR_TYPE_NONE;
			memset(&mtp->u, 0, sizeof(mtp->u));
			break;
		case TEE_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_PARAM_ATTR_TYPE_VALUE_INOUT:
			mtp->attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT + tp->attr -
				TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
			mtp->u.value.a = tp->a;
			mtp->u.value.b = tp->b;
			mtp->u.value.c = tp->c;
			break;
		case TEE_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_PARAM_ATTR_TYPE_MEMREF_INOUT:
			mtp->attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT + tp->attr -
				TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
			mtp->u.rmem.shm_ref = tp->c;
			mtp->u.rmem.size = tp->b;
			mtp->u.rmem.offs = tp->a;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int msg_param_to_param(struct tee_param *param, unsigned int num_param,
			      const struct optee_msg_param *msg_param)
{
	int i;
	struct tee_param *tp = param;
	const struct optee_msg_param *mtp = msg_param;

	if (!param || !msg_param) {
		return -EINVAL;
	}

	for (i = 0; i < num_param; i++, tp++, mtp++) {
		uint32_t attr = mtp->attr & OPTEE_MSG_ATTR_TYPE_MASK;

		if (!tp || !mtp) {
			LOG_ERR("Wrong param on %d iteration", i);
			return -EINVAL;
		}

		switch (attr) {
		case OPTEE_MSG_ATTR_TYPE_NONE:
			memset(tp, 0, sizeof(*tp));
			tp->attr = TEE_PARAM_ATTR_TYPE_NONE;
			break;
		case OPTEE_MSG_ATTR_TYPE_VALUE_INPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_VALUE_INOUT:
			tp->attr = TEE_PARAM_ATTR_TYPE_VALUE_INOUT + attr -
				OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
			tp->a = mtp->u.value.a;
			tp->b = mtp->u.value.b;
			tp->c = mtp->u.value.c;
			break;
		case OPTEE_MSG_ATTR_TYPE_RMEM_INPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_INOUT:
			tp->attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT + attr -
				OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
			tp->b = mtp->u.rmem.size;

			if (!mtp->u.rmem.shm_ref) {
				tp->a = 0;
				tp->c = 0;
				break;
			}

			tp->a = mtp->u.rmem.offs;
			tp->c = mtp->u.rmem.shm_ref;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static uint64_t regs_to_u64(uint32_t reg0, uint32_t reg1)
{
	return (uint64_t)(((uint64_t)reg0 << 32) | reg1);
}

static void u64_to_regs(uint64_t val, uint32_t *reg0, uint32_t *reg1)
{
	*reg0 = val >> 32;
	*reg1 = val;
}

static void handle_rpc_call(const struct device *dev, struct optee_rpc_param *param)
{
	struct tee_shm *shm = NULL;

	switch (OPTEE_SMC_RETURN_GET_RPC_FUNC(param->a0)) {
	case OPTEE_SMC_RPC_FUNC_ALLOC:
		if (!tee_add_shm(dev, NULL, param->a1,
				 TEE_SHM_ALLOC | TEE_SHM_REGISTER, &shm)) {
			u64_to_regs((uint64_t)shm->addr, &param->a1, &param->a2);
			u64_to_regs((uint64_t)shm, &param->a4, &param->a5);
		} else {
			param->a1 = 0;
			param->a2 = 0;
			param->a4 = 0;
			param->a5 = 0;
		}
		break;
	case OPTEE_SMC_RPC_FUNC_FREE:
		shm = (struct tee_shm *)regs_to_u64(param->a1, param->a2);
		tee_rm_shm(dev, shm);
		break;
	case OPTEE_SMC_RPC_FUNC_FOREIGN_INTR:
		/* TODO: We do not support IRQ handler for now */
		break;
	case OPTEE_SMC_RPC_FUNC_CMD:
		/* TODO: Tee supplicatnt function should be called here */
		break;
	default:
		break;
	}

	param->a0 = OPTEE_SMC_CALL_RETURN_FROM_RPC;
}

static int optee_call(const struct device *dev, struct optee_msg_arg *arg)
{
	struct optee_driver_data *data = (struct optee_driver_data *)dev->data;
	struct optee_rpc_param rpc_param = {
		.a0 = OPTEE_SMC_CALL_WITH_ARG
	};

	u64_to_regs((uint64_t)arg, &rpc_param.a0, &rpc_param.a1);
	while (true) {
		struct arm_smccc_res res;

		data->smc_call(rpc_param.a0, rpc_param.a1, rpc_param.a2, rpc_param.a3,
			       rpc_param.a4, rpc_param.a5, rpc_param.a6, rpc_param.a7, &res);

		if (OPTEE_SMC_RETURN_IS_RPC(res.a0)) {
			handle_rpc_call(dev, &rpc_param);
		} else {
			return res.a0 == OPTEE_SMC_RETURN_OK ? TEEC_SUCCESS :
				TEEC_ERROR_BAD_PARAMETERS;
		}
	}
}

static int optee_get_version(const struct device *dev, struct tee_version_info *info)
{
	if (!info) {
		return -EINVAL;
	}

	info->impl_id = TEE_IMPL_ID_OPTEE;
	info->impl_caps = TEE_OPTEE_CAP_TZ;
	info->gen_caps = TEE_GEN_CAP_GP;

	return 0;
}

static int optee_close_session(const struct device *dev, uint32_t session_id)
{
	return 0;
}

static int optee_open_session(const struct device *dev, struct tee_open_session_arg *arg,
			      unsigned int num_param, struct tee_param *param,
			      uint32_t *session_id)
{
	int rc, ret;
	struct tee_shm *shm;
	struct optee_msg_arg *marg;

	if (!arg || !session_id) {
		return -EINVAL;
	}

	/* OPTEE_MSG_NONCONTIG_PAGE_SIZE, TODO should I pass this size here?*/
	rc = tee_add_shm(dev, NULL, OPTEE_MSG_GET_ARG_SIZE(num_param),
			 TEE_SHM_ALLOC, &shm);
	if (rc) {
		LOG_ERR("Unable to get shared memory, rc = %d", rc);
		return rc;
	}

	marg = shm->addr;
	memset(marg, 0, OPTEE_MSG_GET_ARG_SIZE(num_param));

	marg->num_params = num_param + 2;
	marg->cmd = OPTEE_MSG_CMD_OPEN_SESSION;
	marg->params[0].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;
	marg->params[1].attr = OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;

	memcpy(&marg->params[0].u.value, arg->uuid, sizeof(arg->uuid));
	memcpy(&marg->params[1].u.value, arg->uuid, sizeof(arg->clnt_uuid));

	marg->params[1].u.value.c = arg->clnt_login;

	rc = param_to_msg_param(param, num_param, marg->params + 2);
	if (rc) {
		goto out;
	}

	arg->ret = optee_call(dev, marg);
	if (arg->ret) {
		arg->ret_origin = TEEC_ORIGIN_COMMS;
		goto out;
	}

	rc = msg_param_to_param(param, num_param, marg->params);
	if (rc) {
		arg->ret = TEEC_ERROR_COMMUNICATION;
		arg->ret_origin = TEEC_ORIGIN_COMMS;
		/*
		 * Ret is needed here only to print an error. Param conversion error
		 * should be returned from the function.
		 */
		ret = optee_close_session(dev, marg->session);
		if (ret) {
			LOG_ERR("Unable to close session: %d", ret);
		}
		goto out;
	}

	*session_id = marg->session;

	arg->ret = marg->ret;
	arg->ret_origin = marg->ret_origin;
out:
	ret = tee_rm_shm(dev, shm);
	if (ret) {
		LOG_ERR("Unable to free shared memory");
	}

	return (rc) ? rc : ret;
}

static int optee_cancel(const struct device *dev, uint32_t session_id, uint32_t cancel_id)
{
	return 0;
}

static int optee_invoke_func(const struct device *dev, struct tee_invoke_func_arg *arg,
			      unsigned int num_param, struct tee_param *param)
{
	struct optee_msg_arg marg;
	/* struct tee_context *ctx = dev->data; */
	int err;

	marg.cmd = OPTEE_MSG_CMD_INVOKE_COMMAND;
	marg.func = arg->func;
	marg.session = arg->session;
	marg.cancel_id = arg->cancel_id;

	err = optee_call(dev, &marg);
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

static int set_optee_method(void)
{
	const char *method;

	method = DT_PROP(DT_INST(0, DT_DRV_COMPAT), method);

	if (!strcmp("hvc", method)) {
		optee_data.smc_call = optee_smccc_hvc;
	} else if (!strcmp("smc", method)) {
		optee_data.smc_call = optee_smccc_smc;
	} else {
		LOG_ERR("Invalid smc_call method");
		return -EINVAL;
	}

	return 0;
}

static int optee_init(const struct device *dev)
{
	printk("=============== %s %d\n", __func__, __LINE__);

	if (set_optee_method()) {
		return -ENOTSUP;
	}

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

DEVICE_DT_INST_DEFINE(0, optee_init, NULL, &optee_data, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &optee_driver_api);
