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

}

struct tee_context{};

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

static int optee_call(const struct device *dev, struct optee_msg_arg *arg)
{
	return 0;
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

static int optee_open_session(const struct device *dev, struct tee_open_session_arg *arg,
			      unsigned int num_param, struct tee_param *param,
			      uint32_t *session_id)
{
	int rc;
	struct tee_shm *shm;
	struct optee_msg_arg *marg;
	/* struct tee_context *ctx = dev->data; */

	rc = tee_add_shm(dev, NULL, /* OPTEE_MSG_NONCONTIG_PAGE_SIZE, TODO should I pass this size here?*/
			 OPTEE_MSG_GET_ARG_SIZE(num_param), TEE_SHM_ALLOC, &shm);
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
	// todo amoi write to msg
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
		goto out;
	}

	arg->ret = marg->ret;
	arg->ret_origin = marg->ret_origin;
out:
	//todo handle return code
	tee_rm_shm(dev, shm);
	return rc;
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

static int optee_init(const struct device *dev)
{
	printk("====== %s %d\n", __func__, __LINE__);
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
