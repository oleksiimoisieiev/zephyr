/*
 * Copyright 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/tee.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(test_tee);

#define DT_DRV_COMPAT test_tee

static int test_tee_get_version(const struct device *dev, struct tee_version_info *info)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_open_session(const struct device *dev, struct tee_open_session_arg *arg,
				 unsigned int num_param, struct tee_param *param,
				 uint32_t *session_id)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_close_session(const struct device *dev, uint32_t session_id)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_cancel(const struct device *dev, uint32_t session_id, uint32_t cancel_id)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_invoke_func(const struct device *dev, struct tee_invoke_func_arg *arg,
			      unsigned int num_param, struct tee_param *param)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_shm_register(const struct device *dev, void *addr, size_t size, uint32_t flags,
			       struct tee_shm **shm)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_shm_unregister(const struct device *dev, struct tee_shm *shm)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_shm_alloc(const struct device *dev, size_t size, uint32_t flags, void **addr)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_shm_free(const struct device *dev, void *addr)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_suppl_recv(const struct device *dev, uint32_t func, unsigned int num_params,
			       struct tee_param *param)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_suppl_send(const struct device *dev, unsigned int num_params,
			       struct tee_param *param)
{
	printk("%s %d\n", __func__, __LINE__);
	return 0;
}

static int test_tee_init(const struct device *dev)
{
	printk("%s %d\n", __func__, __LINE__);

	return 0;
}

static const struct tee_driver_api test_tee_driver_api = {
	.get_version = test_tee_get_version,
	.open_session = test_tee_open_session,
	.close_session = test_tee_close_session,
	.cancel = test_tee_cancel,
	.invoke_func = test_tee_invoke_func,
	.shm_register = test_tee_shm_register,
	.shm_unregister = test_tee_shm_unregister,
	.shm_alloc = test_tee_shm_alloc,
	.shm_free = test_tee_shm_free,
	.suppl_recv = test_tee_suppl_recv,
	.suppl_send = test_tee_suppl_send,
};

DEVICE_DT_INST_DEFINE(0, test_tee_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &test_tee_driver_api);
