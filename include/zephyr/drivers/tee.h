/**
 * @file drivers/tee.h
 *
 * @brief Public APIs for the tee driver.
 */

/*
 * Copyright (c) 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_TEE_H_
#define ZEPHYR_INCLUDE_DRIVERS_TEE_H_

/**
 * @brief Trusted Execution Environment Interface
 * @defgroup tee_interface TEE Interface
 * @ingroup security
 * @{
 *
 * The generic interface to work with Trusted Execution Environment (TEE).
 * TEE is Trusted OS, running in the Secure Space, such as TrustZone in ARM cpus.
 * It also can be presented as the separate secure co-processors. It allows system
 * to implement logic, separated from the Normal World.
 *
 * Using TEE syscalls:
 * - tee_get_version() to get current TEE capabilities
 * - tee_open_session() to open new session to the TA
 * - tee_close_session() to close session to the TA
 * - tee_cancel() to cancel session or invoke function
 * - tee_invoke_func() to invoke function to the TA
 * - tee_shm_register() to register shared memory region
 * - tee_shm_unregister() to unregister shared memory region
 * - tee_shm_alloc() to allocate shared memory region
 * - tee_shm_free() to free shared memory region
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif
#define TEE_UUID_LEN 16

#define TEE_SHM_REGISTER BIT(0)
#define TEE_SHM_ALLOC BIT(1)

/**
 * @brief TEE version
 * @impl_id:	[out] TEE implementation id
 * @impl_caps:	[out] Implementation specific capabilities
 * @gen_caps:	[out] Generic capabilities, defined by TEE_GEN_CAPS_* above
 *
 * Identifies the TEE implementation, @impl_id is one of TEE_IMPL_ID_* above.
 * @impl_caps is implementation specific, for example TEE_OPTEE_CAP_*
 * is valid when @impl_id == TEE_IMPL_ID_OPTEE.
 */
struct tee_version_info {
	uint32_t impl_id;
	uint32_t impl_caps;
	uint32_t gen_caps;
};

/**
 * struct tee_open_session_arg - Open session argument
 * @uuid:	[in] UUID of the Trusted Application
 * @clnt_uuid:	[in] UUID of client
 * @clnt_login:	[in] Login class of client, TEE_IOCTL_LOGIN_* above
 * @cancel_id:	[in] Cancellation id, a unique value to identify this request
 * @session:	[out] Session id
 * @ret:	[out] return value
 * @ret_origin	[out] origin of the return value
 */
struct tee_open_session_arg {
	uint8_t uuid[TEE_UUID_LEN];
	uint8_t clnt_uuid[TEE_UUID_LEN];
	uint32_t clnt_login;
	uint32_t cancel_id;
	uint32_t session;
	uint32_t ret;
        uint32_t ret_origin;
};

/**
 * struct tee_param - parameter
 * @attr: attributes
 * @a: if a memref, offset into the shared memory object, else a value parameter
 * @b: if a memref, size of the buffer, else a value parameter
 * @c: if a memref, shared memory identifier, else a value parameter
 *
 * @attr & TEE_PARAM_ATTR_TYPE_MASK indicates if memref or value is used in
 * the union. TEE_PARAM_ATTR_TYPE_VALUE_* indicates value and
 * TEE_PARAM_ATTR_TYPE_MEMREF_* indicates memref. TEE_PARAM_ATTR_TYPE_NONE
 * indicates that none of the members are used.
 *
 * Shared memory is allocated with TEE_IOC_SHM_ALLOC which returns an
 * identifier representing the shared memory object. A memref can reference
 * a part of a shared memory by specifying an offset (@a) and size (@b) of
 * the object. To supply the entire shared memory object set the offset
 * (@a) to 0 and size (@b) to the previously returned size of the object.
 */
struct tee_param {
	uint64_t attr;
	uint64_t a;
	uint64_t b;
	uint64_t c;
};

/**
 * struct tee_invoke_func_arg - Invokes a function in a Trusted
 * Application
 * @func:	[in] Trusted Application function, specific to the TA
 * @session:	[in] Session id
 * @cancel_id:	[in] Cancellation id, a unique value to identify this request
 * @ret:	[out] return value
 * @ret_origin	[out] origin of the return value
 */
struct tee_invoke_func_arg {
	uint32_t func;
	uint32_t session;
	uint32_t cancel_id;
	uint32_t ret;
	uint32_t ret_origin;
};

/**
 * struct tee_shm - tee shared memory structure
 * @dev:	[out] Pointer to the device driver structure
 * @addr:	[out] Shared buffer pointer
 * @size:	[out] Shared buffer size
 * @flags:	[out] Shared buffer flags
 */
struct tee_shm
{
	const struct device *dev;
	void *addr;
	uint64_t size;
	uint32_t flags;
};

/**
 * @typedef tee_get_version_t
 *
 * @brief Callback API to get current tee version
 *
 * See @a tee_version_get() for argument definitions.
 */
typedef int (*tee_get_version_t)(const struct device *dev, struct tee_version_info *info);

/**
 * @typedef tee_open_session_t
 *
 * @brief Callback API to open session to Trusted Application
 *
 * See @a tee_open_session() for argument definitions.
 */
typedef int (*tee_open_session_t)(const struct device *dev, struct tee_open_session_arg *arg,
				  unsigned int num_param, struct tee_param *param,
				  uint32_t *session_id);
/**
 * @typedef tee_close_session_t
 *
 * @brief Callback API to close session to TA
 *
 * See @a tee_close_session() for argument definitions.
 */
typedef int (*tee_close_session_t)(const struct device *dev, uint32_t session_id);

/**
 * @typedef tee_cancel_t
 *
 * @brief Callback API to cancel open session of invoke function to TA
 *
 * See @a tee_cancel() for argument definitions.
 */
typedef int (*tee_cancel_t)(const struct device *dev, uint32_t session_id, uint32_t cancel_id);

/**
 * @typedef tee_invoke_func_t
 *
 * @brief Callback API to invoke function to TA
 *
 * See @a tee_invoke_func() for argument definitions.
 */
typedef int (*tee_invoke_func_t)(const struct device *dev, struct tee_invoke_func_arg *arg,
				 unsigned int num_param, struct tee_param *param);
/**
 * @typedef tee_shm_register_t
 *
 * @brief Callback API to register shared memory
 *
 * See @a tee_shm_register() for argument definitions.
 */
typedef int (*tee_shm_register_t)(const struct device *dev, void *addr, uint64_t size,
				  uint32_t flags, struct tee_shm *shm);

/**
 * @typedef tee_shm_unregister_t
 *
 * @brief Callback API to unregister shared memory
 *
 * See @a tee_shm_unregister() for argument definitions.
 */
typedef int (*tee_shm_unregister_t)(const struct device *dev, struct tee_shm *shm);

/**
 * @typedef tee_suppl_recv_t
 *
 * @brief Callback API to receive a request for TEE supplicant
 *
 * See @a tee_suppl_recv() for argument definitions.
 */
typedef int (*tee_suppl_recv_t)(const struct device *dev, uint32_t func, unsigned int num_params,
				struct tee_param *param);

/**
 * @typedef tee_suppl_send_t
 *
 * @brief Callback API to send a request for TEE supplicant
 *
 * See @a tee_suppl_send() for argument definitions.
 */
typedef int (*tee_suppl_send_t)(const struct device *dev, unsigned int num_params,
				struct tee_param *param);

__subsystem struct tee_driver_api {
	tee_get_version_t get_version;
	tee_open_session_t open_session;
	tee_close_session_t close_session;
	tee_cancel_t cancel;
	tee_invoke_func_t invoke_func;
        tee_shm_register_t shm_register;
	tee_shm_unregister_t shm_unregister;
	tee_suppl_recv_t suppl_recv;
	tee_suppl_send_t suppl_send;
};

/**
 * @brief Get the current TEE version info
 *
 * Returns @info as tee version info which includes capabilities description
 *
 * @param dev TEE device
 * @param info Structure to return the capabilities
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_get_version(const struct device *dev, struct tee_version_info *info);

static inline int z_impl_tee_get_version(const struct device *dev, struct tee_version_info *info)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->get_version == NULL) {
		return -ENOSYS;
	}

	return api->get_version(dev, info);
}

/**
 * @brief Open session for Trusted Environment
 *
 * Opens the new session to the Trusted Environment
 *
 * @param dev TEE device
 * @param arg Structure with the session arguments
 * @param num_params Number of the additional params to be passed
 * @param param List of the params to pass to open_session call
 * @param session_id Returns id of the created session
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_open_session(const struct device *dev, struct tee_open_session_arg *arg,
			       unsigned int num_param, struct tee_param *param,
			       uint32_t *session_id);

static inline int z_impl_tee_open_session(const struct device *dev,
					  struct tee_open_session_arg *arg,
					  unsigned int num_param, struct tee_param *param,
					  uint32_t *session_id)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->open_session == NULL) {
		return -ENOSYS;
	}

	return api->open_session(dev, arg, num_param, param, session_id);
}

/**
 * @brief Close session for Trusted Environment
 *
 * Closes session to the Trusted Environment
 *
 * @param dev TEE device
 * @param session_id session to close
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_close_session(const struct device *dev, uint32_t session_id);

static inline int z_impl_tee_close_session(const struct device *dev, uint32_t session_id)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->close_session == NULL) {
		return -ENOSYS;
	}

	return api->close_session(dev, session_id);
}

/**
 * @brief Cancel session or invoke function for Trusted Environment
 *
 * Cancels session or invoke function for TA
 *
 * @param dev TEE device
 * @param session_id session to close
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_cancel(const struct device *dev, uint32_t session_id, uint32_t cancel_id);

static inline int z_impl_tee_cancel(const struct device *dev, uint32_t session_id,
				    uint32_t cancel_id)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->cancel == NULL) {
		return -ENOSYS;
	}

	return api->cancel(dev, session_id, cancel_id);
}

/**
 * @brief Invoke function for Trusted Environment Application
 *
 * Invokes function to the TA
 *
 * @param dev TEE device
 * @param arg Structure with the invoke function arguments
 * @param num_params Number of the additional params to be passed
 * @param param List of the params to pass to open_session call
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_invoke_func(const struct device *dev, struct tee_invoke_func_arg *arg,
			      unsigned int num_param, struct tee_param *param);

static inline int z_impl_tee_invoke_func(const struct device *dev, struct tee_invoke_func_arg *arg,
					 unsigned int num_param, struct tee_param *param)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->invoke_func == NULL) {
		return -ENOSYS;
	}

	return api->invoke_func(dev, arg, num_param, param);
}

static inline int tee_add_shm(const struct device *dev, void *addr, size_t size, uint32_t flags,
			      struct tee_shm **shmp)
{
	int rc;
	void *p = addr;
	struct tee_shm *shm;

	if (!shmp || !size) {
		return -EINVAL;
	}

	if (flags & TEE_SHM_ALLOC) {
		p = k_malloc(size);
	}

	if (!p) {
		return -ENOMEM;
	}

	shm = k_malloc(sizeof(struct tee_shm));
	if (!shm) {
		rc = -ENOMEM;
		goto err;
	}

	shm->addr = p;
	shm->size = size;
	shm->flags = flags;
	shm->dev = dev;

	if (flags & TEE_SHM_REGISTER) {
		const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

		if (api->shm_register == NULL) {
			rc = -ENOSYS;
			goto err;
		}

		rc = api->shm_register(dev, addr, size, flags, shm);
		if (rc) {
			goto err;
		}
	}

	*shmp = shm;

	return 0;
err:
	k_free(shm);
	if (flags & TEE_SHM_ALLOC) {
		k_free(p);
	}

	return rc;
}

static inline int tee_rm_shm(const struct device *dev, struct tee_shm *shm)
{
	int rc = 0;

	if (!shm) {
		return -EINVAL;
	}

	if (shm->flags & TEE_SHM_REGISTER) {
		const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

		if (api->shm_unregister) {
			/*
			 * We don't return immediately if callback returned error,
			 * just return this code after cleanup.
			 */
			rc = api->shm_unregister(dev, shm);
		}
	}

	if (shm->flags & TEE_SHM_ALLOC) {
		k_free(shm->addr);
	}

	k_free(shm);

	return rc;
}

/**
 * @brief Register shared memory for Trusted Environment
 *
 * Registers shared memory for TEE
 *
 * @param dev TEE device
 * @param addr Address of the shared memory
 * @param size Size of the shared memory region
 * @param flags Flags to set registering parameters
 * @param shm Return shared memory structure
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_shm_register(const struct device *dev, void *addr, size_t size,
			       uint32_t flags, struct tee_shm **shm);

static inline int z_impl_tee_shm_register(const struct device *dev, void *addr, size_t size,
					  uint32_t flags, struct tee_shm **shm)
{
	return tee_add_shm(dev, addr, size, flags | ~TEE_SHM_ALLOC | TEE_SHM_REGISTER, shm);
}

/**
 * @brief Unregister shared memory for Trusted Environment
 *
 * Unregisters shared memory for TEE
 *
 * @param dev TEE device
 * @param shm Shared memory structure
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_shm_unregister(const struct device *dev, struct tee_shm *shm);

static inline int z_impl_tee_shm_unregister(const struct device *dev, struct tee_shm *shm)
{
	return tee_rm_shm(dev, shm);
}

/**
 * @brief Allocate shared memory region for Trusted Environment
 *
 * Allocate shared memory for TEE
 *
 * @param dev TEE device
 * @param size Region size
 * @param flags to allocate region
 * @param shm Return shared memory structure
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_shm_alloc(const struct device *dev, size_t size, uint32_t flags, struct tee_shm **shm);

static inline int z_impl_tee_shm_alloc(const struct device *dev, size_t size, uint32_t flags,
				       struct tee_shm **shm)
{
	return tee_add_shm(dev, NULL, size, flags | TEE_SHM_ALLOC | TEE_SHM_REGISTER, shm);
}

/**
 * @brief Free shared memory region for Trusted Environment
 *
 * Frees shared memory for TEE
 *
 * @param dev TEE device
 * @param shm Shared memory structure
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_shm_free(const struct device *dev, struct tee_shm *shm);

static inline int z_impl_tee_shm_free(const struct device *dev, struct tee_shm *shm)
{
	return tee_rm_shm(dev, shm);
}

/**
 * @brief Receive a request for TEE Supplicant
 *
 * @param dev TEE device
 * @param func Supplicant function
 * @param num_params Number of parameters to be passed
 * @param param List of the params for send/receive
 *
 * @retval -ENOSYS If callback was not implemented
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_suppl_recv(const struct device *dev, uint32_t func, unsigned int num_params,
			    struct tee_param *param);

static inline int z_impl_tee_suppl_recv(const struct device *dev, uint32_t func,
				       unsigned int num_params, struct tee_param *param)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->suppl_recv == NULL) {
		return -ENOSYS;
	}

	return api->suppl_recv(dev, func, num_params, param);
}

/**
 * @brief Send a request for TEE Supplicant function
 *
 * @param dev TEE device
 * @param num_params Number of parameters to be passed
 * @param param List of the params for send/receive
 *
 * @retval -ENOSYS If callback was not implemented
 * @retval         Return value for sent request
 *
 * @retval 0       On success, negative on error
 */
__syscall int tee_suppl_send(const struct device *dev, unsigned int num_params,
			     struct tee_param *param);

static inline int z_impl_tee_suppl_send(const struct device *dev, unsigned int num_params,
					struct tee_param *param)
{
	const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

	if (api->suppl_send == NULL) {
		return -ENOSYS;
	}

	return api->suppl_send(dev, num_params, param);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/tee.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_TEE_H_ */
