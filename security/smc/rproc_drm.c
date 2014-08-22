/*
 * Copyright (c) 2011 Texas Instruments, Inc.
 * Copyright (c) 2011 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This file implements the non-secure rproc and smc interface/integration
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/rproc_drm.h>

#include "tee_client_api.h"
#include "tf_defs.h"
#include "tf_util.h"

static TEEC_Result rproc_drm_initialize(TEEC_Context *teec_context,
					TEEC_Session *teec_session)
{
	static const TEEC_UUID drm_uuid = COMMON_SECURE_DRIVER_UUID;
	static u32 drm_gid = 1019;
	TEEC_Result result;

	result = TEEC_InitializeContext(NULL, teec_context);
	if (result != TEEC_SUCCESS) {
		pr_info("%s: TEEC_InitializeContext failed!\n", __func__);
		goto exit;
	}

	result = TEEC_OpenSession(teec_context, teec_session, &drm_uuid,
			TEEC_LOGIN_PRIVILEGED, &drm_gid, NULL, NULL);
	if (result != TEEC_SUCCESS) {
		pr_info("%s: TEEC_OpenSession failed!\n", __func__);
		TEEC_FinalizeContext(teec_context);
	}

exit:
	return result;
}

static TEEC_Result rproc_drm_finalize(TEEC_Context *teec_context,
					TEEC_Session *teec_session)
{
	TEEC_CloseSession(teec_session);
	TEEC_FinalizeContext(teec_context);
	return TEEC_SUCCESS;
}

static TEEC_Result _rproc_drm_invoke_secure_service(enum rproc_service_enum
						    service,
						    struct rproc_sec_params
						    *rproc_params)
{
	TEEC_Result result;
	TEEC_Operation operation;
	TEEC_Context teec_context;
	TEEC_Session teec_session;
	u32 param1 = 0;
	u32 param2 = 0;

	memset(&operation, 0, sizeof(TEEC_Operation));

#if defined(CONFIG_DRM_WIDEVINE)
	switch (service) {
	case AUTHENTICATION_A0:
	case EXIT_SECURE_PLAYBACK:
	{
		param1 = TEEC_NONE;
		param2 = TEEC_NONE;
		break;
	}
	case AUTHENTICATION_A1:
	case AUTHENTICATION_A2:
	case ENTER_SECURE_PLAYBACK_AFTER_AUTHENTICATION:
	{
		if (!rproc_params) {
			dprintk("%s, bad parameters for service: 0x%x, "
				"params: 0x%x\n",
			       __func__, service, rproc_params);
			result = TEEC_ERROR_NO_DATA;
			goto out;
		}
		operation.params[0].tmpref.buffer =
			(void *)rproc_params;
		operation.params[0].tmpref.size =
			sizeof(struct rproc_sec_params);
		param1 = TEEC_MEMREF_TEMP_INPUT;
		break;
	}
	default:
	{
		dprintk("%s, invalid service: 0x%x\n", __func__, service);
		result = TEEC_ERROR_BAD_PARAMETERS;
		goto out;
	}
	}
#elif defined(CONFIG_DRM_PLAYREADY)
	switch (service) {
	case RPROC_EXIT_SECURE_PLAYBACK:
	case RPROC_ENTER_SECURE_PLAYBACK:
		break;
	default:
		dprintk("%s, invalid service: 0x%x\n", __func__, service);
		result = TEEC_ERROR_BAD_PARAMETERS;
		goto out;
	}

	param1 = TEEC_NONE;
	param2 = TEEC_NONE;
#else
  #error "No valid DRM scheme configured"
#endif

	result = rproc_drm_initialize(&teec_context, &teec_session);
	if (result != TEEC_SUCCESS) {
		dprintk("%s, rproc_drm_initialize failed: "
			"service: 0x%x, ret = %d\n",
		       __func__, service, result);
		goto out;
	}

	operation.paramTypes = TEEC_PARAM_TYPES(param1, param2,
						TEEC_NONE, TEEC_NONE);
	result = TEEC_InvokeCommand(&teec_session, service, &operation, NULL);
	dprintk("%s, TEEC_InvokeCommand returned: ret = %d, service = 0x%x\n",
	       __func__, service, result);
	if (result != TEEC_SUCCESS)
		pr_info("%s: TEEC_InvokeCommand failed (service=%d, result=0x%08x)!\n",
				__func__, service, result);
	rproc_drm_finalize(&teec_context, &teec_session);
out:
	return result;
}

static int rproc_drm_invoke_service(enum rproc_service_enum service,
				    struct rproc_sec_params
				    *rproc_params)
{
	int ret;

	ret = _rproc_drm_invoke_secure_service(service, rproc_params);

	if (ret == TEEC_SUCCESS)
		return 0;
	else if (ret == TEEC_ERROR_BUSY)
		return -EBUSY;
	else
		return -EACCES;
}

static int __init rproc_drm_init(void)
{
	dprintk("%s, registering rproc_drm_invoke_service\n", __func__);
	return rproc_register_drm_service(rproc_drm_invoke_service);
}

MODULE_LICENSE("GPL v2");
module_init(rproc_drm_init);


