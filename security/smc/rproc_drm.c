/*
 * Copyright (c) 2011 Texas Instruments Inc
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This software is the confidential and proprietary information of
 * Trusted Logic S.A. ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Trusted Logic S.A.
 *
 * TRUSTED LOGIC S.A. MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE
 * SUITABILITY OF THE SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT. TRUSTED LOGIC S.A. SHALL
 * NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY LICENSEE AS A RESULT OF USING,
 * MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
 */

/*
 * This file implements the non-secure rproc and smc interface/integration
 */

#include <linux/types.h>
#include <linux/module.h>

#include "tee_client_api.h"
#include "tf_defs.h"

/* 7B1DD682-1077-4939-9755-B6192C5CC5FD */
#define WVDRM_UUID {0x7B1DD682, 0x1077, 0x4939, \
			{0x97, 0x55, 0xB6, 0x19, 0x2C, 0x5C, 0xC5, 0xFD} }

#define WVDRM_ENTER_SECURE_PLAYBACK	0x00003000

#define WVDRM_EXIT_SECURE_PLAYBACK	0x00003001

enum rproc_drm_s_state {
	RPROC_DRM_SECURE_LEAVE,
	RPROC_DRM_SECURE_ENTER
};

static enum rproc_drm_s_state s_state;

static TEEC_Result rproc_drm_initialize(TEEC_Context *teec_context,
					TEEC_Session *teec_session)
{
	static const TEEC_UUID drm_uuid = WVDRM_UUID;
	static u32 drm_gid = 1019;
	TEEC_Result result;

	result = TEEC_InitializeContext(NULL, teec_context);
	if (result != TEEC_SUCCESS)
		goto exit;

	result = TEEC_OpenSession(teec_context, teec_session, &drm_uuid,
			TEEC_LOGIN_PRIVILEGED, &drm_gid, NULL, NULL);
	if (result != TEEC_SUCCESS)
		TEEC_FinalizeContext(teec_context);

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

static TEEC_Result _rproc_drm_invoke_secure_service(bool enable)
{
	TEEC_Result result;
	TEEC_Operation operation;
	TEEC_Context teec_context;
	TEEC_Session teec_session;
	u32 command;

	result = rproc_drm_initialize(&teec_context, &teec_session);
	if (result != TEEC_SUCCESS)
		goto out;

	/*
	 * TODO: Check with security team if we need to pass a parameter to get
	 * the result of the service, or the actual return value is enough
	 */
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE,
						TEEC_NONE, TEEC_NONE);
	command = (enable ? WVDRM_ENTER_SECURE_PLAYBACK :
				WVDRM_EXIT_SECURE_PLAYBACK);
	result = TEEC_InvokeCommand(&teec_session, command, &operation, NULL);
	rproc_drm_finalize(&teec_context, &teec_session);
out:
	return result;
}

int rproc_drm_invoke_service(bool enable)
{
	int ret;

	/*
	 * TODO: Need to analyze if we get different errors for
	 * failures in authentication or firewalling
	 */
	if ((s_state == RPROC_DRM_SECURE_ENTER && enable) ||
		(s_state == RPROC_DRM_SECURE_LEAVE && !enable))
		return 0;

	ret = _rproc_drm_invoke_secure_service(enable);
	s_state = (enum rproc_drm_s_state) enable;

	return ret == TEEC_SUCCESS ? 0 : -EACCES;
}
EXPORT_SYMBOL(rproc_drm_invoke_service);
