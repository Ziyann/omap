/**
 * Copyright (c) 2016 Luden
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* SMC PA implementation for tuna devices has the following bug.
 * Once SMC PA is loaded and PKCS11 session is opened at least once,
 * SHA2MD5 clock is stuck in TRANSITION state. This consumes extra
 * battery and prevents the device from going into sleep mode.
 *
 * See also the report of another user about this problem:
 * https://e2e.ti.com/support/omap/f/849/t/235401,
 * potentially the bug reported in this commit
 * https://android.googlesource.com/device/samsung/tuna/+/b74801dc22bb4945ddf79b2e12e6328a862d68c3
 * is also related.
 *
 * It's hard to say what is the exact cause of this bug given that
 * no information about either SHA2MD5, SMC PA or OMAP4 PPA is public.
 *
 * I was able to reproduce the similar issue by setting SHA2MD5
 * SYSCONFIG register to 0x88 (SADVANCED | SDMA_EN), potentially
 * there's a "secure world" version of this register that is
 * programmed incorrectly in PKCS11 session initialization?
 *
 * In any case, given that there's no way to modify SMC PA to fix
 * this bug even if the exact reason was known (SMC PA is signed
 * and the signature is verified during its loading), the whole
 * investigation as to what causes it exactly is not that useful.
 *
 * However, apparently once SHA2MD5 "secure world" functionality
 * is used at least once, the stuckness goes away (most likely HWA
 * is re-initialized at this point in "secure world" and this time
 * it's done properly?). It's enough to do this just once after the
 * boot.
 *
 * Therefore, the code below implements a work-around for the
 * stuckness problem by performing dummy calculation of the MD5
 * hash of 1 byte buffer.
 *
 * Note that the computation has to happen in the "secure world",
 * using just the "normal world" interface to HWA doesn't cut it. */

#include <linux/types.h>
#include <linux/slab.h>

#include "tee_client_api.h"
#include "tf_crypto.h"

#define SERVICE_SYSTEM_UUID \
	{ 0x56304b83, 0x5c4e, 0x4428, \
		{ 0xb9, 0x9e, 0x60, 0x5c, 0x96, 0xae, 0x58, 0xd6 } }

#define CKF_SERIAL_SESSION	0x00000004
/* It doesn't matter which hash is used as long as HWA supports it. */
#define CKM_MD5			0x00000210

#define SERVICE_SYSTEM_PKCS11_C_DIGESTINIT_COMMAND_ID		0x00000026
#define SERVICE_SYSTEM_PKCS11_C_DIGEST_COMMAND_ID		0x00000027
#define SERVICE_SYSTEM_PKCS11_C_OPEN_SESSION_COMMAND_ID		0x00000042
#define SERVICE_SYSTEM_PKCS11_C_CLOSE_SESSION_COMMAND_ID	0x00000043

/* Exact buffer size doesn't matter as long as it's > 0. */
#define BUFFER_SIZE 1
/* Size of MD5 digest. */
#define DIGEST_SIZE 16
#define TOTAL_BUFFER_SIZE (BUFFER_SIZE + DIGEST_SIZE)

static void _tf_crypto_digest_apply_clock_workaround(struct work_struct *work)
{
	uint8_t *buf, *digest;
	u32 cmd;
	u32 crypto_session;
	TEEC_Result ret;
	TEEC_Operation op;
	TEEC_Context teec_context;
	TEEC_Session teec_session;
	TEEC_UUID uuid = SERVICE_SYSTEM_UUID;
	bool result = true;

	tf_crypto_clockdomain_wakeup();

	ret = TEEC_InitializeContext(NULL, &teec_context);
	if (ret != TEEC_SUCCESS) {
		tf_crypto_clockdomain_idle();
		return;
	}

	buf = kmalloc(TOTAL_BUFFER_SIZE, GFP_KERNEL);

	if (buf == NULL) {
		TEEC_FinalizeContext(&teec_context);
		tf_crypto_clockdomain_idle();
		return;
	}

	digest = buf + BUFFER_SIZE;

	/* Call C_OpenSession */
	memset(&op, 0, sizeof(TEEC_Operation));

	op.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE, TEEC_NONE,
		TEEC_NONE);

	ret = TEEC_OpenSession(&teec_context, &teec_session,
		&uuid, TEEC_LOGIN_PUBLIC, NULL, &op, NULL);
	if (ret != TEEC_SUCCESS) {
		TEEC_FinalizeContext(&teec_context);
		tf_crypto_clockdomain_idle();
		kfree(buf);
		return;
	}

	memset(&op, 0, sizeof(TEEC_Operation));

	op.paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INOUT, TEEC_NONE, TEEC_NONE,
		TEEC_NONE);
	op.params[0].value.a = 0;
	op.params[0].value.b = CKF_SERIAL_SESSION;

	cmd = SERVICE_SYSTEM_PKCS11_C_OPEN_SESSION_COMMAND_ID & 0x00007FFF;

	ret = TEEC_InvokeCommand(&teec_session, cmd, &op, NULL);
	if (ret != TEEC_SUCCESS) {
		printk(KERN_ERR "%s: TEEC_InvokeCommand returned 0x%08x\n",
			__func__, ret);
		result = false;
		goto exit;
	}

	crypto_session = op.params[0].value.a;

	/* Call C_DigestInit */
	memset(&op, 0, sizeof(TEEC_Operation));

	op.paramTypes = TEEC_PARAM_TYPES(TEEC_VALUE_INPUT,
		TEEC_MEMREF_TEMP_INPUT, TEEC_NONE, TEEC_NONE);
	op.params[0].value.a = CKM_MD5;
	op.params[1].tmpref.buffer = NULL;
	op.params[1].tmpref.size = 0;

	cmd = (crypto_session << 16) |
		(SERVICE_SYSTEM_PKCS11_C_DIGESTINIT_COMMAND_ID & 0x7fff);

	ret = TEEC_InvokeCommand(&teec_session, cmd, &op, NULL);
	if (ret != TEEC_SUCCESS) {
		printk(KERN_ERR "%s: TEEC_InvokeCommand returned 0x%08x\n",
			__func__, ret);
		result = false;
	}

	/* Call C_Digest */
	memset(&op, 0, sizeof(TEEC_Operation));

	op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
		TEEC_MEMREF_TEMP_OUTPUT, TEEC_NONE, TEEC_NONE);
	op.params[0].tmpref.buffer = (uint8_t *) buf;
	op.params[0].tmpref.size = (uint32_t) BUFFER_SIZE;
	op.params[1].tmpref.buffer = (uint8_t *) digest;
	op.params[1].tmpref.size = (uint32_t) DIGEST_SIZE;

	cmd = (crypto_session << 16) |
		(SERVICE_SYSTEM_PKCS11_C_DIGEST_COMMAND_ID & 0x7fff);

	ret = TEEC_InvokeCommand(&teec_session, cmd, &op, NULL);
	if (ret != TEEC_SUCCESS) {
		printk(KERN_ERR "%s: TEEC_InvokeCommand returned 0x%08x\n",
			__func__, ret);
		result = false;
	}

	/* Call C_CloseSession */
	memset(&op, 0, sizeof(TEEC_Operation));

	op.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE, TEEC_NONE,
		TEEC_NONE);

	cmd = (crypto_session << 16) |
		(SERVICE_SYSTEM_PKCS11_C_CLOSE_SESSION_COMMAND_ID & 0x7fff);

	ret = TEEC_InvokeCommand(&teec_session, cmd, &op, NULL);
	if (ret != TEEC_SUCCESS) {
		printk(KERN_ERR "%s: TEEC_InvokeCommand returned 0x%08x\n",
			__func__, ret);
		result = false;
	}

	if (result) {
		u32 clock_val = tf_crypto_read_clock_value(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);
		result = (clock_val & 0x30000) != 0x10000;
	}

exit:
	TEEC_CloseSession(&teec_session);
	TEEC_FinalizeContext(&teec_context);

	tf_crypto_clockdomain_idle();
	kfree(buf);

	printk(KERN_INFO "%s: SHA2MD5 clock work-around result: %s\n",
		__func__, result ? "succeeded" : "failed");

	return;
}

static DECLARE_WORK(digest_clock_workaround_work, _tf_crypto_digest_apply_clock_workaround);

void tf_crypto_digest_apply_clock_workaround(void)
{
	u32 clock_val;

	clock_val = tf_crypto_read_clock_value(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);
	if ((clock_val & 0x30000) == 0x10000) {
		printk(KERN_INFO "%s: SHA2MD5 clock is stuck, trying to work-around\n", __func__);
		/* The calling path of our workaround function might involve interrupt handlers
		 * and other interesting parts of the kernel code. That doesn't play nice
		 * with the functionality used by that function (e.g. memory allocation) -
		 * therefore, just schedule it for execution from the more appropriate
		 * context instead of executing it directly. */
		schedule_work(&digest_clock_workaround_work);
	}
}
