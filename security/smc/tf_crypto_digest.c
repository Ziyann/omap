/**
 * Copyright (c) 2011 Trusted Logic S.A.
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

#include "tf_defs.h"
#include "tf_util.h"
#include "tf_crypto.h"
#include "tf_dma.h"
#include "tf_zebra.h"

#include <linux/io.h>
#include <linux/crypto.h>
#include <crypto/internal/hash.h>

/*
 * SHA2/MD5 Hardware Accelerator: Base address for SHA2/MD5 HIB2
 * This is referenced as the SHA2MD5 module in the Crypto TRM
 */
#define DIGEST1_REGS_HW_ADDR			0x4B101000

/*
 * IRQSTATUS register Masks
 */
#define DIGEST_IRQSTATUS_OUTPUT_READY_BIT	(1 << 0)
#define DIGEST_IRQSTATUS_INPUT_READY_BIT	(1 << 1)
#define DIGEST_IRQSTATUS_PARTHASH_READY_BIT	(1 << 2)
#define DIGEST_IRQSTATUS_CONTEXT_READY_BIT	(1 << 3)

/*
 * MODE register Masks
 */
#define DIGEST_MODE_GET_ALGO(x) \
	(cpu_is_omap54xx() ? ((x) & 0x7) : (((x) & 0x6) >> 1))
#define DIGEST_MODE_GET_OMAP44XX_ALGO(x)        ((x & 0x6) >> 1)
#define DIGEST_MODE_GET_OMAP54XX_ALGO(x)        ((x) & 0x7)
#define DIGEST_MODE_SET_ALGO(x, a) \
	(cpu_is_omap54xx() ? \
		((a) | ((x) & 0xFFFFFFF8)) : (((a) << 1) | ((x) & 0xFFFFFFF9)))

#define DIGEST_MODE_ALGO_CONST_BIT		(1 << 3)
#define DIGEST_MODE_CLOSE_HASH_BIT		(1 << 4)

/*
 * SYSCONFIG register masks
 */
#define DIGEST_SYSCONFIG_PIT_EN_BIT		(1 << 2)
#define DIGEST_SYSCONFIG_PDMA_EN_BIT		(1 << 3)
#define DIGEST_SYSCONFIG_PCONT_SWT_BIT		(1 << 6)
#define DIGEST_SYSCONFIG_PADVANCED_BIT		(1 << 7)

/*-------------------------------------------------------------------------*/
/*				 Digest Context				*/
/*-------------------------------------------------------------------------*/
/**
 * This structure contains the registers of the SHA1/MD5 HW accelerator.
 */
struct sha1_md5_reg {
	u32 SHA_ODIGEST_A;		/* 0x00 Outer Digest A      */
	u32 SHA_ODIGEST_B;		/* 0x04 Outer Digest B      */
	u32 SHA_ODIGEST_C;		/* 0x08 Outer Digest C      */
	u32 SHA_ODIGEST_D;		/* 0x0C Outer Digest D      */
	u32 SHA_ODIGEST_E;		/* 0x10 Outer Digest E      */
	u32 SHA_ODIGEST_F;		/* 0x14 Outer Digest F      */
	u32 SHA_ODIGEST_G;		/* 0x18 Outer Digest G      */
	u32 SHA_ODIGEST_H;		/* 0x1C Outer Digest H      */
	u32 SHA_IDIGEST_A;		/* 0x20 Inner Digest A      */
	u32 SHA_IDIGEST_B;		/* 0x24 Inner Digest B      */
	u32 SHA_IDIGEST_C;		/* 0x28 Inner Digest C      */
	u32 SHA_IDIGEST_D;		/* 0x2C Inner Digest D      */
	u32 SHA_IDIGEST_E;		/* 0x30 Inner Digest E      */
	u32 SHA_IDIGEST_F;		/* 0x34 Inner Digest F      */
	u32 SHA_IDIGEST_G;		/* 0x38 Inner Digest G      */
	u32 SHA_IDIGEST_H;		/* 0x3C Inner Digest H      */
	u32 SHA_DIGEST_COUNT;		/* 0x40 Digest count        */
	u32 SHA_MODE;			/* 0x44 Digest mode         */
	u32 SHA_LENGTH;			/* 0x48 Data length         */

	u32 reserved0[13];

	u32 SHA_DIN_0;			/* 0x80 Data 0              */
	u32 SHA_DIN_1;			/* 0x84 Data 1              */
	u32 SHA_DIN_2;			/* 0x88 Data 2              */
	u32 SHA_DIN_3;			/* 0x8C Data 3              */
	u32 SHA_DIN_4;			/* 0x90 Data 4              */
	u32 SHA_DIN_5;			/* 0x94 Data 5              */
	u32 SHA_DIN_6;			/* 0x98 Data 6              */
	u32 SHA_DIN_7;			/* 0x9C Data 7              */
	u32 SHA_DIN_8;			/* 0xA0 Data 8              */
	u32 SHA_DIN_9;			/* 0xA4 Data 9              */
	u32 SHA_DIN_10;			/* 0xA8 Data 10             */
	u32 SHA_DIN_11;			/* 0xAC Data 11             */
	u32 SHA_DIN_12;			/* 0xB0 Data 12             */
	u32 SHA_DIN_13;			/* 0xB4 Data 13             */
	u32 SHA_DIN_14;			/* 0xB8 Data 14             */
	u32 SHA_DIN_15;			/* 0xBC Data 15             */
	u32 SHA_DIN_16;			/* 0xC0 Data 16             */
	u32 SHA_DIN_17;			/* 0xC4 Data 17             */
	u32 SHA_DIN_18;			/* 0xC8 Data 18             */
	u32 SHA_DIN_19;			/* 0xCC Data 19             */
	u32 SHA_DIN_20;			/* 0xD0 Data 20             */
	u32 SHA_DIN_21;			/* 0xD4 Data 21             */
	u32 SHA_DIN_22;			/* 0xD8 Data 22             */
	u32 SHA_DIN_23;			/* 0xDC Data 23             */
	u32 SHA_DIN_24;			/* 0xE0 Data 24             */
	u32 SHA_DIN_25;			/* 0xE4 Data 25             */
	u32 SHA_DIN_26;			/* 0xE8 Data 26             */
	u32 SHA_DIN_27;			/* 0xEC Data 27             */
	u32 SHA_DIN_28;			/* 0xF0 Data 28             */
	u32 SHA_DIN_29;			/* 0xF4 Data 29             */
	u32 SHA_DIN_30;			/* 0xF8 Data 30             */
	u32 SHA_DIN_31;			/* 0xFC Data 31             */
	u32 REVISION;			/* 0x100 Revision           */

	u32 reserved2[3];

	u32 SYSCONFIG;			/* 0x110 Config             */
	u32 SYSSTATUS;			/* 0x114 Status             */
	u32 IRQSTATUS;			/* 0x118 IRQ Status         */
	u32 IRQENABLE;			/* 0x11C IRQ Enable         */

	u32 reserved3[56];
	u32 SHA512_ODIGEST_A;		/* 0x200 Outer Digest A      */
	u32 SHA512_ODIGEST_B;		/* 0x204 Outer Digest B      */
	u32 SHA512_ODIGEST_C;		/* 0x208 Outer Digest C      */
	u32 SHA512_ODIGEST_D;		/* 0x20C Outer Digest D      */
	u32 SHA512_ODIGEST_E;		/* 0x210 Outer Digest E      */
	u32 SHA512_ODIGEST_F;		/* 0x214 Outer Digest F      */
	u32 SHA512_ODIGEST_G;		/* 0x218 Outer Digest G      */
	u32 SHA512_ODIGEST_H;		/* 0x21C Outer Digest H      */
	u32 SHA512_ODIGEST_I;		/* 0x220 Outer Digest I      */
	u32 SHA512_ODIGEST_J;		/* 0x224 Outer Digest J      */
	u32 SHA512_ODIGEST_K;		/* 0x228 Outer Digest K      */
	u32 SHA512_ODIGEST_L;		/* 0x22C Outer Digest L      */
	u32 SHA512_ODIGEST_M;		/* 0x230 Outer Digest M      */
	u32 SHA512_ODIGEST_N;		/* 0x234 Outer Digest N      */
	u32 SHA512_ODIGEST_O;		/* 0x238 Outer Digest O      */
	u32 SHA512_ODIGEST_P;		/* 0x23C Outer Digest P      */

	u32 SHA512_IDIGEST_A;		/* 0x240 Inner Digest A      */
	u32 SHA512_IDIGEST_B;		/* 0x244 Inner Digest B      */
	u32 SHA512_IDIGEST_C;		/* 0x248 Inner Digest C      */
	u32 SHA512_IDIGEST_D;		/* 0x24C Inner Digest D      */
	u32 SHA512_IDIGEST_E;		/* 0x250 Inner Digest E      */
	u32 SHA512_IDIGEST_F;		/* 0x254 Inner Digest F      */
	u32 SHA512_IDIGEST_G;		/* 0x258 Inner Digest G      */
	u32 SHA512_IDIGEST_H;		/* 0x25C Inner Digest H      */
	u32 SHA512_IDIGEST_I;		/* 0x260 Inner Digest I      */
	u32 SHA512_IDIGEST_J;		/* 0x264 Inner Digest J      */
	u32 SHA512_IDIGEST_K;		/* 0x268 Inner Digest K      */
	u32 SHA512_IDIGEST_L;		/* 0x26C Inner Digest L      */
	u32 SHA512_IDIGEST_M;		/* 0x270 Inner Digest M      */
	u32 SHA512_IDIGEST_N;		/* 0x274 Inner Digest N      */
	u32 SHA512_IDIGEST_O;		/* 0x278 Inner Digest O      */
	u32 SHA512_IDIGEST_P;		/* 0x27C Inner Digest P      */

	u32 SHA512_DIGEST_COUNT;	/* 0x280 Digest count        */
	u32 SHA512_MODE;		/* 0x284 Digest mode         */
	u32 SHA512_LENGTH;		/* 0x288 Data length         */

};

static struct sha1_md5_reg *sha1_md5_reg;

static const u8 md5OverEmptyString[] = {
	0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04,
	0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8, 0x42, 0x7e
};

static const u8 sha1OverEmptyString[] = {
	0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d,
	0x32, 0x55, 0xbf, 0xef, 0x95, 0x60, 0x18, 0x90,
	0xaf, 0xd8, 0x07, 0x09
};

static const u8 sha224OverEmptyString[] = {
	0xd1, 0x4a, 0x02, 0x8c, 0x2a, 0x3a, 0x2b, 0xc9,
	0x47, 0x61, 0x02, 0xbb, 0x28, 0x82, 0x34, 0xc4,
	0x15, 0xa2, 0xb0, 0x1f, 0x82, 0x8e, 0xa6, 0x2a,
	0xc5, 0xb3, 0xe4, 0x2f
};

static const u8 sha256OverEmptyString[] = {
	0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
	0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
	0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
	0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55
};

static const u8 sha384OverEmptyString[] = {
	0x38, 0xb0, 0x60, 0xa7, 0x51, 0xac, 0x96, 0x38,
	0x4c, 0xd9, 0x32, 0x7e, 0xb1, 0xb1, 0xe3, 0x6a,
	0x21, 0xfd, 0xb7, 0x11, 0x14, 0xbe, 0x07, 0x43,
	0x4c, 0x0c, 0xc7, 0xbf, 0x63, 0xf6, 0xe1, 0xda,
	0x27, 0x4e, 0xde, 0xbf, 0xe7, 0x6f, 0x65, 0xfb,
	0xd5, 0x1a, 0xd2, 0xf1, 0x48, 0x98, 0xb9, 0x5b
};


static const u8 sha512OverEmptyString[] = {
	0xcf, 0x83, 0xe1, 0x35, 0x7e, 0xef, 0xb8, 0xbd,
	0xf1, 0x54, 0x28, 0x50, 0xd6, 0x6d, 0x80, 0x07,
	0xd6, 0x20, 0xe4, 0x05, 0x0b, 0x57, 0x15, 0xdc,
	0x83, 0xf4, 0xa9, 0x21, 0xd3, 0x6c, 0xe9, 0xce,
	0x47, 0xd0, 0xd1, 0x3c, 0x5d, 0x85, 0xf2, 0xb0,
	0xff, 0x83, 0x18, 0xd2, 0x87, 0x7e, 0xec, 0x2f,
	0x63, 0xb9, 0x31, 0xbd, 0x47, 0x41, 0x7a, 0x81,
	0xa5, 0x38, 0x32, 0x7a, 0xf9, 0x27, 0xda, 0x3e
};

/*------------------------------------------------------------------------
 *Forward declarations
 *------------------------------------------------------------------------- */

static void tf_digest_hw_perform_64b(u32 *data,
				u32 algo, u32 bytes_processed);
static bool tf_digest_hw_perform_dma(u8 *data, u32 nDataLength,
				u32 algo, u32 bytes_processed,
				unsigned int buffer_origin);

static bool tf_digest_update_dma(
	struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length, unsigned int buffer_origin);

/*------------------------------------------------------------------------- */

static unsigned long tf_cpy_from(
void *to, const void *from, unsigned long n, unsigned int buffer_origin)
{
	if (buffer_origin == TF_BUFFER_KERNEL) {
		memcpy(to, from, n);
		return 0;
	} else {
		return copy_from_user(to, from, n);
	}
}

/*-------------------------------------------------------------------------
 *Save HWA registers into the specified operation state structure
 *------------------------------------------------------------------------*/
static void tf_digest_save_registers(
	struct tf_crypto_sha_operation_state *sha_state)
{
	dpr_info("%s: State=%p\n", __func__, sha_state);

	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
		DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
		sha_state->SHA_DIGEST_A =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_A);
		sha_state->SHA_DIGEST_B =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_B);
		sha_state->SHA_DIGEST_C =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_C);
		sha_state->SHA_DIGEST_D =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_D);
		sha_state->SHA_DIGEST_E =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_E);
		sha_state->SHA_DIGEST_F =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_F);
		sha_state->SHA_DIGEST_G =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_G);
		sha_state->SHA_DIGEST_H =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_H);
		sha_state->SHA512_DIGEST_I =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_I);
		sha_state->SHA512_DIGEST_J =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_J);
		sha_state->SHA512_DIGEST_K =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_K);
		sha_state->SHA512_DIGEST_L =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_L);
		sha_state->SHA512_DIGEST_M =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_M);
		sha_state->SHA512_DIGEST_N =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_N);
		sha_state->SHA512_DIGEST_O =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_O);
		sha_state->SHA512_DIGEST_P =
			INREG32(&sha1_md5_reg->SHA512_IDIGEST_P);
	} else {
		sha_state->SHA_DIGEST_A =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_A);
		sha_state->SHA_DIGEST_B =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_B);
		sha_state->SHA_DIGEST_C =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_C);
		sha_state->SHA_DIGEST_D =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_D);
		sha_state->SHA_DIGEST_E =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_E);
		sha_state->SHA_DIGEST_F =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_F);
		sha_state->SHA_DIGEST_G =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_G);
		sha_state->SHA_DIGEST_H =
			INREG32(&sha1_md5_reg->SHA_IDIGEST_H);
	}
}

/*-------------------------------------------------------------------------
 *Restore the HWA registers from the operation state structure
 *-------------------------------------------------------------------------*/
static void tf_digest_restore_registers(
	struct tf_crypto_sha_operation_state *sha_state)
{
	dpr_info("%s: State=%p\n", __func__, sha_state);

	if (sha_state->bytes_processed != 0) {
		/*
		 * Some bytes were already processed. Initialize
		 * previous digest
		 */
		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_A,
				sha_state->SHA_DIGEST_A);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_B,
				sha_state->SHA_DIGEST_B);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_C,
				sha_state->SHA_DIGEST_C);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_D,
				sha_state->SHA_DIGEST_D);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_E,
				sha_state->SHA_DIGEST_E);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_F,
				sha_state->SHA_DIGEST_F);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_G,
				sha_state->SHA_DIGEST_G);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_H,
				sha_state->SHA_DIGEST_H);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_I,
				sha_state->SHA512_DIGEST_I);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_J,
				sha_state->SHA512_DIGEST_J);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_K,
				sha_state->SHA512_DIGEST_K);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_L,
				sha_state->SHA512_DIGEST_L);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_M,
				sha_state->SHA512_DIGEST_M);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_N,
				sha_state->SHA512_DIGEST_N);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_O,
				sha_state->SHA512_DIGEST_O);
			OUTREG32(&sha1_md5_reg->SHA512_IDIGEST_P,
				sha_state->SHA512_DIGEST_P);
		} else {
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_A,
				sha_state->SHA_DIGEST_A);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_B,
				sha_state->SHA_DIGEST_B);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_C,
				sha_state->SHA_DIGEST_C);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_D,
				sha_state->SHA_DIGEST_D);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_E,
				sha_state->SHA_DIGEST_E);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_F,
				sha_state->SHA_DIGEST_F);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_G,
				sha_state->SHA_DIGEST_G);
			OUTREG32(&sha1_md5_reg->SHA_IDIGEST_H,
				sha_state->SHA_DIGEST_H);
		}
	}

	OUTREG32(&sha1_md5_reg->SYSCONFIG, 0);
}

/*------------------------------------------------------------------------- */

static bool tf_digest_update(struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length, unsigned int buffer_origin)
{
	u32 dma_use = PUBLIC_CRYPTO_DMA_USE_NONE;
	u32 BlockLength = HASH_BLOCK_BYTES_LENGTH;


	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
		DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
			BlockLength = HASH_BLOCK_BYTES_LENGTH_128;
	}

	/*
	 *Choice of the processing type
	 */
	if (data_length >= DMA_TRIGGER_IRQ_DIGEST)
		dma_use = PUBLIC_CRYPTO_DMA_USE_IRQ;

	dpr_info("%s: Data=0x%08x/%u, Chunk=%u, Processed=%u, dma_use=0x%08x\n",
		__func__, (u32)data, (u32)data_length,
		sha_state->chunk_length, sha_state->bytes_processed,
		dma_use);

	if (data_length == 0) {
		dpr_info("%s: Nothing to process\n", __func__);
		return true;
	}

	if (dma_use != PUBLIC_CRYPTO_DMA_USE_NONE) {
		/*
		 * Restore the registers of the accelerator from the operation
		 * state
		 */
		tf_digest_restore_registers(sha_state);

		/*perform the updates with DMA */
		if (!tf_digest_update_dma(
			sha_state, data, data_length, buffer_origin))
			return false;

		/* Save the accelerator registers into the operation state */
		tf_digest_save_registers(sha_state);
	} else {
		/*Non-DMA transfer */

		/*(1)We take the chunk buffer wich contains the last saved
		 *data that could not be yet processed because we had not
		 *enough data to make a 64B buffer. Then we try to make a
		 *64B buffer by concatenating it with the new passed data
		 */

		/*Is there any data in the chunk? If yes is it possible to
		 *make a 64B buffer with the new data passed ? */
		if ((sha_state->chunk_length != 0)
			&& (sha_state->chunk_length + data_length >=
				BlockLength)) {

			u32 vLengthToComplete =
				BlockLength - sha_state->chunk_length;

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (tf_cpy_from(
				sha_state->chunk_buffer+sha_state->chunk_length,
				data,
				vLengthToComplete,
				buffer_origin))
				return false;

			if (sha_state->chunk_length + data_length ==
				BlockLength) {
				/*We'll keep some data for the final */
				sha_state->chunk_length = BlockLength;
				dpr_info("%s: Done: Chunk=%u; Processed=%u\n",
					__func__, sha_state->chunk_length,
					sha_state->bytes_processed);
				return true;
			}

			/*
			 * Restore the registers of the accelerator from the
			 * operation state
			 */
			tf_digest_restore_registers(sha_state);

			/*Then we send this buffer to the HWA */
			tf_digest_hw_perform_64b(
				(u32 *)sha_state->chunk_buffer, sha_state->CTRL,
				sha_state->bytes_processed);

			/*
			 * Save the accelerator registers into the operation
			 * state
			 */
			tf_digest_save_registers(sha_state);

			if (cpu_is_omap54xx()
				&& IS_DIGEST_GREATER_THAN_SHA256(
				DIGEST_MODE_GET_OMAP54XX_ALGO(
					sha_state->CTRL))) {
					sha_state->bytes_processed = INREG32(
					&sha1_md5_reg->SHA512_DIGEST_COUNT);
			} else {
				sha_state->bytes_processed = INREG32(
					&sha1_md5_reg->SHA_DIGEST_COUNT);
			}
			/*We have flushed the chunk so it is empty now */
			sha_state->chunk_length = 0;

			/*Then we have less data to process */
			data += vLengthToComplete;
			data_length -= vLengthToComplete;
		}

		/*(2)We process all the 64B buffer that we can */
		if (sha_state->chunk_length + data_length >=
			BlockLength) {

			while (data_length > BlockLength) {
				u8 pTempAlignedBuffer[BlockLength];

				/*
				 *We process a 64B buffer
				 */
				/*We copy the data to process to an aligned
				 *buffer */
				if (tf_cpy_from(
					pTempAlignedBuffer,
					data,
					BlockLength,
					buffer_origin))
					return false;

				/*Then we send this buffer to the hash
				 *hardware */
				tf_digest_restore_registers(sha_state);
				tf_digest_hw_perform_64b(
					(u32 *) pTempAlignedBuffer,
					sha_state->CTRL,
					sha_state->bytes_processed);
				tf_digest_save_registers(sha_state);

				if (cpu_is_omap54xx()
					&& IS_DIGEST_GREATER_THAN_SHA256(
						DIGEST_MODE_GET_OMAP54XX_ALGO(
							sha_state->CTRL))) {
					sha_state->bytes_processed = INREG32(
						&sha1_md5_reg->
							SHA512_DIGEST_COUNT);
				} else {
					sha_state->bytes_processed = INREG32(
						&sha1_md5_reg->
							SHA_DIGEST_COUNT);
				}

				/*Then we decrease the remaining data of 64B */
				data += BlockLength;
				data_length -= BlockLength;
			}
		}

		/*(3)We look if we have some data that could not be processed
		 *yet because it is not large enough to fill a buffer of 64B */
		if (data_length > 0) {
			if (sha_state->chunk_length + data_length >
					BlockLength) {
				/* Should never be in this case !!! */
			panic("%s: chunk_length data_length > BlockLength\n",
				__func__);
			}

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (tf_cpy_from(
				sha_state->chunk_buffer+sha_state->chunk_length,
				data,
				data_length,
				buffer_origin))
				return false;
			sha_state->chunk_length += data_length;
		}
	}

	dpr_info("%s: Done: Chunk=%u; Processed=%u\n",
		__func__, sha_state->chunk_length, sha_state->bytes_processed);

	return true;
}

/*------------------------------------------------------------------------- */

static void tf_digest_hw_perform_64b(u32 *data,
					u32 algo, u32 bytes_processed)
{
	u32 algo_constant = 0;


	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(algo)))
		OUTREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT, bytes_processed);
	else
		OUTREG32(&sha1_md5_reg->SHA_DIGEST_COUNT, bytes_processed);

	if (bytes_processed == 0) {
		/* No bytes processed so far. Will use the algo constant instead
			of previous digest */
		algo_constant = 1 << 3;
	}

	if (cpu_is_omap54xx()) {
		if (IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
			OUTREG32(&sha1_md5_reg->SHA512_MODE,
				algo_constant | (algo & 0x7));
			OUTREG32(&sha1_md5_reg->SHA512_LENGTH,
				HASH_BLOCK_BYTES_LENGTH_128);
		} else {
			OUTREG32(&sha1_md5_reg->SHA_MODE,
				algo_constant | (algo & 0x7));
			OUTREG32(&sha1_md5_reg->SHA_LENGTH,
				HASH_BLOCK_BYTES_LENGTH);
		}
	} else {
		OUTREG32(&sha1_md5_reg->SHA_MODE,
			algo_constant | (algo & 0x6));
		OUTREG32(&sha1_md5_reg->SHA_LENGTH, HASH_BLOCK_BYTES_LENGTH);
	}

	if (tf_crypto_wait_for_ready_bit(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_INPUT_READY_BIT)
			!= PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/* Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator" \
			"Input data to be ready\n");
	}

	/*
	 *The data buffer is a buffer of 64 or 128 bytes.
	 */
	OUTREG32(&sha1_md5_reg->SHA_DIN_0, data[0]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_1, data[1]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_2, data[2]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_3, data[3]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_4, data[4]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_5, data[5]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_6, data[6]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_7, data[7]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_8, data[8]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_9, data[9]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_10, data[10]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_11, data[11]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_12, data[12]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_13, data[13]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_14, data[14]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_15, data[15]);
	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
		OUTREG32(&sha1_md5_reg->SHA_DIN_16, data[16]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_17, data[17]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_18, data[18]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_19, data[19]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_20, data[20]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_21, data[21]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_22, data[22]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_23, data[23]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_24, data[24]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_25, data[25]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_26, data[26]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_27, data[27]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_28, data[28]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_29, data[29]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_30, data[30]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_31, data[31]);
	}
	/*
	 *Wait until the hash operation is finished.
	 */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_OUTPUT_READY_BIT);
}

/*------------------------------------------------------------------------- */

static bool tf_digest_hw_perform_dma(u8 *data, u32 nDataLength,
			u32 algo, u32 bytes_processed,
			unsigned int buffer_origin)
{
	/*
	 *Note: The DMA only sees physical addresses !
	 */

	int dma_ch0;
	struct omap_dma_channel_params ch0_parameters;
	u32 length_loop = 0;
	u32 algo_constant;
	u8 *local_buf = NULL;
	dma_addr_t local_buf_phys;
	struct tf_device *dev = tf_get_device();
	bool ret = true;
	u32 BlockLength = HASH_BLOCK_BYTES_LENGTH;
	u32 DmaEltsPerFrame = DMA_CEN_Elts_per_Frame_SHA;


	dpr_info(
		"%s: Buffer=0x%08x/%u\n",
		__func__, (u32)data, (u32)nDataLength);

	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
		BlockLength = HASH_BLOCK_BYTES_LENGTH_128;
		DmaEltsPerFrame = DMA_CEN_Elts_per_Frame_Big_SHA;
	}


	/*lock the DMA */
	if (!mutex_trylock(&dev->sm.dma_mutex)) {
		local_buf = dma_alloc_coherent(NULL, dev->dma_buffer_length,
			&local_buf_phys, GFP_ATOMIC);
		if (local_buf == NULL) {
			printk(KERN_ERR
				"%s: could not allocate a temporary DMA buf\n",
				__func__);
			return false;
		}
	} else {
		local_buf = dev->dma_buffer;
		local_buf_phys = dev->dma_buffer_phys;
	}

	if (tf_dma_request(&dma_ch0) != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		ret = false;
		goto exit;
	}

	while (nDataLength > 0) {
		algo_constant = 0;
		if (bytes_processed == 0) {
			/*No bytes processed so far. Will use the algo
			 *constant instead of previous digest */
			algo_constant = 1 << 3;
		}

		/*check length */
		if (nDataLength <= dev->dma_buffer_length)
			length_loop = nDataLength;
		else
			length_loop = dev->dma_buffer_length;

		/*
		 * Copy the data from the user input buffer into a preallocated
		 * buffer which has correct properties from efficient DMA
		 * transfers.
		 */
		if (tf_cpy_from(local_buf, data, length_loop, buffer_origin)) {
			omap_free_dma(dma_ch0);
			ret = false;
			goto exit;
		}

		/*DMA1: Mem -> HASH */
		tf_dma_set_channel_common_params(&ch0_parameters,
			length_loop / BlockLength,
			DmaEltsPerFrame,
			DIGEST1_REGS_HW_ADDR + 0x80,
			local_buf_phys,
			OMAP44XX_DMA_SHA2_DIN_P);

		/*specific for Mem -> HWA */
		ch0_parameters.src_amode = OMAP_DMA_AMODE_POST_INC;
		ch0_parameters.dst_amode = OMAP_DMA_AMODE_CONSTANT;
		ch0_parameters.src_or_dst_synch = OMAP_DMA_DST_SYNC;

		omap_set_dma_params(dma_ch0, &ch0_parameters);

		omap_set_dma_src_burst_mode(dma_ch0, OMAP_DMA_DATA_BURST_16);
		omap_set_dma_dest_burst_mode(dma_ch0, OMAP_DMA_DATA_BURST_16);

		if (cpu_is_omap54xx()) {
			if (IS_DIGEST_GREATER_THAN_SHA256(
				DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
				OUTREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT,
					bytes_processed);
				OUTREG32(&sha1_md5_reg->SHA512_MODE,
					algo_constant | (algo & 0x7));
			} else {
				OUTREG32(&sha1_md5_reg->SHA_DIGEST_COUNT,
					bytes_processed);
				OUTREG32(&sha1_md5_reg->SHA_MODE,
					algo_constant | (algo & 0x7));
			}
		} else {
			OUTREG32(&sha1_md5_reg->SHA_DIGEST_COUNT,
				bytes_processed);
			OUTREG32(&sha1_md5_reg->SHA_MODE,
				algo_constant | (algo & 0x6));
		}

		/*
		 * Triggers operation
		 * Interrupt, Free Running + GO (DMA on)
		 */
		OUTREG32(&sha1_md5_reg->SYSCONFIG,
			INREG32(&sha1_md5_reg->SYSCONFIG) |
			DIGEST_SYSCONFIG_PDMA_EN_BIT);

		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(
				DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
			OUTREG32(&sha1_md5_reg->SHA512_LENGTH, length_loop);
		} else {
			OUTREG32(&sha1_md5_reg->SHA_LENGTH, length_loop);
		}

		wmb();

		tf_dma_start(dma_ch0, OMAP_DMA_BLOCK_IRQ);

		tf_dma_wait(1);

		OUTREG32(&sha1_md5_reg->SYSCONFIG, 0);

		omap_clear_dma(dma_ch0);

		data += length_loop;
		nDataLength -= length_loop;

		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(
				DIGEST_MODE_GET_OMAP54XX_ALGO(algo))) {
			bytes_processed =
				INREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT);
		} else {
			bytes_processed =
				INREG32(&sha1_md5_reg->SHA_DIGEST_COUNT);
		}
	}

	/*For safety reasons, let's clean the working buffer */
	memset(local_buf, 0, length_loop);

	/*release the DMA */
	omap_free_dma(dma_ch0);

	/*
	 * The dma transfert is finished, now wait until the hash
	 * operation is finished.
	 */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_CONTEXT_READY_BIT);

exit:
	if (dev->dma_buffer == local_buf)
		mutex_unlock(&dev->sm.dma_mutex);
	else
		dma_free_coherent(NULL, dev->dma_buffer_length,
			local_buf, local_buf_phys);

	return ret;
}

/*------------------------------------------------------------------------- */
/*
 *Static function, perform data digest using the DMA for data transfer.
 *
 *inputs:
 *        data : pointer of the input data to process
 *        data_length : number of byte to process
 */
static bool tf_digest_update_dma(
	struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length, unsigned int buffer_origin)
{
u32 BlockLength = HASH_BLOCK_BYTES_LENGTH;

	dpr_info("%s\n", __func__);

	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
		DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
		BlockLength = HASH_BLOCK_BYTES_LENGTH_128;
	}

	if (sha_state->chunk_length != 0) {

		u32 vLengthToComplete;

		/*Fill the chunk first */
		if (sha_state->
			chunk_length + data_length <= BlockLength) {

			/*So we fill the chunk buffer with the new data */
			if (tf_cpy_from(sha_state->chunk_buffer +
					sha_state->chunk_length, data,
					data_length, buffer_origin))
				return false;
			sha_state->chunk_length += data_length;

			/*We'll keep some data for the final */
			return true;
		}

		vLengthToComplete = BlockLength - sha_state->chunk_length;

		if (vLengthToComplete != 0) {
			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (tf_cpy_from(sha_state->chunk_buffer +
					sha_state->chunk_length, data,
					vLengthToComplete, buffer_origin))
				return false;
		}

		/*Then we send this buffer to the HWA (no DMA) */
		tf_digest_hw_perform_64b(
			(u32 *)sha_state->chunk_buffer, sha_state->CTRL,
			sha_state->bytes_processed);

		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
			sha_state->bytes_processed =
				INREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT);
		} else {
			sha_state->bytes_processed =
				INREG32(&sha1_md5_reg->SHA_DIGEST_COUNT);
		}

		/*We have flushed the chunk so it is empty now */
		sha_state->chunk_length = 0;

		/*Update the data buffer depending of the data already
		 *processed */
		data += vLengthToComplete;
		data_length -= vLengthToComplete;
	}

	if (data_length > BlockLength) {

		/*DMA only manages data length that is multiple of 64b */
		u32 vDmaProcessize = data_length & 0xFFFFFFC0;

		if (vDmaProcessize == data_length) {
			/*We keep one block for the final */
			vDmaProcessize -= BlockLength;
		}

		if (!tf_digest_hw_perform_dma(data, vDmaProcessize,
				sha_state->CTRL, sha_state->bytes_processed,
				buffer_origin))
			return false;

		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(sha_state->CTRL))) {
			sha_state->bytes_processed =
				INREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT);
		} else {
			sha_state->bytes_processed =
				INREG32(&sha1_md5_reg->SHA_DIGEST_COUNT);
		}

		data += vDmaProcessize;
		data_length -= vDmaProcessize;
	}

	/*At that point, there is less than 64b left to process*/
	if ((data_length == 0) || (data_length > BlockLength))
		/*Should never be in this case !!! */
		return false;

	/*We now fill the chunk buffer with the remaining data */
	if (tf_cpy_from(
		sha_state->chunk_buffer, data, data_length, buffer_origin))
		return false;
	sha_state->chunk_length = data_length;

	return true;
}

static void tf_digest_init_operation(u32 alg,
	struct tf_crypto_sha_operation_state *state)
{
	memset(state, 0, sizeof(struct tf_crypto_sha_operation_state));

	if (cpu_is_omap54xx())
		state->CTRL = alg;
	else
		state->CTRL = alg << 1;
}

static int static_Hash_HwReadDigest(u32 algo, u8 *out)
{
	u32 regs, tmp;
	u32 idx = 0, i;

	switch (algo) {
	case DIGEST_CTRL_ALGO_MD5:
		regs = 4;
	break;
	case DIGEST_CTRL_ALGO_SHA1:
		regs = 5;
	break;
	case DIGEST_CTRL_ALGO_SHA224:
		regs = 7;
	break;
	case DIGEST_CTRL_ALGO_SHA256:
		regs = 8;
	break;
	case DIGEST_CTRL_ALGO_SHA384:
		regs = 12;
	break;
	case DIGEST_CTRL_ALGO_SHA512:
		regs = 16;
	break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < regs; i++) {
		if (cpu_is_omap54xx()
			&& IS_DIGEST_GREATER_THAN_SHA256(algo)) {
			tmp = INREG32(&sha1_md5_reg->SHA512_IDIGEST_A + i);
		} else {
			tmp = INREG32(&sha1_md5_reg->SHA_IDIGEST_A + i);
		}

		out[idx++] = (u8) ((tmp >>  0) & 0xff);
		out[idx++] = (u8) ((tmp >>  8) & 0xff);
		out[idx++] = (u8) ((tmp >> 16) & 0xff);
		out[idx++] = (u8) ((tmp >> 24) & 0xff);
	}

#ifdef CONFIG_TF_DRIVER_FAULT_INJECTION
#define FAULTY(mask, ctrl_algo, alg_name) \
	(((mask) & TF_CRYPTO_ALG_##alg_name) && \
	 (ctrl_algo) == DIGEST_CTRL_ALGO_##alg_name)
	if (cpu_is_omap54xx()) {
		if (FAULTY(tf_fault_injection_mask, algo, MD5) ||
			FAULTY(tf_fault_injection_mask, algo, SHA1) ||
			FAULTY(tf_fault_injection_mask, algo, SHA224) ||
			FAULTY(tf_fault_injection_mask, algo, SHA256) ||
			FAULTY(tf_fault_injection_mask, algo, SHA384) ||
			FAULTY(tf_fault_injection_mask, algo, SHA512)) {
			pr_notice("TF: injecting fault in digest!\n");
			out[0] = 0xff;
			out[1] ^= 0xff;
		} else {
			dpr_info("%s: no fault (mask=0x%x algo=%u)\n",
				__func__, tf_fault_injection_mask, algo);
	  }
	} else {

		if (FAULTY(tf_fault_injection_mask, algo, MD5) ||
			FAULTY(tf_fault_injection_mask, algo, SHA1) ||
			FAULTY(tf_fault_injection_mask, algo, SHA224) ||
			FAULTY(tf_fault_injection_mask, algo, SHA256)) {
			pr_notice("TF: injecting fault in digest!\n");
			out[0] = 0xff;
			out[1] ^= 0xff;
		} else {
			dpr_info("%s: no fault (mask=0x%x algo=%u)\n",
				__func__, tf_fault_injection_mask, algo);
		}
	}
#undef FAULTY
#endif /* CONFIG_TF_DRIVER_FAULT_INJECTION */
	return 0;
}

static int tf_digest_final(struct tf_crypto_sha_operation_state *state,
	u8 *out)
{
	u32 *data = (u32 *) state->chunk_buffer;

	/* Hashing an empty string? */
	if (state->bytes_processed + state->chunk_length == 0) {
		switch (DIGEST_MODE_GET_ALGO(state->CTRL)) {
		case DIGEST_CTRL_ALGO_MD5:
			memcpy(out, md5OverEmptyString, HASH_MD5_LENGTH);
		break;
		case DIGEST_CTRL_ALGO_SHA1:
			memcpy(out, sha1OverEmptyString, HASH_SHA1_LENGTH);
		break;
		case DIGEST_CTRL_ALGO_SHA224:
			memcpy(out, sha224OverEmptyString, HASH_SHA224_LENGTH);
		break;
		case DIGEST_CTRL_ALGO_SHA256:
			memcpy(out, sha256OverEmptyString, HASH_SHA256_LENGTH);
		break;
		case DIGEST_CTRL_ALGO_SHA384:
			memcpy(out, sha384OverEmptyString, HASH_SHA384_LENGTH);
		break;
		case DIGEST_CTRL_ALGO_SHA512:
			memcpy(out, sha512OverEmptyString, HASH_SHA512_LENGTH);
		break;
		default:
			return -EINVAL;
		}
		return 0;
	}

	tf_digest_restore_registers(state);

	/*
	 * At this point, the chunk buffer should contain the last block of data
	 * needed for the final.
	 */

	if (cpu_is_omap54xx()) {
		if (IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(state->CTRL))) {
			OUTREG32(&sha1_md5_reg->SHA512_DIGEST_COUNT,
				state->bytes_processed);
			OUTREG32(&sha1_md5_reg->SHA512_MODE,
				(state->CTRL & 0x7) | 0x10 |
				(state->bytes_processed == 0) << 3);
			OUTREG32(&sha1_md5_reg->SHA512_LENGTH,
				state->chunk_length);
		} else {
			OUTREG32(&sha1_md5_reg->SHA_DIGEST_COUNT,
				state->bytes_processed);
			OUTREG32(&sha1_md5_reg->SHA_MODE,
				(state->CTRL & 0x7) | 0x10 |
				(state->bytes_processed == 0) << 3);
			OUTREG32(&sha1_md5_reg->SHA_LENGTH,
				state->chunk_length);
		}
	} else {
		OUTREG32(&sha1_md5_reg->SHA_DIGEST_COUNT,
			state->bytes_processed);
		OUTREG32(&sha1_md5_reg->SHA_MODE,
			(state->CTRL & 0x6) | 0x10 |
			(state->bytes_processed == 0) << 3);
		OUTREG32(&sha1_md5_reg->SHA_LENGTH, state->chunk_length);
	}

	if (tf_crypto_wait_for_ready_bit(
		(u32 *) &sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_INPUT_READY_BIT)
			!= PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/* Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator\n");
	}

	OUTREG32(&sha1_md5_reg->SHA_DIN_0, data[0]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_1, data[1]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_2, data[2]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_3, data[3]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_4, data[4]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_5, data[5]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_6, data[6]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_7, data[7]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_8, data[8]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_9, data[9]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_10, data[10]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_11, data[11]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_12, data[12]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_13, data[13]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_14, data[14]);
	OUTREG32(&sha1_md5_reg->SHA_DIN_15, data[15]);
	if (cpu_is_omap54xx()
		&& IS_DIGEST_GREATER_THAN_SHA256(
			DIGEST_MODE_GET_OMAP54XX_ALGO(state->CTRL))) {
		OUTREG32(&sha1_md5_reg->SHA_DIN_16, data[16]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_17, data[17]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_18, data[18]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_19, data[19]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_20, data[20]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_21, data[21]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_22, data[22]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_23, data[23]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_24, data[24]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_25, data[25]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_26, data[26]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_27, data[27]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_28, data[28]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_29, data[29]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_30, data[30]);
		OUTREG32(&sha1_md5_reg->SHA_DIN_31, data[31]);
	}

	/* Wait till the hash operation is finished */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *) &sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_OUTPUT_READY_BIT);

	return static_Hash_HwReadDigest(DIGEST_MODE_GET_ALGO(
		state->CTRL), out);
}

/*
 * Digest HWA registration into kernel crypto framework
 */

static DEFINE_SPINLOCK(digest_lock);

static int digest_update(struct shash_desc *desc, const u8 *data,
	unsigned int len)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_crypto_enable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	spin_lock(&digest_lock);
	tf_digest_update(state, (u8 *) data, len, TF_BUFFER_KERNEL);
	spin_unlock(&digest_lock);

	tf_crypto_disable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	return 0;
}

static int digest_final(struct shash_desc *desc, u8 *out)
{
	int ret;
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_crypto_enable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	spin_lock(&digest_lock);
	ret = tf_digest_final(state, out);
	spin_unlock(&digest_lock);

	tf_crypto_disable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	return ret;
}

static int digest_import(struct shash_desc *desc, const void *in)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	memcpy(state, in, sizeof(*state));
	return 0;
}

static int digest_export(struct shash_desc *desc, void *out)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	memcpy(out, state, sizeof(*state));
	return 0;
}

/* MD5 */
static int md5_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_MD5, state);

	return 0;
}

static struct shash_alg smc_md5_alg = {
	.digestsize	= HASH_MD5_LENGTH,
	.init		= md5_init,
	.update		= digest_update,
	.final		= digest_final,
	.export		= digest_export,
	.import		= digest_import,
	.descsize	= sizeof(struct tf_crypto_sha_operation_state),
	.statesize	= sizeof(struct tf_crypto_sha_operation_state),
	.base		= {
		.cra_name		= "md5",
		.cra_driver_name	= "md5-smc",
		.cra_flags		= CRYPTO_ALG_TYPE_SHASH,
		.cra_priority		= 999,
		.cra_blocksize		= HASH_BLOCK_BYTES_LENGTH,
		.cra_module		= THIS_MODULE,
	}
};

/* SHA1 */
static int sha1_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_SHA1, state);

	return 0;
}

static struct shash_alg smc_sha1_alg = {
	.digestsize	= HASH_SHA1_LENGTH,
	.init		= sha1_init,
	.update		= digest_update,
	.final		= digest_final,
	.export		= digest_export,
	.import		= digest_import,
	.descsize	= sizeof(struct tf_crypto_sha_operation_state),
	.statesize	= sizeof(struct tf_crypto_sha_operation_state),
	.base		= {
		.cra_name		= "sha1",
		.cra_driver_name	= "sha1-smc",
		.cra_flags		= CRYPTO_ALG_TYPE_SHASH,
		.cra_priority		= 999,
		.cra_blocksize		= HASH_BLOCK_BYTES_LENGTH,
		.cra_module		= THIS_MODULE,
	}
};

/* SHA224 */
static int sha224_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_SHA224, state);

	return 0;
}

static struct shash_alg smc_sha224_alg = {
	.digestsize	= HASH_SHA224_LENGTH,
	.init		= sha224_init,
	.update		= digest_update,
	.final		= digest_final,
	.export		= digest_export,
	.import		= digest_import,
	.descsize	= sizeof(struct tf_crypto_sha_operation_state),
	.statesize	= sizeof(struct tf_crypto_sha_operation_state),
	.base		= {
		.cra_name		= "sha224",
		.cra_driver_name	= "sha224-smc",
		.cra_flags		= CRYPTO_ALG_TYPE_SHASH,
		.cra_priority		= 999,
		.cra_blocksize		= HASH_BLOCK_BYTES_LENGTH,
		.cra_module		= THIS_MODULE,
	}
};

/* SHA256 */
static int sha256_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_SHA256, state);

	return 0;
}

static struct shash_alg smc_sha256_alg = {
	.digestsize	= HASH_SHA256_LENGTH,
	.init		= sha256_init,
	.update		= digest_update,
	.final		= digest_final,
	.export		= digest_export,
	.import		= digest_import,
	.descsize	= sizeof(struct tf_crypto_sha_operation_state),
	.statesize	= sizeof(struct tf_crypto_sha_operation_state),
	.base		= {
		.cra_name		= "sha256",
		.cra_driver_name	= "sha256-smc",
		.cra_flags		= CRYPTO_ALG_TYPE_SHASH,
		.cra_priority		= 999,
		.cra_blocksize		= HASH_BLOCK_BYTES_LENGTH,
		.cra_module		= THIS_MODULE,
	}
};

/* SHA384 */
static int sha384_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_SHA384, state);

	return 0;
}

static struct shash_alg smc_sha384_alg = {
	.digestsize     = HASH_SHA384_LENGTH,
	.init           = sha384_init,
	.update         = digest_update,
	.final          = digest_final,
	.export         = digest_export,
	.import         = digest_import,
	.descsize       = sizeof(struct tf_crypto_sha_operation_state),
	.statesize      = sizeof(struct tf_crypto_sha_operation_state),
	.base           = {
		.cra_name               = "sha384",
		.cra_driver_name        = "sha384-smc",
		.cra_flags              = CRYPTO_ALG_TYPE_SHASH,
		.cra_priority           = 999,
		.cra_blocksize          = HASH_BLOCK_BYTES_LENGTH_128,
		.cra_module             = THIS_MODULE,
	}
};

/* SHA512 */
static int sha512_init(struct shash_desc *desc)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	tf_digest_init_operation(DIGEST_CTRL_ALGO_SHA512, state);

	return 0;
}

static struct shash_alg smc_sha512_alg = {
	.digestsize     = HASH_SHA512_LENGTH,
	.init           = sha512_init,
	.update         = digest_update,
	.final          = digest_final,
	.export         = digest_export,
	.import         = digest_import,
	.descsize       = sizeof(struct tf_crypto_sha_operation_state),
	.statesize      = sizeof(struct tf_crypto_sha_operation_state),
	.base           = {
		.cra_name               = "sha512",
		.cra_driver_name        = "sha512-smc",
		.cra_flags              = CRYPTO_ALG_TYPE_SHASH,
		.cra_priority           = 999,
		.cra_blocksize          = HASH_BLOCK_BYTES_LENGTH_128,
		.cra_module             = THIS_MODULE,
	}
};


int register_smc_public_crypto_digest(void)
{
	int ret;

	dpr_info("SMC: Registering digest algorithms\n");

	sha1_md5_reg = ioremap(DIGEST1_REGS_HW_ADDR, SZ_1M);
	if (sha1_md5_reg == NULL)
		return -EFAULT;

	ret = crypto_register_shash(&smc_md5_alg);
	if (ret)
		goto md5_err;

	ret = crypto_register_shash(&smc_sha1_alg);
	if (ret)
		goto sha1_err;

	ret = crypto_register_shash(&smc_sha224_alg);
	if (ret)
		goto sha224_err;

	ret = crypto_register_shash(&smc_sha256_alg);
	if (ret)
		goto sha256_err;

	if (cpu_is_omap54xx()) {
		ret = crypto_register_shash(&smc_sha384_alg);
		if (ret)
			goto sha384_err;

		ret = crypto_register_shash(&smc_sha512_alg);
		if (ret)
			goto sha512_err;
	}
	return 0;

md5_err:
	iounmap(sha1_md5_reg);
sha512_err:
	if (cpu_is_omap54xx())
		crypto_unregister_shash(&smc_sha384_alg);
sha384_err:
	if (cpu_is_omap54xx())
		crypto_unregister_shash(&smc_sha256_alg);
sha256_err:
	crypto_unregister_shash(&smc_sha224_alg);
sha224_err:
	crypto_unregister_shash(&smc_sha1_alg);
sha1_err:
	crypto_unregister_shash(&smc_md5_alg);
	return ret;
}

void unregister_smc_public_crypto_digest(void)
{
	dpr_info("SMC: Unregistering digest algorithms\n");

	crypto_unregister_shash(&smc_md5_alg);
	crypto_unregister_shash(&smc_sha1_alg);
	crypto_unregister_shash(&smc_sha224_alg);
	crypto_unregister_shash(&smc_sha256_alg);
	if (cpu_is_omap54xx()) {
		crypto_unregister_shash(&smc_sha384_alg);
		crypto_unregister_shash(&smc_sha512_alg);
	}

	iounmap(sha1_md5_reg);
}
