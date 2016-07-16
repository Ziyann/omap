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

#ifndef __TF_PUBLIC_CRYPTO_H
#define __TF_PUBLIC_CRYPTO_H

#include "tf_defs.h"
#include <linux/io.h>

#include <clockdomain.h>

#ifdef CONFIG_TF_DRIVER_CRYPTO_FIPS
#include <linux/sysdev.h>
#endif

#ifdef __ASM_ARM_ARCH_OMAP_CLOCKDOMAIN_H
#define clkdm_wakeup omap2_clkdm_wakeup
#define clkdm_allow_idle omap2_clkdm_allow_idle
#endif

/*-------------------------------------------------------------------------- */

#define PUBLIC_CRYPTO_HWA_AES1		0x1
#define PUBLIC_CRYPTO_HWA_DES		0x4
#define PUBLIC_CRYPTO_HWA_SHA		0x8

#define OUTREG32(a, b)	__raw_writel(b, a)
#define INREG32(a)	__raw_readl(a)
#define SETREG32(x, y)	OUTREG32(x, INREG32(x) | (y))
#define CLRREG32(x, y)	OUTREG32(x, INREG32(x) & ~(y))

#define PUBLIC_CRYPTO_CLKSTCTRL_CLOCK_REG	0x4A009580
#define PUBLIC_CRYPTO_AES1_CLOCK_REG		0x4A0095A0
#define PUBLIC_CRYPTO_DES3DES_CLOCK_REG		0x4A0095B0
#define PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG		0x4A0095C8

#define BYTES_TO_LONG(a)(u32)(a[0] | (a[1]<<8) | (a[2]<<16) | (a[3]<<24))
#define LONG_TO_BYTE(a, b) {  a[0] = (u8)((b) & 0xFF);		 \
				a[1] = (u8)(((b) >> 8) & 0xFF);  \
				a[2] = (u8)(((b) >> 16) & 0xFF); \
				a[3] = (u8)(((b) >> 24) & 0xFF); }

#define IS_4_BYTES_ALIGNED(x)((!((x) & 0x3)) ? true : false)

#define TF_SMC_OMAP4_PUBLIC_DMA

/*
 *The size limit to trigger DMA for AES, DES and Digest.
 *0xFFFFFFFF means "never"
 */
#ifdef TF_SMC_OMAP4_PUBLIC_DMA
#define DMA_TRIGGER_IRQ_AES				128
#define DMA_TRIGGER_IRQ_DES				128
#define DMA_TRIGGER_IRQ_DIGEST				1024
#else
#define DMA_TRIGGER_IRQ_AES				0xFFFFFFFF
#define DMA_TRIGGER_IRQ_DES				0xFFFFFFFF
#define DMA_TRIGGER_IRQ_DIGEST				0xFFFFFFFF
#endif

/*Error code constants */
#define PUBLIC_CRYPTO_OPERATION_SUCCESS	0x00000000
#define PUBLIC_CRYPTO_ERR_ACCESS_DENIED	0x00000001
#define PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY	0x00000002
#define PUBLIC_CRYPTO_ERR_BAD_PARAMETERS	0x00000003
#define PUBLIC_CRYPTO_ERR_TIMEOUT		0x00000004

/*DMA mode constants */
#define PUBLIC_CRYPTO_DMA_USE_NONE	0x00000000	/*No DMA used*/
/*DMA with active polling used */
#define PUBLIC_CRYPTO_DMA_USE_POLLING	0x00000001
#define PUBLIC_CRYPTO_DMA_USE_IRQ	0x00000002	/*DMA with IRQ used*/

#define PUBLIC_CRYPTO_REG_SET_BIT(x, y)	OUTREG32(x, INREG32(x) | y);
#define PUBLIC_CRYPTO_REG_UNSET_BIT(x, y)	OUTREG32(x, INREG32(x) & (~y));

#define AES_BLOCK_SIZE			16
#define DES_BLOCK_SIZE			8
#define HASH_BLOCK_SIZE			64

#define HASH_MD5_LENGTH			16
#define HASH_SHA1_LENGTH		20
#define HASH_SHA224_LENGTH		28
#define HASH_SHA256_LENGTH		32
#define HASH_SHA384_LENGTH		48
#define HASH_SHA512_LENGTH		64

#define PUBLIC_CRYPTO_DIGEST_MAX_SIZE   64
#define PUBLIC_CRYPTO_IV_MAX_SIZE	16

#define PUBLIC_CRYPTO_HW_CLOCK_ADDR		(0x48004A14)
#define PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR		(0x48004A34)

#define PUBLIC_CRYPTO_HW_CLOCK1_ADDR		(0x48004A10)
#define PUBLIC_CRYPTO_HW_AUTOIDLE1_ADDR	(0x48004A30)

#ifdef CONFIG_ARCH_OMAP5
#define DIGEST_CTRL_ALGO_MD5		0
#define DIGEST_CTRL_ALGO_SHA1		2
#define DIGEST_CTRL_ALGO_SHA224		4
#define DIGEST_CTRL_ALGO_SHA256		6
#define DIGEST_CTRL_ALGO_SHA384		1
#define DIGEST_CTRL_ALGO_SHA512		3
#else
#define DIGEST_CTRL_ALGO_MD5            0
#define DIGEST_CTRL_ALGO_SHA1           1
#define DIGEST_CTRL_ALGO_SHA224         2
#define DIGEST_CTRL_ALGO_SHA256         3
/* unused but must be defined for compilation */
#define DIGEST_CTRL_ALGO_SHA384         0xFF
#define DIGEST_CTRL_ALGO_SHA512         0xFF
#endif

#define IS_DIGEST_GREATER_THAN_SHA256(x)	\
	({					\
		int _y;				\
		switch (x) {			\
		case DIGEST_CTRL_ALGO_SHA384:	\
		case DIGEST_CTRL_ALGO_SHA512:	\
			_y = 1;			\
		break;				\
		default:			\
			_y = 0;			\
		}				\
		_y;				\
	})

/*-------------------------------------------------------------------------- */
/*
 *The magic word.
 */
#define CUS_CONTEXT_MAGIC		0x45EF683C

/*-------------------------------------------------------------------------- */
/* CUS context structure                                                     */
/*-------------------------------------------------------------------------- */

/* State of an AES operation */
struct tf_crypto_aes_operation_state {
	u32 AES_IV_0;
	u32 AES_IV_1;
	u32 AES_IV_2;
	u32 AES_IV_3;

	u32 CTRL;

	u32 KEY1_0;
	u32 KEY1_1;
	u32 KEY1_2;
	u32 KEY1_3;
	u32 KEY1_4;
	u32 KEY1_5;
	u32 KEY1_6;
	u32 KEY1_7;
};

struct tf_crypto_des_operation_state {
	u32 DES_IV_L;
	u32 DES_IV_H;
};

#define HASH_BLOCK_BYTES_LENGTH		64
#define HASH_BLOCK_BYTES_LENGTH_128	128

struct tf_crypto_sha_operation_state {
	/* Current digest */
	u32 SHA_DIGEST_A;
	u32 SHA_DIGEST_B;
	u32 SHA_DIGEST_C;
	u32 SHA_DIGEST_D;
	u32 SHA_DIGEST_E;
	u32 SHA_DIGEST_F;
	u32 SHA_DIGEST_G;
	u32 SHA_DIGEST_H;
	u32 SHA512_DIGEST_I;
	u32 SHA512_DIGEST_J;
	u32 SHA512_DIGEST_K;
	u32 SHA512_DIGEST_L;
	u32 SHA512_DIGEST_M;
	u32 SHA512_DIGEST_N;
	u32 SHA512_DIGEST_O;
	u32 SHA512_DIGEST_P;

	/* This buffer contains a partial chunk */
	u8 chunk_buffer[HASH_BLOCK_BYTES_LENGTH_128];

	/* Number of bytes stored in chunk_buffer (0..64) */
	u32 chunk_length;

	/*
	 * Total number of bytes processed so far
	 * (not including the partial chunk)
	 */
	u32 bytes_processed;

	/*
	 * bit 0      used also for algo on omap5
	 * bits 1-2   algo
	 * bit 3      flag: 1 if no byte processed, 0 else
	 * bit 4      flag: 1 if SHA, 0 if MD5     not used anymore in OMAP5
	 */
	u32 CTRL;
};

union tf_crypto_operation_state {
	struct tf_crypto_aes_operation_state aes;
	struct tf_crypto_des_operation_state des;
	struct tf_crypto_sha_operation_state sha;
};

/*-------------------------------------------------------------------------- */
/*
 *Helper methods
 */
u32 tf_crypto_wait_for_ready_bit(u32 *reg, u32 bit);
void tf_crypto_wait_for_ready_bit_infinitely(u32 *reg, u32 bit);

void tf_crypto_enable_clock(uint32_t clock_paddr);
void tf_crypto_disable_clock(uint32_t clock_paddr);
u32 tf_crypto_turn_off_clocks(void);

void tf_aes_pm_resume(void);

u32 tf_crypto_init(void);
void tf_crypto_terminate(void);

#ifdef CONFIG_SMC_KERNEL_CRYPTO
int register_smc_public_crypto_aes(void);
void unregister_smc_public_crypto_aes(void);
#else
static inline int register_smc_public_crypto_aes(void)
{
	return 0;
}

static inline void unregister_smc_public_crypto_aes(void) {}
#endif

#ifdef CONFIG_SMC_KERNEL_CRYPTO
int register_smc_public_crypto_digest(void);
void unregister_smc_public_crypto_digest(void);
#else
static inline int register_smc_public_crypto_digest(void)
{
	return 0;
}

static inline void unregister_smc_public_crypto_digest(void) {}
#endif

#define TF_BUFFER_USER   0
#define TF_BUFFER_KERNEL 1

#define TF_CRYPTO_ALG_MD5         0x00000001
#define TF_CRYPTO_ALG_SHA1        0x00000002
#define TF_CRYPTO_ALG_SHA224      0x00000004
#define TF_CRYPTO_ALG_SHA256      0x00000008
#define TF_CRYPTO_ALG_AES_ECB_128 0x00000100
#define TF_CRYPTO_ALG_AES_ECB_192 0x00000200
#define TF_CRYPTO_ALG_AES_ECB_256 0x00000400
#define TF_CRYPTO_ALG_AES_CBC_128 0x00000800
#define TF_CRYPTO_ALG_AES_CBC_192 0x00001000
#define TF_CRYPTO_ALG_AES_CBC_256 0x00002000
#define TF_CRYPTO_ALG_AES_CTR_128 0x00004000
#define TF_CRYPTO_ALG_AES_CTR_192 0x00008000
#define TF_CRYPTO_ALG_AES_CTR_256 0x00010000
#define TF_CRYPTO_ALG_HMAC_MD5    0x00100000
#define TF_CRYPTO_ALG_HMAC_SHA1   0x00200000
#define TF_CRYPTO_ALG_HMAC_SHA224 0x00400000
#define TF_CRYPTO_ALG_HMAC_SHA256 0x00800000
#define TF_CRYPTO_ALG_HMAC_ALL    0x00f00000

#ifdef CONFIG_TF_DRIVER_FAULT_INJECTION
extern unsigned tf_fault_injection_mask;
#endif

#ifdef CONFIG_TF_DRIVER_CRYPTO_FIPS

int __init tf_crypto_hmac_module_init(void);
void __exit tf_crypto_hmac_module_exit(void);

int __init tf_self_test_register_device(void);
void __exit tf_self_test_unregister_device(void);

int tf_self_test_post_init(struct kobject *parent);
void tf_self_test_post_exit(void);
unsigned tf_self_test_post_vectors(void);

#define TF_CRYPTO_ALG_INTEGRITY   0x04000000
extern char *tf_integrity_hmac_sha256_expected_value;

#endif /* CONFIG_TF_DRIVER_CRYPTO_FIPS */

#endif /*__TF_PUBLIC_CRYPTO_H */
