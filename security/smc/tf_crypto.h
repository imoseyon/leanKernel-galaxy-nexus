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
#include <mach/io.h>

#include <clockdomain.h>

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

#define PUBLIC_CRYPTO_DIGEST_MAX_SIZE	32
#define PUBLIC_CRYPTO_IV_MAX_SIZE	16

#define PUBLIC_CRYPTO_HW_CLOCK_ADDR		(0x48004A14)
#define PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR		(0x48004A34)

#define PUBLIC_CRYPTO_HW_CLOCK1_ADDR		(0x48004A10)
#define PUBLIC_CRYPTO_HW_AUTOIDLE1_ADDR	(0x48004A30)

#define DIGEST_CTRL_ALGO_MD5		0
#define DIGEST_CTRL_ALGO_SHA1		1
#define DIGEST_CTRL_ALGO_SHA224		2
#define DIGEST_CTRL_ALGO_SHA256		3

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

	/* Only used by Linux crypto API interface */
	u32 KEY1_0;
	u32 KEY1_1;
	u32 KEY1_2;
	u32 KEY1_3;
	u32 KEY1_4;
	u32 KEY1_5;
	u32 KEY1_6;
	u32 KEY1_7;

	u32 key_is_public;
};

struct tf_crypto_des_operation_state {
	u32 DES_IV_L;
	u32 DES_IV_H;
};

#define HASH_BLOCK_BYTES_LENGTH		64

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

	/* This buffer contains a partial chunk */
	u8 chunk_buffer[HASH_BLOCK_BYTES_LENGTH];

	/* Number of bytes stored in chunk_buffer (0..64) */
	u32 chunk_length;

	/*
	 * Total number of bytes processed so far
	 * (not including the partial chunk)
	 */
	u32 bytes_processed;

	u32 CTRL;
};

union tf_crypto_operation_state {
	struct tf_crypto_aes_operation_state aes;
	struct tf_crypto_des_operation_state des;
	struct tf_crypto_sha_operation_state sha;
};

/*
 *Fully describes a public crypto operation
 *(i.e., an operation that has a shortcut attached).
 */
struct cus_context {
	/*
	 *Identifies the public crypto operation in the list of all public
	 *operations.
	 */
	struct list_head list;

	u32 magic_number;	/*Must be set to
				 *{CUS_CONTEXT_MAGIC} */

	/*basic fields */
	u32 client_session;
	u32 command_id;
	u32 hwa_id;
	u32 hwa_ctrl;
	u32 key_context;
	union tf_crypto_operation_state operation_state;
	u32 use_count;
	bool suspended;
};

struct cus_params {
	/*fields for data processing of an update command */
	u32 input_data_length;
	u8 *input_data;
	struct tf_shmem_desc *input_shmem;

	u32 output_data_length;
	u8 *output_data;
	struct tf_shmem_desc *output_shmem;
};

/*-------------------------------------------------------------------------- */
/*
 *Public crypto API (Top level)
 */

/*
*Initialize the public crypto DMA chanels and global HWA semaphores
 */
u32 tf_crypto_init(void);

/*
 *Initialize the device context CUS fields
 *(shortcut semaphore and public CUS list)
 */
void tf_crypto_init_cus(struct tf_connection *connection);

/**
 *Terminate the public crypto (including DMA)
 */
void tf_crypto_terminate(void);

int tf_crypto_try_shortcuted_update(struct tf_connection *connection,
	struct tf_command_invoke_client_command *command,
	struct tf_answer_invoke_client_command *answer);

int tf_crypto_execute_rpc(u32 rpc_command, void *rpc_shared_buffer);

/*-------------------------------------------------------------------------- */
/*
 *Helper methods
 */
u32 tf_crypto_wait_for_ready_bit(u32 *reg, u32 bit);
void tf_crypto_wait_for_ready_bit_infinitely(u32 *reg, u32 bit);

void tf_crypto_enable_clock(uint32_t clock_paddr);
void tf_crypto_disable_clock(uint32_t clock_paddr);

#define LOCK_HWA	true
#define UNLOCK_HWA	false

void tf_crypto_lock_hwa(u32 hwa_id, bool do_lock);

/*---------------------------------------------------------------------------*/
/*                               AES operations                              */
/*---------------------------------------------------------------------------*/

void tf_aes_init(void);
void tf_aes_exit(void);

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

/**
 *This function performs an AES update operation.
 *
 *The AES1 accelerator is assumed loaded with the correct key
 *
 *AES_CTRL:		defines the mode and direction
 *aes_state:	defines the operation IV
 *src:			Input buffer to process.
 *dest:			Output buffer containing the processed data.
 *
 *nb_blocks number of block(s)to process.
 */
bool tf_aes_update(struct tf_crypto_aes_operation_state *aes_state,
	u8 *src, u8 *dest, u32 nb_blocks);

/*---------------------------------------------------------------------------*/
/*                              DES/DES3 operations                          */
/*---------------------------------------------------------------------------*/

void tf_des_init(void);
void tf_des_exit(void);

/**
 *This function performs a DES update operation.
 *
 *The DES accelerator is assumed loaded with the correct key
 *
 *DES_CTRL:		defines the mode and direction
 *des_state:	defines the operation IV
 *src:			Input buffer to process.
 *dest:			Output buffer containing the processed data.
 *nb_blocks:		Number of block(s)to process.
 */
bool tf_des_update(u32 DES_CTRL,
	struct tf_crypto_des_operation_state *des_state,
	u8 *src, u8 *dest, u32 nb_blocks);

/*---------------------------------------------------------------------------*/
/*                               Digest operations                           */
/*---------------------------------------------------------------------------*/

void tf_digest_init(void);
void tf_digest_exit(void);

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

/**
 *This function performs a HASH update Operation.
 *
 *SHA_CTRL:		defines the algorithm
 *sha_state:	State of the operation
 *data:			Input buffer to process
 *data_length:	Length in bytes of the input buffer.
 */
bool tf_digest_update(
	struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length);

#endif /*__TF_PUBLIC_CRYPTO_H */
