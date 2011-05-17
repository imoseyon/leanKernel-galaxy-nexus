/*
 * Copyright (c)2006-2008 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __SCX_PUBLIC_CRYPTO_H
#define __SCX_PUBLIC_CRYPTO_H

#include "scxlnx_defs.h"
#include <linux/io.h>
#include <mach/io.h>

#include <clockdomain.h>

/*-------------------------------------------------------------------------- */

#define PUBLIC_CRYPTO_HWA_AES1		0x1
#define PUBLIC_CRYPTO_HWA_AES2		0x2
#define PUBLIC_CRYPTO_HWA_DES		0x4
#define PUBLIC_CRYPTO_HWA_SHA		0x8

#define OUTREG32(a, b)	__raw_writel(b, a)
#define INREG32(a)	__raw_readl(a)
#define SETREG32(x, y)	OUTREG32(x, INREG32(x) | (y))
#define CLRREG32(x, y)	OUTREG32(x, INREG32(x) & ~(y))

#define PUBLIC_CRYPTO_CLKSTCTRL_CLOCK_REG	0x4A009580
#define PUBLIC_CRYPTO_AES1_CLOCK_REG		0x4A0095A0
#define PUBLIC_CRYPTO_AES2_CLOCK_REG		0x4A0095A8
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
#define CRYPTOKI_UPDATE_SHORTCUT_CONTEXT_MAGIC		0x45EF683C

/*-------------------------------------------------------------------------- */
/* CUS context structure                                                     */
/*-------------------------------------------------------------------------- */

/* State of an AES operation */
struct PUBLIC_CRYPTO_AES_OPERATION_STATE {
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

struct PUBLIC_CRYPTO_DES_OPERATION_STATE {
	u32 DES_IV_L;
	u32 DES_IV_H;
};

#define HASH_BLOCK_BYTES_LENGTH		64

struct PUBLIC_CRYPTO_SHA_OPERATION_STATE {
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
	u8 pChunkBuffer[HASH_BLOCK_BYTES_LENGTH];

	/* Number of bytes stored in pChunkBuffer (0..64) */
	u32 nChunkLength;

	/*
	 * Total number of bytes processed so far
	 * (not including the partial chunk)
	 */
	u32 nBytesProcessed;

	u32 CTRL;
};

union PUBLIC_CRYPTO_OPERATION_STATE {
	struct PUBLIC_CRYPTO_AES_OPERATION_STATE aes;
	struct PUBLIC_CRYPTO_DES_OPERATION_STATE des;
	struct PUBLIC_CRYPTO_SHA_OPERATION_STATE sha;
};

/*
 *Fully describes a public crypto operation
 *(i.e., an operation that has a shortcut attached).
 */
struct CRYPTOKI_UPDATE_SHORTCUT_CONTEXT {
	/*
	 *Identifies the public crypto operation in the list of all public
	 *operations.
	 */
	struct list_head list;

	u32 nMagicNumber;	/*Must be set to
				 *{CRYPTOKI_UPDATE_SHORTCUT_CONTEXT_MAGIC} */

	/*basic fields */
	u32 hClientSession;
	u32 nCommandID;
	u32 nHWAID;
	u32 nHWA_CTRL;
	u32 hKeyContext;
	union PUBLIC_CRYPTO_OPERATION_STATE sOperationState;
	u32 nUseCount;
	bool bSuspended;
};

struct CRYPTOKI_UPDATE_PARAMS {
	/*fields for data processing of an update command */
	u32 nInputDataLength;
	u8 *pInputData;
	struct SCXLNX_SHMEM_DESC *pInputShmem;

	u32 nResultDataLength;
	u8 *pResultData;
	struct SCXLNX_SHMEM_DESC *pOutputShmem;

	u8 *pS2CDataBuffer;
	u32 nS2CDataBufferMaxLength;
};

/*-------------------------------------------------------------------------- */
/*
 *Public crypto API (Top level)
 */

/*
*Initialize the public crypto DMA chanels and global HWA semaphores
 */
u32 SCXPublicCryptoInit(void);

/*
 *Initialize the device context CUS fields
 *(shortcut semaphore and public CUS list)
 */
void SCXPublicCryptoInitDeviceContext(struct SCXLNX_CONNECTION *pDeviceContext);

/**
 *Terminate the public crypto (including DMA)
 */
void SCXPublicCryptoTerminate(void);

int SCXPublicCryptoTryShortcutedUpdate(struct SCXLNX_CONNECTION *pConn,
	struct SCX_COMMAND_INVOKE_CLIENT_COMMAND *pMessage,
	struct SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer);

int SCXPublicCryptoExecuteRPCCommand(u32 nRPCCommand, void *pRPCSharedBuffer);

/*-------------------------------------------------------------------------- */
/*
 *Helper methods
 */
u32 SCXPublicCryptoWaitForReadyBit(u32 *pRegister, u32 vBit);
void SCXPublicCryptoWaitForReadyBitInfinitely(u32 *pRegister, u32 vBit);

void SCXPublicCryptoEnableClock(uint32_t vClockPhysAddr);
void SCXPublicCryptoDisableClock(uint32_t vClockPhysAddr);

#define LOCK_HWA	true
#define UNLOCK_HWA	false

void PDrvCryptoLockUnlockHWA(u32 nHWAID, bool bDoLock);

/*---------------------------------------------------------------------------*/
/*                               AES operations                              */
/*---------------------------------------------------------------------------*/

void PDrvCryptoAESInit(void);
void PDrvCryptoAESExit(void);

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
 *pAESState:	defines the operation IV
 *pSrc:			Input buffer to process.
 *pDest:			Output buffer containing the processed data.
 *
 *nbBlocks number of block(s)to process.
 */
bool PDrvCryptoUpdateAES(struct PUBLIC_CRYPTO_AES_OPERATION_STATE *pAESState,
	u8 *pSrc, u8 *pDest, u32 nbBlocks);

/*---------------------------------------------------------------------------*/
/*                              DES/DES3 operations                          */
/*---------------------------------------------------------------------------*/

void PDrvCryptoDESInit(void);
void PDrvCryptoDESExit(void);

/**
 *This function performs a DES update operation.
 *
 *The DES accelerator is assumed loaded with the correct key
 *
 *DES_CTRL:		defines the mode and direction
 *pDESState:	defines the operation IV
 *pSrc:			Input buffer to process.
 *pDest:			Output buffer containing the processed data.
 *nbBlocks:		Number of block(s)to process.
 */
bool PDrvCryptoUpdateDES(u32 DES_CTRL,
	struct PUBLIC_CRYPTO_DES_OPERATION_STATE *pDESState,
	u8 *pSrc, u8 *pDest, u32 nbBlocks);

/*---------------------------------------------------------------------------*/
/*                               Digest operations                           */
/*---------------------------------------------------------------------------*/

void PDrvCryptoDigestInit(void);
void PDrvCryptoDigestExit(void);

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
 *pSHAState:	State of the operation
 *pData:			Input buffer to process
 *dataLength:	Length in bytes of the input buffer.
 */
void PDrvCryptoUpdateHash(
	struct PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState,
	u8 *pData, u32 dataLength);

#endif /*__SCX_PUBLIC_CRYPTO_H */
