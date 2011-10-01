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
#include <mach/io.h>
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
#define DIGEST_MODE_GET_ALGO(x)			((x & 0x6) >> 1)
#define DIGEST_MODE_SET_ALGO(x, a)		((a << 1) | (x & 0xFFFFFFF9))

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
	u32 ODIGEST_A;		/* 0x00 Outer Digest A      */
	u32 ODIGEST_B;		/* 0x04 Outer Digest B      */
	u32 ODIGEST_C;		/* 0x08 Outer Digest C      */
	u32 ODIGEST_D;		/* 0x0C Outer Digest D      */
	u32 ODIGEST_E;		/* 0x10 Outer Digest E      */
	u32 ODIGEST_F;		/* 0x14 Outer Digest F      */
	u32 ODIGEST_G;		/* 0x18 Outer Digest G      */
	u32 ODIGEST_H;		/* 0x1C Outer Digest H      */
	u32 IDIGEST_A;		/* 0x20 Inner Digest A      */
	u32 IDIGEST_B;		/* 0x24 Inner Digest B      */
	u32 IDIGEST_C;		/* 0x28 Inner Digest C      */
	u32 IDIGEST_D;		/* 0x2C Inner Digest D      */
	u32 IDIGEST_E;		/* 0x30 Inner Digest E      */
	u32 IDIGEST_F;		/* 0x34 Inner Digest F      */
	u32 IDIGEST_G;		/* 0x38 Inner Digest G      */
	u32 IDIGEST_H;		/* 0x3C Inner Digest H      */
	u32 DIGEST_COUNT;	/* 0x40 Digest count        */
	u32 MODE;		/* 0x44 Digest mode         */
	u32 LENGTH;		/* 0x48 Data length         */

	u32 reserved0[13];

	u32 DIN_0;		/* 0x80 Data 0              */
	u32 DIN_1;		/* 0x84 Data 1              */
	u32 DIN_2;		/* 0x88 Data 2              */
	u32 DIN_3;		/* 0x8C Data 3              */
	u32 DIN_4;		/* 0x90 Data 4              */
	u32 DIN_5;		/* 0x94 Data 5              */
	u32 DIN_6;		/* 0x98 Data 6              */
	u32 DIN_7;		/* 0x9C Data 7              */
	u32 DIN_8;		/* 0xA0 Data 8              */
	u32 DIN_9;		/* 0xA4 Data 9              */
	u32 DIN_10;		/* 0xA8 Data 10             */
	u32 DIN_11;		/* 0xAC Data 11             */
	u32 DIN_12;		/* 0xB0 Data 12             */
	u32 DIN_13;		/* 0xB4 Data 13             */
	u32 DIN_14;		/* 0xB8 Data 14             */
	u32 DIN_15;		/* 0xBC Data 15             */

	u32 reserved1[16];

	u32 REVISION;		/* 0x100 Revision           */

	u32 reserved2[3];

	u32 SYSCONFIG;		/* 0x110 Config             */
	u32 SYSSTATUS;		/* 0x114 Status             */
	u32 IRQSTATUS;		/* 0x118 IRQ Status         */
	u32 IRQENABLE;		/* 0x11C IRQ Enable         */
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

/*------------------------------------------------------------------------
 *Forward declarations
 *------------------------------------------------------------------------- */

static void tf_digest_hw_perform_64b(u32 *data,
				u32 algo, u32 bytes_processed);
static bool tf_digest_hw_perform_dma(u8 *data, u32 nDataLength,
				u32 algo, u32 bytes_processed);

static bool tf_digest_update_dma(
	struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length);


/*-------------------------------------------------------------------------
 *Save HWA registers into the specified operation state structure
 *------------------------------------------------------------------------*/
static void tf_digest_save_registers(
	struct tf_crypto_sha_operation_state *sha_state)
{
	dprintk(KERN_INFO "tf_digest_save_registers: State=%p\n",
		sha_state);

	sha_state->SHA_DIGEST_A = INREG32(&sha1_md5_reg->IDIGEST_A);
	sha_state->SHA_DIGEST_B = INREG32(&sha1_md5_reg->IDIGEST_B);
	sha_state->SHA_DIGEST_C = INREG32(&sha1_md5_reg->IDIGEST_C);
	sha_state->SHA_DIGEST_D = INREG32(&sha1_md5_reg->IDIGEST_D);
	sha_state->SHA_DIGEST_E = INREG32(&sha1_md5_reg->IDIGEST_E);
	sha_state->SHA_DIGEST_F = INREG32(&sha1_md5_reg->IDIGEST_F);
	sha_state->SHA_DIGEST_G = INREG32(&sha1_md5_reg->IDIGEST_G);
	sha_state->SHA_DIGEST_H = INREG32(&sha1_md5_reg->IDIGEST_H);
}

/*-------------------------------------------------------------------------
 *Restore the HWA registers from the operation state structure
 *-------------------------------------------------------------------------*/
static void tf_digest_restore_registers(
	struct tf_crypto_sha_operation_state *sha_state)
{
	dprintk(KERN_INFO "tf_digest_restore_registers: State=%p\n",
		sha_state);

	if (sha_state->bytes_processed != 0) {
		/*
		 * Some bytes were already processed. Initialize
		 * previous digest
		 */
		OUTREG32(&sha1_md5_reg->IDIGEST_A, sha_state->SHA_DIGEST_A);
		OUTREG32(&sha1_md5_reg->IDIGEST_B, sha_state->SHA_DIGEST_B);
		OUTREG32(&sha1_md5_reg->IDIGEST_C, sha_state->SHA_DIGEST_C);
		OUTREG32(&sha1_md5_reg->IDIGEST_D, sha_state->SHA_DIGEST_D);
		OUTREG32(&sha1_md5_reg->IDIGEST_E, sha_state->SHA_DIGEST_E);
		OUTREG32(&sha1_md5_reg->IDIGEST_F, sha_state->SHA_DIGEST_F);
		OUTREG32(&sha1_md5_reg->IDIGEST_G, sha_state->SHA_DIGEST_G);
		OUTREG32(&sha1_md5_reg->IDIGEST_H, sha_state->SHA_DIGEST_H);
	}

	OUTREG32(&sha1_md5_reg->SYSCONFIG, 0);
}

/*------------------------------------------------------------------------- */

void tf_digest_init(void)
{
	sha1_md5_reg = omap_ioremap(DIGEST1_REGS_HW_ADDR, SZ_1M, MT_DEVICE);
	if (sha1_md5_reg == NULL)
		panic("Unable to remap SHA2/MD5 module");
}

void tf_digest_exit(void)
{
	omap_iounmap(sha1_md5_reg);
}

bool tf_digest_update(struct tf_crypto_sha_operation_state *sha_state,
	u8 *data, u32 data_length)
{
	u32 dma_use = PUBLIC_CRYPTO_DMA_USE_NONE;

	/*
	 *Choice of the processing type
	 */
	if (data_length >= DMA_TRIGGER_IRQ_DIGEST)
		dma_use = PUBLIC_CRYPTO_DMA_USE_IRQ;

	dprintk(KERN_INFO "tf_digest_update : "\
		"Data=0x%08x/%u, Chunk=%u, Processed=%u, dma_use=0x%08x\n",
		(u32)data, (u32)data_length,
		sha_state->chunk_length, sha_state->bytes_processed,
		dma_use);

	if (data_length == 0) {
		dprintk(KERN_INFO "tf_digest_update: "\
				"Nothing to process\n");
		return true;
	}

	if (dma_use != PUBLIC_CRYPTO_DMA_USE_NONE) {
		/*
		 * Restore the registers of the accelerator from the operation
		 * state
		 */
		tf_digest_restore_registers(sha_state);

		/*perform the updates with DMA */
		if (!tf_digest_update_dma(sha_state, data, data_length))
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
						HASH_BLOCK_BYTES_LENGTH)) {

			u8 vLengthToComplete =
			HASH_BLOCK_BYTES_LENGTH - sha_state->chunk_length;

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (copy_from_user(
				sha_state->chunk_buffer+sha_state->chunk_length,
				data,
				vLengthToComplete))
				return false;

			if (sha_state->chunk_length + data_length ==
				HASH_BLOCK_BYTES_LENGTH) {
				/*We'll keep some data for the final */
				sha_state->chunk_length =
					HASH_BLOCK_BYTES_LENGTH;
				dprintk(KERN_INFO "tf_digest_update: "\
					"Done: Chunk=%u; Processed=%u\n",
					sha_state->chunk_length,
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

			sha_state->bytes_processed =
				INREG32(&sha1_md5_reg->DIGEST_COUNT);

			/*We have flushed the chunk so it is empty now */
			sha_state->chunk_length = 0;

			/*Then we have less data to process */
			data += vLengthToComplete;
			data_length -= vLengthToComplete;
		}

		/*(2)We process all the 64B buffer that we can */
		if (sha_state->chunk_length + data_length >=
					HASH_BLOCK_BYTES_LENGTH) {

			while (data_length > HASH_BLOCK_BYTES_LENGTH) {
				u8 pTempAlignedBuffer[HASH_BLOCK_BYTES_LENGTH];

				/*
				 *We process a 64B buffer
				 */
				/*We copy the data to process to an aligned
				 *buffer */
				if (copy_from_user(
					pTempAlignedBuffer,
					data,
					HASH_BLOCK_BYTES_LENGTH))
					return false;

				/*Then we send this buffer to the hash
				 *hardware */
				tf_digest_restore_registers(sha_state);
				tf_digest_hw_perform_64b(
					(u32 *) pTempAlignedBuffer,
					sha_state->CTRL,
					sha_state->bytes_processed);
				tf_digest_save_registers(sha_state);

				sha_state->bytes_processed =
					INREG32(&sha1_md5_reg->DIGEST_COUNT);

				/*Then we decrease the remaining data of 64B */
				data += HASH_BLOCK_BYTES_LENGTH;
				data_length -= HASH_BLOCK_BYTES_LENGTH;
			}
		}

		/*(3)We look if we have some data that could not be processed
		 *yet because it is not large enough to fill a buffer of 64B */
		if (data_length > 0) {
			if (sha_state->chunk_length + data_length >
					HASH_BLOCK_BYTES_LENGTH) {
				/*Should never be in this case !!! */
			panic("tf_digest_update: chunk_length data_length > "
				"HASH_BLOCK_BYTES_LENGTH\n");
			}

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (copy_from_user(
				sha_state->chunk_buffer+sha_state->chunk_length,
				data,
				data_length))
				return false;
			sha_state->chunk_length += data_length;
		}
	}

	dprintk(KERN_INFO "tf_digest_update: Done: "\
		"Chunk=%u; Processed=%u\n",
		sha_state->chunk_length, sha_state->bytes_processed);

	return true;
}

/*------------------------------------------------------------------------- */

static void tf_digest_hw_perform_64b(u32 *data,
					u32 algo, u32 bytes_processed)
{
	u32 algo_constant = 0;

	OUTREG32(&sha1_md5_reg->DIGEST_COUNT, bytes_processed);

	if (bytes_processed == 0) {
		/* No bytes processed so far. Will use the algo constant instead
			of previous digest */
		algo_constant = 1 << 3;
	}

	OUTREG32(&sha1_md5_reg->MODE,
		algo_constant | (algo & 0x6));
	OUTREG32(&sha1_md5_reg->LENGTH, HASH_BLOCK_BYTES_LENGTH);

	if (tf_crypto_wait_for_ready_bit(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_INPUT_READY_BIT)
			!= PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/* Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator" \
		      "Input data to be ready\n");
	}

	/*
	 *The data buffer is a buffer of 64 bytes.
	 */
	OUTREG32(&sha1_md5_reg->DIN_0, data[0]);
	OUTREG32(&sha1_md5_reg->DIN_1, data[1]);
	OUTREG32(&sha1_md5_reg->DIN_2, data[2]);
	OUTREG32(&sha1_md5_reg->DIN_3, data[3]);
	OUTREG32(&sha1_md5_reg->DIN_4, data[4]);
	OUTREG32(&sha1_md5_reg->DIN_5, data[5]);
	OUTREG32(&sha1_md5_reg->DIN_6, data[6]);
	OUTREG32(&sha1_md5_reg->DIN_7, data[7]);
	OUTREG32(&sha1_md5_reg->DIN_8, data[8]);
	OUTREG32(&sha1_md5_reg->DIN_9, data[9]);
	OUTREG32(&sha1_md5_reg->DIN_10, data[10]);
	OUTREG32(&sha1_md5_reg->DIN_11, data[11]);
	OUTREG32(&sha1_md5_reg->DIN_12, data[12]);
	OUTREG32(&sha1_md5_reg->DIN_13, data[13]);
	OUTREG32(&sha1_md5_reg->DIN_14, data[14]);
	OUTREG32(&sha1_md5_reg->DIN_15, data[15]);

	/*
	 *Wait until the hash operation is finished.
	 */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_OUTPUT_READY_BIT);
}

/*------------------------------------------------------------------------- */

static bool tf_digest_hw_perform_dma(u8 *data, u32 nDataLength,
			u32 algo, u32 bytes_processed)
{
	/*
	 *Note: The DMA only sees physical addresses !
	 */

	int dma_ch0;
	struct omap_dma_channel_params ch0_parameters;
	u32 length_loop = 0;
	u32 algo_constant;
	struct tf_device *dev = tf_get_device();

	dprintk(KERN_INFO
		"tf_digest_hw_perform_dma: Buffer=0x%08x/%u\n",
		(u32)data, (u32)nDataLength);

	/*lock the DMA */
	mutex_lock(&dev->sm.dma_mutex);
	if (tf_dma_request(&dma_ch0) != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		mutex_unlock(&dev->sm.dma_mutex);
		return false;
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
		if (copy_from_user(dev->dma_buffer, data, length_loop)) {
			omap_free_dma(dma_ch0);
			mutex_unlock(&dev->sm.dma_mutex);
			return false;
		}

		/*DMA1: Mem -> HASH */
		tf_dma_set_channel_common_params(&ch0_parameters,
			length_loop / HASH_BLOCK_BYTES_LENGTH,
			DMA_CEN_Elts_per_Frame_SHA,
			DIGEST1_REGS_HW_ADDR + 0x80,
			dev->dma_buffer_phys,
			OMAP44XX_DMA_SHA2_DIN_P);

		/*specific for Mem -> HWA */
		ch0_parameters.src_amode = OMAP_DMA_AMODE_POST_INC;
		ch0_parameters.dst_amode = OMAP_DMA_AMODE_CONSTANT;
		ch0_parameters.src_or_dst_synch = OMAP_DMA_DST_SYNC;

		omap_set_dma_params(dma_ch0, &ch0_parameters);

		omap_set_dma_src_burst_mode(dma_ch0, OMAP_DMA_DATA_BURST_16);
		omap_set_dma_dest_burst_mode(dma_ch0, OMAP_DMA_DATA_BURST_16);

		OUTREG32(&sha1_md5_reg->DIGEST_COUNT, bytes_processed);
		OUTREG32(&sha1_md5_reg->MODE,
			algo_constant | (algo & 0x6));

		/*
		 * Triggers operation
		 * Interrupt, Free Running + GO (DMA on)
		 */
		OUTREG32(&sha1_md5_reg->SYSCONFIG,
			INREG32(&sha1_md5_reg->SYSCONFIG) |
			DIGEST_SYSCONFIG_PDMA_EN_BIT);
		OUTREG32(&sha1_md5_reg->LENGTH, length_loop);

		wmb();

		tf_dma_start(dma_ch0, OMAP_DMA_BLOCK_IRQ);

		tf_dma_wait(1);

		OUTREG32(&sha1_md5_reg->SYSCONFIG, 0);

		omap_clear_dma(dma_ch0);

		data += length_loop;
		nDataLength -= length_loop;
		bytes_processed =
			INREG32(&sha1_md5_reg->DIGEST_COUNT);
	}

	/*For safety reasons, let's clean the working buffer */
	memset(dev->dma_buffer, 0, length_loop);

	/*release the DMA */
	omap_free_dma(dma_ch0);

	mutex_unlock(&dev->sm.dma_mutex);

	/*
	 * The dma transfert is finished, now wait until the hash
	 * operation is finished.
	 */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *)&sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_CONTEXT_READY_BIT);

	return true;
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
	u8 *data, u32 data_length)
{
	dprintk(KERN_INFO "tf_digest_update_dma\n");

	if (sha_state->chunk_length != 0) {

		u32 vLengthToComplete;

		/*Fill the chunk first */
		if (sha_state->
			chunk_length + data_length <= HASH_BLOCK_BYTES_LENGTH) {

			/*So we fill the chunk buffer with the new data */
			if (copy_from_user(sha_state->chunk_buffer +
					sha_state->chunk_length, data,
					data_length))
				return false;
			sha_state->chunk_length += data_length;

			/*We'll keep some data for the final */
			return true;
		}

		vLengthToComplete = HASH_BLOCK_BYTES_LENGTH - sha_state->
								chunk_length;

		if (vLengthToComplete != 0) {
			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			if (copy_from_user(sha_state->chunk_buffer +
					sha_state->chunk_length, data,
					vLengthToComplete))
				return false;
		}

		/*Then we send this buffer to the HWA (no DMA) */
		tf_digest_hw_perform_64b(
			(u32 *)sha_state->chunk_buffer, sha_state->CTRL,
			sha_state->bytes_processed);

		sha_state->bytes_processed =
			INREG32(&sha1_md5_reg->DIGEST_COUNT);

		/*We have flushed the chunk so it is empty now */
		sha_state->chunk_length = 0;

		/*Update the data buffer depending of the data already
		 *processed */
		data += vLengthToComplete;
		data_length -= vLengthToComplete;
	}

	if (data_length > HASH_BLOCK_BYTES_LENGTH) {

		/*DMA only manages data length that is multiple of 64b */
		u32 vDmaProcessize = data_length & 0xFFFFFFC0;

		if (vDmaProcessize == data_length) {
			/*We keep one block for the final */
			vDmaProcessize -= HASH_BLOCK_BYTES_LENGTH;
		}

		if (!tf_digest_hw_perform_dma(data, vDmaProcessize,
				sha_state->CTRL, sha_state->bytes_processed))
			return false;

		sha_state->bytes_processed =
			INREG32(&sha1_md5_reg->DIGEST_COUNT);
		data += vDmaProcessize;
		data_length -= vDmaProcessize;
	}

	/*At that point, there is less than 64b left to process*/
	if ((data_length == 0) || (data_length > HASH_BLOCK_BYTES_LENGTH))
		/*Should never be in this case !!! */
		return false;

	/*We now fill the chunk buffer with the remaining data */
	if (copy_from_user(sha_state->chunk_buffer, data, data_length))
		return false;
	sha_state->chunk_length = data_length;

	return true;
}

#ifdef CONFIG_SMC_KERNEL_CRYPTO
static void tf_digest_init_operation(u32 alg,
	struct tf_crypto_sha_operation_state *state)
{
	memset(state, 0, sizeof(struct tf_crypto_sha_operation_state));

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
	default:
		return -EINVAL;
	}

	for (i = 0; i < regs; i++) {
		tmp = INREG32(&sha1_md5_reg->IDIGEST_A + i);

		out[idx++] = (u8) ((tmp >>  0) & 0xff);
		out[idx++] = (u8) ((tmp >>  8) & 0xff);
		out[idx++] = (u8) ((tmp >> 16) & 0xff);
		out[idx++] = (u8) ((tmp >> 24) & 0xff);
	}

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
	OUTREG32(&sha1_md5_reg->DIGEST_COUNT, state->bytes_processed);
	OUTREG32(&sha1_md5_reg->MODE,
		(state->CTRL & 0x6) | 0x10 |
		(state->bytes_processed == 0) << 3);
	OUTREG32(&sha1_md5_reg->LENGTH, state->chunk_length);

	if (tf_crypto_wait_for_ready_bit(
		(u32 *) &sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_INPUT_READY_BIT)
			!= PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/* Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator"
		      "Input data to be ready\n");
	}

	OUTREG32(&sha1_md5_reg->DIN_0, data[0]);
	OUTREG32(&sha1_md5_reg->DIN_1, data[1]);
	OUTREG32(&sha1_md5_reg->DIN_2, data[2]);
	OUTREG32(&sha1_md5_reg->DIN_3, data[3]);
	OUTREG32(&sha1_md5_reg->DIN_4, data[4]);
	OUTREG32(&sha1_md5_reg->DIN_5, data[5]);
	OUTREG32(&sha1_md5_reg->DIN_6, data[6]);
	OUTREG32(&sha1_md5_reg->DIN_7, data[7]);
	OUTREG32(&sha1_md5_reg->DIN_8, data[8]);
	OUTREG32(&sha1_md5_reg->DIN_9, data[9]);
	OUTREG32(&sha1_md5_reg->DIN_10, data[10]);
	OUTREG32(&sha1_md5_reg->DIN_11, data[11]);
	OUTREG32(&sha1_md5_reg->DIN_12, data[12]);
	OUTREG32(&sha1_md5_reg->DIN_13, data[13]);
	OUTREG32(&sha1_md5_reg->DIN_14, data[14]);
	OUTREG32(&sha1_md5_reg->DIN_15, data[15]);

	/* Wait till the hash operation is finished */
	tf_crypto_wait_for_ready_bit_infinitely(
		(u32 *) &sha1_md5_reg->IRQSTATUS,
		DIGEST_IRQSTATUS_OUTPUT_READY_BIT);

	return static_Hash_HwReadDigest(DIGEST_MODE_GET_ALGO(state->CTRL), out);
}

/*
 * Digest HWA registration into kernel crypto framework
 */

static int digest_update(struct shash_desc *desc, const u8 *data,
	unsigned int len)
{
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	/* Make sure SHA/MD5 HWA is accessible */
	tf_delayed_secure_resume();

	tf_crypto_lock_hwa(PUBLIC_CRYPTO_HWA_SHA, LOCK_HWA);

	tf_crypto_enable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	tf_digest_update(state, (u8 *) data, len);

	tf_crypto_disable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	tf_crypto_lock_hwa(PUBLIC_CRYPTO_HWA_SHA, UNLOCK_HWA);

	return 0;
}

static int digest_final(struct shash_desc *desc, u8 *out)
{
	int ret;
	struct tf_crypto_sha_operation_state *state = shash_desc_ctx(desc);

	/* Make sure SHA/MD5 HWA is accessible */
	tf_delayed_secure_resume();

	tf_crypto_lock_hwa(PUBLIC_CRYPTO_HWA_SHA, LOCK_HWA);

	tf_crypto_enable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	ret = tf_digest_final(state, out);

	tf_crypto_disable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);

	tf_crypto_lock_hwa(PUBLIC_CRYPTO_HWA_SHA, UNLOCK_HWA);

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

int register_smc_public_crypto_digest(void)
{
	int ret;

	dprintk(KERN_INFO "SMC: Registering digest algorithms\n");

	ret = crypto_register_shash(&smc_md5_alg);
	if (ret)
		return ret;

	ret = crypto_register_shash(&smc_sha1_alg);
	if (ret)
		goto sha1_err;

	ret = crypto_register_shash(&smc_sha224_alg);
	if (ret)
		goto sha224_err;

	ret = crypto_register_shash(&smc_sha256_alg);
	if (ret)
		goto sha256_err;

	return 0;

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
	dprintk(KERN_INFO "SMC: Unregistering digest algorithms\n");

	crypto_unregister_shash(&smc_md5_alg);
	crypto_unregister_shash(&smc_sha1_alg);
	crypto_unregister_shash(&smc_sha224_alg);
	crypto_unregister_shash(&smc_sha256_alg);
}
#endif
