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

#include <linux/io.h>
#include <mach/io.h>

/*
 * DES Hardware Accelerator: Base address
 */
#define DES_REGS_HW_ADDR		0x480A5000

/*
 * CTRL register Masks
 */
#define DES_CTRL_OUTPUT_READY_BIT	(1<<0)
#define DES_CTRL_INPUT_READY_BIT	(1<<1)

#define DES_CTRL_GET_DIRECTION(x)	(x&4)
#define DES_CTRL_DIRECTION_DECRYPT	0
#define DES_CTRL_DIRECTION_ENCRYPT	(1<<2)

#define DES_CTRL_GET_TDES(x)		(x&8)
#define DES_CTRL_TDES_DES		0
#define DES_CTRL_TDES_TRIPLE_DES	(1<<3)

#define DES_CTRL_GET_MODE(x)		(x&0x10)
#define DES_CTRL_MODE_ECB		0
#define DES_CTRL_MODE_CBC		(1<<4)

/*
 * SYSCONFIG register masks
 */
#define DES_SYSCONFIG_DMA_REQ_IN_EN_BIT		(1<<5)
#define DES_SYSCONFIG_DMA_REQ_OUT_EN_BIT	(1<<6)

/*------------------------------------------------------------------------*/
/*				DES/DES3 Context			*/
/*------------------------------------------------------------------------*/
/**
 * This structure contains the registers of the DES HW accelerator.
 */
struct des3_des_reg {
	u32 DES_KEY3_L;	/* DES Key 3 Low Register		*/
	u32 DES_KEY3_H;	/* DES Key 3 High Register		*/
	u32 DES_KEY2_L;	/* DES Key 2 Low Register		*/
	u32 DES_KEY2_H;	/* DES Key 2 High Register		*/
	u32 DES_KEY1_L;	/* DES Key 1 Low Register		*/
	u32 DES_KEY1_H;	/* DES Key 1 High Register		*/
	u32 DES_IV_L;		/* DES Initialization Vector Low Reg	*/
	u32 DES_IV_H;		/* DES Initialization Vector High Reg	*/
	u32 DES_CTRL;		/* DES Control Register			*/
	u32 DES_LENGTH;	/* DES Length Register			*/
	u32 DES_DATA_L;	/* DES Data Input/Output Low Register	*/
	u32 DES_DATA_H;	/* DES Data Input/Output High Register	*/
	u32 DES_REV;		/* DES Revision Register		*/
	u32 DES_SYSCONFIG;	/* DES Mask and Reset Register		*/
	u32 DES_SYSSTATUS;	/* DES System Status Register		*/
};

static struct des3_des_reg *des_reg;

/*------------------------------------------------------------------------
 *Forward declarations
 *------------------------------------------------------------------------ */

static bool tf_des_update_dma(u8 *src, u8 *dest, u32 nb_blocks);

/*-------------------------------------------------------------------------
 *Save HWA registers into the specified operation state structure
 *-------------------------------------------------------------------------*/
static void tf_des_save_registers(u32 DES_CTRL,
	struct tf_crypto_des_operation_state *des_state)
{
	dprintk(KERN_INFO
		"tf_des_save_registers in des_state=%p CTRL=0x%08x\n",
		des_state, DES_CTRL);

	/*Save the IV if we are in CBC mode */
	if (DES_CTRL_GET_MODE(DES_CTRL) == DES_CTRL_MODE_CBC) {
		des_state->DES_IV_L = INREG32(&des_reg->DES_IV_L);
		des_state->DES_IV_H = INREG32(&des_reg->DES_IV_H);
	}
}

/*-------------------------------------------------------------------------
 *Restore the HWA registers from the operation state structure
 *-------------------------------------------------------------------------*/
static void tf_des_restore_registers(u32 DES_CTRL,
	struct tf_crypto_des_operation_state *des_state)
{
	dprintk(KERN_INFO "tf_des_restore_registers from "
		"des_state=%p CTRL=0x%08x\n",
		des_state, DES_CTRL);

	/*Write the IV ctx->reg */
	if (DES_CTRL_GET_MODE(DES_CTRL) == DES_CTRL_MODE_CBC) {
		OUTREG32(&des_reg->DES_IV_L, des_state->DES_IV_L);
		OUTREG32(&des_reg->DES_IV_H, des_state->DES_IV_H);
	}

	/*Set the DIRECTION and CBC bits in the CTRL register.
	 *Keep the TDES from the accelerator */
	OUTREG32(&des_reg->DES_CTRL,
		(INREG32(&des_reg->DES_CTRL) & (1 << 3)) |
		(DES_CTRL & ((1 << 2) | (1 << 4))));

	/*Set the SYSCONFIG register to 0 */
	OUTREG32(&des_reg->DES_SYSCONFIG, 0);
}

/*------------------------------------------------------------------------- */

void tf_des_init(void)
{
	des_reg = omap_ioremap(DES_REGS_HW_ADDR, SZ_1M, MT_DEVICE);
	if (des_reg == NULL)
		panic("Unable to remap DES/3DES module");
}

void tf_des_exit(void)
{
	omap_iounmap(des_reg);
}

bool tf_des_update(u32 DES_CTRL,
	struct tf_crypto_des_operation_state *des_state,
	u8 *src, u8 *dest, u32 nb_blocks)
{
	u32 nbr_of_blocks;
	u32 temp;
	u8 *process_src;
	u8 *process_dest;
	u32 dma_use = PUBLIC_CRYPTO_DMA_USE_NONE;

	/*
	 *Choice of the processing type
	 */
	if (nb_blocks * DES_BLOCK_SIZE >= DMA_TRIGGER_IRQ_DES)
		dma_use = PUBLIC_CRYPTO_DMA_USE_IRQ;

	dprintk(KERN_INFO "tf_des_update: "
		"src=0x%08x, dest=0x%08x, nb_blocks=0x%08x, dma_use=0x%08x\n",
		(unsigned int)src, (unsigned int)dest,
		(unsigned int)nb_blocks, (unsigned int)dma_use);

	if (nb_blocks == 0) {
		dprintk(KERN_INFO "tf_des_update: Nothing to process\n");
		return true;
	}

	if (DES_CTRL_GET_DIRECTION(INREG32(&des_reg->DES_CTRL)) !=
		DES_CTRL_GET_DIRECTION(DES_CTRL)) {
		dprintk(KERN_WARNING "HWA configured for another direction\n");
		return false;
	}

	/*Restore the registers of the accelerator from the operation state */
	tf_des_restore_registers(DES_CTRL, des_state);

	OUTREG32(&des_reg->DES_LENGTH, nb_blocks * DES_BLOCK_SIZE);

	if (dma_use == PUBLIC_CRYPTO_DMA_USE_IRQ) {

		/*perform the update with DMA */
		if (!tf_des_update_dma(src, dest, nb_blocks))
			return false;

	} else {
		u8 buf[DMA_TRIGGER_IRQ_DES];

		process_src = process_dest = buf;

		if (copy_from_user(buf, src, nb_blocks * DES_BLOCK_SIZE))
			return false;

		for (nbr_of_blocks = 0;
			nbr_of_blocks < nb_blocks; nbr_of_blocks++) {

			/*We wait for the input ready */
			/*Crash the system as this should never occur */
			if (tf_crypto_wait_for_ready_bit(
				(u32 *)&des_reg->DES_CTRL,
				DES_CTRL_INPUT_READY_BIT) !=
					PUBLIC_CRYPTO_OPERATION_SUCCESS) {
				panic("Wait too long for DES HW "
					"accelerator Input data to be ready\n");
			}

			/*We copy the 8 bytes of data src->reg */
			temp = (u32) BYTES_TO_LONG(process_src);
			OUTREG32(&des_reg->DES_DATA_L, temp);
			process_src += 4;
			temp = (u32) BYTES_TO_LONG(process_src);
			OUTREG32(&des_reg->DES_DATA_H, temp);
			process_src += 4;

			/*We wait for the output ready */
			tf_crypto_wait_for_ready_bit_infinitely(
						(u32 *)&des_reg->DES_CTRL,
						DES_CTRL_OUTPUT_READY_BIT);

			/*We copy the 8 bytes of data reg->dest */
			temp = INREG32(&des_reg->DES_DATA_L);
			LONG_TO_BYTE(process_dest, temp);
			process_dest += 4;
			temp = INREG32(&des_reg->DES_DATA_H);
			LONG_TO_BYTE(process_dest, temp);
			process_dest += 4;
		}

		if (copy_to_user(dest, buf, nb_blocks * DES_BLOCK_SIZE))
			return false;
	}

	/*Save the accelerator registers into the operation state */
	tf_des_save_registers(DES_CTRL, des_state);

	dprintk(KERN_INFO "tf_des_update: Done\n");
	return true;
}

/*------------------------------------------------------------------------- */
/*
 *Static function, perform DES encryption/decryption using the DMA for data
 *transfer.
 *
 *inputs: src : pointer of the input data to process
 *        nb_blocks : number of block to process
 *        dma_use : PUBLIC_CRYPTO_DMA_USE_IRQ (use irq to monitor end of DMA)
 *output: dest : pointer of the output data (can be eq to src)
 */
static bool tf_des_update_dma(u8 *src, u8 *dest, u32 nb_blocks)
{
	/*
	 *Note: The DMA only sees physical addresses !
	 */

	int dma_ch0;
	int dma_ch1;
	struct omap_dma_channel_params ch0_parameters;
	struct omap_dma_channel_params ch1_parameters;
	u32 length = nb_blocks * DES_BLOCK_SIZE;
	u32 length_loop = 0;
	u32 nb_blocksLoop = 0;
	struct tf_device *dev = tf_get_device();

	dprintk(KERN_INFO
		"tf_des_update_dma: In=0x%08x, Out=0x%08x, Len=%u\n",
		(unsigned int)src, (unsigned int)dest,
		(unsigned int)length);

	/*lock the DMA */
	mutex_lock(&dev->sm.dma_mutex);

	if (tf_dma_request(&dma_ch0) != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		mutex_unlock(&dev->sm.dma_mutex);
		return false;
	}
	if (tf_dma_request(&dma_ch1) != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		omap_free_dma(dma_ch0);
		mutex_unlock(&dev->sm.dma_mutex);
		return false;
	}

	while (length > 0) {

		/*
		 * At this time, we are sure that the DMAchannels are available
		 * and not used by other public crypto operation
		 */

		/*DMA used for Input and Output */
		OUTREG32(&des_reg->DES_SYSCONFIG,
				INREG32(&des_reg->DES_SYSCONFIG)
			| DES_SYSCONFIG_DMA_REQ_OUT_EN_BIT
			| DES_SYSCONFIG_DMA_REQ_IN_EN_BIT);

		/* Check length */
		if (length <= dev->dma_buffer_length)
			length_loop = length;
		else
			length_loop = dev->dma_buffer_length;

		/* The length is always a multiple of the block size */
		nb_blocksLoop = length_loop / DES_BLOCK_SIZE;

		/*
		 * Copy the data from the user input buffer into a preallocated
		 * buffer which has correct properties from efficient DMA
		 * transfers.
		 */
		if (copy_from_user(dev->dma_buffer, src, length_loop)) {
			omap_free_dma(dma_ch0);
			omap_free_dma(dma_ch1);
			mutex_unlock(&dev->sm.dma_mutex);
			return false;
		}

		/* DMA1: Mem -> DES */
		tf_dma_set_channel_common_params(&ch0_parameters,
			nb_blocksLoop,
			DMA_CEN_Elts_per_Frame_DES,
			DES_REGS_HW_ADDR + 0x28,
			dev->dma_buffer_phys,
			OMAP44XX_DMA_DES_P_DATA_IN_REQ);

		ch0_parameters.src_amode = OMAP_DMA_AMODE_POST_INC;
		ch0_parameters.dst_amode = OMAP_DMA_AMODE_CONSTANT;
		ch0_parameters.src_or_dst_synch = OMAP_DMA_DST_SYNC;

		dprintk(KERN_INFO
			"tf_des_update_dma: omap_set_dma_params(ch0)\n");
		omap_set_dma_params(dma_ch0, &ch0_parameters);

		/* DMA2: DES -> Mem */
		tf_dma_set_channel_common_params(&ch1_parameters,
			nb_blocksLoop,
			DMA_CEN_Elts_per_Frame_DES,
			dev->dma_buffer_phys,
			DES_REGS_HW_ADDR + 0x28,
			OMAP44XX_DMA_DES_P_DATA_OUT_REQ);

		ch1_parameters.src_amode = OMAP_DMA_AMODE_CONSTANT;
		ch1_parameters.dst_amode = OMAP_DMA_AMODE_POST_INC;
		ch1_parameters.src_or_dst_synch = OMAP_DMA_SRC_SYNC;

		dprintk(KERN_INFO "tf_des_update_dma: "
			"omap_set_dma_params(ch1)\n");
		omap_set_dma_params(dma_ch1, &ch1_parameters);

		wmb();

		dprintk(KERN_INFO
			"tf_des_update_dma: Start DMA channel %d\n",
			(unsigned int)dma_ch0);
		tf_dma_start(dma_ch0, OMAP_DMA_BLOCK_IRQ);

		dprintk(KERN_INFO
			"tf_des_update_dma: Start DMA channel %d\n",
			(unsigned int)dma_ch1);
		tf_dma_start(dma_ch1, OMAP_DMA_BLOCK_IRQ);
		tf_dma_wait(2);

		/* Unset DMA synchronisation requests */
		OUTREG32(&des_reg->DES_SYSCONFIG,
				INREG32(&des_reg->DES_SYSCONFIG)
			& (~DES_SYSCONFIG_DMA_REQ_OUT_EN_BIT)
			& (~DES_SYSCONFIG_DMA_REQ_IN_EN_BIT));

		omap_clear_dma(dma_ch0);
		omap_clear_dma(dma_ch1);

		/*
		 * The dma transfer is complete
		 */

		/*The DMA output is in the preallocated aligned buffer
		 *and needs to be copied to the output buffer.*/
		if (copy_to_user(dest, dev->dma_buffer, length_loop)) {
			omap_free_dma(dma_ch0);
			omap_free_dma(dma_ch1);
			mutex_unlock(&dev->sm.dma_mutex);
			return false;
		}

		src += length_loop;
		dest += length_loop;
		length -= length_loop;
	}

	/* For safety reasons, let's clean the working buffer */
	memset(dev->dma_buffer, 0, length_loop);

	/* Release the DMA */
	omap_free_dma(dma_ch0);
	omap_free_dma(dma_ch1);

	mutex_unlock(&dev->sm.dma_mutex);

	dprintk(KERN_INFO "tf_des_update_dma: Success\n");

	return true;
}
