/*
 * Copyright (c) 2006-2010 Trusted Logic S.A.
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

#ifndef __SCX_PUBLIC_DMA_H
#define __SCX_PUBLIC_DMA_H

#include <linux/dma-mapping.h>
#include <plat/dma.h>
#include <plat/dma-44xx.h>

#include "scx_public_crypto.h"

/*---------------------------------------------------------------------------
 * Cache management (implemented in the assembler file)
 *-------------------------------------------------------------------------- */

u32 v7_dma_flush_range(u32 nVAStart, u32 nVAEnd);
u32 v7_dma_inv_range(u32 nVAStart, u32 nVAEnd);

/*-------------------------------------------------------------------------- */
/*
 * Public DMA API
 */

/*
 * CEN Masks
 */
#define DMA_CEN_Elts_per_Frame_AES             4
#define DMA_CEN_Elts_per_Frame_DES             2
#define DMA_CEN_Elts_per_Frame_SHA             16

/*
 * Request a DMA channel
 */
u32 scxPublicDMARequest(int *lch);

/*
 * Release a DMA channel
 */
u32 scxPublicDMARelease(int lch);

/**
 * This function waits for the DMA IRQ.
 */
void scxPublicDMAWait(int nr_of_cb);

/*
 * This function starts a DMA operation.
 *
 * lch			DMA channel ID.
 * interruptMask	Configures the Channel Interrupt Control Register.
 */
void scxPublicDMAStart(int lch, int interruptMask);

void scxPublicSetDMAChannelCommonParams(
		struct omap_dma_channel_params *pDMAChannel,
		u32 nbBlocks, u32 nbElements, u32 nDstStart,
		u32 nSrcStart, u32 nTriggerID);
void scxPublicDMASetParams(int lch, struct omap_dma_channel_params *pParams);
void scxPublicDMADisableChannel(int lch);
void scxPublicDMAClearChannel(int lch);

#endif /*__SCX_PUBLIC_DMA_H */
