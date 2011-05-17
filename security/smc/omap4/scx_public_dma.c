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

#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#include "scx_public_dma.h"

#include <asm/atomic.h>

static atomic_t g_dmaEventFlag = ATOMIC_INIT(0);

/*------------------------------------------------------------------------ */
/*
 * Internal functions
 */

static void scxPublicDMACallback(int lch, u16 ch_status, void *data)
{
	atomic_inc(&g_dmaEventFlag);
}

/*------------------------------------------------------------------------ */
/*
 * Public DMA API
 */

u32 scxPublicDMARequest(int *lch)
{
	int dma_ch_out = 0;

	if (lch == NULL)
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;

	if (omap_request_dma(0, "SMC Public Crypto",
			scxPublicDMACallback, NULL, &dma_ch_out) != 0)
		return PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;

	omap_disable_dma_irq(dma_ch_out, OMAP_DMA_DROP_IRQ |
		OMAP_DMA_BLOCK_IRQ);

	*lch = dma_ch_out;

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------ */
/*
 * Release a DMA channel
 */
u32 scxPublicDMARelease(int lch)
{
	omap_free_dma(lch);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------ */

void scxPublicDMASetParams(int lch, struct omap_dma_channel_params *pParams)
{
	omap_set_dma_params(lch, pParams);
}

/*------------------------------------------------------------------------ */

void scxPublicDMAStart(int lch, int interruptMask)
{
	atomic_set(&g_dmaEventFlag, 0);
	omap_enable_dma_irq(lch, interruptMask);
	omap_start_dma(lch);
}

/*------------------------------------------------------------------------ */

void scxPublicDMADisableChannel(int lch)
{
	omap_stop_dma(lch);
}

/*------------------------------------------------------------------------ */

void scxPublicDMAClearChannel(int lch)
{
	omap_clear_dma(lch);
}

/*------------------------------------------------------------------------ */

void scxPublicDMAWait(int nr_of_cb)
{
	while (atomic_read(&g_dmaEventFlag) < nr_of_cb)
		cpu_relax();
}

/*------------------------------------------------------------------------ */
/*
 * Perform common DMA channel setup, used to factorize the code
 *
 * Output: struct omap_dma_channel_params *pDMAChannel
 * Inputs: u32 nbBlocks    Number of block of the transfer
 *         u32 nbElements  Number of elements of the transfer
 *         u32 nDstStart   Destination address
 *         u32 nSrcStart   Source address
 *         u32 nTriggerID  Trigger ID
 */
void scxPublicSetDMAChannelCommonParams(
		struct omap_dma_channel_params *pDMAChannel,
		u32 nbBlocks, u32 nbElements,
		u32 nDstStart, u32 nSrcStart, u32 nTriggerID)
{
	pDMAChannel->data_type = OMAP_DMA_DATA_TYPE_S32;
	pDMAChannel->elem_count = nbElements;
	pDMAChannel->frame_count = nbBlocks;
	pDMAChannel->src_ei = 0;
	pDMAChannel->src_fi = 0;
	pDMAChannel->dst_ei = 0;
	pDMAChannel->dst_fi = 0;
	pDMAChannel->sync_mode = OMAP_DMA_SYNC_FRAME;
	pDMAChannel->src_start = nSrcStart;
	pDMAChannel->dst_start = nDstStart;
	pDMAChannel->trigger = nTriggerID;
}
