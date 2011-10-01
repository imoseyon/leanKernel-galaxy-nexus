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
#include "tf_dma.h"

#include <asm/atomic.h>

static atomic_t g_dmaEventFlag = ATOMIC_INIT(0);

/*------------------------------------------------------------------------ */
/*
 * Internal functions
 */

static void tf_dma_callback(int lch, u16 ch_status, void *data)
{
	atomic_inc(&g_dmaEventFlag);
}

/*------------------------------------------------------------------------ */
/*
 * Public DMA API
 */

u32 tf_dma_request(int *lch)
{
	int dma_ch_out = 0;

	if (lch == NULL)
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;

	if (omap_request_dma(0, "SMC Public Crypto",
			tf_dma_callback, NULL, &dma_ch_out) != 0)
		return PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;

	omap_disable_dma_irq(dma_ch_out, OMAP_DMA_DROP_IRQ |
		OMAP_DMA_BLOCK_IRQ);

	*lch = dma_ch_out;

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------ */

void tf_dma_start(int lch, int interrupt_mask)
{
	atomic_set(&g_dmaEventFlag, 0);
	omap_enable_dma_irq(lch, interrupt_mask);
	omap_start_dma(lch);
}

/*------------------------------------------------------------------------ */

void tf_dma_wait(int nr_of_cb)
{
	while (atomic_read(&g_dmaEventFlag) < nr_of_cb)
		cpu_relax();
}

/*------------------------------------------------------------------------ */
/*
 * Perform common DMA channel setup, used to factorize the code
 *
 * Output: struct omap_dma_channel_params *dma_channel
 * Inputs: u32 nb_blocks    Number of block of the transfer
 *         u32 nb_elements  Number of elements of the transfer
 *         u32 dst_start   Destination address
 *         u32 src_start   Source address
 *         u32 trigger_id  Trigger ID
 */
void tf_dma_set_channel_common_params(
		struct omap_dma_channel_params *dma_channel,
		u32 nb_blocks, u32 nb_elements,
		u32 dst_start, u32 src_start, u32 trigger_id)
{
	dma_channel->data_type = OMAP_DMA_DATA_TYPE_S32;
	dma_channel->elem_count = nb_elements;
	dma_channel->frame_count = nb_blocks;
	dma_channel->src_ei = 0;
	dma_channel->src_fi = 0;
	dma_channel->dst_ei = 0;
	dma_channel->dst_fi = 0;
	dma_channel->sync_mode = OMAP_DMA_SYNC_FRAME;
	dma_channel->src_start = src_start;
	dma_channel->dst_start = dst_start;
	dma_channel->trigger = trigger_id;
}
