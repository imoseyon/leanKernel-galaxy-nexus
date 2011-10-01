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

#ifndef __TF_PUBLIC_DMA_H
#define __TF_PUBLIC_DMA_H

#include <linux/dma-mapping.h>
#include <plat/dma.h>
#include <plat/dma-44xx.h>

#include "tf_crypto.h"

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
u32 tf_dma_request(int *lch);

/**
 * This function waits for the DMA IRQ.
 */
void tf_dma_wait(int nr_of_cb);

/*
 * This function starts a DMA operation.
 *
 * lch			DMA channel ID.
 * interrupt_mask	Configures the Channel Interrupt Control Register.
 */
void tf_dma_start(int lch, int interrupt_mask);

void tf_dma_set_channel_common_params(
		struct omap_dma_channel_params *dma_channel,
		u32 nb_blocks, u32 nb_elements, u32 dst_start,
		u32 src_start, u32 trigger_id);

#endif /*__TF_PUBLIC_DMA_H */
