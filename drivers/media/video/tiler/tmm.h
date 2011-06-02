/*
 * tmm.h
 *
 * TMM interface definition for TI TILER driver.
 *
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TMM_H
#define TMM_H

#include <mach/dmm.h>
/**
 * TMM interface
 */
struct tmm {
	void *pvt;

	/* function table */
	u32 *(*get)	(struct tmm *tmm, u32 num_pages);
	void (*free)	(struct tmm *tmm, u32 *pages);
	s32  (*pin)	(struct tmm *tmm, struct pat_area area, u32 page_pa);
	void (*unpin)	(struct tmm *tmm, struct pat_area area);
	void (*deinit)	(struct tmm *tmm);
};

/**
 * Request a set of pages from the DMM free page stack.
 * @return a pointer to a list of physical page addresses.
 */
static inline
u32 *tmm_get(struct tmm *tmm, u32 num_pages)
{
	if (tmm && tmm->pvt)
		return tmm->get(tmm, num_pages);
	return NULL;
}

/**
 * Return a set of used pages to the DMM free page stack.
 * @param list a pointer to a list of physical page addresses.
 */
static inline
void tmm_free(struct tmm *tmm, u32 *pages)
{
	if (tmm && tmm->pvt)
		tmm->free(tmm, pages);
}

/**
 * Program the physical address translator.
 * @param area PAT area
 * @param list of pages
 */
static inline
s32 tmm_pin(struct tmm *tmm, struct pat_area area, u32 page_pa)
{
	if (tmm && tmm->pin && tmm->pvt)
		return tmm->pin(tmm, area, page_pa);
	return -ENODEV;
}

/**
 * Clears the physical address translator.
 * @param area PAT area
 */
static inline
void tmm_unpin(struct tmm *tmm, struct pat_area area)
{
	if (tmm && tmm->unpin && tmm->pvt)
		tmm->unpin(tmm, area);
}

/**
 * Checks whether tiler memory manager supports mapping
 */
static inline
bool tmm_can_pin(struct tmm *tmm)
{
	return tmm && tmm->pin;
}

/**
 * Deinitialize tiler memory manager
 */
static inline
void tmm_deinit(struct tmm *tmm)
{
	if (tmm && tmm->pvt)
		tmm->deinit(tmm);
}

/**
 * TMM implementation for PAT support.
 *
 * Initialize TMM for PAT with given id.
 */
struct tmm *tmm_pat_init(u32 pat_id, u32 *dmac_va, u32 dmac_pa);

#endif
