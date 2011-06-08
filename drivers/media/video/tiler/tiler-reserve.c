/*
 * tiler-reserve.c
 *
 * TILER driver area reservation functions for TI TILER hardware block.
 *
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "_tiler.h"

static struct tiler_ops *ops;	/* shared methods and variables */

/**
 * Calculate the maximum number buffers that can be packed next to each other,
 * and the area they occupy. This method is used for both 2D and NV12 packing.
 *
 * @author a0194118 (7/16/2010)
 *
 * @param o	desired offset
 * @param w	width of one block (>0)
 * @param a	desired alignment
 * @param b	band width (each block must occupy the same number of bands)
 * @param n	pointer to the desired number of blocks to pack.  It will be
 *		updated with the maximum number of blocks that can be packed.
 * @param _area	pointer to store total area needed
 *
 * @return packing efficiency (0-1024)
 */
u32 tiler_best2pack(u16 o, u16 a, u16 b, u16 w, u16 *n, u16 *_area)
{
	u16 m = 0, max_n = *n;		/* m is mostly n - 1 */
	u16 e = ALIGN(w, a);		/* effective width of one block */
	u32 eff, best_eff = 0;		/* best values */
	u16 stride = ALIGN(o + w, b);	/* block stride */
	u16 area = stride;		/* area needed (for m + 1 blocks) */

	/* NOTE: block #m+1 occupies the range (o + m * e, o + m * e + w) */

	/* see how many blocks we can pack */
	while (m < max_n &&
		/* blocks must fit in tiler container */
		o + m * e + w <= ops->width &&
		/* block stride must be correct */
		stride == ALIGN(area - o - m * e, b)) {

		m++;
		eff = m * w * 1024 / area;
		if (eff > best_eff) {
			/* store packing for best efficiency & smallest area */
			best_eff = eff;
			*n = m;
			if (_area)
				*_area = area;
		}
		/* update area */
		area = ALIGN(o + m * e + w, b);
	}

	return best_eff;
}

/**
 * We also optimize packing regular 2D areas as the auto-packing may result in
 * sub-optimal efficiency. This is most pronounced if the area is wider than
 * half a PAGE_SIZE (e.g. 2048 in 8-bit mode, or 1024 in 16-bit mode).
 */

/* reserve 2d blocks */
static void reserve_blocks(u32 n, enum tiler_fmt fmt, u32 width, u32 height,
			   u32 align, u32 offs, u32 gid,
			   struct process_info *pi)
{
	u32 bpt, res = 0, i;
	u16 o = offs, a = align, band, w, h, n_try;
	struct gid_info *gi;
	const struct tiler_geom *g;

	/* Check input parameters for correctness, and support */
	if (!width || !height || !n ||
	    align > PAGE_SIZE || offs >= align ||
	    fmt < TILFMT_8BIT || fmt > TILFMT_32BIT)
		return;

	/* tiler slot in bytes */
	g = ops->geom(fmt);
	bpt = g->slot_w * g->bpp;

	/*
	 *  For blocks narrower than half PAGE_SIZE the default allocation is
	 *  sufficient.  Also check for basic area info.
	 */
	if (width * g->bpp * 2 <= PAGE_SIZE ||
	    ops->analize(fmt, width, height, &w, &h, &band, &a, &o, NULL))
		return;

	/* get group id */
	gi = ops->get_gi(pi, gid);
	if (!gi)
		return;

	/* reserve in groups until failed or all is reserved */
	for (i = 0; i < n && res >= 0; i += res + 1) {
		/* blocks to allocate in one area */
		n_try = min(n - i, ops->width);
		tiler_best2pack(offs, a, band, w, &n_try, NULL);

		res = -1;
		while (n_try > 1) {
			/* adjust res so we fail on 0 return value */
			res = ops->lay_2d(fmt, n_try, w, h, band, a, o,
						gi, &gi->reserved) - 1;
			if (res >= 0)
				break;

			/* reduce n if failed to allocate area */
			n_try--;
		}
	}
	/* keep reserved blocks even if failed to reserve all */

	ops->release_gi(gi);
}

/* unreserve blocks for a group id */
static void unreserve_blocks(u32 gid, struct process_info *pi)
{
	struct gid_info *gi;

	gi = ops->get_gi(pi, gid);
	if (!gi)
		return;

	ops->release(&gi->reserved);

	ops->release_gi(gi);
}

/* initialize shared method pointers and global static variables */
void tiler_reserve_init(struct tiler_ops *tiler)
{
	ops = tiler;

	ops->reserve = reserve_blocks;
	ops->unreserve = unreserve_blocks;
}
