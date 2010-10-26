/*
 * drivers/media/video/omap/gfx_tiler.c
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "v4gfx.h"
#include "gfx_bc.h"

#ifdef CONFIG_TILER_OMAP
#include <mach/tiler.h>
#define TILER_ALLOCATE_V4L2
#endif

void v4gfx_tiler_buffer_free(struct v4gfx_device *vout, unsigned int count,
			     unsigned int startindex)
{
	int i;

	if (startindex < 0)
		startindex = 0;
	if (startindex + count > VIDEO_MAX_FRAME)
		count = VIDEO_MAX_FRAME - startindex;

	for (i = startindex; i < startindex + count; i++) {
		if (vout->buf_phy_addr_alloced[i])
			tiler_free(vout->buf_phy_addr_alloced[i]);
		if (vout->buf_phy_uv_addr_alloced[i])
			tiler_free(vout->buf_phy_uv_addr_alloced[i]);
		vout->buf_phy_addr[i] = 0;
		vout->buf_phy_addr_alloced[i] = 0;
		vout->buf_phy_uv_addr[i] = 0;
		vout->buf_phy_uv_addr_alloced[i] = 0;
	}
}

/* Allocate the buffers for  TILER space.  Ideally, the buffers will be ONLY
 in tiler space, with different rotated views available by just a convert.
 */
int v4gfx_tiler_buffer_setup(struct v4gfx_device *vout,
			unsigned int *count, unsigned int startindex,
			struct v4l2_pix_format *pix)
{
	/* startindex is always passed as 0, possibly tidy up? */
	int i, aligned = 1, bpp;
	enum tiler_fmt fmt;
	int rv = 0;

	/* normalize buffers to allocate so we stay within bounds */
	int start = (startindex < 0) ? 0 : startindex;
	int n_alloc = (start + *count > VIDEO_MAX_FRAME) ?
				  VIDEO_MAX_FRAME - start : *count;

    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);
	bpp = v4gfx_try_format(pix);
	if (bpp <= 0) {
		rv = bpp;	/* error condition */
		goto end;
	}

	GFXLOG(1, V4L2DEV(vout), "tiler buffer alloc: "
		"count = %d, start = %d :\n", *count, startindex);

	/* special allocation scheme for NV12 format */
	if (V4L2_PIX_FMT_NV12 == pix->pixelformat) {

		tiler_alloc_packed_nv12(&n_alloc, ALIGN(pix->width, 128),
				pix->height,
				(void **) vout->buf_phy_addr + start,
				(void **) vout->buf_phy_uv_addr + start,
				(void **) vout->buf_phy_addr_alloced + start,
				(void **) vout->buf_phy_uv_addr_alloced + start,
				aligned);

	} else {
		/* Only bpp of 1, 2, and 4 is supported by tiler */
		fmt = (bpp == 1 ? TILFMT_8BIT :
			   bpp == 2 ? TILFMT_16BIT :
			   bpp == 4 ? TILFMT_32BIT : TILFMT_INVALID);
		if (fmt == TILFMT_INVALID) {
			rv = -ENOMEM;
			goto end;
		}

		tiler_alloc_packed(&n_alloc, fmt, ALIGN(pix->width, 128 / bpp),
				pix->height,
				(void **) vout->buf_phy_addr + start,
				(void **) vout->buf_phy_addr_alloced + start,
				aligned);
	}

	GFXLOG(1, V4L2DEV(vout),
			"allocated %d buffers\n", n_alloc);

	if (n_alloc < *count) {
		if (n_alloc && (startindex == -1 ||
			V4L2_MEMORY_MMAP != vout->memory)) {
			/* TODO: check this condition's logic */
			v4gfx_tiler_buffer_free(vout, n_alloc, start);
			*count = 0;
			rv = -ENOMEM;
			goto end;
		}
	}

	for (i = start; i < start + n_alloc; i++) {
		GFXLOG(1, V4L2DEV(vout),
				"y=%08lx (%d) uv=%08lx (%d)\n",
				vout->buf_phy_addr[i],
				vout->buf_phy_addr_alloced[i] ? 1 : 0,
				vout->buf_phy_uv_addr[i],
				vout->buf_phy_uv_addr_alloced[i] ? 1 : 0);
	}

	*count = n_alloc;
end:
	GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

void v4gfx_tiler_image_incr(struct v4gfx_device *vout, int *cpu_pgwidth,
			    int *tiler_increment)
{
	/* for NV12, Y buffer is 1bpp*/
	if (V4L2_PIX_FMT_NV12 == vout->pix.pixelformat) {
		*cpu_pgwidth =
			(vout->pix.width + TILER_PAGE - 1) & ~(TILER_PAGE - 1);
		*tiler_increment = 64 * TILER_WIDTH;
	} else {
		*cpu_pgwidth = (vout->pix.width * vout->bpp + TILER_PAGE - 1) &
							~(TILER_PAGE - 1);
		if (vout->bpp > 1)
			*tiler_increment = 2 * 64 * TILER_WIDTH;
		else
			*tiler_increment = 64 * TILER_WIDTH;
	}
}

void v4gfx_tiler_image_incr_uv(struct v4gfx_device *vout, int *tiler_increment)
{
	if (vout->pix.pixelformat == V4L2_PIX_FMT_NV12)
		*tiler_increment = 2 * 64 * TILER_WIDTH;
	/* Otherwise do nothing */
}
