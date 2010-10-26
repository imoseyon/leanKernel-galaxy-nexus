/**********************************************************************
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 ******************************************************************************/

#ifndef __V4L2_GFX_BC_H__
#define __V4L2_GFX_BC_H__

#include <media/v4l2-dev.h>

struct bc_buf_params_common {
	int count;	/*number of buffers */
	int width;	/*buffer width in pixel, multiple of 32 */
	int height;	/*buffer height in pixel  */
	u32 pixel_fmt;	/* V4L2 buffer pixel format */
	int stride;
	int size;
};

struct bc_buf_params {
	int count;	/*number of buffers (in) */
	int width;	/*buffer width in pixel, multiple of 32 (in) */
	int height;	/*buffer height in pixel (in) */
	u32 pixel_fmt;	/* V4L2 buffer pixel format (in) */
	int stride;	/*(out) */
	int size;	/*(out */
};

struct bc_buf_params2 {
	int count;	/*number of buffers (in) */
	int width;	/*buffer width in pixel, multiple of 32 (in) */
	int height;	/*buffer height in pixel (in) */
	u32 pixel_fmt;	/* V4L2 buffer pixel format (in) */
	int stride;	/*(in) */
	int size;	/*(out */
};
extern int bc_init(void);
extern void bc_cleanup(void);

/* bc_setup
 *
 * This API will validate the buffer parameters in order to setup a
 * buffer class device. Buffers should be added with subsequent calls to
 * bc_setup_buffer()
 */
extern int bc_setup(int id, struct bc_buf_params2 *p);

/* bc_setup_buffer
 *
 * Only called after a successful bc_setup(), add a physical buffer reference
 * to this device
 */
extern int bc_setup_buffer(
			int id, struct bc_buf_params2 *p, unsigned long *paddr);

/* bc_setup_complete
 *
 * Called after all physical buffers have been added to the device
 */
extern int bc_setup_complete(int id, struct bc_buf_params2 *p);

/* bc_sync_status
 *
 * Return the synchronization status of this devices buffer
 *
 * Return values:
 * 0	SGX still has pending operations on the buffer
 * 1	SGX done with the buffer
 */
extern int bc_sync_status(int id, int bufidx);
#endif
