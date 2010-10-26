/*
 * drivers/media/video/omapgfx/v4gfx.h
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __V4L2_GFX_H__
#define __V4L2_GFX_H__

#include <linux/version.h>
#include <media/videobuf-core.h>
#include <media/v4l2-device.h>
#include <asm/atomic.h>

#define MAX_VOUT_DEV 3

struct gbl_v4gfx {
	struct mutex mtx;
	int state;
	struct v4l2_device v4l2_dev;
	struct v4gfx_device *vouts[MAX_VOUT_DEV];
};

/* per-device data structure */
struct v4gfx_device {

	struct video_device *vfd;

	struct gbl_v4gfx *gbl_dev;

	int bpp; /* bytes per pixel */

	enum v4l2_buf_type type;

	struct v4l2_pix_format pix;

	struct v4l2_rect crop;

	enum v4l2_memory memory; /* how memory is managed for the device */

	/* we don't allow to change image fmt/size once buffer has
	 * been allocated
	 */
	int buffer_allocated;	/* count of buffers allocated */

	/* allow to reuse previously allocated buffer which is big enough */
	int buffer_size;

	unsigned long buf_phy_addr[VIDEO_MAX_FRAME];

	unsigned long buf_phy_uv_addr[VIDEO_MAX_FRAME]; /* NV12 support*/

	/* keep which buffers we actually allocated (via tiler) */
	unsigned long buf_phy_uv_addr_alloced[VIDEO_MAX_FRAME];

	unsigned long buf_phy_addr_alloced[VIDEO_MAX_FRAME];

	/*
	 For each V4L2 buffer requested we will have an array of page addresses
	 to give through the buffer class API
	 */
	unsigned long **buf_phys_addr_array;

	int mmap_count;

	int opened; /* inc/dec on open/close of the device */

	bool streaming;	/* is streaming is in progress? */

	struct mutex lock;	/* protect shared data structures in ioctl */

	struct videobuf_buffer *cur_frm;

	struct videobuf_buffer *locked_frm;

	struct videobuf_queue vbq;

	/*
	 * Buffers added by QBUF from the producer application
	 */
	struct list_head dma_queue;

	/*
	 * Buffers marked as done with by the consumer application but could
	 * still be being used by the GPU. DQBUF will examine this queue
	 * for available buffers.
	 */
	struct list_head sync_queue;

	wait_queue_head_t sync_done;

	unsigned long producer_ready;

	wait_queue_head_t consumer_wait;

	/*
	 * If acquire_timeout_ms is non-zero the acquire_timer will be reset
	 * when buffers are queued. If the timer expires ETIMEOUT will be
	 * returned via the V4L2_GFX_IOC_ACQ ioctl.
	 */
	struct timer_list acquire_timer;

	unsigned int acquire_timeout_ms;

	unsigned long acquire_timedout;

	spinlock_t vbq_lock;	/* spinlock for videobuf queues */

	unsigned int producer_flags;
};

extern int debug;

#define GFXLOG(level, dev, fmt, arg...) \
do {	\
	if (debug >= level)	\
		printk(KERN_INFO "%s: " fmt, (dev)->name , ## arg);	\
} while (0)

#define GFXLOGA(level, fmt, arg...) \
do {	\
	if (debug >= level)	\
		printk(KERN_INFO "v4l2-gfx: " fmt, ## arg);	\
} while (0)

/*
 * Convert local handle to v4l2_dev, currently only a global dev is supported
 */
#define V4L2DEV(vout) (&vout->gbl_dev->v4l2_dev)

/* tiler */
void v4gfx_tiler_buffer_free(
		struct v4gfx_device *vout, unsigned int count,
		unsigned int startindex);

int v4gfx_tiler_buffer_setup(struct v4gfx_device *vout,
		unsigned int *count, unsigned int startindex,
		struct v4l2_pix_format *pix);

void v4gfx_tiler_image_incr(struct v4gfx_device *vout,
		int *cpu_pgwidth, int *tiler_increment);

void v4gfx_tiler_image_incr_uv(struct v4gfx_device *vout, int *tiler_increment);

/* v4gfx */
int v4gfx_try_format(struct v4l2_pix_format *pix);
void v4gfx_buffer_array_free(struct v4gfx_device *vout, int cnt);
extern struct v4l2_ioctl_ops v4gfx_ioctl_ops;
extern const struct v4l2_file_operations v4gfx_fops;
extern void v4gfx_acquire_timer(unsigned long arg);

/* Other stuff */
#define YUYV_BPP    2
#define RGB565_BPP  2
#define RGB24_BPP   3
#define RGB32_BPP   4

#define VOUT_NAME		"v4gfx"

/* configuration macros */
#define VOUT_MAJOR_VERSION 0
#define VOUT_MINOR_VERSION 0
#define VOUT_RELEASE 0
#define VOUT_VERSION \
    KERNEL_VERSION(VOUT_MAJOR_VERSION, VOUT_MINOR_VERSION, VOUT_RELEASE)

#endif	/* ifndef __V4L2_GFX_H__ */
