/*
 * drivers/media/video/omap/v4gfx.c
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/omap_v4l2_gfx.h>	/* private ioctls */

#include <media/videobuf-dma-contig.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>

#include "v4gfx.h"
#include "gfx_bc.h"

#define V4GFX_WAIT_DEQUE 1	/* Poll buffer sync status during dq */
#define V4GFX_WAIT_UNLOCK 2	/* Poll buffer sync status from render loop */
/*
 * V4GFX_WAITMETHOD is used to select between how we wait for SGX to release
 * buffers sent to it.
 */
/* #define V4GFX_WAITMETHOD V4GFX_WAIT_DEQUE */
#define V4GFX_WAITMETHOD V4GFX_WAIT_UNLOCK

#define VID_MAX_WIDTH		2048 /* Largest width */
#define VID_MAX_HEIGHT		2048 /* Largest height */
#define VID_MIN_WIDTH		0
#define VID_MIN_HEIGHT		0
#define V4GFX_FRAME_UNLOCK_TIMEOUT 16	/* ms */


/*
 * This will enable dumping of the mappings obtain
 */
#ifdef V4L2GFX_DUMPMMAP
#define DUMPMMAP(msg, k, vma, m, pos, p) \
	printk(KERN_NOTICE	\
		"%s: vm_start+%d = 0x%lx, dma->vmalloc+%d = 0x%lx, w=0x%x\n", \
		msg, k, vma->vm_start + k, m, (pos + m), p);
#else
#define DUMPMMAP(msg, k, vma, m, pos, p)
#endif

static struct videobuf_queue_ops video_vbq_ops;

static u32 v4gfx_calc_buffer_size(
		int bpp, u32 width, u32 height, u32 pixelformat);
static u32 v4gfx_calc_stride(int bpp, u32 width);

/*
 * List of image formats supported by the SGX buffer-class api
 */
static const struct v4l2_fmtdesc gfx_bc_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * OMAP video pipelines interpret RGB565 as:
		 *
		 *      Byte 0                    Byte 1
		 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
		 *
		 * GFX ?? TODO
		 */
		.description = "RGB565, le",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		.description = "RGB32, le",
		.pixelformat = V4L2_PIX_FMT_RGB32,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
		.description = "NV12 - YUV420 format",
		.pixelformat = V4L2_PIX_FMT_NV12,
	},
};

#define NUM_OUTPUT_FORMATS (ARRAY_SIZE(gfx_bc_formats))

int v4gfx_try_format(struct v4l2_pix_format *pix)
{
	int ifmt, bpp = 0;

	pix->height =
		clamp(pix->height, (u32)VID_MIN_HEIGHT, (u32)VID_MAX_HEIGHT);
	pix->width = clamp(pix->width, (u32)VID_MIN_WIDTH, (u32)VID_MAX_WIDTH);

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == gfx_bc_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt >= NUM_OUTPUT_FORMATS)
		ifmt = 0;	/* Default V4L2_PIX_FMT_RGB565 */
	pix->pixelformat = gfx_bc_formats[ifmt].pixelformat;

	pix->field = V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
		break;
	case V4L2_PIX_FMT_NV12:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 1;	/* 12bits per pixel, 1 byte for Y */
		break;
	}

	pix->bytesperline = v4gfx_calc_stride(bpp, pix->width);
	pix->sizeimage = v4gfx_calc_buffer_size(bpp, pix->width, pix->height,
						pix->pixelformat);

	if (V4L2_PIX_FMT_NV12 == pix->pixelformat)
		pix->sizeimage += pix->sizeimage >> 1;

	return bpp;
}

void v4gfx_acquire_timer(unsigned long arg)
{
	struct v4gfx_device *vout = (struct v4gfx_device *)arg;

	set_bit(1, &vout->acquire_timedout);
}

#if V4GFX_WAITMETHOD == V4GFX_WAIT_DEQUE
static struct videobuf_buffer *v4gfx_get_next_syncframe(
						struct v4gfx_device *vout)
{
	struct videobuf_buffer *buf;
	mutex_lock(&vout->lock);
	if (list_empty(&vout->sync_queue)) {
		mutex_unlock(&vout->lock);
		return NULL;
	}
	buf = list_entry(vout->sync_queue.next, struct videobuf_buffer, queue);
	mutex_unlock(&vout->lock);
	return buf;
}

static int v4gfx_wait_on_pending(struct v4gfx_device *vout, int bufidx)
{
	int dqable = 0;
	int iteration = 0;

	do {
		dqable = bc_sync_status(0, bufidx);
		if (!dqable) {
			/* printk("w-on %d [%d]\n", bufidx, iteration); */
			if (iteration++ < V4GFX_FRAME_UNLOCK_TIMEOUT) {
				msleep(1);	/* milliseconds */
			} else {
				/*printk("t-o %d\n", bufidx); */
				break;	/* Timed out */
			}
		}
/*
		else {
			printk("dq-o %d\n", bufidx);
		}
 */
	} while (!dqable);

	return dqable;
}

static void v4gfx_done_syncframe(struct v4gfx_device *vout,
				 struct videobuf_buffer *sync_frame)
{
	struct timeval timevalue = {0};
	unsigned long flags;
	mutex_lock(&vout->lock);
	spin_lock_irqsave(&vout->vbq_lock, flags);

	list_del(&sync_frame->queue);

	do_gettimeofday(&timevalue);
	sync_frame->ts = timevalue;
	sync_frame->state = VIDEOBUF_DONE;
	wake_up_interruptible(&sync_frame->done);
	spin_unlock_irqrestore(&vout->vbq_lock, flags);
	mutex_unlock(&vout->lock);
}
#endif /* V4GFX_WAIT_DEQUE */


static u32 v4gfx_calc_stride(int bpp, u32 width)
{
	return PAGE_ALIGN(width * bpp);
}

static u32 v4gfx_calc_buffer_size(
		int bpp, u32 width, u32 height, u32 pixelformat)
{
	int stride;
	stride = v4gfx_calc_stride(bpp, width);

	/* i is the block-width - either 4K or 8K, depending upon input width*/
	/* for NV12 format, buffer is height + height / 2*/
    if (V4L2_PIX_FMT_NV12 == pixelformat)
	return height * 3/2 * stride;
    else
	return height * stride;
}

void v4gfx_buffer_array_free(struct v4gfx_device *vout, int cnt)
{
	/* Fn should be robust and callable with args in a dubious state */
	int i;
	if (!vout || !cnt)
		return;
	if (vout->buf_phys_addr_array) {
		for (i = 0; i < cnt; i++)
			kfree(vout->buf_phys_addr_array[i]);
		kfree(vout->buf_phys_addr_array);
		vout->buf_phys_addr_array = NULL;
	}
}

/*
 * Allocate a buffer array for all the requested buffers
 * If there is an allocation failure the function will clean up after itself
 */
static int v4gfx_buffer_array_realloc(struct v4gfx_device *vout,
				      int oldcnt, int newcnt)
{
	int i;

	if (vout->buf_phys_addr_array)
		v4gfx_buffer_array_free(vout, oldcnt);

	vout->buf_phys_addr_array =
		kzalloc(sizeof(unsigned long *) * newcnt, GFP_KERNEL);
	if (!vout->buf_phys_addr_array)
		return -ENOMEM;

	/* 2048 is the max image height, 2 = (2048 * 4) / CPU_PAGE_SIZE */
	for (i = 0; i < newcnt; i++) {
		vout->buf_phys_addr_array[i] =
			kmalloc(sizeof(unsigned long) * 2048 * 2, GFP_KERNEL);
		if (!vout->buf_phys_addr_array[i]) {
			v4gfx_buffer_array_free(vout, newcnt);
			return -ENOMEM;
		}
	}
	return 0;
}

static void v4gfx_buffer_array_fill(
				struct v4gfx_device *vout,
				int bufno,
				unsigned long tiler_paddr_in,
				unsigned long tiler_paddr_uv_in)
{
	int buf_phys_idx = 0;
	int m = 0, i;
	int cpu_pgwidth;
	int tiler_increment;

	v4gfx_tiler_image_incr(vout, &cpu_pgwidth, &tiler_increment);

	for (i = 0; i < vout->pix.height; i++) {
		unsigned long pg, pgend, tiler_paddr;

		tiler_paddr = tiler_paddr_in+m;
		pg = tiler_paddr;
		pgend = pg + cpu_pgwidth;
		do {
			GFXLOGA(2, "%d %d: = %lx\n", bufno, buf_phys_idx,
								(long)pg);
			vout->buf_phys_addr_array[bufno][buf_phys_idx] = pg;
			pg += 4096;
			buf_phys_idx++;
		} while (pg < pgend);

		m += tiler_increment;
	}

	if (V4L2_PIX_FMT_NV12 == vout->pix.pixelformat) {
		m = 0;
		v4gfx_tiler_image_incr_uv(vout, &tiler_increment);

		/* UV buffer is height / 2 */
		for (i = 0; i < vout->pix.height / 2; i++) {
			unsigned long pg;

			pg = tiler_paddr_uv_in+m;
			vout->buf_phys_addr_array[bufno][buf_phys_idx] = pg;
			m += tiler_increment;
			buf_phys_idx++;
			}

		GFXLOGA(1, "nv12 uv: 0x%lx\n", tiler_paddr_uv_in);
		m += tiler_increment;
	}
}

static int v4gfx_frame_lock(struct v4gfx_device *vout, int *bufid)
{
	struct videobuf_buffer *oldbuf = NULL;
#if V4GFX_WAITMETHOD == V4GFX_WAIT_UNLOCK
	struct timeval timevalue = {0};
#else	/* V4GFX_WAIT_DEQUE */
	int oldbufid = -1;
#endif
	unsigned long flags;
	int rv = 0;

	mutex_lock(&vout->lock);
	spin_lock_irqsave(&vout->vbq_lock, flags);
	if (!vout->streaming || !vout->cur_frm) {
		GFXLOG(1, V4L2DEV(vout),
			"%s: ERROR: device not streaming yet\n", __func__);
		rv = -EAGAIN;
		goto unlock;
	}

	/* vout->cur_frm must be set if streaming */

	if (vout->cur_frm == vout->locked_frm) {
		/*
		 * If this frame has been locked before we will
		 * attempt to get the next buffer in the dma queue.
		 * If there is a next buffer, mark the locked
		 * buffer as done and then promote the next buffer
		 * to the current buffer whilst locking it in the
		 * process.
		 */
		if (list_empty(&vout->dma_queue)) {
			*bufid = vout->cur_frm->i;
			/*
			 * We can't do anything else here, it will be upto
			 * the consumer application to decide whether it wants
			 * to re-render the texture which depends on what the
			 * app is doing.
			 */
			goto unlock;
		}

		/* Deactivate the cur_frm */
		oldbuf = vout->cur_frm;

		vout->cur_frm = list_entry(vout->dma_queue.next,
						struct videobuf_buffer, queue);

		list_del(&vout->cur_frm->queue);

		vout->cur_frm->state = VIDEOBUF_ACTIVE;

		GFXLOG(2, V4L2DEV(vout), "Active frame %d\n", vout->cur_frm->i);

		vout->locked_frm = vout->cur_frm;

#if V4GFX_WAITMETHOD == V4GFX_WAIT_UNLOCK
		/*
		 * Mark the previous current buffer done and release it for
		 * dequeue
		 */
		do_gettimeofday(&timevalue);
		oldbuf->ts = timevalue;
		oldbuf->state = VIDEOBUF_DONE;
		wake_up_interruptible(&oldbuf->done);
#else	/* V4GFX_WAIT_DEQUE */
		oldbufid = oldbuf->i;
		list_add_tail(&oldbuf->queue, &vout->sync_queue);
		wake_up_interruptible(&vout->sync_done);
#endif

	} else {
		/* First time we've tried to lock this frame */
		vout->locked_frm = vout->cur_frm;
		/* We be marked for dequeue next time */
	}
	*bufid = vout->locked_frm->i;
unlock:
	spin_unlock_irqrestore(&vout->vbq_lock, flags);
	mutex_unlock(&vout->lock);

#if V4GFX_WAITMETHOD == V4GFX_WAIT_DEQUE
/*
	if (oldbufid != -1)
		printk("sync_queue + %d\n", oldbufid);
 */
#endif
	return rv;
}

static int v4gfx_frame_unlock(struct v4gfx_device *vout, int bufidx)
{
	struct videobuf_buffer *vbuf;
	int rv = 0;
#if V4GFX_WAITMETHOD == V4GFX_WAIT_UNLOCK
	int iteration = 0;
#endif

	mutex_lock(&vout->lock);
	vbuf = vout->locked_frm;
	if (!vbuf) {
		GFXLOG(1, V4L2DEV(vout),
			"%s: ERROR: trying to unlock a non-existent frame\n",
			__func__);
		rv = -EINVAL;
	} else if (vbuf->i != bufidx) {
		GFXLOG(1, V4L2DEV(vout),
			"%s: ERROR: trying to unlock wrong frame %d %d\n",
			__func__, vbuf->i, bufidx);
		rv = -EINVAL;
	}
	mutex_unlock(&vout->lock);

#if V4GFX_WAITMETHOD == V4GFX_WAIT_UNLOCK
	if (rv != 0)
		goto end;

	do {
		/*
		 * Interrogate the buffer class synch data buffer to see if SGX
		 * is done with this buffer
		 */
		rv = bc_sync_status(0, bufidx);
		if (rv == 0) {
			if (iteration++ < V4GFX_FRAME_UNLOCK_TIMEOUT)
				msleep(1);	/* milliseconds */
		}
	} while (rv == 0);

	if (iteration >= V4GFX_FRAME_UNLOCK_TIMEOUT) {
		printk("%s: INFO: timed out\n", __func__);
		rv = -ETIMEDOUT;
	} else
		rv = 0;
end:
#endif	/* V4GFX_WAIT_UNLOCK */
	return rv;
}

/*
 * Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers
 */
static int vbq_ops_buf_setup(struct videobuf_queue *q, unsigned int *count,
		unsigned int *size)
{
	struct v4gfx_device *vout = q->priv_data;
	int rv = 0;
    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	if (!vout || (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)) {
		rv = -EINVAL; goto end;
	}

	*size = vout->buffer_size = v4gfx_calc_buffer_size(
							vout->bpp,
							vout->pix.width,
							vout->pix.height,
							vout->pix.pixelformat);

	GFXLOG(1, V4L2DEV(vout), "height=%d, size=%d\n",
		   vout->pix.height, *size);

	if (v4gfx_tiler_buffer_setup(vout, count, 0, &vout->pix)) {
			rv = -ENOMEM; goto end;
	}

end:
	GFXLOG(1, V4L2DEV(vout), "Exiting %s\n", __func__);
	return rv;
}

/*
 * This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * user space virtual address into physical address if userptr memory
 * exchange mechanism is used.
 */
static int vbq_ops_buf_prepare(struct videobuf_queue *q,
			    struct videobuf_buffer *vb,
			    enum v4l2_field field)
{
	struct v4gfx_device *vout = q->priv_data;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vout->pix.width;
		vb->height = vout->pix.height;
		vb->size = vb->width * vb->height * vout->bpp;
		vb->field = field;

	}
	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

/*
 * Buffer queue function will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed.
 */
static void vbq_ops_buf_queue(struct videobuf_queue *q,
			  struct videobuf_buffer *vb)
{
	struct v4gfx_device *vout = q->priv_data;

	list_add_tail(&vb->queue, &vout->dma_queue);
	vb->state = VIDEOBUF_QUEUED;
}

/*
 * Buffer release function is called from videobuf layer to release buffer
 * which are already allocated
 */
static void vbq_ops_buf_release(struct videobuf_queue *q,
			    struct videobuf_buffer *vb)
{
	struct v4gfx_device *vout = q->priv_data;

	vb->state = VIDEOBUF_NEEDS_INIT;

	if (V4L2_MEMORY_MMAP != vout->memory)
		return;
}

/*
 *  File operations
 */
static void v4gfx_vm_open(struct vm_area_struct *vma)
{
	struct v4gfx_device *vout = vma->vm_private_data;

	GFXLOG(1, V4L2DEV(vout),
		"vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count++;
}

static void v4gfx_vm_close(struct vm_area_struct *vma)
{
	struct v4gfx_device *vout = vma->vm_private_data;

	GFXLOG(1, V4L2DEV(vout),
		"vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);

	vout->mmap_count--;
}

static struct vm_operations_struct v4gfx_vm_ops = {
	.open = v4gfx_vm_open,
	.close = v4gfx_vm_close,
};

static int vidfop_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct v4gfx_device *vout = file->private_data;
	struct videobuf_queue *q = &vout->vbq;
	int i;
	void *pos;
	int j = 0, k = 0, m = 0, p = 0, m_increment = 0;

	GFXLOG(1, V4L2DEV(vout), "Entering %s\n", __func__);

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i) {
		GFXLOG(1, V4L2DEV(vout),
		"offset invalid [offset=0x%lx]\n",
			(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}
	q->bufs[i]->baddr = vma->vm_start;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &v4gfx_vm_ops;
	vma->vm_private_data = (void *) vout;
	pos = (void *)vout->buf_phy_addr[i];

	/* get line width */
	v4gfx_tiler_image_incr(vout, &p, &m_increment);

	for (j = 0; j < vout->pix.height; j++) {
		/* map each page of the line */
		DUMPMMAP("Y buffer", k, vma, m, pos, p);

		vma->vm_pgoff = ((unsigned long)pos + m) >> PAGE_SHIFT;

		if (remap_pfn_range(vma, vma->vm_start + k,
					((unsigned long)pos + m) >> PAGE_SHIFT,
					p, vma->vm_page_prot))
			return -EAGAIN;
		k += p;
		m += m_increment;
	}
	m = 0;

	/* UV Buffer in case of NV12 format */
	if (V4L2_PIX_FMT_NV12 == vout->pix.pixelformat) {
		pos = (void *)vout->buf_phy_uv_addr[i];

		v4gfx_tiler_image_incr_uv(vout, &m_increment);

		/* UV buffer is height / 2 */
		for (j = 0; j < vout->pix.height / 2; j++) {
			/* map each page of the line */
			DUMPMMAP("UV buffer", k, vma, m, pos, p);

			vma->vm_pgoff = ((unsigned long)pos + m) >> PAGE_SHIFT;

			if (remap_pfn_range(vma, vma->vm_start + k,
					((unsigned long)pos + m) >> PAGE_SHIFT,
					p, vma->vm_page_prot))
				return -EAGAIN;
			k += p;
			m += m_increment;
		}
	}

	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */
	vout->mmap_count++;
	GFXLOG(1, V4L2DEV(vout), "Exiting %s\n", __func__);
	return 0;
}

static int vidfop_release(struct file *file)
{
	struct v4gfx_device *vout = file->private_data;
	struct videobuf_queue *q;
	unsigned int r = 0;

	GFXLOG(1, V4L2DEV(vout), "Entering %s\n", __func__);
	GFXLOG(1, V4L2DEV(vout),
			"current process id/pid is %d\n", current->pid);

	if (!vout)
		goto end;

	vout->opened = vout->opened ? vout->opened - 1 : 0;
	if (vout->opened) {
		r = 0;
		goto end;
	}

	clear_bit(1, &vout->producer_ready);

	q = &vout->vbq;

	if (vout->streaming) {
		del_timer_sync(&vout->acquire_timer);
		clear_bit(1, &vout->acquire_timedout);

		vout->streaming = false;
		videobuf_streamoff(q);
		videobuf_queue_cancel(q);
	}

	if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory))
		videobuf_mmap_free(q);
	vout->mmap_count = 0;

	/* Free buffers */
	if (vout->buffer_allocated) {
		v4gfx_tiler_buffer_free(vout, vout->buffer_allocated, 0);
		vout->buffer_allocated = 0;
	}

	memset(&vout->crop, 0, sizeof(vout->crop));
	memset(&vout->pix, 0, sizeof(vout->pix));

	file->private_data = NULL;

end:
	GFXLOG(1, V4L2DEV(vout), "Exiting %s\n", __func__);
	return r;
}

static int vidfop_open(struct file *file)
{
	struct v4gfx_device *vout = NULL;
	struct videobuf_queue *q;
	int rv = 0;

	vout = video_drvdata(file);
	if (vout == NULL) {
		rv = -ENODEV;
		goto end;
	}

	GFXLOG(1, V4L2DEV(vout), "Entering %s : %x\n", __func__, (int)vout);
	GFXLOG(1, V4L2DEV(vout), "current pid is %d\n", current->pid);

	vout->opened += 1;
	file->private_data = vout;

	if (vout->opened > 1) {
		GFXLOG(1, V4L2DEV(vout), "Another opening....\n");
		goto end;
	}

	clear_bit(1, &vout->producer_ready);

	q = &vout->vbq;
	video_vbq_ops.buf_setup = vbq_ops_buf_setup;
	video_vbq_ops.buf_prepare = vbq_ops_buf_prepare;
	video_vbq_ops.buf_release = vbq_ops_buf_release;
	video_vbq_ops.buf_queue = vbq_ops_buf_queue;

	videobuf_queue_dma_contig_init(q, &video_vbq_ops, q->dev,
				&vout->vbq_lock, vout->type, V4L2_FIELD_NONE,
				sizeof(struct videobuf_buffer), vout);

end:
	GFXLOG(1, V4L2DEV(vout), "Exiting %s :%d\n", __func__, rv);
	return rv;
}

/* V4L2 ioctls */
static int vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct v4gfx_device *vout = fh;
	GFXLOG(1, V4L2DEV(vout), "Entering %s\n", __func__);

	strlcpy(cap->driver, VOUT_NAME, sizeof(cap->driver));
	strlcpy(cap->card, vout->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->version = VOUT_VERSION;
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	return 0;
}

static int vidioc_log_status(struct file *file, void *fh)
{
	/* struct v4gfx_device *vout = fh; */
	printk(KERN_INFO "\n");
	printk(KERN_INFO "============== START LOG STATUS ================\n");
	printk(KERN_INFO "=============== END LOG STATUS =================\n");
	printk(KERN_INFO "\n");
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	struct v4gfx_device *vout = fh;
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;
	int rv = 0;

	GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS) {
		rv = -EINVAL;
		goto end;
	}

	fmt->flags = gfx_bc_formats[index].flags;
	strlcpy(fmt->description, gfx_bc_formats[index].description,
			sizeof(fmt->description));
	fmt->pixelformat = gfx_bc_formats[index].pixelformat;
end:
	GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct v4gfx_device *vout = fh;
    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	f->fmt.pix = vout->pix;

    GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, 0);
	return 0;

}

/*
 * VIDIOC_TRY_FMT ioctl is equivalent to VIDIOC_S_FMT with one
 * exception: it does not change driver state. It can also be called at any
 * time, never returning EBUSY.
 */
static int vidioc_try_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	int r;
	struct v4gfx_device *vout = fh;
    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	r = v4gfx_try_format(&f->fmt.pix);

    GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, r);
	return  (r >= 0) ? 0 : r;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct v4gfx_device *vout = fh;
	int rv = 0;
	int bpp;

    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	mutex_lock(&vout->lock);
	if (vout->streaming) {
		rv = -EBUSY;
		goto end;
	}

	bpp = v4gfx_try_format(&f->fmt.pix);
	if (bpp <= 0) {
		rv = bpp;
		goto end;
	}

	/* try & set the new output format */
	vout->bpp = bpp;
	vout->pix = f->fmt.pix;

end:
	mutex_unlock(&vout->lock);
    GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	struct bc_buf_params2 bc_params;
	struct v4gfx_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	unsigned int i;
	int rv = 0;

    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
		(req->count < 0) ||
		(req->memory != V4L2_MEMORY_MMAP)
	   ) {
		rv = -EINVAL; goto end;
	}


	mutex_lock(&vout->lock);
	/* Cannot be requested when streaming is on */
	if (vout->streaming) {
		mutex_unlock(&vout->lock);
		rv = -EBUSY; goto end;
	}

	/*
	 * TODO A count value of zero frees all buffers, after aborting or
	 * finishing any DMA in progress, an implicit VIDIOC_STREAMOFF.
	 */

	/* If buffers are already allocated free them */
	if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory)) {
		if (vout->mmap_count) {
			mutex_unlock(&vout->lock);
			rv = -EBUSY; goto end;
		}

		v4gfx_tiler_buffer_free(vout, vout->buffer_allocated, 0);
		vout->buffer_allocated = 0;

		videobuf_mmap_free(q);
	}

	bc_params.count = req->count;
	bc_params.width = vout->pix.width;
	bc_params.height = vout->pix.height;
	bc_params.pixel_fmt = vout->pix.pixelformat;
/*	bc_params.stride = vout->pix.bytesperline; */
	rv = bc_setup(0, &bc_params);
	if (rv < 0) {
		GFXLOG(1, V4L2DEV(vout),
				"+%s bc_setup() failed %d\n", __func__, rv);
		goto end;
	}

	/*
	 * Note that the actual buffer allocation is done in
	 * vbq_ops_buf_setup
	 */
	rv = videobuf_reqbufs(q, req);
	if (rv < 0) {
		mutex_unlock(&vout->lock);
		goto end;
	}

	INIT_LIST_HEAD(&vout->dma_queue);
	INIT_LIST_HEAD(&vout->sync_queue);

	/*
	 * The realloc will free the old array and allocate a new one
	 */
	rv = v4gfx_buffer_array_realloc(vout, vout->buffer_allocated,
					req->count);
	if (rv < 0) {
		mutex_unlock(&vout->lock);
		goto end;
	}

	vout->memory = req->memory;
	vout->buffer_allocated = req->count;

	for (i = 0; i < req->count; i++) {

		v4gfx_buffer_array_fill(vout, i,
				vout->buf_phy_addr[i],
				V4L2_PIX_FMT_NV12 == vout->pix.pixelformat ?
					vout->buf_phy_uv_addr[i] : 0);

		bc_setup_buffer(0, &bc_params, vout->buf_phys_addr_array[i]);
	}
	bc_setup_complete(0, &bc_params);

	mutex_unlock(&vout->lock);
end:
    GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_querybuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct v4gfx_device *vout = fh;
	int rv;

    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	rv =  videobuf_querybuf(&vout->vbq, b);

    GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_qbuf(struct file *file, void *fh,
			struct v4l2_buffer *buf)
{
	struct v4gfx_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int rv = 0;

	GFXLOG(1, V4L2DEV(vout), "qbuf buf: %d\n", buf->index);

	if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != buf->type) ||
		(buf->index >= vout->buffer_allocated) ||
		(q->bufs[buf->index]->memory != buf->memory)) {
		return -EINVAL;
	}
	if (V4L2_MEMORY_USERPTR == buf->memory) {
		if ((buf->length < vout->pix.sizeimage) ||
			(0 == buf->m.userptr)) {
			return -EINVAL;
		}
	}

	rv = videobuf_qbuf(q, buf);

	mutex_lock(&vout->lock);
	if (vout->streaming && vout->acquire_timeout_ms) {
		del_timer(&vout->acquire_timer);
		mod_timer(&vout->acquire_timer,
			jiffies + msecs_to_jiffies(vout->acquire_timeout_ms));
	}
	mutex_unlock(&vout->lock);

	GFXLOG(2, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_dqbuf(struct file *file, void *fh,
			struct v4l2_buffer *buf)
{
	struct v4gfx_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int rv = 0;
	int nonblocking = file->f_flags & O_NONBLOCK ? 1 : 0;

	GFXLOG(2, V4L2DEV(vout), "dqbuf buf: %x (%d)\n",
			(int)buf, nonblocking);

	mutex_lock(&vout->lock);
	if (!vout->streaming) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}

	mutex_unlock(&vout->lock);

#if V4GFX_WAITMETHOD == V4GFX_WAIT_DEQUE
{
	struct videobuf_buffer *sync_frame = NULL;

	wait_event_interruptible(vout->sync_done,
						!list_empty(&vout->sync_queue));

	sync_frame = v4gfx_get_next_syncframe(vout);

	if (sync_frame) {
		(void)v4gfx_wait_on_pending(vout, sync_frame->i);
		v4gfx_done_syncframe(vout, sync_frame);
	} else {
		/* Can be from an interrupted task */
		printk(KERN_INFO "No sync frame\n");
	}
}
#endif

	rv = videobuf_dqbuf(q, buf, nonblocking);

	GFXLOG(2, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_streamon(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct v4gfx_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int rv = 0;
    GFXLOG(1, V4L2DEV(vout), "+%s\n", __func__);

	mutex_lock(&vout->lock);

	if (vout->streaming) {
		rv = -EBUSY;
		goto end_unlock;
	}

	vout->cur_frm = NULL;
	vout->locked_frm = NULL;

	rv = videobuf_streamon(q);
	if (rv < 0)
		goto end_unlock;

	if (list_empty(&vout->dma_queue)) {
		rv = -EIO;
		goto end_unlock;
	}

	vout->streaming = true;

	/* Activate the next current buffer */
	vout->cur_frm =
		list_entry(vout->dma_queue.next, struct videobuf_buffer, queue);
	list_del(&vout->cur_frm->queue);
	vout->cur_frm->state = VIDEOBUF_ACTIVE;

	set_bit(1, &vout->producer_ready);
	wake_up_interruptible(&vout->consumer_wait);

end_unlock:
	mutex_unlock(&vout->lock);
	GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);

	return rv;
}

static int vidioc_streamoff(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct v4gfx_device *vout = fh;
	int rv;

	mutex_lock(&vout->lock);
	if (!vout->streaming) {
		rv = -EINVAL;
		goto end;
	}

	del_timer_sync(&vout->acquire_timer);
	clear_bit(1, &vout->acquire_timedout);

	clear_bit(1, &vout->producer_ready);

	vout->streaming = false;

	INIT_LIST_HEAD(&vout->dma_queue);
	INIT_LIST_HEAD(&vout->sync_queue);

	videobuf_streamoff(&vout->vbq);
	videobuf_queue_cancel(&vout->vbq);
end:
	mutex_unlock(&vout->lock);
	GFXLOG(1, V4L2DEV(vout), "-%s [%d]\n", __func__, rv);
	return rv;
}

static int vidioc_cropcap(struct file *file, void *fh,
		struct v4l2_cropcap *cropcap)
{
	struct v4gfx_device *vout = fh;
	struct v4l2_pix_format *pix = &vout->pix;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Width and height are always even */
	cropcap->bounds.width = pix->width & ~1;
	cropcap->bounds.height = pix->height & ~1;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct v4gfx_device *vout = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = vout->crop;
	GFXLOG(1, V4L2DEV(vout), "g_crop w:%d,h:%d\n",
			crop->c.width, crop->c.height);
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct v4gfx_device *vout = fh;
	GFXLOG(1, V4L2DEV(vout), "Entering %s\n", __func__);
	vout->crop = crop->c;
	return 0;
}

static long vidioc_default(struct file *file, void *fh, int cmd, void *arg)
{
	int rv = 0;
	struct v4gfx_device *vout = fh;
	GFXLOG(1, V4L2DEV(vout), "Entering %s (c=0x%x)\n", __func__, cmd);

	switch (cmd) {
	case V4L2_GFX_IOC_CONSUMER:
	{
		struct v4l2_gfx_consumer_params *parms =
					(struct v4l2_gfx_consumer_params *)arg;
		if (parms->type != V4L2_GFX_CONSUMER_WAITSTREAM)
			return -EINVAL;

		clear_bit(1, &vout->acquire_timedout);

		rv = wait_event_interruptible(vout->consumer_wait,
					test_bit(1, &vout->producer_ready));
		mutex_lock(&vout->lock);
		if (rv == -ERESTARTSYS) {
			/*
			 * This condition is hit when the user process
			 * generates a signal, when we return this value the
			 * process will continue to block on the ioctl
			 */
			GFXLOG(1, V4L2DEV(vout), "Woke by signal: %d\n",
								ERESTARTSYS);
		} else {
			vout->acquire_timeout_ms = parms->acquire_timeout_ms;
		}
		mutex_unlock(&vout->lock);
		break;

	}
	case V4L2_GFX_IOC_INFO:
	{
		struct v4l2_gfx_info_params *parms =
					(struct v4l2_gfx_info_params *)arg;
		parms->opencnt = vout->opened;
		break;
	}
	case V4L2_GFX_IOC_PRODUCER:
	{
		struct v4l2_gfx_producer_params *parms =
				(struct v4l2_gfx_producer_params *)arg;
		vout->producer_flags = parms->flags;
		if (!(vout->producer_flags & V4L2_GFX_PRODUCER_MASK_OPEN)) {
			/*
			 * We decrement the count here because the Android
			 * mediaserver threads won't close the V4L2 device
			 */
			if (vout->opened)
				vout->opened--;
		}
		break;
	}
	case V4L2_GFX_IOC_ACQ:
	{
		struct v4l2_gfx_buf_params *parms =
					(struct v4l2_gfx_buf_params *)arg;
		int bufid = -1;
		int timedout;
		rv = v4gfx_frame_lock(vout, &bufid);
		if (!rv) {
			parms->bufid = bufid;
			parms->crop_top = vout->crop.top;
			parms->crop_left = vout->crop.left;
			parms->crop_width = vout->crop.width;
			parms->crop_height = vout->crop.height;
			GFXLOG(3, V4L2DEV(vout), "%d:%d:%d:%d:%d\n",
					parms->bufid ,
					parms->crop_top ,
					parms->crop_left ,
					parms->crop_width ,
					parms->crop_height);
		}
		timedout = test_and_clear_bit(1, &vout->acquire_timedout);
		if (timedout) {
			GFXLOG(1, V4L2DEV(vout), "ACQ Timed out\n");
			rv = -ETIMEDOUT;
		}
		mutex_lock(&vout->lock);
		if (!vout->streaming) {
			GFXLOG(1, V4L2DEV(vout), "ACQ stream off\n");
			rv = -ENODEV;
		}
		mutex_unlock(&vout->lock);
		break;
	}
	case V4L2_GFX_IOC_REL:
	{
		struct v4l2_gfx_buf_params *parms =
					(struct v4l2_gfx_buf_params *)arg;
		int bufid = parms->bufid;
		rv = v4gfx_frame_unlock(vout, bufid);
		break;
	}
	default:
		rv = -EINVAL;
	}
	GFXLOG(1, V4L2DEV(vout), "Leaving %s (%d)\n", __func__, rv);
	return rv;
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct v4gfx_device *vout = fh;
	GFXLOG(1, V4L2DEV(vout), "%s: %d\n", __func__, a->id);
	return 0;
}

struct v4l2_ioctl_ops v4gfx_ioctl_ops = {
	.vidioc_querycap			= vidioc_querycap,
	.vidioc_log_status			= vidioc_log_status,
	.vidioc_enum_fmt_vid_out		= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out			= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out			= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out			= vidioc_s_fmt_vid_out,
	.vidioc_reqbufs				= vidioc_reqbufs,
	.vidioc_querybuf			= vidioc_querybuf,
	.vidioc_qbuf				= vidioc_qbuf,
	.vidioc_dqbuf				= vidioc_dqbuf,
	.vidioc_streamon			= vidioc_streamon,
	.vidioc_streamoff			= vidioc_streamoff,
	.vidioc_cropcap				= vidioc_cropcap,
	.vidioc_g_crop				= vidioc_g_crop,
	.vidioc_s_crop				= vidioc_s_crop,
	.vidioc_default				= vidioc_default,
	.vidioc_s_ctrl				= vidioc_s_ctrl,
};

const struct v4l2_file_operations v4gfx_fops =  {
	.owner		= THIS_MODULE,
	.ioctl		= video_ioctl2,
	.mmap		= vidfop_mmap,
	.open		= vidfop_open,
	.release	= vidfop_release,
};

