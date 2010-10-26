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
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <linux/omap_v4l2_gfx.h>	/* private ioctls */

#include <media/v4l2-ioctl.h>

#include "v4gfx.h"
#include "gfx_bc.h"

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP V4L2 GFX driver");
MODULE_LICENSE("GPL");

/*
 * Device node will be: /dev/video<VOUT_DEVICENODE_SUFFIX>
 * See also /sys/devices/virtual/video4linux/<node>/name which will be
 * whatever the value of VOUT_NAME is
 */
#define VOUT_DEVICENODE_SUFFIX 100

static struct gbl_v4gfx *gbl_dev;

int debug;	/* is used outside this compilation unit too */
module_param(debug, int, 0644);

/*
 * If bypass is set then buffer streaming operations will be bypassed. This
 * enables us to check what the raw performance of stack above the V4L2
 * driver is
 */
static int bypass;
module_param(bypass, int, 0644);


static int bypass_vidioc_qbuf(
	struct file *file, void *fh, struct v4l2_buffer *buf) { return 0; }

static int bypass_vidioc_dqbuf(
	struct file *file, void *fh, struct v4l2_buffer *buf) { return 0; }

static int bypass_vidioc_streamon(
	struct file *file, void *fh, enum v4l2_buf_type i) { return 0; }

static int bypass_vidioc_streamoff(
	struct file *file, void *fh, enum v4l2_buf_type i) { return 0; }

static long bypass_vidioc_default(
	struct file *file, void *fh, int cmd, void *arg)
{
	struct v4l2_gfx_buf_params *parms = (struct v4l2_gfx_buf_params *)arg;
	int rv = 0;

	switch (cmd) {
	case V4L2_GFX_IOC_CONSUMER:
		break;
	case V4L2_GFX_IOC_ACQ:
		/* In bypass mode default the first buffer */
		parms->bufid = 0;
		break;
	case V4L2_GFX_IOC_REL:
		break;
	default:
		rv = -EINVAL;
	}
	return rv;
}

/*
 * If the module is put in bypass mode the following ioctls
 * are effectively nops
 */
static void v4gfx_enable_bypass(void)
{
	v4gfx_ioctl_ops.vidioc_qbuf		= bypass_vidioc_qbuf;
	v4gfx_ioctl_ops.vidioc_dqbuf		= bypass_vidioc_dqbuf;
	v4gfx_ioctl_ops.vidioc_streamon	= bypass_vidioc_streamon;
	v4gfx_ioctl_ops.vidioc_streamoff	= bypass_vidioc_streamoff;
	v4gfx_ioctl_ops.vidioc_default	= bypass_vidioc_default;
}

static void v4gfx_cleanup_device(struct v4gfx_device *vout)
{
	struct video_device *vfd;

	if (!vout)
		return;
	vfd = vout->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
	}

	v4gfx_tiler_buffer_free(vout, vout->buffer_allocated, 0);
	kfree(vout);
}

static int driver_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct gbl_v4gfx *dev = container_of(v4l2_dev, struct
			gbl_v4gfx, v4l2_dev);
	int k;

	v4l2_device_unregister(v4l2_dev);
	for (k = 0; k < pdev->num_resources; k++)
		v4gfx_cleanup_device(dev->vouts[k]);

	kfree(gbl_dev);
	return 0;
}

static int driver_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "Probing: " VOUT_NAME);
	return 0;
}

static int v4gfx_create_instance(struct v4gfx_device **voutp)
{
	int r = 0;
	struct v4gfx_device *vout = NULL;
	struct video_device *vfd = NULL;

	vout = kzalloc(sizeof(struct v4gfx_device), GFP_KERNEL);
	if (vout == NULL) {
		r = -ENOMEM;
		goto end;
	}
	mutex_init(&vout->lock);
	spin_lock_init(&vout->vbq_lock);
	/* TODO set this to an invalid value, need to change unit test though */
	vout->bpp = RGB565_BPP;
	vout->gbl_dev = gbl_dev;
	vout->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	init_timer(&vout->acquire_timer);
	vout->acquire_timer.function = v4gfx_acquire_timer;
	vout->acquire_timer.data = (unsigned long)vout;

    init_waitqueue_head(&vout->sync_done);
    init_waitqueue_head(&vout->consumer_wait);

    vfd = vout->vfd = video_device_alloc();
	if (!vfd)
		goto end;

	strlcpy(vfd->name, VOUT_NAME, sizeof(vfd->name));
	vfd->vfl_type = VFL_TYPE_GRABBER;
	vfd->release = video_device_release;
	vfd->ioctl_ops = &v4gfx_ioctl_ops;
	vfd->fops = &v4gfx_fops;
	vfd->minor = -1;
	vfd->debug = debug;

	r = video_register_device(vfd, VFL_TYPE_GRABBER,
				  VOUT_DEVICENODE_SUFFIX);
	if (r < 0)
		goto end;

	video_set_drvdata(vfd, vout);

	*voutp = vout;
	printk(KERN_INFO VOUT_NAME ":video device registered\n");
	return 0;
end:

	if (vfd)
		video_device_release(vfd);

	kfree(vout); /* safe with null vout */

	return r;
}

static void v4gfx_delete_instance(
	struct v4l2_device *v4l2_dev, struct v4gfx_device *vout)
{
	v4l2_info(v4l2_dev, "unregistering /dev/video%d\n", vout->vfd->num);
	video_unregister_device(vout->vfd);
	v4gfx_buffer_array_free(vout, vout->buffer_allocated);
	kfree(vout);
	return;
}

static struct platform_driver v4gfx_driver = {
	.driver = {
		   .name = VOUT_NAME,
		   },
	.probe = driver_probe,
	.remove = driver_remove,
};

static int module_init_v4gfx(void)
{
	int rv;
	bool v4l2_dev_registered = false;
	bool bc_dev_registered = false;

	if (bypass) {
		printk(KERN_INFO VOUT_NAME ":Enable bypass mode\n");
		v4gfx_enable_bypass();
	}

	rv = platform_driver_register(&v4gfx_driver);
	if (rv != 0) {
		printk(KERN_ERR VOUT_NAME ":platform_driver_register failed\n");
		goto end;
	}

	gbl_dev = kzalloc(sizeof(struct gbl_v4gfx), GFP_KERNEL);
	if (gbl_dev == NULL) {
		rv = -ENOMEM;
		goto end;
	}

	snprintf(gbl_dev->v4l2_dev.name, sizeof(gbl_dev->v4l2_dev.name),
			"%s-%03d", VOUT_NAME, VOUT_DEVICENODE_SUFFIX);

	rv = v4l2_device_register(NULL, &gbl_dev->v4l2_dev);
	if (rv != 0) {
		printk(KERN_ERR VOUT_NAME ":v4l2_device_register failed\n");
		goto end;
	}
	v4l2_dev_registered = true;

	rv = v4gfx_create_instance(&gbl_dev->vouts[0]);
	if (rv != 0)
		goto end;

	rv = bc_init();
	if (rv != 0)
		goto end;

	bc_dev_registered = true;

	printk(KERN_INFO VOUT_NAME ":OMAP V4L2 GFX driver loaded ok\n");
	return rv;
end:
	printk(KERN_INFO VOUT_NAME ":Error %d loading OMAP V4L2 GFX driver\n",
									rv);

	if (bc_dev_registered)
		bc_cleanup();

	if (v4l2_dev_registered)
		v4l2_device_unregister(&gbl_dev->v4l2_dev);

	kfree(gbl_dev); /* gbl_dev can be null */

	return rv;
}

static void module_exit_v4gfx(void)
{
	bc_cleanup();

	v4gfx_delete_instance(&gbl_dev->v4l2_dev, gbl_dev->vouts[0]);

	v4l2_device_unregister(&gbl_dev->v4l2_dev);

	kfree(gbl_dev);

	platform_driver_unregister(&v4gfx_driver);
}

module_init(module_init_v4gfx);
module_exit(module_exit_v4gfx);
