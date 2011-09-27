/**
 * Copyright (c) 2010 Trusted Logic S.A.
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

#include <asm/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pm.h>
#include <linux/sysdev.h>
#include <linux/vmalloc.h>
#include <linux/signal.h>
#ifdef CONFIG_ANDROID
#include <linux/device.h>
#endif
#include <linux/init.h>
#include <linux/bootmem.h>

#include "tf_protocol.h"
#include "tf_defs.h"
#include "tf_util.h"
#include "tf_conn.h"
#include "tf_comm.h"
#include "tf_zebra.h"

#include "s_version.h"

#define TF_PA_CTRL_START		0x1
#define TF_PA_CTRL_STOP		0x2

#ifdef CONFIG_ANDROID
static struct class *tf_ctrl_class;
#endif

#define TF_DEVICE_CTRL_BASE_NAME "tf_ctrl"

struct tf_pa_ctrl {
	u32 nPACommand;

	u32 pa_size;
	u8 *pa_buffer;

	u32 conf_size;
	u8 *conf_buffer;
};

static int tf_ctrl_check_omap_type(void)
{
	/* No need to do anything on a GP device */
	switch (omap_type()) {
	case OMAP2_DEVICE_TYPE_GP:
		dprintk(KERN_INFO "SMC: Running on a GP device\n");
		return 0;

	case OMAP2_DEVICE_TYPE_EMU:
	case OMAP2_DEVICE_TYPE_SEC:
	/*case OMAP2_DEVICE_TYPE_TEST:*/
		dprintk(KERN_INFO "SMC: Running on a EMU or HS device\n");
		return 1;

	default:
		printk(KERN_ERR "SMC: unknown omap type %x\n", omap_type());
		return -EFAULT;
	}
}

#define IOCTL_TF_PA_CTRL _IOWR('z', 0xFF, struct tf_pa_ctrl)

static long tf_ctrl_device_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	int result = S_SUCCESS;
	struct tf_pa_ctrl pa_ctrl;
	u8 *pa_buffer = NULL;
	u8 *conf_buffer = NULL;
	struct tf_device *dev = tf_get_device();

	dprintk(KERN_INFO "tf_ctrl_device_ioctl(%p, %u, %p)\n",
		file, ioctl_num, (void *) ioctl_param);

	mutex_lock(&dev->dev_mutex);

	if (ioctl_num != IOCTL_TF_PA_CTRL) {
		dprintk(KERN_ERR "tf_ctrl_device_ioctl(%p): "
			"ioctl number is invalid (%p)\n",
			file, (void *)ioctl_num);

		result = -EFAULT;
		goto exit;
	}

	if ((ioctl_param & 0x3) != 0) {
		dprintk(KERN_ERR "tf_ctrl_device_ioctl(%p): "
			"ioctl command message pointer is not word "
			"aligned (%p)\n",
			file, (void *)ioctl_param);

		result = -EFAULT;
		goto exit;
	}

	if (copy_from_user(&pa_ctrl, (struct tf_pa_ctrl *)ioctl_param,
			sizeof(struct tf_pa_ctrl))) {
		dprintk(KERN_ERR "tf_ctrl_device_ioctl(%p): "
			"cannot access ioctl parameter (%p)\n",
			file, (void *)ioctl_param);

		result = -EFAULT;
		goto exit;
	}

	switch (pa_ctrl.nPACommand) {
	case TF_PA_CTRL_START:
		dprintk(KERN_INFO "tf_ctrl_device_ioctl(%p): "
			"Start the SMC PA (%d bytes) with conf (%d bytes)\n",
			file, pa_ctrl.pa_size, pa_ctrl.conf_size);

		pa_buffer = (u8 *) internal_kmalloc(pa_ctrl.pa_size,
						GFP_KERNEL);
		if (pa_buffer == NULL) {
			dprintk(KERN_ERR "tf_ctrl_device_ioctl(%p): "
				"Out of memory for PA buffer\n", file);

			result = -ENOMEM;
			goto exit;
		}

		if (copy_from_user(
			pa_buffer, pa_ctrl.pa_buffer, pa_ctrl.pa_size)) {
			dprintk(KERN_ERR "tf_ctrl_device_ioctl(%p): "
				"Cannot access PA buffer (%p)\n",
				file, (void *) pa_ctrl.pa_buffer);

			internal_kfree(pa_buffer);

			result = -EFAULT;
			goto exit;
		}

		if (pa_ctrl.conf_size > 0) {
			conf_buffer = (u8 *) internal_kmalloc(
				pa_ctrl.conf_size, GFP_KERNEL);
			if (conf_buffer == NULL) {
				internal_kfree(pa_buffer);

				result = -ENOMEM;
				goto exit;
			}

			if (copy_from_user(conf_buffer,
				pa_ctrl.conf_buffer, pa_ctrl.conf_size)) {
				internal_kfree(pa_buffer);
				internal_kfree(conf_buffer);

				result = -EFAULT;
				goto exit;
			}
		}

		if (dev->workspace_addr == 0) {
			result = -ENOMEM;
			goto exit;
		}

		result = tf_start(&dev->sm,
			dev->workspace_addr,
			dev->workspace_size,
			pa_buffer,
			pa_ctrl.pa_size,
			conf_buffer,
			pa_ctrl.conf_size);
		if (result)
			dprintk(KERN_ERR "SMC: start failed\n");
		else
			dprintk(KERN_INFO "SMC: started\n");

		internal_kfree(pa_buffer);
		internal_kfree(conf_buffer);
		break;

	case TF_PA_CTRL_STOP:
		dprintk(KERN_INFO "tf_ctrl_device_ioctl(%p): "
			"Stop the SMC PA\n", file);

		result = tf_power_management(&dev->sm,
			TF_POWER_OPERATION_SHUTDOWN);
		if (result)
			dprintk(KERN_WARNING "SMC: stop failed [0x%x]\n",
				result);
		else
			dprintk(KERN_INFO "SMC: stopped\n");
		break;

	default:
		result = -EOPNOTSUPP;
		break;
	}

exit:
	mutex_unlock(&dev->dev_mutex);
	return result;
}

/*----------------------------------------------------------------------------*/

static int tf_ctrl_device_open(struct inode *inode, struct file *file)
{
	int error;

	dprintk(KERN_INFO "tf_ctrl_device_open(%u:%u, %p)\n",
		imajor(inode), iminor(inode), file);

	/* Dummy lseek for non-seekable driver */
	error = nonseekable_open(inode, file);
	if (error != 0) {
		dprintk(KERN_ERR "tf_ctrl_device_open(%p): "
			"nonseekable_open failed (error %d)!\n",
			file, error);
		goto error;
	}

#ifndef CONFIG_ANDROID
	/*
	 * Check file flags. We only autthorize the O_RDWR access
	 */
	if (file->f_flags != O_RDWR) {
		dprintk(KERN_ERR "tf_ctrl_device_open(%p): "
			"Invalid access mode %u\n",
			file, file->f_flags);
		error = -EACCES;
		goto error;
	}
#endif

	error = tf_ctrl_check_omap_type();
	if (error <= 0)
		return error;

	/*
	 * Successful completion.
	 */

	dprintk(KERN_INFO "tf_ctrl_device_open(%p): Success\n", file);
	return 0;

	/*
	 * Error handling.
	 */
error:
	dprintk(KERN_INFO "tf_ctrl_device_open(%p): Failure (error %d)\n",
		file, error);
	return error;
}

static const struct file_operations g_tf_ctrl_device_file_ops = {
	.owner = THIS_MODULE,
	.open = tf_ctrl_device_open,
	.unlocked_ioctl = tf_ctrl_device_ioctl,
	.llseek = no_llseek,
};

int __init tf_ctrl_device_register(void)
{
	int error;
	struct tf_device *dev = tf_get_device();

	cdev_init(&dev->cdev_ctrl, &g_tf_ctrl_device_file_ops);
	dev->cdev_ctrl.owner = THIS_MODULE;

	error = register_chrdev_region(dev->dev_number + 1, 1,
		TF_DEVICE_CTRL_BASE_NAME);
	if (error)
		return error;

	error = cdev_add(&dev->cdev_ctrl,
		dev->dev_number + 1, 1);
	if (error) {
		cdev_del(&(dev->cdev_ctrl));
		unregister_chrdev_region(dev->dev_number + 1, 1);
		return error;
	}

#ifdef CONFIG_ANDROID
	tf_ctrl_class = class_create(THIS_MODULE, TF_DEVICE_CTRL_BASE_NAME);
	device_create(tf_ctrl_class, NULL,
		dev->dev_number + 1,
		NULL, TF_DEVICE_CTRL_BASE_NAME);
#endif

	mutex_init(&dev->dev_mutex);

	return error;
}

static int __initdata smc_mem;
static int __initdata smc_address;

void __init tf_allocate_workspace(void)
{
	struct tf_device *dev = tf_get_device();

	if (tf_ctrl_check_omap_type() <= 0)
		return;

	dev->workspace_size = smc_mem;
	if (dev->workspace_size < 3*SZ_1M)
		dev->workspace_size = 3*SZ_1M;

	if (smc_address == 0)
#if 0
		dev->workspace_addr = (u32) __pa(__alloc_bootmem(
			dev->workspace_size, SZ_1M, __pa(MAX_DMA_ADDRESS)));
#else
		dev->workspace_addr = (u32) 0xBFD00000;
#endif
	else
		dev->workspace_addr = smc_address;

	pr_info("SMC: Allocated workspace of 0x%x Bytes at (0x%x)\n",
		dev->workspace_size,
		dev->workspace_addr);
}

static int __init tf_mem_setup(char *str)
{
	smc_mem = memparse(str, &str);
	if (*str == '@') {
		str += 1;
		get_option(&str, &smc_address);
	}
	return 0;
}

early_param("smc_mem", tf_mem_setup);
