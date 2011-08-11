/*
 * Secure Mode Input interface to remoteproc driver
 *
 * Copyright (C) 2011 Texas Instruments. All rights reserved.
 *
 * Authors: Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <linux/remoteproc.h>


#define RPROC_USER_NAME		"rproc_user"
#define RPROC_USER_DEVICES	1

static DEFINE_MUTEX(rproc_user_mutex);

struct rproc_user_device {
	struct miscdevice mdev;
};

static struct rproc_user_device *ipu_device;
static char *rproc_user_name = RPROC_USER_NAME;
static bool secure_mode;
static bool secure_attempt;


static int rproc_user_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int rproc_user_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t rproc_user_read(struct file *filp, char __user *ubuf,
                                                size_t len, loff_t *offp)
{
	u8 enable;
	int ret = 1;

	if (len != 1)
		return -EINVAL;

	if (mutex_lock_interruptible(&rproc_user_mutex))
		return -EINTR;
	enable = secure_mode ? 1 : 0;
	if (copy_to_user((void *)ubuf, &enable, sizeof(enable)))
		ret = -EFAULT;
	mutex_unlock(&rproc_user_mutex);

	return ret;
}

static ssize_t rproc_user_write(struct file *filp, const char __user *ubuf,
                                                size_t len, loff_t *offp)
{
	int ret;
	u8 enable;

	if (len != 1)
		return -EINVAL;

	//enable = !(*(u8 *)ubuf == 0);
	if (copy_from_user(&enable, (char __user *) ubuf, sizeof(enable)))
		return -EFAULT;
	enable = !(enable == 0);

	if (mutex_lock_interruptible(&rproc_user_mutex))
		return -EINTR;
	if (enable && !secure_mode) {
		ret = rproc_set_secure("ipu", enable);
		if (!ret)
			secure_mode = enable;
		else
			pr_err("rproc secure start failed, 0x%x\n", ret);
		secure_attempt = enable;
	} else if (!enable && secure_attempt) {
		ret = rproc_set_secure("ipu", enable);
		if (ret)
			pr_err("rproc normal start failed 0x%x, urghh!!", ret);
		secure_mode = enable;
		secure_attempt = enable;
	} else
		ret = -EINVAL;
	mutex_unlock(&rproc_user_mutex);

	return ret ? ret : 1;
}

static const struct file_operations rproc_user_fops = {
	.owner		= THIS_MODULE,
	.open		= rproc_user_open,
	.release	= rproc_user_release,
	.read		= rproc_user_read,
	.write		= rproc_user_write,
};

static int __init rproc_user_init(void)
{
	int ret;

	ipu_device = kzalloc(sizeof(struct rproc_user_device), GFP_KERNEL);
	if (!ipu_device) {
		pr_err("%s: memory allocation failed for ipu_device\n",
					__func__);
		ret = -ENOMEM;
		goto exit;
	}

	ipu_device->mdev.minor = MISC_DYNAMIC_MINOR;
	ipu_device->mdev.name = rproc_user_name;
	ipu_device->mdev.fops = &rproc_user_fops;
	ipu_device->mdev.parent = NULL;
	ret = misc_register(&ipu_device->mdev);
	if (ret) {
		pr_err("rproc_user_init: failed to register rproc_user misc "
			"device\n");
		goto misc_fail;
	}
	return ret;

misc_fail:
	kfree(ipu_device);
exit:
	return ret;
}
module_init(rproc_user_init);

static void __exit rproc_user_exit(void)
{
	misc_deregister(&ipu_device->mdev);
	kfree(ipu_device);
}
module_exit(rproc_user_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RemoteProc Secure Mode Interface Driver");
MODULE_AUTHOR("Suman Anna");
