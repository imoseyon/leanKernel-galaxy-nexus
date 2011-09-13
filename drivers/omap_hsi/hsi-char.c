/*
 * hsi-char.c
 *
 * HSI character device driver, implements the character device
 * interface.
 *
 * Copyright (C) 2009 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Andras Domokos <andras.domokos@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/mach-types.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/hsi_driver_if.h>
#include <linux/hsi_char.h>

#include <plat/omap_hsi.h>

#include "hsi-char.h"

#define DRIVER_VERSION  "0.2.1"
#define HSI_CHAR_DEVICE_NAME  "hsi_char"

static unsigned int port = 1;
module_param(port, uint, 1);
MODULE_PARM_DESC(port, "HSI port to be probed");

static unsigned int num_channels;
static unsigned int channels_map[HSI_MAX_CHAR_DEVS] = { 0 };
module_param_array(channels_map, uint, &num_channels, 0);
MODULE_PARM_DESC(channels_map, "HSI channels to be probed");

dev_t hsi_char_dev;

struct char_queue {
	struct list_head list;
	u32 *data;
	unsigned int count;
};

struct hsi_char {
	unsigned int opened;
	int poll_event;
	struct list_head rx_queue;
	struct list_head tx_queue;
	spinlock_t lock;	/* Serialize access to driver data and API */
	struct fasync_struct *async_queue;
	wait_queue_head_t rx_wait;
	wait_queue_head_t tx_wait;
	wait_queue_head_t poll_wait;
};

static struct hsi_char hsi_char_data[HSI_MAX_CHAR_DEVS];

void if_hsi_notify(int ch, struct hsi_event *ev)
{
	struct char_queue *entry;

	pr_debug("%s, ev = {0x%x, 0x%p, %u}\n", __func__, ev->event, ev->data,
		 ev->count);

	spin_lock(&hsi_char_data[ch].lock);

	if (!hsi_char_data[ch].opened) {
		pr_debug("%s, device not opened\n!", __func__);
		spin_unlock(&hsi_char_data[ch].lock);
		return;
	}

	switch (HSI_EV_TYPE(ev->event)) {
	case HSI_EV_IN:
		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!entry) {
			pr_err("HSI-CHAR: entry allocation failed.\n");
			spin_unlock(&hsi_char_data[ch].lock);
			return;
		}
		entry->data = ev->data;
		entry->count = ev->count;
		list_add_tail(&entry->list, &hsi_char_data[ch].rx_queue);
		spin_unlock(&hsi_char_data[ch].lock);
		pr_debug("%s, HSI_EV_IN\n", __func__);
		wake_up_interruptible(&hsi_char_data[ch].rx_wait);
		break;
	case HSI_EV_OUT:
		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!entry) {
			pr_err("HSI-CHAR: entry allocation failed.\n");
			spin_unlock(&hsi_char_data[ch].lock);
			return;
		}
		entry->data = ev->data;
		entry->count = ev->count;
		hsi_char_data[ch].poll_event |= (POLLOUT | POLLWRNORM);
		list_add_tail(&entry->list, &hsi_char_data[ch].tx_queue);
		spin_unlock(&hsi_char_data[ch].lock);
		pr_debug("%s, HSI_EV_OUT\n", __func__);
		wake_up_interruptible(&hsi_char_data[ch].tx_wait);
		break;
	case HSI_EV_EXCEP:
		hsi_char_data[ch].poll_event |= POLLPRI;
		spin_unlock(&hsi_char_data[ch].lock);
		pr_debug("%s, HSI_EV_EXCEP\n", __func__);
		wake_up_interruptible(&hsi_char_data[ch].poll_wait);
		break;
	case HSI_EV_AVAIL:
		hsi_char_data[ch].poll_event |= (POLLIN | POLLRDNORM);
		spin_unlock(&hsi_char_data[ch].lock);
		pr_debug("%s, HSI_EV_AVAIL\n", __func__);
		wake_up_interruptible(&hsi_char_data[ch].poll_wait);
		break;
	default:
		spin_unlock(&hsi_char_data[ch].lock);
		break;
	}
}

static int hsi_char_fasync(int fd, struct file *file, int on)
{
	int ch = (int)file->private_data;
	if (fasync_helper(fd, file, on, &hsi_char_data[ch].async_queue) >= 0)
		return 0;
	else
		return -EIO;
}

static unsigned int hsi_char_poll(struct file *file, poll_table * wait)
{
	int ch = (int)file->private_data;
	unsigned int ret = 0;

	/*printk(KERN_DEBUG "%s\n", __func__); */

	poll_wait(file, &hsi_char_data[ch].poll_wait, wait);
	poll_wait(file, &hsi_char_data[ch].tx_wait, wait);
	spin_lock_bh(&hsi_char_data[ch].lock);
	ret = hsi_char_data[ch].poll_event;
	spin_unlock_bh(&hsi_char_data[ch].lock);

	pr_debug("%s, ret = 0x%x\n", __func__, ret);
	return ret;
}

static ssize_t hsi_char_read(struct file *file, char __user *buf,
			     size_t count, loff_t *ppos)
{
	int ch = (int)file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	u32 *data;
	unsigned int data_len;
	struct char_queue *entry;
	ssize_t ret;

	/*printk(KERN_DEBUG "%s, count = %d\n", __func__, count); */

	/* only 32bit data is supported for now */
	if ((count < 4) || (count & 3))
		return -EINVAL;

	data = kmalloc(count, GFP_ATOMIC);

	ret = if_hsi_read(ch, data, count);
	if (ret < 0) {
		kfree(data);
		goto out2;
	}

	spin_lock_bh(&hsi_char_data[ch].lock);
	add_wait_queue(&hsi_char_data[ch].rx_wait, &wait);
	spin_unlock_bh(&hsi_char_data[ch].lock);

	for (;;) {
		data = NULL;
		data_len = 0;

		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_bh(&hsi_char_data[ch].lock);
		if (!list_empty(&hsi_char_data[ch].rx_queue)) {
			entry = list_entry(hsi_char_data[ch].rx_queue.next,
					   struct char_queue, list);
			data = entry->data;
			data_len = entry->count;
			list_del(&entry->list);
			kfree(entry);
		}
		spin_unlock_bh(&hsi_char_data[ch].lock);

		pr_debug("%s, data = 0x%p, data_len = %d\n",
			 __func__, data, data_len);

		if (data_len) {
			pr_debug("%s, RX finished\n", __func__);
			spin_lock_bh(&hsi_char_data[ch].lock);
			hsi_char_data[ch].poll_event &= ~(POLLIN | POLLRDNORM);
			spin_unlock_bh(&hsi_char_data[ch].lock);
			if_hsi_poll(ch);
			break;
		} else if (file->f_flags & O_NONBLOCK) {
			pr_debug("%s, O_NONBLOCK\n", __func__);
			ret = -EAGAIN;
			goto out;
		} else if (signal_pending(current)) {
			pr_debug("%s, ERESTARTSYS\n", __func__);
			ret = -EAGAIN;
			if_hsi_cancel_read(ch);
			/* goto out; */
			break;
		}

		/*printk(KERN_DEBUG "%s, going to sleep...\n", __func__); */
		schedule();
		/*printk(KERN_DEBUG "%s, woke up\n", __func__); */
	}

	if (data_len) {
		ret = copy_to_user((void __user *)buf, data, data_len);
		if (!ret)
			ret = data_len;
	}

	kfree(data);

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&hsi_char_data[ch].rx_wait, &wait);

out2:
	/*printk(KERN_DEBUG "%s, ret = %d\n", __func__, ret); */
	return ret;
}

static ssize_t hsi_char_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	int ch = (int)file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	u32 *data;
	unsigned int data_len = 0;
	struct char_queue *entry;
	ssize_t ret;

	/*printk(KERN_DEBUG "%s, count = %d\n", __func__, count); */

	/* only 32bit data is supported for now */
	if ((count < 4) || (count & 3))
		return -EINVAL;

	data = kmalloc(count, GFP_ATOMIC);
	if (!data) {
		WARN_ON(1);
		return -ENOMEM;
	}
	if (copy_from_user(data, (void __user *)buf, count)) {
		ret = -EFAULT;
		kfree(data);
		goto out2;
	} else {
		ret = count;
	}

	ret = if_hsi_write(ch, data, count);
	if (ret < 0) {
		kfree(data);
		goto out2;
	}
	spin_lock_bh(&hsi_char_data[ch].lock);
	hsi_char_data[ch].poll_event &= ~(POLLOUT | POLLWRNORM);
	add_wait_queue(&hsi_char_data[ch].tx_wait, &wait);
	spin_unlock_bh(&hsi_char_data[ch].lock);

	for (;;) {
		data = NULL;
		data_len = 0;

		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_bh(&hsi_char_data[ch].lock);
		if (!list_empty(&hsi_char_data[ch].tx_queue)) {
			entry = list_entry(hsi_char_data[ch].tx_queue.next,
					   struct char_queue, list);
			data = entry->data;
			data_len = entry->count;
			list_del(&entry->list);
			kfree(entry);
		}
		spin_unlock_bh(&hsi_char_data[ch].lock);

		if (data_len) {
			pr_debug("%s, TX finished\n", __func__);
			ret = data_len;
			break;
		} else if (file->f_flags & O_NONBLOCK) {
			pr_debug("%s, O_NONBLOCK\n", __func__);
			ret = -EAGAIN;
			goto out;
		} else if (signal_pending(current)) {
			pr_debug("%s, ERESTARTSYS\n", __func__);
			ret = -ERESTARTSYS;
			goto out;
		}

		/*printk(KERN_DEBUG "%s, going to sleep...\n", __func__); */
		schedule();
		/*printk(KERN_DEBUG "%s, woke up\n", __func__); */
	}

	kfree(data);

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&hsi_char_data[ch].tx_wait, &wait);

out2:
	/*printk(KERN_DEBUG "%s, ret = %d\n", __func__, ret); */
	return ret;
}

static long  hsi_char_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int ch = (int)file->private_data;
	unsigned int state;
	size_t occ;
	struct hsi_rx_config rx_cfg;
	struct hsi_tx_config tx_cfg;
	int ret = 0;

	pr_debug("%s, ch = %d, cmd = 0x%08x\n", __func__, ch, cmd);

	switch (cmd) {
	case CS_SEND_BREAK:
		if_hsi_send_break(ch);
		break;
	case CS_FLUSH_RX:
		if_hsi_flush_rx(ch);
		break;
	case CS_FLUSH_TX:
		if_hsi_flush_tx(ch);
		break;
	case CS_SET_ACWAKELINE:
		if (copy_from_user(&state, (void __user *)arg, sizeof(state)))
			ret = -EFAULT;
		else
			if_hsi_set_acwakeline(ch, state);
		break;
	case CS_GET_ACWAKELINE:
		if_hsi_get_acwakeline(ch, &state);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;
	case CS_SET_WAKE_RX_3WIRES_MODE:
		if (copy_from_user(&state, (void __user *)arg, sizeof(state)))
			ret = -EFAULT;
		else
			if_hsi_set_wake_rx_3wires_mode(ch, state);
		break;
	case CS_GET_CAWAKELINE:
		if_hsi_get_cawakeline(ch, &state);
		if (copy_to_user((void __user *)arg, &state, sizeof(state)))
			ret = -EFAULT;
		break;
	case CS_SET_RX:
		if (copy_from_user(&rx_cfg, (void __user *)arg, sizeof(rx_cfg)))
			ret = -EFAULT;
		else
			ret = if_hsi_set_rx(ch, &rx_cfg);
		break;
	case CS_GET_RX:
		if_hsi_get_rx(ch, &rx_cfg);
		if (copy_to_user((void __user *)arg, &rx_cfg, sizeof(rx_cfg)))
			ret = -EFAULT;
		break;
	case CS_SET_TX:
		if (copy_from_user(&tx_cfg, (void __user *)arg, sizeof(tx_cfg)))
			ret = -EFAULT;
		else
			ret = if_hsi_set_tx(ch, &tx_cfg);
		break;
	case CS_GET_TX:
		if_hsi_get_tx(ch, &tx_cfg);
		if (copy_to_user((void __user *)arg, &tx_cfg, sizeof(tx_cfg)))
			ret = -EFAULT;
		break;
	case CS_SW_RESET:
		if_hsi_sw_reset(ch);
		break;
	case CS_GET_FIFO_OCCUPANCY:
		if_hsi_get_fifo_occupancy(ch, &occ);
		if (copy_to_user((void __user *)arg, &occ, sizeof(occ)))
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int hsi_char_open(struct inode *inode, struct file *file)
{
	int ret = 0, ch = iminor(inode);
	int i;

	for (i = 0; i < HSI_MAX_CHAR_DEVS; i++)
		if ((channels_map[i] - 1) == ch)
			break;

	if (i == HSI_MAX_CHAR_DEVS) {
		pr_err("HSI char open: Channel %d not found\n", ch);
		return -ENODEV;
	}

	pr_debug("HSI char open: opening channel %d\n", ch);

	spin_lock_bh(&hsi_char_data[ch].lock);

	if (hsi_char_data[ch].opened) {
		spin_unlock_bh(&hsi_char_data[ch].lock);
		pr_err("HSI char open: Channel %d already opened\n", ch);
		return -EBUSY;
	}

	file->private_data = (void *)ch;
	hsi_char_data[ch].opened++;
	hsi_char_data[ch].poll_event = (POLLOUT | POLLWRNORM);
	spin_unlock_bh(&hsi_char_data[ch].lock);

	ret = if_hsi_start(ch);

	return ret;
}

static int hsi_char_release(struct inode *inode, struct file *file)
{
	int ch = (int)file->private_data;
	struct char_queue *entry;
	struct list_head *cursor, *next;

	pr_debug("%s, ch = %d\n", __func__, ch);

	if_hsi_stop(ch);
	spin_lock_bh(&hsi_char_data[ch].lock);
	hsi_char_data[ch].opened--;

	if (!list_empty(&hsi_char_data[ch].rx_queue)) {
		list_for_each_safe(cursor, next, &hsi_char_data[ch].rx_queue) {
			entry = list_entry(cursor, struct char_queue, list);
			list_del(&entry->list);
			kfree(entry);
		}
	}

	if (!list_empty(&hsi_char_data[ch].tx_queue)) {
		list_for_each_safe(cursor, next, &hsi_char_data[ch].tx_queue) {
			entry = list_entry(cursor, struct char_queue, list);
			list_del(&entry->list);
			kfree(entry);
		}
	}

	spin_unlock_bh(&hsi_char_data[ch].lock);

	return 0;
}

static const struct file_operations hsi_char_fops = {
	.owner = THIS_MODULE,
	.read = hsi_char_read,
	.write = hsi_char_write,
	.poll = hsi_char_poll,
	.unlocked_ioctl = hsi_char_ioctl,
	.open = hsi_char_open,
	.release = hsi_char_release,
	.fasync = hsi_char_fasync,
};

static struct cdev hsi_char_cdev;

static int __init hsi_char_init(void)
{
	int ret, i;

	pr_info("HSI character device version " DRIVER_VERSION "\n");
	pr_info("HSI char driver: %d channels mapped\n", num_channels);

	for (i = 0; i < HSI_MAX_CHAR_DEVS; i++) {
		init_waitqueue_head(&hsi_char_data[i].rx_wait);
		init_waitqueue_head(&hsi_char_data[i].tx_wait);
		init_waitqueue_head(&hsi_char_data[i].poll_wait);
		spin_lock_init(&hsi_char_data[i].lock);
		hsi_char_data[i].opened = 0;
		INIT_LIST_HEAD(&hsi_char_data[i].rx_queue);
		INIT_LIST_HEAD(&hsi_char_data[i].tx_queue);
	}

	/*printk(KERN_DEBUG "%s, devname = %s\n", __func__, devname); */

	ret = if_hsi_init(port, channels_map, num_channels);
	if (ret)
		return ret;

	ret =
	    alloc_chrdev_region(&hsi_char_dev, 0, HSI_MAX_CHAR_DEVS,
				HSI_CHAR_DEVICE_NAME);
	if (ret < 0) {
		pr_err("HSI character driver: Failed to register\n");
		return ret;
	}

	cdev_init(&hsi_char_cdev, &hsi_char_fops);
	ret = cdev_add(&hsi_char_cdev, hsi_char_dev, HSI_MAX_CHAR_DEVS);
	if (ret < 0) {
		pr_err("HSI character device: Failed to add char device\n");
		return ret;
	}

	return 0;
}

static void __exit hsi_char_exit(void)
{
	cdev_del(&hsi_char_cdev);
	unregister_chrdev_region(hsi_char_dev, HSI_MAX_CHAR_DEVS);
	if_hsi_exit();
}

MODULE_AUTHOR("Andras Domokos <andras.domokos@nokia.com>");
MODULE_AUTHOR("Sebatien Jan <s-jan@ti.com> / Texas Instruments");
MODULE_DESCRIPTION("HSI character device");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(hsi_char_init);
module_exit(hsi_char_exit);
