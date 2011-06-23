/*
 * File - hsi_protocol.c
 *
 * Implements HSI protocol for Infineon Modem.
 *
 * Copyright (C) 2011 Samsung Electronics.
 *
 * Author: Rupesh Gujare <rupesh.g@samsung.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#if 0
#define DEBUG 1
#endif

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/io.h>

#include "hsi-protocol-if.h"
#include <linux/hsi_driver_if.h>

#define DRIVER_VERSION  "1.0"

char test_data[10] = "abcdefghij";

dev_t hsi_protocol_dev;

struct protocol_queue {
	struct list_head list;
	u32 *data;
	unsigned int count;
};

struct hsi_protocol {
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

static struct hsi_protocol hsi_protocol_data[HSI_MAX_CHANNELS];

void if_notify(int ch, struct hsi_event *ev)
{
	struct protocol_queue *entry;

	pr_debug("%s, ev = {0x%x, 0x%p, %u}\n",
		 __func__, ev->event, ev->data, ev->count);

	spin_lock(&hsi_protocol_data[ch].lock);

/* Not Required */
	/*if (!hsi_protocol_data[ch].opened) {
		pr_debug("%s, device not opened\n!", __func__);
		printk(KERN_INFO "%s, device not opened\n!", __func__);
		spin_unlock(&hsi_protocol_data[ch].lock);
		return;
	}*/

	switch (HSI_EV_TYPE(ev->event)) {
	case HSI_EV_IN:
		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!entry) {
			pr_err("HSI-CHAR: entry allocation failed.\n");
			spin_unlock(&hsi_protocol_data[ch].lock);
			return;
		}
		entry->data = ev->data;
		entry->count = ev->count;
		list_add_tail(&entry->list, &hsi_protocol_data[ch].rx_queue);
		spin_unlock(&hsi_protocol_data[ch].lock);
		pr_debug("%s, HSI_EV_IN\n", __func__);
		wake_up_interruptible(&hsi_protocol_data[ch].rx_wait);
		break;
	case HSI_EV_OUT:
		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!entry) {
			pr_err("HSI-CHAR: entry allocation failed.\n");
			spin_unlock(&hsi_protocol_data[ch].lock);
			return;
		}
		entry->data = ev->data;
		entry->count = ev->count;
		hsi_protocol_data[ch].poll_event |= (POLLOUT | POLLWRNORM);
		list_add_tail(&entry->list, &hsi_protocol_data[ch].tx_queue);
		spin_unlock(&hsi_protocol_data[ch].lock);
		pr_debug("%s, HSI_EV_OUT\n", __func__);
		wake_up_interruptible(&hsi_protocol_data[ch].tx_wait);
		break;
	case HSI_EV_EXCEP:
		hsi_protocol_data[ch].poll_event |= POLLPRI;
		spin_unlock(&hsi_protocol_data[ch].lock);
		pr_debug("%s, HSI_EV_EXCEP\n", __func__);
		wake_up_interruptible(&hsi_protocol_data[ch].poll_wait);
		break;
	case HSI_EV_AVAIL:
		hsi_protocol_data[ch].poll_event |= (POLLIN | POLLRDNORM);
		spin_unlock(&hsi_protocol_data[ch].lock);
		pr_debug("%s, HSI_EV_AVAIL\n", __func__);
		wake_up_interruptible(&hsi_protocol_data[ch].poll_wait);
		break;
	default:
		spin_unlock(&hsi_protocol_data[ch].lock);
		break;
	}
}

int hsi_proto_read(int ch, u32 *buffer, int count)
{
	DECLARE_WAITQUEUE(wait, current);
	u32 *data;
	unsigned int data_len = 0;
	struct protocol_queue *entry;
	int ret, recv_data = 0;

	/*if (count > MAX_HSI_IPC_BUFFER)
		count = MAX_HSI_IPC_BUFFER;

	data = kmalloc(count, GFP_ATOMIC);*/

	ret = if_hsi_read(ch, buffer, count);
	if (ret < 0) {
		pr_err("Can not submit read. READ Error\n");
		goto out2;
	}

	spin_lock_bh(&hsi_protocol_data[ch].lock);
	add_wait_queue(&hsi_protocol_data[ch].rx_wait, &wait);
	spin_unlock_bh(&hsi_protocol_data[ch].lock);

	for (;;) {
		data = NULL;
		data_len = 0;

		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_bh(&hsi_protocol_data[ch].lock);
		if (!list_empty(&hsi_protocol_data[ch].rx_queue)) {
			entry = list_entry(hsi_protocol_data[ch].rx_queue.next,
					   struct protocol_queue, list);
			data = entry->data;
			data_len = entry->count;
			list_del(&entry->list);
			kfree(entry);
		}
		spin_unlock_bh(&hsi_protocol_data[ch].lock);

		pr_debug("%s, data = 0x%p, data_len = %d\n",
			 __func__, data, data_len);

		if (data_len) {
			pr_debug("%s, RX finished, ch-> %d, length = %d\n",
				__func__, ch, count);
			spin_lock_bh(&hsi_protocol_data[ch].lock);
			hsi_protocol_data[ch].poll_event &=
							~(POLLIN | POLLRDNORM);
			spin_unlock_bh(&hsi_protocol_data[ch].lock);
			if_hsi_poll(ch);
#if 0
			memcpy(buffer, data, count);
#endif
			recv_data += data_len;
#if 0
			buffer += data_len;
			if ((recv_data == count) || (recv_data >= MAX_HSI_IPC_BUFFER))
#endif
			break;
		} else if (signal_pending(current)) {
			pr_debug("%s, ERESTARTSYS\n", __func__);
			recv_data = -EAGAIN;
			if_hsi_cancel_read(ch);
			/* goto out; */
			break;
		}

		/*printk(KERN_DEBUG "%s, going to sleep...\n", __func__); */
		schedule();
		/*printk(KERN_DEBUG "%s, woke up\n", __func__); */
	}

/*out:*/
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&hsi_protocol_data[ch].rx_wait, &wait);

out2:
	/*To Do- Set bit if data to be received is
	* greater than 512K Bytes and return to IPC call
	*/

	return recv_data;
}

int hsi_proto_write(int ch, u32 *buffer, int length)
{

	DECLARE_WAITQUEUE(wait, current);
	u32 *data;
	unsigned int data_len = 0, ret = -1;
	struct protocol_queue *entry;

	ret = if_hsi_write(ch, buffer, length);
	if (ret < 0) {
		pr_err("HSI Write ERROR %s\n", __func__);
		goto out2;
	} else
		spin_lock_bh(&hsi_protocol_data[ch].lock);
	hsi_protocol_data[ch].poll_event &= ~(POLLOUT | POLLWRNORM);
	add_wait_queue(&hsi_protocol_data[ch].tx_wait, &wait);
	spin_unlock_bh(&hsi_protocol_data[ch].lock);

	for (;;) {
		data = NULL;
		data_len = 0;

		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_bh(&hsi_protocol_data[ch].lock);
		if (!list_empty(&hsi_protocol_data[ch].tx_queue)) {
			entry = list_entry(hsi_protocol_data[ch].tx_queue.next,
					   struct protocol_queue, list);
			data = entry->data;
			data_len = entry->count;
			list_del(&entry->list);
			kfree(entry);
		}
		spin_unlock_bh(&hsi_protocol_data[ch].lock);

		if (data_len) {
			pr_debug("%s, TX finished, data_len = %d, ch-> %d\n",
				__func__, length, ch);
			ret = data_len;
			break;
		} else if (signal_pending(current)) {
			pr_debug("%s, ERESTARTSYS\n", __func__);
			ret = -ERESTARTSYS;
			goto out;
		}

		schedule();
	}

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&hsi_protocol_data[ch].tx_wait, &wait);

out2:
	return ret;
}
EXPORT_SYMBOL(hsi_proto_write);

static int __init hsi_protocol_init(void)
{
	int i, ret = 0;

	pr_info("HSI Infineon Protocol driver version " DRIVER_VERSION "\n");

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		init_waitqueue_head(&hsi_protocol_data[i].rx_wait);
		init_waitqueue_head(&hsi_protocol_data[i].tx_wait);
		init_waitqueue_head(&hsi_protocol_data[i].poll_wait);
		spin_lock_init(&hsi_protocol_data[i].lock);
		hsi_protocol_data[i].opened = 0;
		INIT_LIST_HEAD(&hsi_protocol_data[i].rx_queue);
		INIT_LIST_HEAD(&hsi_protocol_data[i].tx_queue);
	}

	printk(KERN_INFO "hsi_protocol_init : hsi_mux_setting Done.\n");

	ret = if_hsi_init();

	return ret;
}


static void __exit hsi_protocol_exit(void)
{
	if_hsi_exit();
}


MODULE_AUTHOR("Rupesh Gujare <rupesh.g@samsung.com> / Samsung Electronics");
MODULE_DESCRIPTION("HSI Protocol for Infineon Modem");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(hsi_protocol_init);
module_exit(hsi_protocol_exit);
