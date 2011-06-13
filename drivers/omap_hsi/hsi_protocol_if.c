/*
 * File -  hsi_protocol_if.c
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

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/bitmap.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>

#include <linux/hsi_driver_if.h>
#include "hsi-protocol-if.h"

//#define DEBUG	1
//#define DEBUG_PHY_DATA	1

#define HSI_CHANNEL_STATE_UNAVAIL       (1 << 0)
#define HSI_CHANNEL_STATE_READING       (1 << 1)
#define HSI_CHANNEL_STATE_WRITING       (1 << 2)


struct if_hsi_iface hsi_protocol_iface;
wait_queue_head_t ipc_read_wait, ipc_write_wait;


static void if_hsi_protocol_port_event(struct hsi_device *dev, unsigned int event,
				       void *arg);
static int __devinit hsi_protocol_probe(struct hsi_device *dev);
static int __devexit hsi_protocol_remove(struct hsi_device *dev);

static struct hsi_device_driver if_hsi_protocol_driver = {
	.ctrl_mask = ANY_HSI_CONTROLLER,
	.probe = hsi_protocol_probe,
	.remove = __devexit_p(hsi_protocol_remove),
		  .driver = {
		.name = "hsi_protocol"
	},
};

struct if_hsi_cmd hsi_cmd_history;
int tx_cmd_history_p = 0;
int rx_cmd_history_p = 0;

static int if_hsi_read_on(int ch, u32 *data, unsigned int count)
{
	struct if_hsi_channel *channel;
	int ret;

	channel = &hsi_protocol_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);

	spin_lock(&channel->lock);
	if (channel->state & HSI_CHANNEL_STATE_READING) {
		pr_err("Read still pending on channel %d\n", ch);
		spin_unlock(&channel->lock);
		return -EBUSY;
	}
	channel->state |= HSI_CHANNEL_STATE_READING;
	channel->rx_data = data;
	channel->rx_count = count;
	spin_unlock(&channel->lock);

	ret = hsi_read(channel->dev, data, count / 4);
	dev_dbg(&channel->dev->device, "%s, ch = %d, ret = %d\n", __func__, ch,
		ret);

	return ret;
}

static void if_hsi_proto_read_done(struct hsi_device *dev, unsigned int size)
{
	struct if_hsi_channel *channel;
	struct hsi_event ev;

#ifdef DEBUG_PHY_DATA
	u32 *tmp;
	u32 i;
#endif

	//printk(KERN_INFO "if_hsi_proto_read_done() is called for ch-> %d\n", dev->n_ch);
	channel = &hsi_protocol_iface.channels[dev->n_ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, dev->n_ch);
	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_READING;
	ev.event = HSI_EV_IN;
	ev.data = channel->rx_data;
	ev.count = 4 * size;
	spin_unlock(&channel->lock);

#ifdef DEBUG_PHY_DATA
	//Check received data -> Commented as it adds delay which causes MSG_BREAK
	tmp = channel->rx_data;
	printk(KERN_INFO "[%s](%d)(%d) RX = ", __func__, dev->n_ch, ev.count);
	for (i = 0; i < ((size > 5) ? 5 : size); i++) {
		printk(KERN_INFO "%08x ", *tmp);
		tmp++;
	}
	printk(KERN_INFO "\n");
#endif

	if_notify(dev->n_ch, &ev);
}

int if_hsi_read(int ch, u32 *data, unsigned int count)
{
	int ret = 0;
	struct if_hsi_channel *channel;
	channel = &hsi_protocol_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = if_hsi_read_on(ch, data, count);
	return ret;
}

int if_hsi_poll(int ch)
{
	struct if_hsi_channel *channel;
	int ret = 0;
	channel = &hsi_protocol_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = hsi_poll(channel->dev);
	return ret;
}

static int if_hsi_write_on(int ch, u32 *address, unsigned int count)
{
	struct if_hsi_channel *channel;
	int ret;

	channel = &hsi_protocol_iface.channels[ch];

	spin_lock(&channel->lock);
	if (channel->state & HSI_CHANNEL_STATE_WRITING) {
		pr_err("Write still pending on channel %d\n", ch);
		printk(KERN_INFO "Write still pending on channel %d\n", ch);
		spin_unlock(&channel->lock);
		return -EBUSY;
	}

	channel->tx_data = address;
	channel->tx_count = count;
	channel->state |= HSI_CHANNEL_STATE_WRITING;
	spin_unlock(&channel->lock);
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = hsi_write(channel->dev, address, count / 4);
	return ret;
}


static void if_hsi_proto_write_done(struct hsi_device *dev, unsigned int size)
{
	struct if_hsi_channel *channel;
	struct hsi_event ev;

#ifdef DEBUG_PHY_DATA
	u32 *tmp;
	u32 i;
#endif

	//printk(KERN_INFO "if_hsi_proto_write_done() is called for ch-> %d\n", dev->n_ch);
	channel = &hsi_protocol_iface.channels[dev->n_ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, dev->n_ch);

	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_WRITING;
	ev.event = HSI_EV_OUT;
	ev.data = channel->tx_data;
	ev.count = 4 * size;
	spin_unlock(&channel->lock);

#ifdef DEBUG_PHY_DATA
	//Check Outgoing data, Commented as it adds delay which causes MSG_BREAK
	tmp = channel->tx_data;
	printk(KERN_INFO "[%s](%d)(%d) TX = ", __func__, dev->n_ch, ev.count);
	for (i = 0; i < ((size > 5) ? 5 : size); i++) {
		printk(KERN_INFO "%08x ", *tmp);
		tmp++;
	}
	printk(KERN_INFO "\n");
#endif

	if_notify(dev->n_ch, &ev);

}

int if_hsi_write(int ch, u32 *data, unsigned int count)
{
	int ret = 0;
	struct if_hsi_channel *channel;
	channel = &hsi_protocol_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = if_hsi_write_on(ch, data, count);
	return ret;
}

void if_hsi_cancel_read(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_protocol_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	if (channel->state & HSI_CHANNEL_STATE_READING)
		hsi_read_cancel(channel->dev);
	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_READING;
	spin_unlock(&channel->lock);
}

void if_hsi_set_wakeline(int ch, unsigned int state)
{
	struct if_hsi_channel *channel;
	channel = &hsi_protocol_iface.channels[ch];
	hsi_ioctl(channel->dev,
		  state ? HSI_IOCTL_ACWAKE_UP : HSI_IOCTL_ACWAKE_DOWN, NULL);
}


static void if_hsi_protocol_port_event(struct hsi_device *dev, unsigned int event,
				       void *arg)
{
	struct hsi_event ev;
	int i;

	ev.event = HSI_EV_EXCEP;
	ev.data = (u32 *) 0;
	ev.count = 0;


	switch (event) {
	case HSI_EVENT_BREAK_DETECTED:
		pr_debug("%s, HWBREAK detected\n", __func__);
		ev.data = (u32 *) HSI_HWBREAK;
		for (i = 0; i < HSI_MAX_CHANNELS; i++) {
			if (hsi_protocol_iface.channels[i].opened)
				if_notify(i, &ev);
		}
		break;
	case HSI_EVENT_HSR_DATAAVAILABLE:
		i = (int)arg;
		pr_debug("%s, HSI_EVENT_HSR_DATAAVAILABLE channel = %d\n",
			 __func__, i);
		ev.event = HSI_EV_AVAIL;
		if (hsi_protocol_iface.channels[i].opened)
			if_notify(i, &ev);
		break;
	case HSI_EVENT_CAWAKE_UP:
		pr_debug("%s, CAWAKE up\n", __func__);
		break;
	case HSI_EVENT_CAWAKE_DOWN:
		pr_debug("%s, CAWAKE down\n", __func__);
		break;
	case HSI_EVENT_ERROR:
		pr_debug("%s, HSI ERROR occured\n", __func__);
		break;
	default:
		pr_warning("%s, Unknown event(%d)\n", __func__, event);
		break;
	}
}

int if_hsi_openchannel(struct if_hsi_channel *channel)
{
	int ret = 0;

	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__,
		channel->channel_id);
	spin_lock(&channel->lock);

	if (channel->state == HSI_CHANNEL_STATE_UNAVAIL) {
		ret = -ENODEV;
		goto leave;
	}

	if (channel->opened) {
		ret = -EBUSY;
		goto leave;
	}

	if (!channel->dev) {
		pr_err("Channel %d is not ready??\n", channel->channel_id);
		ret = -ENODEV;
		goto leave;
	}
	spin_unlock(&channel->lock);

	ret = hsi_open(channel->dev);
	spin_lock(&channel->lock);
	if (ret < 0) {
		pr_err("Could not open channel %d\n", channel->channel_id);
		goto leave;
	}

	channel->opened = 1;
	channel->tx_state = HSI_LL_TX_STATE_IDLE;
	channel->rx_state = HSI_LL_RX_STATE_TO_CONN_READY;
	printk(KERN_INFO "setting channel->opened=1 for channel %d\n", channel->dev->n_ch);
leave:
	spin_unlock(&channel->lock);
	return ret;
}

int if_hsi_closechannel(struct if_hsi_channel *channel)
{
	int ret = 0;

	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__,
		channel->channel_id);
	spin_lock(&channel->lock);

	if (!channel->opened)
		goto leave;

	if (!channel->dev) {
		pr_err("Channel %d is not ready??\n", channel->channel_id);
		ret = -ENODEV;
		goto leave;
	}

	/* Stop any pending read/write */
	if (channel->state & HSI_CHANNEL_STATE_READING) {
		channel->state &= ~HSI_CHANNEL_STATE_READING;
		spin_unlock(&channel->lock);
		hsi_read_cancel(channel->dev);
		spin_lock(&channel->lock);
	}
	if (channel->state & HSI_CHANNEL_STATE_WRITING) {
		channel->state &= ~HSI_CHANNEL_STATE_WRITING;

		spin_unlock(&channel->lock);
		hsi_write_cancel(channel->dev);
	} else
		spin_unlock(&channel->lock);

	hsi_close(channel->dev);

	spin_lock(&channel->lock);
	channel->opened = 0;
	channel->tx_state = HSI_LL_TX_STATE_CLOSED;
	channel->rx_state = HSI_LL_RX_STATE_CLOSED;
leave:
	spin_unlock(&channel->lock);
	return ret;
}


/* Read Thread
* Should be responsible for handling commands
* Should wait on port events - waitqueue
*
*/
static int hsi_read_thrd(void *data)
{
	u32 cmd_data[4], cmd, channel, param = 0;
	int ret;

	printk(KERN_INFO "Inside read thread\n");
	while (1) {
		/*Call hsi_proto_read*/
		/*Read 16 bytes due to Modem limitation*/
		//hsi_proto_read(0, cmd_data, (4 * 4));

		// For es 2.1 ver.
		hsi_proto_read(0, cmd_data, 4);

		hsi_cmd_history.rx_cmd[rx_cmd_history_p] = cmd_data[0];
		hsi_cmd_history.rx_cmd_time[rx_cmd_history_p] = CURRENT_TIME;
		rx_cmd_history_p++;
		if (rx_cmd_history_p >= 50)
			rx_cmd_history_p = 0;

		/*Decode Command*/
		ret = hsi_decode_cmd(&cmd_data[0], &cmd, &channel, &param);
		if (ret != 0) {
			pr_err("Can not decode command\n");
		} else {
			printk(KERN_INFO "%s(),CMD Received->  %x, ch-> %d, param-> %d.\n", __func__, cmd, channel, param);
			/*Rx State Machine*/
			rx_stm(cmd, channel, param);
		}
	}
	return 0;
}


int hsi_start_protocol(void)
{
	struct hst_ctx tx_config;
	struct hsr_ctx rx_config;
	int i, ret = 0;

	printk(KERN_INFO "In function  %s()\n", __func__);
	/*Open All channels */
	for (i = 0; i <= 5; i++) {
		ret = if_hsi_openchannel(&hsi_protocol_iface.channels[i]);
		if (ret < 0)
			pr_err("Can not Open channel->%d . Can not start HSI protocol\n", i);
		else
			printk(KERN_INFO "Channel->%d Open Successful\n", i);

		/*Set Rx Config*/
		hsi_ioctl(hsi_protocol_iface.channels[i].dev, HSI_IOCTL_GET_RX, &rx_config);
		rx_config.mode = 2;
		rx_config.divisor = 1;
		rx_config.channels = HSI_MAX_CHANNELS;
		ret = hsi_ioctl(hsi_protocol_iface.channels[i].dev, HSI_IOCTL_SET_RX, &rx_config);
		if (ret == 0)
			printk(KERN_INFO "SET_RX Successful for ch->%d\n", i);

		/*Set Tx Config*/
		hsi_ioctl(hsi_protocol_iface.channels[i].dev, HSI_IOCTL_GET_TX, &tx_config);
		tx_config.mode = 2;
		tx_config.divisor = 1;
		tx_config.channels = HSI_MAX_CHANNELS;
		ret = hsi_ioctl(hsi_protocol_iface.channels[i].dev, HSI_IOCTL_SET_TX, &tx_config);
		if (ret == 0)
			printk(KERN_INFO "SET_TX Successful for ch->%d\n", i);
	}
	/*Make channel-0 tx_state to IDLE*/
	hsi_protocol_iface.channels[0].tx_state = HSI_LL_TX_STATE_IDLE;
	return ret;
}
EXPORT_SYMBOL(hsi_start_protocol);

static int hsi_protocol_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len, i;

	p += sprintf(p, "======= HISTORY OF CMD =======\n");
	p += sprintf(p, "   tx_cmd_history_p : %d\n", tx_cmd_history_p);
	p += sprintf(p, "   rx_cmd_history_p : %d\n", rx_cmd_history_p);
	for (i = 0; i < 50; i++) {
		p += sprintf(p, "   [%d] tx : 0x%08x(%lu.%09lu), rx : 0x%08x(%lu.%09lu)\n",
			      i, hsi_cmd_history.tx_cmd[i], (unsigned long)hsi_cmd_history.tx_cmd_time[i].tv_sec, (unsigned long)hsi_cmd_history.tx_cmd_time[i].tv_nsec,
			      hsi_cmd_history.rx_cmd[i], (unsigned long)hsi_cmd_history.rx_cmd_time[i].tv_sec, (unsigned long)hsi_cmd_history.rx_cmd_time[i].tv_nsec);
	}
	p += sprintf(p, "======= HISTORY OF CMD =======\n");

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

int __devexit hsi_protocol_remove(struct hsi_device *dev)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int port, ret;

	//dev_dbg(&dev->device, "%s, port = %d, ch = %d\n", __func__, dev->n_p,
	//      dev->n_ch);

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_protocol_driver.ch_mask[port])
			break;
	}

	address = &if_hsi_protocol_driver.ch_mask[port];

	spin_lock_bh(&hsi_protocol_iface.lock);
	if (test_bit(dev->n_ch, address) && (dev->n_p == port)) {
		hsi_set_read_cb(dev, NULL);
		hsi_set_write_cb(dev, NULL);
		hsi_set_port_event_cb(dev, NULL);
		channel = &hsi_protocol_iface.channels[dev->n_ch];
		channel->dev = NULL;
		channel->state = HSI_CHANNEL_STATE_UNAVAIL;
		ret = 0;
	}
	spin_unlock_bh(&hsi_protocol_iface.lock);

	return ret;
}

int __devinit hsi_protocol_probe(struct hsi_device *dev)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int port;

	printk(KERN_INFO "Inside Function %s\n", __func__);
	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_protocol_driver.ch_mask[port])
			break;
	}

	address = &if_hsi_protocol_driver.ch_mask[port];

	spin_lock_bh(&hsi_protocol_iface.lock);
	if (test_bit(dev->n_ch, address) && (dev->n_p == port)) {
		printk(KERN_INFO "Regestering callback functions\n");
		hsi_set_read_cb(dev, if_hsi_proto_read_done);
		hsi_set_write_cb(dev, if_hsi_proto_write_done);
		hsi_set_port_event_cb(dev, if_hsi_protocol_port_event);
		channel = &hsi_protocol_iface.channels[dev->n_ch];
		channel->dev = dev;
		channel->state = 0;
		channel->rx_state = HSI_LL_RX_STATE_CLOSED;
		channel->tx_state = HSI_LL_TX_STATE_CLOSED;
		channel->tx_count = 0;
		channel->rx_count = 0;
		channel->tx_nak_count = 0;
		channel->rx_nak_count = 0;
		channel->rx_buf = NULL;
		channel->tx_buf = NULL;
		hsi_protocol_iface.init_chan_map ^= (1 << dev->n_ch);
	}
	spin_unlock_bh(&hsi_protocol_iface.lock);

	return 0;

}


int __init if_hsi_init(void)
{
	struct if_hsi_channel *channel;
	int i, ret;
	struct proc_dir_entry *dir;

	for (i = 0; i < HSI_MAX_PORTS; i++)
		if_hsi_protocol_driver.ch_mask[i] = 0;

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		channel = &hsi_protocol_iface.channels[i];
		channel->dev = NULL;
		channel->opened = 0;
		channel->state = HSI_CHANNEL_STATE_UNAVAIL;
		channel->channel_id = i;
		spin_lock_init(&channel->lock);
	}

	/*Initialize waitqueue for IPC read*/
	init_waitqueue_head(&ipc_read_wait);
	init_waitqueue_head(&ipc_write_wait);

	/*Select All Channels of PORT-1.*/
	if_hsi_protocol_driver.ch_mask[0] = CHANNEL_MASK;

	ret = hsi_register_driver(&if_hsi_protocol_driver);
	if (ret)
		pr_err("Error while registering HSI driver %d", ret);

	dir = create_proc_read_entry("driver/hsi_cmd", 0, 0, hsi_protocol_proc, NULL);
	if (dir == NULL)
		printk(KERN_INFO "create_proc_read_entry Fail.\n");
	printk(KERN_INFO "create_proc_read_entry Done.\n");

	return ret;
}

int __devexit if_hsi_exit(void)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int i, port;

	pr_debug("%s\n", __func__);

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_protocol_driver.ch_mask[port])
			break;
	}

	address = &if_hsi_protocol_driver.ch_mask[port];

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		channel = &hsi_protocol_iface.channels[i];
		if (channel->opened) {
			if_hsi_set_wakeline(i, HSI_IOCTL_ACWAKE_DOWN);
			if_hsi_closechannel(channel);
		}
	}

	hsi_unregister_driver(&if_hsi_protocol_driver);
	return 0;

}

u32 initialization = 0;

/*Write data to channel*/
int write_hsi(u32 ch, u32 *data, int length)
{
	int ret;
	//u32 cmd[4] = {0x00000000, 0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC};
	struct if_hsi_channel *channel;
	struct task_struct *read_thread;

	channel = &hsi_protocol_iface.channels[ch];
	channel->tx_buf = data;
	channel->tx_count = 0;

	//cmd[0] = protocol_create_cmd(HSI_LL_MSG_OPEN_CONN_OCTET, ch, (void *)&length);
	//printk(KERN_INFO "data ptr is %x\n", data);

	if (initialization == 0) {

#if 0
		/* ACWAKE ->HIGH */
		ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_UP, NULL);
		if (ret == 0)
			printk(KERN_INFO "ACWAKE pulled high in %s()\n", __func__);
		else
			printk(KERN_INFO "ACWAKE pulled high in %s() ERROR : %d\n", __func__, ret);
#endif

		/*Creating read thread*/
		read_thread = kthread_run(hsi_read_thrd, NULL, "hsi_read_thread");

		initialization++;
	}
	/*Wait till previous data transfer is over*/
	while (channel->tx_state != HSI_LL_TX_STATE_IDLE) {
		//printk(KERN_INFO "Wait 5ms previous data transfer isn't over %s()\n", __func__);

		//msleep(5);

		return -EAGAIN;
	}

#if 1
	/* ACWAKE ->HIGH */
	ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_UP, NULL);
	if (ret == 0)
		printk(KERN_INFO "ACWAKE pulled high in %s()\n", __func__);
	else
		printk(KERN_INFO "ACWAKE pulled high in %s() ERROR : %d\n", __func__, ret);
#endif

	channel->tx_state = HSI_LL_TX_STATE_WAIT_FOR_ACK;

	//send_cmd(cmd, channel, data)
	//ret = hsi_proto_write(0, &cmd, 4*4);
	//printk(KERN_INFO "Write returned %d\n", ret);
	hsi_protocol_send_command(HSI_LL_MSG_OPEN_CONN_OCTET, ch, length);

	wait_event_interruptible(ipc_write_wait, channel->tx_count != 0);

	return	channel->tx_count;


}
EXPORT_SYMBOL(write_hsi);


int read_hsi(u8 *data, u32 ch, u32 *length)
{
	int ret, size, tmp, actual_length;
	struct if_hsi_channel *channel;

	channel = &hsi_protocol_iface.channels[ch];
	channel->rx_state = HSI_LL_RX_STATE_IDLE;

	//printk(KERN_INFO "In read_hsi() function, Sleeping ... channel-> %d\n", ch);
	wait_event_interruptible(ipc_read_wait, (channel->rx_count != 0));
	//printk(KERN_INFO "In read_hsi() function, Waking Up ... channel-> %d\n", ch);

	actual_length = channel->rx_count;
	size = channel->rx_count;

#if 0
	// TEMP: send/read by 16 byte unit for v.11A(CP)
	if ((size > 16) && (size % 16))
		size += (16 - (size % 16));
	else if (size < 16)
		size = 16;
#endif

	// For es 2.1 ver.
	if (size % 4)
		size += (4 - (size % 4));

	ret = hsi_proto_read(ch, (u32 *)data, size);
	if (ret < 0)
		printk(KERN_INFO "Read in IPC failed, %s()\n", __func__);

	//printk(KERN_INFO "%s() read returned %d, actual_length = %d, ch-> %d\n", __func__, ret, actual_length, ch);
	//printk(KERN_INFO "%s() sending CONN_CLOSED.\n", __func__);
	tmp = hsi_protocol_send_command(HSI_LL_MSG_CONN_CLOSED, ch, 0);
	//printk(KERN_INFO "%s() Sending CONN_CLOSED Finished. ret = %d\n", __func__, tmp);

	*length = actual_length;
	channel->rx_count = 0;

	//printk(KERN_INFO "%s() RETURNING TO IPC with ret = %d\n", __func__, ret);
	return ret;

}
EXPORT_SYMBOL(read_hsi);


//========================================================//
//                ++ Flashless Boot. ++                   //
//========================================================//
int hsi_start_protocol_single(void)
{
	int ret = 0;

	struct hst_ctx tx_config;
	struct hsr_ctx rx_config;

	/*Open channel 0 */
	ret = if_hsi_openchannel(&hsi_protocol_iface.channels[0]);
	if (ret < 0) {
		pr_err("Can not Open channel 0. Can not start HSI protocol\n");
		goto err;
	} else
		printk(KERN_INFO "if_hsi_openchannel() returned %d\n", ret);


	/*Set Tx Config*/
	hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_GET_TX, &tx_config);
	tx_config.mode = 2;
	tx_config.channels = 1;
	tx_config.divisor = 0;
	ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_SET_TX, &tx_config);
	if (ret < 0) {
		printk(KERN_INFO "write_hsi_direct : SET_TX Fail : %d\n", ret);
		return ret;
	}

	hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_GET_RX, &rx_config);
	rx_config.mode = 2;
	rx_config.channels = 1;
	rx_config.divisor = 0;
	//rx_config.timeout = HZ / 2;
	ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_SET_RX, &rx_config);
	if (ret < 0) {
		printk(KERN_INFO "write_hsi_direct : SET_RX Fail : %d\n", ret);
		return ret;
	}

	/* ACWAKE ->HIGH */
	ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_UP, NULL);
	if (ret == 0)
		printk(KERN_INFO "ACWAKE pulled high in %s()\n", __func__);

err:

	return ret;
}
EXPORT_SYMBOL(hsi_start_protocol_single);

int hsi_reconfigure_protocol(void)
{
	int ret = 0;

	/* ACWAKE ->LOW */
	ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_DOWN, NULL);
	if (ret == 0)
		printk(KERN_INFO "ACWAKE pulled low in %s()\n", __func__);
	else
		printk(KERN_INFO "ACWAKE down fail!! %d\n", ret);


	/*Clse channel 0 */
	ret = if_hsi_closechannel(&hsi_protocol_iface.channels[0]);
	if (ret < 0) {
		pr_err("Can not Close channel 0. Can not Stop HSI protocol for flashless\n");
		goto err;
	}


	printk(KERN_INFO "(%s)(%d) hsi_start_protocol Start.\n", __func__, __LINE__);
	hsi_start_protocol();
	printk(KERN_INFO "(%s)(%d) hsi_start_protocol Done.\n", __func__, __LINE__);

err:

	return ret;
}
EXPORT_SYMBOL(hsi_reconfigure_protocol);

int write_hsi_direct(u32 *data, int length)
{
	int retval = 0;
#if 0
	struct hst_ctx tx_config;


	printk(KERN_INFO "write_hsi_direct : len : %d\n", length);
	hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_GET_TX, &tx_config);
	tx_config.mode = 2;
	tx_config.channels = 1;
	tx_config.divisor = 47;
	retval = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_SET_TX, &tx_config);
	if (retval < 0) {
		printk(KERN_INFO "write_hsi_direct : SET_TX Fail : %d\n", retval);
		return retval;
	}
	printk(KERN_INFO "write_hsi_direct : SET_TX Successful\n");

	retval = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_UP, NULL);
	if (retval < 0) {
		printk(KERN_INFO "write_hsi_direct : ACWAKE High Fail : %d\n", retval);
		return retval;
	}
#endif

#if 0
	if ((length > 16) && (length % 4))
		length += (4 - (length % 4));
	else if (length < 16)
		length = 16;
#endif

//	printk(KERN_INFO "write_hsi_direct : new len : %d\n", length);

	retval = hsi_proto_write(0, data, length);
	if (retval < 0) {
		printk(KERN_INFO "write_hsi_direct : hsi_proto_write Fail : %d\n", retval);
		return retval;
	}
	//printk(KERN_INFO "write_hsi_direct : Write returned %d\n", retval);

	return retval;
}
EXPORT_SYMBOL(write_hsi_direct);

int read_hsi_direct(u32 *data, int length)
{
	int retval = 0;
#if 0
	struct hsr_ctx rx_config;


	printk(KERN_INFO "read_hsi_direct : len : %d\n", length);
	hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_GET_RX, &rx_config);
	rx_config.mode = 2;
	rx_config.channels = 1;
	rx_config.divisor = 47;
	retval = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_SET_RX, &rx_config);
	if (retval < 0) {
		printk(KERN_INFO "read_hsi_direct : SET_RX Fail : %d\n", retval);
		return retval;
	}
	printk(KERN_INFO "read_hsi_direct : SET_RX Successful\n");

	retval = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_UP, NULL);
	if (retval < 0) {
		printk(KERN_INFO "read_hsi_direct : ACWAKE High Fail : %d\n", retval);
		return retval;
	}
	printk(KERN_INFO "read_hsi_direct : ACWAKE High\n");
#endif

#if 0
	if ((length > 16) && (length % 4))
		length += (4 - (length % 4));
	else if (length < 16)
		length = 16;
#endif
	//printk(KERN_INFO "read_hsi_direct : new len : %d\n", length);

	retval = hsi_proto_read(0, data, length);
	if (retval < 0) {
		printk(KERN_INFO "read_hsi_direct : hsi_proto_read Fail : %d\n", retval);
		return retval;
	}
	//printk(KERN_INFO "read_hsi_direct : Read returned %d\n", retval);

	return retval;
}
EXPORT_SYMBOL(read_hsi_direct);

//========================================================//
//                -- Flashless Boot. --                   //
//========================================================//
