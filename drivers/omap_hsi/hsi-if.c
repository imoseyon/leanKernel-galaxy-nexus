	/*
 * hsi-if.c
 *
 * Part of the HSI character driver, implements the HSI interface.
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/bitmap.h>

#include <linux/hsi_driver_if.h>
#include <linux/hsi_char.h>

#include "hsi-char.h"
#include "hsi-if.h"

#define HSI_CHANNEL_STATE_UNAVAIL	(1 << 0)
#define HSI_CHANNEL_STATE_READING	(1 << 1)
#define HSI_CHANNEL_STATE_WRITING	(1 << 2)

#define PORT1	0
#define PORT2	1

#define RXCONV(dst, src) \
	do { \
		(dst)->mode = (src)->mode; \
		(dst)->flow = (src)->flow; \
		(dst)->frame_size = (src)->frame_size; \
		(dst)->channels = (src)->channels; \
		(dst)->divisor = (src)->divisor; \
		(dst)->counters = (src)->counters; \
	} while (0)

#define TXCONV(dst, src) \
	do { \
		(dst)->mode = (src)->mode; \
		(dst)->flow = (src)->flow; \
		(dst)->frame_size = (src)->frame_size; \
		(dst)->channels = (src)->channels; \
		(dst)->divisor = (src)->divisor; \
		(dst)->arb_mode = (src)->arb_mode; \
	} while (0)

struct if_hsi_channel {
	struct hsi_device *dev;
	unsigned int channel_id;
	u32 *tx_data;
	unsigned int tx_count;	/* Number of bytes to be written */
	u32 *rx_data;
	unsigned int rx_count;	/* Number of bytes to be read */
	unsigned int opened;
	unsigned int state;
	spinlock_t lock; /* Serializes access to channel data */
};

struct if_hsi_iface {
	struct if_hsi_channel channels[HSI_MAX_CHAR_DEVS];
	int bootstrap;
	unsigned long init_chan_map;
	spinlock_t lock; /* Serializes access to HSI functional interface */
};

static void if_hsi_port_event(struct hsi_device *dev, unsigned int event,
			      void *arg);
static int __devinit if_hsi_probe(struct hsi_device *dev);
static int __devexit if_hsi_remove(struct hsi_device *dev);

static struct hsi_device_driver if_hsi_char_driver = {
	.ctrl_mask = ANY_HSI_CONTROLLER,
	.probe = if_hsi_probe,
	.remove = __devexit_p(if_hsi_remove),
	.driver = {
		   .name = "hsi_char"},
};

static struct if_hsi_iface hsi_iface;

static int if_hsi_read_on(int ch, u32 *data, unsigned int count)
{
	struct if_hsi_channel *channel;
	int ret;

	channel = &hsi_iface.channels[ch];
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

/* HSI char driver read done callback */
static void if_hsi_read_done(struct hsi_device *dev, unsigned int size)
{
	struct if_hsi_channel *channel;
	struct hsi_event ev;

	channel = &hsi_iface.channels[dev->n_ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, dev->n_ch);
	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_READING;
	ev.event = HSI_EV_IN;
	ev.data = channel->rx_data;
	ev.count = 4 * size;	/* Convert size to number of u8, not u32 */
	spin_unlock(&channel->lock);
	if_hsi_notify(dev->n_ch, &ev);
}

int if_hsi_read(int ch, u32 *data, unsigned int count)
{
	int ret = 0;
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = if_hsi_read_on(ch, data, count);
	return ret;
}

int if_hsi_poll(int ch)
{
	struct if_hsi_channel *channel;
	int ret = 0;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = hsi_poll(channel->dev);
	return ret;
}

static int if_hsi_write_on(int ch, u32 *address, unsigned int count)
{
	struct if_hsi_channel *channel;
	int ret;

	channel = &hsi_iface.channels[ch];

	spin_lock(&channel->lock);
	if (channel->state & HSI_CHANNEL_STATE_WRITING) {
		pr_err("Write still pending on channel %d\n", ch);
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

/* HSI char driver write done callback */
static void if_hsi_write_done(struct hsi_device *dev, unsigned int size)
{
	struct if_hsi_channel *channel;
	struct hsi_event ev;

	channel = &hsi_iface.channels[dev->n_ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, dev->n_ch);

	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_WRITING;
	ev.event = HSI_EV_OUT;
	ev.data = channel->tx_data;
	ev.count = 4 * size;	/* Convert size to number of u8, not u32 */
	spin_unlock(&channel->lock);
	if_hsi_notify(dev->n_ch, &ev);
}

int if_hsi_write(int ch, u32 *data, unsigned int count)
{
	int ret = 0;
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	ret = if_hsi_write_on(ch, data, count);
	return ret;
}

void if_hsi_send_break(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	hsi_ioctl(channel->dev, HSI_IOCTL_SEND_BREAK, NULL);
}

void if_hsi_flush_rx(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev, HSI_IOCTL_FLUSH_RX, NULL);
}

void if_hsi_flush_ch(int ch)
{
	/* FIXME - Check the purpose of this function */
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
}

void if_hsi_flush_tx(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev, HSI_IOCTL_FLUSH_TX, NULL);
}

void if_hsi_get_wakeline(int ch, unsigned int *state)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev, HSI_IOCTL_GET_ACWAKE, state);
}

void if_hsi_set_wakeline(int ch, unsigned int state)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev,
		  state ? HSI_IOCTL_ACWAKE_UP : HSI_IOCTL_ACWAKE_DOWN, NULL);
}

int if_hsi_set_rx(int ch, struct hsi_rx_config *cfg)
{
	int ret;
	struct if_hsi_channel *channel;
	struct hsr_ctx ctx;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	RXCONV(&ctx, cfg);
	ret = hsi_ioctl(channel->dev, HSI_IOCTL_SET_RX, &ctx);
	return ret;
}

void if_hsi_get_rx(int ch, struct hsi_rx_config *cfg)
{
	struct if_hsi_channel *channel;
	struct hsr_ctx ctx;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	hsi_ioctl(channel->dev, HSI_IOCTL_GET_RX, &ctx);
	RXCONV(cfg, &ctx);
}

int if_hsi_set_tx(int ch, struct hsi_tx_config *cfg)
{
	int ret;
	struct if_hsi_channel *channel;
	struct hst_ctx ctx;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	TXCONV(&ctx, cfg);
	ret = hsi_ioctl(channel->dev, HSI_IOCTL_SET_TX, &ctx);
	return ret;
}

void if_hsi_get_tx(int ch, struct hsi_tx_config *cfg)
{
	struct if_hsi_channel *channel;
	struct hst_ctx ctx;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	hsi_ioctl(channel->dev, HSI_IOCTL_GET_TX, &ctx);
	TXCONV(cfg, &ctx);
}

void if_hsi_sw_reset(int ch)
{
	struct if_hsi_channel *channel;
	int i;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev, HSI_IOCTL_SW_RESET, NULL);

	spin_lock_bh(&hsi_iface.lock);
	/* Reset HSI channel states */
	for (i = 0; i < HSI_MAX_PORTS; i++)
		if_hsi_char_driver.ch_mask[i] = 0;

	for (i = 0; i < HSI_MAX_CHAR_DEVS; i++) {
		channel = &hsi_iface.channels[i];
		channel->opened = 0;
		channel->state = HSI_CHANNEL_STATE_UNAVAIL;
	}
	spin_unlock_bh(&hsi_iface.lock);
}

void if_hsi_get_fifo_occupancy(int ch, size_t *occ)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	hsi_ioctl(channel->dev, HSI_IOCTL_GET_FIFO_OCCUPANCY, occ);
}

void if_hsi_cancel_read(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	if (channel->state & HSI_CHANNEL_STATE_READING)
		hsi_read_cancel(channel->dev);
	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_READING;
	spin_unlock(&channel->lock);
}

void if_hsi_cancel_write(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);
	if (channel->state & HSI_CHANNEL_STATE_WRITING)
		hsi_write_cancel(channel->dev);
	spin_lock(&channel->lock);
	channel->state &= ~HSI_CHANNEL_STATE_WRITING;
	spin_unlock(&channel->lock);
}

static int if_hsi_openchannel(struct if_hsi_channel *channel)
{
	int ret = 0;

	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__,
		channel->channel_id);
	spin_lock(&channel->lock);

	if (channel->state == HSI_CHANNEL_STATE_UNAVAIL) {
		pr_err("Channel %d is not available\n", channel->channel_id);
		ret = -ENODEV;
		goto leave;
	}

	if (channel->opened) {
		pr_err("Channel %d is busy\n", channel->channel_id);
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

leave:
	spin_unlock(&channel->lock);
	return ret;
}

static int if_hsi_closechannel(struct if_hsi_channel *channel)
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
leave:
	spin_unlock(&channel->lock);
	return ret;
}

int if_hsi_start(int ch)
{
	struct if_hsi_channel *channel;
	int ret = 0;

	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);

	spin_lock_bh(&channel->lock);
	channel->state = 0;
	spin_unlock_bh(&channel->lock);

	ret = if_hsi_openchannel(channel);
	if (ret < 0) {
		pr_err("Could not open channel %d\n", ch);
		goto error;
	}

	if_hsi_poll(ch);
error:
	return ret;
}

void if_hsi_stop(int ch)
{
	struct if_hsi_channel *channel;
	channel = &hsi_iface.channels[ch];
	dev_dbg(&channel->dev->device, "%s, ch = %d\n", __func__, ch);

	if_hsi_closechannel(channel);
}

static int __devinit if_hsi_probe(struct hsi_device *dev)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int ret = -ENXIO, port;

	dev_dbg(&dev->device, "%s, port = %d, ch = %d\n", __func__, dev->n_p,
		dev->n_ch);

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_char_driver.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	if (dev->n_ch >= HSI_MAX_CHAR_DEVS) {
		pr_err("HSI char driver cannot handle channel %d\n", dev->n_ch);
		return -ENXIO;
	}

	address = &if_hsi_char_driver.ch_mask[port];

	spin_lock_bh(&hsi_iface.lock);
	if (test_bit(dev->n_ch, address) && (dev->n_p == port)) {
		hsi_set_read_cb(dev, if_hsi_read_done);
		hsi_set_write_cb(dev, if_hsi_write_done);
		hsi_set_port_event_cb(dev, if_hsi_port_event);
		channel = &hsi_iface.channels[dev->n_ch];
		channel->dev = dev;
		channel->state = 0;
		ret = 0;
		hsi_iface.init_chan_map ^= (1 << dev->n_ch);
	}
	spin_unlock_bh(&hsi_iface.lock);

	return ret;
}

static int __devexit if_hsi_remove(struct hsi_device *dev)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int ret = -ENXIO, port;

	dev_dbg(&dev->device, "%s, port = %d, ch = %d\n", __func__, dev->n_p,
		dev->n_ch);

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_char_driver.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	address = &if_hsi_char_driver.ch_mask[port];

	spin_lock_bh(&hsi_iface.lock);
	if (test_bit(dev->n_ch, address) && (dev->n_p == port)) {
		hsi_set_read_cb(dev, NULL);
		hsi_set_write_cb(dev, NULL);
		hsi_set_port_event_cb(dev, NULL);
		channel = &hsi_iface.channels[dev->n_ch];
		channel->dev = NULL;
		channel->state = HSI_CHANNEL_STATE_UNAVAIL;
		ret = 0;
	}
	spin_unlock_bh(&hsi_iface.lock);

	return ret;
}

static void if_hsi_port_event(struct hsi_device *dev, unsigned int event,
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
		for (i = 0; i < HSI_MAX_CHAR_DEVS; i++) {
			if (hsi_iface.channels[i].opened)
				if_hsi_notify(i, &ev);
		}
		break;
	case HSI_EVENT_HSR_DATAAVAILABLE:
		i = (int)arg;
		pr_debug("%s, HSI_EVENT_HSR_DATAAVAILABLE channel = %d\n",
			 __func__, i);
		ev.event = HSI_EV_AVAIL;
		if (hsi_iface.channels[i].opened)
			if_hsi_notify(i, &ev);
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

int __init if_hsi_init(unsigned int port, unsigned int *channels_map)
{
	struct if_hsi_channel *channel;
	int i, ret = 0;

	pr_debug("%s, port = %d\n", __func__, port);

	port -= 1;
	if (port >= HSI_MAX_PORTS)
		return -EINVAL;

	hsi_iface.bootstrap = 1;
	spin_lock_init(&hsi_iface.lock);

	for (i = 0; i < HSI_MAX_PORTS; i++)
		if_hsi_char_driver.ch_mask[i] = 0;

	for (i = 0; i < HSI_MAX_CHAR_DEVS; i++) {
		channel = &hsi_iface.channels[i];
		channel->dev = NULL;
		channel->opened = 0;
		channel->state = HSI_CHANNEL_STATE_UNAVAIL;
		channel->channel_id = i;
		spin_lock_init(&channel->lock);
	}

	for (i = 0; (i < HSI_MAX_CHAR_DEVS) && channels_map[i]; i++) {
		pr_debug("%s, port = %d, channels_map[i] = %d\n", __func__,
			 port, channels_map[i]);
		if ((channels_map[i] - 1) < HSI_MAX_CHAR_DEVS)
			if_hsi_char_driver.ch_mask[port] |=
			    (1 << ((channels_map[i] - 1)));
		else {
			pr_err("Channel %d cannot be handled by the HSI "
			       "driver.\n", channels_map[i]);
			return -EINVAL;
		}

	}
	hsi_iface.init_chan_map = if_hsi_char_driver.ch_mask[port];

	ret = hsi_register_driver(&if_hsi_char_driver);
	if (ret)
		pr_err("Error while registering HSI driver %d", ret);

	if (hsi_iface.init_chan_map) {
		ret = -ENXIO;
		pr_err("HSI: Some channels could not be registered (out of "
		       "range or already registered?)\n");
	}
	return ret;
}

int __devexit if_hsi_exit(void)
{
	struct if_hsi_channel *channel;
	unsigned long *address;
	int i, port;

	pr_debug("%s\n", __func__);

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_char_driver.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	address = &if_hsi_char_driver.ch_mask[port];

	for (i = 0; i < HSI_MAX_CHAR_DEVS; i++) {
		channel = &hsi_iface.channels[i];
		if (channel->opened) {
			if_hsi_set_wakeline(i, HSI_IOCTL_ACWAKE_DOWN);
			if_hsi_closechannel(channel);
		}
	}
	hsi_unregister_driver(&if_hsi_char_driver);
	return 0;
}
