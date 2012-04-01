/*
 * hsi_driver.c
 *
 * Implements HSI module interface, initialization, and PM related functions.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <mach/omap4-common.h>
#include <plat/omap_device.h>

#include "hsi_driver.h"

#if 0
static struct pm_qos_request_list *pm_qos_handle;
#endif

#define HSI_MODULENAME "omap_hsi"
#define	HSI_DRIVER_VERSION	"0.4.2"
#define HSI_RESETDONE_MAX_RETRIES	5 /* Max 5*L4 Read cycles waiting for */
					  /* reset to complete */
#define HSI_RESETDONE_NORMAL_RETRIES	1 /* Reset should complete in 1 R/W */

void hsi_save_ctx(struct hsi_dev *hsi_ctrl)
{
	struct hsi_platform_data *pdata = hsi_ctrl->dev->platform_data;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
	void __iomem *base = hsi_ctrl->base;
	struct hsi_port_ctx *p;
	int port;

	pdata->ctx->sysconfig = hsi_inl(base, HSI_SYS_SYSCONFIG_REG);
	pdata->ctx->gdd_gcr = hsi_inl(base, HSI_GDD_GCR_REG);
	if (hsi_driver_device_is_hsi(pdev))
		pdata->ctx->dll = hsi_inl(base, HSI_HSR_DLL_REG);

	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx->pctx[port - 1];
		/* HSI TOP */
		p->sys_mpu_enable[0] = hsi_inl(base,
					       HSI_SYS_MPU_ENABLE_REG(port, 0));
		p->sys_mpu_enable[1] = hsi_inl(base,
				       HSI_SYS_MPU_U_ENABLE_REG(port, 0));

		/* HST */
		p->hst.mode = hsi_inl(base, HSI_HST_MODE_REG(port));
		if (!hsi_driver_device_is_hsi(pdev))
			p->hst.frame_size = hsi_inl(base,
						HSI_HST_FRAMESIZE_REG(port));
		p->hst.divisor = hsi_inl(base, HSI_HST_DIVISOR_REG(port));
		p->hst.channels = hsi_inl(base, HSI_HST_CHANNELS_REG(port));
		p->hst.arb_mode = hsi_inl(base, HSI_HST_ARBMODE_REG(port));

		/* HSR */
		p->hsr.mode = hsi_inl(base, HSI_HSR_MODE_REG(port));
		if (!hsi_driver_device_is_hsi(pdev))
			p->hsr.frame_size = hsi_inl(base,
						HSI_HSR_FRAMESIZE_REG(port));
		p->hsr.divisor = hsi_inl(base, HSI_HSR_DIVISOR_REG(port));
		p->hsr.channels = hsi_inl(base, HSI_HSR_CHANNELS_REG(port));
		p->hsr.counters = hsi_inl(base, HSI_HSR_COUNTERS_REG(port));
	}
}

void hsi_restore_ctx(struct hsi_dev *hsi_ctrl)
{
	struct hsi_platform_data *pdata = hsi_ctrl->dev->platform_data;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
	void __iomem *base = hsi_ctrl->base;
	struct hsi_port_ctx *p;
	int port;

	hsi_outl(pdata->ctx->sysconfig, base, HSI_SYS_SYSCONFIG_REG);
	hsi_outl(pdata->ctx->gdd_gcr, base, HSI_GDD_GCR_REG);
	if (hsi_driver_device_is_hsi(pdev))
		hsi_outl(pdata->ctx->dll, base, HSI_HSR_DLL_REG);

	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx->pctx[port - 1];
		/* HSI TOP */
		hsi_outl(p->sys_mpu_enable[0], base,
			 HSI_SYS_MPU_ENABLE_REG(port, 0));
		hsi_outl(p->sys_mpu_enable[1], base,
			 HSI_SYS_MPU_U_ENABLE_REG(port, 0));

		/* HST */
		hsi_outl(p->hst.mode, base, HSI_HST_MODE_REG(port));
		if (!hsi_driver_device_is_hsi(pdev))
			hsi_outl(p->hst.frame_size, base,
				HSI_HST_FRAMESIZE_REG(port));
		hsi_outl(p->hst.divisor, base, HSI_HST_DIVISOR_REG(port));
		hsi_outl(p->hst.channels, base, HSI_HST_CHANNELS_REG(port));
		hsi_outl(p->hst.arb_mode, base, HSI_HST_ARBMODE_REG(port));

		/* HSR */
		if (!hsi_driver_device_is_hsi(pdev))
			hsi_outl(p->hsr.frame_size, base,
				HSI_HSR_FRAMESIZE_REG(port));
		hsi_outl(p->hsr.divisor, base, HSI_HSR_DIVISOR_REG(port));
		hsi_outl(p->hsr.channels, base, HSI_HSR_CHANNELS_REG(port));
		hsi_outl(p->hsr.counters, base, HSI_HSR_COUNTERS_REG(port));
	}

	if (hsi_driver_device_is_hsi(pdev)) {
		/* SW strategy for HSI fifo management can be changed here */
		hsi_fifo_mapping(hsi_ctrl, hsi_ctrl->fifo_mapping_strategy);
	}

	/* As a last step move HSR from MODE_VAL.SLEEP to the relevant mode. */
	/* This will enable the ACREADY flow control mechanism. */
	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx->pctx[port - 1];
		hsi_outl(p->hsr.mode, base, HSI_HSR_MODE_REG(port));
	}
}


/* NOTE: Function called in soft interrupt context (tasklet) */
int hsi_port_event_handler(struct hsi_port *p, unsigned int event, void *arg)
{
	struct hsi_channel *hsi_channel;
	int ch;


	if (event == HSI_EVENT_HSR_DATAAVAILABLE) {
		/* The data-available event is channel-specific and must not be
		 * broadcasted
		 */
		hsi_channel = p->hsi_channel + (int)arg;
		read_lock(&hsi_channel->rw_lock);
		if ((hsi_channel->dev) && (hsi_channel->port_event))
			hsi_channel->port_event(hsi_channel->dev, event, arg);
		read_unlock(&hsi_channel->rw_lock);
	} else {
		for (ch = 0; ch < p->max_ch; ch++) {
			hsi_channel = p->hsi_channel + ch;
			read_lock(&hsi_channel->rw_lock);
			if ((hsi_channel->dev) && (hsi_channel->port_event))
				hsi_channel->port_event(hsi_channel->dev,
							event, arg);
			read_unlock(&hsi_channel->rw_lock);
		}
	}
	return 0;
}

static void hsi_dev_release(struct device *dev)
{
	/* struct device kfree is already made in unregister_hsi_devices().
	 * Registering this function is necessary to avoid an error from
	 * the device_release() function.
	 */
}

/* Register a hsi_device, linked to a port and channel id */
static int __init reg_hsi_dev_ch(struct hsi_dev *hsi_ctrl, unsigned int p,
				 unsigned int ch)
{
	struct hsi_device *dev;
	struct hsi_port *port = &hsi_ctrl->hsi_port[p];
	int err;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->n_ctrl = hsi_ctrl->id;
	dev->n_p = p;
	dev->n_ch = ch;
	dev->ch = &port->hsi_channel[ch];
	dev->device.bus = &hsi_bus_type;
	dev->device.parent = hsi_ctrl->dev;
	dev->device.release = hsi_dev_release;
	if (dev->n_ctrl < 0)
		dev_set_name(&dev->device, "omap_hsi-p%u.c%u", p, ch);
	else
		dev_set_name(&dev->device, "omap_hsi%d-p%u.c%u", dev->n_ctrl, p,
			     ch);

	dev_dbg(hsi_ctrl->dev,
		"reg_hsi_dev_ch, port %d, ch %d, hsi_ctrl->dev:0x%x,"
		"&dev->device:0x%x\n",
		p, ch, (unsigned int)hsi_ctrl->dev, (unsigned int)&dev->device);

	err = device_register(&dev->device);
	if (err >= 0) {
		write_lock_bh(&port->hsi_channel[ch].rw_lock);
		port->hsi_channel[ch].dev = dev;
		write_unlock_bh(&port->hsi_channel[ch].rw_lock);
	} else {
		kfree(dev);
	}
	return err;
}

static int __init register_hsi_devices(struct hsi_dev *hsi_ctrl)
{
	int port;
	int ch;
	int err;

	for (port = 0; port < hsi_ctrl->max_p; port++)
		for (ch = 0; ch < hsi_ctrl->hsi_port[port].max_ch; ch++) {
			err = reg_hsi_dev_ch(hsi_ctrl, port, ch);
			if (err < 0)
				return err;
		}

	return 0;
}

static void __exit unregister_hsi_devices(struct hsi_dev *hsi_ctrl)
{
	struct hsi_port *hsi_p;
	struct hsi_device *device;
	unsigned int port;
	unsigned int ch;

	for (port = 0; port < hsi_ctrl->max_p; port++) {
		hsi_p = &hsi_ctrl->hsi_port[port];
		for (ch = 0; ch < hsi_p->max_ch; ch++) {
			device = hsi_p->hsi_channel[ch].dev;
			hsi_close(device);
			device_unregister(&device->device);
			kfree(device);
		}
	}
}

void hsi_set_pm_default(struct hsi_dev *hsi_ctrl)
{
	/* Set default SYSCONFIG PM settings */
	hsi_outl((HSI_AUTOIDLE | HSI_SIDLEMODE_SMART_WAKEUP |
				 HSI_MIDLEMODE_SMART_WAKEUP),
		 hsi_ctrl->base, HSI_SYS_SYSCONFIG_REG);
	hsi_outl(HSI_CLK_AUTOGATING_ON, hsi_ctrl->base, HSI_GDD_GCR_REG);

	/* HSI_TODO : use the HWMOD API : omap_hwmod_set_slave_idlemode() */
}

void hsi_set_pm_force_hsi_on(struct hsi_dev *hsi_ctrl)
{
	/* Force HSI to ON by never acknowledging a PRCM idle request */
	/* SIdleAck and MStandby are never asserted */
	hsi_outl((HSI_AUTOIDLE | HSI_SIDLEMODE_NO |
				 HSI_MIDLEMODE_NO),
		hsi_ctrl->base, HSI_SYS_SYSCONFIG_REG);
	hsi_outl(HSI_CLK_AUTOGATING_ON, hsi_ctrl->base, HSI_GDD_GCR_REG);

	/* HSI_TODO : use the HWMOD API : omap_hwmod_set_slave_idlemode() */
}

/**
* hsi_softreset - Force a SW RESET of HSI (core + DMA)
*
* @hsi_ctrl - reference to the hsi controller to be reset.
*
*/
int hsi_softreset(struct hsi_dev *hsi_ctrl)
{
	unsigned int ind = 0;
	unsigned int port;
	void __iomem *base = hsi_ctrl->base;
	u32 status;

	/* HSI-C1BUG00088: i696 : HSI: Issue with SW reset
	 * No recovery from SW reset under specific circumstances
	 * If a SW RESET is done while some HSI errors are still not
	 * acknowledged, the HSR FSM is stucked. */
	if (is_hsi_errata(hsi_ctrl, HSI_ERRATUM_i696_SW_RESET_FSM_STUCK)) {
		for (port = 1; port <= hsi_ctrl->max_p; port++) {
			hsi_outl_and(HSI_HSR_MODE_MODE_VAL_SLEEP, base,
				     HSI_HSR_MODE_REG(port));
			hsi_outl(HSI_HSR_ERROR_ALL, base,
				 HSI_HSR_ERRORACK_REG(port));
		}
	}
	/* Reseting HSI Block */
	hsi_outl_or(HSI_SOFTRESET, base, HSI_SYS_SYSCONFIG_REG);
	do {
		status = hsi_inl(base, HSI_SYS_SYSSTATUS_REG);
		ind++;
	} while ((!(status & HSI_RESETDONE)) &&
		   (ind < HSI_RESETDONE_MAX_RETRIES));

	if (ind >= HSI_RESETDONE_MAX_RETRIES) {
		dev_err(hsi_ctrl->dev, "HSI SW_RESET failed to complete within"
			" %d retries.\n", HSI_RESETDONE_MAX_RETRIES);
		return -EIO;
	} else if (ind > HSI_RESETDONE_NORMAL_RETRIES) {
		dev_warn(hsi_ctrl->dev, "HSI SW_RESET abnormally long:"
			" %d retries to complete.\n", ind);
	}

	ind = 0;
	/* Reseting DMA Engine */
	hsi_outl_or(HSI_GDD_GRST_SWRESET, base, HSI_GDD_GRST_REG);
	do {
		status = hsi_inl(base, HSI_GDD_GRST_REG);
		ind++;
	} while ((status & HSI_GDD_GRST_SWRESET) &&
		 (ind < HSI_RESETDONE_MAX_RETRIES));

	if (ind >= HSI_RESETDONE_MAX_RETRIES) {
		dev_err(hsi_ctrl->dev, "HSI DMA SW_RESET failed to complete"
			" within %d retries.\n", HSI_RESETDONE_MAX_RETRIES);
		return -EIO;
	}

	if (ind > HSI_RESETDONE_NORMAL_RETRIES) {
		dev_warn(hsi_ctrl->dev, "HSI DMA SW_RESET abnormally long:"
			" %d retries to complete.\n", ind);
	}

	return 0;
}

static void hsi_set_ports_default(struct hsi_dev *hsi_ctrl,
					    struct platform_device *pd)
{
	struct hsi_port_ctx *cfg;
	struct hsi_platform_data *pdata = pd->dev.platform_data;
	unsigned int port = 0;
	void __iomem *base = hsi_ctrl->base;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);

	for (port = 1; port <= pdata->num_ports; port++) {
		cfg = &pdata->ctx->pctx[port - 1];
		/* HST */
		hsi_outl(cfg->hst.mode | cfg->hst.flow |
			HSI_HST_MODE_WAKE_CTRL_SW, base,
			HSI_HST_MODE_REG(port));
		if (!hsi_driver_device_is_hsi(pdev))
			hsi_outl(cfg->hst.frame_size, base,
				 HSI_HST_FRAMESIZE_REG(port));
		hsi_outl(cfg->hst.divisor, base, HSI_HST_DIVISOR_REG(port));
		hsi_outl(cfg->hst.channels, base, HSI_HST_CHANNELS_REG(port));
		hsi_outl(cfg->hst.arb_mode, base, HSI_HST_ARBMODE_REG(port));

		/* HSR */
		hsi_outl(cfg->hsr.mode | cfg->hsr.flow, base,
			 HSI_HSR_MODE_REG(port));
		if (!hsi_driver_device_is_hsi(pdev))
			hsi_outl(cfg->hsr.frame_size, base,
				 HSI_HSR_FRAMESIZE_REG(port));
		hsi_outl(cfg->hsr.channels, base, HSI_HSR_CHANNELS_REG(port));
		if (hsi_driver_device_is_hsi(pdev))
			hsi_outl(cfg->hsr.divisor, base,
				 HSI_HSR_DIVISOR_REG(port));
		hsi_outl(cfg->hsr.counters, base, HSI_HSR_COUNTERS_REG(port));
	}

	if (hsi_driver_device_is_hsi(pdev)) {
		/* SW strategy for HSI fifo management can be changed here */
		hsi_fifo_mapping(hsi_ctrl, hsi_ctrl->fifo_mapping_strategy);
		hsi_outl(pdata->ctx->dll, base, HSI_HSR_DLL_REG);
	}
}

static int __init hsi_port_channels_init(struct hsi_port *port)
{
	struct hsi_channel *ch;
	unsigned int ch_i;

	for (ch_i = 0; ch_i < port->max_ch; ch_i++) {
		ch = &port->hsi_channel[ch_i];
		ch->channel_number = ch_i;
		rwlock_init(&ch->rw_lock);
		ch->flags = 0;
		ch->hsi_port = port;
		ch->read_data.addr = NULL;
		ch->read_data.size = 0;
		ch->read_data.lch = -1;
		ch->write_data.addr = NULL;
		ch->write_data.size = 0;
		ch->write_data.lch = -1;
		ch->dev = NULL;
		ch->read_done = NULL;
		ch->write_done = NULL;
		ch->port_event = NULL;
	}

	return 0;
}

static int hsi_port_channels_reset(struct hsi_port *port)
{
	struct hsi_channel *ch;
	unsigned int ch_i;

	for (ch_i = 0; ch_i < port->max_ch; ch_i++) {
		ch = &port->hsi_channel[ch_i];
		ch->flags = 0;
		ch->read_data.addr = NULL;
		ch->read_data.size = 0;
		ch->read_data.lch = -1;
		ch->write_data.addr = NULL;
		ch->write_data.size = 0;
		ch->write_data.lch = -1;
	}

	return 0;
}

/**
* hsi_softreset_driver - Must be called following HSI SW RESET, to re-align
*			 variable states with new HW state.
*
* @hsi_ctrl - reference to the hsi controller to be re-aligned.
*
*/
void hsi_softreset_driver(struct hsi_dev *hsi_ctrl)
{
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	struct hsi_platform_data *pdata = pd->dev.platform_data;
	struct hsi_port *hsi_p;
	unsigned int port;
	u32 revision;

	/* HSI port reset */
	for (port = 0; port < hsi_ctrl->max_p; port++) {
		hsi_p = &hsi_ctrl->hsi_port[port];
		hsi_p->counters_on = 1;
		hsi_p->reg_counters = pdata->ctx->pctx[port].hsr.counters;
		hsi_port_channels_reset(&hsi_ctrl->hsi_port[port]);
	}

	hsi_set_pm_force_hsi_on(hsi_ctrl);

	/* Re-Configure HSI ports */
	hsi_set_ports_default(hsi_ctrl, pd);

	/* Gather info from registers for the driver.(REVISION) */
	revision = hsi_inl(hsi_ctrl->base, HSI_SYS_REVISION_REG);
	if (hsi_driver_device_is_hsi(pd))
		dev_info(hsi_ctrl->dev, "HSI Hardware REVISION 0x%x\n",
			 revision);
	else
		dev_info(hsi_ctrl->dev, "SSI Hardware REVISION %d.%d\n",
			 (revision & HSI_SSI_REV_MAJOR) >> 4,
			 (revision & HSI_SSI_REV_MINOR));
}

static int __init hsi_request_mpu_irq(struct hsi_port *hsi_p)
{
	struct hsi_dev *hsi_ctrl = hsi_p->hsi_controller;
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	struct resource *mpu_irq;

	if (hsi_driver_device_is_hsi(pd))
		mpu_irq = platform_get_resource(pd, IORESOURCE_IRQ,
						hsi_p->port_number - 1);
	else			/* SSI support 2 IRQs per port */
		mpu_irq = platform_get_resource(pd, IORESOURCE_IRQ,
						(hsi_p->port_number - 1) * 2);

	if (!mpu_irq) {
		dev_err(hsi_ctrl->dev, "HSI misses info for MPU IRQ on"
			" port %d\n", hsi_p->port_number);
		return -ENXIO;
	}
	hsi_p->n_irq = 0;	/* We only use one irq line */
	hsi_p->irq = mpu_irq->start;
	return hsi_mpu_init(hsi_p, mpu_irq->name);
}

static int __init hsi_request_cawake_irq(struct hsi_port *hsi_p)
{
	struct hsi_dev *hsi_ctrl = hsi_p->hsi_controller;
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	struct resource *cawake_irq;

	if (hsi_driver_device_is_hsi(pd)) {
		hsi_p->cawake_gpio = -1;
		return 0;
	} else {
		cawake_irq = platform_get_resource(pd, IORESOURCE_IRQ,
						   4 + hsi_p->port_number);
	}

	if (!cawake_irq) {
		dev_err(hsi_ctrl->dev, "SSI device misses info for CAWAKE"
			"IRQ on port %d\n", hsi_p->port_number);
		return -ENXIO;
	}

	if (cawake_irq->flags & IORESOURCE_UNSET) {
		dev_info(hsi_ctrl->dev, "No CAWAKE GPIO support\n");
		hsi_p->cawake_gpio = -1;
		return 0;
	}

	hsi_p->cawake_gpio_irq = cawake_irq->start;
	hsi_p->cawake_gpio = irq_to_gpio(cawake_irq->start);
	return hsi_cawake_init(hsi_p, cawake_irq->name);
}

static void hsi_ports_exit(struct hsi_dev *hsi_ctrl, unsigned int max_ports)
{
	struct hsi_port *hsi_p;
	unsigned int port;

	for (port = 0; port < max_ports; port++) {
		hsi_p = &hsi_ctrl->hsi_port[port];
		hsi_mpu_exit(hsi_p);
		hsi_cawake_exit(hsi_p);
	}
}

static int __init hsi_ports_init(struct hsi_dev *hsi_ctrl)
{
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	struct hsi_platform_data *pdata = pd->dev.platform_data;
	struct hsi_port *hsi_p;
	unsigned int port;
	int err;

	for (port = 0; port < hsi_ctrl->max_p; port++) {
		hsi_p = &hsi_ctrl->hsi_port[port];
		hsi_p->flags = 0;
		hsi_p->port_number = pdata->ctx->pctx[port].port_number;
		hsi_p->hsi_controller = hsi_ctrl;
		hsi_p->max_ch = hsi_driver_device_is_hsi(pd) ?
		    HSI_CHANNELS_MAX : HSI_SSI_CHANNELS_MAX;
		hsi_p->irq = 0;
		hsi_p->wake_rx_3_wires_mode = 0; /* 4 wires */
		hsi_p->cawake_status = -1; /* Unknown */
		hsi_p->cawake_off_event = false;
		hsi_p->acwake_status = 0;
		hsi_p->in_int_tasklet = false;
		hsi_p->in_cawake_tasklet = false;
		hsi_p->counters_on = 1;
		hsi_p->reg_counters = pdata->ctx->pctx[port].hsr.counters;
		spin_lock_init(&hsi_p->lock);
		err = hsi_port_channels_init(hsi_p);
		if (err < 0)
			goto rback;
		err = hsi_request_mpu_irq(hsi_p);
		if (err < 0)
			goto rback;
		err = hsi_request_cawake_irq(hsi_p);
		if (err < 0)
			goto rback;
		dev_info(hsi_ctrl->dev, "HSI port %d initialized\n",
			 hsi_p->port_number);
	}
	return 0;
rback:
	hsi_ports_exit(hsi_ctrl, port + 1);
	return err;
}

static int __init hsi_request_gdd_irq(struct hsi_dev *hsi_ctrl)
{
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	struct resource *gdd_irq;

	if (hsi_driver_device_is_hsi(pd))
		gdd_irq = platform_get_resource(pd, IORESOURCE_IRQ, 2);
	else
		gdd_irq = platform_get_resource(pd, IORESOURCE_IRQ, 4);

	if (!gdd_irq) {
		dev_err(hsi_ctrl->dev, "HSI has no GDD IRQ resource\n");
		return -ENXIO;
	}

	hsi_ctrl->gdd_irq = gdd_irq->start;
	return hsi_gdd_init(hsi_ctrl, gdd_irq->name);
}

static int __init hsi_init_gdd_chan_count(struct hsi_dev *hsi_ctrl)
{
	struct platform_device *pd = to_platform_device(hsi_ctrl->dev);
	u8 gdd_chan_count;
	struct hsi_platform_data *pdata =
			(struct hsi_platform_data *)pd->dev.platform_data;
	int i;

	if (!pdata) {
		dev_err(hsi_ctrl->dev, "HSI has no platform data\n");
		return -ENXIO;
	}

	gdd_chan_count = pdata->hsi_gdd_chan_count;

	if (!gdd_chan_count) {
		dev_warn(hsi_ctrl->dev, "HSI device has no GDD channel count "
			 "(use %d as default)\n",
			 HSI_DMA_CHANNEL_DEFAULT);
		hsi_ctrl->gdd_chan_count = HSI_DMA_CHANNEL_DEFAULT;
	} else {
		hsi_ctrl->gdd_chan_count = gdd_chan_count;
		/* Check that the number of channels is power of 2 */
		for (i = 0; i < 16; i++) {
			if (hsi_ctrl->gdd_chan_count == (1 << i))
				break;
		}
		if (i >= 16)
			dev_err(hsi_ctrl->dev, "The Number of DMA channels "
				"shall be a power of 2! (=%d)\n",
				hsi_ctrl->gdd_chan_count);
	}
	return 0;
}

/**
* hsi_clocks_disable_channel - virtual wrapper for disabling HSI clocks for
* a given channel
* @dev - reference to the hsi device.
* @channel_number - channel number which requests clock to be disabled
*		    0xFF means no particular channel
*
* Note : there is no real HW clock management per HSI channel, this is only
* virtual to keep track of active channels and ease debug
*
* Function to be called with lock
*/
void hsi_clocks_disable_channel(struct device *dev, u8 channel_number,
				const char *s)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	if (channel_number != HSI_CH_NUMBER_NONE)
		dev_dbg(dev, "CLK: hsi_clocks_disable for "
			"channel %d: %s\n", channel_number, s);
	else
		dev_dbg(dev, "CLK: hsi_clocks_disable: %s\n", s);

	if (!hsi_ctrl->clock_enabled) {
		dev_dbg(dev, "Clocks already disabled, skipping...\n");
		return;
	}
	if (hsi_is_hsi_controller_busy(hsi_ctrl)) {
		dev_dbg(dev, "Cannot disable clocks, HSI port busy\n");
		return;
	}

	if (hsi_is_hst_controller_busy(hsi_ctrl))
		dev_warn(dev, "Disabling clocks with HST FSM not IDLE !\n");

#ifdef K3_0_PORTING_HSI_MISSING_FEATURE
	/* Allow Fclk to change */
	if (dpll_cascading_blocker_release(dev) < 0)
		dev_warn(dev, "Error releasing DPLL cascading constraint\n");
#endif

	pm_runtime_put_sync_suspend(dev);
}

/**
* hsi_clocks_enable_channel - virtual wrapper for enabling HSI clocks for
* a given channel
* @dev - reference to the hsi device.
* @channel_number - channel number which requests clock to be enabled
*		    0xFF means no particular channel
*
* Returns: -EEXIST if clocks were already active
*	   0 if clocks were previously inactive
*
* Note : there is no real HW clock management per HSI channel, this is only
* virtual to keep track of active channels and ease debug
*
* Function to be called with lock
*/
int hsi_clocks_enable_channel(struct device *dev, u8 channel_number,
				const char *s)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	if (channel_number != HSI_CH_NUMBER_NONE)
		dev_dbg(dev, "CLK: hsi_clocks_enable for "
			"channel %d: %s\n", channel_number, s);
	else
		dev_dbg(dev, "CLK: hsi_clocks_enable: %s\n", s);

	if (hsi_ctrl->clock_enabled) {
		dev_dbg(dev, "Clocks already enabled, skipping...\n");
		return -EEXIST;
	}

#ifdef K3_0_PORTING_HSI_MISSING_FEATURE
	/* Prevent Fclk change */
	if (dpll_cascading_blocker_hold(dev) < 0)
		dev_warn(dev, "Error holding DPLL cascading constraint\n");
#endif

	return pm_runtime_get_sync(dev);
}

static int __init hsi_controller_init(struct hsi_dev *hsi_ctrl,
					 struct platform_device *pd)
{
	struct hsi_platform_data *pdata = pd->dev.platform_data;
	struct resource *mem, *ioarea;
	int err;

	mem = platform_get_resource(pd, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pd->dev, "HSI device does not have "
			"HSI IO memory region information\n");
		return -ENXIO;
	}
	dev_dbg(&pd->dev, "hsi_controller_init : IORESOURCE_MEM %s [%x, %x]\n",
		 mem->name, mem->start, mem->end);

	ioarea = devm_request_mem_region(&pd->dev, mem->start,
					 (mem->end - mem->start) + 1,
					 dev_name(&pd->dev));
	if (!ioarea) {
		dev_err(&pd->dev, "Unable to request HSI IO mem region\n");
		return -EBUSY;
	}
	dev_dbg(&pd->dev, "hsi_controller_init : ioarea %s [%x, %x]\n",
		ioarea->name, ioarea->start, ioarea->end);

	hsi_ctrl->phy_base = mem->start;
	hsi_ctrl->base = devm_ioremap(&pd->dev, mem->start,
				      (mem->end - mem->start) + 1);
	if (!hsi_ctrl->base) {
		dev_err(&pd->dev, "Unable to ioremap HSI base IO address\n");
		return -ENXIO;
	}
	dev_dbg(&pd->dev, "hsi_controller_init : hsi_ctrl->base=%x\n",
		 (unsigned int)hsi_ctrl->base);

	hsi_ctrl->id = pd->id;
	if (pdata->num_ports > HSI_MAX_PORTS) {
		dev_err(&pd->dev, "The HSI driver does not support enough "
			"ports!\n");
		return -ENXIO;
	}
	hsi_ctrl->max_p = pdata->num_ports;
	hsi_ctrl->clock_enabled = false;
	hsi_ctrl->clock_rate = 0;
	hsi_ctrl->in_dma_tasklet = false;
	hsi_ctrl->fifo_mapping_strategy = pdata->fifo_mapping_strategy;
	hsi_ctrl->dev = &pd->dev;
	spin_lock_init(&hsi_ctrl->lock);
	err = hsi_init_gdd_chan_count(hsi_ctrl);
	if (err < 0)
		goto rback1;

	err = hsi_ports_init(hsi_ctrl);
	if (err < 0)
		goto rback1;

	err = hsi_request_gdd_irq(hsi_ctrl);
	if (err < 0)
		goto rback2;

	/* Everything is fine */
	return 0;
rback2:
	hsi_ports_exit(hsi_ctrl, hsi_ctrl->max_p);
rback1:
	dev_err(&pd->dev, "Error on hsi_controller initialization\n");
	return err;
}

static void hsi_controller_exit(struct hsi_dev *hsi_ctrl)
{
	hsi_gdd_exit(hsi_ctrl);
	hsi_ports_exit(hsi_ctrl, hsi_ctrl->max_p);
}

/* HSI Platform Device probing & hsi_device registration */
static int __init hsi_platform_device_probe(struct platform_device *pd)
{
	struct hsi_platform_data *pdata = pd->dev.platform_data;
	struct hsi_dev *hsi_ctrl;
	u32 revision;
	int err;

	dev_dbg(&pd->dev, "HSI DRIVER : hsi_platform_device_probe\n");

	dev_dbg(&pd->dev, "The platform device probed is an %s\n",
		hsi_driver_device_is_hsi(pd) ? "HSI" : "SSI");

	if (!pdata) {
		dev_err(&pd->dev, "No platform_data found on hsi device\n");
		return -ENXIO;
	}

	hsi_ctrl = kzalloc(sizeof(*hsi_ctrl), GFP_KERNEL);
	if (hsi_ctrl == NULL) {
		dev_err(&pd->dev, "Could not allocate memory for"
			" struct hsi_dev\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pd, hsi_ctrl);
	err = hsi_controller_init(hsi_ctrl, pd);
	if (err < 0) {
		dev_err(&pd->dev, "Could not initialize hsi controller:"
			" %d\n", err);
		goto rollback1;
	}
	/* Wakeup dependency was disabled for HSI <-> MPU PM_L3INIT_HSI_WKDEP */
#if 0
	omap_writel(0x141, 0x4A307338);
#endif
	pm_runtime_enable(hsi_ctrl->dev);
	pm_runtime_irq_safe(hsi_ctrl->dev);
	hsi_clocks_enable(hsi_ctrl->dev, __func__);

	/* Non critical SW Reset */
	err = hsi_softreset(hsi_ctrl);
	if (err < 0)
		goto rollback2;

	hsi_set_pm_force_hsi_on(hsi_ctrl);

	/* Configure HSI ports */
	hsi_set_ports_default(hsi_ctrl, pd);

	/* Gather info from registers for the driver.(REVISION) */
	revision = hsi_inl(hsi_ctrl->base, HSI_SYS_REVISION_REG);
	if (hsi_driver_device_is_hsi(pd))
		dev_info(hsi_ctrl->dev, "HSI Hardware REVISION 0x%x\n",
			 revision);
	else
		dev_info(hsi_ctrl->dev, "SSI Hardware REVISION %d.%d\n",
			 (revision & HSI_SSI_REV_MAJOR) >> 4,
			 (revision & HSI_SSI_REV_MINOR));

	err = hsi_debug_add_ctrl(hsi_ctrl);
	if (err < 0) {
		dev_err(&pd->dev,
			"Could not add hsi controller to debugfs: %d\n", err);
		goto rollback2;
	}

	err = register_hsi_devices(hsi_ctrl);
	if (err < 0) {
		dev_err(&pd->dev, "Could not register hsi_devices: %d\n", err);
		goto rollback3;
	}

	/* Allow HSI to wake up the platform */
	device_init_wakeup(hsi_ctrl->dev, true);

	/* Set the HSI FCLK to default. */
	if (!pdata->device_scale) {
		dev_err(&pd->dev, "%s: No platform device_scale function\n",
			__func__);
		err = -ENXIO;
		goto rollback3;
	}
	err = pdata->device_scale(hsi_ctrl->dev, hsi_ctrl->dev,
				  pdata->default_hsi_fclk);
	if (err == -EBUSY) {
		dev_warn(&pd->dev, "Cannot set HSI FClk to default value: %ld. "
			"Will retry on next open\n",
			pdata->default_hsi_fclk);
	} else if (err) {
		dev_err(&pd->dev, "%s: Error %d setting HSI FClk to %ld.\n",
				__func__, err, pdata->default_hsi_fclk);
		goto rollback3;
	} else {
		hsi_ctrl->clock_rate = pdata->default_hsi_fclk;
	}

	/* From here no need for HSI HW access */
	hsi_clocks_disable(hsi_ctrl->dev, __func__);

	return 0;

rollback3:
	hsi_debug_remove_ctrl(hsi_ctrl);
rollback2:
	hsi_controller_exit(hsi_ctrl);

	/* From here no need for HSI HW access */
	hsi_clocks_disable(hsi_ctrl->dev, __func__);

rollback1:
	kfree(hsi_ctrl);
	return err;
}

static int __exit hsi_platform_device_remove(struct platform_device *pd)
{
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	dev_dbg(&pd->dev, "HSI DRIVER : hsi_platform_device_remove\n");

	if (!hsi_ctrl)
		return 0;

	unregister_hsi_devices(hsi_ctrl);

	/* From here no need for HSI HW access */
	pm_runtime_disable(hsi_ctrl->dev);

	hsi_debug_remove_ctrl(hsi_ctrl);
	hsi_controller_exit(hsi_ctrl);

	kfree(hsi_ctrl);

	return 0;
}

#ifdef CONFIG_SUSPEND
static int hsi_pm_prepare(struct device *dev)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	dev_dbg(dev, "%s\n", __func__);

	/* If HSI is busy, refuse the suspend */
	if (hsi_ctrl->clock_enabled) {
		dev_info(dev, "Platform prepare while HSI active\n");
		return -EBUSY;
	}

	return 0;
}

static int hsi_pm_suspend(struct device *dev)
{
	struct hsi_platform_data *pdata = dev->platform_data;
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);
	unsigned int i;

	dev_dbg(dev, "%s\n", __func__);

	/* If HSI is enabled, CAWAKE IO wakeup has been disabled and */
	/* we don't want to re-enable it here. HSI interrupt shall be */
	/* generated normally because HSI HW is ON. */
	if (hsi_ctrl->clock_enabled) {
		dev_info(dev, "Platform suspend while HSI active\n");
		return -EBUSY;
	}

	/* Perform HSI board specific action before platform suspend */
	if (pdata->board_suspend)
		for (i = 0; i < hsi_ctrl->max_p; i++)
			pdata->board_suspend(hsi_ctrl->hsi_port[i].port_number,
					     device_may_wakeup(dev));

	return 0;
}

/* This callback can be useful in case an HSI interrupt occured between */
/* ->suspend() phase and ->suspend_noirq() phase */
static int hsi_pm_suspend_noirq(struct device *dev)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	dev_dbg(dev, "%s\n", __func__);

	/* If HSI is busy, refuse the suspend */
	if (hsi_ctrl->clock_enabled) {
		dev_info(dev, "Platform suspend_noirq while HSI active\n");
		return -EBUSY;
	}

	return 0;
}

static int hsi_pm_resume(struct device *dev)
{
	struct hsi_platform_data *pdata = dev->platform_data;
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);
	unsigned int i;

	dev_dbg(dev, "%s\n", __func__);

	/* This function shall not schedule the tasklet, because it is */
	/* redundant with what is already done in the PRCM interrupt handler. */
	/* HSI IO checking in PRCM int handler is done when waking up from : */
	/* - Device OFF mode (wake up from suspend) */
	/* - L3INIT in RET (Idle mode) */
	/* hsi_resume is called only when system wakes up from suspend. */
	/* So HSI IO checking in PRCM int handler and hsi_resume_noirq are */
	/* redundant. We need to choose which one will schedule the tasklet */
	/* Since HSI IO checking in PRCM int handler covers more cases, it is */
	/* the winner. */

	/* Perform (optional) HSI board specific action after platform wakeup */
	if (pdata->board_resume)
		for (i = 0; i < hsi_ctrl->max_p; i++)
			pdata->board_resume(hsi_ctrl->hsi_port[i].port_number);

	return 0;
}
#endif /* CONFIG_PM_SUSPEND */

#ifdef CONFIG_PM_RUNTIME
/**
* hsi_runtime_resume - executed by the PM core for the bus type of the device being woken up
* @dev - reference to the hsi device.
*
*
*/
#define HSI_HSR_MODE_FRAME	0x2
#define HSI_PORT1	0x1
int hsi_runtime_resume(struct device *dev)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);
	struct hsi_platform_data *pdata = hsi_ctrl->dev->platform_data;
	unsigned int i;

	dev_dbg(dev, "%s\n", __func__);

	if (hsi_ctrl->clock_enabled)
		dev_warn(dev, "Warning: clock status mismatch vs runtime PM\n");

	hsi_ctrl->clock_enabled = true;

	/* Restore context */
	hsi_restore_ctx(hsi_ctrl);

	/* Restore HSR_MODE register value */
	/* WARNING: works only in this configuration: */
	/* - Flow = Synchronized */
	/* - Mode = frame */
	hsi_outl(HSI_HSR_MODE_FRAME, hsi_ctrl->base,
			HSI_HSR_MODE_REG(HSI_PORT1));

	/* When HSI is ON, no need for IO wakeup mechanism on any HSI port */
	for (i = 0; i < hsi_ctrl->max_p; i++)
		pdata->wakeup_disable(hsi_ctrl->hsi_port[i].port_number);

	/* HSI device is now fully operational and _must_ be able to */
	/* complete I/O operations */

	return 0;
}

/**
* hsi_runtime_suspend - Prepare HSI for low power : device will not process data and will
    not communicate with the CPU
* @dev - reference to the hsi device.
*
* Return value : -EBUSY or -EAGAIN if device is busy and still operational
*
*/
int hsi_runtime_suspend(struct device *dev)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);
	struct hsi_platform_data *pdata = hsi_ctrl->dev->platform_data;
	int port, i;

	dev_dbg(dev, "%s\n", __func__);

	if (!hsi_ctrl->clock_enabled)
		dev_warn(dev, "Warning: clock status mismatch vs runtime PM\n");

	/* Put HSR into SLEEP mode to force ACREADY to low while HSI is idle */
	for (port = 1; port <= pdata->num_ports; port++) {
		hsi_outl_and(HSI_HSR_MODE_MODE_VAL_SLEEP, hsi_ctrl->base,
						HSI_HSR_MODE_REG(port));
	}

	/* Save context */
	hsi_save_ctx(hsi_ctrl);

	hsi_ctrl->clock_enabled = false;

	/* HSI is going to IDLE, it needs IO wakeup mechanism enabled */
	if (device_may_wakeup(dev))
		for (i = 0; i < hsi_ctrl->max_p; i++)
			pdata->wakeup_enable(hsi_ctrl->hsi_port[i].port_number);
	else
		for (i = 0; i < hsi_ctrl->max_p; i++)
			pdata->wakeup_disable(
				hsi_ctrl->hsi_port[i].port_number);

	/* HSI is now ready to be put in low power state */

	return 0;
}

/* Based on counters, device appears to be idle.
 * Check if the device can be suspended.
 */
static int hsi_runtime_idle(struct device *dev)
{
	struct platform_device *pd = to_platform_device(dev);
	struct hsi_dev *hsi_ctrl = platform_get_drvdata(pd);

	dev_dbg(dev, "%s\n", __func__);

	if (hsi_is_hsi_controller_busy(hsi_ctrl)) {
		dev_dbg(dev, "hsi_runtime_idle: HSI port busy\n");
		return -EBUSY;
	}

	if (hsi_is_hst_controller_busy(hsi_ctrl)) {
		dev_dbg(dev, "hsi_runtime_idle: HST FSM not IDLE !\n");
		return -EBUSY;
	}

	/* HSI_TODO : check also the interrupt status registers.*/

	return 0;
}

#endif /* CONFIG_PM_RUNTIME */

int hsi_driver_device_is_hsi(struct platform_device *dev)
{
	struct platform_device_id *id =
	    (struct platform_device_id *)platform_get_device_id(dev);
	return (id->driver_data == HSI_DRV_DEVICE_HSI);
}

/* List of devices supported by this driver */
static struct platform_device_id hsi_id_table[] = {
	{"omap_hsi", HSI_DRV_DEVICE_HSI},
	{"omap_ssi", HSI_DRV_DEVICE_SSI},
	{},
};

MODULE_DEVICE_TABLE(platform, hsi_id_table);

#ifdef CONFIG_PM
static const struct dev_pm_ops hsi_driver_pm_ops = {
#ifdef CONFIG_SUSPEND
	.prepare = hsi_pm_prepare,
	.suspend = hsi_pm_suspend,
	.suspend_noirq = hsi_pm_suspend_noirq,
	.resume = hsi_pm_resume,
#endif
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = hsi_runtime_suspend,
	.runtime_resume = hsi_runtime_resume,
	.runtime_idle = hsi_runtime_idle,
#endif
};

#define HSI_DRIVER_PM_OPS_PTR (&hsi_driver_pm_ops)

#else /* !CONFIG_PM */

#define HSI_DRIVER_PM_OPS_PTR NULL

#endif

static struct platform_driver hsi_pdriver = {
	.driver = {
		   .name = HSI_MODULENAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = HSI_DRIVER_PM_OPS_PTR,
#endif
		   },
	.id_table = hsi_id_table,
	.remove = __exit_p(hsi_platform_device_remove),
};

/* HSI bus and platform driver registration */
static int __init hsi_driver_init(void)
{
	int err = 0;

	pr_info(LOG_NAME "HSI driver version " HSI_DRIVER_VERSION "\n");

	/* Register the (virtual) HSI bus */
	err = hsi_bus_init();
	if (err < 0) {
		pr_err(LOG_NAME "HSI bus_register err %d\n", err);
		return err;
	}

	err = hsi_debug_init();
	if (err < 0) {
		pr_err(LOG_NAME "HSI Debugfs failed %d\n", err);
		goto rback1;
	}

	/* Register the HSI platform driver */
	err = platform_driver_probe(&hsi_pdriver, hsi_platform_device_probe);
	if (err < 0) {
		pr_err(LOG_NAME "Platform DRIVER register FAILED: %d\n", err);
		goto rback2;
	}

	return 0;
rback2:
	hsi_debug_exit();
rback1:
	hsi_bus_exit();
	return err;
}

static void __exit hsi_driver_exit(void)
{
	platform_driver_unregister(&hsi_pdriver);
	hsi_debug_exit();
	hsi_bus_exit();

	pr_info(LOG_NAME "HSI DRIVER removed\n");
}

module_init(hsi_driver_init);
module_exit(hsi_driver_exit);

MODULE_ALIAS("platform:" HSI_MODULENAME);
MODULE_AUTHOR("Carlos Chinea / Nokia");
MODULE_AUTHOR("Sebastien JAN / Texas Instruments");
MODULE_AUTHOR("Djamil ELAIDI / Texas Instruments");
MODULE_DESCRIPTION("MIPI High-speed Synchronous Serial Interface (HSI) Driver");
MODULE_LICENSE("GPL");
