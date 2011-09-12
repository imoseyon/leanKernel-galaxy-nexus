/*
 * arch/arm/mach-omap2/hsi.c
 *
 * HSI device definition
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Original Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <linux/hsi_driver_if.h>

#include <asm/clkdev.h>

#include <plat/omap_hsi.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>

#include <../drivers/omap_hsi/hsi_driver.h>
#include "clock.h"
#include "mux.h"
#include "control.h"

static int omap_hsi_wakeup_enable(int hsi_port);
static int omap_hsi_wakeup_disable(int hsi_port);
#define OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME	"omap_hsi"
#define OMAP_HSI_PLATFORM_DEVICE_NAME		"omap_hsi.0"
#define OMAP_HSI_HWMOD_NAME			"hsi"
#define OMAP_HSI_HWMOD_CLASSNAME		"hsi"
#define OMAP_HSI_PADCONF_CAWAKE_PIN		"usbb1_ulpitll_clk.hsi1_cawake"
#define OMAP_HSI_PADCONF_CAWAKE_MODE           OMAP_MUX_MODE1


#define OMAP_MUX_MODE_MASK	0x7


/* Hack till correct hwmod-mux api gets used */
#define CA_WAKE_MUX_REG		(0x4a1000C2)
#define OMAP44XX_PADCONF_WAKEUPENABLE0 (1 << 14)
#define OMAP44XX_PADCONF_WAKEUPEVENT0  (1 << 15)

static int omap_mux_read_signal(const char *muxname)
{
	u16 val = 0;
	val = omap_readw(CA_WAKE_MUX_REG);
	return val;
}

static int omap_mux_enable_wakeup(const char *muxname)
{
	u16 val = 0;
	val = omap_readw(CA_WAKE_MUX_REG);
	val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_writew(val, CA_WAKE_MUX_REG);
	return 0;
}

static int omap_mux_disable_wakeup(const char *muxname)
{
	u16 val = 0;
	val = omap_readw(CA_WAKE_MUX_REG);
	val &= ~OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_writew(val, CA_WAKE_MUX_REG);
	return 0;
}

/*
 * NOTE: We abuse a little bit the struct port_ctx to use it also for
 * initialization.
 */


static struct port_ctx hsi_port_ctx[] = {
	[0] = {
	       .hst.mode = HSI_MODE_FRAME,
	       .hst.flow = HSI_FLOW_SYNCHRONIZED,
	       .hst.frame_size = HSI_FRAMESIZE_DEFAULT,
	       .hst.divisor = HSI_DIVISOR_DEFAULT,
	       .hst.channels = HSI_CHANNELS_DEFAULT,
	       .hst.arb_mode = HSI_ARBMODE_ROUNDROBIN,
	       .hsr.mode = HSI_MODE_FRAME,
	       .hsr.flow = HSI_FLOW_SYNCHRONIZED,
	       .hsr.frame_size = HSI_FRAMESIZE_DEFAULT,
	       .hsr.channels = HSI_CHANNELS_DEFAULT,
	       .hsr.divisor = HSI_DIVISOR_DEFAULT,
	       .hsr.counters = HSI_COUNTERS_FT_DEFAULT |
			       HSI_COUNTERS_TB_DEFAULT |
			       HSI_COUNTERS_FB_DEFAULT,
	       },
};

static struct ctrl_ctx hsi_ctx = {
		.sysconfig = 0,
		.gdd_gcr = 0,
		.dll = 0,
		.pctx = hsi_port_ctx,
};

static struct hsi_platform_data omap_hsi_platform_data = {
	.num_ports = ARRAY_SIZE(hsi_port_ctx),
	.hsi_gdd_chan_count = HSI_HSI_DMA_CHANNEL_MAX,
	.default_hsi_fclk = HSI_DEFAULT_FCLK,
	.ctx = &hsi_ctx,
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
	.device_shutdown = omap_device_shutdown,
	.wakeup_enable = omap_hsi_wakeup_enable,
	.wakeup_disable = omap_hsi_wakeup_disable,
	.wakeup_is_from_hsi = omap_hsi_is_io_wakeup_from_hsi,
	.board_suspend = omap_hsi_prepare_suspend,
};


static struct platform_device *hsi_get_hsi_platform_device(void)
{
	struct device *dev;
	struct platform_device *pdev;

	/* HSI_TODO: handle platform device id (or port) (0/1) */
	dev = bus_find_device_by_name(&platform_bus_type, NULL,
					OMAP_HSI_PLATFORM_DEVICE_NAME);
	if (!dev) {
		pr_debug("Could not find platform device %s\n",
		       OMAP_HSI_PLATFORM_DEVICE_NAME);
		return 0;
	}

	if (!dev->driver) {
		/* Could not find driver for platform device. */
		return 0;
	}

	pdev = to_platform_device(dev);

	return pdev;
}

static struct hsi_dev *hsi_get_hsi_controller_data(struct platform_device *pd)
{
	struct hsi_dev *hsi_ctrl;

	if (!pd)
		return 0;

	hsi_ctrl = (struct hsi_dev *) platform_get_drvdata(pd);
	if (!hsi_ctrl) {
		pr_err("Could not find HSI controller data\n");
		return 0;
	}

	return hsi_ctrl;
}

/**
* omap_hsi_is_io_pad_hsi - Indicates if IO Pad has been muxed for HSI CAWAKE
*
* Return value :* 0 if CAWAKE Padconf has not been found or CAWAKE not muxed for
*		CAWAKE
*		* else 1
*/
static int omap_hsi_is_io_pad_hsi(void)
{
	u16 val;

	/* Check for IO pad */
	val = omap_mux_read_signal(OMAP_HSI_PADCONF_CAWAKE_PIN);
	if (val == -ENODEV)
		return 0;

	/* Continue only if CAWAKE is muxed */
	if ((val & OMAP_MUX_MODE_MASK) != OMAP_HSI_PADCONF_CAWAKE_MODE)
		return 0;

	return 1;
}

/**
* omap_hsi_is_io_wakeup_from_hsi - Indicates an IO wakeup from HSI CAWAKE
*
* Return value :* 0 if CAWAKE Padconf has not been found or no IOWAKEUP event
*		occured for CAWAKE
*		* else 1
* TODO : return value should indicate the HSI port which has awaken
*/
int omap_hsi_is_io_wakeup_from_hsi(void)
{
	u16 val;

	/* Check for IO pad wakeup */
	val = omap_mux_read_signal(OMAP_HSI_PADCONF_CAWAKE_PIN);
	if (val == -ENODEV)
		return 0;

	/* Continue only if CAWAKE is muxed */
	if ((val & OMAP_MUX_MODE_MASK) != OMAP_HSI_PADCONF_CAWAKE_MODE)
		return 0;

	if (val & OMAP44XX_PADCONF_WAKEUPEVENT0)
		return 1;

	return 0;
}

/**
* omap_hsi_wakeup_enable - Enable HSI wakeup feature from RET/OFF mode
*
* @hsi_port - reference to the HSI port onto which enable wakeup feature.
*
* Return value :* 0 if CAWAKE has been configured to wakeup platform
*		* -ENODEV if CAWAKE is not muxed on padconf
*/
static int omap_hsi_wakeup_enable(int hsi_port)
{
	int ret = -ENODEV;

	if (omap_hsi_is_io_pad_hsi())
		ret = omap_mux_enable_wakeup(OMAP_HSI_PADCONF_CAWAKE_PIN);
	else
		pr_debug("Trying to enable HSI IO wakeup on non HSI board\n");


	/* TODO: handle hsi_port param and use it to find the correct Pad */
	return ret;
}

/**
* omap_hsi_wakeup_disable - Disable HSI wakeup feature from RET/OFF mode
*
* @hsi_port - reference to the HSI port onto which disable wakeup feature.
*
* Return value :* 0 if CAWAKE has been configured to not wakeup platform
*		* -ENODEV if CAWAKE is not muxed on padconf
*/
static int omap_hsi_wakeup_disable(int hsi_port)
{
	int ret = -ENODEV;

	if (omap_hsi_is_io_pad_hsi())
		ret = omap_mux_disable_wakeup(OMAP_HSI_PADCONF_CAWAKE_PIN);
	else
		pr_debug("Trying to disable HSI IO wakeup on non HSI board\n");


	/* TODO: handle hsi_port param and use it to find the correct Pad */

	return ret;
}

/**
* omap_hsi_prepare_suspend - Prepare HSI for suspend mode
*
* Return value :* 0 if CAWAKE padconf has been configured properly
*		* -ENODEV if CAWAKE is not muxed on padconf.
*
*/
int omap_hsi_prepare_suspend(int hsi_port, bool dev_may_wakeup)
{
	int ret;

	if (dev_may_wakeup)
		ret = omap_hsi_wakeup_enable(hsi_port);
	else
		ret = omap_hsi_wakeup_disable(hsi_port);

	return ret;
}

/**
* omap_hsi_wakeup - Prepare HSI for wakeup from suspend mode (RET/OFF)
*
* @hsi_port - reference to the HSI port which triggered wakeup.
*	      Range [1, 2]
*
* Return value : * 0 if HSI tasklet scheduled.
*		 * negative value else.
*/
int omap_hsi_wakeup(int hsi_port)
{
	static struct platform_device *pdev;
	static struct hsi_dev *hsi_ctrl;

	if (!pdev) {
		pdev = hsi_get_hsi_platform_device();
		if (!pdev)
			return -ENODEV;
	}

	if (!device_may_wakeup(&pdev->dev)) {
		dev_info(&pdev->dev, "Modem not allowed to wakeup platform");
		return -EPERM;
	}

	if (!hsi_ctrl) {
		hsi_ctrl = hsi_get_hsi_controller_data(pdev);
		if (!hsi_ctrl)
			return -ENODEV;
	}

	/* Check no other interrupt handler has already scheduled the tasklet */
	if (test_and_set_bit(HSI_FLAGS_TASKLET_LOCK,
			     &hsi_ctrl->hsi_port->flags))
		return -EBUSY;

	dev_dbg(hsi_ctrl->dev, "Modem wakeup detected from HSI CAWAKE Pad");

	/* CAWAKE falling or rising edge detected */
	hsi_ctrl->hsi_port->cawake_off_event = true;
		tasklet_hi_schedule(&hsi_ctrl->hsi_port->hsi_tasklet);

	/* Disable interrupt until Bottom Half has cleared */
	/* the IRQ status register */
		disable_irq_nosync(hsi_ctrl->hsi_port->irq);

	 return 0;
}

/* HSI_TODO : This requires some fine tuning & completion of
 * activate/deactivate latency values
 */
static struct omap_device_pm_latency omap_hsi_latency[] = {
	[0] = {
	       .deactivate_func = omap_device_idle_hwmods,
	       .activate_func = omap_device_enable_hwmods,
	       .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	       },
};

/* HSI device registration */
static int __init omap_hsi_register(struct omap_hwmod *oh, void *user)
{
	struct omap_device *od;
	struct hsi_platform_data *pdata = &omap_hsi_platform_data;

	if (!oh) {
		pr_err("Could not look up %s omap_hwmod\n",
		       OMAP_HSI_HWMOD_NAME);
		return -EEXIST;
	}

	od = omap_device_build(OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME, 0, oh,
			       pdata, sizeof(*pdata), omap_hsi_latency,
			       ARRAY_SIZE(omap_hsi_latency), false);
	WARN(IS_ERR(od), "Can't build omap_device for %s:%s.\n",
	     OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME, oh->name);

	pr_info("HSI: device registered as omap_hwmod: %s\n", oh->name);
	return 0;
}

static void __init omap_4430hsi_pad_conf(void)
{
	/*
	 * HSI pad conf: hsi1_ca/ac_wake/flag/data/ready
	 * Also configure gpio_92/95/157/187 used by modem
	 */
	/* hsi1_cawake */
	omap_mux_init_signal("usbb1_ulpitll_clk.hsi1_cawake", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE | \
		OMAP_PIN_OFF_WAKEUPENABLE);
	/* hsi1_caflag */
	omap_mux_init_signal("usbb1_ulpitll_dir.hsi1_caflag", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_cadata */
	omap_mux_init_signal("usbb1_ulpitll_stp.hsi1_cadata", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acready */
	omap_mux_init_signal("usbb1_ulpitll_nxt.hsi1_acready", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_OUTPUT_LOW);
	/* hsi1_acwake */
	omap_mux_init_signal("usbb1_ulpitll_dat0.hsi1_acwake", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acdata */
	omap_mux_init_signal("usbb1_ulpitll_dat1.hsi1_acdata", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_acflag */
	omap_mux_init_signal("usbb1_ulpitll_dat2.hsi1_acflag", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* hsi1_caready */
	omap_mux_init_signal("usbb1_ulpitll_dat3.hsi1_caready", \
		OMAP_PIN_INPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_92 */
	omap_mux_init_signal("usbb1_ulpitll_dat4.gpio_92", \
		OMAP_PULL_ENA);
	/* gpio_95 */
	omap_mux_init_signal("usbb1_ulpitll_dat7.gpio_95", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE);
	/* gpio_157 */
	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
	/* gpio_187 */
	omap_mux_init_signal("sys_boot3.gpio_187", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);
}

int __init omap_hsi_dev_init(void)
{
	/* Keep this for genericity, although there is only one hwmod for HSI */
	return omap_hwmod_for_each_by_class(OMAP_HSI_HWMOD_CLASSNAME,
					    omap_hsi_register, NULL);
}
postcore_initcall(omap_hsi_dev_init);

/* HSI devices registration */
int __init omap_hsi_init(void)
{
	omap_4430hsi_pad_conf();
	return 0;
}
