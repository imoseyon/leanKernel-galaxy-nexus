/*
 * DMM driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <mach/dmm.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <mach/dmm.h>
#include <mach/tiler.h>

#ifdef CONFIG_TI_TILER

static struct omap_dmm_platform_data dmm_data = {
	.oh_name = "dmm",
};

static struct platform_device omap_tiler_device = {
	.name           = "tiler",
	.id             = -1,
};

static struct omap_device_pm_latency omap_dmm_latency[] = {
	[0] = {
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};


void __init omap_dmm_init(void)
{
	struct omap_hwmod *oh = NULL;
	struct omap_device *od = NULL;

	oh = omap_hwmod_lookup(dmm_data.oh_name);
	if (!oh)
		return;

	dmm_data.base = omap_hwmod_get_mpu_rt_va(oh);
	dmm_data.irq = oh->mpu_irqs[0].irq;

	od = omap_device_build(dmm_data.oh_name, -1, oh, &dmm_data,
				sizeof(dmm_data), omap_dmm_latency,
				ARRAY_SIZE(omap_dmm_latency), false);

	/* register tiler platform device to go along with the dmm device */
	if (platform_device_register(&omap_tiler_device) < 0)
		printk(KERN_ERR "Unable to register OMAP Tiler device\n");

	return;
}

#else
void __init omap_dmm_init(void)
{
}
#endif
