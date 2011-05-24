/*
 * Copyright (C) 2011 Google, Inc.
 *
 * Author: Dima Zavin <dima@android.com>
 *
 * Based on all the other android boards
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include "board-tuna.h"

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_all[] = {
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x4e31,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x18d1,
	.product_id		= 0x4e30,
	.version		= 0x0100,
	.product_name		= "Google, Inc.",
	.manufacturer_name	= "Tuna",
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &android_usb_pdata,
	},
};

static struct platform_device *android_devices[] __initdata = {
	&android_usb_device,
};

void __init omap4_tuna_android_usb_init(void)
{
	platform_add_devices(android_devices, ARRAY_SIZE(android_devices));
}
