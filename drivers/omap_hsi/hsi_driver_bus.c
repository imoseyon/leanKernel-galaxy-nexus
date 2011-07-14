/*
 * hsi_driver_bus.c
 *
 * Implements an HSI bus, device and driver interface.
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

#include <linux/device.h>
#include "hsi_driver.h"

#define HSI_PREFIX		"hsi:"

struct bus_type hsi_bus_type;

static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	return snprintf(buf, PAGE_SIZE + 1, "%s%s\n", HSI_PREFIX,
			dev_name(dev));
}

static struct device_attribute hsi_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static int hsi_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "MODALIAS=%s%s", HSI_PREFIX, dev_name(dev));
	return 0;
}

static int hsi_bus_match(struct device *device, struct device_driver *driver)
{
	struct hsi_device *dev = to_hsi_device(device);
	struct hsi_device_driver *drv = to_hsi_device_driver(driver);

	pr_debug("HSI DRIVER BUS : hsi_bus_match for ctrl:%d, port:%d, ch%d\n",
		 dev->n_ctrl, dev->n_p, dev->n_ch);

	if (!test_bit(dev->n_ctrl, &drv->ctrl_mask))
		return 0;

	if (!test_bit(dev->n_ch, &drv->ch_mask[dev->n_p]))
		return 0;

	pr_info
	    ("HSI DRIVER BUS : hsi_bus_match SUCCESS : ctrl:%d (mask:%x),"
		" port:%d, ch:%d (mask:%x)\n",
	     dev->n_ctrl, (u32) drv->ctrl_mask, dev->n_p, dev->n_ch,
	     (u32) drv->ch_mask[dev->n_p]);

	return 1;
}

int hsi_bus_unreg_dev(struct device *device, void *p)
{
	device->release(device);
	device_unregister(device);

	return 0;
}

int __init hsi_bus_init(void)
{
	return bus_register(&hsi_bus_type);
}

void hsi_bus_exit(void)
{
	bus_for_each_dev(&hsi_bus_type, NULL, NULL, hsi_bus_unreg_dev);
	bus_unregister(&hsi_bus_type);
}

static int hsi_bus_probe(struct device *dev)
{
	struct hsi_device_driver *drv;
	int rc;

	pr_debug("HSI DRIVER BUS : hsi_bus_probe\n");

	if (!dev->driver)
		return 0;

	drv = to_hsi_device_driver(dev->driver);

	if (!drv->probe)
		return -ENODEV;

	rc = drv->probe(to_hsi_device(dev));

	return rc;
}

static int hsi_bus_remove(struct device *dev)
{
	struct hsi_device_driver *drv;
	int ret;

	pr_debug("HSI DRIVER BUS : hsi_bus_remove\n");

	if (!dev->driver)
		return 0;

	drv = to_hsi_device_driver(dev->driver);
	if (drv->remove) {
		ret = drv->remove(to_hsi_device(dev));
	} else {
		dev->driver = NULL;
		ret = 0;
	}

	return ret;
}

static int hsi_bus_suspend(struct device *dev, pm_message_t mesg)
{
	struct hsi_device_driver *drv;

	if (!dev->driver)
		return 0;

	drv = to_hsi_device_driver(dev->driver);
	if (!drv->suspend)
		return 0;

	return drv->suspend(to_hsi_device(dev), mesg);
}

static int hsi_bus_resume(struct device *dev)
{
	struct hsi_device_driver *drv;

	if (!dev->driver)
		return 0;

	drv = to_hsi_device_driver(dev->driver);
	if (!drv->resume)
		return 0;

	return drv->resume(to_hsi_device(dev));
}

struct bus_type hsi_bus_type = {
	.name		= "hsi",
	.dev_attrs	= hsi_dev_attrs,
	.match		= hsi_bus_match,
	.uevent		= hsi_bus_uevent,
	.probe		= hsi_bus_probe,
	.remove		= hsi_bus_remove,
	.suspend	= hsi_bus_suspend,
	.resume		= hsi_bus_resume,
};

/**
 * hsi_register_driver - Register HSI device driver
 * @driver - reference to the HSI device driver.
 */
int hsi_register_driver(struct hsi_device_driver *driver)
{
	int ret = 0;

	if (driver == NULL)
		return -EINVAL;

	driver->driver.bus = &hsi_bus_type;

	ret = driver_register(&driver->driver);

	if (ret == 0)
		pr_debug("hsi: driver %s registered\n", driver->driver.name);

	return ret;
}
EXPORT_SYMBOL(hsi_register_driver);

/**
 * hsi_unregister_driver - Unregister HSI device driver
 * @driver - reference to the HSI device driver.
 */
void hsi_unregister_driver(struct hsi_device_driver *driver)
{
	if (driver == NULL)
		return;

	driver_unregister(&driver->driver);

	pr_debug("hsi: driver %s unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(hsi_unregister_driver);
