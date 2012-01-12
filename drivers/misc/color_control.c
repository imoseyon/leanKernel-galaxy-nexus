/* drivers/misc/color_control.c
 *
 * Copyright 2012  Ezekeel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define COLORCONTROL_VERSION 1

extern void colorcontrol_update(int * v1_offsets);

static int v1_offsets[] = {0, 0, 0};

static ssize_t colorcontrol_v1offsets_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%i %i %i\n", v1_offsets[0], v1_offsets[1], v1_offsets[2]);
}

static ssize_t colorcontrol_v1offsets_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%i %i %i\n", &v1_offsets[0], &v1_offsets[1], &v1_offsets[2]) == 3) 
	{
	    pr_info("COLORCONTROL V1 offsets changed\n");

	    colorcontrol_update(v1_offsets);
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t colorcontrol_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", COLORCONTROL_VERSION);
}

static DEVICE_ATTR(v1_offsets, S_IRUGO | S_IWUGO, colorcontrol_v1offsets_read, colorcontrol_v1offsets_write);
static DEVICE_ATTR(version, S_IRUGO , colorcontrol_version, NULL);

static struct attribute *colorcontrol_attributes[] = 
    {
	&dev_attr_v1_offsets.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group colorcontrol_group = 
    {
	.attrs  = colorcontrol_attributes,
    };

static struct miscdevice colorcontrol_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "colorcontrol",
    };

static int __init colorcontrol_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, colorcontrol_device.name);

    ret = misc_register(&colorcontrol_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, colorcontrol_device.name);

	    return 1;
	}

    if (sysfs_create_group(&colorcontrol_device.this_device->kobj, &colorcontrol_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", colorcontrol_device.name);
	}

    return 0;
}

device_initcall(colorcontrol_init);
