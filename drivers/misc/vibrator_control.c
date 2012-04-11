/* drivers/misc/vibrator_control.c
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

#define VIBRATORCONTROL_VERSION 1

#define MAX_VIBSTRENGTH 1600
#define MIN_VIBSTRENGTH 1000

static int vib_strength;

extern void vibratorcontrol_update(int vibstrength);

void vibratorcontrol_register_vibstrength(int vibstrength)
{
    vib_strength = vibstrength;

    return;
}
EXPORT_SYMBOL(vibratorcontrol_register_vibstrength);

static ssize_t vibratorcontrol_vibstrength_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%i\n", vib_strength);
}

static ssize_t vibratorcontrol_vibstrength_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if (sscanf(buf, "%u\n", &data) == 1)
	{
	    if (data != vib_strength)
		{
		    vib_strength = min(max(data, MIN_VIBSTRENGTH), MAX_VIBSTRENGTH);

		    pr_info("VIBRATORCONTROL vibrator strength changed to %i\n", vib_strength);

		    vibratorcontrol_update(vib_strength);
		}
	}
    else
	{
	    pr_info("VIBRATORCONTROL invalid input\n"); 
	}
	    
    return size;
}

static ssize_t vibratorcontrol_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", VIBRATORCONTROL_VERSION);
}

static DEVICE_ATTR(vibrator_strength, S_IRUGO | S_IWUGO, vibratorcontrol_vibstrength_read, vibratorcontrol_vibstrength_write);
static DEVICE_ATTR(version, S_IRUGO , vibratorcontrol_version, NULL);

static struct attribute *vibratorcontrol_attributes[] = 
    {
	&dev_attr_vibrator_strength.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group vibratorcontrol_group = 
    {
	.attrs  = vibratorcontrol_attributes,
    };

static struct miscdevice vibratorcontrol_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vibratorcontrol",
    };

static int __init vibratorcontrol_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, vibratorcontrol_device.name);

    ret = misc_register(&vibratorcontrol_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, vibratorcontrol_device.name);
	    return 1;
	}

    if (sysfs_create_group(&vibratorcontrol_device.this_device->kobj, &vibratorcontrol_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", vibratorcontrol_device.name);
	}

    return 0;
}

device_initcall(vibratorcontrol_init);
