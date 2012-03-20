/* drivers/misc/sound_control.c
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
#include <linux/sound_control.h>

#define SOUNDCONTROL_VERSION 1

static bool high_perf_mode = false;

static unsigned int volume_boost = 0;

#define MAX_VOLUMEBOOST 3

extern void soundcontrol_updatevolume(unsigned int volumeboost);
extern void soundcontrol_updateperf(bool highperf_enabled);

static ssize_t soundcontrol_highperf_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", (high_perf_mode ? 1 : 0));
}

static ssize_t soundcontrol_highperf_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    pr_devel("%s: %u \n", __FUNCTION__, data);
	    
	    if (data == 1)
		{
		    if (!high_perf_mode) {
			pr_info("%s: SOUNDCONTROL high performance audio enabled\n", __FUNCTION__);

			high_perf_mode = true;

			soundcontrol_updateperf(high_perf_mode);
		    }
		} 
	    else if (data == 0) 
		{
		    if (high_perf_mode) {
			pr_info("%s: SOUNDCONTROL high performance audio disabled\n", __FUNCTION__);

			high_perf_mode = false;

			soundcontrol_updateperf(high_perf_mode);
		    }
		} 
	    else 
		{
		    pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t soundcontrol_volumeboost_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", volume_boost);
}

static ssize_t soundcontrol_volumeboost_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    if (data <= MAX_VOLUMEBOOST)
		{
		    if (data != volume_boost) {
			volume_boost = data;

			soundcontrol_updatevolume(volume_boost);

			pr_info("SOUNDCONTROL volume boost set to %u\n", volume_boost); 
		    }
		}
	    else
		{
		    pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t soundcontrol_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", SOUNDCONTROL_VERSION);
}

static DEVICE_ATTR(highperf_enabled, S_IRUGO | S_IWUGO, soundcontrol_highperf_read, soundcontrol_highperf_write);
static DEVICE_ATTR(volume_boost, S_IRUGO | S_IWUGO, soundcontrol_volumeboost_read, soundcontrol_volumeboost_write);
static DEVICE_ATTR(version, S_IRUGO , soundcontrol_version, NULL);

static struct attribute *soundcontrol_notification_attributes[] = 
    {
	&dev_attr_highperf_enabled.attr,
	&dev_attr_volume_boost.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group soundcontrol_notification_group = 
    {
	.attrs  = soundcontrol_notification_attributes,
    };

static struct miscdevice soundcontrol_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "soundcontrol",
    };

static int __init soundcontrol_control_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, soundcontrol_device.name);

    ret = misc_register(&soundcontrol_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, soundcontrol_device.name);

	    return 1;
	}

    if (sysfs_create_group(&soundcontrol_device.this_device->kobj, &soundcontrol_notification_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", soundcontrol_device.name);
	}

    return 0;
}

device_initcall(soundcontrol_control_init);
