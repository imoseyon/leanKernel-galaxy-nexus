/* drivers/misc/temp_control.c
 * This code is completely free.
 * imoseyon@gmail.com
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define DEF_TEMPLIMIT 64000
#define TEMP_FLOOR 50000
#define TEMP_CEILING 90000

extern void tempcontrol_update(int tlimit);
static int temp_limit;

static ssize_t tempcontrol_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%d\n", temp_limit);
}

static ssize_t tempcontrol_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    sscanf(buf, "%d\n", &temp_limit);
    if (temp_limit == 0) temp_limit = DEF_TEMPLIMIT;
    else {
      if (temp_limit < TEMP_FLOOR) temp_limit = TEMP_FLOOR;
      if (temp_limit > TEMP_CEILING) temp_limit = TEMP_CEILING;
    }
    pr_info("[imoseyon] TEMPCONTROL threshold changed to %d\n", temp_limit);
    tempcontrol_update(temp_limit);
    return size;
}

static DEVICE_ATTR(templimit, 0666, tempcontrol_read, tempcontrol_write);

static struct attribute *tempcontrol_attributes[] = 
    {
	&dev_attr_templimit.attr,
	NULL
    };

static struct attribute_group tempcontrol_group = 
    {
	.attrs  = tempcontrol_attributes,
    };

static struct miscdevice tempcontrol_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tempcontrol",
    };

static int __init tempcontrol_init(void)
{
    int ret;
    temp_limit = DEF_TEMPLIMIT;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, tempcontrol_device.name);

    ret = misc_register(&tempcontrol_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, tempcontrol_device.name);
	    return 1;
	}

    if (sysfs_create_group(&tempcontrol_device.this_device->kobj, &tempcontrol_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", tempcontrol_device.name);
	}

    return 0;
}

device_initcall(tempcontrol_init);
