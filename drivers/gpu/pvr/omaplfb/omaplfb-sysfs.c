/*
 * omaplfb-sysfs.c
 *
 * Copyright (C) 2011 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Author: Gustavo Diaz (gusdp@ti.com)
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"

static ssize_t show_ignore_sync(OMAPLFB_DEVINFO *display_info, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", display_info->ignore_sync);
}

static ssize_t store_ignore_sync(OMAPLFB_DEVINFO *display_info,
	const char *buf, size_t count)
{
	unsigned long new_value;

	if (strict_strtoul(buf, 10, &new_value))
		return -EINVAL;

	if (new_value == 0 || new_value == 1) {
		display_info->ignore_sync = new_value;
		return count;
	}

	return -EINVAL;
}

struct omaplfb_attribute {
	struct attribute attr;
	ssize_t (*show)(OMAPLFB_DEVINFO *, char *);
	ssize_t (*store)(OMAPLFB_DEVINFO *, const char *, size_t);
};

static ssize_t omaplfb_attr_show(struct kobject *kobj, struct attribute *attr,
		char *buf)
{
	OMAPLFB_DEVINFO *display_info;
	struct omaplfb_attribute *omaplfb_attr;

	display_info = container_of(kobj, OMAPLFB_DEVINFO, kobj);
	omaplfb_attr = container_of(attr, struct omaplfb_attribute, attr);

	if (!omaplfb_attr->show)
		return -ENOENT;

	return omaplfb_attr->show(display_info, buf);
}

static ssize_t omaplfb_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t size)
{
	OMAPLFB_DEVINFO *display_info;
	struct omaplfb_attribute *omaplfb_attr;

	display_info = container_of(kobj, OMAPLFB_DEVINFO, kobj);
	omaplfb_attr = container_of(attr, struct omaplfb_attribute, attr);

	if (!omaplfb_attr->store)
		return -ENOENT;

	return omaplfb_attr->store(display_info, buf, size);
}

#define OMAPLFB_ATTR(_name, _mode, _show, _store) \
	struct omaplfb_attribute omaplfb_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static OMAPLFB_ATTR(ignore_sync, S_IRUGO|S_IWUSR, show_ignore_sync,
	store_ignore_sync);

#undef OMAPLFB_ATTR

static struct attribute *omaplfb_sysfs_attrs[] = {
	&omaplfb_attr_ignore_sync.attr,
	NULL
};

static const struct sysfs_ops omaplfb_sysfs_ops = {
	.show = omaplfb_attr_show,
	.store = omaplfb_attr_store,
};

static struct kobj_type omaplfb_ktype = {
	.sysfs_ops = &omaplfb_sysfs_ops,
	.default_attrs = omaplfb_sysfs_attrs,
};

void omaplfb_create_sysfs(struct omaplfb_device *odev)
{
	int i, r;

	/* Create a sysfs entry for every display */
	for (i = 0; i < odev->display_count; i++) {
		OMAPLFB_DEVINFO *display_info = &odev->display_info_list[i];
		r = kobject_init_and_add(&display_info->kobj, &omaplfb_ktype,
			&odev->dev->kobj, "display%d",
			display_info->uDeviceID);
		if (r)
			ERROR_PRINTK("failed to create sysfs file\n");
	}
}

void omaplfb_remove_sysfs(struct omaplfb_device *odev)
{
	int i;
	for (i = 0; i < odev->display_count; i++) {
		OMAPLFB_DEVINFO *display_info = &odev->display_info_list[i];
		kobject_del(&display_info->kobj);
		kobject_put(&display_info->kobj);
	}
}
