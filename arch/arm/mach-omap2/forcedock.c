/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 * Docking mode: Mike O'Connor <moconnore@gmail.com>
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

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/forcedock.h>

int force_dock_mode;
int dock_mode;
int charge_mode;

/* sysfs interface */
static ssize_t force_dock_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", force_dock_mode);
}

static ssize_t force_dock_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &force_dock_mode);
	return count;
}

static ssize_t dock_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dock_mode);
}

static ssize_t dock_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &dock_mode);
	return count;
}
static ssize_t charge_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", charge_mode);
}

static ssize_t charge_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &charge_mode);
	return count;
}


static struct kobj_attribute force_dock_mode_attribute = __ATTR(force_dock_mode, 0666, force_dock_mode_show, force_dock_mode_store);
static struct kobj_attribute dock_mode_attribute = __ATTR(dock_mode, 0666, dock_mode_show, dock_mode_store);
static struct kobj_attribute charge_mode_attribute = __ATTR(charge_mode, 0666, charge_mode_show, charge_mode_store);

static struct attribute *attrs[] = {
	&force_dock_mode_attribute.attr,
	&dock_mode_attribute.attr,
	&charge_mode_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {.attrs = attrs,};

static struct kobject *force_dock_mode_kobj;
static struct kobject *dock_mode_kobj;
static struct kobject *charge_mode_kobj;

int force_dock_mode_init(void)
{
	int retval;

	force_dock_mode = 0;
	dock_mode = 0;
	charge_mode = 0;

        force_dock_mode_kobj = kobject_create_and_add("dock_mode", kernel_kobj);
        if (!force_dock_mode_kobj) {
                return -ENOMEM;
        }

        retval = sysfs_create_group(force_dock_mode_kobj, &attr_group);
        if (retval)
                kobject_put(force_dock_mode_kobj);
        return retval;

	retval = sysfs_create_group(dock_mode_kobj, &attr_group);
        if (retval)
                kobject_put(dock_mode_kobj);
        return retval;

	retval = sysfs_create_group(charge_mode_kobj, &attr_group);
        if (retval)
                kobject_put(charge_mode_kobj);
        return retval;
}

/* end sysfs interface */

void force_dock_mode_exit(void)
{
	kobject_put(force_dock_mode_kobj);
	kobject_put(dock_mode_kobj);
	kobject_put(charge_mode_kobj);
}
module_init(force_dock_mode_init);
module_exit(force_dock_mode_exit);
