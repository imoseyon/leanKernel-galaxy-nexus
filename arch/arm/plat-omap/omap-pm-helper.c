/*
 * omap-pm.c - OMAP power management interface
 *
 * Copyright (C) 2008-2011 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 * Vishwanath BS
 *
 * This code is based on plat-omap/omap-pm-noop.c.
 *
 * Interface developed by (in alphabetical order):
 * Karthik Dasu, Tony Lindgren, Rajendra Nayak, Sakari Poussa, Veeramanikandan
 * Raju, Anand Sawant, Igor Stoppa, Paul Walmsley, Richard Woodruff
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

/* Interface documentation is in mach/omap-pm.h */
#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include "../mach-omap2/powerdomain.h"
#include "../mach-omap2/dvfs.h"
#include "omap-pm-helper.h"

struct omap_opp *dsp_opps;
struct omap_opp *mpu_opps;
struct omap_opp *l3_opps;

static DEFINE_MUTEX(bus_tput_mutex);
static DEFINE_MUTEX(mpu_tput_mutex);
static DEFINE_MUTEX(mpu_lat_mutex);

/* Used to model a Interconnect Throughput */
static struct interconnect_tput {
	/* Total no of users at any point of interconnect */
	u8 no_of_users;
	/* List of all the current users for interconnect */
	struct list_head users_list;
	struct list_head node;
	/* Protect interconnect throughput */
	struct mutex throughput_mutex;
	/* Target level for interconnect throughput */
	u64 target_level;

} *bus_tput;

/* Used to represent a user of a interconnect throughput */
struct users {
	/* Device pointer used to uniquely identify the user */
	struct device *dev;
	struct list_head node;
	/* Current level as requested for interconnect throughput by the user */
	u32 level;
};

/* Private/Internal Functions */

/**
 * user_lookup - look up a user by its device pointer, return a pointer
 * @dev: The device to be looked up
 *
 * Looks for a interconnect user by its device pointer. Returns a
 * pointer to
 * the struct users if found, else returns NULL.
 */
static struct users *user_lookup(struct device *dev)
{
	struct users *usr, *tmp_usr;

	usr = NULL;
	list_for_each_entry(tmp_usr, &bus_tput->users_list, node) {
		if (tmp_usr->dev == dev) {
			usr = tmp_usr;
			break;
		}
	}

	return usr;
}

/**
 * get_user - gets a new users_list struct dynamically
 *
 * This function allocates dynamcially the user node
 * Returns a pointer to users struct on success. On dynamic allocation
 * failure
 * returns a ERR_PTR(-ENOMEM).
 */
static struct users *get_user(void)
{
	struct users *user;

	user = kmalloc(sizeof(struct users), GFP_KERNEL);
	if (!user) {
		pr_err("%s FATAL ERROR: kmalloc failed\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	return user;
}

#ifdef CONFIG_PM_DEBUG
static int pm_dbg_show_tput(struct seq_file *s, void *unused)
{
	struct users *usr;

	mutex_lock(&bus_tput->throughput_mutex);
	list_for_each_entry(usr, &bus_tput->users_list, node)
		seq_printf(s, "%s:	%u\n", dev_name(usr->dev),
				usr->level);
	mutex_unlock(&bus_tput->throughput_mutex);

	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_tput,
			&inode->i_private);
}

static const struct file_operations tputdebugfs_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

/**
 * omap_bus_tput_init - Initializes the interconnect throughput
 * userlist
 * Allocates memory for global throughput variable dynamically.
 * Intializes Userlist, no. of users and throughput target level.
 * Returns 0 on sucess, else returns EINVAL if memory
 * allocation fails.
 */
static int __init omap_bus_tput_init(void)
{
	bus_tput = kmalloc(sizeof(struct interconnect_tput), GFP_KERNEL);
	if (!bus_tput) {
		pr_err("%s FATAL ERROR: kmalloc failed\n", __func__);
		return -EINVAL;
	}
	INIT_LIST_HEAD(&bus_tput->users_list);
	mutex_init(&bus_tput->throughput_mutex);
	bus_tput->no_of_users = 0;
	bus_tput->target_level = 0;

#ifdef CONFIG_PM_DEBUG
	(void) debugfs_create_file("tput", S_IRUGO,
		NULL, (void *)bus_tput, &tputdebugfs_fops);
#endif

	return 0;
}

/**
 * add_req_tput  - Request for a required level by a device
 * @dev: Uniquely identifes the caller
 * @level: The requested level for the interconnect bandwidth in KiB/s
 *
 * This function recomputes the target level of the interconnect
 * bandwidth
 * based on the level requested by all the users.
 * Multiple calls to this function by the same device will
 * replace the previous level requested
 * Returns the updated level of interconnect throughput.
 * In case of Invalid dev or user pointer, it returns 0.
 */
static u64 add_req_tput(struct device *dev, unsigned long level)
{
	u64 ret = 0;
	struct users *user;

	if (!dev) {
		pr_err("Invalid dev pointer\n");
		return ret;
	}
	mutex_lock(&bus_tput->throughput_mutex);
	user = user_lookup(dev);
	if (user == NULL) {
		user = get_user();
		if (IS_ERR(user)) {
			pr_err("Couldn't get user from the list to"
			       "add new throughput constraint");
			ret = 0;
			goto unlock;
		}
		bus_tput->target_level += level;
		bus_tput->no_of_users++;
		user->dev = dev;
		list_add(&user->node, &bus_tput->users_list);
		user->level = level;
	} else {
		bus_tput->target_level -= user->level;
		bus_tput->target_level += level;
		user->level = level;
	}
	ret = bus_tput->target_level;
unlock:
	mutex_unlock(&bus_tput->throughput_mutex);
	return ret;
}

/**
 * remove_req_tput - Release a previously requested level of
 * a throughput level for interconnect
 * @dev: Device pointer to dev
 *
 * This function recomputes the target level of the interconnect
 * throughput after removing
 * the level requested by the user.
 * Returns 0, if the dev structure is invalid
 * else returns modified interconnect throughput rate.
 */
static u64 remove_req_tput(struct device *dev)
{
	struct users *user;
	int found = 0;
	u64 ret = 0;

	mutex_lock(&bus_tput->throughput_mutex);
	list_for_each_entry(user, &bus_tput->users_list, node) {
		if (user->dev == dev) {
			found = 1;
			break;
		}
	}
	if (!found) {
		/* No such user exists */
		pr_err("Invalid Device Structure\n");
		goto unlock;
	}
	bus_tput->target_level -= user->level;
	bus_tput->no_of_users--;
	list_del(&user->node);
	kfree(user);
	ret = bus_tput->target_level;
unlock:
	mutex_unlock(&bus_tput->throughput_mutex);
	return ret;
}

int omap_pm_set_min_bus_tput_helper(struct device *dev, u8 agent_id, long r)
{

	int ret = 0;
	struct device *l3_dev;
	static struct device dummy_l3_dev = {
		.init_name = "omap_pm_set_min_bus_tput",
	};
	u64 target_level = 0;
	unsigned long freq = ULONG_MAX;

	mutex_lock(&bus_tput_mutex);

	l3_dev = omap2_get_l3_device();
	if (!l3_dev) {
		pr_err("Unable to get l3 device pointer");
		ret = -EINVAL;
		goto unlock;
	}

	/* find maximum supported opp frequency */
	rcu_read_lock();
	opp_find_freq_floor(l3_dev, &freq);
	rcu_read_unlock();

	if (r == -1)
		target_level = remove_req_tput(dev);
	else
		target_level = add_req_tput(dev, r);

	/* Convert the throughput(in KiB/s) into Hz. */
	target_level = (target_level>>2) * 1000;
	/* Align wit max supported frequency (freq holds max) */
	freq = (target_level > (u64)freq) ? freq : (unsigned long)target_level;

	ret = omap_device_scale(&dummy_l3_dev, l3_dev, freq);
	if (ret)
		pr_err("Failed: change interconnect bandwidth to %lld\n",
		     target_level);
unlock:
	mutex_unlock(&bus_tput_mutex);
	return ret;
}

int omap_pm_set_max_dev_wakeup_lat_helper(struct device *req_dev,
					  struct device *dev, long t)
{
	struct omap_device *odev;
	struct powerdomain *pwrdm_dev;
	struct platform_device *pdev;
	int ret = 0;

	if (!req_dev || !dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	};

	/* Look for the devices Power Domain */
	pdev = container_of(dev, struct platform_device, dev);

	/* Try to catch non platform devices. */
	if (pdev->name == NULL) {
		pr_err("OMAP-PM: Error: platform device not valid\n");
		return -EINVAL;
	}

	odev = to_omap_device(pdev);
	if (odev) {
		pwrdm_dev = omap_device_get_pwrdm(odev);
	} else {
		pr_err("OMAP-PM: Error: Could not find omap_device for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	/* Catch devices with undefined powerdomains. */
	if (!pwrdm_dev) {
		pr_err("OMAP-PM: Error: could not find parent pwrdm for %s\n",
		       pdev->name);
		return -EINVAL;
	}

	if (t == -1)
		ret = pwrdm_wakeuplat_release_constraint(pwrdm_dev, req_dev);
	else
		ret = pwrdm_wakeuplat_set_constraint(pwrdm_dev, req_dev, t);

	return ret;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init_helper(void)
{
	int ret;
	ret = omap_bus_tput_init();
	if (ret)
		pr_err("Failed: init of interconnect bandwidth users list\n");
	return ret;
}
