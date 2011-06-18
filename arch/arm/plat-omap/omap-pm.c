/*
 * omap-pm.c - OMAP power management interface
 *
 * Copyright (C) 2008-2010 Texas Instruments, Inc.
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

/* Interface documentation is in mach/omap-pm.h */
#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include "../mach-omap2/powerdomain.h"

struct omap_opp *dsp_opps;
struct omap_opp *mpu_opps;
struct omap_opp *l3_opps;

static DEFINE_MUTEX(bus_tput_mutex);
static DEFINE_MUTEX(mpu_tput_mutex);
static DEFINE_MUTEX(mpu_lat_mutex);

static bool off_mode_enabled;

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
	unsigned long target_level;

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
 **/

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
 **/

static struct users *get_user(void)
{
	struct users *user;

	user = kmalloc(sizeof(struct  users), GFP_KERNEL);
	if (!user) {
		pr_err("%s FATAL ERROR: kmalloc "
			"failed\n",  __func__);
		return ERR_PTR(-ENOMEM);
	}
	return user;
}


/**
 * omap_bus_tput_init - Initializes the interconnect throughput
 * userlist
 * Allocates memory for global throughput variable dynamically.
 * Intializes Userlist, no. of users and throughput target level.
 * Returns 0 on sucess, else returns EINVAL if memory
 * allocation fails.
 */
int omap_bus_tput_init(void)
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
static unsigned long add_req_tput(struct device *dev, unsigned long level)
{
	int ret;
	struct  users *user;

	if (!dev) {
		pr_err("Invalid dev pointer\n");
		ret = 0;
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
static unsigned long remove_req_tput(struct device *dev)
{
	struct users *user;
	int found = 0;
	int ret;

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
		ret = 0;
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

/*
 * Device-driver-originated constraints (via board-*.c files)
 * WARNING: Device drivers need to now use pm_qos directly.
 */
int omap_pm_set_max_mpu_wakeup_lat(struct pm_qos_request_list **qos_request,
					long t)
{
	WARN(1, "Deprecated %s: Driver should use pm_qos to add request\n",
		__func__);

	return -EINVAL;
}

int omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, long r)
{

	int ret;
	struct device *l3_dev;
	static struct device dummy_l3_dev;
	unsigned long target_level = 0;

	if (!dev || (agent_id != OCP_INITIATOR_AGENT &&
	    agent_id != OCP_TARGET_AGENT)) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	};

	mutex_lock(&bus_tput_mutex);

	l3_dev = omap2_get_l3_device();
	if (!l3_dev) {
		pr_err("Unable to get l3 device pointer");
		ret = -EINVAL;
		goto unlock;
	}
	if (r == -1) {
		pr_debug("OMAP PM: remove min bus tput constraint for: "
			"interconnect dev %s for agent_id %d\n", dev_name(dev),
				agent_id);
		target_level = remove_req_tput(dev);
	} else {
		pr_debug("OMAP PM: add min bus tput constraint for: "
			"interconnect dev %s for agent_id %d: rate %ld KiB\n",
				dev_name(dev), agent_id, r);
		target_level = add_req_tput(dev, r);
	}

	/* Convert the throughput(in KiB/s) into Hz. */
	target_level = (target_level * 1000)/4;

	WARN(1, "OMAP PM: %s: constraint not called, needs DVFS", __func__);
#if 0
	ret = omap_device_scale(&dummy_l3_dev, l3_dev, target_level);
#endif
	if (ret)
		pr_err("Unable to change level for interconnect bandwidth to %ld\n",
			target_level);
unlock:
	mutex_unlock(&bus_tput_mutex);
	return ret;
}

int omap_pm_set_max_dev_wakeup_lat(struct device *req_dev, struct device *dev,
				   long t)
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
		pr_err("OMAP-PM: Error: Could not find omap_device "
			"for %s\n", pdev->name);
		return -EINVAL;
	}

	/* Catch devices with undefined powerdomains. */
	if (!pwrdm_dev) {
		pr_err("OMAP-PM: Error: could not find parent "
			"powerdomain for %s\n", pdev->name);
		return -EINVAL;
	}

	if (t == -1) {
		pr_debug("OMAP PM: remove max device latency constraint: "
			 "dev %s, pwrdm %s, req by %s\n", dev_name(dev),
				pwrdm_dev->name, dev_name(req_dev));
		ret = pwrdm_wakeuplat_release_constraint(pwrdm_dev, req_dev);
	} else {
		pr_debug("OMAP PM: add max device latency constraint: "
			 "dev %s, t = %ld usec, pwrdm %s, req by %s\n",
			 dev_name(dev), t, pwrdm_dev->name, dev_name(req_dev));
		ret = pwrdm_wakeuplat_set_constraint(pwrdm_dev, req_dev, t);
	}

	/*
	 * For current Linux, this needs to map the device to a
	 * powerdomain, then go through the list of current max lat
	 * constraints on that powerdomain and find the smallest.  If
	 * the latency constraint has changed, the code should
	 * recompute the state to enter for the next powerdomain
	 * state.  Conceivably, this code should also determine
	 * whether to actually disable the device clocks or not,
	 * depending on how long it takes to re-enable the clocks.
	 *
	 * TI CDP code can call constraint_set here.
	 */

	return ret;
}

/* WARNING: Device drivers need to now use pm_qos directly. */
int omap_pm_set_max_sdma_lat(struct pm_qos_request_list **qos_request,
					long t)
{
	WARN(1, "Deprecated %s: Driver should use pm_qos to add request\n",
		__func__);

	return -EINVAL;
}

int omap_pm_set_min_clk_rate(struct device *dev, struct clk *c, long r)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);
	return -EINVAL;
}

/*
 * DSP Bridge-specific constraints
 * WARNING: Device drivers need to now use opp layer/omap_device_scale directly.
 */
const struct omap_opp *omap_pm_dsp_get_opp_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_dsp_set_min_opp(u8 opp_id)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return;
}


u8 omap_pm_dsp_get_opp(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return 0;
}

/*
 * CPUFreq-originated constraint
 *
 * In the future, this should be handled by custom OPP clocktype
 * functions.
 */

struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_cpu_set_freq(unsigned long f)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return;
}

unsigned long omap_pm_cpu_get_freq(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return 0;
}

/**
 * omap_pm_enable_off_mode - notify OMAP PM that off-mode is enabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been enabled.
 */
void omap_pm_enable_off_mode(void)
{
	off_mode_enabled = true;
}

/**
 * omap_pm_disable_off_mode - notify OMAP PM that off-mode is disabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been disabled.
 */
void omap_pm_disable_off_mode(void)
{
	off_mode_enabled = false;
}

/*
 * Device context loss tracking
 */

int omap_pm_get_dev_context_loss_count(struct device *dev)
{
	static u32 counter = 1;

	if (!dev) {
		WARN_ON(1);
		return -EINVAL;
	};

	pr_debug("OMAP PM: returning context loss count for dev %s\n",
		 dev_name(dev));

	/*
	 * Map the device to the powerdomain.  Return the powerdomain
	 * off counter.
	 */

	/* Let the counter roll-over: its for test only */
	return counter++;
}


/* Should be called before clk framework init */
int __init omap_pm_if_early_init()
{
	return 0;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init(void)
{
	int ret;
	ret = omap_bus_tput_init();
	if (ret)
		pr_err("Failed to initialize interconnect"
			" bandwidth users list\n");
	return ret;
}

void omap_pm_if_exit(void)
{
	/* Deallocate CPUFreq frequency table here */
}

int omap_pm_set_min_mpu_freq(struct device *dev, unsigned long f)
{
	WARN(1, "Deprecated %s: Driver should NOT use this function\n",
		__func__);

	return -EINVAL;

}
EXPORT_SYMBOL(omap_pm_set_min_mpu_freq);
