/*
 * OMAP3/OMAP4 DVFS Management Routines
 *
 * Author: Vishwanath BS <vishwanath.bs@ti.com>
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Vishwanath BS <vishwanath.bs@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/plist.h>
#include <linux/slab.h>
#include <linux/opp.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <plat/common.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/clock.h>
#include "dvfs.h"
#include "smartreflex.h"
#include "powerdomain.h"
#include "pm.h"

/**
 * DOC: Introduction
 * =================
 * DVFS is a technique that uses the optimal operating frequency and voltage to
 * allow a task to be performed in the required amount of time.
 * OMAP processors have voltage domains whose voltage can be scaled to
 * various levels depending on which the operating frequencies of certain
 * devices belonging to the domain will also need to be scaled. This voltage
 * frequency tuple is known as Operating Performance Point (OPP). A device
 * can have multiple OPP's. Also a voltage domain could be shared between
 * multiple devices. Also there could be dependencies between various
 * voltage domains for maintaining system performance like VDD<X>
 * should be at voltage v1 when VDD<Y> is at voltage v2.
 *
 * The design of this framework takes into account all the above mentioned
 * points. To summarize the basic design of DVFS framework:-
 *
 * 1. Have device opp tables for each device whose operating frequency can be
 *    scaled. This is easy now due to the existance of hwmod layer which
 *    allow storing of device specific info. The device opp tables contain
 *    the opp pairs (frequency voltage tuples), the voltage domain pointer
 *    to which the device belongs to, the device specific set_rate and
 *    get_rate API's which will do the actual scaling of the device frequency
 *    and retrieve the current device frequency.
 * 2. Introduce use counting on a per VDD basis. This is to take care multiple
 *    requests to scale a VDD. The VDD will be scaled to the maximum of the
 *    voltages requested.
 * 3. Keep track of all scalable devices belonging to a particular voltage
 *    domain the voltage layer.
 * 4. Keep track of frequency requests for each of the device. This will enable
 *    to scale individual devices to different frequency (even w/o scaling
 *    voltage aka frequency throttling)
 * 5. Generic dvfs API that can be called by anybody to scale a device opp.
 *    This API takes the device pointer and frequency to which the device
 *    needs to be scaled to. This API then internally finds out the voltage
 *    domain to which the device belongs to and the voltage to which the voltage
 *    domain needs to be put to for the device to be scaled to the new frequency
 *    from the device opp table. Then this API will add requested frequency into
 *    the corresponding target device frequency list and add voltage request to
 *    the corresponding vdd. Subsequently it calls voltage scale function which
 *    will find out the highest requested voltage for the given vdd and scales
 *    the voltage to the required one and also adds corresponding frequency
 *    request for that voltage. It also runs through the list of all
 *    scalable devices belonging to this voltage domain and scale them to the
 *    appropriate frequencies using the set_rate pointer in the device opp
 *    tables.
 * 6. Handle inter VDD dependecies. This will take care of scaling domain's voltage
 *    and frequency together.
 *
 *
 * DOC: The Core DVFS data structure:
 * ==================================
 *  Structure Name                   Example Tree
 *  ---------
 *    /|\         +-------------------+      +-------------------+
 *     |          |User2 (dev2, freq2)+---\  |User4 (dev4, freq4)+---\
 *     |          +-------------------+   |  +-------------------+   |
 * (struct omap_dev_user_list)            |                          |
 *     |          +-------------------+   |  +-------------------+   |
 *     |          |User1 (dev1, freq1)+---|  |User3 (dev3, freq3)+---|
 *    \|/         +-------------------+   |  +-------------------+   |
 *  ---------                             |                          |
 *    /|\                    +------------+------+   +---------------+--+
 *     |                     | DEV1 (dev,        |   | DEV2 (dev)       |
 * (struct omap_vdd_dev_list)|omap_dev_user_list)|   |omap_dev_user_list|
 *     |                     +------------+------+   +--+---------------+
 *    \|/           /|\             /-----+-------------+------> others..
 *  ---------    Frequency          |
 *    /|\                        +--+------------------+
 *     |                         |       VDD_n         |
 *     |                         | (omap_vdd_dev_list, |
 * (struct omap_vdd_dvfs_info)** | omap_vdd_user_list) |
 *     |                         +--+------------------+
 *     |                            |   (ROOT NODE: omap_dvfs_info_list)
 *    \|/                           |
 *  ---------    Voltage            \---+-------------+----------> others..
 *    /|\          \|/          +-------+----+  +-----+--------+
 *     |                        |  vdd_user2 |  |   vdd_user3  |
 * (struct omap_vdd_user_list)  | (dev, volt)|  | (dev, volt)  |
 *    \|/                       +------------+  +--------------+
 *  ---------
 * Key: ** -> Root of the tree.
 * NOTE: we use the priority to store the voltage/frequency
 *
 * For voltage dependency description, see: struct dependency:
 * voltagedomain -> (description of the voltagedomain)
 *	omap_vdd_info -> (vdd information)
 *		omap_vdd_dep_info[]-> (stores array of depedency info)
 *			omap_vdd_dep_volt[] -> (stores array of maps)
 *				(main_volt -> dep_volt) (a singular map)
 */

/* Macros to give idea about scaling directions */
#define DVFS_VOLT_SCALE_DOWN	0
#define DVFS_VOLT_SCALE_NONE	1
#define DVFS_VOLT_SCALE_UP	2

/**
 * struct omap_dev_user_list - Structure maitain userlist per devide
 * @dev:	The device requesting for a particular frequency
 * @node:	The list head entry
 *
 * Using this structure, user list (requesting dev * and frequency) for
 * each device is maintained. This is how we can have different devices
 * at different frequencies (to support frequency locking and throttling).
 * Even if one of the devices in a given vdd has locked it's frequency,
 * other's can still scale their frequency using this list.
 * If no one has placed a frequency request for a device, then device is
 * set to the frequency from it's opp table.
 */
struct omap_dev_user_list {
	struct device *dev;
	struct plist_node node;
};

/**
 * struct omap_vdd_dev_list - Device list per vdd
 * @dev:	The device belonging to a particular vdd
 * @node:	The list head entry
 * @freq_user_list: The list of users for vdd device
 * @clk:	frequency control clock for this dev
 * @user_lock:	The lock for plist manipulation
 */
struct omap_vdd_dev_list {
	struct device *dev;
	struct list_head node;
	struct plist_head freq_user_list;
	struct clk *clk;
	spinlock_t user_lock; /* spinlock for plist */
};

/**
 * struct omap_vdd_user_list - The per vdd user list
 * @dev:	The device asking for the vdd to be set at a particular
 *		voltage
 * @node:	The list head entry
 */
struct omap_vdd_user_list {
	struct device *dev;
	struct plist_node node;
};

/**
 * struct omap_vdd_dvfs_info - The per vdd dvfs info
 * @node:	list node for vdd_dvfs_info list
 * @user_lock:	spinlock for plist operations
 * @vdd_user_list: The vdd user list
 * @voltdm:	Voltage domains for which dvfs info stored
 * @dev_list:	Device list maintained per domain
 *
 * This is a fundamental structure used to store all the required
 * DVFS related information for a vdd.
 */
struct omap_vdd_dvfs_info {
	struct list_head node;

	spinlock_t user_lock; /* spin lock */
	struct plist_head vdd_user_list;
	struct voltagedomain *voltdm;
	struct list_head dev_list;
};

static LIST_HEAD(omap_dvfs_info_list);
DEFINE_MUTEX(omap_dvfs_lock);

/* Dvfs scale helper function */
static int _dvfs_scale(struct device *req_dev, struct device *target_dev,
		struct omap_vdd_dvfs_info *tdvfs_info);

/* Few search functions to traverse and find pointers of interest */

/**
 * _dvfs_info_to_dev() - Locate the parent device associated to dvfs_info
 * @dvfs_info:	dvfs_info to search for
 *
 * Returns NULL on failure.
 */
static struct device *_dvfs_info_to_dev(struct omap_vdd_dvfs_info *dvfs_info)
{
	struct omap_vdd_dev_list *tmp_dev;
	if (IS_ERR_OR_NULL(dvfs_info))
		return NULL;
	if (list_empty(&dvfs_info->dev_list))
		return NULL;
	tmp_dev = list_first_entry(&dvfs_info->dev_list,
					struct omap_vdd_dev_list, node);
	return tmp_dev->dev;
}

/**
 * _dev_to_dvfs_info() - Locate the dvfs_info for a device
 * @dev:	dev to search for
 *
 * Returns NULL on failure.
 */
static struct omap_vdd_dvfs_info *_dev_to_dvfs_info(struct device *dev)
{
	struct omap_vdd_dvfs_info *dvfs_info;
	struct omap_vdd_dev_list *temp_dev;

	if (IS_ERR_OR_NULL(dev))
		return NULL;

	list_for_each_entry(dvfs_info, &omap_dvfs_info_list, node) {
		list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
			if (temp_dev->dev == dev)
				return dvfs_info;
		}
	}

	return NULL;
}

/**
 * _voltdm_to_dvfs_info() - Locate a dvfs_info given a voltdm pointer
 * @voltdm:	voltdm to search for
 *
 * Returns NULL on failure.
 */
static
struct omap_vdd_dvfs_info *_voltdm_to_dvfs_info(struct voltagedomain *voltdm)
{
	struct omap_vdd_dvfs_info *dvfs_info;

	if (IS_ERR_OR_NULL(voltdm))
		return NULL;

	list_for_each_entry(dvfs_info, &omap_dvfs_info_list, node) {
		if (dvfs_info->voltdm == voltdm)
			return dvfs_info;
	}

	return NULL;
}

/**
 * _volt_to_opp() - Find OPP corresponding to a given voltage
 * @dev:	device pointer associated with the OPP list
 * @volt:	voltage to search for in uV
 *
 * Searches for exact match in the OPP list and returns handle to the matching
 * OPP if found, else return the max available OPP.
 * If there are multiple opps with same voltage, it will return
 * the first available entry. Return pointer should be checked against IS_ERR.
 *
 * NOTE: since this uses OPP functions, use under rcu_lock. This function also
 * assumes that the cpufreq table and OPP table are in sync - any modifications
 * to either should be synchronized.
 */
static struct opp *_volt_to_opp(struct device *dev, unsigned long volt)
{
	struct opp *opp = ERR_PTR(-ENODEV);
	unsigned long f = 0;

	do {
		opp = opp_find_freq_ceil(dev, &f);
		if (IS_ERR(opp)) {
			/*
			 * if there is no OPP for corresponding volt
			 * then return max available instead
			 */
			opp = opp_find_freq_floor(dev, &f);
			break;
		}
		if (opp_get_voltage(opp) >= volt)
			break;
		f++;
	} while (1);

	return opp;
}

/* rest of the helper functions */
/**
 * _add_vdd_user() - Add a voltage request
 * @dvfs_info:	omap_vdd_dvfs_info pointer for the required vdd
 * @dev:	device making the request
 * @volt:	requested voltage in uV
 *
 * Adds the given device's voltage request into corresponding
 * vdd's omap_vdd_dvfs_info user list (plist). This list is used
 * to find the maximum voltage request for a given vdd.
 *
 * Returns 0 on success.
 */
static int _add_vdd_user(struct omap_vdd_dvfs_info *dvfs_info,
			struct device *dev, unsigned long volt)
{
	struct omap_vdd_user_list *user = NULL, *temp_user;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(dev, "%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	spin_lock(&dvfs_info->user_lock);
	plist_for_each_entry(temp_user, &dvfs_info->vdd_user_list, node) {
		if (temp_user->dev == dev) {
			user = temp_user;
			break;
		}
	}

	if (!user) {
		user = kzalloc(sizeof(struct omap_vdd_user_list), GFP_ATOMIC);
		if (!user) {
			dev_err(dev,
				"%s: Unable to creat a new user for vdd_%s\n",
				__func__, dvfs_info->voltdm->name);
			spin_unlock(&dvfs_info->user_lock);
			return -ENOMEM;
		}
		user->dev = dev;
	} else {
		plist_del(&user->node, &dvfs_info->vdd_user_list);
	}

	plist_node_init(&user->node, volt);
	plist_add(&user->node, &dvfs_info->vdd_user_list);

	spin_unlock(&dvfs_info->user_lock);
	return 0;
}

/**
 * _remove_vdd_user() - Remove a voltage request
 * @dvfs_info:	omap_vdd_dvfs_info pointer for the required vdd
 * @dev:	device making the request
 *
 * Removes the given device's voltage request from corresponding
 * vdd's omap_vdd_dvfs_info user list (plist).
 *
 * Returns 0 on success.
 */
static int _remove_vdd_user(struct omap_vdd_dvfs_info *dvfs_info,
		struct device *dev)
{
	struct omap_vdd_user_list *user = NULL, *temp_user;
	int ret = 0;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_err(dev, "%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	spin_lock(&dvfs_info->user_lock);
	plist_for_each_entry(temp_user, &dvfs_info->vdd_user_list, node) {
		if (temp_user->dev == dev) {
			user = temp_user;
			break;
		}
	}

	if (user)
		plist_del(&user->node, &dvfs_info->vdd_user_list);
	else {
		dev_err(dev, "%s: Unable to find the user for vdd_%s\n",
					__func__, dvfs_info->voltdm->name);
		ret = -ENOENT;
	}

	spin_unlock(&dvfs_info->user_lock);
	kfree(user);

	return ret;
}

/**
 * _add_freq_request() - Add a requested device frequency
 * @dvfs_info:	omap_vdd_dvfs_info pointer for the required vdd
 * @req_dev:	device making the request
 * @target_dev:	target device for which frequency request is being made
 * @freq:	target device frequency
 *
 * This adds a requested frequency into target device's frequency list.
 *
 * Returns 0 on success.
 */
static int _add_freq_request(struct omap_vdd_dvfs_info *dvfs_info,
	struct device *req_dev, struct device *target_dev, unsigned long freq)
{
	struct omap_dev_user_list *dev_user = NULL, *tmp_user;
	struct omap_vdd_dev_list *temp_dev;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(target_dev, "%s: VDD specified does not exist!\n",
			__func__);
		return -EINVAL;
	}

	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == target_dev)
			break;
	}

	if (temp_dev->dev != target_dev) {
		dev_warn(target_dev, "%s: target_dev does not exist!\n",
			__func__);
		return -EINVAL;
	}

	spin_lock(&temp_dev->user_lock);
	plist_for_each_entry(tmp_user, &temp_dev->freq_user_list, node) {
		if (tmp_user->dev == req_dev) {
			dev_user = tmp_user;
			break;
		}
	}

	if (!dev_user) {
		dev_user = kzalloc(sizeof(struct omap_dev_user_list),
					GFP_ATOMIC);
		if (!dev_user) {
			dev_err(target_dev,
				"%s: Unable to creat a new user for vdd_%s\n",
				__func__, dvfs_info->voltdm->name);
			spin_unlock(&temp_dev->user_lock);
			return -ENOMEM;
		}
		dev_user->dev = req_dev;
	} else {
		plist_del(&dev_user->node, &temp_dev->freq_user_list);
	}

	plist_node_init(&dev_user->node, freq);
	plist_add(&dev_user->node, &temp_dev->freq_user_list);
	spin_unlock(&temp_dev->user_lock);
	return 0;
}

/**
 * _remove_freq_request() - Remove the requested device frequency
 *
 * @dvfs_info:	omap_vdd_dvfs_info pointer for the required vdd
 * @req_dev:	device removing the request
 * @target_dev:	target device from which frequency request is being removed
 *
 * This removes a requested frequency from target device's frequency list.
 *
 * Returns 0 on success.
 */
static int _remove_freq_request(struct omap_vdd_dvfs_info *dvfs_info,
	struct device *req_dev, struct device *target_dev)
{
	struct omap_dev_user_list *dev_user = NULL, *tmp_user;
	int ret = 0;
	struct omap_vdd_dev_list *temp_dev;

	if (!dvfs_info || IS_ERR(dvfs_info)) {
		dev_warn(target_dev, "%s: VDD specified does not exist!\n",
			__func__);
		return -EINVAL;
	}


	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == target_dev)
			break;
	}

	if (temp_dev->dev != target_dev) {
		dev_warn(target_dev, "%s: target_dev does not exist!\n",
			__func__);
		return -EINVAL;
	}

	spin_lock(&temp_dev->user_lock);
	plist_for_each_entry(tmp_user, &temp_dev->freq_user_list, node) {
		if (tmp_user->dev == req_dev) {
			dev_user = tmp_user;
			break;
		}
	}

	if (dev_user) {
		plist_del(&dev_user->node, &temp_dev->freq_user_list);
	} else {
		dev_err(target_dev,
			"%s: Unable to remove the user for vdd_%s\n",
			__func__, dvfs_info->voltdm->name);
		ret = -EINVAL;
	}

	spin_unlock(&temp_dev->user_lock);
	kfree(dev_user);

	return ret;
}

/**
 * _dep_scan_table() - Scan a dependency table and mark for scaling
 * @dev:	device requesting the dependency scan (req_dev)
 * @dep_info:	dependency information (contains the table)
 * @main_volt:	voltage dependency to search for
 *
 * This runs down the table provided to find the match for main_volt
 * provided and sets up a scale request for the dependent domain
 * for the dependent voltage.
 *
 * Returns 0 if all went well.
 */
static int _dep_scan_table(struct device *dev,
		struct omap_vdd_dep_info *dep_info, unsigned long main_volt)
{
	struct omap_vdd_dep_volt *dep_table = dep_info->dep_table;
	struct device *target_dev;
	struct omap_vdd_dvfs_info *tdvfs_info;
	struct opp *opp;
	int i, ret;
	unsigned long dep_volt = 0, new_freq = 0;

	if (!dep_table) {
		dev_err(dev, "%s: deptable not present for vdd%s\n",
			__func__, dep_info->name);
		return -EINVAL;
	}

	/* Now scan through the the dep table for a match */
	for (i = 0; i < dep_info->nr_dep_entries; i++) {
		if (dep_table[i].main_vdd_volt == main_volt) {
			dep_volt = dep_table[i].dep_vdd_volt;
			break;
		}
	}
	if (!dep_volt) {
		dev_warn(dev, "%s: %ld volt map missing in vdd_%s\n",
			__func__, main_volt, dep_info->name);
		return -EINVAL;
	}

	/* populate voltdm if it is not present */
	if (!dep_info->_dep_voltdm) {
		dep_info->_dep_voltdm = voltdm_lookup(dep_info->name);
		if (!dep_info->_dep_voltdm) {
			dev_warn(dev, "%s: unable to get vdm%s\n",
				__func__, dep_info->name);
			return -ENODEV;
		}
	}

	/* See if dep_volt is possible for the vdd*/
	ret = _add_vdd_user(_voltdm_to_dvfs_info(dep_info->_dep_voltdm),
			dev, dep_volt);
	if (ret)
		dev_err(dev, "%s: Failed to add dep to domain %s volt=%ld\n",
				__func__, dep_info->name, dep_volt);

	/* And also add corresponding freq request */
	tdvfs_info = _voltdm_to_dvfs_info(dep_info->_dep_voltdm);
	if (!tdvfs_info) {
		dev_warn(dev, "%s: no dvfs_info\n",
				__func__);
		return -ENODEV;
	}
	target_dev = _dvfs_info_to_dev(tdvfs_info);
	if (!target_dev) {
		dev_warn(dev, "%s: no target_dev\n",
			__func__);
		return -ENODEV;
	}

	rcu_read_lock();
	opp = _volt_to_opp(target_dev, dep_volt);
	if (!IS_ERR(opp))
		new_freq = opp_get_freq(opp);
	rcu_read_unlock();

	if (new_freq) {
		ret = _add_freq_request(tdvfs_info, dev, target_dev, new_freq);
		if (ret) {
			dev_err(target_dev, "%s: freqadd(%s) failed %d[f=%ld,"
					"v=%ld]\n", __func__, dev_name(dev),
					i, new_freq, dep_volt);
			return ret;
		}
	}

	return ret;
}

/**
 * _dep_scan_domains() - Scan dependency domains for a device
 * @dev:	device requesting the scan
 * @vdd:	vdd_info corresponding to the device
 * @main_volt:	voltage to scan for
 *
 * Since each domain *may* have multiple dependent domains, we scan
 * through each of the dependent domains and invoke _dep_scan_table to
 * scan each table for dependent domain for dependency scaling.
 *
 * This assumes that the dependent domain information is NULL entry terminated.
 * Returns 0 if all went well.
 */
static int _dep_scan_domains(struct device *dev,
		struct omap_vdd_info *vdd, unsigned long main_volt)
{
	struct omap_vdd_dep_info *dep_info = vdd->dep_vdd_info;
	int ret = 0, r;

	if (!dep_info) {
		dev_dbg(dev, "%s: No dependent VDD\n", __func__);
		return 0;
	}

	/* First scan through the mydomain->dep_domain list */
	while (dep_info->nr_dep_entries) {
		r = _dep_scan_table(dev, dep_info, main_volt);
		/* Store last failed value */
		ret = (r) ? r : ret;
		dep_info++;
	}

	return ret;
}

/**
 * _dep_scale_domains() - Cause a scale of all dependent domains
 * @req_dev:	device requesting the scale
 * @req_vdd:	vdd_info corresponding to the requesting device.
 *
 * This walks through every dependent domain and triggers a scale
 * It is assumed that the corresponding scale handling for the
 * domain translates this to freq and voltage scale operations as
 * needed.
 *
 * Note: This is uses _dvfs_scale and one should be careful not to
 * create a circular depedency (e.g. vdd_mpu->vdd_core->vdd->mpu)
 * which can create deadlocks. No protection is provided to prevent
 * this condition and a tree organization is assumed.
 *
 * Returns 0 if all went fine.
 */
static int _dep_scale_domains(struct device *req_dev,
				struct omap_vdd_info *req_vdd)
{
	struct omap_vdd_dep_info *dep_info = req_vdd->dep_vdd_info;
	int ret = 0, r;

	if (!dep_info) {
		dev_dbg(req_dev, "%s: No dependent VDD\n", __func__);
		return 0;
	}

	/* First scan through the mydomain->dep_domain list */
	while (dep_info->nr_dep_entries) {
		struct voltagedomain *tvoltdm = dep_info->_dep_voltdm;

		r = 0;
		/* Scale it only if I have a voltdm mapped up for the dep */
		if (tvoltdm) {
			struct omap_vdd_dvfs_info *tdvfs_info;
			struct device *target_dev;
			tdvfs_info = _voltdm_to_dvfs_info(tvoltdm);
			if (!tdvfs_info) {
				dev_warn(req_dev, "%s: no dvfs_info\n",
						__func__);
				goto next;
			}
			target_dev = _dvfs_info_to_dev(tdvfs_info);
			if (!target_dev) {
				dev_warn(req_dev, "%s: no target_dev\n",
						__func__);
				goto next;
			}
			r = _dvfs_scale(req_dev, target_dev, tdvfs_info);
next:
			if (r)
				dev_err(req_dev, "%s: dvfs_scale to %s =%d\n",
					__func__, dev_name(target_dev), r);
		}
		/* Store last failed value */
		ret = (r) ? r : ret;
		dep_info++;
	}

	return ret;
}

/**
 * _dvfs_scale() : Scale the devices associated with a voltage domain
 * @req_dev:	Device requesting the scale
 * @target_dev:	Device requesting to be scaled
 * @tdvfs_info:	omap_vdd_dvfs_info pointer for the target domain
 *
 * This runs through the list of devices associated with the
 * voltage domain and scales the device rates to the one requested
 * by the user or those corresponding to the new voltage of the
 * voltage domain. Target voltage is the highest voltage in the vdd_user_list.
 *
 * Returns 0 on success else the error value.
 */
static int _dvfs_scale(struct device *req_dev, struct device *target_dev,
		struct omap_vdd_dvfs_info *tdvfs_info)
{
	unsigned long curr_volt, new_volt;
	int volt_scale_dir = DVFS_VOLT_SCALE_DOWN;
	struct omap_vdd_dev_list *temp_dev;
	struct plist_node *node;
	int ret = 0;
	struct voltagedomain *voltdm;
	struct omap_vdd_info *vdd;
	struct omap_volt_data *new_vdata;
	struct omap_volt_data *curr_vdata;

	voltdm = tdvfs_info->voltdm;
	if (IS_ERR_OR_NULL(voltdm)) {
		dev_err(target_dev, "%s: bad voltdm\n", __func__);
		return -EINVAL;
	}
	vdd = voltdm->vdd;

	/* Find the highest voltage being requested */
	node = plist_last(&tdvfs_info->vdd_user_list);
	new_volt = node->prio;

	new_vdata = omap_voltage_get_voltdata(voltdm, new_volt);
	if (IS_ERR_OR_NULL(new_vdata)) {
		pr_err("%s:%s: Bad New voltage data for %ld\n",
			__func__, voltdm->name, new_volt);
		return PTR_ERR(new_vdata);
	}
	new_volt = omap_get_operation_voltage(new_vdata);
	curr_vdata = omap_voltage_get_curr_vdata(voltdm);
	if (IS_ERR_OR_NULL(curr_vdata)) {
		pr_err("%s:%s: Bad Current voltage data\n",
			__func__, voltdm->name);
		return PTR_ERR(curr_vdata);
	}

	/* Disable smartreflex module across voltage and frequency scaling */
	omap_sr_disable(voltdm);

	/* Pick up the current voltage ONLY after ensuring no changes occur */
	curr_volt = omap_vp_get_curr_volt(voltdm);
	if (!curr_volt)
		curr_volt = omap_get_operation_voltage(curr_vdata);

	/* Make a decision to scale dependent domain based on nominal voltage */
	if (omap_get_nominal_voltage(new_vdata) >
			omap_get_nominal_voltage(curr_vdata)) {
		ret = _dep_scale_domains(target_dev, vdd);
		if (ret) {
			dev_err(target_dev,
				"%s: Error(%d)scale dependent with %ld volt\n",
				__func__, ret, new_volt);
			goto fail;
		}
	}

	if (voltdm->abb && omap_get_nominal_voltage(new_vdata) >
			omap_get_nominal_voltage(curr_vdata)) {
		ret = omap_ldo_abb_pre_scale(voltdm, new_vdata);
		if (ret) {
			pr_err("%s: ABB prescale failed for vdd%s: %d\n",
			__func__, voltdm->name, ret);
			goto fail;
		}
	}

	/* Now decide on switching OPP */
	if (curr_volt == new_volt) {
		volt_scale_dir = DVFS_VOLT_SCALE_NONE;
	} else if (curr_volt < new_volt) {
		ret = voltdm_scale(voltdm, new_vdata);
		if (ret) {
			dev_err(target_dev,
				"%s: Unable to scale the %s to %ld volt\n",
				__func__, voltdm->name, new_volt);
			goto fail;
		}
		volt_scale_dir = DVFS_VOLT_SCALE_UP;
	}

	if (voltdm->abb && omap_get_nominal_voltage(new_vdata) >
			omap_get_nominal_voltage(curr_vdata)) {
		ret = omap_ldo_abb_post_scale(voltdm, new_vdata);
		if (ret) {
			pr_err("%s: ABB prescale failed for vdd%s: %d\n",
			__func__, voltdm->name, ret);
			goto fail;
		}
	}

	/* Move all devices in list to the required frequencies */
	list_for_each_entry(temp_dev, &tdvfs_info->dev_list, node) {
		struct device *dev;
		struct opp *opp;
		unsigned long freq = 0;
		int r;

		dev = temp_dev->dev;
		if (!plist_head_empty(&temp_dev->freq_user_list)) {
			node = plist_last(&temp_dev->freq_user_list);
			freq = node->prio;
		} else {
			/*
			 * Is the dev of dep domain target_device?
			 * we'd probably have a voltage request without
			 * a frequency dependency, scale appropriate frequency
			 * if there are none pending
			 */
			if (target_dev == dev) {
				rcu_read_lock();
				opp = _volt_to_opp(dev, new_volt);
				if (!IS_ERR(opp))
					freq = opp_get_freq(opp);
				rcu_read_unlock();
			}
			if (!freq)
				continue;
		}

		if (freq == clk_get_rate(temp_dev->clk)) {
			dev_dbg(dev, "%s: Already at the requested"
				"rate %ld\n", __func__, freq);
			continue;
		}

		r = clk_set_rate(temp_dev->clk, freq);
		if (r < 0) {
			dev_err(dev, "%s: clk set rate frq=%ld failed(%d)\n",
				__func__, freq, r);
			ret = r;
		}
	}

	if (ret)
		goto fail;

	if (voltdm->abb && omap_get_nominal_voltage(new_vdata) <
			omap_get_nominal_voltage(curr_vdata)) {
		ret = omap_ldo_abb_pre_scale(voltdm, new_vdata);
		if (ret) {
			pr_err("%s: ABB prescale failed for vdd%s: %d\n",
			__func__, voltdm->name, ret);
			goto fail;
		}
	}

	if (DVFS_VOLT_SCALE_DOWN == volt_scale_dir)
		voltdm_scale(voltdm, new_vdata);

	if (voltdm->abb && omap_get_nominal_voltage(new_vdata) <
			omap_get_nominal_voltage(curr_vdata)) {
		ret = omap_ldo_abb_post_scale(voltdm, new_vdata);
		if (ret)
			pr_err("%s: ABB postscale failed for vdd%s: %d\n",
			__func__, voltdm->name, ret);
	}

	/* Make a decision to scale dependent domain based on nominal voltage */
	if (omap_get_nominal_voltage(new_vdata) <
			omap_get_nominal_voltage(curr_vdata)) {
		_dep_scale_domains(target_dev, vdd);
	}

	/* Ensure that current voltage data pointer points to new volt */
	if (curr_volt == new_volt && omap_get_nominal_voltage(new_vdata) !=
			omap_get_nominal_voltage(curr_vdata)) {
		voltdm->curr_volt = new_vdata;
		omap_vp_update_errorgain(voltdm, new_vdata);
	}

	/* All clear.. go out gracefully */
	goto out;

fail:
	pr_warning("%s: domain%s: No clean recovery available! could be bad!\n",
			__func__, voltdm->name);
out:
	/* Re-enable Smartreflex module */
	omap_sr_enable(voltdm, new_vdata);

	return ret;
}

/* Public functions */

/**
 * omap_device_scale() - Set a new rate at which the device is to operate
 * @req_dev:	pointer to the device requesting the scaling.
 * @target_dev:	pointer to the device that is to be scaled
 * @rate:	the rnew rate for the device.
 *
 * This API gets the device opp table associated with this device and
 * tries putting the device to the requested rate and the voltage domain
 * associated with the device to the voltage corresponding to the
 * requested rate. Since multiple devices can be assocciated with a
 * voltage domain this API finds out the possible voltage the
 * voltage domain can enter and then decides on the final device
 * rate.
 *
 * Return 0 on success else the error value
 */
int omap_device_scale(struct device *req_dev, struct device *target_dev,
			unsigned long rate)
{
	struct opp *opp;
	unsigned long volt, freq = rate, new_freq = 0;
	struct omap_vdd_dvfs_info *tdvfs_info;
	struct platform_device *pdev;
	struct omap_device *od;
	struct device *dev;
	int ret = 0;

	pdev = container_of(target_dev, struct platform_device, dev);
	if (IS_ERR_OR_NULL(pdev)) {
		pr_err("%s: pdev is null!\n", __func__);
		return -EINVAL;
	}

	od = container_of(pdev, struct omap_device, pdev);
	if (IS_ERR_OR_NULL(od)) {
		pr_err("%s: od is null!\n", __func__);
		return -EINVAL;
	}

	if (!omap_pm_is_ready()) {
		dev_dbg(target_dev, "%s: pm is not ready yet\n", __func__);
		return -EBUSY;
	}

	/* Lock me to ensure cross domain scaling is secure */
	mutex_lock(&omap_dvfs_lock);

	rcu_read_lock();
	opp = opp_find_freq_ceil(target_dev, &freq);
	/* If we dont find a max, try a floor at least */
	if (IS_ERR(opp))
		opp = opp_find_freq_floor(target_dev, &freq);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(target_dev, "%s: Unable to find OPP for freq%ld\n",
			__func__, rate);
		ret = -ENODEV;
		goto out;
	}
	volt = opp_get_voltage(opp);
	rcu_read_unlock();

	tdvfs_info = _dev_to_dvfs_info(target_dev);
	if (IS_ERR_OR_NULL(tdvfs_info)) {
		dev_err(target_dev, "%s: (req=%s) no vdd![f=%ld, v=%ld]\n",
			__func__, dev_name(req_dev), freq, volt);
		ret = -ENODEV;
		goto out;
	}

	ret = _add_freq_request(tdvfs_info, req_dev, target_dev, freq);
	if (ret) {
		dev_err(target_dev, "%s: freqadd(%s) failed %d[f=%ld, v=%ld]\n",
			__func__, dev_name(req_dev), ret, freq, volt);
		goto out;
	}

	ret = _add_vdd_user(tdvfs_info, req_dev, volt);
	if (ret) {
		dev_err(target_dev, "%s: vddadd(%s) failed %d[f=%ld, v=%ld]\n",
			__func__, dev_name(req_dev), ret, freq, volt);
		_remove_freq_request(tdvfs_info, req_dev,
			target_dev);
		goto out;
	}

	/* Check for any dep domains and add the user request */
	ret = _dep_scan_domains(target_dev, tdvfs_info->voltdm->vdd, volt);
	if (ret) {
		dev_err(target_dev,
			"%s: Error in scan domains for vdd_%s\n",
			__func__, tdvfs_info->voltdm->name);
		goto out;
	}

	dev = _dvfs_info_to_dev(tdvfs_info);
	if (!dev) {
		dev_warn(dev, "%s: no target_dev\n",
			__func__);
		ret = -ENODEV;
		goto out;
	}

	if (dev != target_dev) {
		rcu_read_lock();
		opp = _volt_to_opp(dev, volt);
		if (!IS_ERR(opp))
			new_freq = opp_get_freq(opp);
		rcu_read_unlock();
		if (new_freq) {
			ret = _add_freq_request(tdvfs_info, req_dev, dev,
						new_freq);
			if (ret) {
				dev_err(target_dev, "%s: freqadd(%s) failed %d"
					"[f=%ld, v=%ld]\n", __func__,
					dev_name(req_dev), ret, freq, volt);
				goto out;
			}
		}
	}

	/* Do the actual scaling */
	ret = _dvfs_scale(req_dev, target_dev, tdvfs_info);
	if (ret) {
		dev_err(target_dev, "%s: scale by %s failed %d[f=%ld, v=%ld]\n",
			__func__, dev_name(req_dev), ret, freq, volt);
		_remove_freq_request(tdvfs_info, req_dev,
			target_dev);
		_remove_vdd_user(tdvfs_info, target_dev);
		/* Fall through */
	}
	/* Fall through */
out:
	mutex_unlock(&omap_dvfs_lock);
	return ret;
}
EXPORT_SYMBOL(omap_device_scale);

#ifdef CONFIG_PM_DEBUG
static int dvfs_dump_vdd(struct seq_file *sf, void *unused)
{
	int k;
	struct omap_vdd_dvfs_info *dvfs_info;
	struct omap_vdd_dev_list *tdev;
	struct omap_dev_user_list *duser;
	struct omap_vdd_user_list *vuser;
	struct omap_vdd_info *vdd;
	struct omap_vdd_dep_info *dep_info;
	struct voltagedomain *voltdm;
	struct omap_volt_data *volt_data;
	int anyreq;
	int anyreq2;

	dvfs_info = (struct omap_vdd_dvfs_info *)sf->private;
	if (IS_ERR_OR_NULL(dvfs_info)) {
		pr_err("%s: NO DVFS?\n", __func__);
		return -EINVAL;
	}

	voltdm = dvfs_info->voltdm;
	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: NO voltdm?\n", __func__);
		return -EINVAL;
	}

	vdd = voltdm->vdd;
	if (IS_ERR_OR_NULL(vdd)) {
		pr_err("%s: NO vdd data?\n", __func__);
		return -EINVAL;
	}

	seq_printf(sf, "vdd_%s\n", voltdm->name);
	mutex_lock(&omap_dvfs_lock);
	spin_lock(&dvfs_info->user_lock);

	seq_printf(sf, "|- voltage requests\n|  |\n");
	anyreq = 0;
	plist_for_each_entry(vuser, &dvfs_info->vdd_user_list, node) {
		seq_printf(sf, "|  |-%d: %s:%s\n",
			   vuser->node.prio,
			   dev_driver_string(vuser->dev), dev_name(vuser->dev));
		anyreq = 1;
	}

	spin_unlock(&dvfs_info->user_lock);

	if (!anyreq)
		seq_printf(sf, "|  `-none\n");
	else
		seq_printf(sf, "|  X\n");
	seq_printf(sf, "|\n");

	seq_printf(sf, "|- frequency requests\n|  |\n");
	anyreq2 = 0;
	list_for_each_entry(tdev, &dvfs_info->dev_list, node) {
		anyreq = 0;
		seq_printf(sf, "|  |- %s:%s\n",
			   dev_driver_string(tdev->dev), dev_name(tdev->dev));
		spin_lock(&tdev->user_lock);
		plist_for_each_entry(duser, &tdev->freq_user_list, node) {
			seq_printf(sf, "|  |  |-%d: %s:%s\n",
				   duser->node.prio,
				   dev_driver_string(duser->dev),
				   dev_name(duser->dev));
			anyreq = 1;
		}

		spin_unlock(&tdev->user_lock);

		if (!anyreq)
			seq_printf(sf, "|  |  `-none\n");
		else
			seq_printf(sf, "|  |  X\n");
		anyreq2 = 1;
	}
	if (!anyreq2)
		seq_printf(sf, "|  `-none\n");
	else
		seq_printf(sf, "|  X\n");

	volt_data = vdd->volt_data;
	seq_printf(sf, "|- Supported voltages\n|  |\n");
	anyreq = 0;
	while (volt_data && volt_data->volt_nominal) {
		seq_printf(sf, "|  |-%d\n", volt_data->volt_nominal);
		anyreq = 1;
		volt_data++;
	}
	if (!anyreq)
		seq_printf(sf, "|  `-none\n");
	else
		seq_printf(sf, "|  X\n");

	dep_info = vdd->dep_vdd_info;
	seq_printf(sf, "`- voltage dependencies\n   |\n");
	anyreq = 0;
	while (dep_info && dep_info->nr_dep_entries) {
		struct omap_vdd_dep_volt *dep_table = dep_info->dep_table;

		seq_printf(sf, "   |-on vdd_%s\n", dep_info->name);

		for (k = 0; k < dep_info->nr_dep_entries; k++) {
			seq_printf(sf, "   |  |- %d => %d\n",
				   dep_table[k].main_vdd_volt,
				   dep_table[k].dep_vdd_volt);
		}

		anyreq = 1;
		dep_info++;
	}

	if (!anyreq)
		seq_printf(sf, "   `- none\n");
	else
		seq_printf(sf, "   X  X\n");

	mutex_unlock(&omap_dvfs_lock);
	return 0;
}

static int dvfs_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, dvfs_dump_vdd, inode->i_private);
}

static struct file_operations debugdvfs_fops = {
	.open = dvfs_dbg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry __initdata *dvfsdebugfs_dir;

static void __init dvfs_dbg_init(struct omap_vdd_dvfs_info *dvfs_info)
{
	struct dentry *ddir;

	/* create a base dir */
	if (!dvfsdebugfs_dir)
		dvfsdebugfs_dir = debugfs_create_dir("dvfs", NULL);
	if (IS_ERR_OR_NULL(dvfsdebugfs_dir)) {
		WARN_ONCE("%s: Unable to create base DVFS dir\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(dvfs_info->voltdm)) {
		pr_err("%s: no voltdm\n", __func__);
		return;
	}

	ddir = debugfs_create_dir(dvfs_info->voltdm->name, dvfsdebugfs_dir);
	if (IS_ERR_OR_NULL(ddir)) {
		pr_warning("%s: unable to create subdir %s\n", __func__,
			   dvfs_info->voltdm->name);
		return;
	}

	debugfs_create_file("info", S_IRUGO, ddir,
			    (void *)dvfs_info, &debugdvfs_fops);
}
#else				/* CONFIG_PM_DEBUG */
static inline void dvfs_dbg_init(struct omap_vdd_dvfs_info *dvfs_info)
{
	return;
}
#endif				/* CONFIG_PM_DEBUG */

/**
 * omap_dvfs_register_device - Add a parent device into dvfs managed list
 * @dev:		Device to be added
 * @voltdm_name:	Name of the voltage domain for the device
 * @clk_name:		Name of the clock for the device
 *
 * This function adds a given device into user_list of corresponding
 * vdd's omap_vdd_dvfs_info strucure. This list is traversed to scale
 * frequencies of all the devices on a given vdd.
 *
 * Returns 0 on success.
 */
int __init omap_dvfs_register_device(struct device *dev, char *voltdm_name,
		char *clk_name)
{
	struct omap_vdd_dev_list *temp_dev;
	struct omap_vdd_dvfs_info *dvfs_info;
	struct clk *clk = NULL;
	struct voltagedomain *voltdm;
	int ret = 0;

	if (!voltdm_name) {
		dev_err(dev, "%s: Bad voltdm name!\n", __func__);
		return -EINVAL;
	}
	if (!clk_name) {
		dev_err(dev, "%s: Bad clk name!\n", __func__);
		return -EINVAL;
	}

	/* Lock me to secure structure changes */
	mutex_lock(&omap_dvfs_lock);

	voltdm = voltdm_lookup(voltdm_name);
	if (!voltdm) {
		dev_warn(dev, "%s: unable to find voltdm %s!\n",
			__func__, voltdm_name);
		ret = -EINVAL;
		goto out;
	}
	dvfs_info = _voltdm_to_dvfs_info(voltdm);
	if (!dvfs_info) {
		dvfs_info = kzalloc(sizeof(struct omap_vdd_dvfs_info),
				GFP_KERNEL);
		if (!dvfs_info) {
			dev_warn(dev, "%s: unable to alloc memory!\n",
				__func__);
			ret = -ENOMEM;
			goto out;
		}
		dvfs_info->voltdm = voltdm;

		/* Init the plist */
		spin_lock_init(&dvfs_info->user_lock);
		plist_head_init(&dvfs_info->vdd_user_list);
		/* Init the device list */
		INIT_LIST_HEAD(&dvfs_info->dev_list);

		list_add(&dvfs_info->node, &omap_dvfs_info_list);

		dvfs_dbg_init(dvfs_info);
	}

	/* If device already added, we dont need to do more.. */
	list_for_each_entry(temp_dev, &dvfs_info->dev_list, node) {
		if (temp_dev->dev == dev)
			goto out;
	}

	temp_dev = kzalloc(sizeof(struct omap_vdd_dev_list), GFP_KERNEL);
	if (!temp_dev) {
		dev_err(dev, "%s: Unable to creat a new device for vdd_%s\n",
			__func__, dvfs_info->voltdm->name);
		ret = -ENOMEM;
		goto out;
	}

	clk = clk_get(dev, clk_name);
	if (IS_ERR_OR_NULL(clk)) {
		dev_warn(dev, "%s: Bad clk pointer!\n", __func__);
		kfree(temp_dev);
		ret = -EINVAL;
		goto out;
	}

	/* Initialize priority ordered list */
	spin_lock_init(&temp_dev->user_lock);
	plist_head_init(&temp_dev->freq_user_list);

	temp_dev->dev = dev;
	temp_dev->clk = clk;
	list_add_tail(&temp_dev->node, &dvfs_info->dev_list);

	/* Fall through */
out:
	mutex_unlock(&omap_dvfs_lock);
	return ret;
}
