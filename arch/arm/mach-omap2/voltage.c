/*
 * OMAP3/OMAP4 Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 *
 * Copyright (C) 2008, 2011 Nokia Corporation
 * Kalle Jokiniemi
 * Paul Walmsley
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include <plat/common.h>

#include "prm-regbits-34xx.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "prcm44xx.h"
#include "prminst44xx.h"
#include "control.h"

#include "voltage.h"
#include "powerdomain.h"

#include "vc.h"
#include "vp.h"

static LIST_HEAD(voltdm_list);

static int __init _config_common_vdd_data(struct voltagedomain *voltdm)
{
	struct clk *sys_ck;

	/*
	 * Sys clk rate is require to calculate vp timeout value and
	 * smpswaittimemin and smpswaittimemax.
	 */
	sys_ck = clk_get(NULL, voltdm->sys_clk.name);
	if (IS_ERR(sys_ck)) {
		pr_warning("%s: Could not get the sys clk to calculate"
			"various vdd_%s params\n", __func__, voltdm->name);
		return -EINVAL;
	}
	voltdm->sys_clk.rate = clk_get_rate(sys_ck);
	WARN_ON(!voltdm->sys_clk.rate);

	/* Generic voltage parameters */
	voltdm->scale = omap_vp_forceupdate_scale;

	return 0;
}

static int __init omap_vdd_data_configure(struct voltagedomain *voltdm)
{
	int ret = -EINVAL;

	if (!voltdm->pmic) {
		pr_err("%s: PMIC info requried to configure vdd_%s not"
			"populated.Hence cannot initialize vdd_%s\n",
			__func__, voltdm->name, voltdm->name);
		goto ovdc_out;
	}

	if (IS_ERR_VALUE(_config_common_vdd_data(voltdm)))
		goto ovdc_out;

	ret = 0;

ovdc_out:
	return ret;
}

/* Public functions */
/**
 * omap_voltage_get_curr_vdata() - Gets the current voltage data
 * @voltdm:	pointer to the VDD for which current voltage info is needed
 *
 * API to get the current voltage data pointer for a VDD, returns NULL on error
 */
struct omap_volt_data *omap_voltage_get_curr_vdata(struct voltagedomain *voltdm)
{
	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return NULL;
	}

	return voltdm->curr_volt;
}

/**
 * voltdm_scale() - API to scale voltage of a particular voltage domain.
 * @voltdm: pointer to the voltage domain which is to be scaled.
 * @target_volt: The target voltage of the voltage domain
 *
 * This API should be called by the kernel to do the voltage scaling
 * for a particular voltage domain during DVFS.
 */
int voltdm_scale(struct voltagedomain *voltdm,
		 struct omap_volt_data *target_v)
{
	int ret = 0;
	struct omap_voltage_notifier notify;
	unsigned long target_volt = omap_get_operation_voltage(target_v);

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	if (!voltdm->scale) {
		pr_err("%s: No voltage scale API registered for vdd_%s\n",
			__func__, voltdm->name);
		return -ENODATA;
	}

	notify.voltdm = voltdm;
	notify.target_volt = target_volt;

	srcu_notifier_call_chain(&voltdm->change_notify_list,
			OMAP_VOLTAGE_PRECHANGE,
			(void *)&notify);

	ret = voltdm->scale(voltdm, target_v);
	if (ret)
		pr_err("%s: voltage scale failed for vdd%s: %d\n",
			__func__, voltdm->name, ret);

	notify.op_result = ret;
	srcu_notifier_call_chain(&voltdm->change_notify_list,
			OMAP_VOLTAGE_POSTCHANGE,
			(void *)&notify);

	return ret;
}

/**
 * voltdm_reset() - Resets the voltage of a particular voltage domain
 * 	            to that of the current OPP.
 * @voltdm: pointer to the voltage domain whose voltage is to be reset.
 *
 * This API finds out the correct voltage the voltage domain is supposed
 * to be at and resets the voltage to that level. Should be used especially
 * while disabling any voltage compensation modules.
 *
 * NOTE: appropriate locks should be held for mutual exclusivity.
 */
void voltdm_reset(struct voltagedomain *voltdm)
{
	struct omap_volt_data *target_volt;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	target_volt = omap_voltage_get_curr_vdata(voltdm);
	if (!target_volt) {
		pr_err("%s: unable to find current voltage for vdd_%s\n",
			__func__, voltdm->name);
		return;
	}

	voltdm_scale(voltdm, target_volt);
}

/**
 * omap_voltage_get_volttable() - API to get the voltage table associated with a
 *				particular voltage domain.
 * @voltdm:	pointer to the VDD for which the voltage table is required
 * @volt_data:	the voltage table for the particular vdd which is to be
 *		populated by this API
 *
 * This API populates the voltage table associated with a VDD into the
 * passed parameter pointer. Returns the count of distinct voltages
 * supported by this vdd.
 *
 */
void omap_voltage_get_volttable(struct voltagedomain *voltdm,
		struct omap_volt_data **volt_data)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	vdd = voltdm->vdd;

	*volt_data = vdd->volt_data;
}

/**
 * omap_voltage_get_voltdata() - API to get the voltage table entry for a
 *				particular voltage
 * @voltdm:	pointer to the VDD whose voltage table has to be searched
 * @volt:	the voltage to be searched in the voltage table
 *
 * This API searches through the voltage table for the required voltage
 * domain and tries to find a matching entry for the passed voltage volt.
 * If a matching entry is found volt_data is populated with that entry.
 * This API searches only through the non-compensated voltages int the
 * voltage table.
 * Returns pointer to the voltage table entry corresponding to volt on
 * success. Returns -ENODATA if no voltage table exisits for the passed voltage
 * domain or if there is no matching entry.
 */
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt)
{
	struct omap_vdd_info *vdd;
	int i;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	vdd = voltdm->vdd;

	if (!vdd->volt_data) {
		pr_warning("%s: voltage table does not exist for vdd_%s\n",
			__func__, voltdm->name);
		return ERR_PTR(-ENODATA);
	}

	for (i = 0; vdd->volt_data[i].volt_nominal != 0; i++) {
		if (vdd->volt_data[i].volt_nominal == volt)
			return &vdd->volt_data[i];
	}

	pr_notice("%s: Unable to match the current voltage %lu with the voltage"
		"table for vdd_%s\n", __func__, volt, voltdm->name);

	return ERR_PTR(-ENODATA);
}

/**
 * omap_voltage_register_pmic() - API to register PMIC specific data
 * @voltdm:	pointer to the VDD for which the PMIC specific data is
 *		to be registered
 * @pmic:	the structure containing pmic info
 *
 * This API is to be called by the SOC/PMIC file to specify the
 * pmic specific info as present in omap_voltdm_pmic structure.
 */
int omap_voltage_register_pmic(struct voltagedomain *voltdm,
			       struct omap_voltdm_pmic *pmic)
{
	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	voltdm->pmic = pmic;

	return 0;
}

/**
 * omap_change_voltscale_method() - API to change the voltage scaling method.
 * @voltdm:	pointer to the VDD whose voltage scaling method
 *		has to be changed.
 * @voltscale_method:	the method to be used for voltage scaling.
 *
 * This API can be used by the board files to change the method of voltage
 * scaling between vpforceupdate and vcbypass. The parameter values are
 * defined in voltage.h
 */
void omap_change_voltscale_method(struct voltagedomain *voltdm,
				  int voltscale_method)
{
	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	switch (voltscale_method) {
	case VOLTSCALE_VPFORCEUPDATE:
		voltdm->scale = omap_vp_forceupdate_scale;
		return;
	case VOLTSCALE_VCBYPASS:
		voltdm->scale = omap_vc_bypass_scale_voltage;
		return;
	default:
		pr_warning("%s: Trying to change the method of voltage scaling"
			"to an unsupported one!\n", __func__);
	}
}

/* Voltage debugfs support */
static int vp_volt_debug_get(void *data, u64 *val)
{
	struct voltagedomain *voltdm = (struct voltagedomain *)data;

	if (!voltdm) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}
	*val = omap_vp_get_curr_volt(voltdm);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vp_volt_debug_fops, vp_volt_debug_get, NULL, "%llu\n");

static int dyn_volt_debug_get(void *data, u64 *val)
{
	struct voltagedomain *voltdm = (struct voltagedomain *)data;
	struct omap_volt_data *volt_data;

	if (!voltdm) {
		pr_warning("%s: Wrong paramater passed\n", __func__);
		return -EINVAL;
	}

	volt_data = omap_voltage_get_curr_vdata(voltdm);
	if (IS_ERR_OR_NULL(volt_data)) {
		pr_warning("%s: No voltage/domain?\n", __func__);
		return -ENODEV;
	}

	*val = volt_data->volt_dynamic_nominal;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dyn_volt_debug_fops, dyn_volt_debug_get, NULL,
								"%llu\n");

static int calib_volt_debug_get(void *data, u64 *val)
{
	struct voltagedomain *voltdm = (struct voltagedomain *)data;
	struct omap_volt_data *volt_data;

	if (!voltdm) {
		pr_warning("%s: Wrong paramater passed\n", __func__);
		return -EINVAL;
	}

	volt_data = omap_voltage_get_curr_vdata(voltdm);
	if (IS_ERR_OR_NULL(volt_data)) {
		pr_warning("%s: No voltage/domain?\n", __func__);
		return -ENODEV;
	}

	*val = volt_data->volt_calibrated;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(calib_volt_debug_fops, calib_volt_debug_get, NULL,
								"%llu\n");
static int margin_volt_debug_get(void *data, u64 *val)
{
	struct voltagedomain *voltdm = (struct voltagedomain *) data;
	struct omap_volt_data *vdata;

	if (!voltdm) {
		pr_warning("%s: Wrong parameter passed\n", __func__);
		return -EINVAL;
	}

	vdata = omap_voltage_get_curr_vdata(voltdm);
	if (IS_ERR_OR_NULL(vdata)) {
		pr_warning("%s: unable to get volt for vdd_%s\n",
			   __func__, voltdm->name);
		return -ENODEV;
	}
	*val = vdata->volt_margin;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(margin_volt_debug_fops, margin_volt_debug_get, NULL,
								"%llu\n");

static int nom_volt_debug_get(void *data, u64 *val)
{
	struct voltagedomain *voltdm = (struct voltagedomain *) data;
	struct omap_volt_data *vdata;

	if (!voltdm) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	vdata = omap_voltage_get_curr_vdata(voltdm);
	if (IS_ERR_OR_NULL(vdata)) {
		pr_warning("%s: unable to get volt for vdd_%s\n",
			   __func__, voltdm->name);
		return -ENODEV;
	}
	*val = vdata->volt_nominal;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(nom_volt_debug_fops, nom_volt_debug_get, NULL,
								"%llu\n");

static void __init voltdm_debugfs_init(struct dentry *voltage_dir,
					struct voltagedomain *voltdm)
{
	char *name;

	name = kasprintf(GFP_KERNEL, "vdd_%s", voltdm->name);
	if (!name) {
		pr_warning("%s:vdd_%s: no mem for debugfs\n", __func__,
				voltdm->name);
		return;
	}

	voltdm->debug_dir = debugfs_create_dir(name, voltage_dir);
	kfree(name);
	if (IS_ERR_OR_NULL(voltdm->debug_dir)) {
		pr_warning("%s: Unable to create debugfs directory for"
			" vdd_%s\n", __func__, voltdm->name);
		voltdm->debug_dir = NULL;
		return;
	}

	(void) debugfs_create_file("curr_vp_volt", S_IRUGO, voltdm->debug_dir,
				(void *) voltdm, &vp_volt_debug_fops);
	(void) debugfs_create_file("curr_nominal_volt", S_IRUGO,
				voltdm->debug_dir, (void *) voltdm,
				&nom_volt_debug_fops);
	(void) debugfs_create_file("curr_dyn_nominal_volt", S_IRUGO,
				voltdm->debug_dir, (void *) voltdm,
				&dyn_volt_debug_fops);
	(void) debugfs_create_file("curr_calibrated_volt", S_IRUGO,
				voltdm->debug_dir, (void *) voltdm,
				&calib_volt_debug_fops);
	(void) debugfs_create_file("curr_margin_volt", S_IRUGO,
				voltdm->debug_dir, (void *) voltdm,
				&margin_volt_debug_fops);
}

/**
 * omap_voltage_late_init() - Init the various voltage parameters
 *
 * This API is to be called in the later stages of the
 * system boot to init the voltage controller and
 * voltage processors.
 */
int __init omap_voltage_late_init(void)
{
	struct voltagedomain *voltdm;
	struct dentry *voltage_dir;

	if (list_empty(&voltdm_list)) {
		pr_err("%s: Voltage driver support not added\n",
			__func__);
		return -EINVAL;
	}

	voltage_dir = debugfs_create_dir("voltage", NULL);

	list_for_each_entry(voltdm, &voltdm_list, node) {
		if (!voltdm->scalable)
			continue;

		if (voltdm->vdd) {
			if (omap_vdd_data_configure(voltdm))
				continue;
			omap_vp_init(voltdm);
		}

		if (voltdm->vc)
			omap_vc_init_channel(voltdm);

		if (voltdm->abb)
			omap_ldo_abb_init(voltdm);

		if (voltage_dir)
			voltdm_debugfs_init(voltage_dir, voltdm);

		srcu_init_notifier_head(&voltdm->change_notify_list);
	}

	return 0;
}

static struct voltagedomain *_voltdm_lookup(const char *name)
{
	struct voltagedomain *voltdm, *temp_voltdm;

	voltdm = NULL;

	list_for_each_entry(temp_voltdm, &voltdm_list, node) {
		if (!strcmp(name, temp_voltdm->name)) {
			voltdm = temp_voltdm;
			break;
		}
	}

	return voltdm;
}

/**
 * omap_voltage_calib_reset() - reset the calibrated voltage entries
 * @voltdm: voltage domain to reset the entries for
 *
 * when the calibrated entries are no longer valid, this api allows
 * the calibrated voltages to be reset.
 *
 * NOTE: Appropriate locks must be held by calling path to ensure mutual
 * exclusivity
 */
int omap_voltage_calib_reset(struct voltagedomain *voltdm)
{
	struct omap_volt_data *volt_data;

	if (!voltdm) {
		pr_warning("%s: voltdm NULL!\n", __func__);
		return -EINVAL;
	}

	volt_data = voltdm->vdd->volt_data;

	/* reset the calibrated voltages as 0 */
	while (volt_data->volt_nominal) {
		volt_data->volt_calibrated = 0;
		volt_data++;
	}
	return 0;
}

/**
 * voltdm_add_pwrdm - add a powerdomain to a voltagedomain
 * @voltdm: struct voltagedomain * to add the powerdomain to
 * @pwrdm: struct powerdomain * to associate with a voltagedomain
 *
 * Associate the powerdomain @pwrdm with a voltagedomain @voltdm.  This
 * enables the use of voltdm_for_each_pwrdm().  Returns -EINVAL if
 * presented with invalid pointers; -ENOMEM if memory could not be allocated;
 * or 0 upon success.
 */
int voltdm_add_pwrdm(struct voltagedomain *voltdm, struct powerdomain *pwrdm)
{
	if (!voltdm || !pwrdm)
		return -EINVAL;

	pr_debug("voltagedomain: associating powerdomain %s with voltagedomain "
		 "%s\n", pwrdm->name, voltdm->name);

	list_add(&pwrdm->voltdm_node, &voltdm->pwrdm_list);

	return 0;
}

/**
 * voltdm_for_each_pwrdm - call function for each pwrdm in a voltdm
 * @voltdm: struct voltagedomain * to iterate over
 * @fn: callback function *
 *
 * Call the supplied function @fn for each powerdomain in the
 * voltagedomain @voltdm.  Returns -EINVAL if presented with invalid
 * pointers; or passes along the last return value of the callback
 * function, which should be 0 for success or anything else to
 * indicate failure.
 */
int voltdm_for_each_pwrdm(struct voltagedomain *voltdm,
			  int (*fn)(struct voltagedomain *voltdm,
				    struct powerdomain *pwrdm))
{
	struct powerdomain *pwrdm;
	int ret = 0;

	if (!fn)
		return -EINVAL;

	list_for_each_entry(pwrdm, &voltdm->pwrdm_list, voltdm_node)
		ret = (*fn)(voltdm, pwrdm);

	return ret;
}

/**
 * voltdm_for_each - call function on each registered voltagedomain
 * @fn: callback function *
 *
 * Call the supplied function @fn for each registered voltagedomain.
 * The callback function @fn can return anything but 0 to bail out
 * early from the iterator.  Returns the last return value of the
 * callback function, which should be 0 for success or anything else
 * to indicate failure; or -EINVAL if the function pointer is null.
 */
int voltdm_for_each(int (*fn)(struct voltagedomain *voltdm, void *user),
		    void *user)
{
	struct voltagedomain *temp_voltdm;
	int ret = 0;

	if (!fn)
		return -EINVAL;

	list_for_each_entry(temp_voltdm, &voltdm_list, node) {
		ret = (*fn)(temp_voltdm, user);
		if (ret)
			break;
	}

	return ret;
}

static int _voltdm_register(struct voltagedomain *voltdm)
{
	if (!voltdm || !voltdm->name)
		return -EINVAL;

	INIT_LIST_HEAD(&voltdm->pwrdm_list);
	list_add(&voltdm->node, &voltdm_list);

	pr_debug("voltagedomain: registered %s\n", voltdm->name);

	return 0;
}

/**
 * voltdm_lookup - look up a voltagedomain by name, return a pointer
 * @name: name of voltagedomain
 *
 * Find a registered voltagedomain by its name @name.  Returns a pointer
 * to the struct voltagedomain if found, or NULL otherwise.
 */
struct voltagedomain *voltdm_lookup(const char *name)
{
	struct voltagedomain *voltdm ;

	if (!name)
		return NULL;

	voltdm = _voltdm_lookup(name);

	return voltdm;
}

/**
 * voltdm_init - set up the voltagedomain layer
 * @voltdm_list: array of struct voltagedomain pointers to register
 *
 * Loop through the array of voltagedomains @voltdm_list, registering all
 * that are available on the current CPU. If voltdm_list is supplied
 * and not null, all of the referenced voltagedomains will be
 * registered.  No return value.
 */
void voltdm_init(struct voltagedomain **voltdms)
{
	struct voltagedomain **v;

	if (voltdms) {
		for (v = voltdms; *v; v++)
			_voltdm_register(*v);
	}
}
