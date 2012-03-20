/*
 * OMAP3/4 LDO users core
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Mike Turquette <mturquette@ti.com>
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <plat/cpu.h>
#include "voltage.h"
#include "ldo.h"

/**
 * _is_abb_enabled() - check if abb is enabled
 * @voltdm:	voltage domain to check for
 * @abb:	abb instance pointer
 *
 * Returns true if enabled, else returns false
 */
static inline bool _is_abb_enabled(struct voltagedomain *voltdm,
				   struct omap_ldo_abb_instance *abb)
{
	return (voltdm->read(abb->setup_reg) & abb->setup_bits->enable_mask) ?
	    true : false;
}

/**
 * _abb_set_availability() - sets the availability of the ABB LDO
 * @voltdm:	voltage domain for which we would like to set
 * @abb:	abb instance pointer
 * @available:	should I enable/disable the LDO?
 *
 * Depending on the request, it enables/disables the LDO if it was not
 * in that state already.
 */
static inline void _abb_set_availability(struct voltagedomain *voltdm,
					 struct omap_ldo_abb_instance *abb,
					 bool available)
{
	if (_is_abb_enabled(voltdm, abb) == available)
		return;

	voltdm->rmw(abb->setup_bits->enable_mask,
		    (available) ? abb->setup_bits->enable_mask : 0,
		    abb->setup_reg);
}

/**
 * _abb_wait_tranx() - wait for abb tranxdone event
 * @voltdm:	voltage domain we are operating on
 * @abb:	pointer to the abb instance
 *
 * Returns -ETIMEDOUT if the event is not set on time.
 */
static int _abb_wait_tranx(struct voltagedomain *voltdm,
			    struct omap_ldo_abb_instance *abb)
{
	int timeout;
	int ret;

	timeout = 0;
	while (timeout++ < abb->tranx_timeout) {
		ret = abb->ops->check_txdone(abb->prm_irq_id);
		if (ret)
			break;

		udelay(1);
	}

	if (timeout >= abb->tranx_timeout) {
		pr_warning("%s:%s: ABB TRANXDONE waittimeout(timeout=%d)\n",
			   __func__, voltdm->name, timeout);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * _abb_clear_tranx() - clear abb tranxdone event
 * @voltdm:	voltage domain we are operating on
 * @abb:	pointer to the abb instance
 *
 * Returns -ETIMEDOUT if the event is not cleared on time.
 */
static int _abb_clear_tranx(struct voltagedomain *voltdm,
			    struct omap_ldo_abb_instance *abb)
{
	int timeout;
	int ret;

	/* clear interrupt status */
	timeout = 0;
	while (timeout++ < abb->tranx_timeout) {
		abb->ops->clear_txdone(abb->prm_irq_id);

		ret = abb->ops->check_txdone(abb->prm_irq_id);
		if (!ret)
			break;

		udelay(1);
	}

	if (timeout >= abb->tranx_timeout) {
		pr_warning("%s:%s: ABB TRANXDONE timeout(timeout=%d)\n",
			   __func__, voltdm->name, timeout);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * _abb_set_abb() - helper to actually set ABB (NOMINAL/FAST)
 * @voltdm:	voltage domain we are operating on
 * @abb_type:	ABB type we want to set
 */
static int _abb_set_abb(struct voltagedomain *voltdm, int abb_type)
{
	struct omap_ldo_abb_instance *abb = voltdm->abb;
	int ret;

	ret = _abb_clear_tranx(voltdm, abb);
	if (ret)
		return ret;

	/* program next state of ABB ldo */
	voltdm->rmw(abb->ctrl_bits->opp_sel_mask,
		    abb_type << __ffs(abb->ctrl_bits->opp_sel_mask),
		    abb->ctrl_reg);

	/* initiate ABB ldo change */
	voltdm->rmw(abb->ctrl_bits->opp_change_mask,
		    abb->ctrl_bits->opp_change_mask, abb->ctrl_reg);

	/* Wait for conversion completion */
	ret = _abb_wait_tranx(voltdm, abb);
	WARN_ONCE(ret, "%s: voltdm %s ABB TRANXDONE was not set on time:%d\n",
			__func__, voltdm->name, ret);
	/* clear interrupt status */
	ret |= _abb_clear_tranx(voltdm, abb);

	return ret;
}

/**
 * _abb_scale() - wrapper which does the necessary things for pre and post scale
 * @voltdm:		voltage domain to operate on
 * @target_volt:	voltage we are going to
 * @is_prescale:	are we doing a prescale operation?
 *
 * NOTE: We expect caller ensures that a specific voltdm is modified
 * sequentially. All locking is expected to be implemented by users
 * of LDO functions
 */
static int _abb_scale(struct voltagedomain *voltdm,
		      struct omap_volt_data *target_vdata, bool is_prescale)
{
	int ret = 0;
	int curr_abb, target_abb;
	struct omap_ldo_abb_instance *abb;

	if (IS_ERR_OR_NULL(target_vdata)) {
		pr_err("%s:%s: Invalid volt data tv=%p!\n", __func__,
		       voltdm->name, target_vdata);
		return -EINVAL;
	}

	abb = voltdm->abb;
	if (IS_ERR_OR_NULL(abb)) {
		WARN(1, "%s:%s: no abb structure!\n", __func__, voltdm->name);
		return -EINVAL;
	}

	curr_abb = abb->__cur_abb_type;
	target_abb = target_vdata->abb_type;

	pr_debug("%s: %s: Enter: t_v=%ld scale=%d c_abb=%d t_abb=%d ret=%d\n",
		 __func__, voltdm->name, omap_get_nominal_voltage(target_vdata),
		 is_prescale, curr_abb, target_abb, ret);

	/* If we were'nt booting and there is no change, we get out */
	if (target_abb == curr_abb && voltdm->curr_volt)
		goto out;

	/* Do we have an invalid ABB entry? scream for a fix! */
	if (curr_abb == OMAP_ABB_NONE || target_abb == OMAP_ABB_NONE) {
		WARN(1, "%s:%s: INVALID abb entries? curr=%d target=%d\n",
		     __func__, voltdm->name, curr_abb, target_abb);
		return -EINVAL;
	}

	/*
	 * We set up ABB as follows:
	 * if we are scaling *to* a voltage which needs ABB, do it in post
	 * if we are scaling *from* a voltage which needs ABB, do it in pre
	 * So, if the conditions are in reverse, we just return happy
	 */
	if (is_prescale && (target_abb > curr_abb))
		goto out;

	if (!is_prescale && (target_abb < curr_abb))
		goto out;

	/* Time to set ABB now */
	ret = _abb_set_abb(voltdm, target_abb);
	if (!ret) {
		abb->__cur_abb_type = target_abb;
		pr_debug("%s: %s:  scaled - t_abb=%d!\n", __func__,
			 voltdm->name, target_abb);
	} else {
		pr_warning("%s: %s:  failed scale: t_abb=%d (%d)!\n", __func__,
			   voltdm->name, target_abb, ret);
	}

out:
	pr_debug("%s: %s:Exit: t_v=%ld scale=%d c_abb=%d t_abb=%d ret=%d\n",
		 __func__, voltdm->name, omap_get_nominal_voltage(target_vdata),
		 is_prescale, curr_abb, target_abb, ret);
	return ret;

}

/**
 * omap_ldo_abb_pre_scale() - Enable required ABB strategy before voltage scale
 * @voltdm:		voltage domain to operate on
 * @target_volt:	target voltage data we moved to.
 */
int omap_ldo_abb_pre_scale(struct voltagedomain *voltdm,
			   struct omap_volt_data *target_vdata)
{
	return _abb_scale(voltdm, target_vdata, true);
}

/**
 * omap_ldo_abb_pre_scale() - Enable required ABB strategy after voltage scale
 * @voltdm:		voltage domain operated on
 * @target_volt:	target voltage we are going to
 */
int omap_ldo_abb_post_scale(struct voltagedomain *voltdm,
			    struct omap_volt_data *target_vdata)
{
	return _abb_scale(voltdm, target_vdata, false);
}

/**
 * omap_ldo_abb_init() - initialize the ABB LDO for associated for this domain
 * @voltdm:	voltdm for which we need to initialize the ABB LDO
 *
 * Programs up the the configurations that dont change in the domain
 *
 * Return 0 if all goes fine, else returns appropriate error value
 */
void __init omap_ldo_abb_init(struct voltagedomain *voltdm)
{
	u32 sys_clk_rate;
	u32 cycle_rate;
	u32 settling_time;
	u32 wait_count_val;
	struct omap_ldo_abb_instance *abb;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: No voltdm?\n", __func__);
		return;
	}
	if (!voltdm->read || !voltdm->write || !voltdm->rmw) {
		pr_err("%s: No read/write/rmw API for accessing vdd_%s regs\n",
		       __func__, voltdm->name);
		return;
	}

	abb = voltdm->abb;
	if (IS_ERR_OR_NULL(abb))
		return;
	if (IS_ERR_OR_NULL(abb->ctrl_bits) || IS_ERR_OR_NULL(abb->setup_bits)) {
		pr_err("%s: Corrupted ABB configuration on vdd_%s regs\n",
		       __func__, voltdm->name);
		return;
	}

	/*
	 * SR2_WTCNT_VALUE must be programmed with the expected settling time
	 * for ABB ldo transition.  This value depends on the cycle rate for
	 * the ABB IP (varies per OMAP family), and the system clock frequency
	 * (varies per board).  The formula is:
	 *
	 * SR2_WTCNT_VALUE = SettlingTime / (CycleRate / SystemClkRate))
	 * where SettlingTime is in micro-seconds and SystemClkRate is in MHz.
	 *
	 * To avoid dividing by zero multiply both CycleRate and SettlingTime
	 * by 10 such that the final result is the one we want.
	 */

	/* Convert SYS_CLK rate to MHz & prevent divide by zero */
	sys_clk_rate = DIV_ROUND_CLOSEST(voltdm->sys_clk.rate, 1000000);
	cycle_rate = abb->cycle_rate * 10;
	settling_time = abb->settling_time * 10;

	/* Calculate cycle rate */
	cycle_rate = DIV_ROUND_CLOSEST(cycle_rate, sys_clk_rate);

	/* Calulate SR2_WTCNT_VALUE */
	wait_count_val = DIV_ROUND_CLOSEST(settling_time, cycle_rate);

	voltdm->rmw(abb->setup_bits->wait_count_mask,
		    wait_count_val << __ffs(abb->setup_bits->wait_count_mask),
		    abb->setup_reg);

	/* Allow Forward Body-Bias */
	voltdm->rmw(abb->setup_bits->active_fbb_mask,
		    abb->setup_bits->active_fbb_mask, abb->setup_reg);

	/* Enable ABB */
	_abb_set_availability(voltdm, abb, true);

	/*
	 * Beware of the bootloader!
	 * Initialize current abb type based on what we read off the reg.
	 * we cant trust the initial state based off boot voltage's volt_data
	 * even. Not all bootloaders are nice :(
	 */
	abb->__cur_abb_type = (voltdm->read(abb->ctrl_reg) &
			       abb->ctrl_bits->opp_sel_mask) >>
				    __ffs(abb->ctrl_bits->opp_sel_mask);

	return;
}
