/*
 * Smart reflex Class 1.5 specific implementations
 *
 * Copyright (C) 2010-2011 Texas Instruments, Inc.
 * Nishanth Menon <nm@ti.com>
 *
 * Smart reflex class 1.5 is also called periodic SW Calibration
 * Some of the highlights are as follows:
 * – Host CPU triggers OPP calibration when transitioning to non calibrated
 *   OPP
 * – SR-AVS + VP modules are used to perform calibration
 * – Once completed, the SmartReflex-AVS module can be disabled
 * – Enables savings based on process, supply DC accuracy and aging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/opp.h>

#include "smartreflex.h"
#include "voltage.h"
#include "dvfs.h"

#define MAX_VDDS		3
#define SR1P5_SAMPLING_DELAY_MS	1
#define SR1P5_STABLE_SAMPLES	10
#define SR1P5_MAX_TRIGGERS	5

/*
 * We expect events in 10uS, if we don't receive it in twice as long,
 * we stop waiting for the event and use the current value
 */
#define MAX_CHECK_VPTRANS_US	20

/**
 * struct sr_class1p5_work_data - data meant to be used by calibration work
 * @work:	calibration work
 * @voltdm:		voltage domain for which we are triggering
 * @vdata:	voltage data we are calibrating
 * @num_calib_triggers:	number of triggers from calibration loop
 * @num_osc_samples:	number of samples collected by isr
 * @u_volt_samples:	private data for collecting voltage samples in
 *			case oscillations. filled by the notifier and
 *			consumed by the work item.
 * @work_active:	have we scheduled a work item?
 */
struct sr_class1p5_work_data {
	struct delayed_work work;
	struct voltagedomain *voltdm;
	struct omap_volt_data *vdata;
	u8 num_calib_triggers;
	u8 num_osc_samples;
	unsigned long u_volt_samples[SR1P5_STABLE_SAMPLES];
	bool work_active;
};

extern bool enable_highvolt_sr;

#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY
/* recal_work:	recalibration calibration work */
static struct delayed_work recal_work;
#endif

/**
 * sr_class1p5_notify() - isr notifier for status events
 * @voltdm:	voltage domain for which we were triggered
 * @voltdm_cdata: voltage domain specific private class data
 * @status:	notifier event to use
 *
 * This basically collects data for the work to use.
 */
static int sr_class1p5_notify(struct voltagedomain *voltdm,
			      void *voltdm_cdata,
			      u32 status)
{
	struct sr_class1p5_work_data *work_data;
	int idx = 0;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	work_data = (struct sr_class1p5_work_data *)voltdm_cdata;
	if (IS_ERR_OR_NULL(work_data)) {
		pr_err("%s:%s no work data!!\n", __func__, voltdm->name);
		return -EINVAL;
	}

	/* Wait for transdone so that we know the voltage to read */
	do {
		if (omap_vp_is_transdone(voltdm))
			break;
		idx++;
		/* get some constant delay */
		udelay(1);
	} while (idx < MAX_CHECK_VPTRANS_US);

	/*
	 * NOTE:
	 * If we timeout, we still read the data,
	 * if we are oscillating+irq latencies are too high, we could
	 * have scenarios where we miss transdone event. since
	 * we waited long enough, it is still safe to read the voltage
	 * as we would have waited long enough - Dont warn for this.
	 */
	idx = (work_data->num_osc_samples) % SR1P5_STABLE_SAMPLES;
	work_data->u_volt_samples[idx] = omap_vp_get_curr_volt(voltdm);
	work_data->num_osc_samples++;

	omap_vp_clear_transdone(voltdm);


	return 0;
}

/**
 * sr_class1p5_calib_work() - work which actually does the calibration
 * @work: pointer to the work
 *
 * calibration routine uses the following logic:
 * on the first trigger, we start the isr to collect sr voltages
 * wait for stabilization delay (reschdule self instead of sleeping)
 * after the delay, see if we collected any isr events
 * if none, we have calibrated voltage.
 * if there are any, we retry untill we giveup.
 * on retry timeout, select a voltage to use as safe voltage.
 */
static void sr_class1p5_calib_work(struct work_struct *work)
{
	struct sr_class1p5_work_data *work_data =
	    container_of(work, struct sr_class1p5_work_data, work.work);
	unsigned long u_volt_safe = 0, u_volt_current = 0, u_volt_margin;
	struct omap_volt_data *volt_data;
	struct voltagedomain *voltdm;
	int idx = 0;

	if (!work) {
		pr_err("%s: ooops.. null work_data?\n", __func__);
		return;
	}

	/*
	 * Handle the case where we might have just been scheduled AND
	 * 1.5 disable was called.
	 */
	if (!mutex_trylock(&omap_dvfs_lock)) {
		schedule_delayed_work(&work_data->work,
				      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
						       SR1P5_STABLE_SAMPLES));
		return;
	}

	voltdm = work_data->voltdm;
	/*
	 * In the unlikely case that we did get through when unplanned,
	 * flag and return.
	 */
	if (unlikely(!work_data->work_active)) {
		pr_err("%s:%s unplanned work invocation!\n", __func__,
		       voltdm->name);
		mutex_unlock(&omap_dvfs_lock);
		return;
	}

	volt_data = work_data->vdata;

	work_data->num_calib_triggers++;
	/* if we are triggered first time, we need to start isr to sample */
	if (work_data->num_calib_triggers == 1) {
		/* We could be interrupted many times, so, only for debug */
		pr_debug("%s: %s: Calibration start: Voltage Nominal=%d\n",
			 __func__, voltdm->name, volt_data->volt_nominal);
		goto start_sampling;
	}

	/* Stop isr from interrupting our measurements :) */
	sr_notifier_control(voltdm, false);

	/*
	 * Quit sampling
	 * a) if we have oscillations
	 * b) if we have nominal voltage as the voltage
	 */
	if (work_data->num_calib_triggers == SR1P5_MAX_TRIGGERS)
		goto stop_sampling;

	/* if there are no samples captured.. SR is silent, aka stability! */
	if (!work_data->num_osc_samples) {
		/* Did we interrupt too early? */
		u_volt_current = omap_vp_get_curr_volt(voltdm);
		if (u_volt_current >= volt_data->volt_nominal)
			goto start_sampling;
		u_volt_safe = u_volt_current;
		goto done_calib;
	}

	/* we have potential oscillations/first sample */
start_sampling:
	work_data->num_osc_samples = 0;

	/* Clear transdone events so that we can go on. */
	do {
		if (!omap_vp_is_transdone(voltdm))
			break;
		idx++;
		/* get some constant delay */
		udelay(1);
		omap_vp_clear_transdone(voltdm);
	} while (idx < MAX_CHECK_VPTRANS_US);
	if (idx >= MAX_CHECK_VPTRANS_US)
		pr_warning("%s: timed out waiting for transdone clear!!\n",
			   __func__);

	/* Clear pending events */
	sr_notifier_control(voltdm, false);
	/* trigger sampling */
	sr_notifier_control(voltdm, true);
	schedule_delayed_work(&work_data->work,
			      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
					       SR1P5_STABLE_SAMPLES));
	mutex_unlock(&omap_dvfs_lock);
	return;

stop_sampling:
	/*
	 * We are here for Oscillations due to two scenarios:
	 * a) SR is attempting to adjust voltage lower than VLIMITO
	 *    which VP will ignore, but SR will re-attempt
	 * b) actual oscillations
	 * NOTE: For debugging, enable debug to see the samples.
	 */
	pr_warning("%s: %s Stop sampling: Voltage Nominal=%d samples=%d\n",
		   __func__, work_data->voltdm->name,
		   volt_data->volt_nominal, work_data->num_osc_samples);

	/* pick up current voltage */
	u_volt_current = omap_vp_get_curr_volt(voltdm);

	/* Just in case we got more interrupts than our tiny buffer */
	if (work_data->num_osc_samples > SR1P5_STABLE_SAMPLES)
		idx = SR1P5_STABLE_SAMPLES;
	else
		idx = work_data->num_osc_samples;
	/* Index at 0 */
	idx -= 1;
	u_volt_safe = u_volt_current;
	/* Grab the max of the samples as the stable voltage */
	for (; idx >= 0; idx--) {
		pr_debug("%s: osc_v[%d]=%ld, safe_v=%ld\n", __func__, idx,
			work_data->u_volt_samples[idx], u_volt_safe);
		if (work_data->u_volt_samples[idx] > u_volt_safe)
			u_volt_safe = work_data->u_volt_samples[idx];
	}
	/* Use the nominal voltage as the safe voltage to recover bad osc */
	if (u_volt_safe > volt_data->volt_nominal)
		u_volt_safe = volt_data->volt_nominal;


	/* Fall through to close up common stuff */
done_calib:
	sr_disable_errgen(voltdm);
	omap_vp_disable(voltdm);
	sr_disable(voltdm);

	/* Add margin if needed */
	if (volt_data->volt_margin) {
		struct omap_voltdm_pmic *pmic = voltdm->pmic;
		/* Convert to rounded to PMIC step level if available */
		if (pmic && pmic->vsel_to_uv && pmic->uv_to_vsel) {
			/*
			 * To ensure conversion works:
			 * use a proper base voltage - we use the current volt
			 * then convert it with pmic routine to vsel and back
			 * to voltage, and finally remove the base voltage
			 */
			u_volt_margin = u_volt_current + volt_data->volt_margin;
			u_volt_margin = pmic->uv_to_vsel(u_volt_margin);
			u_volt_margin = pmic->vsel_to_uv(u_volt_margin);
			u_volt_margin -= u_volt_current;
		} else {
			u_volt_margin = volt_data->volt_margin;
		}
		/* Add margin IF we are lower than nominal */
		if ((u_volt_safe + u_volt_margin) < volt_data->volt_nominal) {
			u_volt_safe += u_volt_margin;
		} else {
			pr_err("%s: %s could not add %ld[%d] margin"
				"to vnom %d curr_v=%ld\n",
				__func__, voltdm->name, u_volt_margin,
				volt_data->volt_margin, volt_data->volt_nominal,
				u_volt_current);
		}
	}

	volt_data->volt_calibrated = u_volt_safe;
	/* Setup my dynamic voltage for the next calibration for this opp */
	volt_data->volt_dynamic_nominal = omap_get_dyn_nominal(volt_data);

	/*
	 * if the voltage we decided as safe is not the current voltage,
	 * switch
	 */
	if (volt_data->volt_calibrated != u_volt_current) {
		pr_debug("%s: %s reconfiguring to voltage %d\n",
			 __func__, voltdm->name, volt_data->volt_calibrated);
		voltdm_scale(voltdm, volt_data);
	}

	pr_info("%s: %s: Calibration complete: Voltage:Nominal=%d,"
		"Calib=%d,margin=%d\n",
		 __func__, voltdm->name, volt_data->volt_nominal,
		 volt_data->volt_calibrated, volt_data->volt_margin);
	/*
	 * TODO: Setup my wakeup voltage to allow immediate going to OFF and
	 * on - Pending twl and voltage layer cleanups.
	 * This is necessary, as this is not done as part of regular
	 * Dvfs flow.
	 * vc_setup_on_voltage(voltdm, volt_data->volt_calibrated);
	 */
	work_data->work_active = false;
	mutex_unlock(&omap_dvfs_lock);
}

#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY

/**
 * sr_class1p5_voltdm_recal() - Helper routine to reset calibration.
 * @voltdm:	Voltage domain to reset calibration for
 * @user:	unused
 *
 * NOTE: Appropriate locks must be held by calling path to ensure mutual
 * exclusivity
 */
static int sr_class1p5_voltdm_recal(struct voltagedomain *voltdm,
		void *user)
{
	struct omap_volt_data *vdata;

	/*
	 * we need to go no further if sr is not enabled for this domain or
	 * voltage processor is not present for this voltage domain
	 * (example vdd_wakeup). Class 1.5 requires Voltage processor
	 * to function.
	 */
	if (!voltdm->vp || !is_sr_enabled(voltdm))
		return 0;

	vdata = omap_voltage_get_curr_vdata(voltdm);
	if (!vdata) {
		pr_err("%s: unable to find current voltage for vdd_%s\n",
			__func__, voltdm->name);
		return -ENXIO;
	}

	omap_sr_disable(voltdm);
	omap_voltage_calib_reset(voltdm);
	voltdm_reset(voltdm);
	omap_sr_enable(voltdm, vdata);
	pr_info("%s: %s: calibration reset\n", __func__, voltdm->name);

	return 0;
}

/**
 * sr_class1p5_recal_work() - work which actually does the calibration
 * @work: pointer to the work
 *
 * on a periodic basis, we come and reset our calibration setup
 * so that a recalibration of the OPPs take place. This takes
 * care of aging factor in the system.
 */
static void sr_class1p5_recal_work(struct work_struct *work)
{
	mutex_lock(&omap_dvfs_lock);
	if (voltdm_for_each(sr_class1p5_voltdm_recal, NULL))
		pr_err("%s: Recalibration failed\n", __func__);
	mutex_unlock(&omap_dvfs_lock);
	/* We come back again after time the usual delay */
	schedule_delayed_work(&recal_work,
			      msecs_to_jiffies
			      (CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY));
}
#endif			/* CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY */

/**
 * sr_class1p5_enable() - class 1.5 mode of enable for a voltage domain
 * @voltdm:		voltage domain to enable SR for
 * @voltdm_cdata:	voltage domain specific private class data
 * @volt_data:		voltdata for the current OPP being transitioned to
 *
 * when this gets called, we use the h/w loop to setup our voltages
 * to an calibrated voltage, detect any oscillations, recover from the same
 * and finally store the optimized voltage as the calibrated voltage in the
 * system.
 *
 * NOTE: Appropriate locks must be held by calling path to ensure mutual
 * exclusivity
 */
static int sr_class1p5_enable(struct voltagedomain *voltdm,
			      void *voltdm_cdata,
			      struct omap_volt_data *volt_data)
{
	int r;
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(volt_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	/* If already calibrated, nothing to do here.. */
	if (volt_data->volt_calibrated)
		return 0;
        // SR1.5 does not seem to calibrate high freqs properly - this is a
        // workaround until I find a better way
 	if (!enable_highvolt_sr && volt_data->volt_nominal > 1310000) {
		pr_info("[imoseyon] nominal@%d, skipping sr_enable\n", 
			volt_data->volt_nominal);
		volt_data->volt_calibrated = volt_data->volt_nominal;
		volt_data->volt_dynamic_nominal = volt_data->volt_nominal;
		return 0;
	}

	work_data = (struct sr_class1p5_work_data *)voltdm_cdata;
	if (IS_ERR_OR_NULL(work_data)) {
		pr_err("%s: bad work data??\n", __func__);
		return -EINVAL;
	}

	if (work_data->work_active)
		return 0;

	omap_vp_enable(voltdm);
	r = sr_enable(voltdm, volt_data);
	if (r) {
		pr_err("%s: sr[%s] failed\n", __func__, voltdm->name);
		sr_disable_errgen(voltdm);
		omap_vp_disable(voltdm);
		return r;
	}
	work_data->vdata = volt_data;
	work_data->work_active = true;
	work_data->num_calib_triggers = 0;
	/* program the workqueue and leave it to calibrate offline.. */
	schedule_delayed_work(&work_data->work,
			      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
					       SR1P5_STABLE_SAMPLES));

	return 0;
}

/**
 * sr_class1p5_disable() - disable 1.5 mode for a voltage domain
 * @voltdm: voltage domain for the sr which needs disabling
 * @volt_data:	voltage data for current OPP to disable
 * @voltdm_cdata: voltage domain specific private class data
 * @is_volt_reset: reset the voltage?
 *
 * This function has the necessity to either disable SR alone OR disable SR
 * and reset voltage to appropriate level depending on is_volt_reset parameter.
 *
 * Disabling SR H/w loop:
 * If calibration is complete or not yet triggered, we have no need to disable
 * SR h/w loop.
 * If calibration is complete, we would have already disabled SR AVS at the end
 * of calibration and h/w loop is inactive when this is called.
 * If it was never calibrated before, H/w loop was never enabled in the first
 * place to disable.
 * If calibration is underway, we cancel the work queue and disable SR. This is
 * to provide priority to DVFS transition as such transitions cannot wait
 * without impacting user experience.
 *
 * Resetting voltage:
 * If we have already completed calibration, then resetting to nominal voltage
 * is not required as we are functioning at safe voltage levels.
 * If we have not started calibration, we would like to reset to nominal voltage
 * If calibration is underway and we are attempting to reset voltage as
 * well, it implies we are in idle/suspend paths where we give priority
 * to calibration activity and a retry will be attempted.
 *
 * NOTE: Appropriate locks must be held by calling path to ensure mutual
 * exclusivity
 */
static int sr_class1p5_disable(struct voltagedomain *voltdm,
			       void *voltdm_cdata,
			       struct omap_volt_data *volt_data,
			       int is_volt_reset)
{
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(volt_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	work_data = (struct sr_class1p5_work_data *)voltdm_cdata;
	if (IS_ERR_OR_NULL(work_data)) {
		pr_err("%s: bad work data??\n", __func__);
		return -EINVAL;
	}
	if (work_data->work_active) {
		/* if volt reset and work is active, we dont allow this */
		if (is_volt_reset)
			return -EBUSY;
		/* flag work is dead and remove the old work */
		work_data->work_active = false;
		cancel_delayed_work_sync(&work_data->work);
		sr_notifier_control(voltdm, false);
		sr_disable_errgen(voltdm);
		omap_vp_disable(voltdm);
		sr_disable(voltdm);
	}

	/* If already calibrated, don't need to reset voltage */
	if (volt_data->volt_calibrated)
		return 0;

	if (is_volt_reset)
		voltdm_reset(voltdm);
	return 0;
}

/**
 * sr_class1p5_configure() - configuration function
 * @voltdm:	configure for which voltage domain
 * @voltdm_cdata: voltage domain specific private class data
 *
 * we dont do much here other than setup some registers for
 * the sr module involved.
 */
static int sr_class1p5_configure(struct voltagedomain *voltdm,
				 void *voltdm_cdata)
{
	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	return sr_configure_errgen(voltdm);
}

/**
 * sr_class1p5_init() - class 1p5 init
 * @voltdm:		sr voltage domain
 * @voltdm_cdata:	voltage domain specific private class data
 *			allocated by class init with work item data
 *			freed by deinit.
 * @class_priv_data:	private data for the class (unused)
 *
 * we do class specific initialization like creating sysfs/debugfs entries
 * needed, spawning of a kthread if needed etc.
 */
static int sr_class1p5_init(struct voltagedomain *voltdm,
			    void **voltdm_cdata, void *class_priv_data)
{
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(voltdm_cdata)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(*voltdm_cdata)) {
		pr_err("%s: ooopps.. class already initialized for %s! bug??\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}
	/* setup our work params */
	work_data = kzalloc(sizeof(struct sr_class1p5_work_data), GFP_KERNEL);
	if (!work_data) {
		pr_err("%s: no memory to allocate work data on domain %s\n",
			__func__, voltdm->name);
		return -ENOMEM;
	}

	work_data->voltdm = voltdm;
	INIT_DELAYED_WORK_DEFERRABLE(&work_data->work, sr_class1p5_calib_work);
	*voltdm_cdata = (void *)work_data;

	return 0;
}

/**
 * sr_class1p5_deinit() - class 1p5 deinitialization
 * @voltdm:	voltage domain for which to do this.
 * @voltdm_cdata: voltage domain specific private class data
 *		allocated by class init with work item data
 *		freed by deinit.
 * @class_priv_data: class private data for deinitialiation (unused)
 *
 * currently only resets the calibrated voltage forcing DVFS voltages
 * to be used in the system
 *
 * NOTE: Appropriate locks must be held by calling path to ensure mutual
 * exclusivity
 */
static int sr_class1p5_deinit(struct voltagedomain *voltdm,
			      void **voltdm_cdata, void *class_priv_data)
{
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(voltdm_cdata)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(*voltdm_cdata)) {
		pr_err("%s: ooopps.. class not initialized for %s! bug??\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	work_data = (struct sr_class1p5_work_data *) *voltdm_cdata;

	/*
	 * we dont have SR periodic calib anymore.. so reset calibs
	 * we are already protected by appropriate locks, so no lock needed
	 * here.
	 */
	if (work_data->work_active)
		sr_class1p5_disable(voltdm, work_data, work_data->vdata, 0);

	/* Ensure worker canceled. */
	cancel_delayed_work_sync(&work_data->work);
	omap_voltage_calib_reset(voltdm);
	voltdm_reset(voltdm);

	*voltdm_cdata = NULL;
	kfree(work_data);

	return 0;
}

/* SR class1p5 structure */
static struct omap_sr_class_data class1p5_data = {
	.enable = sr_class1p5_enable,
	.disable = sr_class1p5_disable,
	.configure = sr_class1p5_configure,
	.class_type = SR_CLASS1P5,
	.init = sr_class1p5_init,
	.deinit = sr_class1p5_deinit,
	.notify = sr_class1p5_notify,
	/*
	 * trigger for bound - this tells VP that SR has a voltage
	 * change. we should try and ensure transdone is set before reading
	 * vp voltage.
	 */
	.notify_flags = SR_NOTIFY_MCUBOUND,
};

/**
 * sr_class1p5_driver_init() - register class 1p5 as default
 *
 * board files call this function to use class 1p5, we register with the
 * smartreflex subsystem
 */
static int __init sr_class1p5_driver_init(void)
{
	int r;

	/* Enable this class only for OMAP3630 and OMAP4 */
	if (!(cpu_is_omap3630() || cpu_is_omap44xx()))
		return -EINVAL;

	r = sr_register_class(&class1p5_data);
	if (r) {
		pr_err("SmartReflex class 1.5 driver: "
		       "failed to register with %d\n", r);
	} else {
#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY
		INIT_DELAYED_WORK_DEFERRABLE(&recal_work,
					     sr_class1p5_recal_work);
		schedule_delayed_work(&recal_work,
			      msecs_to_jiffies
			      (CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY));
#endif
		pr_info("SmartReflex class 1.5 driver: initialized (%dms)\n",
			CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY);
	}
	return r;
}
late_initcall(sr_class1p5_driver_init);
