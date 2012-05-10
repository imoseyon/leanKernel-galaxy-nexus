/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/opp.h>

#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>

#include "voltage.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "pm.h"

/**
 * struct omap2_pm_lp_description - Describe low power behavior of the system
 * @oscillator_startup_time:	Time rounded up to uSec for the oscillator to
 *				provide a stable clock from power on.
 * @oscillator_shutdown_time:	Time rounded up to uSec for oscillator to safely
 *				switch off.
 * @pmic_startup_time:		Time rounded up to uSec for the PMIC to
 *				provide be ready for operation from low power
 *				state. Note: this is not the same as voltage
 *				rampup time, instead, consider the PMIC to be
 *				in lowest power state(say OFF), this is the time
 *				required for it to become ready for it's DCDCs
 *				or LDOs to start operation.
 * @pmic_shutdown_time:		Time rounded up to uSec for the PMIC to
 *				go to low power after the LDOs are pulled to
 *				appropriate state. Note: this is not the same as
 *				voltage rampdown time, instead, consider the
 *				PMIC to have switched it's LDOs down, this is
 *				time taken to reach it's lowest power state(say
 *				sleep/OFF).
 *
 * With complex systems like OMAP, we need a generic description of system
 * behavior beyond the normal description of device/peripheral operation
 * which in conjunction with other parameters describe and control the low
 * power operation of the device. This information tends to be specific
 * to every board.
 */
struct omap2_pm_lp_description {
	u32 oscillator_startup_time;
	u32 oscillator_shutdown_time;
	u32 pmic_startup_time;
	u32 pmic_shutdown_time;
};

/*
 * Setup time to be the max... we want to err towards the worst
 * as default. rest of the system can populate these with more
 * optimal values
 */
static struct omap2_pm_lp_description _pm_lp_desc = {
	.oscillator_startup_time = ULONG_MAX,
	.oscillator_shutdown_time = ULONG_MAX,
	.pmic_startup_time = ULONG_MAX,
	.pmic_shutdown_time = ULONG_MAX,
};

static struct omap_device_pm_latency *pm_lats;

static struct device *mpu_dev;
static struct device *iva_dev;
static struct device *l3_dev;
static struct device *dsp_dev;
static struct device *fdif_dev;

bool omap_pm_is_ready_status;

struct device *omap2_get_mpuss_device(void)
{
	WARN_ON_ONCE(!mpu_dev);
	return mpu_dev;
}
EXPORT_SYMBOL(omap2_get_mpuss_device);

struct device *omap2_get_iva_device(void)
{
	WARN_ON_ONCE(!iva_dev);
	return iva_dev;
}
EXPORT_SYMBOL(omap2_get_iva_device);

struct device *omap2_get_l3_device(void)
{
	WARN_ON_ONCE(!l3_dev);
	return l3_dev;
}
EXPORT_SYMBOL(omap2_get_l3_device);

struct device *omap4_get_dsp_device(void)
{
	WARN_ON_ONCE(!dsp_dev);
	return dsp_dev;
}
EXPORT_SYMBOL(omap4_get_dsp_device);

struct device *omap4_get_fdif_device(void)
{
	WARN_ON_ONCE(!fdif_dev);
	return fdif_dev;
}
EXPORT_SYMBOL(omap4_get_fdif_device);

/**
 * omap_pm_get_pmic_lp_time() - retrieve the oscillator time
 * @tstart:	pointer to startup time in uSec
 * @tshut:	pointer to shutdown time in uSec
 *
 * if the pointers are invalid, returns error, else
 * populates the tstart and tshut values with the currently
 * stored values.
 */
int omap_pm_get_osc_lp_time(u32 *tstart, u32 *tshut)
{
	if (!tstart || !tshut)
		return -EINVAL;

	*tstart = _pm_lp_desc.oscillator_startup_time;
	*tshut = _pm_lp_desc.oscillator_shutdown_time;

	return 0;
}

/**
 * omap_pm_get_pmic_lp_time() - retrieve the PMIC time
 * @tstart:	pointer to startup time in uSec
 * @tshut:	pointer to shutdown time in uSec
 *
 * if the pointers are invalid, returns error, else
 * populates the tstart and tshut values with the currently
 * stored values.
 */
int omap_pm_get_pmic_lp_time(u32 *tstart, u32 *tshut)
{
	if (!tstart || !tshut)
		return -EINVAL;

	*tstart = _pm_lp_desc.pmic_startup_time;
	*tshut = _pm_lp_desc.pmic_shutdown_time;

	return 0;
}

/**
 * omap_pm_set_osc_lp_time() - setup the system oscillator time
 * @tstart:	startup time rounded up to uSec
 * @tshut:	shutdown time rounded up to uSec
 *
 * All boards do need an oscillator for the device to function.
 * The startup and stop time of these oscillators vary. Populate
 * from the board file to optimize the timing.
 * This function is meant to be used at boot-time configuration.
 *
 * NOTE: This API is intended to be invoked from board file
 */
void __init omap_pm_set_osc_lp_time(u32 tstart, u32 tshut)
{
	_pm_lp_desc.oscillator_startup_time = tstart;
	_pm_lp_desc.oscillator_shutdown_time = tshut;
}

/**
 * omap_pm_set_pmic_lp_time() - setup the pmic low power time
 * @tstart:	startup time rounded up to uSec
 * @tshut:	shutdown time rounded up to uSec
 *
 * Store the time for PMIC to enter to lowest state supported.
 * in the case of multiple PMIC on a platform, choose the one
 * that ends the sequence for LP state such as OFF and starts
 * the sequence such as wakeup from OFF - e.g. a PMIC that
 * controls core-domain.
 * This function is meant to be used at boot-time configuration.
 */
void __init omap_pm_set_pmic_lp_time(u32 tstart, u32 tshut)
{
	_pm_lp_desc.pmic_startup_time = tstart;
	_pm_lp_desc.pmic_shutdown_time = tshut;
}

/* static int _init_omap_device(struct omap_hwmod *oh, void *user) */
static int _init_omap_device(char *name, struct device **new_dev)
{
	struct omap_hwmod *oh;
	struct omap_device *od;

	oh = omap_hwmod_lookup(name);
	if (WARN(!oh, "%s: could not find omap_hwmod for %s\n",
		 __func__, name))
		return -ENODEV;

	od = omap_device_build(oh->name, 0, oh, NULL, 0, pm_lats, 0, false);
	if (WARN(IS_ERR(od), "%s: could not build omap_device for %s\n",
		 __func__, name))
		return -ENODEV;

	*new_dev = &od->pdev.dev;

	return 0;
}

/*
 * Build omap_devices for processors and bus.
 */
static void omap2_init_processor_devices(void)
{
	_init_omap_device("mpu", &mpu_dev);
	if (omap3_has_iva())
		_init_omap_device("iva", &iva_dev);

	if (cpu_is_omap44xx()) {
		_init_omap_device("l3_main_1", &l3_dev);
		_init_omap_device("dsp", &dsp_dev);
		_init_omap_device("iva", &iva_dev);
		_init_omap_device("fdif", &fdif_dev);
	} else {
		_init_omap_device("l3_main", &l3_dev);
	}
}

/* Types of sleep_switch used in omap_set_pwrdm_state */
#define FORCEWAKEUP_SWITCH	0
#define LOWPOWERSTATE_SWITCH	1

/*
 * This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported.
 */
int omap_set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = -1;
	int ret = 0;
	int hwsup = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		if ((pwrdm_read_pwrst(pwrdm) > state) &&
			(pwrdm->flags & PWRDM_HAS_LOWPOWERSTATECHANGE)) {
			sleep_switch = LOWPOWERSTATE_SWITCH;
		} else {
			hwsup = clkdm_is_idle(pwrdm->pwrdm_clkdms[0]);
			clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
			pwrdm_wait_transition(pwrdm);
			sleep_switch = FORCEWAKEUP_SWITCH;
		}
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	switch (sleep_switch) {
	case FORCEWAKEUP_SWITCH:
		if (hwsup)
			clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		else
			clkdm_sleep(pwrdm->pwrdm_clkdms[0]);
		break;
	case LOWPOWERSTATE_SWITCH:
		pwrdm_set_lowpwrstchange(pwrdm);
		break;
	default:
		return ret;
	}

	pwrdm_wait_transition(pwrdm);
	pwrdm_state_switch(pwrdm);
err:
	return ret;
}

static int __init boot_volt_scale(struct voltagedomain *voltdm,
				  unsigned long boot_v)
{
	struct omap_volt_data *vdata;
	int ret = 0;

	vdata = omap_voltage_get_voltdata(voltdm, boot_v);
	if (IS_ERR_OR_NULL(vdata)) {
		pr_err("%s:%s: Bad New voltage data for %ld\n",
			__func__, voltdm->name, boot_v);
		return PTR_ERR(vdata);
	}
	/*
	 * DO NOT DO abb prescale -
	 * case 1: OPP needs FBB, bootloader configured FBB
	 *  - doing a prescale results in bypass -> system fail
	 * case 2: OPP needs FBB, bootloader does not configure FBB
	 * - FBB will be configured in postscale
	 * case 3: OPP needs bypass, bootloader configures FBB
	 * - bypass will be configured in postscale
	 * case 4: OPP needs bypass, bootloader configured in bypass
	 * - bypass programming in postscale skipped
	 */
	ret = voltdm_scale(voltdm, vdata);
	if (ret) {
		pr_err("%s: Fail set voltage(v=%ld)on vdd%s\n",
			__func__, boot_v, voltdm->name);
		return ret;
	}
	if (voltdm->abb) {
		ret = omap_ldo_abb_post_scale(voltdm, vdata);
		if (ret) {
			pr_err("%s: Fail abb postscale(v=%ld)vdd%s\n",
				__func__, boot_v, voltdm->name);
		}
	}
	return ret;
}

/*
 * This API is to be called during init to put the various voltage
 * domains to the voltage as per the opp table. Typically we boot up
 * at the nominal voltage. So this function finds out the rate of
 * the clock associated with the voltage domain, finds out the correct
 * opp entry and puts the voltage domain to the voltage specifies
 * in the opp entry
 */
static int __init omap2_set_init_voltage(char *vdd_name, char *clk_name,
						struct device *dev)
{
	struct voltagedomain *voltdm;
	struct clk *clk;
	struct opp *opp;
	unsigned long freq_cur, freq_valid, bootup_volt;
	int ret = -EINVAL;

	if (!vdd_name || !clk_name || !dev) {
		printk(KERN_ERR "%s: Invalid parameters!\n", __func__);
		goto exit;
	}

	voltdm = voltdm_lookup(vdd_name);
	if (IS_ERR(voltdm)) {
		printk(KERN_ERR "%s: Unable to get vdd pointer for vdd_%s\n",
			__func__, vdd_name);
		goto exit;
	}

	clk =  clk_get(NULL, clk_name);
	if (IS_ERR(clk)) {
		printk(KERN_ERR "%s: unable to get clk %s\n",
			__func__, clk_name);
		goto exit;
	}

	freq_cur = clk->rate;
	freq_valid = freq_cur;

	rcu_read_lock();
	opp = opp_find_freq_ceil(dev, &freq_valid);
	if (IS_ERR(opp)) {
		opp = opp_find_freq_floor(dev, &freq_valid);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			pr_err("%s: no boot OPP match for %ld on vdd_%s\n",
				__func__, freq_cur, vdd_name);
			ret = -ENOENT;
			goto exit_ck;
		}
	}

	bootup_volt = opp_get_voltage(opp);
	rcu_read_unlock();
	if (!bootup_volt) {
		printk(KERN_ERR "%s: unable to find voltage corresponding"
			"to the bootup OPP for vdd_%s\n", __func__, vdd_name);
		ret = -ENOENT;
		goto exit_ck;
	}

	/*
	 * Frequency and Voltage have to be sequenced: if we move from
	 * a lower frequency to higher frequency, raise voltage, followed by
	 * frequency, and vice versa. we assume that the voltage at boot
	 * is the required voltage for the frequency it was set for.
	 * NOTE:
	 * we can check the frequency, but there is numerous ways to set
	 * voltage. We play the safe path and just set the voltage.
	 */

	if (freq_cur < freq_valid) {
		ret = boot_volt_scale(voltdm, bootup_volt);
		if (ret) {
			pr_err("%s: Fail set voltage-%s(f=%ld v=%ld)on vdd%s\n",
				__func__, vdd_name, freq_valid,
				bootup_volt, vdd_name);
			goto exit_ck;
		}
	}

	/* Set freq only if there is a difference in freq */
	if (freq_valid != freq_cur) {
		ret = clk_set_rate(clk, freq_valid);
		if (ret) {
			pr_err("%s: Fail set clk-%s(f=%ld v=%ld)on vdd%s\n",
				__func__, clk_name, freq_valid,
				bootup_volt, vdd_name);
			goto exit_ck;
		}
	}

	if (freq_cur >= freq_valid) {
		ret = boot_volt_scale(voltdm, bootup_volt);
		if (ret) {
			pr_err("%s: Fail set voltage-%s(f=%ld v=%ld)on vdd%s\n",
				__func__, clk_name, freq_valid,
				bootup_volt, vdd_name);
			goto exit_ck;
		}
	}

	ret = 0;
exit_ck:
	clk_put(clk);

	if (!ret)
		return 0;

exit:
	printk(KERN_ERR "%s: Unable to put vdd_%s to its init voltage\n\n",
		__func__, vdd_name);
	return -EINVAL;
}

static void __init omap3_init_voltages(void)
{
	if (!cpu_is_omap34xx())
		return;

	omap2_set_init_voltage("mpu_iva", "dpll1_ck", mpu_dev);
	omap2_set_init_voltage("core", "l3_ick", l3_dev);
}

static void __init omap4_init_voltages(void)
{
	if (!cpu_is_omap44xx())
		return;

	if (cpu_is_omap446x()) {
		omap2_set_init_voltage("mpu", "virt_dpll_mpu_ck", mpu_dev);
	} else {
		omap2_set_init_voltage("mpu", "dpll_mpu_ck", mpu_dev);
	}
	omap2_set_init_voltage("core", "virt_l3_ck", l3_dev);
	omap2_set_init_voltage("iva", "dpll_iva_m5x2_ck", iva_dev);
}

static int __init omap2_common_pm_init(void)
{
	omap2_init_processor_devices();
	omap_pm_if_init();

	return 0;
}
postcore_initcall(omap2_common_pm_init);

static int __init omap2_common_pm_late_init(void)
{
	/* Init the OMAP PMIC parameters */
	omap_pmic_data_init();

	/* Init the voltage layer */
	omap_voltage_late_init();

	/* Initialize the voltages */
	omap3_init_voltages();
	omap4_init_voltages();

	/* Smartreflex device init */
	omap_devinit_smartreflex();

	return 0;
}
late_initcall(omap2_common_pm_late_init);
