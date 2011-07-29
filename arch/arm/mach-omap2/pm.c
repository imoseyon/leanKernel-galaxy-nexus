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
		ret = voltdm_scale(voltdm, bootup_volt);
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
		ret = voltdm_scale(voltdm, bootup_volt);
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

	omap_pm_is_ready_status = true;
	/* let the other CPU know as well */
	smp_wmb();

	return 0;
}
late_initcall(omap2_common_pm_late_init);
