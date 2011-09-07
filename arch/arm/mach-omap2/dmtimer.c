/**
 * OMAP2+ Dual-Mode Timers - platform device registration
 *
 * Contains first level initialization routines which extracts timers
 * information from hwmod database and registers with linux device model.
 * It also has low level function to change the timer input clock source.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/dmtimer.h>
#include <plat/omap_device.h>
#include <plat/cpu.h>
#include <plat/omap_hwmod.h>
#include <plat/omap-pm.h>

#include "powerdomain.h"

static u8 __initdata system_timer_id;

/**
 * omap2_dm_timer_set_src - change the timer input clock source
 * @pdev:	timer platform device pointer
 * @source:	array index of parent clock source
 */
static int omap2_dm_timer_set_src(struct platform_device *pdev, int source)
{
	int ret;
	struct dmtimer_platform_data *pdata = pdev->dev.platform_data;
	struct clk *new_fclk;
	char *fclk_name = "32k_ck"; /* default name */

	struct clk *fclk = clk_get(&pdev->dev, "fck");
	if (IS_ERR_OR_NULL(fclk)) {
		dev_err(&pdev->dev, "%s: %d: clk_get() FAILED\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	switch (source) {
	case OMAP_TIMER_SRC_SYS_CLK:
		fclk_name = "sys_ck";
		break;

	case OMAP_TIMER_SRC_32_KHZ:
		fclk_name = "32k_ck";
		break;

	case OMAP_TIMER_SRC_EXT_CLK:
		if (pdata->timer_ip_type == OMAP_TIMER_IP_VERSION_1) {
			fclk_name = "alt_ck";
			break;
		}
	default:
		dev_err(&pdev->dev, "%s: %d: invalid clk src.\n",
			__func__, __LINE__);
		clk_put(fclk);
		return -EINVAL;
	}

	new_fclk = clk_get(&pdev->dev, fclk_name);
	if (IS_ERR_OR_NULL(new_fclk)) {
		dev_err(&pdev->dev, "%s: %d: clk_get() %s FAILED\n",
			__func__, __LINE__, fclk_name);
		clk_put(fclk);
		return -EINVAL;
	}

	ret = clk_set_parent(fclk, new_fclk);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "%s: clk_set_parent() to %s FAILED\n",
			__func__, fclk_name);
		ret = -EINVAL;
	}

	clk_put(new_fclk);
	clk_put(fclk);

	return ret;
}

struct omap_device_pm_latency omap2_dmtimer_latency[] = {
	{
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func   = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

/**
 * omap_timer_init - build and register timer device with an
 * associated timer hwmod
 * @oh:	timer hwmod pointer to be used to build timer device
 * @user:	parameter that can be passed from calling hwmod API
 *
 * Called by omap_hwmod_for_each_by_class to register each of the timer
 * devices present in the system. The number of timer devices is known
 * by parsing through the hwmod database for a given class name. At the
 * end of function call memory is allocated for timer device and it is
 * registered to the framework ready to be proved by the driver.
 */
static int __init omap_timer_init(struct omap_hwmod *oh, void *unused)
{
	int id;
	int ret = 0;
	char *name = "omap_timer";
	struct dmtimer_platform_data *pdata;
	struct omap_device *od;
	struct omap_secure_timer_dev_attr *secure_timer_dev_attr;
	struct powerdomain *pwrdm;

	/*
	 * Extract the IDs from name field in hwmod database
	 * and use the same for constructing ids' for the
	 * timer devices. In a way, we are avoiding usage of
	 * static variable witin the function to do the same.
	 * CAUTION: We have to be careful and make sure the
	 * name in hwmod database does not change in which case
	 * we might either make corresponding change here or
	 * switch back static variable mechanism.
	 */
	sscanf(oh->name, "timer%2d", &id);
	if (unlikely(id == system_timer_id))
		return ret;

	pr_debug("%s: %s\n", __func__, oh->name);

	/* do not register secure timer */
	secure_timer_dev_attr = oh->dev_attr;
	if (secure_timer_dev_attr && secure_timer_dev_attr->is_secure_timer)
			return ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: No memory for [%s]\n", __func__, oh->name);
		return -ENOMEM;
	}
	pdata->set_timer_src = omap2_dm_timer_set_src;
	pdata->timer_ip_type = oh->class->rev;
	pwrdm = omap_hwmod_get_pwrdm(oh);
	if (!pwrdm) {
		pr_debug("%s: could not find pwrdm for (%s) in omap hwmod!\n",
			__func__, oh->name);
		return -EINVAL;
	}
	pdata->loses_context = pwrdm_can_ever_lose_context(pwrdm);

	od = omap_device_build(name, id, oh, pdata, sizeof(*pdata),
			omap2_dmtimer_latency,
			ARRAY_SIZE(omap2_dmtimer_latency),
			pdata->is_early_init);

	if (IS_ERR(od)) {
		pr_err("%s: Can't build omap_device for %s: %s.\n",
			__func__, name, oh->name);
		ret = -EINVAL;
	}

	kfree(pdata);

	return ret;
}

/**
 * omap2_system_timer_init - top level system timer initialization
 * called from omap2_gp_timer_init() in timer-gp.c
 * @id	: system timer id
 *
 * This function does hwmod setup for the system timer entry needed
 * prior to building and registering the device. After the device is
 * registered early probe initiated.
 */
int __init omap2_system_timer_init(u8 id)
{
	int ret = 0;
	char *name = "omap_timer";
	struct dmtimer_platform_data *pdata;
	struct omap_device *od;
	struct omap_hwmod *oh;
	char system_timer_name[8]; /* 8 = sizeof("timerXX0") */

	system_timer_id = id;

	sprintf(system_timer_name, "timer%d", id);
	ret = omap_hwmod_setup_one(system_timer_name);
	if (ret) {
		pr_err("%s: omap_hwmod_setup_one(%s) failed.\n",
				__func__, system_timer_name);
		return ret;
	}
	oh = omap_hwmod_lookup(system_timer_name);
	if (!oh) {
		pr_debug("%s: could not find (%s) in omap_hwmod_list!\n",
			__func__, system_timer_name);
		return -EINVAL;
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: No memory for [%s]\n", __func__, oh->name);
		return -ENOMEM;
	}
	pdata->is_early_init = 1;
	pdata->set_timer_src = omap2_dm_timer_set_src;
	pdata->timer_ip_type = oh->class->rev;
	pdata->needs_manual_reset = 0;

	od = omap_device_build(name, id, oh, pdata, sizeof(*pdata),
			omap2_dmtimer_latency,
			ARRAY_SIZE(omap2_dmtimer_latency),
			pdata->is_early_init);

	if (IS_ERR(od)) {
		pr_err("%s: Can't build omap_device for %s: %s.\n",
			__func__, name, oh->name);
		ret = -EINVAL;
	}

	kfree(pdata);

	if (!ret) {
		early_platform_driver_register_all("earlytimer");
		early_platform_driver_probe("earlytimer", 1, 0);
	}

	return 0;
}

/**
 * omap2_system_timer_set_src - change the timer input clock source
 * Allow system timer to program clock source before pm_runtime
 * framework is available during system boot.
 * @timer:       pointer to struct omap_dm_timer
 * @source:     array index of parent clock source
 */
int __init omap2_system_timer_set_src(struct omap_dm_timer *timer, int source)
{
	int ret;

	if (IS_ERR_OR_NULL(timer) || IS_ERR_OR_NULL(timer->fclk))
		return -EINVAL;

	clk_disable(timer->fclk);
	ret = omap2_dm_timer_set_src(timer->pdev, source);
	clk_enable(timer->fclk);

	return ret;
}

/**
 * omap2_dm_timer_init - top level regular device initialization
 *
 * Uses dedicated hwmod api to parse through hwmod database for
 * given class name and then build and register the timer device.
 */
static int __init omap2_dm_timer_init(void)
{
	int ret;

	ret = omap_hwmod_for_each_by_class("timer", omap_timer_init, NULL);
	if (unlikely(ret)) {
		pr_err("%s: device registration failed.\n", __func__);
		return -EINVAL;
	}

	return 0;
}
arch_initcall(omap2_dm_timer_init);
