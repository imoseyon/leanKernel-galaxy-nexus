/*
 * OMAP on die Temperature sensor device file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <plat/omap_device.h>
#include "control.h"
#include "pm.h"
#include <plat/temperature_sensor.h>

void omap_temp_sensor_resume_idle(void)
{
	omap_temp_sensor_idle(0);
}

void omap_temp_sensor_prepare_idle(void)
{
	omap_temp_sensor_idle(1);
}

static struct omap_device_pm_latency omap_temp_sensor_latency[] = {
	{
	 .deactivate_func = omap_device_idle_hwmods,
	 .activate_func = omap_device_enable_hwmods,
	 .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	}
};

static int temp_sensor_dev_init(struct omap_hwmod *oh, void *user)
{
	struct omap_temp_sensor_pdata *temp_sensor_pdata;
	struct omap_device *od;
	static int i;
	int ret = 0;

	temp_sensor_pdata =
	    kzalloc(sizeof(struct omap_temp_sensor_pdata), GFP_KERNEL);
	if (!temp_sensor_pdata) {
		pr_err
		    ("%s: Unable to allocate memory for %s.Error!\n",
			__func__, oh->name);
		return -ENOMEM;
	}

	temp_sensor_pdata->offset = OMAP4_CTRL_MODULE_CORE_TEMP_SENSOR;

	temp_sensor_pdata->name = "omap_temp_sensor";

	od = omap_device_build(temp_sensor_pdata->name, i, oh, temp_sensor_pdata,
			       sizeof(*temp_sensor_pdata),
			       omap_temp_sensor_latency,
			       ARRAY_SIZE(omap_temp_sensor_latency), 0);
	if (IS_ERR(od)) {
		pr_warning("%s: Could not build omap_device for %s: %s.\n\n",
			   __func__, temp_sensor_pdata->name, oh->name);
		ret = -EINVAL;
		goto done;
	}

	i++;
done:
	kfree(temp_sensor_pdata);
	return ret;
}

int __init omap_devinit_temp_sensor(void)
{
	if (!cpu_is_omap446x())
		return 0;

	return omap_hwmod_for_each_by_class("thermal_sensor",
			temp_sensor_dev_init, NULL);
}

arch_initcall(omap_devinit_temp_sensor);
