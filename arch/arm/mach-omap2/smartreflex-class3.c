/*
 * Smart reflex Class 3 specific implementations
 *
 * Author: Thara Gopinath       <thara@ti.com>
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <plat/cpu.h>
#include "smartreflex.h"

static int sr_class3_enable(struct voltagedomain *voltdm,
			    void *voltdm_cdata,
			    struct omap_volt_data *volt_data)
{
	omap_vp_enable(voltdm);
	return sr_enable(voltdm, volt_data);
}

static int sr_class3_disable(struct voltagedomain *voltdm,
			     void *voltdm_cdata,
			     struct omap_volt_data *vdata,
			     int is_volt_reset)
{
	sr_disable_errgen(voltdm);
	omap_vp_disable(voltdm);
	sr_disable(voltdm);
	if (is_volt_reset)
		voltdm_reset(voltdm);

	return 0;
}

static int sr_class3_configure(struct voltagedomain *voltdm,
			       void *voltdm_cdata)
{
	return sr_configure_errgen(voltdm);
}

/* SR class3 structure */
static struct omap_sr_class_data class3_data = {
	.enable = sr_class3_enable,
	.disable = sr_class3_disable,
	.configure = sr_class3_configure,
	.class_type = SR_CLASS3,
};

/* Smartreflex Class3 init API to be called from board file */
static int __init sr_class3_init(void)
{
	/* Enable this class only for OMAP343x */
	if (!cpu_is_omap343x())
		return -EINVAL;

	pr_info("SmartReflex Class3 initialized\n");
	return sr_register_class(&class3_data);
}
late_initcall(sr_class3_init);
