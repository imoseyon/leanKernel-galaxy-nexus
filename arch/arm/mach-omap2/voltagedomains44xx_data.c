/*
 * OMAP3/OMAP4 Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/clk.h>

#include <plat/common.h>

#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "prcm44xx.h"
#include "prminst44xx.h"
#include "voltage.h"
#include "omap_opp_data.h"
#include "vc.h"
#include "vp.h"
#include "ldo.h"

static const struct omap_vfsm_instance omap4_vdd_mpu_vfsm = {
	.voltsetup_reg = OMAP4_PRM_VOLTSETUP_MPU_RET_SLEEP_OFFSET,
	.voltsetup_mask =	OMAP4430_RAMP_DOWN_PRESCAL_MASK |
				OMAP4430_RAMP_DOWN_COUNT_MASK |
				OMAP4430_RAMP_UP_PRESCAL_MASK |
				OMAP4430_RAMP_UP_COUNT_MASK,
	.voltsetupoff_reg = OMAP4_PRM_VOLTSETUP_MPU_OFF_OFFSET,
};

static struct omap_vdd_info omap4_vdd_mpu_info;

static const struct omap_vfsm_instance omap4_vdd_iva_vfsm = {
	.voltsetup_reg = OMAP4_PRM_VOLTSETUP_IVA_RET_SLEEP_OFFSET,
	.voltsetup_mask =	OMAP4430_RAMP_DOWN_PRESCAL_MASK |
				OMAP4430_RAMP_DOWN_COUNT_MASK |
				OMAP4430_RAMP_UP_PRESCAL_MASK |
				OMAP4430_RAMP_UP_COUNT_MASK,
	.voltsetupoff_reg = OMAP4_PRM_VOLTSETUP_IVA_OFF_OFFSET,
};

static struct omap_vdd_info omap4_vdd_iva_info;

static const struct omap_vfsm_instance omap4_vdd_core_vfsm = {
	.voltsetup_reg = OMAP4_PRM_VOLTSETUP_CORE_RET_SLEEP_OFFSET,
	.voltsetup_mask =	OMAP4430_RAMP_DOWN_PRESCAL_MASK |
				OMAP4430_RAMP_DOWN_COUNT_MASK |
				OMAP4430_RAMP_UP_PRESCAL_MASK |
				OMAP4430_RAMP_UP_COUNT_MASK,
	.voltsetupoff_reg = OMAP4_PRM_VOLTSETUP_CORE_OFF_OFFSET,
};

static struct omap_vdd_info omap4_vdd_core_info;

static struct voltagedomain omap4_voltdm_mpu = {
	.name = "mpu",
	.scalable = true,
	.read = omap4_prm_vcvp_read,
	.write = omap4_prm_vcvp_write,
	.rmw = omap4_prm_vcvp_rmw,
	.vc = &omap4_vc_mpu,
	.vfsm = &omap4_vdd_mpu_vfsm,
	.vp = &omap4_vp_mpu,
	.vdd = &omap4_vdd_mpu_info,
	.abb = &omap4_ldo_abb_mpu_instance,
};

static struct voltagedomain omap4_voltdm_iva = {
	.name = "iva",
	.scalable = true,
	.read = omap4_prm_vcvp_read,
	.write = omap4_prm_vcvp_write,
	.rmw = omap4_prm_vcvp_rmw,
	.vc = &omap4_vc_iva,
	.vfsm = &omap4_vdd_iva_vfsm,
	.vp = &omap4_vp_iva,
	.vdd = &omap4_vdd_iva_info,
	.abb = &omap4_ldo_abb_iva_instance,
};

static struct voltagedomain omap4_voltdm_core = {
	.name = "core",
	.scalable = true,
	.read = omap4_prm_vcvp_read,
	.write = omap4_prm_vcvp_write,
	.rmw = omap4_prm_vcvp_rmw,
	.vc = &omap4_vc_core,
	.vfsm = &omap4_vdd_core_vfsm,
	.vp = &omap4_vp_core,
	.vdd = &omap4_vdd_core_info,
};

static struct voltagedomain omap4_voltdm_wkup = {
	.name = "wakeup",
};

static struct voltagedomain *voltagedomains_omap4[] __initdata = {
	&omap4_voltdm_mpu,
	&omap4_voltdm_iva,
	&omap4_voltdm_core,
	&omap4_voltdm_wkup,
	NULL,
};

/*
 * Handle Mutant pre_scalar to sysclk cycles map:
 * Due to "Errata Id: i623: Retention/Sleep Voltage Transitions Ramp Time"
 * on OMAP4430 specifically, the maps is 64, 256, 512, 2048 cycles.
 * Handle this condition dynamically from version detection logic
 */
static u16 pre_scaler_to_sysclk_cycles_443x[] = {64, 256, 512, 2048};

static const char *sys_clk_name __initdata = "sys_clkin_ck";

void __init omap44xx_voltagedomains_init(void)
{
	struct voltagedomain *voltdm;
	int i;

	/*
	 * XXX Will depend on the process, validation, and binning
	 * for the currently-running IC
	 */
	if (cpu_is_omap443x()) {
		struct setup_time_ramp_params *params =
			omap4_vc_core.common->setup_time_params;

		if (params) {
			params->pre_scaler_to_sysclk_cycles =
				pre_scaler_to_sysclk_cycles_443x;
		}
		omap4_vdd_mpu_info.volt_data = omap443x_vdd_mpu_volt_data;
		omap4_vdd_iva_info.volt_data = omap443x_vdd_iva_volt_data;
		omap4_vdd_core_info.volt_data = omap443x_vdd_core_volt_data;
		omap4_vdd_mpu_info.dep_vdd_info = omap443x_vddmpu_dep_info;
		omap4_vdd_iva_info.dep_vdd_info = omap443x_vddiva_dep_info;
	} else if (cpu_is_omap446x()) {
		omap4_vdd_mpu_info.volt_data = omap446x_vdd_mpu_volt_data;
		omap4_vdd_iva_info.volt_data = omap446x_vdd_iva_volt_data;
		omap4_vdd_core_info.volt_data = omap446x_vdd_core_volt_data;
		omap4_vdd_mpu_info.dep_vdd_info = omap446x_vddmpu_dep_info;
		omap4_vdd_iva_info.dep_vdd_info = omap446x_vddiva_dep_info;
	} else {
		return;
	}

	for (i = 0; voltdm = voltagedomains_omap4[i], voltdm; i++)
		voltdm->sys_clk.name = sys_clk_name;

	voltdm_init(voltagedomains_omap4);
};
