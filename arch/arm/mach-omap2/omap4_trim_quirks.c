/*
 * OMAP LDO control and configuration
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cpu.h>

#include "control.h"

/**
 * omap4_ldo_trim_configure() - Handle device trim variance
 *
 * Few of the silicon out of the fab come out without trim parameters
 * efused in. These need some software support to allow the device to
 * function normally. Handle these silicon quirks here.
 */
static int __init omap4_ldo_trim_configure(void)
{
	u32 is_trimmed = 0;
	u32 val;

	/* Applicable only for OMAP4 */
	if (!cpu_is_omap44xx())
		return 0;

	/*
	 * Some ES2.2 efuse  values for BGAP and SLDO trim
	 * are not programmed. For these units
	 * 1. we can set overide mode for SLDO trim,
	 * and program the max multiplication factor, to ensure
	 * high enough voltage on SLDO output.
	 * 2. trim VDAC value for TV output as per recomendation
	 */

	if (omap_rev() == CHIP_IS_OMAP4430ES2_2)
		is_trimmed = omap_ctrl_readl(
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_MPU_VOLTAGE_CTRL);

	/* if not trimmed, we set force overide, insted of efuse. */
	if (!is_trimmed) {
		/* Fill in recommended values */
		val = 0x0f << OMAP4_LDOSRAMCORE_ACTMODE_VSET_OUT_SHIFT;
		val |= OMAP4_LDOSRAMCORE_ACTMODE_MUX_CTRL_MASK;
		val |= 0x1 << OMAP4_LDOSRAMCORE_RETMODE_VSET_OUT_SHIFT;
		val |= OMAP4_LDOSRAMCORE_RETMODE_MUX_CTRL_MASK;

		omap_ctrl_writel(val,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_MPU_VOLTAGE_CTRL);
		omap_ctrl_writel(val,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_CORE_VOLTAGE_CTRL);
		omap_ctrl_writel(val,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_IVA_VOLTAGE_CTRL);

		/* write value as per trim recomendation */
		val =  0xc0 << OMAP4_AVDAC_TRIM_BYTE0_SHIFT;
		val |=  0x01 << OMAP4_AVDAC_TRIM_BYTE1_SHIFT;
		omap_ctrl_writel(val,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_1);
	}

	/*
	 * For all ESx.y trimmed and untrimmed units LPDDR IO and
	 * Smart IO override efuse.
	 */
	val = OMAP4_LPDDR2_PTV_P5_MASK | OMAP4_LPDDR2_PTV_N5_MASK;
	omap_ctrl_writel(val, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_2);

	return 0;
}
arch_initcall(omap4_ldo_trim_configure);
