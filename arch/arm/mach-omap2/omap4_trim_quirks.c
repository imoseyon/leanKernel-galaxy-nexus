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
#include "pm.h"

#define OMAP4_DPLL_MPU_TRIMMED_VAL_2P4	(0x1 << 18)
#define OMAP4_DPLL_MPU_TRIMMED_VAL_3P0	(0x3 << 18)
#define OMAP4_DPLL_MPU_TRIMMED_MASK	(BIT(19) | BIT(18))

static bool bgap_trim_sw_overide;
static bool dpll_trim_override;

/**
 * omap4_ldo_trim_configure() - Handle device trim variance
 *
 * Few of the silicon out of the fab come out without trim parameters
 * efused in. These need some software support to allow the device to
 * function normally. Handle these silicon quirks here.
 */
int omap4_ldo_trim_configure(void)
{
	u32 val;

	/* if not trimmed, we set force overide, insted of efuse. */
	if (bgap_trim_sw_overide) {
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
	}

	/* For all trimmed and untrimmed write value as per recomendation */
	val =  0x10 << OMAP4_AVDAC_TRIM_BYTE0_SHIFT;
	val |=  0x01 << OMAP4_AVDAC_TRIM_BYTE1_SHIFT;
	val |=  0x4d << OMAP4_AVDAC_TRIM_BYTE2_SHIFT;
	val |=  0x1C << OMAP4_AVDAC_TRIM_BYTE3_SHIFT;
	omap4_ctrl_pad_writel(val,
		OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_1);
	/*
	 * For all ESx.y trimmed and untrimmed units LPDDR IO and
	 * Smart IO override efuse.
	 */
	val = OMAP4_LPDDR2_PTV_P5_MASK | OMAP4_LPDDR2_PTV_N5_MASK;
	omap4_ctrl_pad_writel(val, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_2);

	/* Required for DPLL_MPU to lock at 2.4 GHz */
	if (dpll_trim_override)
		omap_ctrl_writel(0x29, OMAP4_CTRL_MODULE_CORE_DPLL_NWELL_TRIM_0);

	return 0;
}

/**
 * omap4460_mpu_dpll_trim_override() - provide a selective s/w trim overide
 */
static __init void omap4460_mpu_dpll_trim_override(void)
{
	u32 val;

	val = omap_ctrl_readl(OMAP4_CTRL_MODULE_CORE_STD_FUSE_OPP_DPLL_1) &
			OMAP4_DPLL_MPU_TRIMMED_MASK;
	switch (val) {
	case OMAP4_DPLL_MPU_TRIMMED_VAL_3P0:
		/* all ok.. */
		break;
	case OMAP4_DPLL_MPU_TRIMMED_VAL_2P4:
		/* Cross check! */
		if (omap4_has_mpu_1_5ghz()) {
			WARN(1, "%s: OMAP is 1.5GHz capable, trimmed=1.2GHz!\n",
				__func__);
		}
		break;
	default:
		WARN(1, "%s: UNKNOWN TRIM:0x%08x, using s/w override\n",
			__func__, val);
		/* fall through and use override */
	case 0:
		/*
		 * For PRE_RTP devices: Not trimmed, use s/w override!
		 * We only support unto 1.2GHz with s/w override,
		 * so just give a gentle warning if higher opp is attempted
		 */
		dpll_trim_override = true;
		/* Confirm */
		if (omap4_has_mpu_1_5ghz()) {
			pr_err("%s: OMAP is 1.5GHz capable, s/w trim=1.2GHz!\n",
				__func__);
		}
		break;
	}
}

static __init int omap4_ldo_trim_init(void)
{
	u32 bgap_trimmed = 0;

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
	if (omap_rev() >= CHIP_IS_OMAP4430ES2_2)
		bgap_trimmed = omap_ctrl_readl(
			OMAP4_CTRL_MODULE_CORE_STD_FUSE_OPP_BGAP);

	bgap_trimmed &= OMAP4_STD_FUSE_OPP_BGAP_MASK_LSB;

	/* if not trimmed, we set force overide, insted of efuse. */
	if (!bgap_trimmed)
		bgap_trim_sw_overide = true;

	/* If not already trimmed, use s/w override */
	if (cpu_is_omap446x())
		omap4460_mpu_dpll_trim_override();

	return omap4_ldo_trim_configure();
}
arch_initcall(omap4_ldo_trim_init);
