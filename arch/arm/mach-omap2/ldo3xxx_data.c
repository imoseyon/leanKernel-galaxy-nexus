/*
 * OMAP3xxx LDO data
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Mike Turquette <mturquette@ti.com>
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "voltage.h"
#include "ldo.h"
#include "prm2xxx_3xxx.h"
#include "prm-regbits-34xx.h"

static struct omap_ldo_abb_ops omap3630_ldo_abb_ops = {
	.check_txdone	= omap36xx_prm_abb_check_txdone,
	.clear_txdone	= omap36xx_prm_abb_clear_txdone,
};

/* WARNING: OMAP3630 as per TRM rev J, has the register names inverted */

static struct omap_ldo_abb_setup_bits omap3630_ldo_abb_setup_bits = {
	.enable_mask		= OMAP_LDO_ABB_SETUP_SR2EN_MASK,
	.active_fbb_mask	= OMAP_LDO_ABB_SETUP_ACTIVE_FBB_SEL_MASK,
	.wait_count_mask	= OMAP_LDO_ABB_SETUP_SR2_WTCNT_VALUE_MASK,
};

static struct omap_ldo_abb_ctrl_bits omap3630_ldo_abb_ctrl_bits = {
	.in_tansition_mask	= OMAP_LDO_ABB_CTRL_SR2_IN_TRANSITION_MASK,
	.status_mask		= OMAP_LDO_ABB_CTRL_SR2_STATUS_MASK,
	.opp_change_mask	= OMAP_LDO_ABB_CTRL_OPP_CHANGE_MASK,
	.opp_sel_mask		= OMAP_LDO_ABB_CTRL_OPP_SEL_MASK,
};

struct omap_ldo_abb_instance omap3630_ldo_abb_mpu_instance = {
	.prm_irq_id	= OMAP3_PRM_IRQ_VDD_MPU_ID,
	.ctrl_reg	= OMAP3_PRM_LDO_ABB_CTRL_OFFSET,
	.setup_reg	= OMAP3_PRM_LDO_ABB_SETUP_OFFSET,
	.ctrl_bits	= &omap3630_ldo_abb_ctrl_bits,
	.setup_bits	= &omap3630_ldo_abb_setup_bits,
	.ops		= &omap3630_ldo_abb_ops,

	.settling_time	= 30,
	.cycle_rate	= 8,
	.tranx_timeout	= OMAP_ABB_TRANXDONE_TIMEOUT_US,
};
