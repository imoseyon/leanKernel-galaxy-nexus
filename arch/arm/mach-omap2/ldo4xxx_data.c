/*
 * OMAP4xxx LDO data
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
#include "prm44xx.h"
#include "prm-regbits-44xx.h"

static struct omap_ldo_abb_ops omap4_ldo_abb_ops = {
	.check_txdone	= omap4_prm_abb_check_txdone,
	.clear_txdone	= omap4_prm_abb_clear_txdone,
};

static struct omap_ldo_abb_setup_bits omap4_ldo_abb_setup_bits = {
	.enable_mask		= OMAP_LDO_ABB_SETUP_SR2EN_MASK,
	.active_fbb_mask	= OMAP_LDO_ABB_SETUP_ACTIVE_FBB_SEL_MASK,
	.wait_count_mask	= OMAP_LDO_ABB_SETUP_SR2_WTCNT_VALUE_MASK,
};

static struct omap_ldo_abb_ctrl_bits omap4_ldo_abb_ctrl_bits = {
	.in_tansition_mask	= OMAP_LDO_ABB_CTRL_SR2_IN_TRANSITION_MASK,
	.status_mask		= OMAP_LDO_ABB_CTRL_SR2_STATUS_MASK,
	.opp_change_mask	= OMAP_LDO_ABB_CTRL_OPP_CHANGE_MASK,
	.opp_sel_mask		= OMAP_LDO_ABB_CTRL_OPP_SEL_MASK,
};

struct omap_ldo_abb_instance omap4_ldo_abb_mpu_instance = {
	.prm_irq_id	= OMAP4_PRM_IRQ_VDD_MPU_ID,
	.ctrl_reg	= OMAP4_PRM_LDO_ABB_MPU_CTRL_OFFSET,
	.setup_reg	= OMAP4_PRM_LDO_ABB_MPU_SETUP_OFFSET,
	.ctrl_bits	= &omap4_ldo_abb_ctrl_bits,
	.setup_bits	= &omap4_ldo_abb_setup_bits,
	.ops		= &omap4_ldo_abb_ops,

	.settling_time	= 50,
	.cycle_rate	= 16,
	.tranx_timeout	= OMAP_ABB_TRANXDONE_TIMEOUT_US,
};

struct omap_ldo_abb_instance omap4_ldo_abb_iva_instance = {
	.prm_irq_id = OMAP4_PRM_IRQ_VDD_IVA_ID,
	.ctrl_reg	= OMAP4_PRM_LDO_ABB_IVA_CTRL_OFFSET,
	.setup_reg	= OMAP4_PRM_LDO_ABB_IVA_SETUP_OFFSET,
	.ctrl_bits	= &omap4_ldo_abb_ctrl_bits,
	.setup_bits	= &omap4_ldo_abb_setup_bits,
	.ops		= &omap4_ldo_abb_ops,

	.settling_time	= 50,
	.cycle_rate	= 16,
	.tranx_timeout	= OMAP_ABB_TRANXDONE_TIMEOUT_US,
};
