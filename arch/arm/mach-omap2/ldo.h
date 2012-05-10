/*
 * OMAP3/4 LDO structure and macro definitions
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Mike Turquette <mturquette@ti.com>
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_LDO_H
#define __ARCH_ARM_MACH_OMAP2_LDO_H


/**
 * struct omap_ldo_abb_ops - ABB LDO status operation pointers
 * @check_txdone:	check if the transaction is done
 * @clear_txdone:	clear the transaction done event
 */
struct omap_ldo_abb_ops {
	u32(*check_txdone) (u8 irq_id);
	void (*clear_txdone) (u8 irq_id);
};

/*
 * NOTE: OMAP3630 calls this the ctrl register, while
 * OMAP4430, OMAP4460 is setup
 */
#define OMAP_LDO_ABB_SETUP_SR2_WTCNT_VALUE_MASK	(0xFF << 8)
#define OMAP_LDO_ABB_SETUP_ACTIVE_FBB_SEL_MASK	BIT(2)
#define OMAP_LDO_ABB_SETUP_SR2EN_MASK		BIT(0)

/**
 * struct omap_ldo_abb_setup_bits - setup register bit defns
 * @enable_mask:	SR2EN field
 * @active_fbb_mask:	ACTIVE_FBB_SEL field
 * @wait_count_mask:	SR2_WTCNT_VALUE field
 */
struct omap_ldo_abb_setup_bits {
	u32 enable_mask;
	u32 active_fbb_mask;
	/* RBB is not recommended to be used and hence not supported */
	u32 wait_count_mask;
};

/*
 * NOTE: OMAP3630 calls this the setup register, while
 * OMAP4430, OMAP4460 is ctrl
 */
#define OMAP_LDO_ABB_CTRL_SR2_IN_TRANSITION_MASK	BIT(6)
#define	OMAP_LDO_ABB_CTRL_SR2_STATUS_MASK		(0x3 << 3)
#define	OMAP_LDO_ABB_CTRL_OPP_CHANGE_MASK		BIT(2)
#define	OMAP_LDO_ABB_CTRL_OPP_SEL_MASK			(0x3 << 0)

/**
 * struct omap_ldo_abb_ctrl_bits - ctrl register bit defns
 * @in_tansition_mask:	SR2_IN_TRANSITION field
 * @status_mask:	SR2_STATUS field
 * @opp_change_mask:	OPP_CHANGE field
 * @opp_sel_mask:	OPP_SEL field
 */
struct omap_ldo_abb_ctrl_bits {
	u32 in_tansition_mask;
	u32 status_mask;
	u32 opp_change_mask;
	u32 opp_sel_mask;
};

#define OMAP_ABB_TRANXDONE_TIMEOUT_US  50

/**
 * struct omap_ldo_abb_instance - Describe an LDO instance
 * @prm_irq_id:	PRM irq id for relevant for this block
 * @ctrl_reg:	control reg offset
 * @setup_reg:	setup reg offset
 * @ctrl_bits:	pointer to control register bitfield
 * @setup_bits:	pointer to setup register bitfield
 * @settling_time:	OMAP internal settling time(in uS)
 * @cycle_rate:		Cycle rate for the IP block
 * @tranx_timeout:	timeout count in uSec
 * @ops:		operations for ldo_abb
 * @__cur_abb_type:	private structure used by the driver, donot use.
 */
struct omap_ldo_abb_instance {
	u8 prm_irq_id;

	u32 ctrl_reg;
	u32 setup_reg;
	struct omap_ldo_abb_ctrl_bits *ctrl_bits;
	struct omap_ldo_abb_setup_bits *setup_bits;

	unsigned long settling_time;
	unsigned long cycle_rate;
	unsigned int tranx_timeout;

	struct omap_ldo_abb_ops *ops;
	int __cur_abb_type;
};

extern struct omap_ldo_abb_instance omap3630_ldo_abb_mpu_instance;

extern struct omap_ldo_abb_instance omap4_ldo_abb_mpu_instance;
extern struct omap_ldo_abb_instance omap4_ldo_abb_iva_instance;

extern int omap_ldo_abb_pre_scale(struct voltagedomain *voltdm,
			   struct omap_volt_data *target_vdata);
extern int omap_ldo_abb_post_scale(struct voltagedomain *voltdm,
			   struct omap_volt_data *target_vdata);
extern void __init omap_ldo_abb_init(struct voltagedomain *voltdm);

#endif				/* __ARCH_ARM_MACH_OMAP2_LDO_H */
