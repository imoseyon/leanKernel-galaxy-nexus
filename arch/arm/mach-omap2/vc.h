/*
 * OMAP3/4 Voltage Controller (VC) structure and macro definitions
 *
 * Copyright (C) 2007, 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2008, 2011 Nokia Corporation
 * Kalle Jokiniemi
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_VC_H
#define __ARCH_ARM_MACH_OMAP2_VC_H

#include <linux/kernel.h>

struct voltagedomain;

/**
 * struct setup_time_ramp_params - ramp time parameters
 * @pre_scaler_to_sysclk_cycles: The array represents correlation of prescaler
 *	to the number of system clock cycles, for which rampdown counter is
 *	incremented or decremented in PRM_VOLTSETUP_XXX_RET_SLEEP registers.
 *	This is to handle variances in defined values due to conditions such
 *	as "Errata Id: i623: Retention/Sleep Voltage Transitions Ramp Time"
 * @pre_scaler_to_sysclk_cycles_count: number of entries available
 *
 * Add parameters that allow us to compute the ramp time for the device
 */
struct setup_time_ramp_params {
	u16 *pre_scaler_to_sysclk_cycles;
	u8 pre_scaler_to_sysclk_cycles_count;
};

/**
 * struct omap_vc_common - per-VC register/bitfield data
 * @cmd_on_mask: ON bitmask in PRM_VC_CMD_VAL* register
 * @valid: VALID bitmask in PRM_VC_BYPASS_VAL register
 * @smps_sa_reg: Offset of PRM_VC_SMPS_SA reg from PRM start
 * @smps_volra_reg: Offset of PRM_VC_SMPS_VOL_RA reg from PRM start
 * @bypass_val_reg: Offset of PRM_VC_BYPASS_VAL reg from PRM start
 * @data_shift: DATA field shift in PRM_VC_BYPASS_VAL register
 * @slaveaddr_shift: SLAVEADDR field shift in PRM_VC_BYPASS_VAL register
 * @regaddr_shift: REGADDR field shift in PRM_VC_BYPASS_VAL register
 * @cmd_on_shift: ON field shift in PRM_VC_CMD_VAL_* register
 * @cmd_onlp_shift: ONLP field shift in PRM_VC_CMD_VAL_* register
 * @cmd_ret_shift: RET field shift in PRM_VC_CMD_VAL_* register
 * @cmd_off_shift: OFF field shift in PRM_VC_CMD_VAL_* register
 * @i2c_cfg_reg: I2C configuration register offset
 * @i2c_cfg_hsen_mask: high-speed mode bit field mask in I2C config register
 * @i2c_mcode_mask: MCODE field mask for I2C config register
 * @setup_time_params: setup time parameters
 *
 * XXX One of cmd_on_mask and cmd_on_shift are not needed
 * XXX VALID should probably be a shift, not a mask
 */
struct omap_vc_common {
	u32 cmd_on_mask;
	u32 valid;
	u8 smps_sa_reg;
	u8 smps_volra_reg;
	u8 smps_cmdra_reg;
	u8 bypass_val_reg;
	u8 data_shift;
	u8 slaveaddr_shift;
	u8 regaddr_shift;
	u8 cmd_on_shift;
	u8 cmd_onlp_shift;
	u8 cmd_ret_shift;
	u8 cmd_off_shift;
	u8 cfg_channel_reg;
	u8 i2c_cfg_reg;
	u8 i2c_cfg_hsen_mask;
	u8 i2c_mcode_mask;
	struct setup_time_ramp_params *setup_time_params;
};

/**
 * struct omap_vc_auto_trans - describe the auto transition for the domain
 * @reg:		register to modify (usually PRM_VOLTCTRL)
 * @sleep_val:		value to set for enabling sleep transition
 * @retention_val:	value to set for enabling retention transition
 * @off_val:		value to set for enabling off transition
 */
struct omap_vc_auto_trans {
	u8 reg;
	u8 sleep_val;
	u8 retention_val;
	u8 off_val;
};

/* omap_vc_channel.flags values */
#define OMAP_VC_CHANNEL_DEFAULT BIT(0)
#define OMAP_VC_CHANNEL_CFG_MUTANT BIT(1)

/**
 * struct omap_vc_channel - VC per-instance data
 * @flags: VC channel-specific flags (optional)
 * @common: pointer to VC common data for this platform
 * @smps_sa_mask: i2c slave address bitmask in the PRM_VC_SMPS_SA register
 * @smps_volra_mask: VOLRA* bitmask in the PRM_VC_VOL_RA register
 * @auto_trans: Auto transition information
 * @auto_trans_mask: Auto transition mask for this channel
 */
struct omap_vc_channel {
	u8 flags;

	/* channel state */
	u16 i2c_slave_addr;
	u16 volt_reg_addr;
	u16 cmd_reg_addr;
	u8 cfg_channel;
	u32 setup_time;
	u32 setup_voltage_common;
	bool i2c_high_speed;

	/* register access data */
	const struct omap_vc_common *common;
	u32 smps_sa_mask;
	u32 smps_volra_mask;
	u32 smps_cmdra_mask;
	u8 cmdval_reg;
	u8 cfg_channel_sa_shift;

	const struct omap_vc_auto_trans *auto_trans;
	u32 auto_trans_mask;
};

extern struct omap_vc_channel omap3_vc_mpu;
extern struct omap_vc_channel omap3_vc_core;

extern struct omap_vc_channel omap4_vc_mpu;
extern struct omap_vc_channel omap4_vc_iva;
extern struct omap_vc_channel omap4_vc_core;

void omap_vc_init_channel(struct voltagedomain *voltdm);
int omap_vc_pre_scale(struct voltagedomain *voltdm,
		      unsigned long target_volt,
		      struct omap_volt_data *target_vdata,
		      u8 *target_vsel, u8 *current_vsel);
void omap_vc_post_scale(struct voltagedomain *voltdm,
			unsigned long target_volt,
			struct omap_volt_data *target_vdata,
			u8 target_vsel, u8 current_vsel);

/* Auto transition flags for users */
#define OMAP_VC_CHANNEL_AUTO_TRANSITION_DISABLE		0
#define OMAP_VC_CHANNEL_AUTO_TRANSITION_SLEEP		1
#define OMAP_VC_CHANNEL_AUTO_TRANSITION_RETENTION	2
#define OMAP_VC_CHANNEL_AUTO_TRANSITION_OFF		3
/* For silicon data to mark unsupported transition */
#define OMAP_VC_CHANNEL_AUTO_TRANSITION_UNSUPPORTED	0xff
int omap_vc_set_auto_trans(struct voltagedomain *voltdm, u8 flag);
int omap_vc_bypass_scale_voltage(struct voltagedomain *voltdm,
				 struct omap_volt_data *target_volt);
int omap_vc_bypass_send_i2c_msg(struct voltagedomain *voltdm,
		u8 slave_addr, u8 reg_addr, u8 data);
#endif

