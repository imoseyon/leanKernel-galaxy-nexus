/*
 * OMAP Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_VOLTAGE_H
#define __ARCH_ARM_MACH_OMAP2_VOLTAGE_H

#include <linux/notifier.h>
#include <linux/err.h>

struct omap_volt_data;

#include "vc.h"
#include "vp.h"
#include "ldo.h"

struct powerdomain;

/* XXX document */
#define VOLTSCALE_VPFORCEUPDATE		1
#define VOLTSCALE_VCBYPASS		2

/*
 * OMAP3 GENERIC setup times. Revisit to see if these needs to be
 * passed from board or PMIC file
 */
#define OMAP3_CLKSETUP		0xff
#define OMAP3_VOLTOFFSET	0xff
#define OMAP3_VOLTSETUP2	0xff

struct omap_vdd_info;

/**
 * struct omap_vfsm_instance - per-voltage manager FSM register/bitfield
 * data
 * @voltsetup_mask: SETUP_TIME* bitmask of PRM_VOLTSETUP* register(RET/SLEEP)
 * @voltsetup_reg: register offset of PRM_VOLTSETUP from PRM base(RET/SLEEP)
 * @voltsetup_shift: SETUP_TIME* field shift in the PRM_VOLTSETUP* register
 * @voltsetupoff_reg: register offset of PRM_VOLTSETUP*_OFF from PRM base
 *
 * XXX What about VOLTOFFSET/VOLTCTRL?
 * XXX It is not necessary to have both a _mask and a _shift for the same
 *     bitfield - remove one!
 */
struct omap_vfsm_instance {
	u32 voltsetup_mask;
	u8 voltsetup_reg;
	u8 voltsetup_shift;
	u8 voltsetupoff_reg;
};

/* Dynamic nominal voltage margin common for OMAP3630 and OMAP4 */
#define OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV	50000

/**
 * struct voltagedomain - omap voltage domain global structure.
 * @name: Name of the voltage domain which can be used as a unique identifier.
 * @scalable: Whether or not this voltage domain is scalable
 * @node: list_head linking all voltage domains
 * @pwrdm_node: list_head linking all powerdomains in this voltagedomain
 * @vdd: to be removed
 * @pwrdms: powerdomains in this voltagedomain
 * @scale: function used to scale the voltage of the voltagedomain
 * @curr_volt: current nominal voltage for this voltage domain
 * @change_notify_list: notifiers that need to be told on pre and post change
 */
struct voltagedomain {
	char *name;
	bool scalable;
	struct list_head node;
	struct list_head pwrdm_list;
	struct omap_vc_channel *vc;
	const struct omap_vfsm_instance *vfsm;
	struct omap_vp_instance *vp;
	struct omap_voltdm_pmic *pmic;
	struct omap_ldo_abb_instance *abb;

	/* VC/VP register access functions: SoC specific */
	u32 (*read) (u8 offset);
	void (*write) (u32 val, u8 offset);
	u32 (*rmw)(u32 mask, u32 bits, u8 offset);

	union {
		const char *name;
		u32 rate;
	} sys_clk;

	int (*scale) (struct voltagedomain *voltdm,
		      struct omap_volt_data *target_volt);
	struct omap_volt_data *curr_volt;

	struct omap_vdd_info *vdd;
	struct srcu_notifier_head change_notify_list;
	struct dentry *debug_dir;
};

/* Notifier values for voltage changes */
#define OMAP_VOLTAGE_PRECHANGE	1
#define OMAP_VOLTAGE_POSTCHANGE	2

/**
 * struct omap_voltage_notifier - notifier data that is passed along
 * @voltdm:		voltage domain for the notification
 * @target_volt:	what voltage is happening
 * @op_result:		valid only for POSTCHANGE, tells the result of
 *			the operation.
 *
 * This provides notification
 */
struct omap_voltage_notifier {
	struct voltagedomain	*voltdm;
	unsigned long		target_volt;
	int			op_result;
};

/* Flags for various ABB options */
#define OMAP_ABB_NONE		-1
#define OMAP_ABB_NOMINAL_OPP	0
#define OMAP_ABB_FAST_OPP	1

/**
 * struct omap_volt_data - Omap voltage specific data.
 * @voltage_nominal:	The possible voltage value in uV
 * @voltage_calibrated:	The Calibrated voltage value in uV
 * @voltage_dynamic_nominal:	The run time optimized nominal voltage for
 *			the device. Dynamic nominal is the nominal voltage
 *			specialized for that OPP on the device in uV.
 * @volt_margin:	Additional sofware margin in uV to add to OPP calibrated
 *			voltage
 * @sr_efuse_offs:	The offset of the efuse register(from system
 *			control module base address) from where to read
 *			the n-target value for the smartreflex module.
 * @sr_errminlimit:	Error min limit value for smartreflex. This value
 *			differs at differnet opp and thus is linked
 *			with voltage.
 * @vp_errorgain:	Error gain value for the voltage processor. This
 *			field also differs according to the voltage/opp.
 * @abb_type:		Either OMAP_ABB_NONE - which implies that there is no
 *			usage of ABB; OMAP_ABB_NOMINAL_OPP - which bypasses ABB
 *			LDO; or OMAP_ABB_FAST_OPP, which enables Forward-Body
 *			Bias.
 */
struct omap_volt_data {
	u32	volt_nominal;
	u32	volt_calibrated;
	u32	volt_dynamic_nominal;
	u32	volt_margin;
	u32	sr_efuse_offs;
	u8	sr_errminlimit;
	u8	vp_errgain;
	int	abb_type;
};

/*
 * Introduced in OMAP4, is a concept of a default channel - in OMAP4, this
 * channel is MPU, all other domains such as IVA/CORE, could optionally
 * link their i2c reg configuration to use MPU channel's configuration if
 * required. To do this, mark in the PMIC structure's
 * i2c_slave_addr, volt_reg_addr,cmd_reg_addr with this macro.
 */
#define USE_DEFAULT_CHANNEL_I2C_PARAM  0x8000

/* Min and max voltages from OMAP perspective */
#define OMAP3430_VP1_VLIMITTO_VDDMIN	850000
#define OMAP3430_VP1_VLIMITTO_VDDMAX	1425000
#define OMAP3430_VP2_VLIMITTO_VDDMIN	900000
#define OMAP3430_VP2_VLIMITTO_VDDMAX	1150000

#define OMAP3630_VP1_VLIMITTO_VDDMIN	900000
#define OMAP3630_VP1_VLIMITTO_VDDMAX	1350000
#define OMAP3630_VP2_VLIMITTO_VDDMIN	900000
#define OMAP3630_VP2_VLIMITTO_VDDMAX	1200000

#define OMAP4_VP_MPU_VLIMITTO_VDDMIN	830000
#define OMAP4_VP_MPU_VLIMITTO_VDDMAX	1450000
#define OMAP4_VP_IVA_VLIMITTO_VDDMIN	830000
#define OMAP4_VP_IVA_VLIMITTO_VDDMAX	1260000
#define OMAP4_VP_CORE_VLIMITTO_VDDMIN	830000
#define OMAP4_VP_CORE_VLIMITTO_VDDMAX	1200000

#define OMAP4_VP_CONFIG_ERROROFFSET	0x00
#define OMAP4_VP_VSTEPMIN_VSTEPMIN	0x01
#define OMAP4_VP_VSTEPMAX_VSTEPMAX	0x04
#define OMAP4_VP_VLIMITTO_TIMEOUT_US	200

/**
 * struct omap_voltdm_pmic - PMIC specific data required by voltage driver.
 * @slew_rate:	PMIC slew rate (in uv/us)
 * @step_size:	PMIC voltage step size (in uv)
 * @i2c_high_speed: whether VC uses I2C high-speed mode to PMIC
 * @i2c_mcode: master code value for I2C high-speed preamble transmission
 * @vsel_to_uv:	PMIC API to convert vsel value to actual voltage in uV.
 * @uv_to_vsel:	PMIC API to convert voltage in uV to vsel value.
 * @i2c_hscll_low: PMIC interface speed config for highspeed mode (T low)
 * @i2c_hscll_high: PMIC interface speed config for highspeed mode (T high)
 * @i2c_scll_low: PMIC interface speed config for fullspeed mode (T low)
 * @i2c_scll_high: PMIC interface speed config for fullspeed mode (T high)
 * @switch_on_time: time taken for switch on the DCDC in uSec
 */
struct omap_voltdm_pmic {
	int slew_rate;
	int step_size;
	u32 on_volt;
	u32 onlp_volt;
	u32 ret_volt;
	u32 off_volt;
	u16 volt_setup_time;
	u16 switch_on_time;
	u8 vp_erroroffset;
	u8 vp_vstepmin;
	u8 vp_vstepmax;
	u32 vp_vddmin;
	u32 vp_vddmax;
	u8 vp_timeout_us;
	u16 i2c_slave_addr;
	u16 volt_reg_addr;
	u16 cmd_reg_addr;
	bool i2c_high_speed;
	u8 i2c_hscll_low;
	u8 i2c_hscll_high;
	u8 i2c_scll_low;
	u8 i2c_scll_high;
	u8 i2c_mcode;
	unsigned long (*vsel_to_uv) (const u8 vsel);
	u8 (*uv_to_vsel) (unsigned long uV);
};

/**
 * struct omap_vdd_dep_volt - Map table for voltage dependencies
 * @main_vdd_volt	: The main vdd voltage
 * @dep_vdd_volt	: The voltage at which the dependent vdd should be
 *			  when the main vdd is at <main_vdd_volt> voltage
 *
 * Table containing the parent vdd voltage and the dependent vdd voltage
 * corresponding to it.
 */
struct omap_vdd_dep_volt {
	u32 main_vdd_volt;
	u32 dep_vdd_volt;
};

/**
 * struct omap_vdd_dep_info -  Dependent vdd info
 * @name		: Dependent vdd name
 * @_dep_voltdm		: internal structure meant to prevent multiple lookups
 * @dep_table		: Table containing the dependent vdd voltage
 *			  corresponding to every main vdd voltage.
 * @nr_dep_entries	: number of dependency voltage entries
 */
struct omap_vdd_dep_info {
	char *name;
	struct voltagedomain *_dep_voltdm;
	struct omap_vdd_dep_volt *dep_table;
	int nr_dep_entries;
};

/**
 * omap_vdd_info - Per Voltage Domain info
 *
 * @volt_data		: Array ending with a 0 terminator containing the
 *			  voltage table with distinct voltages supported
 *			  by the domain and other associated per voltage data.
 * @dep_vdd_info	: Array ending with a 0 terminator for dependency
 *			  voltage information.
 */
struct omap_vdd_info {
	struct omap_volt_data *volt_data;
	struct omap_vdd_dep_info *dep_vdd_info;
};

void omap_voltage_get_volttable(struct voltagedomain *voltdm,
		struct omap_volt_data **volt_data);
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt);
struct omap_volt_data *omap_voltage_get_curr_vdata(struct voltagedomain *voldm);
#ifdef CONFIG_PM
int omap_voltage_register_pmic(struct voltagedomain *voltdm,
			       struct omap_voltdm_pmic *pmic);
void omap_change_voltscale_method(struct voltagedomain *voltdm,
		int voltscale_method);
int omap_voltage_late_init(void);
#else
static inline int omap_voltage_register_pmic(struct voltagedomain *voltdm,
					     struct omap_voltdm_pmic *pmic)
{
	return -EINVAL;
}
static inline  void omap_change_voltscale_method(struct voltagedomain *voltdm,
		int voltscale_method) {}
static inline int omap_voltage_late_init(void)
{
	return -EINVAL;
}
#endif

extern void omap2xxx_voltagedomains_init(void);
extern void omap3xxx_voltagedomains_init(void);
extern void omap44xx_voltagedomains_init(void);

struct voltagedomain *voltdm_lookup(const char *name);
void voltdm_init(struct voltagedomain **voltdm_list);
int voltdm_add_pwrdm(struct voltagedomain *voltdm, struct powerdomain *pwrdm);
int voltdm_for_each(int (*fn)(struct voltagedomain *voltdm, void *user),
		    void *user);
int voltdm_for_each_pwrdm(struct voltagedomain *voltdm,
			  int (*fn)(struct voltagedomain *voltdm,
				    struct powerdomain *pwrdm));
int voltdm_scale(struct voltagedomain *voltdm,
		 struct omap_volt_data *target_volt);
void voltdm_reset(struct voltagedomain *voltdm);

static inline int voltdm_register_notifier(struct voltagedomain *voltdm,
						struct notifier_block *nb)
{
	return srcu_notifier_chain_register(&voltdm->change_notify_list, nb);
}

static inline int voltdm_unregister_notifier(struct voltagedomain *voltdm,
						struct notifier_block *nb)
{
	return srcu_notifier_chain_unregister(&voltdm->change_notify_list, nb);
}

/* convert volt data to the voltage for the voltage data */
static inline unsigned long omap_get_operation_voltage(
				struct omap_volt_data *vdata)
{
	if (!vdata)
		return 0;
	return (vdata->volt_calibrated) ? vdata->volt_calibrated :
		(vdata->volt_dynamic_nominal) ? vdata->volt_dynamic_nominal :
			vdata->volt_nominal;
}

/* what is my dynamic nominal? */
static inline unsigned long omap_get_dyn_nominal(struct omap_volt_data *vdata)
{
	if (IS_ERR_OR_NULL(vdata))
		return 0;
	if (vdata->volt_calibrated) {
		unsigned long v = vdata->volt_calibrated +
			OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV;
		if (v > vdata->volt_nominal)
			return vdata->volt_nominal;
		return v;
	}
	return vdata->volt_nominal;
}
static inline unsigned long omap_get_nominal_voltage(
				struct omap_volt_data *vdata)
{
	if (IS_ERR_OR_NULL(vdata))
		return 0;
	return vdata->volt_nominal;
}

int omap_voltage_calib_reset(struct voltagedomain *voltdm);

#endif
