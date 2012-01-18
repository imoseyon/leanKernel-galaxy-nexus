/*
 * OMAP and TPS6236x specific initialization
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Vishwanath BS
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>

#include "pm.h"
#include "vc.h"
#include "mux.h"

/* Voltage limits supported */
#define MIN_VOLTAGE_TPS62360_62_UV	770000
#define MAX_VOLTAGE_TPS62360_62_UV	1400000

#define MIN_VOLTAGE_TPS62361_UV		500000
#define MAX_VOLTAGE_TPS62361_UV		1770000

#define MAX_VOLTAGE_RAMP_TPS6236X_UV	32000

/*
 * This is the voltage delta between 2 values in voltage register.
 * when switching voltage V1 to V2, TPS62361 can ramp up or down
 * initially with step sizes of 20mV with a last step of 10mV.
 * In the case of TPS6236[0|2], it is a constant 10mV steps
 * we choose the 10mV step for linearity when SR is configured.
 */
#define STEP_SIZE_TPS6236X		10000

/* I2C access parameters */
#define I2C_TPS6236X_SLAVE_ADDR		0x60

#define DEF_SET_REG(VSEL0, VSEL1)	(((VSEL1) << 1| (VSEL0) << 0) & 0x3)
#define REG_TPS6236X_SET_0		0x00
#define REG_TPS6236X_SET_1		0x01
#define REG_TPS6236X_SET_2		0x02
#define REG_TPS6236X_SET_3		0x03
#define REG_TPS6236X_CTRL		0x04
#define REG_TPS6236X_TEMP		0x05
#define REG_TPS6236X_RAMP_CTRL		0x06
#define REG_TPS6236X_CHIP_ID0		0x08
#define REG_TPS6236X_CHIP_ID1		0x09

#define MODE_TPS6236X_AUTO_PFM_PWM	0x00
#define MODE_TPS6236X_FORCE_PWM		BIT(7)

/* We use Auto PFM/PWM mode currently seems to have the best trade off */
#define VOLTAGE_PFM_MODE_VAL		MODE_TPS6236X_AUTO_PFM_PWM

#define REG_TPS6236X_RAMP_CTRL_RMP_MASK	(0x7 << 5)
#define REG_TPS6236X_RAMP_CTRL_EN_DISC	BIT(2)
#define REG_TPS6236X_RAMP_CTRL_RAMP_PFM	BIT(1)

#define REG_TPS6236X_CTRL_PD_EN		BIT(7)
#define REG_TPS6236X_CTRL_PD_VSEL0	BIT(6)
#define REG_TPS6236X_CTRL_PD_VSEL1	BIT(5)

/* TWL usage */
#define TWL6030_REG_SYSEN_CFG_GRP			0xB3
#define TWL6030_REG_SYSEN_CFG_TRANS			0xB4
#define TWL6030_REG_VCORE3_CFG_GRP			0x5E
#define TWL6030_REG_VMEM_CFG_GRP			0x64
#define TWL6030_REG_MSK_TRANSITION			0x20
#define TWL6030_BIT_APE_GRP				BIT(0)
#define TWL6030_BIT_CON_GRP				BIT(1)
#define TWL6030_BIT_MOD_GRP				BIT(2)
#define TWL6030_MSK_PREQ1				BIT(5)
#define TWL6030_MSK_SYSEN_OFF				(0x3 << 4)
#define TWL6030_MSK_SYSEN_SLEEP				(0x3 << 2)
#define TWL6030_MSK_SYSEN_ACTIVE			(0x3 << 0)

/* Voltage params of the attached device (all in uV) */
static unsigned long	voltage_min;
static unsigned long	voltage_max;

/* Which register do we use by default? */
static int __initdata	default_reg = -1;;

/* Do we need to setup internal pullups? */
static int __initdata	pd_vsel0 = -1;
static int __initdata	pd_vsel1 = -1;

static int __init _bd_setup(char *name,int gpio_vsel, int *pull, int *pd_vsel)
{
	int pull_dir;
	int r;

	if (gpio_vsel == -1) {
		if (*pull != -1) {
			*pd_vsel = (*pull == OMAP_PIN_OFF_OUTPUT_HIGH);
			*pull = *pd_vsel;
		} else {
			*pull = 0;
		}
		return 0;
	}

	/* if we have a pull gpio, with bad dir, pull low */
	if (*pull == -1 || (*pull != OMAP_PIN_OFF_OUTPUT_HIGH &&
				*pull != OMAP_PIN_OFF_OUTPUT_LOW))
		*pull = OMAP_PIN_OFF_OUTPUT_LOW;

	r = omap_mux_init_gpio(gpio_vsel, *pull);
	if (r) {
		pr_err("%s: unable to mux gpio%d=%d\n", __func__,
			gpio_vsel, r);
		goto out;
	}

	pull_dir = (*pull == OMAP_PIN_OFF_OUTPUT_HIGH);
	*pull = pull_dir;

	r = gpio_request(gpio_vsel, name);
	if (r) {
		pr_err("%s: unable to req gpio%d=%d\n", __func__,
			gpio_vsel, r);
		goto out;
	}
	r = gpio_direction_output(gpio_vsel, pull_dir);
	if (r) {
		pr_err("%s: unable to pull[%d] gpio%d=%d\n", __func__,
			gpio_vsel, pull_dir, r);
		gpio_free(gpio_vsel);
		goto out;
	}
out:
	return r;
}

/* Convert the ramp voltage to ramp value. */
static u8 __init tps6236x_ramp_value(unsigned long uv)
{
	if (!uv)
		return 0;

	if (uv > MAX_VOLTAGE_RAMP_TPS6236X_UV) {
		pr_err("%s: uv%ld greater than max %d\n", __func__,
			uv, MAX_VOLTAGE_RAMP_TPS6236X_UV);
		uv = MAX_VOLTAGE_RAMP_TPS6236X_UV;
	}
	return fls(MAX_VOLTAGE_RAMP_TPS6236X_UV / uv) - 1;
}

static unsigned long tps6236x_vsel_to_uv(const u8 vsel)
{
	return (voltage_min +
		(STEP_SIZE_TPS6236X * (vsel & ~VOLTAGE_PFM_MODE_VAL)));
}

static u8 tps6236x_uv_to_vsel(unsigned long uv)
{
	if (!uv)
		return 0;

	/* Round off requests to limits */
	if (uv > voltage_max) {
		pr_err("%s:Request for overvoltage[%ld] than supported[%ld]\n",
				__func__, uv, voltage_max);
		uv = voltage_max;
	}
	if (uv < voltage_min) {
		pr_err("%s:Request for undervoltage[%ld] than supported[%ld]\n",
				__func__, uv, voltage_min);
		uv = voltage_min;
	}
	return DIV_ROUND_UP(uv - voltage_min, STEP_SIZE_TPS6236X) |
			VOLTAGE_PFM_MODE_VAL;
}

static struct omap_voltdm_pmic omap4_mpu_pmic = {
	.slew_rate		= 32000,
	.step_size		= STEP_SIZE_TPS6236X,
	.on_volt		= 1375000,
	.onlp_volt		= 1375000,
	.ret_volt		= 830000,
	.off_volt		= 0,
	.volt_setup_time	= 0,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= OMAP4_VP_MPU_VLIMITTO_VDDMIN,
	.vp_vddmax		= OMAP4_VP_MPU_VLIMITTO_VDDMAX,
	.vp_timeout_us		= OMAP4_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr		= I2C_TPS6236X_SLAVE_ADDR,
	.volt_reg_addr		= REG_TPS6236X_SET_0,
	.cmd_reg_addr		= REG_TPS6236X_SET_0,
	.i2c_high_speed		= true,
	.i2c_scll_low		= 0x28,
	.i2c_scll_high		= 0x2C,
	.i2c_hscll_low		= 0x0B,
	.i2c_hscll_high		= 0x00,
	.vsel_to_uv		= tps6236x_vsel_to_uv,
	.uv_to_vsel		= tps6236x_uv_to_vsel,
};

/* As per SLVSAU9 */
static __initdata struct omap_pmic_description tps_pmic_desc = {
	.pmic_lp_tshut = 1,	/* T-OFF 1ns rounded */
	.pmic_lp_tstart = 1000,	/* T-start */
};
/**
 * _twl_i2c_rmw_u8() - Tiny helper function to do a read modify write for twl
 * @mod_no:	module number
 * @mask:	mask for the val
 * @value:	value to write
 * @reg:	register to write to
 */
static int __init _twl_i2c_rmw_u8(u8 mod_no, u8 mask, u8 value, u8 reg)
{
	int ret;
	u8 val;

	ret = twl_i2c_read_u8(mod_no, &val, reg);
	if (ret)
		goto out;

	val &= ~mask;
	val |= (value & mask);

	ret = twl_i2c_write_u8(mod_no, val, reg);
out:
	return ret;
}

/**
 * omap4_twl_tps62361_enable() - Enable tps chip
 *
 * This function enables TPS chip by associating SYSEN signal
 * to APE resource group of TWL6030.
 *
 * Returns 0 on sucess, error is returned if I2C read/write fails.
 */
static int __init omap4_twl_tps62361_enable(struct voltagedomain *voltdm)
{
	int ret = 0;
	int ret1;
	u8 val;

	/* Dont trust the bootloader. start with max, pm will set to proper */
	val = voltdm->pmic->uv_to_vsel(voltdm->pmic->vp_vddmax);
	ret = omap_vc_bypass_send_i2c_msg(voltdm, voltdm->pmic->i2c_slave_addr,
			default_reg, val);

	/* Setup Ramp */
	val = tps6236x_ramp_value(voltdm->pmic->slew_rate) <<
		__ffs(REG_TPS6236X_RAMP_CTRL_RMP_MASK);
	val &= REG_TPS6236X_RAMP_CTRL_RMP_MASK;

	/* We would like to ramp the voltage asap */
	val |= REG_TPS6236X_RAMP_CTRL_RAMP_PFM;

	/* We would like to ramp down the voltage asap as well*/
	val |= REG_TPS6236X_RAMP_CTRL_EN_DISC;

	ret = omap_vc_bypass_send_i2c_msg(voltdm, voltdm->pmic->i2c_slave_addr,
			REG_TPS6236X_RAMP_CTRL, val);
	if (ret)
		goto out;

	/* Setup the internal pulls to select if needed */
	if (pd_vsel0 != -1 || pd_vsel1 != -1) {
		val = REG_TPS6236X_CTRL_PD_EN;
		val |= (pd_vsel0) ? 0 : REG_TPS6236X_CTRL_PD_VSEL0;
		val |= (pd_vsel1) ? 0 : REG_TPS6236X_CTRL_PD_VSEL1;
		ret = omap_vc_bypass_send_i2c_msg(voltdm,
					voltdm->pmic->i2c_slave_addr,
					REG_TPS6236X_CTRL, val);
		if (ret)
			goto out;
	}

	/* Enable thermal shutdown - 0 is enable :) */
	ret = omap_vc_bypass_send_i2c_msg(voltdm,
				voltdm->pmic->i2c_slave_addr,
				REG_TPS6236X_TEMP, 0x0);
	if (ret)
		goto out;

	/* if we have to work with TWL */
#ifdef CONFIG_TWL4030_CORE

	/* unmask PREQ transition Executes ACT2SLP and SLP2ACT sleep sequence */
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_MSK_PREQ1,
				0x00, TWL6030_REG_MSK_TRANSITION);
	if (ret1) {
		pr_err("%s:Err:TWL6030: map APE PREQ1(%d)\n", __func__, ret1);
		ret = ret1;
	}

	/* Setup SYSEN to be 1 on Active and 0 for sleep and OFF states */
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_MSK_SYSEN_ACTIVE,
				0x01, TWL6030_REG_SYSEN_CFG_TRANS);
	if (ret1) {
		pr_err("%s:Err:TWL6030: sysen active(%d)\n", __func__, ret1);
		ret = ret1;
	}
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_MSK_SYSEN_SLEEP,
				0x00, TWL6030_REG_SYSEN_CFG_TRANS);
	if (ret1) {
		pr_err("%s:Err:TWL6030: sysen sleep(%d)\n", __func__, ret1);
		ret = ret1;
	}
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_MSK_SYSEN_OFF,
				0x00, TWL6030_REG_SYSEN_CFG_TRANS);
	if (ret1) {
		pr_err("%s:Err:TWL6030: sysen off(%d)\n", __func__, ret1);
		ret = ret1;
	}

	/* Map up SYSEN on TWL core to control TPS */
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_BIT_APE_GRP |
				TWL6030_BIT_MOD_GRP | TWL6030_BIT_CON_GRP,
			TWL6030_BIT_APE_GRP, TWL6030_REG_SYSEN_CFG_GRP);
	if (ret1) {
		pr_err("%s:Err:TWL6030: map APE SYEN(%d)\n", __func__, ret1);
		ret = ret1;
	}

	/* Since we dont use VCORE3, this should not be associated with APE */
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_BIT_APE_GRP,
			0x00, TWL6030_REG_VCORE3_CFG_GRP);
	if (ret1) {
		pr_err("%s:Err:TWL6030:unmap APE VCORE3(%d)\n", __func__, ret1);
		ret = ret1;
	}

	/* Since we dont use VMEM, this should not be associated with APE */
	ret1 = _twl_i2c_rmw_u8(TWL6030_MODULE_ID0, TWL6030_BIT_APE_GRP,
			0x00, TWL6030_REG_VMEM_CFG_GRP);
	if (ret1) {
		pr_err("%s:Err:TWL6030: unmap APE VMEM(%d)\n", __func__, ret1);
		ret = ret1;
	}
#endif

out:
	if (ret)
		pr_err("%s: Error enabling TPS(%d)\n", __func__, ret);

	return ret;
}

static __initdata struct omap_pmic_map omap_tps_map[] = {
	{
		.name = "mpu",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP446X),
		.pmic_data = &omap4_mpu_pmic,
		.special_action = omap4_twl_tps62361_enable,
	},
	/* Terminator */
	{ .name = NULL,.pmic_data = NULL},
};

int __init omap_tps6236x_init(void)
{
	struct omap_pmic_map *map;

	/* Without registers, I wont proceed */
	if (default_reg == -1)
		return -EINVAL;

	map = omap_tps_map;

	/* setup all the pmic's voltage addresses to the default one */
	while (map->name) {
		map->pmic_data->volt_reg_addr = default_reg;
		map->pmic_data->cmd_reg_addr = default_reg;
		map++;
	}

	return omap_pmic_register_data(omap_tps_map, &tps_pmic_desc);
}

/**
 * omap_tps6236x_board_setup() - provide the board config for TPS connect
 * @use_62361:	Do we use TPS62361 variant?
 * @gpio_vsel0:	If using GPIO to control VSEL0, provide gpio number, else -1
 * @gpio_vsel1:	If using GPIO to control VSEL1, provide gpio number, else -1
 * @pull0:	If using GPIO, provide mux mode OMAP_PIN_OFF_OUTPUT_[HIGH|LOW]
 *		else provide any internal pull required, -1 if unused.
 * @pull1:	If using GPIO, provide mux mode OMAP_PIN_OFF_OUTPUT_[HIGH|LOW]
 *		else provide any internal pull required, -1 if unused.
 *
 * TPS6236x variants of PMIC can be hooked in numerous combinations on to the
 * board. Some platforms can choose to hardwire and save on a GPIO for other
 * uses, while others may hook a single line for GPIO control and may ground
 * the other line. support these configurations.
 *
 * WARNING: for platforms using GPIO, be careful to provide MUX setting
 * considering OFF mode configuration as well.
 */
int __init omap_tps6236x_board_setup(bool use_62361, int gpio_vsel0,
		int gpio_vsel1, int pull0, int pull1)
{
	int r;

	r = _bd_setup("tps6236x_vsel0", gpio_vsel0, &pull0, &pd_vsel0);
	if (r)
		goto out;
	r = _bd_setup("tps6236x_vsel1", gpio_vsel1, &pull1, &pd_vsel1);
	if (r) {
		if (gpio_vsel0 != -1)
			gpio_free(gpio_vsel0);
		goto out;
	}

	default_reg = ((pull1 & 0x1) << 1) | (pull0 & 0x1);

	if (use_62361) {
		voltage_min = MIN_VOLTAGE_TPS62361_UV;
		voltage_max = MAX_VOLTAGE_TPS62361_UV;
	} else {
		voltage_min = MIN_VOLTAGE_TPS62360_62_UV;
		voltage_max = MAX_VOLTAGE_TPS62360_62_UV;
	}
out:
	return r;
}

int __init omap_tps6236x_update(char *name, u32 old_chip_id, u32 new_chip_id)
{
	return omap_pmic_update(omap_tps_map, name, old_chip_id, new_chip_id);
}
