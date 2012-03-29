#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/clk.h>

#include <plat/cpu.h>

#include "voltage.h"
#include "vc.h"
#include "prm-regbits-34xx.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"

#define OMAP_VC_I2C_ACK_DELAY 3

/**
 * struct omap_vc_channel_cfg - describe the cfg_channel bitfield
 * @sa: bit for slave address 
 * @rav: bit for voltage configuration register
 * @rac: bit for command configuration register	
 * @racen: enable bit for RAC
 * @cmd: bit for command value set selection
 *
 * Channel configuration bits, common for OMAP3+
 * OMAP3 register: PRM_VC_CH_CONF
 * OMAP4 register: PRM_VC_CFG_CHANNEL
 * OMAP5 register: PRM_VC_SMPS_<voltdm>_CONFIG
 */
struct omap_vc_channel_cfg {
	u8 sa;
	u8 rav;
	u8 rac;
	u8 racen;
	u8 cmd;
};

static struct omap_vc_channel_cfg vc_default_channel_cfg = {
	.sa    = BIT(0),
	.rav   = BIT(1),
	.rac   = BIT(2),
	.racen = BIT(3),
	.cmd   = BIT(4),
};

/*
 * On OMAP3+, all VC channels have the above default bitfield
 * configuration, except the OMAP4 MPU channel.  This appears
 * to be a freak accident as every other VC channel has the
 * default configuration, thus creating a mutant channel config.
 */
static struct omap_vc_channel_cfg vc_mutant_channel_cfg = {
	.sa    = BIT(0),
	.rav   = BIT(2),
	.rac   = BIT(3),
	.racen = BIT(4),
	.cmd   = BIT(1),
};

static struct omap_vc_channel_cfg *vc_cfg_bits;
#define CFG_CHANNEL_MASK 0x1f

/**
 * omap_vc_config_channel - configure VC channel to PMIC mappings
 * @voltdm: pointer to voltagdomain defining the desired VC channel
 *
 * Configures the VC channel to PMIC mappings for the following
 * PMIC settings
 * - i2c slave address (SA)
 * - voltage configuration address (RAV)
 * - command configuration address (RAC) and enable bit (RACEN)
 * - command values for ON, ONLP, RET and OFF (CMD)
 *
 * This function currently only allows flexible configuration of the
 * non-default channel.  Starting with OMAP4, there are more than 2
 * channels, with one defined as the default (on OMAP4, it's MPU.)
 * Only the non-default channel can be configured.
 */
static int omap_vc_config_channel(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc = voltdm->vc;

	/*
	 * For default channel, the only configurable bit is RACEN.
	 * All others must stay at zero (see function comment above.)
	 */
	if (vc->flags & OMAP_VC_CHANNEL_DEFAULT)
		vc->cfg_channel &= vc_cfg_bits->racen;

	voltdm->rmw(CFG_CHANNEL_MASK << vc->cfg_channel_sa_shift,
		    vc->cfg_channel << vc->cfg_channel_sa_shift,
		    vc->common->cfg_channel_reg);

	return 0;
}

/* Voltage scale and accessory APIs */
int omap_vc_pre_scale(struct voltagedomain *voltdm,
		      unsigned long target_volt,
		      struct omap_volt_data *target_v,
		      u8 *target_vsel, u8 *current_vsel)
{
	struct omap_vc_channel *vc = voltdm->vc;
	u32 vc_cmdval;

	/* Check if sufficient pmic info is available for this vdd */
	if (!voltdm->pmic) {
		pr_err("%s: Insufficient pmic info to scale the vdd_%s\n",
			__func__, voltdm->name);
		return -EINVAL;
	}

	if (!voltdm->pmic->uv_to_vsel) {
		pr_err("%s: PMIC function to convert voltage in uV to"
			"vsel not registered. Hence unable to scale voltage"
			"for vdd_%s\n", __func__, voltdm->name);
		return -ENODATA;
	}

	if (!voltdm->read || !voltdm->write) {
		pr_err("%s: No read/write API for accessing vdd_%s regs\n",
			__func__, voltdm->name);
		return -EINVAL;
	}

	*target_vsel = voltdm->pmic->uv_to_vsel(target_volt);
	*current_vsel = voltdm->read(voltdm->vp->voltage);

	/* Setting the ON voltage to the new target voltage */
	vc_cmdval = voltdm->read(vc->cmdval_reg);
	vc_cmdval &= ~vc->common->cmd_on_mask;
	vc_cmdval |= (*target_vsel << vc->common->cmd_on_shift);
	voltdm->write(vc_cmdval, vc->cmdval_reg);

	omap_vp_update_errorgain(voltdm, target_v);

	return 0;
}

/**
 * omap_vc_set_auto_trans() - set auto transition parameters for a domain
 * @voltdm:	voltage domain we are interested in
 * @flag:	which state should we program this to
 */
int omap_vc_set_auto_trans(struct voltagedomain *voltdm, u8 flag)
{
	struct omap_vc_channel *vc;
	const struct omap_vc_auto_trans *auto_trans;
	u8 val = OMAP_VC_CHANNEL_AUTO_TRANSITION_UNSUPPORTED;

	if (!voltdm) {
		pr_err("%s: NULL Voltage domain!\n", __func__);
		return -ENOENT;
	}
	vc = voltdm->vc;
	if (!vc) {
		pr_err("%s: NULL VC Voltage domain %s!\n", __func__,
		       voltdm->name);
		return -ENOENT;
	}

	auto_trans = vc->auto_trans;
	if (!auto_trans) {
		pr_debug("%s: No auto trans %s!\n", __func__, voltdm->name);
		return 0;
	}

	/* Handle value and masks per silicon data */
	switch (flag) {
	case OMAP_VC_CHANNEL_AUTO_TRANSITION_DISABLE:
		val = 0x0;
		break;
	case OMAP_VC_CHANNEL_AUTO_TRANSITION_SLEEP:
		val = auto_trans->sleep_val;
		break;
	case OMAP_VC_CHANNEL_AUTO_TRANSITION_RETENTION:
		val = auto_trans->retention_val;
		break;
	case OMAP_VC_CHANNEL_AUTO_TRANSITION_OFF:
		val = auto_trans->off_val;
		break;
	default:
		pr_err("%s: Voltdm %s invalid flag %d\n", __func__,
		       voltdm->name, flag);
		return -EINVAL;
	}

	if (val == OMAP_VC_CHANNEL_AUTO_TRANSITION_UNSUPPORTED) {
		pr_err("%s: transition to %d on %s is NOT supported\n",
		       __func__, flag, voltdm->name);
		return -EINVAL;
	}

	/* All ready - set it and move on.. */
	voltdm->rmw(vc->auto_trans_mask, val << __ffs(vc->auto_trans_mask),
		    auto_trans->reg);
	return 0;
}

void omap_vc_post_scale(struct voltagedomain *voltdm,
			unsigned long target_volt,
			struct omap_volt_data *target_vdata,
			u8 target_vsel, u8 current_vsel)
{
	struct omap_vc_channel *vc;
	u32 smps_steps = 0, smps_delay = 0;
	u8 on_vsel, onlp_vsel;
	u32 val;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return;
	}

	smps_steps = abs(target_vsel - current_vsel);
	/* SMPS slew rate / step size. 2us added as buffer. */
	smps_delay = ((smps_steps * voltdm->pmic->step_size) /
			voltdm->pmic->slew_rate) + 2;
	udelay(smps_delay);

	voltdm->curr_volt = target_vdata;

	/* Set up the on voltage for wakeup from lp and OFF */
	on_vsel = voltdm->pmic->uv_to_vsel(target_volt);
	onlp_vsel = voltdm->pmic->uv_to_vsel(target_volt);
	val = (on_vsel << vc->common->cmd_on_shift) |
	       (onlp_vsel << vc->common->cmd_onlp_shift) |
	       vc->setup_voltage_common;
	voltdm->write(val, vc->cmdval_reg);
}

static int omap_vc_bypass_send_value(struct voltagedomain *voltdm,
		struct omap_vc_channel *vc, u8 sa, u8 reg, u32 data)
{
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_valid, vc_bypass_val_reg, vc_bypass_value;

	if (IS_ERR_OR_NULL(vc->common)) {
		pr_err("%s voldm=%s bad value for vc->common\n",
				__func__, voltdm->name);
		return -EINVAL;
	}

	vc_valid = vc->common->valid;
	vc_bypass_val_reg = vc->common->bypass_val_reg;
	vc_bypass_value = (data << vc->common->data_shift) |
		(reg << vc->common->regaddr_shift) |
		(sa << vc->common->slaveaddr_shift);

	voltdm->write(vc_bypass_value, vc_bypass_val_reg);
	voltdm->write(vc_bypass_value | vc_valid, vc_bypass_val_reg);

	vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	/*
	 * Loop till the bypass command is acknowledged from the SMPS.
	 * NOTE: This is legacy code. The loop count and retry count needs
	 * to be revisited.
	 */
	while (vc_bypass_value & vc_valid) {
		loop_cnt++;

		if (retries_cnt > 10) {
			pr_warning("%s: Retry count exceeded\n", __func__);
			return -ETIMEDOUT;
		}

		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = voltdm->read(vc_bypass_val_reg);
	}

	return 0;

}

/* vc_bypass_scale_voltage - VC bypass method of voltage scaling */
int omap_vc_bypass_scale_voltage(struct voltagedomain *voltdm,
			      struct omap_volt_data *target_v)
{
	struct omap_vc_channel *vc;
	u8 target_vsel, current_vsel;
	int ret;
	unsigned long target_volt = omap_get_operation_voltage(target_v);

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return -EINVAL;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return -EINVAL;
	}

	ret = omap_vc_pre_scale(voltdm, target_volt, target_v, &target_vsel,
				&current_vsel);
	if (ret)
		return ret;

	ret = omap_vc_bypass_send_value(voltdm, vc, vc->i2c_slave_addr,
					vc->volt_reg_addr, target_vsel);
	if (ret)
		return ret;

	omap_vc_post_scale(voltdm, target_volt, target_v, target_vsel,
			current_vsel);
	return 0;
}

/**
 * omap_vc_bypass_send_i2c_msg() - Function to control PMIC registers over SRI2C
 * @voltdm:	voltage domain
 * @slave_addr:	slave address of the device.
 * @reg_addr:	register address to access
 * @data:	what do we want to write there
 *
 * Many simpler PMICs with a single I2C interface still have configuration
 * registers that may need population. Typical being slew rate configurations
 * thermal shutdown configuration etc. When these PMICs are hooked on I2C_SR,
 * this function allows these configuration registers to be accessed.
 *
 * WARNING: Though this could be used for voltage register configurations over
 * I2C_SR, DONOT use it for that purpose, all the Voltage controller's internal
 * information is bypassed using this function and must be used judiciously.
 */
int omap_vc_bypass_send_i2c_msg(struct voltagedomain *voltdm, u8 slave_addr,
				u8 reg_addr, u8 data)
{
	struct omap_vc_channel *vc;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s bad voldm\n", __func__);
		return -EINVAL;
	}

	vc = voltdm->vc;
	if (IS_ERR_OR_NULL(vc)) {
		pr_err("%s voldm=%s bad vc\n", __func__, voltdm->name);
		return -EINVAL;
	}

	return omap_vc_bypass_send_value(voltdm, vc, slave_addr,
					reg_addr, data);
}

static void __init omap3_vfsm_init(struct voltagedomain *voltdm)
{
	/*
	 * Voltage Manager FSM parameters init
	 * XXX This data should be passed in from the board file
	 */
	voltdm->write(OMAP3_CLKSETUP, OMAP3_PRM_CLKSETUP_OFFSET);
	voltdm->write(OMAP3_VOLTOFFSET, OMAP3_PRM_VOLTOFFSET_OFFSET);
	voltdm->write(OMAP3_VOLTSETUP2, OMAP3_PRM_VOLTSETUP2_OFFSET);
}

static void __init omap3_vc_init_channel(struct voltagedomain *voltdm)
{
	static bool is_initialized;

	if (is_initialized)
		return;

	omap3_vfsm_init(voltdm);

	is_initialized = true;
}


/* OMAP4 specific voltage init functions */
static void __init omap4_vc_init_channel(struct voltagedomain *voltdm)
{
	static bool is_initialized;
	struct omap_voltdm_pmic *pmic = voltdm->pmic;
	u32 vc_val = 0;

	if (is_initialized)
		return;

	if (pmic->i2c_high_speed) {
		vc_val |= pmic->i2c_hscll_low << OMAP4430_HSCLL_SHIFT;
		vc_val |= pmic->i2c_hscll_high << OMAP4430_HSCLH_SHIFT;
	}

	vc_val |= pmic->i2c_scll_low << OMAP4430_SCLL_SHIFT;
	vc_val |= pmic->i2c_scll_high << OMAP4430_SCLH_SHIFT;

	if (vc_val)
		voltdm->write(vc_val, OMAP4_PRM_VC_CFG_I2C_CLK_OFFSET);

	is_initialized = true;
}

/**
 * omap_vc_i2c_init - initialize I2C interface to PMIC
 * @voltdm: voltage domain containing VC data
 *
 * Use PMIC supplied seetings for I2C high-speed mode and
 * master code (if set) and program the VC I2C configuration
 * register.
 *
 * The VC I2C configuration is common to all VC channels,
 * so this function only configures I2C for the first VC
 * channel registers.  All other VC channels will use the
 * same configuration.
 */
static void __init omap_vc_i2c_init(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc = voltdm->vc;
	static bool initialized;
	static bool i2c_high_speed;
	u8 mcode;

	if (initialized) {
		if (voltdm->pmic->i2c_high_speed != i2c_high_speed)
			pr_warn("%s: I2C config for all channels must match.",
				__func__);
		return;
	}

	i2c_high_speed = voltdm->pmic->i2c_high_speed;
	if (i2c_high_speed)
		voltdm->rmw(vc->common->i2c_cfg_hsen_mask,
			    vc->common->i2c_cfg_hsen_mask,
			    vc->common->i2c_cfg_reg);

	mcode = voltdm->pmic->i2c_mcode;
	if (mcode)
		voltdm->rmw(vc->common->i2c_mcode_mask,
			    mcode << __ffs(vc->common->i2c_mcode_mask),
			    vc->common->i2c_cfg_reg);

	initialized = true;
}

/**
 * omap_vc_setup_lp_time() - configure the voltage ramp time for low states.
 * @voltdm:	voltagedomain we are interested in.
 * @is_retention:	Are we interested in retention or OFF?
 *
 * The ramp times are calculated based on the worst case voltage drop,
 * which is the difference of on_volt and the ret_volt. This time is used
 * for computing the duration necessary for low power states such as retention.
 */
static int __init omap_vc_setup_lp_time(struct voltagedomain *voltdm,
					bool is_retention)
{
	u32 volt_drop = 0, volt_ramptime = 0, volt_rampcount;
	u32 sys_clk_mhz = 0, sysclk_cycles = 0, max_latency_for_prescaler = 0;
	struct clk *sys_ck;
	u8 pre_scaler = 0;
	struct omap_voltdm_pmic *pmic = voltdm->pmic;
	struct omap_vc_channel *vc = voltdm->vc;
	const struct setup_time_ramp_params *params;

	params = vc->common->setup_time_params;
	/* If the VC data does not have params for us, return PMIC's value */
	if (!params)
		return pmic->volt_setup_time;
	if (!params->pre_scaler_to_sysclk_cycles_count)
		return pmic->volt_setup_time;

	/* No of sys_clk cycles for pre_scaler 0 */
	sysclk_cycles = params->pre_scaler_to_sysclk_cycles[0];

	sys_ck = clk_get(NULL, "sys_clkin_ck");
	if (IS_ERR_OR_NULL(sys_ck)) {
		WARN_ONCE(1, "%s: unable to get sys_clkin_ck (voldm %s)\n",
			__func__, voltdm->name);
		return pmic->volt_setup_time;
	}
	sys_clk_mhz = clk_get_rate(sys_ck) / 1000000;
	clk_put(sys_ck);

	/*
	 *  If we chose prescaler 0x0, then we have a limit on the maximum
	 *  latency for which we can chose a correct count. This is because,
	 *  the count field is limited to 6 bits and max value can be 63 and
	 *  for prescaler 0, ramp up/down counter is incremented every
	 *  64 system clock cycles.
	 *  for eg, max latency for prescaler for 38.4Mhz sys clk would be
	 *  105 = (63 * 64) / 38.4
	 */
	max_latency_for_prescaler = (63 * sysclk_cycles) / sys_clk_mhz;

	if (is_retention)
		volt_drop = pmic->on_volt - pmic->ret_volt;
	else
		volt_drop = pmic->on_volt;
	volt_ramptime = DIV_ROUND_UP(volt_drop, pmic->slew_rate);
	volt_ramptime += OMAP_VC_I2C_ACK_DELAY;

	/* many PMICs need additional time to switch back on */
	if (!is_retention)
		volt_ramptime += pmic->switch_on_time;

	if (volt_ramptime < max_latency_for_prescaler)
		pre_scaler = 0x0;
	else
		pre_scaler = 0x1;

	/*
	 * IF we mess up values, then try to have some form of recovery using
	 * PMIC's value.
	 */
	if (pre_scaler > params->pre_scaler_to_sysclk_cycles_count) {
		pr_err("%s: prescaler idx %d > available %d on domain %s\n",
		       __func__, pre_scaler,
		       params->pre_scaler_to_sysclk_cycles_count, voltdm->name);
		return pmic->volt_setup_time;
	}

	sysclk_cycles = params->pre_scaler_to_sysclk_cycles[pre_scaler];

	volt_rampcount = ((volt_ramptime * sys_clk_mhz) / sysclk_cycles) + 1;

	return (pre_scaler << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
	    (pre_scaler << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
	    (volt_rampcount << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
	    (volt_rampcount << OMAP4430_RAMP_UP_COUNT_SHIFT);
}

void __init omap_vc_init_channel(struct voltagedomain *voltdm)
{
	struct omap_vc_channel *vc = voltdm->vc;
	u8 on_vsel, onlp_vsel, ret_vsel, off_vsel;
	u32 val;

	if (!voltdm->pmic || !voltdm->pmic->uv_to_vsel) {
		pr_err("%s: PMIC info requried to configure vc for"
			"vdd_%s not populated.Hence cannot initialize vc\n",
			__func__, voltdm->name);
		return;
	}

	if (!voltdm->read || !voltdm->write) {
		pr_err("%s: No read/write API for accessing vdd_%s regs\n",
			__func__, voltdm->name);
		return;
	}

	vc->cfg_channel = 0;
	if (vc->flags & OMAP_VC_CHANNEL_CFG_MUTANT)
		vc_cfg_bits = &vc_mutant_channel_cfg;
	else
		vc_cfg_bits = &vc_default_channel_cfg;

	/* get PMIC/board specific settings */
	vc->i2c_slave_addr = voltdm->pmic->i2c_slave_addr;
	vc->volt_reg_addr = voltdm->pmic->volt_reg_addr;
	vc->cmd_reg_addr = voltdm->pmic->cmd_reg_addr;
	/* Calculate the RET voltage setup time and update volt_setup_time */
	vc->setup_time = omap_vc_setup_lp_time(voltdm, true);

	if ((vc->flags & OMAP_VC_CHANNEL_DEFAULT) &&
		((vc->i2c_slave_addr == USE_DEFAULT_CHANNEL_I2C_PARAM) ||
		(vc->cmd_reg_addr == USE_DEFAULT_CHANNEL_I2C_PARAM) ||
		(vc->volt_reg_addr == USE_DEFAULT_CHANNEL_I2C_PARAM))) {
		pr_err("%s: voltdm %s: default channel "
			"bad config-sa=%2x vol=%2x, cmd=%2x?\n", __func__,
			voltdm->name, vc->i2c_slave_addr, vc->volt_reg_addr,
			vc->cmd_reg_addr);
		return;
	}

	/* Configure the i2c slave address for this VC */
	if (vc->i2c_slave_addr != USE_DEFAULT_CHANNEL_I2C_PARAM) {
		voltdm->rmw(vc->smps_sa_mask,
			vc->i2c_slave_addr << __ffs(vc->smps_sa_mask),
			vc->common->smps_sa_reg);
		vc->cfg_channel |= vc_cfg_bits->sa;
	}

	/*
	 * Configure the PMIC register addresses.
	 */
	if (vc->volt_reg_addr != USE_DEFAULT_CHANNEL_I2C_PARAM) {
		voltdm->rmw(vc->smps_volra_mask,
			    vc->volt_reg_addr << __ffs(vc->smps_volra_mask),
			    vc->common->smps_volra_reg);
		vc->cfg_channel |= vc_cfg_bits->rav;
	}

	if (vc->cmd_reg_addr != USE_DEFAULT_CHANNEL_I2C_PARAM) {
		voltdm->rmw(vc->smps_cmdra_mask,
			    vc->cmd_reg_addr << __ffs(vc->smps_cmdra_mask),
			    vc->common->smps_cmdra_reg);
		vc->cfg_channel |= vc_cfg_bits->rac;
	}

	/* If voltage and cmd regs are same, we can use cmdra register */
	if (vc->volt_reg_addr == vc->cmd_reg_addr)
		vc->cfg_channel |= vc_cfg_bits->racen;

	/* Set up the on, inactive, retention and off voltage */
	on_vsel = voltdm->pmic->uv_to_vsel(voltdm->pmic->on_volt);
	onlp_vsel = voltdm->pmic->uv_to_vsel(voltdm->pmic->onlp_volt);
	ret_vsel = voltdm->pmic->uv_to_vsel(voltdm->pmic->ret_volt);
	off_vsel = voltdm->pmic->uv_to_vsel(voltdm->pmic->off_volt);
	vc->setup_voltage_common =
	       (ret_vsel << vc->common->cmd_ret_shift) |
	       (off_vsel << vc->common->cmd_off_shift);
	val = (on_vsel << vc->common->cmd_on_shift) |
	       (onlp_vsel << vc->common->cmd_onlp_shift) |
	       vc->setup_voltage_common;
	voltdm->write(val, vc->cmdval_reg);
	vc->cfg_channel |= vc_cfg_bits->cmd;

	/* Channel configuration */
	omap_vc_config_channel(voltdm);

	/* Configure the setup times */
	voltdm->rmw(voltdm->vfsm->voltsetup_mask,
		    vc->setup_time << __ffs(voltdm->vfsm->voltsetup_mask),
		    voltdm->vfsm->voltsetup_reg);
	voltdm->rmw(voltdm->vfsm->voltsetup_mask,
		    omap_vc_setup_lp_time(voltdm, false) <<
			ffs(voltdm->vfsm->voltsetup_mask),
		    voltdm->vfsm->voltsetupoff_reg);

	omap_vc_i2c_init(voltdm);

	if (cpu_is_omap34xx())
		omap3_vc_init_channel(voltdm);
	else if (cpu_is_omap44xx())
		omap4_vc_init_channel(voltdm);
}
