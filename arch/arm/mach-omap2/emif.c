/*
 * OMAP4 EMIF platform driver
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 * Aneesh V <aneesh@ti.com>
 * Vibhore Vardhan <vvardhan@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <mach/emif-44xx.h>
#include <mach/emif.h>
#include <mach/lpddr2-jedec.h>
#include <mach/omap4-common.h>

#include "voltage.h"

/* Utility macro for masking and setting a field in a register/variable */
#define mask_n_set(reg, shift, msk, val) \
	(reg) = (((reg) & ~(msk))|(((val) << (shift)) & msk))

struct emif_instance {
	void __iomem *base;
	u16 irq;
	struct platform_device *pdev;
	bool ddr_refresh_disabled;
};
static struct emif_instance emif[EMIF_NUM_INSTANCES];
static struct emif_regs *emif_curr_regs[EMIF_NUM_INSTANCES];
static struct emif_regs *emif1_regs_cache[EMIF_MAX_NUM_FREQUENCIES];
static struct emif_regs *emif2_regs_cache[EMIF_MAX_NUM_FREQUENCIES];
static struct emif_device_details *emif_devices[2];
static u32 emif_temperature_level[EMIF_NUM_INSTANCES] = { SDRAM_TEMP_NOMINAL,
	SDRAM_TEMP_NOMINAL
};

static u32 emif_notify_pending;
static u32 emif_thermal_handling_pending;
static u32 T_den, T_num;

static struct omap_device_pm_latency omap_emif_latency[] = {
	[0] = {
	       .deactivate_func = omap_device_idle_hwmods,
	       .activate_func = omap_device_enable_hwmods,
	       .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	       },
};

static u32 get_temperature_level(u32 emif_nr);

void emif_dump(int emif_nr)
{
	void __iomem *base = emif[emif_nr].base;

	printk("EMIF%d s=0x%x is_sys=0x%x is_ll=0x%x temp=0x%02x\n",
	       emif_nr + 1,
	       __raw_readl(base + OMAP44XX_EMIF_STATUS),
	       __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_SYS),
	       __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_LL),
	       get_temperature_level(emif_nr));
}

static void do_cancel_out(u32 *num, u32 *den, u32 factor)
{
	while (1) {
		if (((*num) / factor * factor == (*num)) &&
		    ((*den) / factor * factor == (*den))) {
			(*num) /= factor;
			(*den) /= factor;
		} else
			break;
	}
}

static void cancel_out(u32 *num, u32 *den)
{
	do_cancel_out(num, den, 2);
	do_cancel_out(num, den, 3);
	do_cancel_out(num, den, 5);
	do_cancel_out(num, den, 7);
	do_cancel_out(num, den, 11);
	do_cancel_out(num, den, 13);
	do_cancel_out(num, den, 17);
}

/*
 * Get the period in ns (in fraction form) for a given frequency:
 * Getting it in fraction form is for better accuracy in integer arithmetics
 * freq_hz - input: frequency in Hertz
 * den_limit - input: upper limit for denominator. see the description of
 *		EMIF_PERIOD_DEN_LIMIT for more details
 * period_den - output: pointer to denominator of period in ns
 * period_num - output: pointer to numerator of period in ns
 */
static void get_period(u32 freq_hz, u32 den_limit, u32 *period_num,
		       u32 *period_den)
{
	*period_num = 1000000000;	/* 10^9 to convert the period to 'ns' */
	*period_den = freq_hz;
	cancel_out(period_num, period_den);
	/* make sure den <= den_limit at the cost of some accuracy */
	while ((*period_den) > den_limit) {
		*period_num /= 2;
		*period_den /= 2;
	}
}

/*
 * Calculate the period of DDR clock from frequency value and set the
 * denominator and numerator in global variables for easy access later
 */
static void set_ddr_clk_period(u32 freq)
{
	get_period(freq, EMIF_PERIOD_DEN_LIMIT, &T_num, &T_den);
}

/*
 * Convert time in nano seconds to number of cycles of DDR clock
 */
static u32 ns_2_cycles(u32 ns)
{
	return ((ns * T_den) + T_num - 1) / T_num;
}

/*
 * ns_2_cycles with the difference that the time passed is 2 times the actual
 * value(to avoid fractions). The cycles returned is for the original value of
 * the timing parameter
 */
static u32 ns_x2_2_cycles(u32 ns)
{
	return ((ns * T_den) + T_num * 2 - 1) / (T_num * 2);
}

/*
 * Find addressing table index based on the device's type(S2 or S4) and
 * density
 */
static s8 addressing_table_index(u8 type, u8 density, u8 width)
{
	u8 index;
	if (unlikely((density > LPDDR2_DENSITY_8Gb) ||
		     (width == LPDDR2_IO_WIDTH_8)))
		return -1;

	/*
	 * Look at the way ADDR_TABLE_INDEX* values have been defined
	 * in emif.h compared to LPDDR2_DENSITY_* values
	 * The table is layed out in the increasing order of density
	 * (ignoring type). The exceptions 1GS2 and 2GS2 have been placed
	 * at the end
	 */
	if ((type == LPDDR2_TYPE_S2) && (density == LPDDR2_DENSITY_1Gb))
		index = ADDR_TABLE_INDEX1GS2;
	else if ((type == LPDDR2_TYPE_S2) && (density == LPDDR2_DENSITY_2Gb))
		index = ADDR_TABLE_INDEX2GS2;
	else
		index = density;

	pr_debug("emif: addressing table index %d", index);

	return index;
}

/*
 * Find the the right timing table from the array of timing
 * tables of the device using DDR clock frequency
 */
static const struct lpddr2_timings *get_timings_table(
		const struct lpddr2_timings * const *device_timings, u32 freq)
{
	u32 i, temp, freq_nearest;
	const struct lpddr2_timings *timings = NULL;

	emif_assert(freq <= MAX_LPDDR2_FREQ);
	emif_assert(device_timings);

	/*
	 * Start with the maximum allowed frequency - that is always safe
	 */
	freq_nearest = MAX_LPDDR2_FREQ;
	/*
	 * Find the timings table that has the max frequency value:
	 *   i.  Above or equal to the DDR frequency - safe
	 *   ii. The lowest that satisfies condition (i) - optimal
	 */
	for (i = 0; i < MAX_NUM_SPEEDBINS; i++) {
		if (device_timings[i]) {
			temp = device_timings[i]->max_freq;
			if ((temp >= freq) && (temp <= freq_nearest)) {
				freq_nearest = temp;
				timings = device_timings[i];
			}
		}
	}
	pr_debug("emif: timings table: %d", freq_nearest);
	return timings;
}

/*
 * Finds the value of emif_sdram_config_reg
 * All parameters are programmed based on the device on CS0.
 * If there is a device on CS1, it will be same as that on CS0 or
 * it will be NVM. We don't support NVM yet.
 * If cs1_device pointer is NULL it is assumed that there is no device
 * on CS1
 */
static u32 get_sdram_config_reg(const struct lpddr2_device_info *cs0_device,
				const struct lpddr2_device_info *cs1_device,
				const struct lpddr2_addressing *addressing,
				u8 RL)
{
	u32 config_reg = 0;

	mask_n_set(config_reg, OMAP44XX_REG_SDRAM_TYPE_SHIFT,
		   OMAP44XX_REG_SDRAM_TYPE_MASK, cs0_device->type + 4);

	mask_n_set(config_reg, OMAP44XX_REG_IBANK_POS_SHIFT,
		   OMAP44XX_REG_IBANK_POS_MASK,
		   EMIF_INTERLEAVING_POLICY_MAX_INTERLEAVING);

	mask_n_set(config_reg, OMAP44XX_REG_NARROW_MODE_SHIFT,
		   OMAP44XX_REG_NARROW_MODE_MASK, cs0_device->io_width);

	mask_n_set(config_reg, OMAP44XX_REG_CL_SHIFT, OMAP44XX_REG_CL_MASK, RL);

	mask_n_set(config_reg, OMAP44XX_REG_ROWSIZE_SHIFT,
		   OMAP44XX_REG_ROWSIZE_MASK,
		   addressing->row_sz[cs0_device->io_width]);

	mask_n_set(config_reg, OMAP44XX_REG_IBANK_SHIFT,
		   OMAP44XX_REG_IBANK_MASK, addressing->num_banks);

	mask_n_set(config_reg, OMAP44XX_REG_EBANK_SHIFT,
		   OMAP44XX_REG_EBANK_MASK,
		   (cs1_device ? EBANK_CS1_EN : EBANK_CS1_DIS));

	mask_n_set(config_reg, OMAP44XX_REG_PAGESIZE_SHIFT,
		   OMAP44XX_REG_PAGESIZE_MASK,
		   addressing->col_sz[cs0_device->io_width]);

	return config_reg;
}

static u32 get_sdram_ref_ctrl(u32 freq,
			      const struct lpddr2_addressing *addressing)
{
	u32 ref_ctrl = 0, val = 0, freq_khz;
	freq_khz = freq / 1000;
	/*
	 * refresh rate to be set is 'tREFI * freq in MHz
	 * division by 10000 to account for khz and x10 in t_REFI_us_x10
	 */
	val = addressing->t_REFI_us_x10 * freq_khz / 10000;
	mask_n_set(ref_ctrl, OMAP44XX_REG_REFRESH_RATE_SHIFT,
		   OMAP44XX_REG_REFRESH_RATE_MASK, val);

	/* enable refresh */
	mask_n_set(ref_ctrl, OMAP44XX_REG_INITREF_DIS_SHIFT,
		   OMAP44XX_REG_INITREF_DIS_MASK, 1);
	return ref_ctrl;
}

static u32 get_sdram_tim_1_reg(const struct lpddr2_timings *timings,
			       const struct lpddr2_min_tck *min_tck,
			       const struct lpddr2_addressing *addressing)
{
	u32 tim1 = 0, val = 0;
	val = max(min_tck->tWTR, ns_x2_2_cycles(timings->tWTRx2)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_WTR_SHIFT, OMAP44XX_REG_T_WTR_MASK,
		   val);

	if (addressing->num_banks == BANKS8)
		val = (timings->tFAW * T_den + 4 * T_num - 1) / (4 * T_num) - 1;
	else
		val = max(min_tck->tRRD, ns_2_cycles(timings->tRRD)) - 1;

	mask_n_set(tim1, OMAP44XX_REG_T_RRD_SHIFT, OMAP44XX_REG_T_RRD_MASK,
		   val);

	val = ns_2_cycles(timings->tRASmin + timings->tRPab) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RC_SHIFT, OMAP44XX_REG_T_RC_MASK, val);

	val = max(min_tck->tRAS_MIN, ns_2_cycles(timings->tRASmin)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RAS_SHIFT, OMAP44XX_REG_T_RAS_MASK,
		   val);

	val = max(min_tck->tWR, ns_2_cycles(timings->tWR)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_WR_SHIFT, OMAP44XX_REG_T_WR_MASK, val);

	val = max(min_tck->tRCD, ns_2_cycles(timings->tRCD)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RCD_SHIFT, OMAP44XX_REG_T_RCD_MASK,
		   val);
	val = max(min_tck->tRP_AB, ns_2_cycles(timings->tRPab)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RP_SHIFT, OMAP44XX_REG_T_RP_MASK, val);

	return tim1;
}

/*
 * Finds the de-rated value for EMIF_SDRAM_TIM1 register
 * All the de-rated timings are limited to this register
 * Adds 2ns instead of 1.875ns to the affected timings as
 * we can not use float.
 */
static u32 get_sdram_tim_1_reg_derated(const struct lpddr2_timings *timings,
				       const struct lpddr2_min_tck *min_tck,
				       const struct lpddr2_addressing
				       *addressing)
{
	u32 tim1 = 0, val = 0;
	val = max(min_tck->tWTR, ns_x2_2_cycles(timings->tWTRx2)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_WTR_SHIFT, OMAP44XX_REG_T_WTR_MASK,
		   val);

	if (addressing->num_banks == BANKS8)
		/*
		 * tFAW is approximately 4 times tRRD. So add 1.875*4 = 7.5 ~ 8
		 * to tFAW for de-rating
		 */
		val = ((timings->tFAW + 8) * T_den + 4 * T_num - 1)
		    / (4 * T_num) - 1;
	else
		val = max(min_tck->tRRD, ns_2_cycles(timings->tRRD + 2)) - 1;

	mask_n_set(tim1, OMAP44XX_REG_T_RRD_SHIFT, OMAP44XX_REG_T_RRD_MASK,
		   val);

	val = ns_2_cycles(timings->tRASmin + timings->tRPab + 2) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RC_SHIFT, OMAP44XX_REG_T_RC_MASK, val);

	val = max(min_tck->tRAS_MIN, ns_2_cycles(timings->tRASmin + 2)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RAS_SHIFT, OMAP44XX_REG_T_RAS_MASK,
		   val);

	val = max(min_tck->tWR, ns_2_cycles(timings->tWR)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_WR_SHIFT, OMAP44XX_REG_T_WR_MASK, val);

	val = max(min_tck->tRCD, ns_2_cycles(timings->tRCD + 2)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RCD_SHIFT, OMAP44XX_REG_T_RCD_MASK,
		   val);
	val = max(min_tck->tRP_AB, ns_2_cycles(timings->tRPab + 2)) - 1;
	mask_n_set(tim1, OMAP44XX_REG_T_RP_SHIFT, OMAP44XX_REG_T_RP_MASK, val);

	return tim1;
}

static u32 get_sdram_tim_2_reg(const struct lpddr2_timings *timings,
			       const struct lpddr2_min_tck *min_tck)
{
	u32 tim2 = 0, val = 0;
	val = max(min_tck->tCKE, timings->tCKE) - 1;
	mask_n_set(tim2, OMAP44XX_REG_T_CKE_SHIFT, OMAP44XX_REG_T_CKE_MASK,
		   val);

	val = max(min_tck->tRTP, ns_x2_2_cycles(timings->tRTPx2)) - 1;
	mask_n_set(tim2, OMAP44XX_REG_T_RTP_SHIFT, OMAP44XX_REG_T_RTP_MASK,
		   val);

	/*
	 * tXSRD = tRFCab + 10 ns. XSRD and XSNR should have the
	 * same value
	 */
	val = ns_2_cycles(timings->tXSR) - 1;
	mask_n_set(tim2, OMAP44XX_REG_T_XSRD_SHIFT, OMAP44XX_REG_T_XSRD_MASK,
		   val);
	mask_n_set(tim2, OMAP44XX_REG_T_XSNR_SHIFT, OMAP44XX_REG_T_XSNR_MASK,
		   val);

	val = max(min_tck->tXP, ns_x2_2_cycles(timings->tXPx2)) - 1;
	mask_n_set(tim2, OMAP44XX_REG_T_XP_SHIFT, OMAP44XX_REG_T_XP_MASK, val);

	return tim2;
}

static u32 get_sdram_tim_3_reg(const struct lpddr2_timings *timings,
			       const struct lpddr2_min_tck *min_tck,
			       const struct lpddr2_addressing *addressing)
{
	u32 tim3 = 0, val = 0;
	val = min(timings->tRASmax * 10 / addressing->t_REFI_us_x10 - 1, 0xF);
	mask_n_set(tim3, OMAP44XX_REG_T_RAS_MAX_SHIFT,
		   OMAP44XX_REG_T_RAS_MAX_MASK, val);

	val = ns_2_cycles(timings->tRFCab) - 1;
	mask_n_set(tim3, OMAP44XX_REG_T_RFC_SHIFT, OMAP44XX_REG_T_RFC_MASK,
		   val);

	val = ns_x2_2_cycles(timings->tDQSCKMAXx2) - 1;
	mask_n_set(tim3, OMAP44XX_REG_T_TDQSCKMAX_SHIFT,
		   OMAP44XX_REG_T_TDQSCKMAX_MASK, val);

	val = ns_2_cycles(timings->tZQCS) - 1;
	mask_n_set(tim3, OMAP44XX_REG_ZQ_ZQCS_SHIFT,
		   OMAP44XX_REG_ZQ_ZQCS_MASK, val);

	val = max(min_tck->tCKESR, ns_2_cycles(timings->tCKESR)) - 1;
	mask_n_set(tim3, OMAP44XX_REG_T_CKESR_SHIFT,
		   OMAP44XX_REG_T_CKESR_MASK, val);

	return tim3;
}

static u32 get_zq_config_reg(const struct lpddr2_device_info *cs1_device,
			     const struct lpddr2_addressing *addressing,
			     bool volt_ramp)
{
	u32 zq = 0, val = 0;
	if (volt_ramp)
		val =
		    EMIF_ZQCS_INTERVAL_DVFS_IN_US * 10 /
		    addressing->t_REFI_us_x10;
	else
		val =
		    EMIF_ZQCS_INTERVAL_NORMAL_IN_US * 10 /
		    addressing->t_REFI_us_x10;
	mask_n_set(zq, OMAP44XX_REG_ZQ_REFINTERVAL_SHIFT,
		   OMAP44XX_REG_ZQ_REFINTERVAL_MASK, val);

	mask_n_set(zq, OMAP44XX_REG_ZQ_ZQCL_MULT_SHIFT,
		   OMAP44XX_REG_ZQ_ZQCL_MULT_MASK, REG_ZQ_ZQCL_MULT - 1);

	mask_n_set(zq, OMAP44XX_REG_ZQ_ZQINIT_MULT_SHIFT,
		   OMAP44XX_REG_ZQ_ZQINIT_MULT_MASK, REG_ZQ_ZQINIT_MULT - 1);

	mask_n_set(zq, OMAP44XX_REG_ZQ_SFEXITEN_SHIFT,
		   OMAP44XX_REG_ZQ_SFEXITEN_MASK, REG_ZQ_SFEXITEN_ENABLE);

	/*
	 * Assuming that two chipselects have a single calibration resistor
	 * If there are indeed two calibration resistors, then this flag should
	 * be enabled to take advantage of dual calibration feature.
	 * This data should ideally come from board files. But considering
	 * that none of the boards today have calibration resistors per CS,
	 * it would be an unnecessary overhead.
	 */
	mask_n_set(zq, OMAP44XX_REG_ZQ_DUALCALEN_SHIFT,
		   OMAP44XX_REG_ZQ_DUALCALEN_MASK, REG_ZQ_DUALCALEN_DISABLE);

	mask_n_set(zq, OMAP44XX_REG_ZQ_CS0EN_SHIFT,
		   OMAP44XX_REG_ZQ_CS0EN_MASK, REG_ZQ_CS0EN_ENABLE);

	mask_n_set(zq, OMAP44XX_REG_ZQ_CS1EN_SHIFT,
		   OMAP44XX_REG_ZQ_CS1EN_MASK, (cs1_device ? 1 : 0));

	return zq;
}

static u32 get_temp_alert_config(const struct lpddr2_device_info *cs1_device,
				 const struct lpddr2_addressing *addressing,
				 bool is_derated)
{
	u32 alert = 0, interval;
	interval =
	    TEMP_ALERT_POLL_INTERVAL_MS * 10000 / addressing->t_REFI_us_x10;
	if (is_derated)
		interval *= 4;
	mask_n_set(alert, OMAP44XX_REG_TA_REFINTERVAL_SHIFT,
		   OMAP44XX_REG_TA_REFINTERVAL_MASK, interval);

	mask_n_set(alert, OMAP44XX_REG_TA_DEVCNT_SHIFT,
		   OMAP44XX_REG_TA_DEVCNT_MASK, TEMP_ALERT_CONFIG_DEVCT_1);

	mask_n_set(alert, OMAP44XX_REG_TA_DEVWDT_SHIFT,
		   OMAP44XX_REG_TA_DEVWDT_MASK, TEMP_ALERT_CONFIG_DEVWDT_32);

	mask_n_set(alert, OMAP44XX_REG_TA_SFEXITEN_SHIFT,
		   OMAP44XX_REG_TA_SFEXITEN_MASK, 1);

	mask_n_set(alert, OMAP44XX_REG_TA_CS0EN_SHIFT,
		   OMAP44XX_REG_TA_CS0EN_MASK, 1);

	mask_n_set(alert, OMAP44XX_REG_TA_CS1EN_SHIFT,
		   OMAP44XX_REG_TA_CS1EN_MASK, (cs1_device ? 1 : 0));

	return alert;
}

static u32 get_read_idle_ctrl_reg(bool volt_ramp)
{
	u32 idle = 0, val = 0;
	if (volt_ramp)
		val = ns_2_cycles(READ_IDLE_INTERVAL_DVFS) / 64 - 1;
	else
		/*Maximum value in normal conditions - suggested by hw team */
		val = 0x1FF;
	mask_n_set(idle, OMAP44XX_REG_READ_IDLE_INTERVAL_SHIFT,
		   OMAP44XX_REG_READ_IDLE_INTERVAL_MASK, val);

	mask_n_set(idle, OMAP44XX_REG_READ_IDLE_LEN_SHIFT,
		   OMAP44XX_REG_READ_IDLE_LEN_MASK, EMIF_REG_READ_IDLE_LEN_VAL);

	return idle;
}

static u32 get_ddr_phy_ctrl_1(u32 freq, u8 RL)
{
	u32 phy = 0, val = 0;

	mask_n_set(phy, OMAP44XX_REG_READ_LATENCY_SHIFT,
		   OMAP44XX_REG_READ_LATENCY_MASK, RL + 2);

	if (freq <= 100000000)
		val = EMIF_DLL_SLAVE_DLY_CTRL_100_MHZ_AND_LESS;
	else if (freq <= 200000000)
		val = EMIF_DLL_SLAVE_DLY_CTRL_200_MHZ;
	else
		val = EMIF_DLL_SLAVE_DLY_CTRL_400_MHZ;
	mask_n_set(phy, OMAP44XX_REG_DLL_SLAVE_DLY_CTRL_SHIFT,
		   OMAP44XX_REG_DLL_SLAVE_DLY_CTRL_MASK, val);

	/* Other fields are constant magic values. Hardcode them together */
	mask_n_set(phy, OMAP44XX_EMIF_DDR_PHY_CTRL_1_BASE_VAL_SHIFT,
		   OMAP44XX_EMIF_DDR_PHY_CTRL_1_BASE_VAL_MASK,
		   EMIF_DDR_PHY_CTRL_1_BASE_VAL);

	phy >>= OMAP44XX_REG_DDR_PHY_CTRL_1_SHIFT;

	return phy;
}

/*
 * get_lp_mode - Get the LP Mode of a EMIF instance.
 *
 * It returns the REG_LP_MODE of EMIF_PWR_MGMT_CTRL[10:8]
 * for a EMIF.
 *
 */
static u32 get_lp_mode(u32 emif_nr)
{
	u32 temp, lpmode;
	void __iomem *base = emif[emif_nr].base;

	temp = readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL);
	lpmode = (temp & OMAP44XX_REG_LP_MODE_MASK) >>
			OMAP44XX_REG_LP_MODE_SHIFT;

	return lpmode;
}

/*
 * set_lp_mode - Set the LP Mode of a EMIF instance.
 *
 * It replaces the REG_LP_MODE of EMIF_PWR_MGMT_CTRL[10:8]
 * with the new value for a EMIF.
 *
 */
static void set_lp_mode(u32 emif_nr, u32 lpmode)
{
	u32 temp;
	void __iomem *base = emif[emif_nr].base;

	/* Extract current lp mode value */
	temp = readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL);

	/* Write out the new lp mode value */
	temp &= ~OMAP44XX_REG_LP_MODE_MASK;
	temp |= lpmode << OMAP44XX_REG_LP_MODE_SHIFT;
	writel(temp, base + OMAP44XX_EMIF_PWR_MGMT_CTRL);

}

/*
 * Get the temperature level of the EMIF instance:
 * Reads the MR4 register of attached SDRAM parts to find out the temperature
 * level. If there are two parts attached(one on each CS), then the temperature
 * level for the EMIF instance is the higher of the two temperatures.
 */
static u32 get_temperature_level(u32 emif_nr)
{
	u32 temp, tmp_temperature_level;
	bool cs1_used;
	void __iomem *base;

	base = emif[emif_nr].base;

	temp = __raw_readl(base + OMAP44XX_EMIF_SDRAM_CONFIG);
	cs1_used = (temp & OMAP44XX_REG_EBANK_MASK) ? true : false;

	/* Read mode register 4 */
	__raw_writel(LPDDR2_MR4, base + OMAP44XX_EMIF_LPDDR2_MODE_REG_CFG);
	tmp_temperature_level = __raw_readl(base +
					    OMAP44XX_EMIF_LPDDR2_MODE_REG_DATA);

	tmp_temperature_level = (tmp_temperature_level &
				 MR4_SDRAM_REF_RATE_MASK) >>
	    MR4_SDRAM_REF_RATE_SHIFT;

	if (cs1_used) {
		__raw_writel(LPDDR2_MR4 | OMAP44XX_REG_CS_MASK,
			     base + OMAP44XX_EMIF_LPDDR2_MODE_REG_CFG);
		temp = __raw_readl(base + OMAP44XX_EMIF_LPDDR2_MODE_REG_DATA);
		temp = (temp & MR4_SDRAM_REF_RATE_MASK)
		    >> MR4_SDRAM_REF_RATE_SHIFT;
		tmp_temperature_level = max(temp, tmp_temperature_level);
	}

	/* treat everything less than nominal(3) in MR4 as nominal */
	if (unlikely(tmp_temperature_level < SDRAM_TEMP_NOMINAL))
		tmp_temperature_level = SDRAM_TEMP_NOMINAL;

	/* if we get reserved value in MR4 persist with the existing value */
	if (unlikely(tmp_temperature_level == SDRAM_TEMP_RESERVED_4))
		tmp_temperature_level = emif_temperature_level[emif_nr];

	return tmp_temperature_level;
}

/*
 * Program EMIF shadow registers:
 * Sets the shadow registers using pre-caulated register values
 * When volt_state indicates that this function is called just before
 * a voltage scaling, set only the registers relevant for voltage scaling
 * Otherwise, set all the registers relevant for a frequency change
 */
static void setup_registers(u32 emif_nr, struct emif_regs *regs, u32 volt_state)
{
	u32 temp,read_idle;
	void __iomem *base = emif[emif_nr].base;

	__raw_writel(regs->ref_ctrl, base + OMAP44XX_EMIF_SDRAM_REF_CTRL_SHDW);

	__raw_writel(regs->sdram_tim2, base + OMAP44XX_EMIF_SDRAM_TIM_2_SHDW);
	__raw_writel(regs->sdram_tim3, base + OMAP44XX_EMIF_SDRAM_TIM_3_SHDW);
	/*
	 * Do not change the RL part in PHY CTRL register
	 * RL is not changed during DVFS
	 */
	temp = __raw_readl(base + OMAP44XX_EMIF_DDR_PHY_CTRL_1_SHDW);
	mask_n_set(temp, OMAP44XX_REG_DDR_PHY_CTRL_1_SHDW_SHIFT,
		   OMAP44XX_REG_DDR_PHY_CTRL_1_SHDW_MASK,
		   regs->emif_ddr_phy_ctlr_1_final);
	__raw_writel(temp, base + OMAP44XX_EMIF_DDR_PHY_CTRL_1_SHDW);

	__raw_writel(regs->temp_alert_config,
		     base + OMAP44XX_EMIF_TEMP_ALERT_CONFIG);

	/*
	 * When voltage ramps forced read idle should
	 * happen more often.
	 */
	if (volt_state == LPDDR2_VOLTAGE_RAMPING)
		read_idle = regs->read_idle_ctrl_volt_ramp;
	else
		read_idle = regs->read_idle_ctrl_normal;
	__raw_writel(read_idle, base + OMAP44XX_EMIF_READ_IDLE_CTRL_SHDW);

	/*
	 * Reading back the last written register to ensure all writes are
	 * complete
	 */
	temp = __raw_readl(base + OMAP44XX_EMIF_READ_IDLE_CTRL_SHDW);
}

/*
 * setup_temperature_sensitive_regs() - set the timings for temperature
 * sensitive registers. This happens once at initialization time based
 * on the temperature at boot time and subsequently based on the temperature
 * alert interrupt. Temperature alert can happen when the temperature
 * increases or drops. So this function can have the effect of either
 * derating the timings or going back to nominal values.
 */
static void setup_temperature_sensitive_regs(u32 emif_nr,
					     struct emif_regs *regs)
{
	u32 tim1, ref_ctrl, temp_alert_cfg;
	void __iomem *base = emif[emif_nr].base;
	u32 temperature = emif_temperature_level[emif_nr];

	if (unlikely(temperature == SDRAM_TEMP_HIGH_DERATE_REFRESH)) {
		tim1 = regs->sdram_tim1;
		ref_ctrl = regs->ref_ctrl_derated;
		temp_alert_cfg = regs->temp_alert_config_derated;
	} else if (unlikely(temperature ==
			    SDRAM_TEMP_HIGH_DERATE_REFRESH_AND_TIMINGS)) {
		tim1 = regs->sdram_tim1_derated;
		ref_ctrl = regs->ref_ctrl_derated;
		temp_alert_cfg = regs->temp_alert_config_derated;
	} else {
		/*
		 * Nominal timings - you may switch back to the
		 * nominal timings if the temperature falls
		 */
		tim1 = regs->sdram_tim1;
		ref_ctrl = regs->ref_ctrl;
		temp_alert_cfg = regs->temp_alert_config;
	}

	__raw_writel(tim1, base + OMAP44XX_EMIF_SDRAM_TIM_1_SHDW);
	__raw_writel(temp_alert_cfg, base + OMAP44XX_EMIF_TEMP_ALERT_CONFIG);
	__raw_writel(ref_ctrl, base + OMAP44XX_EMIF_SDRAM_REF_CTRL_SHDW);

	/* read back last written register to ensure write is complete */
	__raw_readl(base + OMAP44XX_EMIF_SDRAM_REF_CTRL_SHDW);
}

static irqreturn_t handle_temp_alert(void __iomem *base, u32 emif_nr)
{
	u32 old_temperature_level;
	old_temperature_level = emif_temperature_level[emif_nr];
	emif_temperature_level[emif_nr] = get_temperature_level(emif_nr);

	if (unlikely(emif_temperature_level[emif_nr] == old_temperature_level))
		return IRQ_HANDLED;

	emif_notify_pending |= (1 << emif_nr);
	if (likely(emif_temperature_level[emif_nr] < old_temperature_level)) {
		/* Temperature coming down - defer handling to thread */
		emif_thermal_handling_pending |= (1 << emif_nr);
	} else if (likely(emif_temperature_level[emif_nr] !=
			  SDRAM_TEMP_VERY_HIGH_SHUTDOWN)) {
		/* Temperature is going up - handle immediately */
		setup_temperature_sensitive_regs(emif_nr,
						 emif_curr_regs[emif_nr]);
		/*
		 * EMIF de-rated timings register needs to be setup using
		 * freq update method only
		 */
		omap4_prcm_freq_update();
	}
	return IRQ_WAKE_THREAD;
}

static void setup_volt_sensitive_registers(u32 emif_nr, struct emif_regs *regs,
					   u32 volt_state)
{
	u32 read_idle;
	void __iomem *base = emif[emif_nr].base;
	/*
	 * When voltage ramps forced read idle should
	 * happen more often.
	 */
	if (volt_state == LPDDR2_VOLTAGE_RAMPING)
		read_idle = regs->read_idle_ctrl_volt_ramp;
	else
		read_idle = regs->read_idle_ctrl_normal;

	__raw_writel(read_idle, base + OMAP44XX_EMIF_READ_IDLE_CTRL_SHDW);

	/* read back last written register to ensure write is complete */
	__raw_readl(base + OMAP44XX_EMIF_READ_IDLE_CTRL_SHDW);

	return;
}

/*
 * Interrupt Handler for EMIF1 and EMIF2
 */
static irqreturn_t emif_interrupt_handler(int irq, void *dev_id)
{
	void __iomem *base;
	irqreturn_t ret = IRQ_HANDLED;
	u32 sys, ll;
	u8 emif_nr = EMIF1;

	if (emif[EMIF2].irq == irq)
		emif_nr = EMIF2;

	base = emif[emif_nr].base;

	/* Save the status and clear it */
	sys = __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_SYS);
	ll = __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_LL);
	__raw_writel(sys, base + OMAP44XX_EMIF_IRQSTATUS_SYS);
	__raw_writel(ll, base + OMAP44XX_EMIF_IRQSTATUS_LL);
	/*
	 * Handle temperature alert
	 * Temperature alert should be same for both ports
	 * So, it's enough to process it for only one of the ports
	 */
	if (sys & OMAP44XX_REG_TA_SYS_MASK)
		ret = handle_temp_alert(base, emif_nr);

	if (sys & OMAP44XX_REG_ERR_SYS_MASK)
		pr_err("EMIF: Access error from EMIF%d SYS port - %x",
		       emif_nr, sys);

	if (ll & OMAP44XX_REG_ERR_LL_MASK)
		pr_err("EMIF Error: Access error from EMIF%d LL port - %x",
		       emif_nr, ll);

	return ret;
}

static irqreturn_t emif_threaded_isr(int irq, void *dev_id)
{
	u8 emif_nr = EMIF1;
	if (emif[EMIF2].irq == irq)
		emif_nr = EMIF2;

	if (emif_thermal_handling_pending & (1 << emif_nr)) {
		setup_temperature_sensitive_regs(emif_nr,
						 emif_curr_regs[emif_nr]);
		/*
		 * EMIF de-rated timings register needs to be setup using
		 * freq update method only
		 */
		omap4_prcm_freq_update();
		/* clear the bit */
		emif_thermal_handling_pending &= ~(1 << emif_nr);
	}
	if (emif_notify_pending & (1 << emif_nr)) {
		sysfs_notify(&(emif[emif_nr].pdev->dev.kobj), NULL,
			     "temperature");
		kobject_uevent(&(emif[emif_nr].pdev->dev.kobj), KOBJ_CHANGE);
		/* clear the bit */
		emif_notify_pending &= ~(1 << emif_nr);
	}

	return IRQ_HANDLED;
}

static int __init setup_emif_interrupts(u32 emif_nr)
{
	u32 temp;
	void __iomem *base = emif[emif_nr].base;
	int r;

	/* Clear any pendining interrupts */
	__raw_writel(0xFFFFFFFF, base + OMAP44XX_EMIF_IRQSTATUS_SYS);
	__raw_writel(0xFFFFFFFF, base + OMAP44XX_EMIF_IRQSTATUS_LL);

	/* Enable the relevant interrupts for both LL and SYS */
	temp = OMAP44XX_REG_EN_TA_SYS_MASK | OMAP44XX_REG_EN_ERR_SYS_MASK;
	__raw_writel(temp, base + OMAP44XX_EMIF_IRQENABLE_SET_SYS);
	__raw_writel(temp, base + OMAP44XX_EMIF_IRQENABLE_SET_LL);

	/* Dummy read to make sure writes are complete */
	__raw_readl(base + OMAP44XX_EMIF_IRQENABLE_SET_LL);

	/* setup IRQ handlers */
	r = request_threaded_irq(emif[emif_nr].irq,
				    emif_interrupt_handler,
				    emif_threaded_isr,
				    IRQF_SHARED, emif[emif_nr].pdev->name,
				    emif[emif_nr].pdev);
	if (r) {
		pr_err("%s: Failed: request_irq emif[%d] IRQ%d:%d\n",
			__func__, emif_nr, emif[emif_nr].irq, r);
		return r;
	}

	/*
	 * Even if we fail to make the irq  wakeup capable, we are at risk only
	 * while going to suspend where the device is cooler, we might lose a
	 * bit of power due to pending interrupt preventing core from hitting
	 * low power state but we can continue to handle events in active use
	 * cases. So don't free interrupt on failure of marking wakeup capable,
	 * just warn and continue.
	 */
	if (enable_irq_wake(emif[emif_nr].irq))
		pr_err("%s: Failed: wakeupen emif[%d] IRQ%d\n", __func__,
			emif_nr, emif[emif_nr].irq);

	return 0;
}

static ssize_t emif_temperature_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	u32 temperature;
	if (dev == &(emif[EMIF1].pdev->dev))
		temperature = emif_temperature_level[EMIF1];
	else if (dev == &(emif[EMIF2].pdev->dev))
		temperature = emif_temperature_level[EMIF2];
	else
		return 0;

	return snprintf(buf, 20, "%u\n", temperature);
}
static DEVICE_ATTR(temperature, S_IRUGO, emif_temperature_show, NULL);

static int __devinit omap_emif_probe(struct platform_device *pdev)
{
	int id;
	struct resource *res;

	if (!pdev)
		return -EINVAL;

	id = pdev->id;
	emif[id].pdev = pdev;
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_err("EMIF %i Invalid IRQ resource\n", id);
		return -ENODEV;
	}

	emif[id].irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("EMIF%i Invalid mem resource\n", id);
		return -ENODEV;
	}

	emif[id].base = ioremap(res->start, SZ_1M);
	if (!emif[id].base) {
		pr_err("Could not ioremap EMIF%i\n", id);
		return -ENOMEM;
	}

	pr_info("EMIF%d is enabled with IRQ%d\n", id, emif[id].irq);

	emif[id].ddr_refresh_disabled = false;

	return 0;
}

static int emif_init(struct omap_hwmod *oh, void *user)
{
	char *name = "omap_emif";
	struct omap_device *od;
	static int id;

	od = omap_device_build(name, id, oh, NULL, 0, omap_emif_latency,
			       ARRAY_SIZE(omap_emif_latency), false);
	WARN(IS_ERR(od), "Can't build omap_device for %s:%s.\n",
	     name, oh->name);
	id++;
	return 0;

}

static void emif_calculate_regs(const struct emif_device_details *devices,
				u32 freq, struct emif_regs *regs)
{
	u32 temp;
	const struct lpddr2_addressing *addressing;
	const struct lpddr2_timings *timings;
	const struct lpddr2_min_tck *min_tck;
	const struct lpddr2_device_info *cs0_device = devices->cs0_device;
	const struct lpddr2_device_info *cs1_device = devices->cs1_device;

	emif_assert(devices);
	emif_assert(regs);
	/*
	 * You can not have a device on CS1 without one on CS0
	 * So configuring EMIF without a device on CS0 doesn't
	 * make sense
	 */
	emif_assert(cs0_device);
	emif_assert(cs0_device->type != LPDDR2_TYPE_NVM);
	/*
	 * If there is a device on CS1 it should be same type as CS0
	 * (or NVM. But NVM is not supported in this driver yet)
	 */
	emif_assert((cs1_device == NULL) ||
		    (cs1_device->type == LPDDR2_TYPE_NVM) ||
		    (cs0_device->type == cs1_device->type));
	emif_assert(freq <= MAX_LPDDR2_FREQ);

	set_ddr_clk_period(freq);

	/*
	 * The device on CS0 is used for all timing calculations
	 * There is only one set of registers for timings per EMIF. So, if the
	 * second CS(CS1) has a device, it should have the same timings as the
	 * device on CS0
	 */
	timings = get_timings_table(cs0_device->device_timings, freq);
	emif_assert(timings);
	min_tck = cs0_device->min_tck;

	temp =
	    addressing_table_index(cs0_device->type, cs0_device->density,
				   cs0_device->io_width);
	emif_assert((temp >= 0));
	addressing = &(lpddr2_jedec_addressing_table[temp]);
	emif_assert(addressing);

	regs->RL_final = timings->RL;
	/*
	 * Initial value of EMIF_SDRAM_CONFIG corresponds to the base
	 * frequency - 19.2 MHz
	 */
	regs->sdram_config_init =
	    get_sdram_config_reg(cs0_device, cs1_device, addressing,
				 RL_19_2_MHZ);

	regs->sdram_config_final = regs->sdram_config_init;
	mask_n_set(regs->sdram_config_final, OMAP44XX_REG_CL_SHIFT,
		   OMAP44XX_REG_CL_MASK, timings->RL);

	regs->ref_ctrl = get_sdram_ref_ctrl(freq, addressing);
	regs->ref_ctrl_derated = regs->ref_ctrl / 4;

	regs->sdram_tim1 = get_sdram_tim_1_reg(timings, min_tck, addressing);

	regs->sdram_tim1_derated =
	    get_sdram_tim_1_reg_derated(timings, min_tck, addressing);

	regs->sdram_tim2 = get_sdram_tim_2_reg(timings, min_tck);

	regs->sdram_tim3 = get_sdram_tim_3_reg(timings, min_tck, addressing);

	regs->read_idle_ctrl_normal =
	    get_read_idle_ctrl_reg(LPDDR2_VOLTAGE_STABLE);

	regs->read_idle_ctrl_volt_ramp =
	    get_read_idle_ctrl_reg(LPDDR2_VOLTAGE_RAMPING);

	regs->zq_config_normal =
	    get_zq_config_reg(cs1_device, addressing, LPDDR2_VOLTAGE_STABLE);

	regs->zq_config_volt_ramp =
	    get_zq_config_reg(cs1_device, addressing, LPDDR2_VOLTAGE_RAMPING);

	regs->temp_alert_config =
	    get_temp_alert_config(cs1_device, addressing, false);

	regs->temp_alert_config_derated =
	    get_temp_alert_config(cs1_device, addressing, true);

	regs->emif_ddr_phy_ctlr_1_init =
	    get_ddr_phy_ctrl_1(EMIF_FREQ_19_2_MHZ, RL_19_2_MHZ);

	regs->emif_ddr_phy_ctlr_1_final =
	    get_ddr_phy_ctrl_1(freq, regs->RL_final);

	/* save the frequency in the struct to act as a tag when cached */
	regs->freq = freq;

	pr_debug("Calculated EMIF configuration register values "
		 "for %d MHz", freq / 1000000);
	pr_debug("sdram_config_init\t\t: 0x%08x\n", regs->sdram_config_init);
	pr_debug("sdram_config_final\t\t: 0x%08x\n", regs->sdram_config_final);
	pr_debug("sdram_ref_ctrl\t\t: 0x%08x\n", regs->ref_ctrl);
	pr_debug("sdram_ref_ctrl_derated\t\t: 0x%08x\n",
		 regs->ref_ctrl_derated);
	pr_debug("sdram_tim_1_reg\t\t: 0x%08x\n", regs->sdram_tim1);
	pr_debug("sdram_tim_1_reg_derated\t\t: 0x%08x\n",
		 regs->sdram_tim1_derated);
	pr_debug("sdram_tim_2_reg\t\t: 0x%08x\n", regs->sdram_tim2);
	pr_debug("sdram_tim_3_reg\t\t: 0x%08x\n", regs->sdram_tim3);
	pr_debug("emif_read_idle_ctrl_normal\t: 0x%08x\n",
		 regs->read_idle_ctrl_normal);
	pr_debug("emif_read_idle_ctrl_dvfs\t: 0x%08x\n",
		 regs->read_idle_ctrl_volt_ramp);
	pr_debug("zq_config_reg_normal\t: 0x%08x\n", regs->zq_config_normal);
	pr_debug("zq_config_reg_dvfs\t\t: 0x%08x\n", regs->zq_config_volt_ramp);
	pr_debug("temp_alert_config\t: 0x%08x\n", regs->temp_alert_config);
	pr_debug("emif_ddr_phy_ctlr_1_init\t: 0x%08x\n",
		 regs->emif_ddr_phy_ctlr_1_init);
	pr_debug("emif_ddr_phy_ctlr_1_final\t: 0x%08x\n",
		 regs->emif_ddr_phy_ctlr_1_final);
}

/*
 * get_regs() - gets the cached emif_regs structure for a given EMIF instance
 * (emif_nr) for a given frequency(freq):
 *
 * As an optimization, only one cache array(that of EMIF1) if both EMIF1 and
 * EMIF2 has identical devices
 *
 * If we do not have an entry corresponding to the frequency given, we
 * allocate a new entry and calculate the values
 */
static struct emif_regs *get_regs(u32 emif_nr, u32 freq)
{
	int i;
	struct emif_regs **regs_cache;
	struct emif_regs *regs = NULL;

	/*
	 * If EMIF2 has the same devices as EMIF1 use the register
	 * cache of EMIF1
	 */
	if ((emif_nr == EMIF1) ||
	    ((emif_nr == EMIF2)
	     && (emif_devices[EMIF1] == emif_devices[EMIF2])))
		regs_cache = emif1_regs_cache;
	else
		regs_cache = emif2_regs_cache;

	for (i = 0; i < EMIF_MAX_NUM_FREQUENCIES && regs_cache[i]; i++) {
		if (regs_cache[i]->freq == freq) {
			regs = regs_cache[i];
			break;
		}
	}

	/*
	 * If we don't have an entry for this frequency in the cache create one
	 * and calculate the values
	 */
	if (!regs) {
		regs = kmalloc(sizeof(struct emif_regs), GFP_ATOMIC);
		if (!regs)
			return NULL;
		emif_calculate_regs(emif_devices[emif_nr], freq, regs);

		/*
		 * Now look for an un-used entry in the cache and save the
		 * newly created struct. If there are no free entries
		 * over-write the last entry
		 */
		for (i = 0; i < EMIF_MAX_NUM_FREQUENCIES && regs_cache[i]; i++)
			;

		if (i >= EMIF_MAX_NUM_FREQUENCIES) {
			pr_warning("emif: emif regs_cache full - more number"
				   " of frequencies used than expected!!");
			i = EMIF_MAX_NUM_FREQUENCIES - 1;
			kfree(regs_cache[i]);
		}
		regs_cache[i] = regs;
	}
	return regs;
}

static int do_emif_setup_registers(u32 emif_nr, u32 freq, u32 volt_state)
{
	struct emif_regs *regs;
	regs = get_regs(emif_nr, freq);
	if (!regs)
		return -ENOMEM;

	emif_curr_regs[emif_nr] = regs;
	setup_registers(emif_nr, regs, volt_state);
	setup_temperature_sensitive_regs(emif_nr, regs);

	return 0;
}

static int do_setup_device_details(u32 emif_nr,
				   const struct emif_device_details *devices)
{
	if (!emif_devices[emif_nr]) {
		emif_devices[emif_nr] =
		    kmalloc(sizeof(struct emif_device_details), GFP_KERNEL);
		if (!emif_devices[emif_nr])
			return -ENOMEM;
		*emif_devices[emif_nr] = *devices;
	}

	return 0;
}

/*
 * Initialize the temperature level and setup the sysfs nodes
 * and uvent for temperature monitoring
 */
static void init_temperature(u32 emif_nr)
{
	if (!emif_devices[emif_nr])
		return;

	emif_temperature_level[emif_nr] = get_temperature_level(emif_nr);
	WARN_ON(device_create_file(&(emif[emif_nr].pdev->dev),
				   &dev_attr_temperature));
	kobject_uevent(&(emif[emif_nr].pdev->dev.kobj), KOBJ_ADD);

	if (emif_temperature_level[emif_nr] == SDRAM_TEMP_VERY_HIGH_SHUTDOWN)
		pr_emerg("EMIF %d: SDRAM temperature exceeds operating"
			 "limit.. Needs shut down!!!", emif_nr + 1);
}

/*
 * omap_emif_device_init needs to be done before
 * ddr reconfigure function call.
 * Hence omap_emif_device_init is a postcore_initcall.
 */
static int __init omap_emif_device_init(void)
{
	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!cpu_is_omap44xx())
		return -ENODEV;

	return omap_hwmod_for_each_by_class("emif", emif_init, NULL);
}
postcore_initcall(omap_emif_device_init);


/* We need to disable interrupts of the EMIF
 * module, because in a warm reboot scenario, there
 * may be a pending irq that is not serviced and emif
 * is stuck in transition. On the next boot HW mod
 * fails emif inizalization with a timeout.
 */
void emif_clear_irq(int emif_id)
{
	u32 irq_mask = 0;
	u32 base = 0;
	u32 reg = 0;

	if (emif_id == 0)
		base = OMAP44XX_EMIF1_VIRT;
	else
		base = OMAP44XX_EMIF2_VIRT;

	/* Disable the relevant interrupts for both LL and SYS */
	irq_mask = OMAP44XX_REG_EN_TA_SYS_MASK | OMAP44XX_REG_EN_ERR_SYS_MASK
		| OMAP44XX_REG_EN_DNV_SYS_MASK;
	__raw_writel(irq_mask, base + OMAP44XX_EMIF_IRQENABLE_CLR_SYS);
	__raw_writel(irq_mask, base + OMAP44XX_EMIF_IRQENABLE_CLR_LL);

	/* Clear any pendining interrupts without overwritng reserved bits*/
	reg = __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_SYS);
	reg |= irq_mask;
	__raw_writel(reg, base + OMAP44XX_EMIF_IRQSTATUS_SYS);

	reg = __raw_readl(base + OMAP44XX_EMIF_IRQSTATUS_LL);
	reg |= irq_mask;
	__raw_writel(reg, base + OMAP44XX_EMIF_IRQSTATUS_LL);

	 /* Dummy read to make sure writes are complete */
	__raw_readl(base + OMAP44XX_EMIF_IRQENABLE_SET_LL);

	return;
}

void emif_driver_shutdown(struct platform_device *pdev)
{
	emif_clear_irq(pdev->id);
}

static struct platform_driver omap_emif_driver = {
	.probe = omap_emif_probe,
	.driver = {
		   .name = "omap_emif",
		   },

	.shutdown = emif_driver_shutdown,
};

static int __init omap_emif_register(void)
{
	return platform_driver_register(&omap_emif_driver);
}
postcore_initcall(omap_emif_register);

/*
 * omap_emif_notify_voltage - setup the voltage sensitive
 * registers based on the voltage situation (voltage ramping or stable)
 * read_idle_ctrl and zq_config are the registers that are voltage sensitive
 * They need to have a very safe value(more frequent zq calibration and
 * read idle forcing) when voltage is scaling and can have a more relaxed
 * nominal value(frequency dependent) when voltage is stable
 */
int omap_emif_notify_voltage(struct notifier_block *nb,
		unsigned long val, void *data)
{
	u32 volt_state;

	if (val == OMAP_VOLTAGE_PRECHANGE)
		volt_state =  LPDDR2_VOLTAGE_RAMPING;
	 else
		volt_state =  LPDDR2_VOLTAGE_STABLE;

	if (likely(emif_curr_regs[EMIF1]))
		setup_volt_sensitive_registers(EMIF1, emif_curr_regs[EMIF1],
					       volt_state);

	if (likely(emif_curr_regs[EMIF2]))
		setup_volt_sensitive_registers(EMIF2, emif_curr_regs[EMIF2],
					       volt_state);

	if (unlikely(!emif_curr_regs[EMIF1] && !emif_curr_regs[EMIF2])) {
		pr_err_once("emif: voltage state notification came before the"
		       " initial setup - ignoring the notification");
		return -EINVAL;
	}

	/*
	 * EMIF read-idle control needs to be setup using
	 * freq update method only
	 */
	return omap4_prcm_freq_update();
}

static struct notifier_block emif_volt_notifier_block = {
	.notifier_call = omap_emif_notify_voltage,
};

static int __init omap_emif_late_init(void)
{
	struct voltagedomain *voltdm = voltdm_lookup("core");

	if (!voltdm) {
		pr_err("CORE voltage domain lookup failed\n");
		return -EINVAL;
	}

	voltdm_register_notifier(voltdm, &emif_volt_notifier_block);

	return 0;
}
late_initcall(omap_emif_late_init);

/*
 * omap_emif_setup_registers - setup the shadow registers for a given
 * frequency. This will be typically followed by a FREQ_UPDATE procedure
 * to lock at the new frequency and this will update the EMIF main registers
 * with shadow register values
 */
int omap_emif_setup_registers(u32 freq, u32 volt_state)
{
	int err = 0;
	if (likely(emif_devices[EMIF1]))
		err = do_emif_setup_registers(EMIF1, freq, volt_state);
	if (likely(!err && emif_devices[EMIF2]))
		err = do_emif_setup_registers(EMIF2, freq, volt_state);
	return err;
}


/*
 * omap_emif_frequency_pre_notify - Disable DDR self refresh of both EMIFs
 *
 * It disables the LP mode if the LP mode of EMIFs was LP_MODE_SELF_REFRESH.
 *
 * It should be called before any PRCM frequency update sequence.
 * After the frequency update sequence, omap_emif_frequency_post_notify
 * should be called to restore the original LP MODE setting of the EMIFs.
 *
 */
void omap_emif_frequency_pre_notify(void)
{
	int emif_num;

	for (emif_num = EMIF1; emif_num < EMIF_NUM_INSTANCES; emif_num++) {

		/*
		 * Only disable ddr self-refresh
		 * if ddr self-refresh was enabled
		 */
		if (likely(LP_MODE_SELF_REFRESH == get_lp_mode(emif_num))) {

			set_lp_mode(emif_num, LP_MODE_DISABLE);
			emif[emif_num].ddr_refresh_disabled = true;
		}

	}
}

/*
 * omap_emif_frequency_post_notify - Enable DDR self refresh of both EMIFs
 *
 * It restores the LP mode of the EMIFs back to LP_MODE_SELF_REFRESH if it
 * was previously disabled by omap_emif_frequency_pre_notify()
 *
 */
void omap_emif_frequency_post_notify(void)
{
	int emif_num;

	for (emif_num = EMIF1; emif_num < EMIF_NUM_INSTANCES; emif_num++) {

		/*
		 * Only re-enable ddr self-refresh
		 * if ddr self-refresh was disabled
		 */
		if (likely(emif[emif_num].ddr_refresh_disabled)) {

			set_lp_mode(emif_num, LP_MODE_SELF_REFRESH);
			emif[emif_num].ddr_refresh_disabled = false;
		}
	}
}

/*
 * omap_emif_setup_device_details - save the SDRAM device details passed
 * from the board file
 */
int omap_emif_setup_device_details(const struct emif_device_details
				   *emif1_devices,
				   const struct emif_device_details
				   *emif2_devices)
{
	if (emif1_devices)
		BUG_ON(do_setup_device_details(EMIF1, emif1_devices));

	/*
	 * If memory devices connected to both the EMIFs are identical
	 * (which is normally the case), then no need to calculate the
	 * registers again for EMIF1 and allocate the structure for registers
	 */
	if (emif2_devices && (emif1_devices != emif2_devices))
		BUG_ON(do_setup_device_details(EMIF2, emif2_devices));
	else if (emif2_devices) {
		emif_devices[EMIF2] = emif_devices[EMIF1];
		/* call for temperature related setup */
		BUG_ON(do_setup_device_details(EMIF2, emif2_devices));
	}

	return 0;
}

static void __init setup_lowpower_regs(u32 emif_nr,
				       struct emif_device_details *emif_dev)
{
	u32 temp;
	void __iomem *base = emif[emif_nr].base;
	const struct lpddr2_device_info *dev;

	if (!emif_dev) {
		pr_err("%s: no emif %d\n", __func__, emif_nr);
		return;
	}

	/*
	 * All devices on this specific EMIF should have the same Selfrefresh
	 * timing, so use cs0
	 */
	dev = emif_dev->cs0_device;
	if (!dev) {
		pr_err("%s: no CS0 device in emif %d\n", __func__, emif_nr);
		return;
	}
	if (dev->emif_ddr_selfrefresh_cycles >= 0) {
		u32 num_cycles, ddr_sr_timer;

		/* Enable self refresh if not already configured */
		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL) &
			OMAP44XX_REG_LP_MODE_MASK;
		/*
		 * Configure the self refresh timing
		 * base value starts at 16 cycles mapped to 1( __fls(16) = 4)
		 */
		num_cycles = dev->emif_ddr_selfrefresh_cycles;
		if (num_cycles >= 16)
			ddr_sr_timer = __fls(num_cycles) - 3;
		else
			ddr_sr_timer = 0;

		/* Program the idle delay */
		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL_SHDW);
		mask_n_set(temp, OMAP44XX_REG_SR_TIM_SHDW_SHIFT,
			   OMAP44XX_REG_SR_TIM_SHDW_MASK, ddr_sr_timer);
		/*
		 * Some weird magic number to a field which should'nt impact..
		 * but seems to make this work..
		 */
		mask_n_set(temp, OMAP44XX_REG_CS_TIM_SHDW_SHIFT,
			   OMAP44XX_REG_CS_TIM_SHDW_MASK, 0xf);
		__raw_writel(temp, base + OMAP44XX_EMIF_PWR_MGMT_CTRL_SHDW);

		/* Enable Self Refresh */
		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL);
		mask_n_set(temp, OMAP44XX_REG_LP_MODE_SHIFT,
			   OMAP44XX_REG_LP_MODE_MASK, LP_MODE_SELF_REFRESH);
		__raw_writel(temp, base + OMAP44XX_EMIF_PWR_MGMT_CTRL);
	} else {
		/* Disable Automatic power management if < 0 and not disabled */
		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL) &
			OMAP44XX_REG_LP_MODE_MASK;

		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL_SHDW);
		mask_n_set(temp, OMAP44XX_REG_SR_TIM_SHDW_SHIFT,
			   OMAP44XX_REG_SR_TIM_SHDW_MASK, 0x0);
		__raw_writel(temp, base + OMAP44XX_EMIF_PWR_MGMT_CTRL_SHDW);

		temp = __raw_readl(base + OMAP44XX_EMIF_PWR_MGMT_CTRL);
		mask_n_set(temp, OMAP44XX_REG_LP_MODE_SHIFT,
			   OMAP44XX_REG_LP_MODE_MASK, LP_MODE_DISABLE);
		__raw_writel(temp, base + OMAP44XX_EMIF_PWR_MGMT_CTRL);
	}
}

/*
 * omap_init_emif_timings - reprogram EMIF timing parameters
 *
 * Sets the CORE DPLL3 M2 divider to the same value that it's at
 * currently.  This has the effect of setting the EMIF DDR AC timing
 * registers to the values currently defined by the kernel.
 */
static int __init omap_init_emif_timings(void)
{
	struct clk *dpll_core_m2_clk;
	int ret;
	long rate;

	/*
	 * Setup the initial temperatures sysfs nodes etc.
	 * Subsequent updates to temperature is done through interrupts
	 */
	init_temperature(EMIF1);
	init_temperature(EMIF2);

	/* FREQ_UPDATE sequence isn't supported on early vesion */
	if (omap_rev() == OMAP4430_REV_ES1_0)
		return -EINVAL;

	dpll_core_m2_clk = clk_get(NULL, "dpll_core_m2_ck");
	if (!dpll_core_m2_clk)
		pr_err("Could not get LPDDR2 clock - dpll_core_m2_ck\n");

	rate = clk_get_rate(dpll_core_m2_clk);
	pr_info("Reprogramming LPDDR2 timings to %ld Hz\n", rate >> 1);

	ret = clk_set_rate(dpll_core_m2_clk, rate);
	if (ret)
		pr_err("Unable to set LPDDR2 rate to %ld:\n", rate);

	/* registers are setup correctly - now enable interrupts */
	if (emif_devices[EMIF1]) {
		ret = setup_emif_interrupts(EMIF1);
		setup_lowpower_regs(EMIF1, emif_devices[EMIF1]);
	}
	if (!ret && emif_devices[EMIF2]) {
		ret = setup_emif_interrupts(EMIF2);
		setup_lowpower_regs(EMIF2, emif_devices[EMIF2]);
	}

	clk_put(dpll_core_m2_clk);

	return ret;
}
late_initcall(omap_init_emif_timings);
