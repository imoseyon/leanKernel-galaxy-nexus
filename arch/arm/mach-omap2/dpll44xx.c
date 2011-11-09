/*
 * OMAP4-specific DPLL control functions
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Rajendra Nayak
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/common.h>

#include <mach/emif.h>
#include <mach/omap4-common.h>

#include "clock.h"
#include "clock44xx.h"
#include "cm.h"
#include "cm44xx.h"
#include "cm1_44xx.h"
#include "cm2_44xx.h"
#include "cminst44xx.h"
#include "clock44xx.h"
#include "clockdomain.h"
#include "cm-regbits-44xx.h"
#include "prcm44xx.h"

#define MAX_FREQ_UPDATE_TIMEOUT  100000

static struct clockdomain *l3_emif_clkdm;
static DEFINE_SPINLOCK(l3_emif_lock);

/**
 * omap4_core_dpll_m2_set_rate - set CORE DPLL M2 divider
 * @clk: struct clk * of DPLL to set
 * @rate: rounded target rate
 *
 * Programs the CM shadow registers to update CORE DPLL M2
 * divider. M2 divider is used to clock external DDR and its
 * reconfiguration on frequency change is managed through a
 * hardware sequencer. This is managed by the PRCM with EMIF
 * uding shadow registers.
 * Returns -EINVAL/-1 on error and 0 on success.
 */
int omap4_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate)
{
	int i = 0;
	u32 validrate = 0, shadow_freq_cfg1 = 0, new_div = 0;
	unsigned long flags;

	if (!clk || !rate)
		return -EINVAL;

	validrate = omap2_clksel_round_rate_div(clk, rate, &new_div);
	if (validrate != rate)
		return -EINVAL;

	/* Just to avoid look-up on every call to speed up */
	if (!l3_emif_clkdm) {
		l3_emif_clkdm = clkdm_lookup("l3_emif_clkdm");
		if (!l3_emif_clkdm) {
			pr_err("%s: clockdomain lookup failed\n", __func__);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&l3_emif_lock, flags);

	/* Configures MEMIF domain in SW_WKUP */
	clkdm_wakeup(l3_emif_clkdm);

	/*
	 * Program EMIF timing parameters in EMIF shadow registers
	 * for targetted DRR clock.
	 * DDR Clock = core_dpll_m2 / 2
	 */
	omap_emif_setup_registers(validrate >> 1, LPDDR2_VOLTAGE_STABLE);

	/*
	 * FREQ_UPDATE sequence:
	 * - DLL_OVERRIDE=0 (DLL lock & code must not be overridden
	 *	after CORE DPLL lock)
	 * - DLL_RESET=1 (DLL must be reset upon frequency change)
	 * - DPLL_CORE_M2_DIV with same value as the one already
	 *	in direct register
	 * - DPLL_CORE_DPLL_EN=0x7 (to make CORE DPLL lock)
	 * - FREQ_UPDATE=1 (to start HW sequence)
	 */
	shadow_freq_cfg1 = (1 << OMAP4430_DLL_RESET_SHIFT) |
			(new_div << OMAP4430_DPLL_CORE_M2_DIV_SHIFT) |
			(DPLL_LOCKED << OMAP4430_DPLL_CORE_DPLL_EN_SHIFT) |
			(1 << OMAP4430_FREQ_UPDATE_SHIFT);
	shadow_freq_cfg1 &= ~OMAP4430_DLL_OVERRIDE_MASK;
	__raw_writel(shadow_freq_cfg1, OMAP4430_CM_SHADOW_FREQ_CONFIG1);

	/* wait for the configuration to be applied */
	omap_test_timeout(((__raw_readl(OMAP4430_CM_SHADOW_FREQ_CONFIG1)
				& OMAP4430_FREQ_UPDATE_MASK) == 0),
				MAX_FREQ_UPDATE_TIMEOUT, i);

	/* Configures MEMIF domain back to HW_WKUP */
	clkdm_allow_idle(l3_emif_clkdm);

	spin_unlock_irqrestore(&l3_emif_lock, flags);

	if (i == MAX_FREQ_UPDATE_TIMEOUT) {
		pr_err("%s: Frequency update for CORE DPLL M2 change failed\n",
				__func__);
		return -1;
	}

	/* Update the clock change */
	clk->rate = validrate;

	return 0;
}


/**
 * omap4_prcm_freq_update - set freq_update bit
 *
 * Programs the CM shadow registers to update EMIF
 * parametrs. Few usecase only few registers needs to
 * be updated using prcm freq update sequence.
 * EMIF read-idle control and zq-config needs to be
 * updated for temprature alerts and voltage change
 * Returns -1 on error and 0 on success.
 */
int omap4_prcm_freq_update(void)
{
	u32 shadow_freq_cfg1;
	int i = 0;
	unsigned long flags;

	if (!l3_emif_clkdm) {
		pr_err("%s: clockdomain lookup failed\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&l3_emif_lock, flags);
	/* Configures MEMIF domain in SW_WKUP */
	clkdm_wakeup(l3_emif_clkdm);

	/*
	 * FREQ_UPDATE sequence:
	 * - DLL_OVERRIDE=0 (DLL lock & code must not be overridden
	 *	after CORE DPLL lock)
	 * - FREQ_UPDATE=1 (to start HW sequence)
	 */
	shadow_freq_cfg1 = __raw_readl(OMAP4430_CM_SHADOW_FREQ_CONFIG1);
	shadow_freq_cfg1 |= (1 << OMAP4430_DLL_RESET_SHIFT) |
			   (1 << OMAP4430_FREQ_UPDATE_SHIFT);
	shadow_freq_cfg1 &= ~OMAP4430_DLL_OVERRIDE_MASK;
	__raw_writel(shadow_freq_cfg1, OMAP4430_CM_SHADOW_FREQ_CONFIG1);

	/* wait for the configuration to be applied */
	omap_test_timeout(((__raw_readl(OMAP4430_CM_SHADOW_FREQ_CONFIG1)
				& OMAP4430_FREQ_UPDATE_MASK) == 0),
				MAX_FREQ_UPDATE_TIMEOUT, i);

	/* Configures MEMIF domain back to HW_WKUP */
	clkdm_allow_idle(l3_emif_clkdm);

	spin_unlock_irqrestore(&l3_emif_lock, flags);

	if (i == MAX_FREQ_UPDATE_TIMEOUT) {
		pr_err("%s: Frequency update failed (call from %pF)\n",
			__func__, (void *)_RET_IP_);
		return -1;
	}

	return 0;
}

/* Use a very high retry count - we should not hit this condition */
#define MAX_DPLL_WAIT_TRIES	1000000

#define OMAP_1_5GHz	1500000000
#define OMAP_1_2GHz	1200000000
#define OMAP_1GHz	1000000000
#define OMAP_920MHz	920000000
#define OMAP_748MHz	748000000

/* Supported only on OMAP4 */
int omap4_dpllmx_gatectrl_read(struct clk *clk)
{
	u32 v;
	u32 mask;

	if (!clk || !clk->clksel_reg || !cpu_is_omap44xx())
		return -EINVAL;

	mask = clk->flags & CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;

	v = __raw_readl(clk->clksel_reg);
	v &= mask;
	v >>= __ffs(mask);

	return v;
}

void omap4_dpllmx_allow_gatectrl(struct clk *clk)
{
	u32 v;
	u32 mask;

	if (!clk || !clk->clksel_reg || !cpu_is_omap44xx())
		return;

	mask = clk->flags & CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;

	v = __raw_readl(clk->clksel_reg);
	/* Clear the bit to allow gatectrl */
	v &= ~mask;
	__raw_writel(v, clk->clksel_reg);
}

void omap4_dpllmx_deny_gatectrl(struct clk *clk)
{
	u32 v;
	u32 mask;

	if (!clk || !clk->clksel_reg || !cpu_is_omap44xx())
		return;

	mask = clk->flags & CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;

	v = __raw_readl(clk->clksel_reg);
	/* Set the bit to deny gatectrl */
	v |= mask;
	__raw_writel(v, clk->clksel_reg);
}

const struct clkops clkops_omap4_dpllmx_ops = {
	.allow_idle	= omap4_dpllmx_allow_gatectrl,
	.deny_idle	= omap4_dpllmx_deny_gatectrl,
};

static void omap4460_mpu_dpll_update_children(unsigned long rate)
{
	u32 v;

	/*
	 * The interconnect frequency to EMIF should
	 * be switched between MPU clk divide by 4 (for
	 * frequencies higher than 920Mhz) and MPU clk divide
	 * by 2 (for frequencies lower than or equal to 920Mhz)
	 * Also the async bridge to ABE must be MPU clk divide
	 * by 8 for MPU clk > 748Mhz and MPU clk divide by 4
	 * for lower frequencies.
	 */
	v = __raw_readl(OMAP4430_CM_MPU_MPU_CLKCTRL);
	if (rate > OMAP_920MHz)
		v |= OMAP4460_CLKSEL_EMIF_DIV_MODE_MASK;
	else
		v &= ~OMAP4460_CLKSEL_EMIF_DIV_MODE_MASK;

	if (rate > OMAP_748MHz)
		v |= OMAP4460_CLKSEL_ABE_DIV_MODE_MASK;
	else
		v &= ~OMAP4460_CLKSEL_ABE_DIV_MODE_MASK;
	__raw_writel(v, OMAP4430_CM_MPU_MPU_CLKCTRL);
}

int omap4460_mpu_dpll_set_rate(struct clk *clk, unsigned long rate)
{
	struct dpll_data *dd;
	u32 v;
	unsigned long dpll_rate;

	if (!clk || !rate || !clk->parent)
		return -EINVAL;

	dd = clk->parent->dpll_data;

	if (!dd)
		return -EINVAL;

	if (!clk->parent->set_rate)
		return -EINVAL;

	if (rate > clk->rate)
		omap4460_mpu_dpll_update_children(rate);

	/*
	 * On OMAP4460, to obtain MPU DPLL frequency higher
	 * than 1GHz, DCC (Duty Cycle Correction) needs to
	 * be enabled.
	 * And needs to be kept disabled for < 1 Ghz.
	 */
	dpll_rate = omap2_get_dpll_rate(clk->parent);
	if (rate <= OMAP_1_5GHz) {
		/* If DCC is enabled, disable it */
		v = __raw_readl(dd->mult_div1_reg);
		if (v & OMAP4460_DCC_EN_MASK) {
			v &= ~OMAP4460_DCC_EN_MASK;
			__raw_writel(v, dd->mult_div1_reg);
		}

		if (rate != dpll_rate)
			clk->parent->set_rate(clk->parent, rate);
	} else {
		/*
		 * On 4460, the MPU clk for frequencies higher than 1Ghz
		 * is sourced from CLKOUTX2_M3, instead of CLKOUT_M2, while
		 * value of M3 is fixed to 1. Hence for frequencies higher
		 * than 1 Ghz, lock the DPLL at half the rate so the
		 * CLKOUTX2_M3 then matches the requested rate.
		 */
		if (rate != dpll_rate * 2)
			clk->parent->set_rate(clk->parent, rate / 2);

		v = __raw_readl(dd->mult_div1_reg);
		v &= ~OMAP4460_DCC_COUNT_MAX_MASK;
		v |= (5 << OMAP4460_DCC_COUNT_MAX_SHIFT);
		__raw_writel(v, dd->mult_div1_reg);

		v |= OMAP4460_DCC_EN_MASK;
		__raw_writel(v, dd->mult_div1_reg);
	}

	if (rate < clk->rate)
		omap4460_mpu_dpll_update_children(rate);

	clk->rate = rate;

	return 0;
}

long omap4460_mpu_dpll_round_rate(struct clk *clk, unsigned long rate)
{
	if (!clk || !rate || !clk->parent)
		return -EINVAL;

	if (clk->parent->round_rate)
		return clk->parent->round_rate(clk->parent, rate);
	else
		return 0;
}

unsigned long omap4460_mpu_dpll_recalc(struct clk *clk)
{
	struct dpll_data *dd;
	u32 v;

	if (!clk || !clk->parent)
		return -EINVAL;

	dd = clk->parent->dpll_data;

	if (!dd)
		return -EINVAL;

	v = __raw_readl(dd->mult_div1_reg);
	if (v & OMAP4460_DCC_EN_MASK)
		return omap2_get_dpll_rate(clk->parent) * 2;
	else
		return omap2_get_dpll_rate(clk->parent);
}

unsigned long omap4_dpll_regm4xen_recalc(struct clk *clk)
{
       u32 v;
       unsigned long rate;
       struct dpll_data *dd;

       if (!clk || !clk->dpll_data)
               return -EINVAL;

       dd = clk->dpll_data;

       rate = omap2_get_dpll_rate(clk);

       /* regm4xen adds a multiplier of 4 to DPLL calculations */
       v = __raw_readl(dd->control_reg);
       if (v & OMAP4430_DPLL_REGM4XEN_MASK)
               rate *= OMAP4430_REGM4XEN_MULT;

       return rate;
}

long omap4_dpll_regm4xen_round_rate(struct clk *clk, unsigned long target_rate)
{
       u32 v;
       struct dpll_data *dd;

       if (!clk || !clk->dpll_data)
               return -EINVAL;

       dd = clk->dpll_data;

       /* regm4xen adds a multiplier of 4 to DPLL calculations */
       v = __raw_readl(dd->control_reg) & OMAP4430_DPLL_REGM4XEN_MASK;

       if (v)
               target_rate = target_rate / OMAP4430_REGM4XEN_MULT;

       omap2_dpll_round_rate(clk, target_rate);

       if (v)
               clk->dpll_data->last_rounded_rate *= OMAP4430_REGM4XEN_MULT;

       return clk->dpll_data->last_rounded_rate;
}

struct dpll_reg_tuple {
	u16 addr;
	u32 val;
};

struct omap4_dpll_regs {
	char *name;
	u32 mod_partition;
	u32 mod_inst;
	struct dpll_reg_tuple clkmode;
	struct dpll_reg_tuple autoidle;
	struct dpll_reg_tuple idlest;
	struct dpll_reg_tuple clksel;
	struct dpll_reg_tuple div_m2;
	struct dpll_reg_tuple div_m3;
	struct dpll_reg_tuple div_m4;
	struct dpll_reg_tuple div_m5;
	struct dpll_reg_tuple div_m6;
	struct dpll_reg_tuple div_m7;
	struct dpll_reg_tuple clkdcoldo;
};

static struct omap4_dpll_regs dpll_regs[] = {
	/* MPU DPLL */
	{ .name		= "mpu",
	  .mod_partition = OMAP4430_CM1_PARTITION,
	  .mod_inst	= OMAP4430_CM1_CKGEN_INST,
	  .clkmode	= {.addr = OMAP4_CM_CLKMODE_DPLL_MPU_OFFSET},
	  .autoidle	= {.addr = OMAP4_CM_AUTOIDLE_DPLL_MPU_OFFSET},
	  .idlest	= {.addr = OMAP4_CM_IDLEST_DPLL_MPU_OFFSET},
	  .clksel	= {.addr = OMAP4_CM_CLKSEL_DPLL_MPU_OFFSET},
	  .div_m2	= {.addr = OMAP4_CM_DIV_M2_DPLL_MPU_OFFSET},
	},
	/* IVA DPLL */
	{ .name		= "iva",
	  .mod_partition = OMAP4430_CM1_PARTITION,
	  .mod_inst	= OMAP4430_CM1_CKGEN_INST,
	  .clkmode	= {.addr = OMAP4_CM_CLKMODE_DPLL_IVA_OFFSET},
	  .autoidle	= {.addr = OMAP4_CM_AUTOIDLE_DPLL_IVA_OFFSET},
	  .idlest	= {.addr = OMAP4_CM_IDLEST_DPLL_IVA_OFFSET},
	  .clksel	= {.addr = OMAP4_CM_CLKSEL_DPLL_IVA_OFFSET},
	  .div_m4	= {.addr = OMAP4_CM_DIV_M4_DPLL_IVA_OFFSET},
	  .div_m5	= {.addr = OMAP4_CM_DIV_M5_DPLL_IVA_OFFSET},
	},
	/* ABE DPLL */
	{ .name		= "abe",
	  .mod_partition = OMAP4430_CM1_PARTITION,
	  .mod_inst	= OMAP4430_CM1_CKGEN_INST,
	  .clkmode	= {.addr = OMAP4_CM_CLKMODE_DPLL_ABE_OFFSET},
	  .autoidle	= {.addr = OMAP4_CM_AUTOIDLE_DPLL_ABE_OFFSET},
	  .idlest	= {.addr = OMAP4_CM_IDLEST_DPLL_ABE_OFFSET},
	  .clksel	= {.addr = OMAP4_CM_CLKSEL_DPLL_ABE_OFFSET},
	  .div_m2	= {.addr = OMAP4_CM_DIV_M2_DPLL_ABE_OFFSET},
	  .div_m3	= {.addr = OMAP4_CM_DIV_M3_DPLL_ABE_OFFSET},
	},
	/* USB DPLL */
	{ .name		= "usb",
	  .mod_partition = OMAP4430_CM2_PARTITION,
	  .mod_inst	= OMAP4430_CM2_CKGEN_INST,
	  .clkmode	= {.addr = OMAP4_CM_CLKMODE_DPLL_USB_OFFSET},
	  .autoidle	= {.addr = OMAP4_CM_AUTOIDLE_DPLL_USB_OFFSET},
	  .idlest	= {.addr = OMAP4_CM_IDLEST_DPLL_USB_OFFSET},
	  .clksel	= {.addr = OMAP4_CM_CLKSEL_DPLL_USB_OFFSET},
	  .div_m2	= {.addr = OMAP4_CM_DIV_M2_DPLL_USB_OFFSET},
	  .clkdcoldo	= {.addr = OMAP4_CM_CLKDCOLDO_DPLL_USB_OFFSET},
	 },
	/* PER DPLL */
	{ .name		= "per",
	  .mod_partition = OMAP4430_CM2_PARTITION,
	  .mod_inst	= OMAP4430_CM2_CKGEN_INST,
	  .clkmode	= {.addr = OMAP4_CM_CLKMODE_DPLL_PER_OFFSET},
	  .autoidle	= {.addr = OMAP4_CM_AUTOIDLE_DPLL_PER_OFFSET},
	  .idlest	= {.addr = OMAP4_CM_IDLEST_DPLL_PER_OFFSET},
	  .clksel	= {.addr = OMAP4_CM_CLKSEL_DPLL_PER_OFFSET},
	  .div_m2	= {.addr = OMAP4_CM_DIV_M2_DPLL_PER_OFFSET},
	  .div_m3	= {.addr = OMAP4_CM_DIV_M3_DPLL_PER_OFFSET},
	  .div_m4	= {.addr = OMAP4_CM_DIV_M4_DPLL_PER_OFFSET},
	  .div_m5	= {.addr = OMAP4_CM_DIV_M5_DPLL_PER_OFFSET},
	  .div_m6	= {.addr = OMAP4_CM_DIV_M6_DPLL_PER_OFFSET},
	  .div_m7	= {.addr = OMAP4_CM_DIV_M7_DPLL_PER_OFFSET},
	},
};

static inline void omap4_dpll_store_reg(struct omap4_dpll_regs *dpll_reg,
					struct dpll_reg_tuple *tuple)
{
	if (tuple->addr)
		tuple->val =
		    omap4_cminst_read_inst_reg(dpll_reg->mod_partition,
					       dpll_reg->mod_inst, tuple->addr);
}

void omap4_dpll_prepare_off(void)
{
	u32 i;
	struct omap4_dpll_regs *dpll_reg = dpll_regs;

	for (i = 0; i < ARRAY_SIZE(dpll_regs); i++, dpll_reg++) {
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->clkmode);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->autoidle);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->clksel);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m2);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m3);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m4);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m5);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m6);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->div_m7);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->clkdcoldo);
		omap4_dpll_store_reg(dpll_reg, &dpll_reg->idlest);
	}
}

static void omap4_dpll_print_reg(struct omap4_dpll_regs *dpll_reg, char *name,
				 struct dpll_reg_tuple *tuple)
{
	if (tuple->addr)
		pr_warn("%s - Address offset = 0x%08x, value=0x%08x\n", name,
			tuple->addr, tuple->val);
}

static void omap4_dpll_dump_regs(struct omap4_dpll_regs *dpll_reg)
{
	pr_warn("%s: Unable to lock dpll %s[part=%x inst=%x]:\n",
		__func__, dpll_reg->name, dpll_reg->mod_partition,
		dpll_reg->mod_inst);
	omap4_dpll_print_reg(dpll_reg, "clksel", &dpll_reg->clksel);
	omap4_dpll_print_reg(dpll_reg, "div_m2", &dpll_reg->div_m2);
	omap4_dpll_print_reg(dpll_reg, "div_m3", &dpll_reg->div_m3);
	omap4_dpll_print_reg(dpll_reg, "div_m4", &dpll_reg->div_m4);
	omap4_dpll_print_reg(dpll_reg, "div_m5", &dpll_reg->div_m5);
	omap4_dpll_print_reg(dpll_reg, "div_m6", &dpll_reg->div_m6);
	omap4_dpll_print_reg(dpll_reg, "div_m7", &dpll_reg->div_m7);
	omap4_dpll_print_reg(dpll_reg, "clkdcoldo", &dpll_reg->clkdcoldo);
	omap4_dpll_print_reg(dpll_reg, "clkmode", &dpll_reg->clkmode);
	omap4_dpll_print_reg(dpll_reg, "autoidle", &dpll_reg->autoidle);
	if (dpll_reg->idlest.addr)
		pr_warn("idlest - Address offset = 0x%08x, before val=0x%08x"
			" after = 0x%08x\n", dpll_reg->idlest.addr,
			dpll_reg->idlest.val,
			omap4_cminst_read_inst_reg(dpll_reg->mod_partition,
						   dpll_reg->mod_inst,
						   dpll_reg->idlest.addr));
}

static void omap4_wait_dpll_lock(struct omap4_dpll_regs *dpll_reg)
{
	int j = 0;

	/* Return if we dont need to lock. */
	if ((dpll_reg->clkmode.val & OMAP4430_DPLL_EN_MASK) !=
	     DPLL_LOCKED << OMAP4430_DPLL_EN_SHIFT);
		return;

	while ((omap4_cminst_read_inst_reg(dpll_reg->mod_partition,
					    dpll_reg->mod_inst,
					    dpll_reg->idlest.addr)
		 & OMAP4430_ST_DPLL_CLK_MASK) !=
		 0x1 << OMAP4430_ST_DPLL_CLK_SHIFT
	       && j < MAX_DPLL_WAIT_TRIES) {
		j++;
		udelay(1);
	}

	/* if we are unable to lock, warn and move on.. */
	if (j == MAX_DPLL_WAIT_TRIES)
		omap4_dpll_dump_regs(dpll_reg);
}

static inline void omap4_dpll_restore_reg(struct omap4_dpll_regs *dpll_reg,
					  struct dpll_reg_tuple *tuple)
{
	if (tuple->addr)
		omap4_cminst_write_inst_reg(tuple->val, dpll_reg->mod_partition,
					    dpll_reg->mod_inst, tuple->addr);
}

void omap4_dpll_resume_off(void)
{
	u32 i;
	struct omap4_dpll_regs *dpll_reg = dpll_regs;

	for (i = 0; i < ARRAY_SIZE(dpll_regs); i++, dpll_reg++) {
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->clksel);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m2);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m3);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m4);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m5);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m6);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->div_m7);
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->clkdcoldo);

		/* Restore clkmode after the above registers are restored */
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->clkmode);

		omap4_wait_dpll_lock(dpll_reg);

		/* Restore autoidle settings after the dpll is locked */
		omap4_dpll_restore_reg(dpll_reg, &dpll_reg->autoidle);
	}
}
