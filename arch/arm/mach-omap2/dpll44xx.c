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

#include <plat/cpu.h>
#include <plat/clock.h>

#include "clock.h"
#include "cm1_44xx.h"
#include "cm-regbits-44xx.h"

#define OMAP_1GHz	1000000000

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

	/*
	 * On OMAP4460, to obtain MPU DPLL frequency higher
	 * than 1GHz, DCC (Duty Cycle Correction) needs to
	 * be enabled.
	 * And needs to be kept disabled for < 1 Ghz.
	 */
	dpll_rate = omap2_get_dpll_rate(clk->parent);
	v = __raw_readl(dd->mult_div1_reg);
	if (rate < OMAP_1GHz) {
		if (rate == dpll_rate)
			return 0;
		/* If DCC is enabled, disable it */
		if (v & OMAP4460_DCC_EN_MASK) {
			v &= ~OMAP4460_DCC_EN_MASK;
			__raw_writel(v, dd->mult_div1_reg);
		}
		clk->parent->set_rate(clk->parent, rate);
	} else {
		if (rate == dpll_rate/2)
			return 0;
		v &= ~OMAP4460_DCC_COUNT_MAX_MASK;
		v |= (5 << OMAP4460_DCC_COUNT_MAX_SHIFT);
		v |= OMAP4460_DCC_EN_MASK;
		__raw_writel(v, dd->mult_div1_reg);
		/*
		 * On 4460, the MPU clk for frequencies higher than 1Ghz
		 * is sourced from CLKOUTX2_M3, instead of CLKOUT_M2, while
		 * value of M3 is fixed to 1. Hence for frequencies higher
		 * than 1 Ghz, lock the DPLL at half the rate so the
		 * CLKOUTX2_M3 then matches the requested rate.
		 */
		clk->parent->set_rate(clk->parent, rate/2);
	}

	clk->rate = rate;

	return 0;
}

long omap4460_mpu_dpll_round_rate(struct clk *clk, unsigned long rate)
{
	if (!clk || !rate || !clk->parent)
		return -EINVAL;

	if (clk->parent->round_rate)
		return clk->parent->round_rate(clk, rate);
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
