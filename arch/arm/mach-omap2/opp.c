/*
 * OMAP SoC specific OPP wrapper function
 *
 * Copyright (C) 2009-2010 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 * Copyright (C) 2010 Nokia Corporation.
 *      Eduardo Valentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/opp.h>
#include <linux/clk.h>

#include <plat/omap_device.h>
#include <plat/clock.h>

#include "omap_opp_data.h"
#include "dvfs.h"

/* Temp variable to allow multiple calls */
static u8 __initdata omap_table_init;

/**
 * omap_init_opp_table() - Initialize opp table as per the CPU type
 * @opp_def:		opp default list for this silicon
 * @opp_def_size:	number of opp entries for this silicon
 *
 * Register the initial OPP table with the OPP library based on the CPU
 * type. This is meant to be used only by SoC specific registration.
 */
int __init omap_init_opp_table(struct omap_opp_def *opp_def,
		u32 opp_def_size)
{
	int i, r;
	struct clk *clk;
	long round_rate;

	if (!opp_def || !opp_def_size) {
		pr_err("%s: invalid params!\n", __func__);
		return -EINVAL;
	}

	/*
	 * Initialize only if not already initialized even if the previous
	 * call failed, because, no reason we'd succeed again.
	 */
	if (omap_table_init)
		return -EEXIST;
	omap_table_init = 1;

	/* Lets now register with OPP library */
	for (i = 0; i < opp_def_size; i++, opp_def++) {
		struct omap_hwmod *oh;
		struct device *dev;

		if (!opp_def->hwmod_name) {
			WARN(1, "%s: NULL name of omap_hwmod, failing"
				" [%d].\n", __func__, i);
			continue;
		}
		oh = omap_hwmod_lookup(opp_def->hwmod_name);
		if (!oh || !oh->od) {
			WARN(1, "%s: no hwmod or odev for %s, [%d] "
				"cannot add OPPs.\n", __func__,
				opp_def->hwmod_name, i);
			continue;
		}
		dev = &oh->od->pdev.dev;

		clk = omap_clk_get_by_name(opp_def->clk_name);
		if (clk) {
			round_rate = clk_round_rate(clk, opp_def->freq);
			if (round_rate > 0) {
				opp_def->freq = round_rate;
			} else {
				WARN(1, "%s: round_rate for clock %s failed\n",
					__func__, opp_def->clk_name);
				continue; /* skip Bad OPP */
			}
		} else {
			WARN(1, "%s: No clock by name %s found\n", __func__,
				opp_def->clk_name);
			continue; /* skip Bad OPP */
		}
		r = opp_add(dev, opp_def->freq, opp_def->u_volt);
		if (r) {
			dev_err(dev, "%s: add OPP %ld failed for %s [%d] "
				"result=%d\n",
			       __func__, opp_def->freq,
			       opp_def->hwmod_name, i, r);
		} else {
			if (!opp_def->default_available)
				r = opp_disable(dev, opp_def->freq);
			if (r)
				dev_err(dev, "%s: disable %ld failed for %s "
					"[%d] result=%d\n",
					__func__, opp_def->freq,
					opp_def->hwmod_name, i, r);

			r  = omap_dvfs_register_device(dev,
				opp_def->voltdm_name, opp_def->clk_name);
			if (r)
				dev_err(dev, "%s:%s:err dvfs register %d %d\n",
					__func__, opp_def->hwmod_name, r, i);
		}
	}

	return 0;
}
