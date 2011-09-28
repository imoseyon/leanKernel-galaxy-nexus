/*
 * Remote processor machine-specific module for OMAP4
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/remoteproc.h>
#include <linux/memblock.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/remoteproc.h>
#include <plat/dsp.h>
#include <plat/io.h>
#include "cm2_44xx.h"
#include "cm-regbits-44xx.h"

#define OMAP4430_CM_M3_M3_CLKCTRL (OMAP4430_CM2_BASE + OMAP4430_CM2_CORE_INST \
		+ OMAP4_CM_DUCATI_DUCATI_CLKCTRL_OFFSET)

static struct omap_rproc_timers_info ipu_timers[] = {
	{ .id = 3 },
	{ .id = 4 },
#ifdef CONFIG_REMOTEPROC_WATCHDOG
	{ .id = 9 },
	{ .id = 11 },
#endif
};

static struct omap_rproc_pdata omap4_rproc_data[] = {
	{
		.name		= "dsp",
		.iommu_name	= "tesla",
		.firmware	= "tesla-dsp.bin",
		.oh_name	= "dsp_c0",
		.clkdm_name	= "dsp_clkdm",
	},
	{
		.name		= "ipu",
		.iommu_name	= "ducati",
		.firmware	= "ducati-m3.bin",
		.oh_name	= "ipu_c0",
		.oh_name_opt	= "ipu_c1",
		.clkdm_name	= "ducati_clkdm",
		.timers		= ipu_timers,
		.timers_cnt	= ARRAY_SIZE(ipu_timers),
		.idle_addr	= OMAP4430_CM_M3_M3_CLKCTRL,
		.idle_mask	= OMAP4430_STBYST_MASK,
		.suspend_addr	= 0xb3bf02d8,
		.suspend_mask	= ~0,
		.sus_timeout	= 5000,
		.sus_mbox_name	= "mailbox-1",
	},
};

static struct omap_device_pm_latency omap_rproc_latency[] = {
	{
		OMAP_RPROC_DEFAULT_PM_LATENCY,
	},
};

static struct rproc_mem_pool *omap_rproc_get_pool(const char *name)
{
	struct rproc_mem_pool *pool = NULL;

	/* check for ipu currently. dsp will be handled later */
	if (!strcmp("ipu", name)) {
		phys_addr_t paddr1 = omap_ipu_get_mempool_base(
						OMAP_RPROC_MEMPOOL_STATIC);
		phys_addr_t paddr2 = omap_ipu_get_mempool_base(
						OMAP_RPROC_MEMPOOL_DYNAMIC);
		u32 len1 = omap_ipu_get_mempool_size(OMAP_RPROC_MEMPOOL_STATIC);
		u32 len2 = omap_ipu_get_mempool_size(OMAP_RPROC_MEMPOOL_DYNAMIC);

		if (!paddr1 && !paddr2) {
			pr_err("no carveout memory available at all for "
				"remotproc\n");
			return pool;
		}
		if (!paddr1 || !len1)
			pr_warn("static memory is unavailable: 0x%x, 0x%x\n",
				paddr1, len1);
		if (!paddr2 || !len2)
			pr_warn("carveout memory is unavailable: 0x%x, 0x%x\n",
				paddr2, len2);

		pool = kzalloc(sizeof(*pool), GFP_KERNEL);
		if (pool) {
			pool->st_base = paddr1;
			pool->st_size = len1;
			pool->mem_base = paddr2;
			pool->mem_size = len2;
			pool->cur_base = paddr2;
			pool->cur_size = len2;
		}
	}

	return pool;
}

static int __init omap_rproc_init(void)
{
	const char *pdev_name = "omap-rproc";
	struct omap_hwmod *oh[2];
	struct omap_device *od;
	int i, ret = 0, oh_count;

	/* names like ipu_cx/dsp_cx might show up on other OMAPs, too */
	if (!cpu_is_omap44xx())
		return 0;

	for (i = 0; i < ARRAY_SIZE(omap4_rproc_data); i++) {
		const char *oh_name = omap4_rproc_data[i].oh_name;
		const char *oh_name_opt = omap4_rproc_data[i].oh_name_opt;
		oh_count = 0;

		oh[0] = omap_hwmod_lookup(oh_name);
		if (!oh[0]) {
			pr_err("could not look up %s\n", oh_name);
			continue;
		}
		oh_count++;

		if (oh_name_opt) {
			oh[1] = omap_hwmod_lookup(oh_name_opt);
			if (!oh[1]) {
				pr_err("could not look up %s\n", oh_name_opt);
				continue;
			}
			oh_count++;
		}

		omap4_rproc_data[i].memory_pool =
				omap_rproc_get_pool(omap4_rproc_data[i].name);
		od = omap_device_build_ss(pdev_name, i, oh, oh_count,
					&omap4_rproc_data[i],
					sizeof(struct omap_rproc_pdata),
					omap_rproc_latency,
					ARRAY_SIZE(omap_rproc_latency),
					false);
		if (IS_ERR(od)) {
			pr_err("Could not build omap_device for %s:%s\n",
							pdev_name, oh_name);
			ret = PTR_ERR(od);
		}
	}

	return ret;
}
/* must be ready in time for device_initcall users */
subsys_initcall(omap_rproc_init);
