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
#include <linux/remoteproc.h>
#include <linux/memblock.h>

#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/remoteproc.h>
#include <plat/dsp.h>
#include <plat/io.h>

#define L4_PERIPHERAL_L4CFG	(L4_44XX_BASE)
#define IPU_PERIPHERAL_L4CFG	0xAA000000

#define IPU_MEM_TEXT		0x0
#define IPU_MEM_DATA		0x80000000
#define IPU_MEM_IPC		0xA0000000

/*
 * Memory mappings for the remote M3 subsystem
 *
 * Don't change the device addresses (first parameter), otherwise you'd have
 * to update the firmware (BIOS image) accordingly.
 *
 * A 0 physical address (second parameter) means this physical region should
 * be dynamically carved out at boot time.
 */
static struct rproc_mem_entry ipu_memory_maps[] = {
	{IPU_MEM_IPC, 0, SZ_1M}, /* keep this IPC region first */
	{IPU_MEM_TEXT, 0, SZ_4M},
	{IPU_MEM_DATA, 0, SZ_32M},
	{IPU_PERIPHERAL_L4CFG, L4_PERIPHERAL_L4CFG, SZ_16M},
	{ }
};

static struct omap_rproc_pdata omap4_rproc_data[] = {
	{
		.name		= "dsp",
		.iommu_name	= "tesla",
		.firmware	= "tesla-dsp.bin",
		.oh_name	= "dsp_c0",
	},
	{
		.name		= "ipu",
		.iommu_name	= "ducati",
		.firmware	= "ducati-m3.bin",
		.oh_name	= "ipu_c0",
		.oh_name_opt	= "ipu_c1",
		.memory_maps	= ipu_memory_maps,
	},
};

static struct omap_device_pm_latency omap_rproc_latency[] = {
	{
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

static int __init omap_rproc_init(void)
{
	const char *pdev_name = "omap-rproc";
	struct omap_hwmod *oh[2];
	struct omap_device *od;
	int i, ret = 0, oh_count;
	phys_addr_t paddr, size;

	/* names like ipu_cx/dsp_cx might show up on other OMAPs, too */
	if (!cpu_is_omap44xx())
		return 0;

	paddr = omap_dsp_get_mempool_base();
	size = omap_dsp_get_mempool_size();
	if (!paddr || !size) {
		pr_warn("carveout memory is unavailable: 0x%x, 0x%x\n",
								paddr, size);
		return -ENOMEM;
	}

	/* dynamically allocate carveout memory as required by the ipu */
	for (i = 0; i < ARRAY_SIZE(ipu_memory_maps); i++) {
		struct rproc_mem_entry *me = &ipu_memory_maps[i];

		if (!me->pa && me->size) {
			if (me->size > size) {
				pr_warn("out of carveout memory\n");
				return -ENOMEM;
			}

			me->pa = paddr;
			paddr += me->size;
			size -= me->size;

			pr_info("0x%x bytes at 0x%x %d\n", me->size, me->pa, i);
		}
	}

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
