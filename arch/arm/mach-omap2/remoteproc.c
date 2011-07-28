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
#include "cm2_44xx.h"
#include "cm-regbits-44xx.h"

#define OMAP4430_CM_M3_M3_CLKCTRL (OMAP4430_CM2_BASE + OMAP4430_CM2_CORE_INST \
		+ OMAP4_CM_DUCATI_DUCATI_CLKCTRL_OFFSET)

#define L4_PERIPHERAL_L4CFG	(L4_44XX_BASE)
#define IPU_PERIPHERAL_L4CFG	0xAA000000

#define L4_PERIPHERAL_L4PER	0x48000000
#define IPU_PERIPHERAL_L4PER	0xA8000000

#define L4_PERIPHERAL_L4EMU	0x54000000
#define IPU_PERIPHERAL_L4EMU	0xB4000000

#define L3_IVAHD_CONFIG		0x5A000000
#define IPU_IVAHD_CONFIG	0xBA000000

#define L3_IVAHD_SL2		0x5B000000
#define IPU_IVAHD_SL2		0xBB000000

#define L3_TILER_MODE_0_1	0x60000000
#define IPU_TILER_MODE_0_1	0x60000000

#define L3_TILER_MODE_2		0x70000000
#define IPU_TILER_MODE_2	0x70000000

#define L3_TILER_MODE_3		0x78000000
#define IPU_TILER_MODE_3	0x78000000

#define L3_IVAHD_CONFIG		0x5A000000
#define IPU_IVAHD_CONFIG	0xBA000000

#define L3_IVAHD_SL2		0x5B000000
#define IPU_IVAHD_SL2		0xBB000000

#define IPU_MEM_TEXT		0x0
#define IPU_MEM_DATA		0x80000000
#define IPU_MEM_IPC		0xA0000000

/* TODO: Remove hardcoded RAM Addresses once we have the PA->VA lookup integrated.
 * IPC region should not be hard-coded. Text area is also not hard-coded since
 * VA to PA translation is not required. */
#define PHYS_MEM_DATA		0xB9800000

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
	{IPU_MEM_DATA, PHYS_MEM_DATA, SZ_1M * 96},
	{IPU_PERIPHERAL_L4CFG, L4_PERIPHERAL_L4CFG, SZ_16M},
	{IPU_PERIPHERAL_L4PER, L4_PERIPHERAL_L4PER, SZ_16M},
	{IPU_TILER_MODE_0_1, L3_TILER_MODE_0_1, SZ_256M},
	{IPU_TILER_MODE_2, L3_TILER_MODE_2, SZ_128M},
	{IPU_TILER_MODE_3, L3_TILER_MODE_3, SZ_128M},
	{IPU_IVAHD_CONFIG, L3_IVAHD_CONFIG, SZ_16M},
	{IPU_IVAHD_SL2, L3_IVAHD_SL2, SZ_16M},
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
		.idle_addr	= OMAP4430_CM_M3_M3_CLKCTRL,
		.idle_mask	= OMAP4430_STBYST_MASK,
		.suspend_addr	= 0xb98f02d8,
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

	paddr = omap_ipu_get_mempool_base(OMAP_RPROC_MEMPOOL_DYNAMIC);
	size = omap_ipu_get_mempool_size(OMAP_RPROC_MEMPOOL_DYNAMIC);
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
