/*
 * omap iommu: omap device registration
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/err.h>

#include <plat/iommu.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>


struct iommu_device {
	resource_size_t base;
	int irq;
	struct iommu_platform_data pdata;
	struct resource res[2];
};
static struct iommu_platform_data *devices_data;
static int num_iommu_devices;

#ifdef CONFIG_ARCH_OMAP3
static struct iommu_platform_data omap3_devices_data[] = {
	{
		.name = "isp",
		.oh_name = "isp",
		.nr_tlb_entries = 8,
		.da_start = 0x0,
		.da_end = 0xFFFFF000,
	},
#if defined(CONFIG_OMAP_IOMMU_IVA2)
	{
		.name = "iva2",
		.oh_name = "dsp",
		.nr_tlb_entries = 32,
		.da_start = 0x11000000,
		.da_end = 0xFFFFF000,
	},
#endif
};
#define NR_OMAP3_IOMMU_DEVICES ARRAY_SIZE(omap3_devices_data)
#else
#define omap3_devices_data	NULL
#define NR_OMAP3_IOMMU_DEVICES	0
#endif

#ifdef CONFIG_ARCH_OMAP4
static struct iommu_platform_data omap4_devices_data[] = {
	{
		.name = "ducati",
		.oh_name = "ipu",
		.nr_tlb_entries = 32,
		.da_start = 0x0,
		.da_end = 0xFFFFF000,
	},
	{
		.name = "tesla",
		.oh_name = "dsp",
		.nr_tlb_entries = 32,
		.da_start = 0x0,
		.da_end = 0xFFFFF000,
	},
};
#define NR_OMAP4_IOMMU_DEVICES ARRAY_SIZE(omap4_devices_data)
#else
#define omap4_devices_data	NULL
#define NR_OMAP4_IOMMU_DEVICES	0
#endif

static struct omap_device_pm_latency omap_iommu_latency[] = {
	[0] = {
		.deactivate_func = omap_device_idle_hwmods,
		.activate_func	 = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

int iommu_get_plat_data_size(void)
{
	return num_iommu_devices;
}
EXPORT_SYMBOL(iommu_get_plat_data_size);

struct iommu_platform_data *iommu_get_device_data(void)
{
	return devices_data;
}

static int __init omap_iommu_init(void)
{
	int i, ohl_cnt;
	struct omap_hwmod *oh;
	struct omap_device *od;
	struct omap_device_pm_latency *ohl;

	if (cpu_is_omap34xx()) {
		devices_data = omap3_devices_data;
		num_iommu_devices = NR_OMAP3_IOMMU_DEVICES;
	} else if (cpu_is_omap44xx()) {
		devices_data = omap4_devices_data;
		num_iommu_devices = NR_OMAP4_IOMMU_DEVICES;
	} else
		return -ENODEV;

	ohl = omap_iommu_latency;
	ohl_cnt = ARRAY_SIZE(omap_iommu_latency);

	for (i = 0; i < num_iommu_devices; i++) {
		struct iommu_platform_data *data = &devices_data[i];

		oh = omap_hwmod_lookup(data->oh_name);
		data->io_base = oh->_mpu_rt_va;
		data->irq = oh->mpu_irqs[0].irq;

		if (!oh) {
			pr_err("%s: could not look up %s\n", __func__,
							data->oh_name);
			continue;
		}
		od = omap_device_build("omap-iommu", i, oh,
					data, sizeof(*data),
					ohl, ohl_cnt, false);
		WARN(IS_ERR(od), "Could not build omap_device"
				"for %s %s\n", "omap-iommu", data->oh_name);
	}
	return 0;
}
module_init(omap_iommu_init);

MODULE_AUTHOR("Hiroshi DOYU");
MODULE_AUTHOR("Hari Kanigeri");
MODULE_DESCRIPTION("omap iommu: omap device registration");
MODULE_LICENSE("GPL v2");
