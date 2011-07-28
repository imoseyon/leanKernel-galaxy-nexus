/*
 * Remote Processor - omap-specific bits
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

#ifndef _PLAT_REMOTEPROC_H
#define _PLAT_REMOTEPROC_H

#include <linux/remoteproc.h>
#include <plat/omap_device.h>

/*
 * struct omap_rproc_pdata - platform data for the omap rproc implementation
 *
 * @name: human readable name of the rproc, cannot exceed RPROC_MAN_NAME bytes
 * @iommu_name: iommu device we're behind of
 * @oh_name: omap hwmod device
 * @oh_name_opt: optional, secondary omap hwmod device
 * @firmware: name of firmware file to be loaded
 * @ops: platform-specific start/stop rproc handlers
 * @memory_maps: table of da-to-pa iommu memory maps
 */
struct omap_rproc_pdata {
	const char *name;
	const char *iommu_name;
	const char *oh_name;
	const char *oh_name_opt;
	const char *firmware;
	const struct rproc_ops *ops;
	const struct rproc_mem_entry *memory_maps;
	u32 idle_addr;
	u32 idle_mask;
	u32 suspend_addr;
	u32 suspend_mask;
	unsigned sus_timeout;
	char *sus_mbox_name;
};

enum omap_rproc_mempool_type {
	OMAP_RPROC_MEMPOOL_STATIC,
	OMAP_RPROC_MEMPOOL_DYNAMIC
};

#if defined(CONFIG_OMAP_REMOTE_PROC)
void omap_ipu_reserve_sdram_memblock(void);
u32 omap_ipu_get_mempool_size(enum omap_rproc_mempool_type type);
phys_addr_t omap_ipu_get_mempool_base(enum omap_rproc_mempool_type type);
void omap_ipu_set_static_mempool(u32 start, u32 size);
#else
static inline void omap_ipu_reserve_sdram_memblock(void) { }
static inline u32 omap_ipu_get_mempool_size(enum omap_rproc_mempool_type type)
{
	return 0;
}
static inline phys_addr_t omap_ipu_get_mempool_base(
					enum omap_rproc_mempool_type type)
{
	return 0;
}
static inline void omap_ipu_set_static_mempool(u32 start, u32 size) { }
#endif

int omap_rproc_deactivate(struct omap_device *od);
int omap_rproc_activate(struct omap_device *od);
#define OMAP_RPROC_DEFAULT_PM_LATENCY \
	.deactivate_func = omap_rproc_deactivate, \
	.activate_func = omap_rproc_activate, \
	.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST

#endif /* _PLAT_REMOTEPROC_H */
