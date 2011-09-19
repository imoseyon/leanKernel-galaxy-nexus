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
 * struct omap_rproc_timers_info - optional timers for the omap rproc
 *
 * @id: timer id to use by the remoteproc
 * @odt: timer pointer
 */
struct omap_rproc_timers_info {
	int id;
	struct omap_dm_timer *odt;
};

/*
 * struct omap_rproc_pdata - platform data for the omap rproc implementation
 *
 * @name: human readable name of the rproc, cannot exceed RPROC_MAN_NAME bytes
 * @iommu_name: iommu device we're behind of
 * @oh_name: omap hwmod device
 * @oh_name_opt: optional, secondary omap hwmod device
 * @firmware: name of firmware file to be loaded
 * @clkdm_name: name of clock domain in which this device is located
 * @clkdm: clock domain in which this device is located
 * @ops: platform-specific start/stop rproc handlers
 * @memory_maps: table of da-to-pa iommu memory maps
 * @memory_pool: platform-specific pool data
 * @omap_rproc_timers_info: optional, timer(s) rproc can use
 */
struct omap_rproc_pdata {
	const char *name;
	const char *iommu_name;
	const char *oh_name;
	const char *oh_name_opt;
	const char *firmware;
	const char *clkdm_name;
	struct clockdomain *clkdm;
	const struct rproc_ops *ops;
	struct rproc_mem_pool *memory_pool;
	struct omap_rproc_timers_info *timers;
	u32 idle_addr;
	u32 idle_mask;
	u32 suspend_addr;
	u32 suspend_mask;
	unsigned sus_timeout;
	char *sus_mbox_name;
	u8 timers_cnt;
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

struct exc_regs {
	u32  r0;
	u32  r1;
	u32  r2;
	u32  r3;
	u32  r4;
	u32  r5;
	u32  r6;
	u32  r7;
	u32  r8;
	u32  r9;
	u32  r10;
	u32  r11;
	u32  r12;
	u32  sp;
	u32  lr;
	u32  pc;
	u32  psr;
	u32  ICSR; /* NVIC registers */
	u32  MMFSR;
	u32  BFSR;
	u32  UFSR;
	u32  HFSR;
	u32  DFSR;
	u32  MMAR;
	u32  BFAR;
	u32  AFSR;
};

static inline void remoteproc_fill_pt_regs(struct pt_regs *regs,
				struct exc_regs *xregs)
{
	regs->ARM_r0  = xregs->r0;
	regs->ARM_ORIG_r0  = xregs->r0;
	regs->ARM_r1  = xregs->r1;
	regs->ARM_r2  = xregs->r2;
	regs->ARM_r3  = xregs->r3;
	regs->ARM_r4  = xregs->r4;
	regs->ARM_r5  = xregs->r5;
	regs->ARM_r6  = xregs->r6;
	regs->ARM_r7  = xregs->r7;
	regs->ARM_r8  = xregs->r8;
	regs->ARM_r9  = xregs->r9;
	regs->ARM_r10 = xregs->r10;
	regs->ARM_fp  = xregs->r11;
	regs->ARM_ip  = xregs->r12;
	regs->ARM_sp  = xregs->sp;
	regs->ARM_lr  = xregs->lr;
	regs->ARM_pc  = xregs->pc;
	regs->ARM_cpsr = xregs->psr;
}

#endif /* _PLAT_REMOTEPROC_H */
