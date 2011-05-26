/*
 * omap4-common.h: OMAP4 specific common header file
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Author:
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef OMAP_ARCH_OMAP4_COMMON_H
#define OMAP_ARCH_OMAP4_COMMON_H

#include <asm/proc-fns.h>

#ifndef __ASSEMBLER__
/*
 * wfi used in low power code. Directly opcode is used instead
 * of instruction to avoid mulit-omap build break
 */
#ifdef CONFIG_THUMB2_KERNEL
#define do_wfi() __asm__ __volatile__ ("wfi" : : : "memory")
#else
#define do_wfi()			\
		__asm__ __volatile__ (".word	0xe320f003" : : : "memory")
#endif

#ifdef CONFIG_CACHE_L2X0
extern void __iomem *omap4_get_l2cache_base(void);
#endif

#ifdef CONFIG_SMP
extern void __iomem *omap4_get_scu_base(void);
#else
static inline void __iomem *omap4_get_scu_base(void)
{
	return NULL;
}
#endif

extern void __iomem *omap4_get_gic_dist_base(void);
extern void __iomem *omap4_get_gic_cpu_base(void);
extern void __iomem *omap4_get_sar_ram_base(void);
extern void __init gic_init_irq(void);
extern void omap_smc1(u32 fn, u32 arg);

/*
 * Read MPIDR: Multiprocessor affinity register
 */
static inline unsigned int hard_smp_processor_id(void)
{
	unsigned int cpunum;

	asm volatile (
	"mrc	 p15, 0, %0, c0, c0, 5\n"
		: "=r" (cpunum));
	return cpunum &= 0x0f;
}

#ifdef CONFIG_SMP
/* Needed for secondary core boot */
extern void omap_secondary_startup(void);
extern u32 omap_modify_auxcoreboot0(u32 set_mask, u32 clear_mask);
extern void omap_auxcoreboot_addr(u32 cpu_addr);
extern u32 omap_read_auxcoreboot0(void);

#ifdef CONFIG_PM
extern int omap4_mpuss_init(void);
extern int omap4_enter_lowpower(unsigned int cpu, unsigned int power_state);
extern void omap4_cpu_suspend(unsigned int cpu, unsigned int save_state);
extern void omap4_cpu_resume(void);
#else
static inline int omap4_enter_lowpower(unsigned int cpu,
					unsigned int power_state)
{
	cpu_do_idle();
	return 0;
}

static inline int omap4_mpuss_init(void)
{
	return 0;
}

static inline void omap4_cpu_suspend(unsigned int cpu, unsigned int save_state)
{
}

static inline void omap4_cpu_resume(void)
{
}

#endif	/* CONFIG_PM */
#endif	/* CONFIG_SMP */
#endif /* __ASSEMBLER__ */
#endif /* OMAP_ARCH_OMAP4_COMMON_H */
