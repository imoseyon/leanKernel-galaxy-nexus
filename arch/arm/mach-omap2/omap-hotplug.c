/*
 * OMAP4 SMP cpu-hotplug support
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author:
 *      Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Platform file needed for the OMAP4 SMP. This file is based on arm
 * realview smp platform.
 * Copyright (c) 2002 ARM Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>

#include <mach/omap4-common.h>
#include <mach/omap-wakeupgen.h>

#include "powerdomain.h"
#include "clockdomain.h"

int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

/*
 * platform-specific code to shutdown a CPU
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	unsigned int this_cpu;
	static struct clockdomain *cpu1_clkdm;

	if (!cpu1_clkdm)
		cpu1_clkdm = clkdm_lookup("mpu1_clkdm");

	flush_cache_all();
	dsb();

	/*
	 * we're ready for shutdown now, so do it
	 */
	if (omap_modify_auxcoreboot0(0x0, 0x200) != 0x0)
		pr_err("Secure clear status failed\n");

	for (;;) {
		/*
		 * Enter into low power state
		 * clear all interrupt wakeup sources
		 */
		omap_wakeupgen_irqmask_all(cpu, 1);
		gic_cpu_disable();
		omap4_enter_lowpower(cpu, PWRDM_POWER_OFF);
		this_cpu = hard_smp_processor_id();
		if (omap_read_auxcoreboot0() == this_cpu) {
			/*
			 * OK, proper wakeup, we're done
			 */
			omap_wakeupgen_irqmask_all(this_cpu, 0);
			gic_cpu_enable();

			/* Restore clockdomain to hardware supervised */
			clkdm_allow_idle(cpu1_clkdm);
			break;
		}
		pr_debug("CPU%u: spurious wakeup call\n", cpu);
	}
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
