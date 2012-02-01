/*
 * OMAP4 SMP source file. It contains platform specific fucntions
 * needed for the linux smp kernel.
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author:
 *      Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Platform file needed for the OMAP4 SMP. This file is based on arm
 * realview smp platform.
 * * Copyright (c) 2002 ARM Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/hrtimer.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/smp_scu.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>

#include "clockdomain.h"

/* SCU base address */
static void __iomem *scu_base;

static DEFINE_SPINLOCK(boot_lock);


void __iomem *omap4_get_scu_base(void)
{
	return scu_base;
}

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	u32 diag0_errata_flags = 0;
	/* Enable NS access to SMP bit for this CPU on HS devices */
	if (cpu_is_omap446x() || cpu_is_omap443x()) {
		if (omap_type() != OMAP2_DEVICE_TYPE_GP)
			omap4_secure_dispatcher(PPA_SERVICE_DEFAULT_POR_NS_SMP,
					FLAG_START_CRITICAL,
					0, 0, 0, 0, 0);
		else {
			diag0_errata_flags =
				omap4_get_diagctrl0_errata_flags();
			if (diag0_errata_flags)
				omap_smc1(HAL_DIAGREG_0, diag0_errata_flags);
		}

	}

	/*
	 * If any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	static struct clockdomain *cpu1_clkdm;
	static bool booted;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * Update the AuxCoreBoot0 with boot state for secondary core.
	 * omap_secondary_startup() routine will hold the secondary core till
	 * the AuxCoreBoot1 register is updated with cpu state
	 * A barrier is added to ensure that write buffer is drained
	 */
	omap_modify_auxcoreboot0(0x200, 0xfffffdff);
	flush_cache_all();
	smp_wmb();

	if (!cpu1_clkdm)
		cpu1_clkdm = clkdm_lookup("mpu1_clkdm");

	/*
	 * The SGI(Software Generated Interrupts) are not wakeup capable
	 * from low power states. This is known limitation on OMAP4 and
	 * needs to be worked around by using software forced clockdomain
	 * wake-up. To wakeup CPU1, CPU0 forces the CPU1 clockdomain to
	 * software force wakeup. After the wakeup, CPU1 restores its
	 * clockdomain hardware supervised mode.
	 * More details can be found in OMAP4430 TRM - Version J
	 * Section :
	 *	4.3.4.2 Power States of CPU0 and CPU1
	 */
	if (booted) {
		/*
		 * GIC distributor control register has changed between
		 * CortexA9 r1pX and r2pX. The Control Register secure
		 * banked version is now composed of 2 bits:
		 * bit 0 == Secure Enable
		 * bit 1 == Non-Secure Enable
		 * The Non-Secure banked register has not changed
		 * Because the ROM Code is based on the r1pX GIC, the CPU1
		 * GIC restoration will cause a problem to CPU0 Non-Secure SW.
		 * The workaround must be:
		 * 1) Before doing the CPU1 wakeup, CPU0 must disable
		 * the GIC distributor
		 * 2) CPU1 must re-enable the GIC distributor on
		 * it's wakeup path.
		 */
		if (!cpu_is_omap443x()) {
			local_irq_disable();
			gic_dist_disable();
		}

		clkdm_wakeup(cpu1_clkdm);

		if (!cpu_is_omap443x()) {
			while (gic_dist_disabled()) {
				udelay(1);
				cpu_relax();
			}
			gic_timer_retrigger();
			local_irq_enable();
		}

	} else {
		dsb_sev();
		booted = true;
	}

	/*
	 * Now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return 0;
}

static void __init wakeup_secondary(void)
{
	/*
	 * Write the address of secondary startup routine into the
	 * AuxCoreBoot1 where ROM code will jump and start executing
	 * on secondary core once out of WFE
	 * A barrier is added to ensure that write buffer is drained
	 */
	omap_auxcoreboot_addr(virt_to_phys(omap_secondary_startup));
	smp_wmb();

	/*
	 * Send a 'sev' to wake the secondary core from WFE.
	 * Drain the outstanding writes to memory
	 */
	dsb_sev();
	mb();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores;

	/* Never released */
	scu_base = ioremap(OMAP44XX_SCU_BASE, SZ_256);
	BUG_ON(!scu_base);

	ncores = scu_get_core_count(scu_base);

	/* sanity check */
	if (ncores > NR_CPUS) {
		printk(KERN_WARNING
		       "OMAP4: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU and wake up the secondary core using
	 * wakeup_secondary().
	 */
	scu_enable(scu_base);
	wakeup_secondary();
}
