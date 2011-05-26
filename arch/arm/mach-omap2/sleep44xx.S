/*
 * OMAP44xx CPU low power powerdown and powerup code.
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Written by Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>
#include <asm/system.h>
#include <asm/smp_scu.h>
#include <asm/memory.h>

#include <plat/omap44xx.h>
#include <mach/omap4-common.h>
#include <asm/hardware/cache-l2x0.h>

#include "omap4-sar-layout.h"

#ifdef CONFIG_SMP

/* Masks used for MMU manipulation */
#define TTRBIT_MASK				0xffffc000
#define TABLE_INDEX_MASK			0xfff00000
#define TABLE_ENTRY				0x00000c02
#define CACHE_DISABLE_MASK			0xffffe7fb
#define TABLE_ADDRESS_OFFSET			0x04
#define CR_VALUE_OFFSET				0x08

/*
 * =============================
 * == CPU suspend entry point ==
 * =============================
 *
 * void omap4_cpu_suspend(unsigned int cpu, unsigned int save_state)
 *
 * This function code saves the CPU context and performs the CPU
 * power down sequence. Calling WFI effectively changes the CPU
 * power domains states to the desired target power state.
 *
 * @cpu : contains cpu id (r0)
 * @save_state : contains context save state (r1)
 *	0 - No context lost
 * 	1 - CPUx L1 and logic lost: MPUSS CSWR
 * 	2 - CPUx L1 and logic lost + GIC lost: MPUSS OSWR
 *	3 - CPUx L1 and logic lost + GIC + L2 lost: MPUSS OFF
 * @return: This function never returns for CPU OFF and DORMANT power states.
 * Post WFI, CPU transitions to DORMANT or OFF power state and on wake-up
 * from this follows a full CPU reset path via ROM code to CPU restore code.
 * It returns to the caller for CPU INACTIVE and ON power states or in case
 * CPU failed to transition to targeted OFF/DORMANT state.
 */

ENTRY(omap4_cpu_suspend)
	stmfd	sp!, {r0-r12, lr}		@ Save registers on stack
	cmp	r1, #0x0
	beq	do_WFI				@ Nothing to save, jump to WFI
	mov	r5, r0
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	ands	r5, r5, #0x0f
	orreq	r8, r8, #CPU0_SAVE_OFFSET
	orrne	r8, r8, #CPU1_SAVE_OFFSET

	/*
	 * Save only needed CPU CP15 registers. VFP, breakpoint,
	 * performance monitor registers are not saved. Generic
	 * code suppose to take care of those.
	 */
	mov	r4, sp				@ Store sp
	mrs	r5, spsr			@ Store spsr
	mov	r6, lr				@ Store lr
	stmia	r8!, {r4-r6}

	/* c1 and c2 registers */
	mrc	p15, 0, r4, c1, c0, 2		@ CPACR
	mrc	p15, 0, r5, c2, c0, 0		@ TTBR0
	mrc	p15, 0, r6, c2, c0, 1		@ TTBR1
	mrc	p15, 0, r7, c2, c0, 2		@ TTBCR
	stmia	r8!, {r4-r7}

	/* c3 and c10 registers */
	mrc	p15, 0, r4, c3, c0, 0		@ DACR
	mrc	p15, 0, r5, c10, c2, 0		@ PRRR
	mrc	p15, 0, r6, c10, c2, 1		@ NMRR
	stmia	r8!,{r4-r6}

	/* c12, c13 and CPSR registers */
	mrc	p15, 0, r4, c13, c0, 1		@ Context ID
	mrc	p15, 0, r5, c13, c0, 2		@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0		@ Secure or NS VBAR
	mrs	r7, cpsr			@ Store CPSR
	stmia	r8!, {r4-r7}

	/* c1 control register */
	mrc	p15, 0, r4, c1, c0, 0		@ Save control register
	stmia	r8!, {r4}

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	bl	v7_flush_dcache_all

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)		@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	bl	v7_flush_dcache_all

	/*
	 * Switch the CPU from Symmetric Multiprocessing (SMP) mode
	 * to AsymmetricMultiprocessing (AMP) mode by programming
	 * the SCU power status to DORMANT or OFF mode.
	 * This enables the CPU to be taken out of coherency by
	 * preventing the CPU from receiving cache, TLB, or BTB
	 * maintenance operations broadcast by other CPUs in the cluster.
	 */
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	ldreq	r1, [r8, #SCU_OFFSET0]
	ldrne	r1, [r8, #SCU_OFFSET1]
	bl	omap4_get_scu_base
	bl     scu_power_mode

do_WFI:
	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state.
	 */
	wfi					@ Wait For Interrupt

	/*
	 * CPU is here when it failed to enter OFF/DORMANT or
	 * no low power state was attempted.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	tst	r0, #(1 << 2)			@ Check C bit enabled?
	orreq	r0, r0, #(1 << 2)		@ Enable the C bit
	mcreq	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Ensure the CPU power state is set to NORMAL in
	 * SCU power state so that CPU is back in coherency.
	 * In non-coherent mode CPU can lock-up and lead to
	 * system deadlock.
	 */
	bl	omap4_get_scu_base
	mov	r1, #SCU_PM_NORMAL
	bl     scu_power_mode
	isb
	dsb

	ldmfd	sp!, {r0-r12, pc}		@ Restore regs and return
ENDPROC(omap4_cpu_suspend)

/*
 * ============================
 * == CPU resume entry point ==
 * ============================
 *
 * void omap4_cpu_resume(void)
 *
 * ROM code jumps to this function while waking up from CPU
 * OFF or DORMANT state. Physical address of the function is
 * stored in the SAR RAM while entering to OFF or DORMANT mode.
 */

ENTRY(omap4_cpu_resume)
	/*
	 * Check the wakeup cpuid and use appropriate
	 * SAR BANK location for context restore.
	 */
	ldr	r3, =OMAP44XX_SAR_RAM_BASE
	mov	r1, #0
	mcr	p15, 0, r1, c7, c5, 0		@ Invalidate L1 I
	mrc	p15, 0, r0, c0, c0, 5		@ MPIDR
	ands	r0, r0, #0x0f
	orreq	r3, r3, #CPU0_SAVE_OFFSET
	orrne	r3, r3, #CPU1_SAVE_OFFSET

	/* Restore cp15 registers */
	ldmia	r3!, {r4-r6}
	mov	sp, r4				@ Restore sp
	msr	spsr_cxsf, r5			@ Restore spsr
	mov	lr, r6				@ Restore lr

	/* c1 and c2 registers */
	ldmia	r3!, {r4-r7}
	mcr	p15, 0, r4, c1, c0, 2		@ CPACR
	mcr	p15, 0, r5, c2, c0, 0		@ TTBR0
	mcr	p15, 0, r6, c2, c0, 1		@ TTBR1
	mcr	p15, 0, r7, c2, c0, 2		@ TTBCR

	/* c3 and c10 registers */
	ldmia	r3!,{r4-r6}
	mcr	p15, 0, r4, c3, c0, 0		@ DACR
	mcr	p15, 0, r5, c10, c2, 0		@ PRRR
	mcr	p15, 0, r6, c10, c2, 1		@ NMRR

	/* c12, c13 and CPSR registers */
	ldmia	r3!,{r4-r7}
	mcr	p15, 0, r4, c13, c0, 1		@ Context ID
	mcr	p15, 0, r5, c13, c0, 2		@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0		@ Secure or NS VBAR
	msr	cpsr, r7			@ store cpsr

	/*
	 * Enabling MMU here. Page entry needs to be altered
	 * to create temporary 1:1 map and then resore the entry
	 * ones MMU is enabled
	 */
	mrc	p15, 0, r7, c2, c0, 2		@ Read TTBRControl
	and	r7, #0x7			@ Extract N (0:2) to decide
	cmp	r7, #0x0			@ TTBR0/TTBR1
	beq	use_ttbr0
ttbr_error:
	b	ttbr_error			@ Only N = 0 supported
use_ttbr0:
	mrc	p15, 0, r2, c2, c0, 0		@ Read TTBR0
	ldr	r5, =TTRBIT_MASK
	and	r2, r5
	mov	r4, pc
	ldr	r5, =TABLE_INDEX_MASK
	and	r4, r5				@ r4 = 31 to 20 bits of pc
	ldr	r1, =TABLE_ENTRY
	add	r1, r1, r4			@ r1 has value of table entry
	lsr	r4, #18				@ Address of table entry
	add	r2, r4				@ r2 - location to be modified

	/* Ensure the modified entry makes it to main memory */
#ifdef CONFIG_CACHE_L2X0
	ldr	r5, =OMAP44XX_L2CACHE_BASE
	str	r2, [r5, #L2X0_CLEAN_INV_LINE_PA]
wait_l2:
	ldr	r0, [r5, #L2X0_CLEAN_INV_LINE_PA]
	ands	r0, #1
	bne	wait_l2
#endif

	/* Storing previous entry of location being modified */
	ldr     r5, =OMAP44XX_SAR_RAM_BASE
	ldr	r4, [r2]
	str	r4, [r5, #MMU_OFFSET]		@ Modify the table entry
	str	r1, [r2]

	/*
	 * Storing address of entry being modified
	 * It will be restored after enabling MMU
	 */
	ldr     r5, =OMAP44XX_SAR_RAM_BASE
	orr	r5, r5, #MMU_OFFSET
	str	r2, [r5, #TABLE_ADDRESS_OFFSET]
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4		@ Flush prefetch buffer
	mcr	p15, 0, r0, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r0, c8, c5, 0		@ Invalidate ITLB
	mcr	p15, 0, r0, c8, c6, 0		@ Invalidate DTLB

	/*
	 * Restore control register  but don't enable Data caches here.
	 * Caches will be enabled after restoring MMU table entry.
	 */
	ldmia	r3!, {r4}
	str	r4, [r5, #CR_VALUE_OFFSET]	@ Store previous value of CR
	ldr	r2, =CACHE_DISABLE_MASK
	and	r4, r2
	mcr	p15, 0, r4, c1, c0, 0
	isb
	dsb
	ldr	r0, =mmu_on_label
	bx	r0
mmu_on_label:
	/* Set up the per-CPU stacks */
	bl	cpu_init

	/*
	 * Restore the MMU table entry that was modified for
	 * enabling MMU.
	 */
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	orr	r8, r8, #MMU_OFFSET		@ Get address of entry that..
	ldr	r2, [r8, #TABLE_ADDRESS_OFFSET]	@ was modified
	ldr	r3, =local_va2pa_offet
	add	r2, r2, r3
	ldr	r0, [r8]			@ Get the previous value..
	str	r0, [r2]			@ which needs to be restored
	mov	r0, #0
	mcr	p15, 0, r0, c7, c1, 6		@ flush TLB and issue barriers
	mcr	p15, 0, r0, c7, c5, 4		@ Flush prefetch buffer
	mcr	p15, 0, r0, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r0, c8, c5, 0		@ Invalidate ITLB
	mcr	p15, 0, r0, c8, c6, 0		@ Invalidate DTLB
	dsb
	isb
	ldr	r0, [r8, #CR_VALUE_OFFSET]	@ Restore the Control register
	mcr     p15, 0, r0, c1, c0, 0		@ with caches enabled.
	isb

	ldmfd	sp!, {r0-r12, pc}		@ restore regs and return

	.equ	local_va2pa_offet, (PLAT_PHYS_OFFSET + PAGE_OFFSET)

ENDPROC(omap4_cpu_resume)

#endif
