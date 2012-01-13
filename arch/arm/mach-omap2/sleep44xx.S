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
#include <asm/hardware/cache-l2x0.h>

#include <plat/omap44xx.h>
#include <mach/omap4-common.h>

#include "omap4-sar-layout.h"

#ifdef CONFIG_SMP

/* Masks used for MMU manipulation */
#define TTRBIT_MASK				0xffffc000
#define TABLE_INDEX_MASK			0xfff00000
#define TABLE_ENTRY				0x00000c02
#define CACHE_DISABLE_MASK			0xffffe7fb
#define TABLE_ADDRESS_OFFSET			0x04
#define CR_VALUE_OFFSET				0x08
#define SCU_POWER_SECURE_INDEX			0x108


/*
 * Macro to call PPA svc when MMU is OFF
 * Caller must setup r0 and r3 before calling this macro
 * @r0: PPA service ID
 * @r3: Pointer to params
*/
.macro LM_CALL_PPA_SERVICE_PA
	mov	r1, #0x0        @ Process ID
	mov	r2, #0x4	@ Flag
	mov	r6, #0xff
	mov	r12, #0x00      @ Secure Service ID
	dsb
	smc     #0
.endm

/*
 * To load POR which was saved in SAR RAM
 */
POR_params:
.word 1, 0


ppa_zero_params:
	.word		0x0

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
	mov	r6, r1
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	ands	r5, r5, #0x0f
	streq	r6, [r8, #L2X0_SAVE_OFFSET0]	@ Store save state
	strne   r6, [r8, #L2X0_SAVE_OFFSET1]
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

	bl	omap4_get_sar_ram_base
	ldr	r9, [r0, #OMAP_TYPE_OFFSET]
	cmp	r9, #0x1			@ Check for HS device
	bne	skip_secure_l1_flush
	mov	r0, #SCU_PM_NORMAL
	mov	r1, #0xFF			@ clean seucre L1
	stmfd   r13!, {r4-r12, r14}
	ldr	r12, =SCU_POWER_SECURE_INDEX
	dsb
	smc	#0
	dsb
	ldmfd   r13!, {r4-r12, r14}
skip_secure_l1_flush:

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
	ldr	r9, [r8, #OMAP_TYPE_OFFSET]
	cmp	r9, #0x1			@ Check for HS device
	bne	scu_gp_set
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	ldreq	r0, [r8, #SCU_OFFSET0]
	ldrne	r0, [r8, #SCU_OFFSET1]
	mov	r1, #0x00			@ Secure L1 is clean already
	stmfd   r13!, {r4-r12, r14}
	ldr	r12, =SCU_POWER_SECURE_INDEX
	dsb
	smc	#0
	dsb
	ldmfd   r13!, {r4-r12, r14}
	b	skip_scu_gp_set
scu_gp_set:
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	ldreq	r1, [r8, #SCU_OFFSET0]
	ldrne	r1, [r8, #SCU_OFFSET1]
	bl	omap4_get_scu_base
	bl     scu_power_mode
skip_scu_gp_set:
	isb
	dsb

	mrc	p15, 0, r0, c1, c1, 2		@Read NSACR data
	tst	r0, #(1 << 18)
	mrcne	p15, 0, r0, c1, c0, 1
	bicne	r0, r0, #(1 << 6)
	mcrne	p15, 0, r0, c1, c0, 1
	isb


#ifdef CONFIG_CACHE_L2X0
	/*
	 * Clean and invalidate the L2 cache.
	 * Common cache-l2x0.c functions can't be used here since it
	 * uses spinlocks. We are out of coherency here with data cache
	 * disabled. The spinlock implementation uses exclusive load/store
	 * instruction which can fail without data cache being enabled.
	 * OMAP4 hardware doesn't support exclusive monitor which can
	 * overcome exclusive access issue. Because of this, CPU can
	 * lead to deadlock.
	 */
l2x_clean_inv:
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	mrc	p15, 0, r5, c0, c0, 5           @ Read MPIDR
	ands	r5, r5, #0x0f
	ldreq	r0, [r8, #L2X0_SAVE_OFFSET0]
	ldrne	r0, [r8, #L2X0_SAVE_OFFSET1]
	cmp	r0, #3
	bne	do_WFI
#ifdef CONFIG_PL310_ERRATA_727915
	mov	r0, #0x03
	mov	r12, #0x100
	dsb
	smc	#0
	dsb
#endif
	bl	omap4_get_l2cache_base
	mov	r2, r0
	ldr	r0, =0xffff
	str	r0, [r2, #L2X0_CLEAN_INV_WAY]
wait:
	ldr	r0, [r2, #L2X0_CLEAN_INV_WAY]
	ands    r0, r0, #0xff
	bne	wait
#ifdef CONFIG_PL310_ERRATA_727915
	mov	r0, #0x00
	mov	r12, #0x100
	dsb
	smc	#0
	dsb
#endif
l2x_sync:
	bl	omap4_get_l2cache_base
	mov	r2, r0
	mov	r0, #0x0
	str	r0, [r2, #L2X0_CACHE_SYNC]
sync:
	ldr	r0, [r2, #L2X0_CACHE_SYNC]
	ands	r0, r0, #0x1
	bne	sync
#endif

do_WFI:
	bl	omap_do_wfi

	/*
	 * CPU is here when it failed to enter OFF/DORMANT or
	 * no low power state was attempted.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	tst	r0, #(1 << 2)			@ Check C bit enabled?
	orreq	r0, r0, #(1 << 2)		@ Enable the C bit
	mcreq	p15, 0, r0, c1, c0, 0
	isb

	/* Enable SMP bit if it's being disabled */
	mrc	p15, 0, r0, c1, c0, 1
	tst	r0, #(1 << 6)			@ Check SMP bit enabled?
	orreq	r0, r0, #(1 << 6)
	mcreq	p15, 0, r0, c1, c0, 1
	isb

	/*
	 * Ensure the CPU power state is set to NORMAL in
	 * SCU power state so that CPU is back in coherency.
	 * In non-coherent mode CPU can lock-up and lead to
	 * system deadlock.
	 */
	bl	omap4_get_sar_ram_base
	mov	r8, r0
	ldr	r9, [r8, #OMAP_TYPE_OFFSET]
	cmp	r9, #0x1			@ Check for HS device
	bne	scu_gp_clear
	mov	r0, #SCU_PM_NORMAL
	mov	r1, #0x00
	stmfd   r13!, {r4-r12, r14}
	ldr	r12, =SCU_POWER_SECURE_INDEX
	dsb
	smc	#0
	dsb
	ldmfd   r13!, {r4-r12, r14}
	b	skip_scu_gp_clear
scu_gp_clear:
	bl	omap4_get_scu_base
	mov	r1, #SCU_PM_NORMAL
	bl     scu_power_mode
skip_scu_gp_clear:
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
	 * CPU1 must check if CPU0 is alive/awaken.
	 * if PL310 is OFF, MPUSS was OFF and CPU0 is still off,
	 * CPU1 must go to sleep and wait for CPU0.
	 * CPU0 is needed for any PPA API to work.
	 */
	mrc     p15, 0, r0, c0, c0, 5	@ Get cpuID
	ands    r0, r0, #0x0f		@ Continue boot if CPU0
	beq	continue_boot
	ldr     r8, =OMAP44XX_SAR_RAM_BASE
	ldr	r9, [r8, #OMAP_TYPE_OFFSET]
	cmp	r9, #0x1		@ Check for HS device
	bne	continue_boot		@ Continue on GP devcies
	ldr	r2, =OMAP44XX_L2CACHE_BASE
	ldr	r0, [r2, #L2X0_CTRL]
	and	r0, #0x0f
	cmp	r0, #1				@ is CPU0 already UP?
	beq	ppa_cp15_cpu1_configure		@ CPU1 HS go to next stage
	/*
	 * When CPU1 is released to control of HLOS in the case of OSWR
	 * and OFF mode, PPA below v1.7.3[1] is not performing all
	 * Memory coherency and TLB operations required.
	 *
	 * A WA to recover cleanly from this scenario is to switch CPU1 back to
	 * previous OFF state. This forces a reset of CPU1, which in turn
	 * forces CPU1 not to override MMU descriptors already in place in
	 * internal RAM setup by CPU0. CPU1 will also sync to the in-place
	 * descriptors on the next wakeup. CPU1 wakeup is done by
	 * later kernel subsystems depending on suspend or cpuidle path
	 * being exercised.
	 * NOTE - for OSWR, state provided is 2, and for OFF, state is 3,
	 * Since the bug impacts OFF and OSWR, we need to force a 0x3 to
	 * shut off CPU1
	 *
	 * Since many distributions may not be able to update PPA OR would like
	 * to support platforms with older PPA, we provide a config option.
	 * This is simpler and makes the current code remain cleaner in
	 * comparison to a flag based handling in CPU1 recovery for
	 * board + PPA revision combinations.
	 *
	 * Having this config option enabled even on platforms with fixed PPA
	 * should not impact stability, however, ability to make CPU1 available
	 * for operations a little earlier is curtailed.
	 *
	 * Foot note [1]:
	 * v1.7.3 is the official TI PPA version. Custom PPA could have
	 * the relevant changes ported over to it.
	 */
#ifdef CONFIG_OMAP4_PPA_CPU1_ONLINE_BUG
	mov	r0, #0x03		@ target CPU1 to OFF(mpusspd=OSWR/OFF)
	mov	r1, #0x00		@ Secure L1 is already clean
	ldr	r12, =SCU_POWER_SECURE_INDEX
	dsb
	smc	#0

	isb				@ Necessary barriers before wfi
	dsb
	dmb
	wfi				@ wait for interrupt
	nop
	nop

	/*
	 * IF we came out of WFI immediately, something unknown happend.
	 * Fall through AND loop back to the checks. Failing which retry WFI.
	 */
#endif
	/*
	 * CPU0 and CPU1 are release together from OFF mode, however,
	 * CPU0 can be busy doing restore operations while waking
	 * from OFF mode, However, for many PPA services we need
	 * CPU0, so, we ask CPU1 to loop back to stagger CPU1 behind CPU0
	 */
	b omap4_cpu_resume

ppa_cp15_cpu1_configure:
	/*
	 * Configure CP15 for CPU1 on HS devices:
	 * In HS devices CPU0's CP15 is configured at wakeup by PPA, CPU1 must
	 * call PPA to configure it.
	 * In 4430 devices CPU1 this call also enables the access to SMP bit,
	 * on 4460 devices, CPU1 will have SMP bit access by default.
	 */
	mov     r0, #PPA_SERVICE_DEFAULT_POR_NS_SMP
	adr	r3, ppa_zero_params		@ Pointer to parameters
	LM_CALL_PPA_SERVICE_PA
	isb
	dsb
	cmp	r0, #0x0			@ API returns 0 on success.
	bne	ppa_cp15_cpu1_configure		@ retry if we did succeed

	/* Fall through to continue with boot */

continue_boot:

#ifdef CONFIG_CACHE_L2X0
	/*
	 * Restore the L2 AUXCTRL and enable the L2 cache.
	 * 0x109 =  Program the L2X0 AUXCTRL
	 * 0x102 =  Enable the L2 using L2X0 CTRL
	 * register r0 contains value to be programmed.
	 * L2 cache is already invalidate by ROM code as part
	 * of MPUSS OFF wakeup path.
	 */
	ldr	r2, =OMAP44XX_L2CACHE_BASE
	ldr	r0, [r2, #L2X0_CTRL]
	and	r0, #0x0f
	cmp	r0, #1
	beq	skip_l2en			@ Skip if already enabled

check_por:
	ldr	r0, =OMAP44XX_SAR_RAM_BASE  @ Check DEVICE type
	ldr	r1, [r0, #OMAP_TYPE_OFFSET]
	cmp	r1, #0x1                   @ Check for HS device
	bne	skip_por
	ldr	r0, =PPA_SERVICE_PL310_POR @ Setup PPA HAL call
	ldr	r1, =OMAP44XX_SAR_RAM_BASE
	ldr	r4, [r1, #L2X0_PREFETCHCTRL_OFFSET]
	adr	r3, POR_params
	str	r4, [r3, #0x04]
	LM_CALL_PPA_SERVICE_PA
skip_por:
	ldr	r3, =OMAP44XX_SAR_RAM_BASE
	ldr	r0, [r3, #L2X0_AUXCTRL_OFFSET]
	ldr	r12, =0x109			@ Setup L2 AUXCTRL value
	dsb
	smc	#0

	ldr	r2, =OMAP44XX_L2CACHE_BASE
	ldr	r4, =OMAP44XX_SAR_RAM_BASE
	ldr	r9, [r4, #L2X0_LOCKDOWN_OFFSET0]
	str	r9, [r2, #L2X0_LOCKDOWN_WAY_D0]
	str	r9, [r2, #L2X0_LOCKDOWN_WAY_D1]
	str	r9, [r2, #L2X0_LOCKDOWN_WAY_I0]
	str	r9, [r2, #L2X0_LOCKDOWN_WAY_I1]

	dsb
	mov	r0, #0x1
	ldr	r12, =0x102			@ Enable L2 Cache controller
	dsb
	smc	#0
	dsb
skip_l2en:
#endif

	/* Check if we have Public access to SMP bit */
	mrc	p15, 0, r0, c1, c1, 2		@ Read NSACR data
	tst	r0, #(1 << 18)
	beq	skip_ns_smp_enable		@ Skip if still no access

	/* Set the SMP bit if it is not already set */
	mrc	p15, 0, r0, c1, c0, 1
	tst	r0, #(1 << 6)			@ Check SMP bit enabled?
	orreq	r0, r0, #(1 << 6)
	mcreq	p15, 0, r0, c1, c0, 1
	isb
skip_ns_smp_enable:

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
	mcr	p15, 0, r6, c12, c0, 0		@ Secure or NS VBAR
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
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	streq	r4, [r5, #MMU_OFFSET0]		@ Modify the table entry
	strne	r4, [r5, #MMU_OFFSET1]
	str	r1, [r2]

	/*
	 * Storing address of entry being modified
	 * It will be restored after enabling MMU
	 */
	ldr     r5, =OMAP44XX_SAR_RAM_BASE
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	orreq	r5, r5, #MMU_OFFSET0
	orrne	r5, r5, #MMU_OFFSET1
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
	mrc	p15, 0, r0, c0, c0, 5		@ Read MPIDR
	ands	r0, r0, #0x0f
	orreq	r8, r8, #MMU_OFFSET0		@ Get address of entry that..
	orrne	r8, r8, #MMU_OFFSET1		@ was modified
	ldr	r2, [r8, #TABLE_ADDRESS_OFFSET]
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

ENTRY(omap_bus_sync)
	stmfd	sp!, {r9, lr}
	/* SO write to drain of MPU-2-DDR T2ASYNC FIFO */
	bl	omap_get_dram_barrier_base
	ldr	r2, [r0]
	str	r2, [r0]
	/* SO write to drain MPU-2-L3 T2ASYNC FIFO */
	bl	omap_get_sram_barrier_base
	ldr	r2, [r0]
	str	r2, [r0]
	isb
	ldmfd	sp!, {r9, pc}
ENDPROC(omap_bus_sync)

ENTRY(omap_do_wfi)
	stmfd	sp!, {lr}
	/* Drain interconnect write buffers. */
	bl omap_bus_sync

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
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Sixteen
	 * NOPs as per Cortex-A9 pipeline.
	 */
	wfi					@ Wait For Interrupt
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ldmfd	sp!, {pc}
ENDPROC(omap_do_wfi)

#endif
