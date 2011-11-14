/*
 * OMAP4 MPUSS low power code
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Written by Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * OMAP4430 MPUSS mainly consists of dual Cortex-A9 with per-CPU
 * Local timer and Watchdog, GIC, SCU, PL310 L2 cache controller,
 * CPU0 and CPU1 LPRM modules.
 * CPU0, CPU1 and MPUSS each have there own power domain and
 * hence multiple low power combinations of MPUSS are possible.
 *
 * The CPU0 and CPU1 can't support Closed switch Retention (CSWR)
 * because the mode is not supported by hw constraints of dormant
 * mode. While waking up from the dormant mode, a reset  signal
 * to the Cortex-A9 processor must be asserted by the external
 * power controller.
 *
 * With architectural inputs and hardware recommendations, only
 * below modes are supported from power gain vs latency point of view.
 *
 *	CPU0		CPU1		MPUSS
 *	----------------------------------------------
 *	ON		ON		ON
 *	ON(Inactive)	OFF		ON(Inactive)
 *	OFF		OFF		CSWR
 *	OFF		OFF		OSWR
 *	OFF		OFF		OFF
 *	----------------------------------------------
 *
 * Note: CPU0 is the master core and it is the last CPU to go down
 * and first to wake-up when MPUSS low power states are excercised
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

#include <asm/tlbflush.h>
#include <asm/smp_scu.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <plat/omap44xx.h>
#include <mach/omap4-common.h>
#include <mach/omap-wakeupgen.h>

#include "omap4-sar-layout.h"
#include "pm.h"
#include "prcm_mpu44xx.h"
#include "prminst44xx.h"
#include "prcm44xx.h"
#include "prm44xx.h"
#include "prm-regbits-44xx.h"
#include "cm.h"
#include "prm.h"
#include "cm44xx.h"
#include "prcm-common.h"

#ifdef CONFIG_SMP

#define GIC_MASK_ALL			0x0
#define GIC_ISR_NON_SECURE		0xffffffff
#define SPI_ENABLE_SET_OFFSET		0x04
#define PPI_PRI_OFFSET			0x1c
#define SPI_PRI_OFFSET			0x20
#define SPI_TARGET_OFFSET		0x20
#define SPI_CONFIG_OFFSET		0x20

/* GIC save SAR bank base */
static struct powerdomain *mpuss_pd;
/*
 * Maximum Secure memory storage size.
 */
#define OMAP4_SECURE_RAM_STORAGE		(88 * SZ_1K)
/*
 * Physical address of secure memory storage
 */
dma_addr_t omap4_secure_ram_phys;
static void *secure_ram;

/* Variables to store maximum spi(Shared Peripheral Interrupts) registers. */
static u32 max_spi_irq, max_spi_reg;

struct omap4_cpu_pm_info {
	struct powerdomain *pwrdm;
	void __iomem *scu_sar_addr;
};

static void __iomem *gic_dist_base;
static void __iomem *gic_cpu_base;
static void __iomem *sar_base;

static DEFINE_PER_CPU(struct omap4_cpu_pm_info, omap4_pm_info);

#define PPI_CONTEXT_SIZE 11
static DEFINE_PER_CPU(u32[PPI_CONTEXT_SIZE], gic_ppi_context);
static DEFINE_PER_CPU(u32, gic_ppi_enable_mask);

/* Helper functions */
static inline void sar_writel(u32 val, u32 offset, u8 idx)
{
	__raw_writel(val, sar_base + offset + 4 * idx);
}

static inline u32 gic_readl(u32 offset, u8 idx)
{
	return __raw_readl(gic_dist_base + offset + 4 * idx);
}

u32 gic_cpu_read(u32 reg)
{
	return __raw_readl(gic_cpu_base + reg);
}

/*
 * Set the CPUx powerdomain's previous power state
 */
static inline void set_cpu_next_pwrst(unsigned int cpu_id,
				unsigned int power_state)
{
	struct omap4_cpu_pm_info *pm_info = &per_cpu(omap4_pm_info, cpu_id);

	pwrdm_set_next_pwrst(pm_info->pwrdm, power_state);
}

/*
 * Read CPU's previous power state
 */
static inline unsigned int read_cpu_prev_pwrst(unsigned int cpu_id)
{
	struct omap4_cpu_pm_info *pm_info = &per_cpu(omap4_pm_info, cpu_id);

	return pwrdm_read_prev_pwrst(pm_info->pwrdm);
}

/*
 * Clear the CPUx powerdomain's previous power state
 */
static inline void clear_cpu_prev_pwrst(unsigned int cpu_id)
{
	struct omap4_cpu_pm_info *pm_info = &per_cpu(omap4_pm_info, cpu_id);

	pwrdm_clear_all_prev_pwrst(pm_info->pwrdm);
}

struct reg_tuple {
	void __iomem *addr;
	u32 val;
};

static struct reg_tuple tesla_reg[] = {
	{.addr = OMAP4430_CM_TESLA_CLKSTCTRL},
	{.addr = OMAP4430_CM_TESLA_TESLA_CLKCTRL},
	{.addr = OMAP4430_PM_TESLA_PWRSTCTRL},
};

static struct reg_tuple ivahd_reg[] = {
	{.addr = OMAP4430_CM_IVAHD_CLKSTCTRL},
	{.addr = OMAP4430_CM_IVAHD_IVAHD_CLKCTRL},
	{.addr = OMAP4430_CM_IVAHD_SL2_CLKCTRL},
	{.addr = OMAP4430_PM_IVAHD_PWRSTCTRL}
};

static struct reg_tuple l3instr_reg[] = {
	{.addr = OMAP4430_CM_L3INSTR_L3_3_CLKCTRL},
	{.addr = OMAP4430_CM_L3INSTR_L3_INSTR_CLKCTRL},
	{.addr = OMAP4430_CM_L3INSTR_OCP_WP1_CLKCTRL},
};

/*
 * Store the SCU power status value to scratchpad memory
 */
static void scu_pwrst_prepare(unsigned int cpu_id, unsigned int cpu_state)
{
	struct omap4_cpu_pm_info *pm_info = &per_cpu(omap4_pm_info, cpu_id);
	u32 scu_pwr_st;

	switch (cpu_state) {
	case PWRDM_POWER_RET:
		scu_pwr_st = SCU_PM_DORMANT;
		break;
	case PWRDM_POWER_OFF:
		scu_pwr_st = SCU_PM_POWEROFF;
		break;
	case PWRDM_POWER_ON:
	case PWRDM_POWER_INACTIVE:
	default:
		scu_pwr_st = SCU_PM_NORMAL;
		break;
	}

	__raw_writel(scu_pwr_st, pm_info->scu_sar_addr);
}

static void gic_save_ppi(void)
{
	void __iomem *gic_dist_base = omap4_get_gic_dist_base();
	u32 *context = __get_cpu_var(gic_ppi_context);
	int i = 0;

	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x4);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x8);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0xc);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x10);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x14);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x18);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_PRI + 0x1c);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_CONFIG);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_CONFIG + 0x4);
	context[i++] = readl_relaxed(gic_dist_base + GIC_DIST_ENABLE_SET);

	BUG_ON(i != PPI_CONTEXT_SIZE);
}

static void gic_restore_ppi(void)
{
	void __iomem *gic_dist_base = omap4_get_gic_dist_base();
	u32 *context = __get_cpu_var(gic_ppi_context);
	int i = 0;

	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x4);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x8);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0xc);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x10);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x14);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x18);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_PRI + 0x1c);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_CONFIG);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_CONFIG + 0x4);
	writel_relaxed(context[i++], gic_dist_base + GIC_DIST_ENABLE_SET);

	BUG_ON(i != PPI_CONTEXT_SIZE);
}

/*
 * Mask all the PPIs. This should only be called after they have been saved
 * through secure trap or through save_ppi(). This is primarily needed to
 * mask the local timer irq that could be pending since timekeeping gets
 * suspended after the local irqs are disabled. The pending interrupt would
 * kick the CPU out of WFI immediately, and prevent it from going to the lower
 * power states. The correct value will be restored when the CPU is brought
 * back up by restore.
 */
static void gic_mask_ppi(void)
{
	void __iomem *gic_dist_base = omap4_get_gic_dist_base();

	__get_cpu_var(gic_ppi_enable_mask) =
		readl_relaxed(gic_dist_base + GIC_DIST_ENABLE_SET);
	writel_relaxed(0xffffffff, gic_dist_base + GIC_DIST_ENABLE_CLEAR);
}

static void gic_unmask_ppi(void)
{
	void __iomem *gic_dist_base = omap4_get_gic_dist_base();
	writel_relaxed(__get_cpu_var(gic_ppi_enable_mask),
		       gic_dist_base + GIC_DIST_ENABLE_SET);
}

/*
 * Save GIC context in SAR RAM. Restore is done by ROM code
 * GIC is lost only when MPU hits OSWR or OFF. It consists
 * of a distributor and a per-CPU interface module. The GIC
 * save restore is optimised to save only necessary registers.
 */
static void gic_save_context(void)
{
	u8 i;
	u32 val;

	/*
	 * Interrupt Clear Enable registers are inverse of set enable
	 * and hence not needed to be saved. ROM code programs it
	 * based on Set Enable register values.
	 */

	/* Save CPU 0 Interrupt Set Enable register */
	val = gic_readl(GIC_DIST_ENABLE_SET, 0);
	sar_writel(val, ICDISER_CPU0_OFFSET, 0);

	/* Disable interrupts on CPU1 */
	sar_writel(GIC_MASK_ALL, ICDISER_CPU1_OFFSET, 0);

	/* Save all SPI Set Enable register */
	for (i = 0; i < max_spi_reg; i++) {
		val = gic_readl(GIC_DIST_ENABLE_SET + SPI_ENABLE_SET_OFFSET, i);
		sar_writel(val, ICDISER_SPI_OFFSET, i);
	}

	/*
	 * Interrupt Priority Registers
	 * Secure sw accesses, last 5 bits of the 8 bits (bit[7:3] are used)
	 * Non-Secure sw accesses, last 4 bits (i.e. bits[7:4] are used)
	 * But the Secure Bits[7:3] are shifted by 1 in Non-Secure access.
	 * Secure (bits[7:3] << 1)== Non Secure bits[7:4]
	 * Hence right shift the value by 1 while saving the priority
	 */

	/* Save SGI priority registers (Software Generated Interrupt) */
	for (i = 0; i < 4; i++) {
		val = gic_readl(GIC_DIST_PRI, i);

		/* Save the priority bits of the Interrupts */
		sar_writel(val >> 0x1, ICDIPR_SFI_CPU0_OFFSET, i);

		/* Disable the interrupts on CPU1 */
		sar_writel(GIC_MASK_ALL, ICDIPR_SFI_CPU1_OFFSET, i);
	}

	/* Save PPI priority registers (Private Peripheral Intterupts) */
	val = gic_readl(GIC_DIST_PRI + PPI_PRI_OFFSET, 0);
	sar_writel(val >> 0x1, ICDIPR_PPI_CPU0_OFFSET, 0);
	sar_writel(GIC_MASK_ALL, ICDIPR_PPI_CPU1_OFFSET, 0);

	/* SPI priority registers - 4 interrupts/register */
	for (i = 0; i < (max_spi_irq / 4); i++) {
		val = gic_readl((GIC_DIST_PRI + SPI_PRI_OFFSET), i);
		sar_writel(val >> 0x1, ICDIPR_SPI_OFFSET, i);
	}

	/* SPI Interrupt Target registers - 4 interrupts/register */
	for (i = 0; i < (max_spi_irq / 4); i++) {
		val = gic_readl((GIC_DIST_TARGET + SPI_TARGET_OFFSET), i);
		sar_writel(val, ICDIPTR_SPI_OFFSET, i);
	}

	/* SPI Interrupt Congigeration eegisters- 16 interrupts/register */
	for (i = 0; i < (max_spi_irq / 16); i++) {
		val = gic_readl((GIC_DIST_CONFIG + SPI_CONFIG_OFFSET), i);
		sar_writel(val, ICDICFR_OFFSET, i);
	}

	/* Set the Backup Bit Mask status for GIC */
	val = __raw_readl(sar_base + SAR_BACKUP_STATUS_OFFSET);
	val |= (SAR_BACKUP_STATUS_GIC_CPU0 | SAR_BACKUP_STATUS_GIC_CPU1);
	__raw_writel(val, sar_base + SAR_BACKUP_STATUS_OFFSET);
}
/*
 * API to save GIC and Wakeupgen using secure API
 * for HS/EMU device
 */
static void save_gic_wakeupgen_secure(void)
{
	u32 ret;
	ret = omap4_secure_dispatcher(HAL_SAVEGIC_INDEX,
					FLAG_START_CRITICAL,
					0, 0, 0, 0, 0);
	if (!ret)
		pr_debug("GIC and Wakeupgen context save failed\n");
}


/*
 * API to save Secure RAM, GIC, WakeupGen Registers using secure API
 * for HS/EMU device
 */
static void save_secure_all(void)
{
	u32 ret;
	ret = omap4_secure_dispatcher(HAL_SAVEALL_INDEX,
					FLAG_START_CRITICAL,
					1, omap4_secure_ram_phys, 0, 0, 0);
	if (ret)
		pr_debug("Secure all context save failed\n");
}

/*
 * API to save Secure RAM using secure API
 * for HS/EMU device
 */
static void save_secure_ram(void)
{
	u32 ret;
	ret = omap4_secure_dispatcher(HAL_SAVESECURERAM_INDEX,
					FLAG_START_CRITICAL,
					1, omap4_secure_ram_phys, 0, 0, 0);
	if (!ret)
		pr_debug("Secure ram context save failed\n");
}

/* Helper functions for MPUSS OSWR */
static inline u32 mpuss_read_prev_logic_pwrst(void)
{
	u32 reg;

	reg = omap4_prminst_read_inst_reg(OMAP4430_PRM_PARTITION,
		OMAP4430_PRM_MPU_INST, OMAP4_RM_MPU_MPU_CONTEXT_OFFSET);
	reg &= OMAP4430_LOSTCONTEXT_DFF_MASK;
	return reg;
}

static inline void mpuss_clear_prev_logic_pwrst(void)
{
	u32 reg;

	reg = omap4_prminst_read_inst_reg(OMAP4430_PRM_PARTITION,
		OMAP4430_PRM_MPU_INST, OMAP4_RM_MPU_MPU_CONTEXT_OFFSET);
	omap4_prminst_write_inst_reg(reg, OMAP4430_PRM_PARTITION,
		OMAP4430_PRM_MPU_INST, OMAP4_RM_MPU_MPU_CONTEXT_OFFSET);
}

static inline void cpu_clear_prev_logic_pwrst(unsigned int cpu_id)
{
	u32 reg;

	if (cpu_id) {
		reg = omap4_prcm_mpu_read_inst_reg(OMAP4430_PRCM_MPU_CPU1_INST,
					OMAP4_RM_CPU1_CPU1_CONTEXT_OFFSET);
		omap4_prcm_mpu_write_inst_reg(reg, OMAP4430_PRCM_MPU_CPU1_INST,
					OMAP4_RM_CPU1_CPU1_CONTEXT_OFFSET);
	} else {
		reg = omap4_prcm_mpu_read_inst_reg(OMAP4430_PRCM_MPU_CPU0_INST,
					OMAP4_RM_CPU0_CPU0_CONTEXT_OFFSET);
		omap4_prcm_mpu_write_inst_reg(reg, OMAP4430_PRCM_MPU_CPU0_INST,
					OMAP4_RM_CPU0_CPU0_CONTEXT_OFFSET);
	}
}

static inline void save_ivahd_tesla_regs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tesla_reg); i++)
		tesla_reg[i].val = __raw_readl(tesla_reg[i].addr);

	for (i = 0; i < ARRAY_SIZE(ivahd_reg); i++)
		ivahd_reg[i].val = __raw_readl(ivahd_reg[i].addr);
}

static inline void restore_ivahd_tesla_regs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tesla_reg); i++)
		__raw_writel(tesla_reg[i].val, tesla_reg[i].addr);

	for (i = 0; i < ARRAY_SIZE(ivahd_reg); i++)
		__raw_writel(ivahd_reg[i].val, ivahd_reg[i].addr);
}

static inline void save_l3instr_regs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(l3instr_reg); i++)
		l3instr_reg[i].val = __raw_readl(l3instr_reg[i].addr);
}

static inline void restore_l3instr_regs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(l3instr_reg); i++)
		__raw_writel(l3instr_reg[i].val, l3instr_reg[i].addr);
}

/*
 * OMAP4 MPUSS Low Power Entry Function
 *
 * The purpose of this function is to manage low power programming
 * of OMAP4 MPUSS subsystem
 * Paramenters:
 *	cpu : CPU ID
 *	power_state: Targetted Low power state.
 *
 * MPUSS Low power states
 * The basic rule is that the MPUSS power domain must be at the higher or
 * equal power state (state that consume more power) than the higher of the
 * two CPUs. For example, it is illegal for system power to be OFF, while
 * the power of one or both of the CPU is DORMANT. When an illegal state is
 * entered, then the hardware behavior is unpredictable.
 *
 * MPUSS state for the context save
 * save_state =
 *	0 - Nothing lost and no need to save: MPUSS INACTIVE
 *	1 - CPUx L1 and logic lost: MPUSS CSWR
 *	2 - CPUx L1 and logic lost + GIC lost: MPUSS OSWR
 *	3 - CPUx L1 and logic lost + GIC + L2 lost: MPUSS OFF
 */
int omap4_enter_lowpower(unsigned int cpu, unsigned int power_state)
{
	unsigned int save_state = 0;
	unsigned int wakeup_cpu;

	if ((cpu >= NR_CPUS) || (omap_rev() == OMAP4430_REV_ES1_0))
		goto ret;

	switch (power_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_INACTIVE:
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 1;
		break;
	case PWRDM_POWER_RET:
	default:
		/*
		 * CPUx CSWR is invalid hardware state. Also CPUx OSWR
		 * doesn't make much scense, since logic is lost and $L1
		 * needs to be cleaned because of coherency. This makes
		 * CPUx OSWR equivalent to CPUX OFF and hence not supported
		 */
		WARN_ON(1);
		goto ret;
	}

	/*
	 * MPUSS book keeping should be executed by master
	 * CPU only which is also the last CPU to go down.
	 */
	if (cpu)
		goto cpu_prepare;

	pwrdm_pre_transition();

	/*
	 * Check MPUSS next state and save GIC if needed
	 * GIC lost during MPU OFF and OSWR
	 */
	pwrdm_clear_all_prev_pwrst(mpuss_pd);
	mpuss_clear_prev_logic_pwrst();
	if (omap4_device_next_state_off()) {
		if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
			omap_wakeupgen_save();
			gic_save_context();
		} else {
			save_secure_all();
			save_ivahd_tesla_regs();
			save_l3instr_regs();
		}
		save_state = 3;
		goto cpu_prepare;
	}

	switch (pwrdm_read_next_pwrst(mpuss_pd)) {
	case PWRDM_POWER_RET:
		/*
		 * MPUSS OSWR - Complete logic lost + L2$ retained.
		 * MPUSS CSWR - Complete logic retained + L2$ retained.
		 */
		if (pwrdm_read_logic_retst(mpuss_pd) == PWRDM_POWER_OFF) {
			if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
				omap_wakeupgen_save();
				gic_save_context();
			} else {
				save_gic_wakeupgen_secure();
				save_ivahd_tesla_regs();
				save_l3instr_regs();
			}
			save_state = 2;
		}
		break;
	case PWRDM_POWER_OFF:
		 /* MPUSS OFF - logic lost + L2$ lost */
		if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
			omap_wakeupgen_save();
			gic_save_context();
		} else {
			save_gic_wakeupgen_secure();
			save_ivahd_tesla_regs();
			save_l3instr_regs();
			save_secure_ram();
		}
		save_state = 3;
		break;
	case PWRDM_POWER_ON:
	case PWRDM_POWER_INACTIVE:
		/* No need to save MPUSS context */
	default:
		;
	}

cpu_prepare:
	if (cpu)
		gic_save_ppi();

	/*
	 * mask all PPIs to prevent them from kicking us out of wfi.
	 */
	gic_mask_ppi();

	clear_cpu_prev_pwrst(cpu);
	cpu_clear_prev_logic_pwrst(cpu);
	set_cpu_next_pwrst(cpu, power_state);
	scu_pwrst_prepare(cpu, power_state);

	/*
	 * Call low level function  with targeted CPU id
	 * and its low power state.
	 */
	stop_critical_timings();
	omap4_cpu_suspend(cpu, save_state);
	start_critical_timings();

	/*
	 * Restore the CPUx power state to ON otherwise CPUx
	 * power domain can transitions to programmed low power
	 * state while doing WFI outside the low powe code. On
	 * secure devices, CPUx does WFI which can result in
	 * domain transition
	 */
	wakeup_cpu = hard_smp_processor_id();
	set_cpu_next_pwrst(wakeup_cpu, PWRDM_POWER_ON);

	/*
	 * If we didn't actually get into the low power state (e.g. immediately
	 * exited wfi due to a pending interrupt), the secure side
	 * would not have restored CPU0's GIC PPI enable mask.
	 * For other CPUs, gic_restore_ppi will do that for us.
	 */
	if (cpu)
		gic_restore_ppi();
	else
		gic_unmask_ppi();

	/*
	 * If !master cpu return to hotplug-path.
	 *
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
	if (wakeup_cpu) {
		if (!cpu_is_omap443x())
			gic_dist_enable();
		goto ret;
	}

	/* Check if MPUSS lost it's logic */
	if (mpuss_read_prev_logic_pwrst()) {
		/* Clear SAR BACKUP status on GP devices */
		if (omap_type() == OMAP2_DEVICE_TYPE_GP)
			__raw_writel(0x0, sar_base + SAR_BACKUP_STATUS_OFFSET);
		/* Enable GIC distributor and inteface on CPU0*/
		gic_cpu_enable();
		gic_dist_enable();

		/*
		 * Dummy dispatcher call after OSWR and OFF
		 * Restore the right return Kernel address (with MMU on) for
		 * subsequent calls to secure ROM. Otherwise the return address
		 * will be to a PA return address and the system will hang.
		 */
		if (omap_type() != OMAP2_DEVICE_TYPE_GP)
			omap4_secure_dispatcher(PPA_SERVICE_0,
						FLAG_START_CRITICAL,
						0, 0, 0, 0, 0);
	}

	if (omap4_device_prev_state_off()) {
		restore_ivahd_tesla_regs();
		restore_l3instr_regs();
	}

	pwrdm_post_transition();

ret:
	return 0;
}

static void save_l2x0_auxctrl(void)
{
#ifdef CONFIG_CACHE_L2X0
	/*
	 * Save the L2X0 AUXCTRL value to SAR memory. Its used to
	 * in every restore patch MPUSS OFF path.
	 */
	void __iomem *l2x0_base = omap4_get_l2cache_base();
	u32 val;

	val = __raw_readl(l2x0_base + L2X0_AUX_CTRL);
	__raw_writel(val, sar_base + L2X0_AUXCTRL_OFFSET);

	/*
	 * Save the L2X0 PREFETCH_CTRL value to SAR memory.
	 * Its used in every restore path MPUSS OFF path.
	 */

	val = __raw_readl(l2x0_base + L2X0_PREFETCH_CTRL);
	__raw_writel(val, sar_base + L2X0_PREFETCHCTRL_OFFSET);

	/* Save L2X0 LOCKDOWN_OFFSET0 during SAR */
	val = readl_relaxed(l2x0_base + 0x900);
	writel_relaxed(val, sar_base + L2X0_LOCKDOWN_OFFSET0);
#endif
}

/*
 * Initialise OMAP4 MPUSS
 */
int __init omap4_mpuss_init(void)
{
	struct omap4_cpu_pm_info *pm_info;
	u8 i;

	/* Get GIC and SAR RAM base addresses */
	sar_base = omap4_get_sar_ram_base();
	gic_dist_base = omap4_get_gic_dist_base();
	gic_cpu_base = omap4_get_gic_cpu_base();

	if (omap_rev() == OMAP4430_REV_ES1_0) {
		WARN(1, "Power Management not supported on OMAP4430 ES1.0\n");
		return -ENODEV;
	}

	/* Initilaise per CPU PM information */
	pm_info = &per_cpu(omap4_pm_info, 0x0);
	pm_info->scu_sar_addr = sar_base + SCU_OFFSET0;
	pm_info->pwrdm = pwrdm_lookup("cpu0_pwrdm");
	if (!pm_info->pwrdm) {
		pr_err("Lookup failed for CPU0 pwrdm\n");
		return -ENODEV;
	}

	/* Clear CPU previous power domain state */
	pwrdm_clear_all_prev_pwrst(pm_info->pwrdm);
	cpu_clear_prev_logic_pwrst(0);

	/* Initialise CPU0 power domain state to ON */
	pwrdm_set_next_pwrst(pm_info->pwrdm, PWRDM_POWER_ON);

	pm_info = &per_cpu(omap4_pm_info, 0x1);
	pm_info->scu_sar_addr = sar_base + SCU_OFFSET1;
	pm_info->pwrdm = pwrdm_lookup("cpu1_pwrdm");
	if (!pm_info->pwrdm) {
		pr_err("Lookup failed for CPU1 pwrdm\n");
		return -ENODEV;
	}

	/*
	 * Check the OMAP type and store it to scratchpad
	 */
	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		/* Memory not released */
		secure_ram = dma_alloc_coherent(NULL, OMAP4_SECURE_RAM_STORAGE,
			(dma_addr_t *)&omap4_secure_ram_phys, GFP_ATOMIC);
		if (!secure_ram)
			pr_err("Unable to allocate secure ram storage\n");
		writel(0x1, sar_base + OMAP_TYPE_OFFSET);
	} else {
		writel(0x0, sar_base + OMAP_TYPE_OFFSET);
	}

	/* Clear CPU previous power domain state */
	pwrdm_clear_all_prev_pwrst(pm_info->pwrdm);
	cpu_clear_prev_logic_pwrst(1);

	/* Initialise CPU1 power domain state to ON */
	pwrdm_set_next_pwrst(pm_info->pwrdm, PWRDM_POWER_ON);

	/*
	 * Program the wakeup routine address for the CPU0 and CPU1
	 * used for OFF or DORMANT wakeup. Wakeup routine address
	 * is fixed so programit in init itself.
	 */
	__raw_writel(virt_to_phys(omap4_cpu_resume),
			sar_base + CPU1_WAKEUP_NS_PA_ADDR_OFFSET);
	__raw_writel(virt_to_phys(omap4_cpu_resume),
			sar_base + CPU0_WAKEUP_NS_PA_ADDR_OFFSET);

	mpuss_pd = pwrdm_lookup("mpu_pwrdm");
	if (!mpuss_pd) {
		pr_err("Failed to get lookup for MPUSS pwrdm\n");
		return -ENODEV;
	}

	/* Clear CPU previous power domain state */
	pwrdm_clear_all_prev_pwrst(mpuss_pd);
	mpuss_clear_prev_logic_pwrst();

	/*
	 * Find out how many interrupts are supported.
	 * OMAP4 supports max of 128 SPIs where as GIC can support
	 * up to 1020 interrupt sources. On OMAP4, maximum SPIs are
	 * fused in DIST_CTR bit-fields as 128. Hence the code is safe
	 * from reserved register writes since its well within 1020.
	 */
	max_spi_reg = __raw_readl(gic_dist_base + GIC_DIST_CTR) & 0x1f;
	max_spi_irq = max_spi_reg * 32;

	/*
	 * Mark the PPI and SPI interrupts as non-secure.
	 * program the SAR locations for interrupt security registers to
	 * reflect the same.
	 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
		sar_writel(GIC_ISR_NON_SECURE, ICDISR_CPU0_OFFSET, 0);
		sar_writel(GIC_ISR_NON_SECURE, ICDISR_CPU1_OFFSET, 0);
		for (i = 0; i < max_spi_reg; i++)
			sar_writel(GIC_ISR_NON_SECURE, ICDISR_SPI_OFFSET, i);
	}
	save_l2x0_auxctrl();

	return 0;
}

#endif

