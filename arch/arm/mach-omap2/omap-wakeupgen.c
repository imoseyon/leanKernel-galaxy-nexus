/*
 * OMAP WakeupGen Source file
 *
 * The WakeupGen unit is responsible for generating wakeup event from the
 * incoming interrupts and enable bits. The WakeupGen is implemented in MPU
 * always-On power domain. The WakeupGen consists of two sub-units, one for
 * each CPU and manages only SPI interrupts. Hardware requirements is that
 * the GIC and WakeupGen should be kept in sync for proper operation.
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Written by Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <asm/hardware/gic.h>

#include <mach/omap-wakeupgen.h>
#include <mach/omap4-common.h>

#include "omap4-sar-layout.h"

#define NR_BANKS		4
#define MAX_IRQS		128
#define WKG_MASK_ALL		0x00000000
#define WKG_UNMASK_ALL		0xffffffff
#define CPU_ENA_OFFSET		0x400
#define CPU0_ID			0x0
#define CPU1_ID			0x1

/* WakeupGen Base addres */
static void __iomem *wakeupgen_base;
static void __iomem *sar_base;
static DEFINE_PER_CPU(u32 [NR_BANKS], irqmasks);
static DEFINE_SPINLOCK(wakeupgen_lock);

/*
 * Static helper functions
 */

static inline u32 wakeupgen_readl(u8 idx, u32 cpu)
{
	return __raw_readl(wakeupgen_base + OMAP_WKG_ENB_A_0 +
				(cpu * CPU_ENA_OFFSET) + (idx * 4));
}

static inline void wakeupgen_writel(u32 val, u8 idx, u32 cpu)
{
	__raw_writel(val, wakeupgen_base + OMAP_WKG_ENB_A_0 +
				(cpu * CPU_ENA_OFFSET) + (idx * 4));
}

static inline void sar_writel(u32 val, u32 offset, u8 idx)
{
	__raw_writel(val, sar_base + offset + (idx * 4));
}

static void _wakeupgen_set_all(unsigned int cpu, unsigned int reg)
{
	u8 i;

	for (i = 0; i < NR_BANKS; i++)
		wakeupgen_writel(reg, i, cpu);
}

static inline int _wakeupgen_get_irq_info(u32 irq, u32 *bit_posn, u8 *reg_index)
{
	unsigned int spi_irq;

	/*
	 * PPIs and SGIs are not supported
	 */
	if (irq < OMAP44XX_IRQ_GIC_START)
		return -EINVAL;

	/*
	 * Subtract the GIC offset
	 */
	spi_irq = irq - OMAP44XX_IRQ_GIC_START;
	if (spi_irq > MAX_IRQS) {
		pr_err("omap wakeupGen: Invalid IRQ%d\n", irq);
		return -EINVAL;
	}

	/*
	 * Each wakeup gen register controls 32
	 * interrupts. i.e 1 bit per SPI IRQ
	 */
	*reg_index = spi_irq >> 5;
	*bit_posn = spi_irq %= 32;

	return 0;
}

static void _wakeupgen_clear(unsigned int irq, unsigned int cpu)
{
	u32 val, bit_number;
	u8 i;

	if (_wakeupgen_get_irq_info(irq, &bit_number, &i))
		return;

	val = wakeupgen_readl(i, cpu);
	val &= ~BIT(bit_number);
	wakeupgen_writel(val, i, cpu);
}

static void _wakeupgen_set(unsigned int irq, unsigned int cpu)
{
	u32 val, bit_number;
	u8 i;

	if (_wakeupgen_get_irq_info(irq, &bit_number, &i))
		return;

	val = wakeupgen_readl(i, cpu);
	val |= BIT(bit_number);
	wakeupgen_writel(val, i, cpu);
}

static void _wakeupgen_save_masks(unsigned int cpu)
{
	u8 i;

	for (i = 0; i < NR_BANKS; i++)
		per_cpu(irqmasks, cpu)[i] = wakeupgen_readl(i, cpu);
}

static void _wakeupgen_restore_masks(unsigned int cpu)
{
	u8 i;

	for (i = 0; i < NR_BANKS; i++)
		wakeupgen_writel(per_cpu(irqmasks, cpu)[i], i, cpu);
}

/*
 * Architecture specific Mask extensiom
 */
static void wakeupgen_mask(struct irq_data *d)
{
	spin_lock(&wakeupgen_lock);
	_wakeupgen_clear(d->irq, d->node);
	spin_unlock(&wakeupgen_lock);
}

/*
 * Architecture specific Unmask extensiom
 */
static void wakeupgen_unmask(struct irq_data *d)
{
	spin_lock(&wakeupgen_lock);
	_wakeupgen_set(d->irq, d->node);
	spin_unlock(&wakeupgen_lock);
}

/**
 * omap_wakeupgen_irqmask_all() -  Mask or unmask interrupts
 * @cpu - CPU ID
 * @set - The IRQ register mask.
 *	0 = Mask all interrupts on the 'cpu'
 *	1 = Unmask all interrupts on the 'cpu'
 *
 * Ensure that the initial mask is maintained. This is faster than
 * iterating through GIC rgeisters to arrive at the correct masks
 */
void omap_wakeupgen_irqmask_all(unsigned int cpu, unsigned int set)
{
	if (omap_rev() == OMAP4430_REV_ES1_0)
		return;

	spin_lock(&wakeupgen_lock);
	if (set) {
		_wakeupgen_save_masks(cpu);
		_wakeupgen_set_all(cpu, WKG_MASK_ALL);
	} else {
		_wakeupgen_set_all(cpu, WKG_UNMASK_ALL);
		_wakeupgen_restore_masks(cpu);
	}
	spin_unlock(&wakeupgen_lock);
}

#ifdef CONFIG_PM
/*
 * Masking wakeup irqs is handled by the IRQCHIP_MASK_ON_SUSPEND flag,
 * so no action is necessary in set_wake, but implement an empty handler
 * here to prevent enable_irq_wake() returning an error.
 */
static int wakeupgen_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}
#else
#define wakeupgen_set_wake	NULL
#endif

/*
 * Initialse the wakeupgen module
 */
int __init omap_wakeupgen_init(void)
{
	u8 i;

	/* Not supported on on OMAP4 ES1.0 silicon */
	if (omap_rev() == OMAP4430_REV_ES1_0) {
		WARN(1, "WakeupGen: Not supported on OMAP4430 ES1.0\n");
		return -EPERM;
	}

	/* Static mapping, never released */
	wakeupgen_base = ioremap(OMAP44XX_WKUPGEN_BASE, SZ_4K);
	if (WARN_ON(!wakeupgen_base))
		return -ENODEV;

	/* Clear all IRQ bitmasks at wakeupGen level */
	for (i = 0; i < NR_BANKS; i++) {
		wakeupgen_writel(0, i, CPU0_ID);
		wakeupgen_writel(0, i, CPU1_ID);
	}

	/*
	 * Override gic architecture specific fucntioms to add
	 * OMAP WakeupGen interrupt controller along with GIC
	 */
	gic_arch_extn.irq_mask = wakeupgen_mask;
	gic_arch_extn.irq_unmask = wakeupgen_unmask;
	gic_arch_extn.irq_set_wake = wakeupgen_set_wake;
	gic_arch_extn.flags = IRQCHIP_MASK_ON_SUSPEND;

	return 0;
}

/**
 * omap_wakeupgen_save() - WakeupGen context save function
 *
 * Save WakewupGen context in SAR BANK3. Restore is done by ROM code.
 * WakeupGen IP is integrated along with GIC to manage the
 * interrupt wakeups from CPU low power states. It's located in
 * always ON power domain. It manages masking/unmasking of
 * Shared peripheral interrupts(SPI).So the interrupt enable/disable
 * control should be in sync and consistent at WakeupGen and GIC so
 * that interrupts are not lost. Hence GIC and WakeupGen are saved
 * and restored together.

 * During normal operation, WakeupGen delivers external interrupts
 * directly to the GIC. When the CPU asserts StandbyWFI, indicating
 * it wants to enter lowpower state, the Standby Controller checks
 * with the WakeupGen unit using the idlereq/idleack handshake to make
 * sure there is no incoming interrupts.
 */

void omap_wakeupgen_save(void)
{
	u8 i;
	u32 val;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		return;

	if (!sar_base)
		sar_base = omap4_get_sar_ram_base();

	for (i = 0; i < NR_BANKS; i++) {
		/* Save the CPUx interrupt mask for IRQ 0 to 127 */
		val = wakeupgen_readl(i, 0);
		sar_writel(val, WAKEUPGENENB_OFFSET_CPU0, i);
		val = wakeupgen_readl(i, 1);
		sar_writel(val, WAKEUPGENENB_OFFSET_CPU1, i);

		/*
		 * Disable the secure interrupts for CPUx. The restore
		 * code blindly restores secure and non-secure interrupt
		 * masks from SAR RAM. Secure interrupts are not suppose
		 * to be enabled from HLOS. So overwrite the SAR location
		 * so that the secure interrupt remains disabled.
		 */
		sar_writel(0x0, WAKEUPGENENB_SECURE_OFFSET_CPU0, i);
		sar_writel(0x0, WAKEUPGENENB_SECURE_OFFSET_CPU1, i);
	}

	/* Save AuxBoot* registers */
	val = __raw_readl(wakeupgen_base + OMAP_AUX_CORE_BOOT_0);
	__raw_writel(val, sar_base + AUXCOREBOOT0_OFFSET);
	val = __raw_readl(wakeupgen_base + OMAP_AUX_CORE_BOOT_0);
	__raw_writel(val, sar_base + AUXCOREBOOT1_OFFSET);

	/* Save SyncReq generation logic */
	val = __raw_readl(wakeupgen_base + OMAP_AUX_CORE_BOOT_0);
	__raw_writel(val, sar_base + AUXCOREBOOT0_OFFSET);
	val = __raw_readl(wakeupgen_base + OMAP_AUX_CORE_BOOT_0);
	__raw_writel(val, sar_base + AUXCOREBOOT1_OFFSET);

	/* Save SyncReq generation logic */
	val = __raw_readl(wakeupgen_base + OMAP_PTMSYNCREQ_MASK);
	__raw_writel(val, sar_base + PTMSYNCREQ_MASK_OFFSET);
	val = __raw_readl(wakeupgen_base + OMAP_PTMSYNCREQ_EN);
	__raw_writel(val, sar_base + PTMSYNCREQ_EN_OFFSET);

	/* Set the Backup Bit Mask status */
	val = __raw_readl(sar_base + SAR_BACKUP_STATUS_OFFSET);
	val |= SAR_BACKUP_STATUS_WAKEUPGEN;
	__raw_writel(val, sar_base + SAR_BACKUP_STATUS_OFFSET);
}
