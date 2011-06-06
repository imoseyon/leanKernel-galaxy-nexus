/*
 * omap4-sar-layout.h: OMAP4 SAR RAM layout header file
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef OMAP_ARCH_OMAP4_SAR_LAYOUT_H
#define OMAP_ARCH_OMAP4_SAR_LAYOUT_H

/*
 * SAR BANK offsets from base address OMAP44XX_SAR_RAM_BASE
 */
#define SAR_BANK1_OFFSET		0x0000
#define SAR_BANK2_OFFSET		0x1000
#define SAR_BANK3_OFFSET		0x2000
#define SAR_BANK4_OFFSET		0x3000

/* Scratch pad memory offsets from SAR_BANK1 */
#define CPU0_SAVE_OFFSET			0xb00
#define CPU1_SAVE_OFFSET			0xc00
#define MMU_OFFSET0				0xd00
#define MMU_OFFSET1				0xd10
#define SCU_OFFSET0				0xd20
#define SCU_OFFSET1				0xd24
#define L2X0_OFFSET				0xd28
#define L2X0_AUXCTRL_OFFSET			0xd2c
#define L1_OFFSET0				0xd30
#define L1_OFFSET1				0xd34
#define OMAP_TYPE_OFFSET			0xd38
#define L2X0_LOCKDOWN_OFFSET0			0xd3c

/* CPUx Wakeup Non-Secure Physical Address offsets in SAR_BANK3 */
#define CPU0_WAKEUP_NS_PA_ADDR_OFFSET		0xa04
#define CPU1_WAKEUP_NS_PA_ADDR_OFFSET		0xa08

/* GIC save restore offset from SAR_BANK3 */
#define SAR_BACKUP_STATUS_OFFSET		(SAR_BANK3_OFFSET + 0x500)
#define SAR_SECURE_RAM_SIZE_OFFSET		(SAR_BANK3_OFFSET + 0x504)
#define SAR_SECRAM_SAVED_AT_OFFSET		(SAR_BANK3_OFFSET + 0x508)
#define ICDISR_CPU0_OFFSET			(SAR_BANK3_OFFSET + 0x50c)
#define ICDISR_CPU1_OFFSET			(SAR_BANK3_OFFSET + 0x510)
#define ICDISR_SPI_OFFSET			(SAR_BANK3_OFFSET + 0x514)
#define ICDISER_CPU0_OFFSET			(SAR_BANK3_OFFSET + 0x524)
#define ICDISER_CPU1_OFFSET			(SAR_BANK3_OFFSET + 0x528)
#define ICDISER_SPI_OFFSET			(SAR_BANK3_OFFSET + 0x52c)
#define ICDIPR_SFI_CPU0_OFFSET			(SAR_BANK3_OFFSET + 0x53c)
#define ICDIPR_PPI_CPU0_OFFSET			(SAR_BANK3_OFFSET + 0x54c)
#define ICDIPR_SFI_CPU1_OFFSET			(SAR_BANK3_OFFSET + 0x550)
#define ICDIPR_PPI_CPU1_OFFSET			(SAR_BANK3_OFFSET + 0x560)
#define ICDIPR_SPI_OFFSET			(SAR_BANK3_OFFSET + 0x564)
#define ICDIPTR_SPI_OFFSET			(SAR_BANK3_OFFSET + 0x5e4)
#define ICDICFR_OFFSET				(SAR_BANK3_OFFSET + 0x664)
#define SAR_BACKUP_STATUS_GIC_CPU0		0x1
#define SAR_BACKUP_STATUS_GIC_CPU1		0x2

/* WakeUpGen save restore offset from OMAP44XX_SAR_RAM_BASE */
#define WAKEUPGENENB_OFFSET_CPU0		(SAR_BANK3_OFFSET + 0x684)
#define WAKEUPGENENB_SECURE_OFFSET_CPU0		(SAR_BANK3_OFFSET + 0x694)
#define WAKEUPGENENB_OFFSET_CPU1		(SAR_BANK3_OFFSET + 0x6a4)
#define WAKEUPGENENB_SECURE_OFFSET_CPU1		(SAR_BANK3_OFFSET + 0x6b4)
#define AUXCOREBOOT0_OFFSET			(SAR_BANK3_OFFSET + 0x6c4)
#define AUXCOREBOOT1_OFFSET			(SAR_BANK3_OFFSET + 0x6c8)
#define PTMSYNCREQ_MASK_OFFSET			(SAR_BANK3_OFFSET + 0x6cc)
#define PTMSYNCREQ_EN_OFFSET			(SAR_BANK3_OFFSET + 0x6d0)
#define SAR_BACKUP_STATUS_WAKEUPGEN		0x10

#endif
