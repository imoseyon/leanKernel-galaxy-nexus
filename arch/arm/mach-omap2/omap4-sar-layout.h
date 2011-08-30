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

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif-44xx.h>
#include <mach/dmm-44xx.h>
#include <mach/ctrl_module_pad_core_44xx.h>

#include "cm1_44xx.h"
#include "cm2_44xx.h"
#include "prcm-common.h"

/*
 * The SAR RAM is maintained during Device OFF mode.
 * It is split into 4 banks with different privilege accesses
 *
 * ---------------------------------------------------------------------
 * Access mode			Bank	Address Range
 * ---------------------------------------------------------------------
 * HS/GP : Public		1	0x4A32_6000 - 0x4A32_6FFF (4kB)
 * HS/GP : Public, Secured
 * if padconfaccdisable=1	2	0x4A32_7000 - 0x4A32_73FF (1kB)
 * HS/EMU : Secured
 * GP : Public			3	0x4A32_8000 - 0x4A32_87FF (2kB)
 * HS/GP :
 * Secure Priviledge,
 * write once.			4	0x4A32_9000 - 0x4A32_93FF (1kB)
 * ---------------------------------------------------------------------
 * The SAR RAM save regiter layout is fixed since restore is done by hardware.
 */

#define MODULE_ADDR_IDX				0
#define MODULE_OFFSET_IDX			1
#define MODULE_NB_REGS_IDX			2
#define SAR_RAM_OFFSET_IDX			3

/*
 * Module Index used to lookup VA using index
 */
#define MAX_SAR_MODULES				14
#define EMIF1_INDEX				0
#define EMIF2_INDEX				1
#define DMM_INDEX				2
#define CM1_INDEX				3
#define CM2_INDEX				4
#define C2C_INDEX				5
#define CTRL_MODULE_PAD_CORE_INDEX		6
#define L3_CLK1_INDEX				7
#define L3_CLK2_INDEX				8
#define L3_CLK3_INDEX				9
#define USBTLL_INDEX				10
#define UHH_INDEX				11
#define L4CORE_INDEX				12
#define L4PER_INDEX				13

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
#define L2X0_AUXCTRL_OFFSET			0xd28
#define OMAP_TYPE_OFFSET			0xd2c
#define L2X0_LOCKDOWN_OFFSET0			0xd30
#define L2X0_PREFETCHCTRL_OFFSET		0xd34
#define L2X0_SAVE_OFFSET0			0xd38
#define L2X0_SAVE_OFFSET1			0xd3c

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
