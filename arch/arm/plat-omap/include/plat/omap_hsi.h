/*
 * /mach/omap_hsi.h
 *
 * Hardware definitions for HSI and SSI.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/* NOTE: This file defines the registers address offsets for both the
 * SSI and HSI devices. Most of the registers share the same offset between
 * these devices.
 * When common or HSI only, the constants are name HSI*. Else the SSI specific
 * constants are name HSI_SSI*
 */

#ifndef __OMAP_HSI_H__
#define __OMAP_HSI_H__

/* Set the HSI Functional Clock to 96MHz.
 * This is to ensure HSI will function even at OPP50. */
#define HSI_DEFAULT_FCLK		96000000	/* 96 MHz */


#define HSI_PORT_OFFSET			0x1000

/*
 * GDD base addr : 0x48059000 (SSI)
 * GDD base addr : 0x4A059000 (HSI)
 */
#define HSI_GDD_OFFSET			0x1000
#define HSI_GDD_BASE			HSI_GDD_OFFSET	/* 0x9000 */

/*
 * HST base addr:
 *	port 1: 0x4805a000 (SSI) - 0x4A05a000 (HSI)
 *	port 2: 0x4805b000 (SSI) - 0x4a05b000 (HSI)
 */
#define HSI_HST_OFFSET			0x2000
#define HSI_HST_BASE(port)		(HSI_HST_OFFSET + (((port) - 1) *\
							(HSI_PORT_OFFSET)))
 /*
 * HSR base addr:
 *	port 1: 0x4805a800 (SSI) - 0x4A05a800 (HSI)
 *	port 2: 0x4805b800 (SSI) - 0x4A05b800 (HSI)
 */
#define HSI_HSR_OFFSET			0x2800
#define HSI_HSR_BASE(port)		(HSI_HSR_OFFSET + (((port) - 1) *\
							(HSI_PORT_OFFSET)))
/*
 * HSI SYS registers
 */
#define HSI_SYS_REVISION_REG		0x0000
#define HSI_SSI_REV_MASK		0x000000ff
#define HSI_SSI_REV_MAJOR		0xf0
#define HSI_SSI_REV_MINOR		0x0f

#define HSI_SYS_SYSCONFIG_REG		0x0010
#define HSI_AUTOIDLE			(1 << 0)
#define HSI_SOFTRESET			(1 << 1)
#define HSI_FREE_EMU			(1 << 2)	/* Only for HSI */
#define HSI_SIDLEMODE_FORCE		0
#define HSI_SIDLEMODE_NO		(1 << 3)
#define HSI_SIDLEMODE_SMART		(1 << 4)
#define HSI_SIDLEMODE_SMART_WAKEUP	(3 << 3)
#define HSI_SIDLEMODE_MASK		0x00000018
#define HSI_MIDLEMODE_FORCE		0
#define HSI_MIDLEMODE_NO		(1 << 12)
#define HSI_MIDLEMODE_SMART		(1 << 13)
#define HSI_MIDLEMODE_SMART_WAKEUP	(3 << 12)
#define HSI_MIDLEMODE_MASK		0x00003000

#define HSI_SYS_SYSSTATUS_REG		0x0014
#define HSI_RESETDONE			1

#define HSI_SYS_MPU_STATUS_BASE		0x0808
#define HSI_SYS_MPU_STATUS_PORT_OFFSET	0x10
#define HSI_SYS_MPU_STATUS_IRQ_OFFSET	8

#define HSI_SYS_MPU_STATUS_REG(port, irq)				\
			(HSI_SYS_MPU_STATUS_BASE +			\
			((((port) - 1) * HSI_SYS_MPU_STATUS_PORT_OFFSET) +\
			((irq) * HSI_SYS_MPU_STATUS_IRQ_OFFSET)))
#define HSI_SYS_MPU_ENABLE_BASE		0x080c
#define HSI_SYS_MPU_ENABLE_PORT_OFFSET	0x10
#define HSI_SYS_MPU_ENABLE_IRQ_OFFSET	8

#define HSI_SYS_MPU_ENABLE_REG(port, irq)				\
			(HSI_SYS_MPU_ENABLE_BASE +			\
			((((port) - 1) * HSI_SYS_MPU_ENABLE_PORT_OFFSET) +\
			((irq) * HSI_SYS_MPU_ENABLE_IRQ_OFFSET)))
#define HSI_HST_DATAACCEPT(channel)	(((channel) < 8) ?		\
					(1 << (channel)) :		\
					(1 << ((channel) - 8)))
#define HSI_HSR_DATAAVAILABLE(channel)	((channel) < 8 ?		\
					(1 << ((channel) + 8)) :	\
					(1 << ((channel) - 8 + 8)))
#define HSI_HSR_DATAOVERRUN(channel)	((channel) < 8 ?		\
					(1 << ((channel) + 16)) :	\
					(1 << ((channel) - 8 + 16)))

#define HSI_ERROROCCURED		(1 << 24)
#define HSI_BREAKDETECTED		(1 << 25)
#define HSI_CAWAKEDETECTED		(1 << 26)

#define HSI_SYS_GDD_MPU_IRQ_STATUS_REG	0x0800
#define HSI_SYS_GDD_MPU_IRQ_ENABLE_REG	0x0804
#define HSI_GDD_LCH(channel)		(1 << (channel))


#define HSI_SYS_WAKE_OFFSET		0x10
#define HSI_SYS_WAKE_BASE		0x0c00
#define HSI_SYS_WAKE_REG(port)		(HSI_SYS_WAKE_BASE +\
					(((port) - 1) * HSI_SYS_WAKE_OFFSET))

#define HSI_SYS_CLEAR_WAKE_BASE		0x0c04
#define HSI_SYS_CLEAR_WAKE_REG(port)	(HSI_SYS_CLEAR_WAKE_BASE +\
					(((port) - 1) * HSI_SYS_WAKE_OFFSET))

#define HSI_SYS_SET_WAKE_BASE		0x0c08
#define HSI_SYS_SET_WAKE_REG(port)	(HSI_SYS_SET_WAKE_BASE +\
					(((port) - 1) * HSI_SYS_WAKE_OFFSET))

#define HSI_SSI_WAKE_MASK		0xff	/* for SSI */
#define HSI_WAKE_MASK			0xffff	/* for HSI */
#define HSI_SET_WAKE_4_WIRES		(0 << 16)
#define HSI_SET_WAKE_3_WIRES		(1 << 16)
#define HSI_SET_WAKE_3_WIRES_MASK	0xfffcffff /* 3-wires + ACREADY to 1 */
#define HSI_SET_WAKE_READY_LVL_0	(0 << 17)
#define HSI_SET_WAKE_READY_LVL_1	(1 << 17)
#define HSI_SET_WAKE(channel)		(1 << (channel))
#define HSI_CLEAR_WAKE(channel)		(1 << (channel))
#define HSI_WAKE(channel)		(1 << (channel))

#define HSI_SYS_HWINFO_REG		0x0004	/* only for HSI */

/* Additional registers definitions (for channels 8 .. 15) for HSI */
#define HSI_SYS_MPU_U_STATUS_BASE	0x0408
#define HSI_SYS_MPU_U_STATUS_REG(port, irq)				\
			(HSI_SYS_MPU_U_STATUS_BASE +			\
			((((port) - 1) * HSI_SYS_MPU_STATUS_PORT_OFFSET) +\
			((irq) * HSI_SYS_MPU_STATUS_IRQ_OFFSET)))

#define HSI_SYS_MPU_U_ENABLE_BASE	0x040c
#define HSI_SYS_MPU_U_ENABLE_REG(port, irq)				\
			(HSI_SYS_MPU_U_ENABLE_BASE +			\
			((((port) - 1) * HSI_SYS_MPU_ENABLE_PORT_OFFSET) +\
			((irq) * HSI_SYS_MPU_ENABLE_IRQ_OFFSET)))

/*
 * HSI HST registers
 */
#define HSI_HST_ID_REG(port)		(HSI_HST_BASE(port) + 0x0000)

#define HSI_HST_MODE_REG(port)		(HSI_HST_BASE(port) + 0x0004)
#define HSI_MODE_VAL_MASK		3
#define HSI_MODE_SLEEP			0
#define HSI_MODE_STREAM			1
#define HSI_MODE_FRAME			2
#define HSI_SSI_MODE_MULTIPOINTS	3		/* SSI only */
#define HSI_FLOW_OFFSET			2		/* HSI only */
#define HSI_FLOW_VAL_MASK		3		/* HSI only */
#define HSI_FLOW_SYNCHRONIZED		0		/* HSI only */
#define HSI_FLOW_PIPELINED		1		/* HSI only */
#define HSI_FLOW_REAL_TIME		2		/* HSI only */
#define HSI_HST_MODE_WAKE_CTRL_AUTO	(1 << 4)	/* HSI only */
#define HSI_HST_MODE_WAKE_CTRL_SW	(0 << 4)	/* HSI only */

#define HSI_HST_FRAMESIZE_REG(port)	(HSI_HST_BASE(port) + 0x0008)
#define HSI_FRAMESIZE_DEFAULT		31
#define HSI_FRAMESIZE_MAX		0x1f

#define HSI_HST_TXSTATE_REG(port)	(HSI_HST_BASE(port) + 0x000c)
#define	HSI_HST_TXSTATE_VAL_MASK	0x07
#define	HSI_HST_TXSTATE_IDLE		0

#define HSI_HST_BUFSTATE_REG(port)	(HSI_HST_BASE(port) + 0x0010)
#define HSI_HST_BUFSTATE_FIFO_REG(fifo)	(((fifo) < 8) ?			\
					HSI_HST_BUFSTATE_REG(1) :	\
					HSI_HST_BUFSTATE_REG(2))
#define	HSI_BUFSTATE_CHANNEL(channel)	((channel) < 8 ?		\
					(1 << (channel)) :		\
					(1 << ((channel) - 8)))

#define HSI_HST_DIVISOR_REG(port)	(HSI_HST_BASE(port) + 0x0018)
#define HSI_DIVISOR_DEFAULT		1
#define HSI_SSI_MAX_TX_DIVISOR		0x7f	/* for SSI */
#define HSI_MAX_TX_DIVISOR		0xff	/* for HSI */

#define HSI_HST_BREAK_REG(port)		(HSI_HST_BASE(port) + 0x0020)
#define HSI_HST_CHANNELS_REG(port)	(HSI_HST_BASE(port) + 0x0024)
#define HSI_CHANNELS_DEFAULT		4
#define HSI_SSI_CHANNELS_MAX		8	/* for SSI */
#define HSI_CHANNELS_MAX		16	/* for HSI */

#define HSI_HST_ARBMODE_REG(port)	(HSI_HST_BASE(port) + 0x0028)
#define HSI_ARBMODE_ROUNDROBIN		0
#define HSI_ARBMODE_PRIORITY		1

#define HSI_HST_BUFFER_BASE(port)		(HSI_HST_BASE(port) + 0x0080)
#define HSI_HST_BUFFER_CH_REG(port, channel)	(HSI_HST_BUFFER_BASE(port) +\
						((channel) * 4))
#define HSI_HST_BUFFER_FIFO_REG(fifo)	(((fifo) < 8) ?			\
			(HSI_HST_BUFFER_CH_REG(1, (fifo))) :		\
			(HSI_HST_BUFFER_CH_REG(2, (fifo) - 8)))

#define HSI_HST_SWAPBUF_BASE(port)		(HSI_HST_BASE(port) + 0x00c0)
#define HSI_HST_SWAPBUF_CH_REG(port, channel)	(HSI_HST_SWAPBUF_BASE(port) +\
						((channel) * 4))


/* Additional registers for HSI */
#define	HSI_HST_FIFO_COUNT			16
#define	HSI_HST_FIFO_SIZE			8
#define HSI_HST_MAPPING_FIFO_REG(fifo)		(HSI_HST_BASE(1) + 0x0100 +\
						((fifo) * 4))
#define HSI_MAPPING_ENABLE		1
#define HSI_MAPPING_CH_NUMBER_OFFSET	1
#define HSI_MAPPING_PORT_NUMBER_OFFSET	7
#define HSI_HST_MAPPING_THRESH_OFFSET	10
#define HSI_HST_MAPPING_THRESH_VALUE	(0x0 << HSI_HST_MAPPING_THRESH_OFFSET)

/*
 * HSI HSR registers
 */
#define HSI_HSR_ID_REG(port)		(HSI_HSR_BASE(port) + 0x0000)

#define HSI_HSR_MODE_REG(port)		(HSI_HSR_BASE(port) + 0x0004)

#define HSI_HSR_MODE_MODE_VAL_MASK     (3 << 0)       /* HSI only */
#define HSI_HSR_MODE_FLOW_VAL_MASK     (3 << 2)       /* HSI only */
#define HSI_HSR_MODE_WAKE_STATUS	(1 << 4)	/* HSI only */
#define HSI_HSR_MODE_MODE_VAL_SLEEP    0xFFFFFFFC     /* HSI only */

#define HSI_HSR_FRAMESIZE_REG(port)	(HSI_HSR_BASE(port) + 0x0008)

#define HSI_HSR_RXSTATE_REG(port)	(HSI_HSR_BASE(port) + 0x000c)

#define HSI_HSR_BUFSTATE_REG(port)	(HSI_HSR_BASE(port) + 0x0010)
#define HSI_HSR_BUFSTATE_FIFO_REG(fifo)	(((fifo) < 8) ?			\
					HSI_HSR_BUFSTATE_REG(1) :	\
					HSI_HSR_BUFSTATE_REG(2))

#define HSI_HSR_BREAK_REG(port)		(HSI_HSR_BASE(port) + 0x001c)

#define HSI_HSR_ERROR_REG(port)		(HSI_HSR_BASE(port) + 0x0020)
#define HSI_HSR_ERROR_SIG		1
#define HSI_HSR_ERROR_FTE		(1 << 1)	/* HSI only */
#define HSI_HSR_ERROR_TBE		(1 << 4)	/* HSI only */
#define HSI_HSR_ERROR_RME		(1 << 7)	/* HSI only */
#define HSI_HSR_ERROR_TME		(1 << 11)	/* HSI only */
#define HSI_HSR_ERROR_ALL		(HSI_HSR_ERROR_SIG |    \
					 HSI_HSR_ERROR_FTE |    \
					 HSI_HSR_ERROR_TBE |    \
					 HSI_HSR_ERROR_RME |    \
					 HSI_HSR_ERROR_TME)

#define HSI_HSR_ERRORACK_REG(port)	(HSI_HSR_BASE(port) + 0x0024)

#define HSI_HSR_CHANNELS_REG(port)	(HSI_HSR_BASE(port) + 0x0028)

#define HSI_HSR_OVERRUN_REG(port)	(HSI_HSR_BASE(port) + 0x002c)

#define HSI_HSR_OVERRUNACK_REG(port)	(HSI_HSR_BASE(port) + 0x0030)

/* HSR_COUNTERS_Pp is former SSI_TIMEOUT_REG */
#define HSI_HSR_COUNTERS_REG(port)	(HSI_HSR_BASE(port) + 0x0034)
#define SSI_TIMEOUT_REG(port)		(HSI_HSR_COUNTERS_REG(port))
#define HSI_TIMEOUT_DEFAULT		0	/* SSI only */
#define HSI_SSI_RX_TIMEOUT_OFFSET	0	/* SSI only */
#define HSI_SSI_RX_TIMEOUT_MASK		0x1ff	/* SSI only */
#define HSI_COUNTERS_FT_MASK		0x000fffff	/* HSI only */
#define HSI_COUNTERS_TB_MASK		0x00f00000	/* HSI only */
#define HSI_COUNTERS_FB_MASK		0xff000000	/* HSI only */
#define HSI_COUNTERS_FT_OFFSET		0	/* HSI only */
#define HSI_COUNTERS_TB_OFFSET		20	/* HSI only */
#define HSI_COUNTERS_FB_OFFSET		24	/* HSI only */
/* Default FT value: 2 x max_bits_per_frame + 20% margin */
#define HSI_COUNTERS_FT_DEFAULT		(90 << HSI_COUNTERS_FT_OFFSET)
#define HSI_COUNTERS_TB_DEFAULT		(6 << HSI_COUNTERS_TB_OFFSET)
#define HSI_COUNTERS_FB_DEFAULT		(8 << HSI_COUNTERS_FB_OFFSET)
#define HSI_HSR_COMBINE_COUNTERS(FB, TB, FT)				  \
		(((FB << HSI_COUNTERS_FB_OFFSET) & HSI_COUNTERS_FB_MASK) \
		 ((TB << HSI_COUNTERS_TB_OFFSET) & HSI_COUNTERS_TB_MASK) \
		 ((FT << HSI_COUNTERS_FT_OFFSET) & HSI_COUNTERS_FT_MASK))
/* For SSI */
#define SSI_SSR_COMBINE_COUNTERS(FT)				  \
		((FT << HSI_SSI_RX_TIMEOUT_OFFSET) & HSI_SSI_RX_TIMEOUT_MASK)

#define HSI_HSR_BUFFER_BASE(port)	(HSI_HSR_BASE(port) + 0x0080)
#define HSI_HSR_BUFFER_CH_REG(port, channel)	(HSI_HSR_BUFFER_BASE(port) +\
						((channel) * 4))
#define HSI_HSR_BUFFER_FIFO_REG(fifo)	(((fifo) < 8) ?			\
			(HSI_HSR_BUFFER_CH_REG(1, (fifo))) :		\
			(HSI_HSR_BUFFER_CH_REG(2, (fifo) - 8)))

#define HSI_HSR_SWAPBUF_BASE(port)	(HSI_HSR_BASE(port) + 0x00c0)
#define HSI_HSR_SWAPBUF_CH_REG(port, channel)	(HSI_HSR_SWAPBUF_BASE(port) +\
						((channel) * 4))

/* Additional registers for HSI */
#define	HSI_HSR_FIFO_COUNT		16
#define	HSI_HSR_FIFO_SIZE		8
#define HSI_HSR_MAPPING_FIFO_REG(fifo)	(HSI_HSR_BASE(1) + 0x0100 +\
					((fifo) * 4))
#define HSI_HSR_MAPPING_WORDS_MASK	(0xf << 10)

#define HSI_HSR_DLL_REG			(HSI_HSR_BASE(1) + 0x0144)
#define HSI_HSR_DLL_COCHRE		1
#define HSI_HSR_DLL_COCHGR		(1 << 4)
#define HSI_HSR_DLL_INCO_MASK		0x0003ff00
#define HSI_HSR_DLL_INCO_OFFSET		8

#define HSI_HSR_DIVISOR_REG(port)	(HSI_HSR_BASE(port) + 0x014C)
#define HSI_HSR_DIVISOR_MASK		0xff
#define HSI_MAX_RX_DIVISOR		0xff

/*
 * HSI GDD registers
 */
#define HSI_SSI_DMA_CHANNEL_MAX		8
#define HSI_HSI_DMA_CHANNEL_MAX		16

#define HSI_SSI_GDD_HW_ID_REG		(HSI_GDD_BASE + 0x0000)

#define HSI_SSI_GDD_PPORT_ID_REG	(HSI_GDD_BASE + 0x0010)

#define HSI_SSI_GDD_MPORT_ID_REG	(HSI_GDD_BASE + 0x0014)

#define HSI_SSI_GDD_PPORT_SR_REG	(HSI_GDD_BASE + 0x0020)
#define HSI_PPORT_ACTIVE_LCH_NUMBER_MASK	0xff

#define HSI_GDD_MPORT_SR_REG		(HSI_GDD_BASE + 0x0024)
#define HSI_SSI_MPORT_ACTIVE_LCH_NUMBER_MASK	0xff

#define HSI_SSI_GDD_TEST_REG		(HSI_GDD_BASE + 0x0040)
#define HSI_SSI_TEST			1

#define HSI_GDD_GCR_REG			(HSI_GDD_BASE + 0x0100)
#define	HSI_CLK_AUTOGATING_ON		(1 << 3)
#define	HSI_SWITCH_OFF			(1 << 0)

#define HSI_GDD_GRST_REG		(HSI_GDD_BASE + 0x0200)
#define HSI_GDD_GRST_SWRESET		1

#define HSI_GDD_CSDP_BASE		(HSI_GDD_BASE + 0x0800)
#define HSI_GDD_CSDP_OFFSET		0x40
#define HSI_GDD_CSDP_REG(channel)	(HSI_GDD_CSDP_BASE +\
					((channel) * HSI_GDD_CSDP_OFFSET))

#define HSI_DST_BURST_EN_MASK		0xc000
#define HSI_DST_SINGLE_ACCESS0		0
#define HSI_DST_SINGLE_ACCESS		(1 << 14)
#define HSI_DST_BURST_4X32_BIT		(2 << 14)
#define HSI_DST_BURST_8x32_BIT		(3 << 14)

#define HSI_DST_MASK			0x1e00
#define HSI_DST_MEMORY_PORT		(8 << 9)
#define HSI_DST_PERIPHERAL_PORT		(9 << 9)

#define HSI_SRC_BURST_EN_MASK		0x0180
#define HSI_SRC_SINGLE_ACCESS0		0
#define HSI_SRC_SINGLE_ACCESS		(1 << 7)
#define HSI_SRC_BURST_4x32_BIT		(2 << 7)
#define HSI_SRC_BURST_8x32_BIT		(3 << 7)

#define HSI_SRC_MASK			0x003c
#define HSI_SRC_MEMORY_PORT		(8 << 2)
#define HSI_SRC_PERIPHERAL_PORT		(9 << 2)

#define HSI_DATA_TYPE_MASK		3
#define HSI_DATA_TYPE_S32		2

#define HSI_GDD_CCR_BASE		(HSI_GDD_BASE + 0x0802)
#define HSI_GDD_CCR_OFFSET		0x40
#define HSI_GDD_CCR_REG(channel)	(HSI_GDD_CCR_BASE +\
					((channel) * HSI_GDD_CCR_OFFSET))
#define HSI_DST_AMODE_MASK		(3 << 14)
#define HSI_DST_AMODE_CONST		0
#define HSI_DST_AMODE_POSTINC		(1 << 14)

#define HSI_SRC_AMODE_MASK		(3 << 12)
#define HSI_SRC_AMODE_CONST		0
#define HSI_SRC_AMODE_POSTINC		(1 << 12)

#define HSI_CCR_ENABLE			(1 << 7)

#define HSI_CCR_SYNC_MASK		0x001f	/* only for SSI */

#define HSI_GDD_CCIR_BASE		(HSI_GDD_BASE + 0x0804)
#define HSI_GDD_CCIR_OFFSET		0x40
#define HSI_GDD_CCIR_REG(channel)	(HSI_GDD_CCIR_BASE +\
					((channel) * HSI_GDD_CCIR_OFFSET))

#define HSI_BLOCK_IE			(1 << 5)
#define HSI_HALF_IE			(1 << 2)
#define HSI_TOUT_IE			(1 << 0)

#define HSI_GDD_CSR_BASE		(HSI_GDD_BASE + 0x0806)
#define HSI_GDD_CSR_OFFSET		0x40
#define HSI_GDD_CSR_REG(channel)	(HSI_GDD_CSR_BASE +\
					((channel) * HSI_GDD_CSR_OFFSET))

#define HSI_CSR_SYNC			(1 << 6)
#define HSI_CSR_BLOCK			(1 << 5) /* Full block is transferred */
#define HSI_CSR_HALF			(1 << 2) /* Half block is transferred */
#define HSI_CSR_TOUT			(1 << 0) /* Time-out overflow occurs */

#define HSI_GDD_CSSA_BASE		(HSI_GDD_BASE + 0x0808)
#define HSI_GDD_CSSA_OFFSET		0x40
#define HSI_GDD_CSSA_REG(channel)	(HSI_GDD_CSSA_BASE +\
					((channel) * HSI_GDD_CSSA_OFFSET))


#define HSI_GDD_CDSA_BASE		(HSI_GDD_BASE + 0x080c)
#define HSI_GDD_CDSA_OFFSET		0x40
#define HSI_GDD_CDSA_REG(channel)	(HSI_GDD_CDSA_BASE +\
					((channel) * HSI_GDD_CDSA_OFFSET))

#define HSI_GDD_CEN_BASE		(HSI_GDD_BASE + 0x0810)
#define HSI_GDD_CEN_OFFSET		0x40
#define HSI_GDD_CEN_REG(channel)	(HSI_GDD_CEN_BASE +\
					((channel) * HSI_GDD_CEN_OFFSET))


#define HSI_GDD_CSAC_BASE		(HSI_GDD_BASE + 0x0818)
#define HSI_GDD_CSAC_OFFSET		0x40
#define HSI_GDD_CSAC_REG(channel)	(HSI_GDD_CSAC_BASE +\
					((channel) * HSI_GDD_CSAC_OFFSET))

#define HSI_GDD_CDAC_BASE		(HSI_GDD_BASE + 0x081a)
#define HSI_GDD_CDAC_OFFSET		0x40
#define HSI_GDD_CDAC_REG(channel)	(HSI_GDD_CDAC_BASE +\
					((channel) * HSI_GDD_CDAC_OFFSET))

#define HSI_SSI_GDD_CLNK_CTRL_BASE	(HSI_GDD_BASE + 0x0828)
#define HSI_SSI_GDD_CLNK_CTRL_OFFSET	0x40
#define HSI_SSI_GDD_CLNK_CTRL_REG(channel)	(HSI_SSI_GDD_CLNK_CTRL_BASE +\
				(channel * HSI_SSI_GDD_CLNK_CTRL_OFFSET))

#define HSI_SSI_ENABLE_LNK		(1 << 15)
#define HSI_SSI_STOP_LNK		(1 << 14)
#define HSI_SSI_NEXT_CH_ID_MASK		0xf

/*
 * HSI Helpers
 */
#define HSI_SYS_MPU_ENABLE_CH_REG(port, irq, channel)			\
				(((channel) < HSI_SSI_CHANNELS_MAX) ?	\
				HSI_SYS_MPU_ENABLE_REG(port, irq) :	\
				HSI_SYS_MPU_U_ENABLE_REG(port, irq))

#define HSI_SYS_MPU_STATUS_CH_REG(port, irq, channel)      \
			      (((channel) < HSI_SSI_CHANNELS_MAX) ?    \
			      HSI_SYS_MPU_STATUS_REG(port, irq) :    \
			      HSI_SYS_MPU_U_STATUS_REG(port, irq))


/* HSI errata handling */
#define IS_HSI_ERRATA(errata, id)		(errata & (id))
#define SET_HSI_ERRATA(errata, id)		(errata |= (id))

/* HSI-C1BUG00088: i696: HSI: Issue with SW reset
 * No recovery from SW reset under specific circumstances
 * If a SW RESET is done while some HSI errors are still not
 * acknowledged, the HSR FSM is stucked. */
#define HSI_ERRATUM_i696_SW_RESET_FSM_STUCK		BIT(0)

/* HSI-C1BUG00085: ixxx: HSI wakeup issue in 3 wires mode
 * HSI will NOT generate the Swakeup for 2nd frame if it entered
 * IDLE after 1st received frame */
#define HSI_ERRATUM_ixxx_3WIRES_NO_SWAKEUP		BIT(1)

/*
* HSI - OMAP4430-2.2BUG00055: i702
* HSI: DSP Swakeup generated is the same than MPU Swakeup.
* System cannot enter in off mode due to the DSP.
*/
#define HSI_ERRATUM_i702_PM_HSI_SWAKEUP			BIT(2)


/**
 *	struct omap_ssi_config - SSI board configuration
 *	@num_ports: Number of ports in use
 *	@cawake_line: Array of cawake gpio lines
 */
struct omap_ssi_board_config {
	unsigned int num_ports;
	int cawake_gpio[2];
};
extern int omap_ssi_config(struct omap_ssi_board_config *ssi_config);

/**
 *	struct omap_hsi_config - HSI board configuration
 *	@num_ports: Number of ports in use
 */
struct omap_hsi_board_config {
	unsigned int num_ports;
};
extern int omap_hsi_config(struct omap_hsi_board_config *hsi_config);

#ifdef CONFIG_OMAP_HSI
extern int omap_hsi_prepare_suspend(int hsi_port, bool dev_may_wakeup);
extern int omap_hsi_io_wakeup_check(void);
extern int omap_hsi_wakeup(int hsi_port);
extern bool omap_hsi_is_io_wakeup_from_hsi(int *hsi_port);
#else
inline int omap_hsi_prepare_suspend(int hsi_port,
					bool dev_may_wakeup) { return -ENOSYS; }
inline int omap_hsi_io_wakeup_check(void) { return -ENOSYS; }
inline int omap_hsi_wakeup(int hsi_port) { return -ENOSYS; }
inline bool omap_hsi_is_io_wakeup_from_hsi(int *hsi_port) { return false; }
#endif

#endif /* __OMAP_HSI_H__ */
