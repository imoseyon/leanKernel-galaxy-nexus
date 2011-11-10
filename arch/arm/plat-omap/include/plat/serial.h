/*
 * arch/arm/plat-omap/include/mach/serial.h
 *
 * Copyright (C) 2009 Texas Instruments
 * Addded OMAP4 support- Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

#include <linux/init.h>

/*
 * Memory entry used for the DEBUG_LL UART configuration. See also
 * uncompress.h and debug-macro.S.
 *
 * Note that using a memory location for storing the UART configuration
 * has at least two limitations:
 *
 * 1. Kernel uncompress code cannot overlap OMAP_UART_INFO as the
 *    uncompress code could then partially overwrite itself
 * 2. We assume printascii is called at least once before paging_init,
 *    and addruart has a chance to read OMAP_UART_INFO
 */
#define OMAP_UART_INFO		(PLAT_PHYS_OFFSET + 0x3ffc)

/* OMAP1 serial ports */
#define OMAP1_UART1_BASE	0xfffb0000
#define OMAP1_UART2_BASE	0xfffb0800
#define OMAP1_UART3_BASE	0xfffb9800

/* OMAP2 serial ports */
#define OMAP2_UART1_BASE	0x4806a000
#define OMAP2_UART2_BASE	0x4806c000
#define OMAP2_UART3_BASE	0x4806e000

/* OMAP3 serial ports */
#define OMAP3_UART1_BASE	OMAP2_UART1_BASE
#define OMAP3_UART2_BASE	OMAP2_UART2_BASE
#define OMAP3_UART3_BASE	0x49020000
#define OMAP3_UART4_BASE	0x49042000	/* Only on 36xx */

/* OMAP4 serial ports */
#define OMAP4_UART1_BASE	OMAP2_UART1_BASE
#define OMAP4_UART2_BASE	OMAP2_UART2_BASE
#define OMAP4_UART3_BASE	0x48020000
#define OMAP4_UART4_BASE	0x4806e000

/* TI816X serial ports */
#define TI816X_UART1_BASE	0x48020000
#define TI816X_UART2_BASE	0x48022000
#define TI816X_UART3_BASE	0x48024000

/* External port on Zoom2/3 */
#define ZOOM_UART_BASE		0x10000000
#define ZOOM_UART_VIRT		0xfa400000

#define OMAP_PORT_SHIFT		2
#define OMAP7XX_PORT_SHIFT	0
#define ZOOM_PORT_SHIFT		1

#define OMAP1510_BASE_BAUD	(12000000/16)
#define OMAP16XX_BASE_BAUD	(48000000/16)
#define OMAP24XX_BASE_BAUD	(48000000/16)

/*
 * DEBUG_LL port encoding stored into the UART1 scratchpad register by
 * decomp_setup in uncompress.h
 */
#define OMAP1UART1		11
#define OMAP1UART2		12
#define OMAP1UART3		13
#define OMAP2UART1		21
#define OMAP2UART2		22
#define OMAP2UART3		23
#define OMAP3UART1		OMAP2UART1
#define OMAP3UART2		OMAP2UART2
#define OMAP3UART3		33
#define OMAP3UART4		34		/* Only on 36xx */
#define OMAP4UART1		OMAP2UART1
#define OMAP4UART2		OMAP2UART2
#define OMAP4UART3		43
#define OMAP4UART4		44
#define TI816XUART1		81
#define TI816XUART2		82
#define TI816XUART3		83
#define ZOOM_UART		95		/* Only on zoom2/3 */

/* This is only used by 8250.c for omap1510 */
#define is_omap_port(pt)	({int __ret = 0;			\
			if ((pt)->port.mapbase == OMAP1_UART1_BASE ||	\
			    (pt)->port.mapbase == OMAP1_UART2_BASE ||	\
			    (pt)->port.mapbase == OMAP1_UART3_BASE)	\
				__ret = 1;				\
			__ret;						\
			})

#ifndef __ASSEMBLER__

struct omap_board_data;
struct omap_uart_port_info;
struct omap_device_pad;

extern void omap_serial_init(void);
extern void omap_serial_board_init(struct omap_uart_port_info *platform_data);
extern void omap_serial_init_port(struct omap_board_data *bdata,
		struct omap_uart_port_info *platform_data);
void __init omap_serial_init_port_pads(int id, struct omap_device_pad *pads,
	int size, struct omap_uart_port_info *info);
extern u32 omap_uart_resume_idle(void);
extern int omap_uart_wake(u8 id);
extern int omap_uart_enable(u8 uart_num);
extern int omap_uart_disable(u8 uart_num);

#define MUX_PULL_UP	((1<<8) | (1<<4) | (1<<3) | (7))
void omap_rts_mux_write(u16 val, int num);

#endif

#endif
