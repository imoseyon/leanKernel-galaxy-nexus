/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_OMAP_FIQ_DEBUGGER_H
#define __MACH_OMAP_FIQ_DEBUGGER_H

#ifdef CONFIG_OMAP_FIQ_DEBUGGER
int __init omap_serial_debug_init(int id, bool is_fiq, bool is_high_prio_irq,
				  struct omap_device_pad *pads, int num_pads);
u32 omap_debug_uart_resume_idle(void);

#else
static inline int __init omap_serial_debug_init(int id, bool is_fiq, bool is_high_prio_irq,
				  struct omap_device_pad *pads, int num_pads)
{
	return 0;
}

static inline u32 omap_debug_uart_resume_idle(void)
{
	return 0;
}
#endif

#endif
