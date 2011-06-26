/*
 * OMAP memory barrier header.
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *  Santosh Shilimkar <santosh.shilimkar@ti.com>
 *  Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __MACH_BARRIERS_H
#define __MACH_BARRIERS_H

#include <linux/types.h>

/* provide func ptr so to allow safe calling at any point */
struct omap_bus_post_fns {
	void (*sync)(void);
};

extern struct omap_bus_post_fns omap_bus_post;

#ifdef CONFIG_ARCH_OMAP4
static inline void bus_sync(void)
{
	omap_bus_post.sync();
}
#else
static inline void bus_sync(void)
{ }
#endif

#define rmb()		dsb()
#define wmb()		do { dsb(); outer_sync(); bus_sync(); } while (0)
#define mb()		wmb()

#endif	/* __MACH_BARRIERS_H */
