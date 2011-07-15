/**
 * OMAP Dual-Mode Timers - early initialization interface
 *
 * Function interface called first to start dmtimer early initialization.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_OMAP2_DMTIMER_H
#define __ASM_ARCH_OMAP2_DMTIMER_H

#include <plat/dmtimer.h>

/*
 * dmtimer is required during early part of boot sequence even before
 * device model and pm_runtime if fully up and running. This function
 * is called from following sequence:
 * start_kernel()->time_init()->timer->init()->omap2_gp_timer_init()
 */
extern int __init omap2_system_timer_init(u8 id);
extern int __init omap2_system_timer_set_src(struct omap_dm_timer *, int);
#endif
