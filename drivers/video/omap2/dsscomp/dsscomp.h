/*
 * linux/drivers/video/omap2/dsscomp/base.c
 *
 * DSS Composition basic operation support
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _DSSCOMP_H
#define _DSSCOMP_H

#include <linux/miscdevice.h>
#include <linux/debugfs.h>

#define MAX_OVERLAYS	5
#define MAX_MANAGERS	3
#define MAX_DISPLAYS	4

#define DEBUG_OVERLAYS		(1 << 0)
#define DEBUG_COMPOSITIONS	(1 << 1)
#define DEBUG_PHASES		(1 << 2)
#define DEBUG_WAITS		(1 << 3)

/*
 * Utility macros
 */
#define ZERO(c)		memset(&c, 0, sizeof(c))
#define ZEROn(c, n)	memset(c, 0, sizeof(*c) * n)
#define DEV(c)		(c->dev.this_device)

/**
 * DSS Composition Device Driver
 *
 * @dev:   misc device base
 * @dbgfs: debugfs hook
 */
struct dsscomp_dev {
	struct miscdevice dev;
	struct dentry *dbgfs;

	/* cached DSS objects */
	u32 num_ovls;
	struct omap_overlay *ovls[MAX_OVERLAYS];
	u32 num_mgrs;
	struct omap_overlay_manager *mgrs[MAX_MANAGERS];
	u32 num_displays;
	struct omap_dss_device *displays[MAX_DISPLAYS];
};

extern int debug;

/*
 * Kernel interface
 */
int dsscomp_queue_init(struct dsscomp_dev *cdev);
void dsscomp_queue_exit(void);

/* basic operation - if not using queues */
int set_dss_ovl_info(struct dss2_ovl_info *oi);
int set_dss_mgr_info(struct dss2_mgr_info *mi);
struct omap_overlay_manager *find_dss_mgr(int display_ix);

/*
 * Debug functions
 */
void dump_ovl_info(struct dsscomp_dev *cdev, struct dss2_ovl_info *oi);
void dump_comp_info(struct dsscomp_dev *cdev, struct dsscomp_setup_mgr_data *d,
				const char *phase);

#endif
