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
#include <linux/seq_file.h>
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
#include <linux/hrtimer.h>
#endif

#define MAX_OVERLAYS	5
#define MAX_MANAGERS	3
#define MAX_DISPLAYS	4

#define DEBUG_OVERLAYS		(1 << 0)
#define DEBUG_COMPOSITIONS	(1 << 1)
#define DEBUG_PHASES		(1 << 2)
#define DEBUG_WAITS		(1 << 3)
#define DEBUG_GRALLOC_PHASES	(1 << 4)

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
	struct notifier_block state_notifiers[MAX_DISPLAYS];
};

extern int debug;

#ifdef CONFIG_DEBUG_FS
extern struct mutex dbg_mtx;
extern struct list_head dbg_comps;
#define DO_IF_DEBUG_FS(cmd) {	\
	mutex_lock(&dbg_mtx);	\
	cmd;			\
	mutex_unlock(&dbg_mtx);	\
}
#else
#define DO_IF_DEBUG_FS(cmd)
#endif

enum dsscomp_state {
	DSSCOMP_STATE_ACTIVE		= 0xAC54156E,
	DSSCOMP_STATE_APPLYING		= 0xB554C591,
	DSSCOMP_STATE_APPLIED		= 0xB60504C1,
	DSSCOMP_STATE_PROGRAMMED	= 0xC0520652,
	DSSCOMP_STATE_DISPLAYED		= 0xD15504CA,
};

struct dsscomp_data {
	enum dsscomp_state state;
	/*
	 * :TRICKY: before applying, overlays used in a composition are stored
	 * in ovl_mask and the other masks are empty.  Once composition is
	 * applied, blank is set to see if all overlays are to be disabled on
	 * this composition, any disabled overlays in the composition are set in
	 * ovl_dmask, and ovl_mask is updated to include ALL overlays that are
	 * actually on the display - even if they are not part of the
	 * composition. The reason: we use ovl_mask to see if an overlay is used
	 * or planned to be used on a manager.  We update ovl_mask when
	 * composition is programmed (removing the disabled overlays).
	 */
	bool blank;		/* true if all overlays are to be disabled */
	u32 ovl_mask;		/* overlays used on this frame */
	u32 ovl_dmask;		/* overlays disabled on this frame */
	u32 ix;			/* manager index that this frame is on */
	struct dsscomp_setup_mgr_data frm;
	struct dss2_ovl_info ovls[5];
	void (*extra_cb)(void *data, int status);
	void *extra_cb_data;
	bool must_apply;	/* whether composition must be applied */

#ifdef CONFIG_DEBUG_FS
	struct list_head dbg_q;
	u32 dbg_used;
	struct {
		u32 t, state;
	} dbg_log[8];
#endif
};

struct dsscomp_sync_obj {
	int state;
	int fd;
	atomic_t refs;
};

/*
 * Kernel interface
 */
int dsscomp_queue_init(struct dsscomp_dev *cdev);
void dsscomp_queue_exit(void);
void dsscomp_gralloc_init(struct dsscomp_dev *cdev);
void dsscomp_gralloc_exit(void);
int dsscomp_gralloc_queue_ioctl(struct dsscomp_setup_dispc_data *d);
int dsscomp_wait(struct dsscomp_sync_obj *sync, enum dsscomp_wait_phase phase,
								int timeout);
int dsscomp_state_notifier(struct notifier_block *nb,
						unsigned long arg, void *ptr);

/* basic operation - if not using queues */
int set_dss_ovl_info(struct dss2_ovl_info *oi);
int set_dss_mgr_info(struct dss2_mgr_info *mi, struct omapdss_ovl_cb *cb);
struct omap_overlay_manager *find_dss_mgr(int display_ix);
void swap_rb_in_ovl_info(struct dss2_ovl_info *oi);
void swap_rb_in_mgr_info(struct dss2_mgr_info *mi);

/*
 * Debug functions
 */
void dump_ovl_info(struct dsscomp_dev *cdev, struct dss2_ovl_info *oi);
void dump_comp_info(struct dsscomp_dev *cdev, struct dsscomp_setup_mgr_data *d,
				const char *phase);
void dump_total_comp_info(struct dsscomp_dev *cdev,
				struct dsscomp_setup_dispc_data *d,
				const char *phase);
const char *dsscomp_get_color_name(enum omap_color_mode m);

void dsscomp_dbg_comps(struct seq_file *s);
void dsscomp_dbg_gralloc(struct seq_file *s);

#define log_state_str(s) (\
	(s) == DSSCOMP_STATE_ACTIVE		? "ACTIVE"	: \
	(s) == DSSCOMP_STATE_APPLYING		? "APPLY'N"	: \
	(s) == DSSCOMP_STATE_APPLIED		? "APPLIED"	: \
	(s) == DSSCOMP_STATE_PROGRAMMED		? "PROGR'D"	: \
	(s) == DSSCOMP_STATE_DISPLAYED		? "DISPL'D"	: "INVALID")

#define log_status_str(ev) ( \
	((ev) & DSS_COMPLETION_CHANGED)		? "CHANGED"	: \
	(ev) == DSS_COMPLETION_DISPLAYED	? "DISPLAYED"	: \
	(ev) == DSS_COMPLETION_PROGRAMMED	? "PROGRAMMED"	: \
	(ev) == DSS_COMPLETION_TORN		? "TORN"	: \
	(ev) == DSS_COMPLETION_RELEASED		? "RELEASED"	: \
	((ev) & DSS_COMPLETION_RELEASED)	? "ECLIPSED"	: "???")

#ifdef CONFIG_DSSCOMP_DEBUG_LOG
extern struct dbg_event_t {
	u32 ms, a1, a2, ix;
	void *data;
	const char *fmt;
} dbg_events[128];
extern u32 dbg_event_ix;

void dsscomp_dbg_events(struct seq_file *s);
#endif

static inline
void __log_event(u32 ix, u32 ms, void *data, const char *fmt, u32 a1, u32 a2)
{
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
	if (!ms)
		ms = ktime_to_ms(ktime_get());
	dbg_events[dbg_event_ix].ms = ms;
	dbg_events[dbg_event_ix].data = data;
	dbg_events[dbg_event_ix].fmt = fmt;
	dbg_events[dbg_event_ix].a1 = a1;
	dbg_events[dbg_event_ix].a2 = a2;
	dbg_events[dbg_event_ix].ix = ix;
	dbg_event_ix = (dbg_event_ix + 1) % ARRAY_SIZE(dbg_events);
#endif
}

#define log_event(ix, ms, data, fmt, a1, a2) \
	DO_IF_DEBUG_FS(__log_event(ix, ms, data, fmt, a1, a2))

#endif
