/*
 * linux/drivers/video/omap2/dsscomp/queue.c
 *
 * DSS Composition queueing support
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

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#include <linux/debugfs.h>

#include "dsscomp.h"
/* queue state */

static DEFINE_MUTEX(mtx);

/* free overlay structs */
struct maskref {
	u32 mask;
	u32 refs[MAX_OVERLAYS];
};

static struct {
	struct workqueue_struct *apply_workq;

	u32 ovl_mask;		/* overlays used on this display */
	struct maskref ovl_qmask;		/* overlays queued to this display */
	bool blanking;
} mgrq[MAX_MANAGERS];

static struct workqueue_struct *cb_wkq;		/* callback work queue */
static struct dsscomp_dev *cdev;

#ifdef CONFIG_DEBUG_FS
LIST_HEAD(dbg_comps);
DEFINE_MUTEX(dbg_mtx);
#endif

#ifdef CONFIG_DSSCOMP_DEBUG_LOG
struct dbg_event_t dbg_events[128];
u32 dbg_event_ix;
#endif

static inline void __log_state(dsscomp_t c, void *fn, u32 ev)
{
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
	if (c->dbg_used < ARRAY_SIZE(c->dbg_log)) {
		u32 t = (u32) ktime_to_ms(ktime_get());
		c->dbg_log[c->dbg_used].t = t;
		c->dbg_log[c->dbg_used++].state = c->state;
		__log_event(20 * c->ix + 20, t, c, ev ? "%pf on %s" : "%pf",
				(u32) fn, (u32) log_status_str(ev));
	}
#endif
}
#define log_state(c, fn, ev) DO_IF_DEBUG_FS(__log_state(c, fn, ev))

static inline void maskref_incbit(struct maskref *om, u32 ix)
{
	om->refs[ix]++;
	om->mask |= 1 << ix;
}

static void maskref_decmask(struct maskref *om, u32 mask)
{
	while (mask) {
		u32 ix = fls(mask) - 1, m = 1 << ix;
		if (!--om->refs[ix])
			om->mask &= ~m;
		mask &= ~m;
	}
}

/*
 * ===========================================================================
 *		EXIT
 * ===========================================================================
 */

/* Initialize queue structures, and set up state of the displays */
int dsscomp_queue_init(struct dsscomp_dev *cdev_)
{
	u32 i, j;
	cdev = cdev_;

	if (ARRAY_SIZE(mgrq) < cdev->num_mgrs)
		return -EINVAL;

	ZERO(mgrq);
	for (i = 0; i < cdev->num_mgrs; i++) {
		struct omap_overlay_manager *mgr;
		mgrq[i].apply_workq = create_singlethread_workqueue("dsscomp_apply");
		if (!mgrq[i].apply_workq)
			goto error;

		/* record overlays on this display */
		mgr = cdev->mgrs[i];
		for (j = 0; j < cdev->num_ovls; j++) {
			if (cdev->ovls[j]->info.enabled &&
			    mgr &&
			    cdev->ovls[j]->manager == mgr)
				mgrq[i].ovl_mask |= 1 << j;
		}
	}

	cb_wkq = create_singlethread_workqueue("dsscomp_cb");
	if (!cb_wkq)
		goto error;

	return 0;
error:
	while (i--)
		destroy_workqueue(mgrq[i].apply_workq);
	return -ENOMEM;
}

/* get display index from manager */
static u32 get_display_ix(struct omap_overlay_manager *mgr)
{
	u32 i;

	/* handle if manager is not attached to a display */
	if (!mgr || !mgr->device)
		return cdev->num_displays;

	/* find manager's display */
	for (i = 0; i < cdev->num_displays; i++)
		if (cdev->displays[i] == mgr->device)
			break;

	return i;
}

/*
 * ===========================================================================
 *		QUEUING SETUP OPERATIONS
 * ===========================================================================
 */

/* create a new composition for a display */
dsscomp_t dsscomp_new(struct omap_overlay_manager *mgr)
{
	struct dsscomp_data *comp = NULL;
	u32 display_ix = get_display_ix(mgr);

	/* check manager */
	u32 ix = mgr ? mgr->id : cdev->num_mgrs;
	if (ix >= cdev->num_mgrs || display_ix >= cdev->num_displays)
		return ERR_PTR(-EINVAL);

	/* allocate composition */
	comp = kzalloc(sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return NULL;

	/* initialize new composition */
	comp->ix = ix;	/* save where this composition came from */
	comp->ovl_mask = comp->ovl_dmask = 0;
	comp->frm.sync_id = 0;
	comp->frm.mgr.ix = display_ix;
	comp->state = DSSCOMP_STATE_ACTIVE;

	DO_IF_DEBUG_FS({
		__log_state(comp, dsscomp_new, 0);
		list_add(&comp->dbg_q, &dbg_comps);
	});

	return comp;
}
EXPORT_SYMBOL(dsscomp_new);

/* returns overlays used in a composition */
u32 dsscomp_get_ovls(dsscomp_t comp)
{
	u32 mask;

	mutex_lock(&mtx);
	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);
	mask = comp->ovl_mask;
	mutex_unlock(&mtx);

	return mask;
}
EXPORT_SYMBOL(dsscomp_get_ovls);

/* set overlay info */
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl)
{
	int r = -EBUSY;
	u32 i, mask, oix, ix;
	struct omap_overlay *o;

	mutex_lock(&mtx);

	BUG_ON(!ovl);
	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);

	ix = comp->ix;

	if (ovl->cfg.ix >= cdev->num_ovls) {
		r = -EINVAL;
		goto done;
	}

	/* if overlay is already part of the composition */
	mask = 1 << ovl->cfg.ix;
	if (mask & comp->ovl_mask) {
		/* look up overlay */
		for (oix = 0; oix < comp->frm.num_ovls; oix++) {
			if (comp->ovls[oix].cfg.ix == ovl->cfg.ix)
				break;
		}
		BUG_ON(oix == comp->frm.num_ovls);
	} else {
		/* check if ovl is free to use */
		if (comp->frm.num_ovls >= ARRAY_SIZE(comp->ovls))
			goto done;

		/* not in any other displays queue */
		if (mask & ~mgrq[ix].ovl_qmask.mask) {
			for (i = 0; i < cdev->num_mgrs; i++) {
				if (i == ix)
					continue;
				if (mgrq[i].ovl_qmask.mask & mask)
					goto done;
			}
		}

		/* and disabled (unless forced) if on another manager */
		o = cdev->ovls[ovl->cfg.ix];
		if (o->info.enabled && (!o->manager || o->manager->id != ix))
			goto done;

		/* add overlay to composition & display */
		comp->ovl_mask |= mask;
		oix = comp->frm.num_ovls++;
		maskref_incbit(&mgrq[ix].ovl_qmask, ovl->cfg.ix);
	}

	comp->ovls[oix] = *ovl;
	r = 0;
done:
	mutex_unlock(&mtx);

	return r;
}
EXPORT_SYMBOL(dsscomp_set_ovl);

/* get overlay info */
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl)
{
	int r;
	u32 oix;

	mutex_lock(&mtx);

	BUG_ON(!ovl);
	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);

	if (ix >= cdev->num_ovls) {
		r = -EINVAL;
	} else if (comp->ovl_mask & (1 << ix)) {
		r = 0;
		for (oix = 0; oix < comp->frm.num_ovls; oix++)
			if (comp->ovls[oix].cfg.ix == ovl->cfg.ix) {
				*ovl = comp->ovls[oix];
				break;
			}
		BUG_ON(oix == comp->frm.num_ovls);
	} else {
		r = -ENOENT;
	}

	mutex_unlock(&mtx);

	return r;
}
EXPORT_SYMBOL(dsscomp_get_ovl);

/* set manager info */
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	mutex_lock(&mtx);

	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);
	BUG_ON(mgr->ix != comp->frm.mgr.ix);

	comp->frm.mgr = *mgr;

	mutex_unlock(&mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_set_mgr);

/* get manager info */
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	mutex_lock(&mtx);

	BUG_ON(!mgr);
	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);

	*mgr = comp->frm.mgr;

	mutex_unlock(&mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_get_mgr);

/* get manager info */
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win)
{
	mutex_lock(&mtx);

	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);

	comp->frm.mode = mode;
	comp->frm.win = win;

	mutex_unlock(&mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_setup);

/*
 * ===========================================================================
 *		QUEUING COMMITTING OPERATIONS
 * ===========================================================================
 */
void dsscomp_drop(dsscomp_t comp)
{
	/* decrement unprogrammed references */
	if (comp->state < DSSCOMP_STATE_PROGRAMMED)
		maskref_decmask(&mgrq[comp->ix].ovl_qmask, comp->ovl_mask);
	comp->state = 0;

	if (debug & DEBUG_COMPOSITIONS)
		dev_info(DEV(cdev), "[%p] released\n", comp);

	DO_IF_DEBUG_FS(list_del(&comp->dbg_q));

	kfree(comp);
}
EXPORT_SYMBOL(dsscomp_drop);

struct dsscomp_cb_work {
	struct work_struct work;
	struct dsscomp_data *comp;
	int status;
};

static void dsscomp_mgr_delayed_cb(struct work_struct *work)
{
	struct dsscomp_cb_work *wk = container_of(work, typeof(*wk), work);
	struct dsscomp_data *comp = wk->comp;
	int status = wk->status;
	u32 ix;

	kfree(work);

	mutex_lock(&mtx);

	BUG_ON(comp->state == DSSCOMP_STATE_ACTIVE);
	ix = comp->ix;

	/* call extra callbacks if requested */
	if (comp->extra_cb)
		comp->extra_cb(comp->extra_cb_data, status);

	/* handle programming & release */
	if (status == DSS_COMPLETION_PROGRAMMED) {
		comp->state = DSSCOMP_STATE_PROGRAMMED;
		log_state(comp, dsscomp_mgr_delayed_cb, status);

		/* update used overlay mask */
		mgrq[ix].ovl_mask = comp->ovl_mask & ~comp->ovl_dmask;
		maskref_decmask(&mgrq[ix].ovl_qmask, comp->ovl_mask);

		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%p] programmed\n", comp);
	} else if ((status == DSS_COMPLETION_DISPLAYED) &&
		   comp->state == DSSCOMP_STATE_PROGRAMMED) {
		/* composition is 1st displayed */
		comp->state = DSSCOMP_STATE_DISPLAYED;
		log_state(comp, dsscomp_mgr_delayed_cb, status);
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%p] displayed\n", comp);
	} else if (status & DSS_COMPLETION_RELEASED) {
		/* composition is no longer displayed */
		log_event(20 * comp->ix + 20, 0, comp, "%pf on %s",
				(u32) dsscomp_mgr_delayed_cb,
				(u32) log_status_str(status));
		dsscomp_drop(comp);
	}
	mutex_unlock(&mtx);
}

static u32 dsscomp_mgr_callback(void *data, int id, int status)
{
	struct dsscomp_data *comp = data;

	if (status == DSS_COMPLETION_PROGRAMMED ||
	    (status == DSS_COMPLETION_DISPLAYED &&
	     comp->state != DSSCOMP_STATE_DISPLAYED) ||
	    (status & DSS_COMPLETION_RELEASED)) {
		struct dsscomp_cb_work *wk = kzalloc(sizeof(*wk), GFP_ATOMIC);
		wk->comp = comp;
		wk->status = status;
		INIT_WORK(&wk->work, dsscomp_mgr_delayed_cb);
		queue_work(cb_wkq, &wk->work);
	}

	/* get each callback only once */
	return ~status;
}

static inline bool dssdev_manually_updated(struct omap_dss_device *dev)
{
	return dev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
		dev->driver->get_update_mode(dev) != OMAP_DSS_UPDATE_AUTO;
}

/* apply composition */
/* at this point the composition is not on any queue */
static int dsscomp_apply(dsscomp_t comp)
{
	int i, r = -EFAULT;
	u32 dmask, display_ix;
	struct omap_dss_device *dssdev;
	struct omap_dss_driver *drv;
	struct omap_overlay_manager *mgr;
	struct omap_overlay *ovl;
	struct dsscomp_setup_mgr_data *d;
	u32 oix;
	bool cb_programmed = false;

	struct omapdss_ovl_cb cb = {
		.fn = dsscomp_mgr_callback,
		.data = comp,
		.mask = DSS_COMPLETION_DISPLAYED |
		DSS_COMPLETION_PROGRAMMED | DSS_COMPLETION_RELEASED,
	};

	BUG_ON(comp->state != DSSCOMP_STATE_APPLYING);

	/* check if the display is valid and used */
	r = -ENODEV;
	d = &comp->frm;
	display_ix = d->mgr.ix;
	if (display_ix >= cdev->num_displays)
		goto done;
	dssdev = cdev->displays[display_ix];
	if (!dssdev)
		goto done;

	drv = dssdev->driver;
	mgr = dssdev->manager;
	if (!mgr || !drv || mgr->id >= cdev->num_mgrs)
		goto done;

	dump_comp_info(cdev, d, "apply");

	r = 0;
	dmask = 0;
	for (oix = 0; oix < comp->frm.num_ovls; oix++) {
		struct dss2_ovl_info *oi = comp->ovls + oix;

		/* keep track of disabled overlays */
		if (!oi->cfg.enabled)
			dmask |= 1 << oi->cfg.ix;

		if (r && !comp->must_apply)
			continue;

		dump_ovl_info(cdev, oi);

		if (oi->cfg.ix >= cdev->num_ovls) {
			r = -EINVAL;
			continue;
		}
		ovl = cdev->ovls[oi->cfg.ix];

		/* set overlays' manager & info */
		if (ovl->info.enabled && ovl->manager != mgr) {
			r = -EBUSY;
			goto skip_ovl_set;
		}
		if (ovl->manager != mgr) {
			/*
			 * Ideally, we should call ovl->unset_manager(ovl),
			 * but it may block on go even though the disabling
			 * of the overlay already went through.  So instead,
			 * we are just clearing the manager.
			 */
			ovl->manager = NULL;
			r = ovl->set_manager(ovl, mgr);
			if (r)
				goto skip_ovl_set;
		}

		r = set_dss_ovl_info(oi);
skip_ovl_set:
		if (r && comp->must_apply) {
			dev_err(DEV(cdev), "[%p] set ovl%d failed %d", comp,
								oi->cfg.ix, r);
			oi->cfg.enabled = false;
			dmask |= 1 << oi->cfg.ix;
			set_dss_ovl_info(oi);
		}
	}

	/*
	 * set manager's info - this also sets the completion callback,
	 * so if it succeeds, we will use the callback to complete the
	 * composition.  Otherwise, we can skip the composition now.
	 */
	if (!r || comp->must_apply) {
		r = set_dss_mgr_info(&d->mgr, &cb);
		cb_programmed = r == 0;
	}

	if (r && !comp->must_apply) {
		dev_err(DEV(cdev), "[%p] set failed %d\n", comp, r);
		goto done;
	} else {
		if (r)
			dev_warn(DEV(cdev), "[%p] ignoring set failure %d\n",
								comp, r);
		comp->blank = dmask == comp->ovl_mask;
		comp->ovl_dmask = dmask;

		/*
		 * Check other overlays that may also use this display.
		 * NOTE: This is only needed in case someone changes
		 * overlays via sysfs.  We use comp->ovl_mask to refresh
		 * the overlays actually used on a manager when the
		 * composition is programmed.
		 */
		for (i = 0; i < cdev->num_ovls; i++) {
			u32 mask = 1 << i;
			if ((~comp->ovl_mask & mask) &&
			    cdev->ovls[i]->info.enabled &&
			    cdev->ovls[i]->manager == mgr) {
				mutex_lock(&mtx);
				comp->ovl_mask |= mask;
				maskref_incbit(&mgrq[comp->ix].ovl_qmask, i);
				mutex_unlock(&mtx);
			}
		}
	}

	/* apply changes and call update on manual panels */
	/* no need for mutex as no callbacks are scheduled yet */
	comp->state = DSSCOMP_STATE_APPLIED;
	log_state(comp, dsscomp_apply, 0);

	if (!d->win.w && !d->win.x)
		d->win.w = dssdev->panel.timings.x_res - d->win.x;
	if (!d->win.h && !d->win.y)
		d->win.h = dssdev->panel.timings.y_res - d->win.y;

	mutex_lock(&mtx);
	if (mgrq[comp->ix].blanking) {
		pr_info_ratelimited("ignoring apply mgr(%s) while blanking\n",
				    mgr->name);
		r = -ENODEV;
	} else {
		r = mgr->apply(mgr);
		if (r)
			dev_err(DEV(cdev), "failed while applying %d", r);
		/* keep error if set_mgr_info failed */
		if (!r && !cb_programmed)
			r = -EINVAL;
	}
	mutex_unlock(&mtx);

	/*
	 * TRICKY: try to unregister callback to see if callbacks have
	 * been applied (moved into DSS2 pipeline).  Unregistering also
	 * avoids having to unnecessarily kick out compositions (which
	 * would result in screen blinking).  If callbacks failed to apply,
	 * (e.g. could not set them or apply them) we will need to call
	 * them ourselves (we note this by returning an error).
	 */
	if (cb_programmed && r) {
		/* clear error if callback already registered */
		if (omap_dss_manager_unregister_callback(mgr, &cb))
			r = 0;
	}
	/* if failed to apply, kick out prior composition */
	if (comp->must_apply && r)
		mgr->blank(mgr, true);

	if (!r && (d->mode & DSSCOMP_SETUP_MODE_DISPLAY)) {
		/* cannot handle update errors, so ignore them */
		if (dssdev_manually_updated(dssdev) && drv->update)
			drv->update(dssdev, d->win.x,
					d->win.y, d->win.w, d->win.h);
		else
			/* wait for sync to do smooth animations */
			mgr->wait_for_vsync(mgr);
	}

done:
	return r;
}

struct dsscomp_apply_work {
	struct work_struct work;
	dsscomp_t comp;
};

int dsscomp_state_notifier(struct notifier_block *nb,
						unsigned long arg, void *ptr)
{
	struct omap_dss_device *dssdev = ptr;
	enum omap_dss_display_state state = arg;
	struct omap_overlay_manager *mgr = dssdev->manager;
	if (mgr) {
		mutex_lock(&mtx);
		if (state == OMAP_DSS_DISPLAY_DISABLED) {
			mgr->blank(mgr, true);
			mgrq[mgr->id].blanking = true;
		} else if (state == OMAP_DSS_DISPLAY_ACTIVE) {
			mgrq[mgr->id].blanking = false;
		}
		mutex_unlock(&mtx);
	}
	return 0;
}


static void dsscomp_do_apply(struct work_struct *work)
{
	struct dsscomp_apply_work *wk = container_of(work, typeof(*wk), work);
	/* complete compositions that failed to apply */
	if (dsscomp_apply(wk->comp))
		dsscomp_mgr_callback(wk->comp, -1, DSS_COMPLETION_ECLIPSED_SET);
	kfree(wk);
}

int dsscomp_delayed_apply(dsscomp_t comp)
{
	/* don't block in case we are called from interrupt context */
	struct dsscomp_apply_work *wk = kzalloc(sizeof(*wk), GFP_NOWAIT);
	if (!wk)
		return -ENOMEM;

	mutex_lock(&mtx);

	BUG_ON(comp->state != DSSCOMP_STATE_ACTIVE);
	comp->state = DSSCOMP_STATE_APPLYING;
	log_state(comp, dsscomp_delayed_apply, 0);

	if (debug & DEBUG_PHASES)
		dev_info(DEV(cdev), "[%p] applying\n", comp);
	mutex_unlock(&mtx);

	wk->comp = comp;
	INIT_WORK(&wk->work, dsscomp_do_apply);
	return queue_work(mgrq[comp->ix].apply_workq, &wk->work) ? 0 : -EBUSY;
}
EXPORT_SYMBOL(dsscomp_delayed_apply);

/*
 * ===========================================================================
 *		DEBUGFS
 * ===========================================================================
 */

#ifdef CONFIG_DEBUG_FS
void seq_print_comp(struct seq_file *s, dsscomp_t c)
{
	struct dsscomp_setup_mgr_data *d = &c->frm;
	int i;

	seq_printf(s, "  [%p]: %s%s\n", c, c->blank ? "blank " : "",
		   c->state == DSSCOMP_STATE_ACTIVE ? "ACTIVE" :
		   c->state == DSSCOMP_STATE_APPLYING ? "APPLYING" :
		   c->state == DSSCOMP_STATE_APPLIED ? "APPLIED" :
		   c->state == DSSCOMP_STATE_PROGRAMMED ? "PROGRAMMED" :
		   c->state == DSSCOMP_STATE_DISPLAYED ? "DISPLAYED" :
		   "???");
	seq_printf(s, "    sync_id=%x, flags=%c%c%c\n",
		   d->sync_id,
		   (d->mode & DSSCOMP_SETUP_MODE_APPLY) ? 'A' : '-',
		   (d->mode & DSSCOMP_SETUP_MODE_DISPLAY) ? 'D' : '-',
		   (d->mode & DSSCOMP_SETUP_MODE_CAPTURE) ? 'C' : '-');
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi;
		struct dss2_ovl_cfg *g;
		oi = d->ovls + i;
		g = &oi->cfg;
		if (g->zonly) {
			seq_printf(s, "    ovl%d={%s z%d}\n",
				   g->ix, g->enabled ? "ON" : "off", g->zorder);
		} else {
			seq_printf(s, "    ovl%d={%s z%d %s%s *%d%%"
						" %d*%d:%d,%d+%d,%d rot%d%s"
						" => %d,%d+%d,%d %p/%p|%d}\n",
				g->ix, g->enabled ? "ON" : "off", g->zorder,
				dsscomp_get_color_name(g->color_mode) ? : "N/A",
				g->pre_mult_alpha ? " premult" : "",
				(g->global_alpha * 100 + 128) / 255,
				g->width, g->height, g->crop.x, g->crop.y,
				g->crop.w, g->crop.h,
				g->rotation, g->mirror ? "+mir" : "",
				g->win.x, g->win.y, g->win.w, g->win.h,
				(void *) oi->ba, (void *) oi->uv, g->stride);
		}
	}
	if (c->extra_cb)
		seq_printf(s, "    gsync=[%p] %pf\n\n", c->extra_cb_data,
								c->extra_cb);
	else
		seq_printf(s, "    gsync=[%p] (called)\n\n", c->extra_cb_data);
}
#endif

void dsscomp_dbg_comps(struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	dsscomp_t c;
	u32 i;

	mutex_lock(&dbg_mtx);
	for (i = 0; i < cdev->num_mgrs; i++) {
		struct omap_overlay_manager *mgr = cdev->mgrs[i];
		seq_printf(s, "ACTIVE COMPOSITIONS on %s\n\n", mgr->name);
		list_for_each_entry(c, &dbg_comps, dbg_q) {
			struct dss2_mgr_info *mi = &c->frm.mgr;
			if (mi->ix < cdev->num_displays &&
			    cdev->displays[mi->ix]->manager == mgr)
				seq_print_comp(s, c);
		}

		/* print manager cache */
		mgr->dump_cb(mgr, s);
	}
	mutex_unlock(&dbg_mtx);
#endif
}

void dsscomp_dbg_events(struct seq_file *s)
{
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
	u32 i;
	struct dbg_event_t *d;

	mutex_lock(&dbg_mtx);
	for (i = dbg_event_ix; i < dbg_event_ix + ARRAY_SIZE(dbg_events); i++) {
		d = dbg_events + (i % ARRAY_SIZE(dbg_events));
		if (!d->ms)
			continue;
		seq_printf(s, "[% 5d.%03d] %*s[%08x] ",
			   d->ms / 1000, d->ms % 1000,
			   d->ix + ((u32) d->data) % 7,
			   "", (u32) d->data);
		seq_printf(s, d->fmt, d->a1, d->a2);
		seq_printf(s, "\n");
	}
	mutex_unlock(&dbg_mtx);
#endif
}

/*
 * ===========================================================================
 *		EXIT
 * ===========================================================================
 */
void dsscomp_queue_exit(void)
{
	if (cdev) {
		int i;
		for (i = 0; i < cdev->num_displays; i++)
			destroy_workqueue(mgrq[i].apply_workq);
		destroy_workqueue(cb_wkq);
		cdev = NULL;
	}
}
EXPORT_SYMBOL(dsscomp_queue_exit);
