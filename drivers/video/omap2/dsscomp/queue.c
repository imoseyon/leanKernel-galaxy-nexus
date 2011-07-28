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

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

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
} mgrq[MAX_MANAGERS];

static struct workqueue_struct *cb_wkq;		/* callback work queue */
static struct dsscomp_dev *cdev;

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

		/* update used overlay mask */
		mgrq[ix].ovl_mask = comp->ovl_mask & ~comp->ovl_dmask;
		maskref_decmask(&mgrq[ix].ovl_qmask, comp->ovl_mask);

		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%p] programmed\n", comp);
	} else if ((status == DSS_COMPLETION_DISPLAYED) &&
		   comp->state == DSSCOMP_STATE_PROGRAMMED) {
		/* composition is 1st displayed */
		comp->state = DSSCOMP_STATE_DISPLAYED;
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%p] displayed\n", comp);
	} else if (status & DSS_COMPLETION_RELEASED) {
		/* composition is no longer displayed */
		dsscomp_drop(comp);
	}
	mutex_unlock(&mtx);
}

static u32 dsscomp_mgr_callback(void *data, int id, int status)
{
	struct dsscomp_data *comp = data;

	/* do any other callbacks */
	dss_ovl_cb(&comp->cb, id, status);

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
	return ~status | (comp->cb.fn ? comp->cb.mask : 0);
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
	if (!r || comp->must_apply)
		r = set_dss_mgr_info(&d->mgr);

	if (r && !comp->must_apply) {
		dev_err(DEV(cdev), "[%p] set failed %d\n", comp, r);
		/* extra callbacks in case of delayed apply */
		dsscomp_mgr_callback(comp, mgr->id, DSS_COMPLETION_ECLIPSED_SET);
		goto done;
	} else {
		if (r)
			dev_warn(DEV(cdev), "[%p] ignoring set failure %d\n",
								comp, r);
		/* override manager's callback to avoid eclipsed cb */
		comp->blank = dmask == comp->ovl_mask;
		comp->ovl_dmask = dmask;
		comp->cb = mgr->info.cb;
		mgr->info.cb.fn = dsscomp_mgr_callback;
		mgr->info.cb.data = comp;
		mgr->info.cb.mask = DSS_COMPLETION_DISPLAYED |
			DSS_COMPLETION_PROGRAMMED | DSS_COMPLETION_RELEASED;

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

	if (!d->win.w && !d->win.x)
		d->win.w = dssdev->panel.timings.x_res - d->win.x;
	if (!d->win.h && !d->win.y)
		d->win.h = dssdev->panel.timings.y_res - d->win.y;

	r = mgr->apply(mgr);
	if (r)
		dev_err(DEV(cdev), "failed while applying %d", r);

	/* ignore this error if callback has already been registered */
	if (!mgr->info_dirty)
		r = 0;

	if (!r && (d->mode & DSSCOMP_SETUP_MODE_DISPLAY)) {
		if (dssdev_manually_updated(dssdev) && drv->update)
			r = drv->update(dssdev, d->win.x,
					d->win.y, d->win.w, d->win.h);
		else
			/* wait for sync to do smooth animations */
			r = mgr->wait_for_vsync(mgr);
	}

done:
	return r;
}
EXPORT_SYMBOL(dsscomp_apply);

struct dsscomp_apply_work {
	struct work_struct work;
	dsscomp_t comp;
};

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
