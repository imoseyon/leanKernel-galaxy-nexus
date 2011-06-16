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
static LIST_HEAD(free_ois);

#define QUEUE_SIZE	3

#define MUTEXED(exp) ({ typeof(exp) __r;\
	mutex_lock(&mtx); __r = (exp); mutex_unlock(&mtx); __r; })

#undef STRICT_CHECK

#define MAGIC_ACTIVE		0xAC54156E
#define MAGIC_APPLIED		0xA50504C1
#define MAGIC_PROGRAMMED	0x50520652
#define MAGIC_DISPLAYED		0xD15504CA

static struct {
	struct list_head q_ci;		/* compositions */

	wait_queue_head_t wq;

	u32 ovl_mask;		/* overlays used on this display */
	u32 ovl_qmask;		/* overlays queued to this display */
} mgrq[MAX_MANAGERS];

static struct workqueue_struct *wkq;	/* work queue */
static struct dsscomp_dev *cdev;

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

	INIT_LIST_HEAD(&free_ois);

	if (ARRAY_SIZE(mgrq) < cdev->num_mgrs)
		return -EINVAL;

	ZERO(mgrq);
	for (i = 0; i < cdev->num_mgrs; i++) {
		struct omap_overlay_manager *mgr;
		INIT_LIST_HEAD(&mgrq[i].q_ci);
		init_waitqueue_head(&mgrq[i].wq);

		/* record overlays on this display */
		mgr = cdev->mgrs[i];
		for (j = 0; j < cdev->num_ovls; j++) {
			if (cdev->ovls[j]->info.enabled &&
			    mgr &&
			    cdev->ovls[j]->manager == mgr)
				mgrq[i].ovl_mask |= 1 << j;
		}
	}

	wkq = create_workqueue("dsscomp");
	if (!wkq)
		return -EFAULT;

	return 0;
}

/* returns if composition is valid and active */
static inline struct dsscomp_data *validate(struct dsscomp_data *comp)
{
#ifdef STRICT_CHECK
	u32 ix, q_ix;
	struct dsscomp_data *c;
	struct omap_overlay_manager *mgr;

	if (!comp)
		return ERR_PTR(-EFAULT);

	ix = comp->frm.mgr.ix;
	if (ix >= cdev->num_displays || !cdev->displays[ix])
		return ERR_PTR(-EINVAL);
	mgr = cdev->displays[ix]->manager;
	if (!mgr || mgr->id >= cdev->num_mgrs)
		return ERR_PTR(-ENODEV);

	/* check if composition is active */
	list_for_each_entry(c, &mgrq[mgr->id].q_ci, q)
		if (c == comp)
			return c;
	return ERR_PTR(-ESRCH);
#else
	if (!comp)
		return ERR_PTR(-EFAULT);

	return (comp->magic == MAGIC_PROGRAMMED ||
		comp->magic == MAGIC_DISPLAYED ||
		comp->magic == MAGIC_APPLIED ||
		comp->magic == MAGIC_ACTIVE) ? comp : ERR_PTR(-ESRCH);
#endif
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
	int r;
	u32 display_ix = get_display_ix(mgr);

	/* check manager */
	u32 ix = mgr ? mgr->id : cdev->num_mgrs;
	if (ix >= cdev->num_mgrs || display_ix >= cdev->num_displays)
		return ERR_PTR(-EINVAL);

	mutex_lock(&mtx);

	/* allocate composition */
	comp = kzalloc(sizeof(*comp), GFP_KERNEL);
	if (!comp)
		goto done;

	/* initialize new composition */
	list_add_tail(&comp->q, &mgrq[ix].q_ci);

	comp->ix = ix;	/* save where this composition came from */
	comp->ovl_mask = comp->ovl_dmask = 0;
	comp->frm.sync_id = 0;
	comp->frm.mgr.ix = display_ix;

	/* :TODO: retrieve last manager configuration */

	comp->magic = MAGIC_ACTIVE;
	r = 0;
 done:
	mutex_unlock(&mtx);
	return r ? ERR_PTR(r) : comp;
}
EXPORT_SYMBOL(dsscomp_new);

/* returns overlays used in a composition */
u32 dsscomp_get_ovls(dsscomp_t comp)
{
	BUG_ON(MUTEXED(IS_ERR(validate(comp))));

	return comp->ovl_mask;
}
EXPORT_SYMBOL(dsscomp_get_ovls);

/* set overlay info */
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl)
{
	int r = -EFAULT;

	if (comp && ovl) {
		u32 i, mask = 1 << ovl->cfg.ix;
		struct omap_overlay *o;
		u32 oix;
		u32 ix = comp->frm.mgr.ix;
		if (ix < cdev->num_displays &&
		    cdev->displays[ix] &&
		    cdev->displays[ix]->manager)
			ix = cdev->displays[ix]->manager->id;
		else
			ix = cdev->num_mgrs;
		if (ix >= cdev->num_mgrs)
			return -ENODEV;

		mutex_lock(&mtx);

		/* check if composition is active */
		comp = validate(comp);
		if (IS_ERR(comp)) {
			r = PTR_ERR(comp);
			goto done;
		}

		if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
			goto done;
		}

		if (ovl->cfg.ix >= cdev->num_ovls) {
			r = -EINVAL;
			goto done;
		}

		/* if overlay is already part of the composition */
		if (mask & comp->ovl_mask) {
			/* look up overlay */
			for (oix = 0; oix < comp->frm.num_ovls; oix++)
				if (comp->ovls[oix].cfg.ix == ovl->cfg.ix)
					break;
			BUG_ON(oix == comp->frm.num_ovls);
		} else {
			/* check if ovl is free to use */
			r = -EBUSY;
			if (comp->frm.num_ovls >= ARRAY_SIZE(comp->ovls))
				goto done;

			/* not in any other displays queue */
			if (mask & ~mgrq[ix].ovl_qmask) {
				for (i = 0; i < cdev->num_mgrs; i++) {

					if (i == ix)
						continue;
					if (mgrq[i].ovl_qmask & mask)
						goto done;
				}
			}

			/* and disabled (unless forced) if on another manager */
			o = cdev->ovls[ovl->cfg.ix];
			if (o->info.enabled &&
					(!o->manager || o->manager->id != ix))
				goto done;

			/* add overlay to composition & display */
			comp->ovl_mask |= mask;
			oix = comp->frm.num_ovls++;
			mgrq[ix].ovl_qmask |= mask;
		}

		comp->ovls[oix] = *ovl;
		r = 0;
 done:
		mutex_unlock(&mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_set_ovl);

/* get overlay info */
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl)
{
	int r = -EFAULT;
	u32 oix;

	if (comp && ovl) {
		mutex_lock(&mtx);

		/* check if composition is active */
		comp = validate(comp);
		if (IS_ERR(comp)) {
			r = PTR_ERR(comp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			if (ix >= cdev->num_ovls) {
				r = -EINVAL;
			} else if (comp->ovl_mask & (1 << ix)) {
				r = 0;
				for (oix = 0; oix < comp->frm.num_ovls; oix++)
					if (comp->ovls[oix].cfg.ix ==
								ovl->cfg.ix) {
						*ovl = comp->ovls[oix];
						break;
					}
				BUG_ON(oix == comp->frm.num_ovls);
			} else {
				/* :TODO: get past overlay info */
				r = -ENOENT;
			}
		}

		mutex_unlock(&mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_get_ovl);

/* set manager info */
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	int r = -EFAULT;

	if (comp && mgr) {
		mutex_lock(&mtx);

		/* check if composition is active */
		comp = validate(comp);
		if (IS_ERR(comp)) {
			r = PTR_ERR(comp);
			goto done;
		}

		if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
			goto done;
		}

		/* set display index in manager info */
		mgr->ix = comp->frm.mgr.ix;
		comp->frm.mgr = *mgr;
		r = 0;
 done:
		mutex_unlock(&mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_set_mgr);

/* get manager info */
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr)
{
	int r = -EFAULT;

	if (comp && mgr) {
		mutex_lock(&mtx);

		/* check if composition is active */
		comp = validate(comp);
		if (IS_ERR(comp)) {
			r = PTR_ERR(comp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			r = 0;
			*mgr = comp->frm.mgr;
		}

		mutex_unlock(&mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_get_mgr);

/* get manager info */
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win)
{
	int r = -EFAULT;

	if (comp) {
		mutex_lock(&mtx);

		/* check if composition is active */
		comp = validate(comp);
		if (IS_ERR(comp)) {
			r = PTR_ERR(comp);
		} else if (comp->magic != MAGIC_ACTIVE) {
			r = -EACCES;
		} else {
			r = 0;
			comp->frm.mode = mode;
			comp->frm.win = win;
		}

		mutex_unlock(&mtx);
	}

	return r;
}
EXPORT_SYMBOL(dsscomp_setup);

/*
 * ===========================================================================
 *		QUEUING COMMITTING OPERATIONS
 * ===========================================================================
 */

static void refresh_masks(u32 ix)
{
	struct dsscomp_data *c;

	mgrq[ix].ovl_qmask = mgrq[ix].ovl_mask;
	list_for_each_entry(c, &mgrq[ix].q_ci, q) {
		if (c->magic != MAGIC_PROGRAMMED)
			mgrq[ix].ovl_qmask |= c->ovl_mask;
	}

	wake_up_interruptible_sync(&mgrq[ix].wq);
}

void dsscomp_drop(dsscomp_t c)
{
	if (debug & DEBUG_COMPOSITIONS)
		dev_info(DEV(cdev), "[%p] released\n", c);

	list_del(&c->q);
	c->magic = 0;
	kfree(c);
}
EXPORT_SYMBOL(dsscomp_drop);

static void dsscomp_mgr_callback(void *data, int id, int status)
{
	struct dsscomp_data *comp = data;
	u32 ix;

	/* do any other callbacks */
	if (comp->cb.fn)
		comp->cb.fn(comp->cb.data, id, status);

	/* verify validity */
	comp = validate(comp);
	if (IS_ERR(comp))
		return;

	ix = comp->frm.mgr.ix;
	if (ix >= cdev->num_displays ||
	    !cdev->displays[ix] ||
	    !cdev->displays[ix]->manager)
		return;
	ix = cdev->displays[ix]->manager->id;
	if (ix >= cdev->num_mgrs)
		return;

	/* call extra callbacks if requested */
	if (comp->extra_cb)
		comp->extra_cb(comp, status);

	/* handle programming & release */
	if (status == DSS_COMPLETION_PROGRAMMED) {
		comp->magic = MAGIC_PROGRAMMED;
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev),
				"[%p] programmed\n", comp);

		/* update used overlay mask */
		mgrq[ix].ovl_mask = comp->ovl_mask & ~comp->ovl_dmask;

		/* if all overlays were disabled, the composition is complete */
		if (comp->blank)
			dsscomp_drop(comp);
		refresh_masks(ix);
	} else if ((status & DSS_COMPLETION_DISPLAYED) &&
		   comp->magic == MAGIC_PROGRAMMED) {
		/* composition is 1st displayed */
		comp->magic = MAGIC_DISPLAYED;
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev),
				"[%p] displayed\n", comp);
		wake_up_interruptible_sync(&mgrq[ix].wq);
	} else if (status & DSS_COMPLETION_RELEASED) {
		/* composition is no longer displayed */
		dsscomp_drop(comp);
		refresh_masks(ix);
	}
}

static inline bool dssdev_manually_updated(struct omap_dss_device *dev)
{
	return dev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
		dev->driver->get_update_mode(dev) != OMAP_DSS_UPDATE_AUTO;
}

/* apply composition */
int dsscomp_apply(dsscomp_t comp)
{
	int i, r = -EFAULT;
	u32 dmask, display_ix;
	struct omap_dss_device *dssdev;
	struct omap_dss_driver *drv;
	struct omap_overlay_manager *mgr;
	struct omap_overlay *ovl;
	struct dsscomp_setup_mgr_data *d;
	struct dsscomp_data *c, *c2;
	u32 oix;
	bool change = false;

	mutex_lock(&mtx);

	/* check if composition is active */
	comp = validate(comp);
	if (IS_ERR(comp)) {
		r = PTR_ERR(comp);
		goto done;
	}

	if (comp->magic != MAGIC_ACTIVE) {
		r = -EACCES;
		goto done;
	}

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

	/* skip all unapplied prior compositions */
	list_for_each_entry_safe(c, c2, &mgrq[mgr->id].q_ci, q) {
		if (c == comp)
			break;

		/* keep applied compositions, as callback has been scheduled */
		if (c->magic == MAGIC_ACTIVE) {
			dsscomp_drop(c);
			change = true;
		}
	}

	dump_comp_info(cdev, d, "apply");

	r = 0;
	dmask = 0;
	for (oix = 0; oix < comp->frm.num_ovls; oix++) {
		struct dss2_ovl_info *oi = comp->ovls + oix;

		/* keep track of disabled overlays */
		if (!oi->cfg.enabled)
			dmask |= 1 << oi->cfg.ix;

		if (r)
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
			continue;
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
				continue;
		}

		r = set_dss_ovl_info(oi);
	}

	/*
	 * set manager's info - this also sets the completion callback,
	 * so if it succeeds, we will use the callback to complete the
	 * composition.  Otherwise, we can skip the composition now.
	 */
	r = r ? : set_dss_mgr_info(&d->mgr);
	if (r) {
		dev_err(DEV(cdev), "[%p] set failed %d\n", comp, r);
		/* FIXME: this only needs to be called for delayed apply */
		dsscomp_mgr_callback(comp, -1, DSS_COMPLETION_ECLIPSED_SET);
		dsscomp_drop(comp);
		change = true;
		goto done;
	} else {
		/* override manager's callback to avoid eclipsed cb */
		comp->blank = dmask == comp->ovl_mask;
		comp->ovl_dmask = dmask;
		comp->cb = mgr->info.cb;
		mgr->info.cb.fn = dsscomp_mgr_callback;
		mgr->info.cb.data = comp;

		/*
		 * Check other overlays that may also use this display.
		 * NOTE: This is only needed in case someone changes
		 * overlays via sysfs.
		 */
		for (i = 0; i < cdev->num_ovls; i++) {
			u32 mask = 1 << i;
			if ((~comp->ovl_mask & mask) &&
			    cdev->ovls[i]->info.enabled &&
			    cdev->ovls[i]->manager == mgr)
				comp->ovl_mask |= mask;
		}
	}

	/* apply changes and call update on manual panels */
	comp->magic = MAGIC_APPLIED;

	if (dssdev_manually_updated(dssdev)) {
		if (!d->win.w && !d->win.x)
			d->win.w = dssdev->panel.timings.x_res - d->win.x;
		if (!d->win.h && !d->win.y)
			d->win.h = dssdev->panel.timings.y_res - d->win.y;

		/* sync to prevent frame loss */
		r = drv->sync(dssdev) ? : mgr->apply(mgr);

		if (!r && (d->mode & DSSCOMP_SETUP_MODE_DISPLAY)) {
#if 0
			/* schedule update if supported */
			if (drv->sched_update)
				r = drv->sched_update(dssdev, d->win.x,
					d->win.y, d->win.w, d->win.h);
			else if (drv->update)
#else
			if (drv->update)
#endif
				r = drv->update(dssdev, d->win.x,
					d->win.y, d->win.w, d->win.h);
		}
	} else {
		/* wait for sync to avoid tear */
		r = mgr->wait_for_vsync(mgr) ? : mgr->apply(mgr);
	}
done:
	if (change)
		refresh_masks(display_ix);

	mutex_unlock(&mtx);

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
	wk->comp = comp;
	INIT_WORK(&wk->work, dsscomp_do_apply);
	return queue_work(wkq, &wk->work) ? 0 : -EBUSY;
}
EXPORT_SYMBOL(dsscomp_delayed_apply);

/*
 * ===========================================================================
 *		WAIT OPERATIONS
 * ===========================================================================
 */

/* return true iff composition phase has passed */
static bool is_wait_over(dsscomp_t comp, enum dsscomp_wait_phase phase)
{
	comp = validate(comp);
	return IS_ERR(comp) ||
		(phase == DSSCOMP_WAIT_PROGRAMMED &&
			(comp->magic == MAGIC_PROGRAMMED ||
			 comp->magic == MAGIC_DISPLAYED)) ||
		(phase == DSSCOMP_WAIT_DISPLAYED &&
			comp->magic == MAGIC_DISPLAYED);
}

/* wait for programming or release of a composition */
int dsscomp_wait(dsscomp_t comp, enum dsscomp_wait_phase phase, int timeout)
{
	dsscomp_t id;

	mutex_lock(&mtx);

	comp = validate(comp);
	id = IS_ERR(comp) ? NULL : comp;
	if (debug & DEBUG_WAITS)
		dev_info(DEV(cdev), "wait %s on [%p]\n",
			phase == DSSCOMP_WAIT_DISPLAYED ? "display" :
			phase == DSSCOMP_WAIT_PROGRAMMED ? "program" :
			"release", id);

	if (!IS_ERR(comp) && !is_wait_over(comp, phase)) {
		u32 ix = comp->frm.mgr.ix;

		mutex_unlock(&mtx);

		/*
		 * we can check being active without mutex because we will
		 * also check it while holding the mutex before returning
		 */
		timeout = wait_event_interruptible_timeout(mgrq[ix].wq,
			is_wait_over(comp, phase), timeout);
		if (debug & DEBUG_WAITS)
			dev_info(DEV(cdev), "wait over [%p]: %s %d\n", id,
				 timeout < 0 ? "signal" :
				 timeout > 0 ? "ok" : "timeout",
				 timeout);
		if (timeout <= 0)
			return timeout ? : -ETIME;

		mutex_lock(&mtx);
	}

	mutex_unlock(&mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_wait);

/*
 * ===========================================================================
 *		EXIT
 * ===========================================================================
 */
void dsscomp_queue_exit(void)
{
	struct dsscomp_data *c, *c2;
	if (cdev) {
		int i;
		for (i = 0; i < cdev->num_displays; i++) {
			list_for_each_entry_safe(c, c2, &mgrq[i].q_ci, q)
				dsscomp_drop(c);
		}
		destroy_workqueue(wkq);
		cdev = NULL;
	}
}
EXPORT_SYMBOL(dsscomp_queue_exit);
