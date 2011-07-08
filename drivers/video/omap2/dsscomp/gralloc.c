#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <mach/tiler.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
static bool blanked;

#define NUM_TILER1D_SLOTS 8
#define TILER1D_SLOT_SIZE (4 << 20)

static struct tiler1d_slot {
	struct list_head q;
	tiler_blk_handle slot;
	u32 phys;
} slots[NUM_TILER1D_SLOTS];
static struct list_head free_slots;
static struct dsscomp_dev *cdev;
static DEFINE_MUTEX(mtx);
static struct semaphore free_slots_sem =
				__SEMAPHORE_INITIALIZER(free_slots_sem, 0);

static u32 ovl_set_mask;

static void unpin_tiler_blocks(struct list_head *slots)
{
	struct tiler1d_slot *slot;

	/* unpin any tiler memory */
	list_for_each_entry(slot, slots, q) {
		tiler_unpin_block(slot->slot);
		up(&free_slots_sem);
	}

	/* free tiler slots */
	list_splice_init(slots, &free_slots);
}

static void dsscomp_gralloc_cb(dsscomp_t comp, int status)
{
	if (status & DSS_COMPLETION_RELEASED) {
		mutex_lock(&mtx);
		unpin_tiler_blocks(&comp->slots);
		mutex_unlock(&mtx);
	}
	if ((status == DSS_COMPLETION_DISPLAYED) ||
	    (status & DSS_COMPLETION_RELEASED)) {
		/* complete composition if eclipsed or displayed */
		if (comp->gralloc_cb_fn) {
			if (debug & DEBUG_PHASES)
				dev_info(DEV(cdev), "[%p] complete flip\n",
									comp);
			comp->gralloc_cb_fn(comp->gralloc_cb_arg, 1);
		}
		comp->gralloc_cb_fn = NULL;
	}
}

/* This is just test code for now that does the setup + apply.
   It still uses userspace virtual addresses, but maps non
   TILER buffers into 1D */
int dsscomp_gralloc_queue_ioctl(struct dsscomp_setup_mgr_data *d)
{
	struct tiler_pa_info *pas[MAX_OVERLAYS];
	s32 ret;
	u32 i;

	/* convert virtual addresses to physical and get tiler pa infos */
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 addr = (u32) oi->address;

		pas[i] = NULL;

		/* assume virtual NV12 for now */
		if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
			oi->uv = tiler_virt2phys(addr +
					oi->cfg.height * oi->cfg.stride);
		else
			oi->uv = 0;
		oi->ba = tiler_virt2phys(addr);

		/* map non-TILER buffers to 1D */
		if ((oi->ba < 0x60000000 || oi->ba >= 0x80000000) && oi->ba)
			pas[i] = user_block_to_pa(addr & PAGE_MASK,
				PAGE_ALIGN(oi->cfg.height * oi->cfg.stride +
					(addr & ~PAGE_MASK)) >> PAGE_SHIFT);
	}
	ret = dsscomp_gralloc_queue(d, pas, NULL, NULL);
	for (i = 0; i < d->num_ovls; i++)
		tiler_pa_free(pas[i]);
	return ret;
}

/* This is just test code for now that does the setup + apply.
   It still uses userspace virtual addresses, but maps non
   TILER buffers into 1D */
int dsscomp_gralloc_queue(struct dsscomp_setup_mgr_data *d,
			struct tiler_pa_info **pas,
			void (*cb_fn)(void *, int), void *cb_arg)
{
	u32 i;
	int r;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	dsscomp_t comp;
	u32 ovl_new_set_mask = 0;
	int skip;

	/* reserve tiler areas if not already done so */
	dsscomp_gralloc_init(cdev);

	dump_comp_info(cdev, d, "queue");
	for (i = 0; i < d->num_ovls; i++)
		dump_ovl_info(cdev, d->ovls + i);

	/* ignore frames while we are blanked */
	mutex_lock(&mtx);
	skip = blanked;
	/* mark blank frame by NULL tiler pa pointer */
	if (!skip && pas == NULL)
		blanked = true;
	mutex_unlock(&mtx);

	if (skip) {
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%08x] ignored\n", d->sync_id);
		if (cb_fn)
			cb_fn(cb_arg, 0);
		return 0;
	}

	/* verify display is valid and connected */
	if (d->mgr.ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[d->mgr.ix];
	if (!dev)
		return -EINVAL;
	mgr = dev->manager;
	if (!mgr)
		return -ENODEV;

	comp = dsscomp_new(mgr);
	if (IS_ERR(comp))
		return PTR_ERR(comp);
	INIT_LIST_HEAD(&comp->slots);

	comp->frm.mode = DSSCOMP_SETUP_DISPLAY;
	comp->must_apply = true;

	/* swap red & blue if requested */
	if (d->mgr.swap_rb) {
		swap_rb_in_mgr_info(&d->mgr);
		for (i = 0; i < d->num_ovls; i++)
			swap_rb_in_ovl_info(d->ovls + i);
	}

	/* NOTE: none of the dsscomp sets should fail as composition is new */
	r = dsscomp_set_mgr(comp, &d->mgr);
	if (r)
		dev_err(DEV(cdev), "failed to set mgr (%d)\n", r);
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;

		/* map non-TILER buffers to 1D */
		if (pas[i] && oi->cfg.enabled) {
			struct tiler1d_slot *slot = NULL;
			down(&free_slots_sem);
			mutex_lock(&mtx);
			slot = list_first_entry(&free_slots, typeof(*slot), q);
			r = tiler_pin_block(slot->slot, pas[i]->mem,
								pas[i]->num_pg);
			if (r) {
				dev_err(DEV(cdev), "failed to pin %d pages into"
					" %d-pg slots (%d)\n", pas[i]->num_pg,
					TILER1D_SLOT_SIZE >> PAGE_SHIFT, r);
				mutex_unlock(&mtx);
				up(&free_slots_sem);
				/* disable unpinned layers */
				oi->cfg.enabled = true;
				break;
			}
			list_move(&slot->q, &comp->slots);
			oi->ba = slot->phys + (oi->ba & ~PAGE_MASK);
			mutex_unlock(&mtx);
		}

		if (oi->cfg.enabled)
			ovl_new_set_mask |= 1 << oi->cfg.ix;

		r = dsscomp_set_ovl(comp, oi);
		if (r)
			dev_err(DEV(cdev), "failed to set ovl%d (%d)\n",
								oi->cfg.ix, r);
	}

	r = dsscomp_setup(comp, d->mode, d->win);
	if (r)
		dev_err(DEV(cdev), "failed to setup comp (%d)\n", r);

	if (r) {
		dsscomp_drop(comp);
	} else {
		/* disable all overlays not specifically set from prior frame */
		u32 mask = ovl_set_mask & ~ovl_new_set_mask;
		while (mask) {
			struct dss2_ovl_info oi = {
				.cfg.zonly = true,
				.cfg.enabled = false,
				.cfg.ix = fls(mask) - 1,
			};
			dsscomp_set_ovl(comp, &oi);
			mask &= ~(1 << oi.cfg.ix);
		}
		comp->extra_cb = dsscomp_gralloc_cb;
		comp->gralloc_cb_fn = cb_fn;
		comp->gralloc_cb_arg = cb_arg;
		r = dsscomp_delayed_apply(comp);
		if (r)
			dev_err(DEV(cdev), "failed to apply comp (%d)\n", r);
		else
			ovl_set_mask = ovl_new_set_mask;
	}

	/* complete composition if failed to queue */
	if (r && cb_fn)
		cb_fn(cb_arg, 0);

	return r;
}

#ifdef CONFIG_EARLYSUSPEND
static int blank_complete;
static DECLARE_WAIT_QUEUE_HEAD(early_suspend_wq);

static void dsscomp_early_suspend_cb(void *data, int status)
{
	blank_complete = true;
	wake_up_interruptible_sync(&early_suspend_wq);
}

static void dsscomp_early_suspend(struct early_suspend *h)
{
	struct dsscomp_setup_mgr_data d = {
		.mgr.alpha_blending = 1,
	};
	int err;

	pr_info("DSSCOMP: %s\n", __func__);

	/* use gralloc queue as we need to blank all screens */
	dsscomp_gralloc_queue(&d, NULL, dsscomp_early_suspend_cb, NULL);
	blank_complete = false;

	/* wait until composition is displayed */
	err = wait_event_timeout(early_suspend_wq, blank_complete,
				 msecs_to_jiffies(500));
	if (err == 0)
		pr_warn("DSSCOMP: timeout blanking screen\n");
	else
		pr_info("DSSCOMP: blanked screen\n");
}

static void dsscomp_late_resume(struct early_suspend *h)
{
	pr_info("DSSCOMP: %s\n", __func__);
	blanked = false;
}

static struct early_suspend early_suspend_info = {
	.suspend = dsscomp_early_suspend,
	.resume = dsscomp_late_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};
#endif

void dsscomp_gralloc_init(struct dsscomp_dev *cdev_)
{
	int i;

	/* save at least cdev pointer */
	if (!cdev && cdev_) {
		cdev = cdev_;

#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&early_suspend_info);
#endif
	}

	if (!free_slots.next) {
		INIT_LIST_HEAD(&free_slots);
		for (i = 0; i < NUM_TILER1D_SLOTS; i++) {
			u32 phys;
			tiler_blk_handle slot =
				tiler_alloc_block_area(TILFMT_PAGE,
					TILER1D_SLOT_SIZE, 1, &phys, NULL);
			if (IS_ERR_OR_NULL(slot))
				break;
			slots[i].slot = slot;
			slots[i].phys = phys;
			list_add(&slots[i].q, &free_slots);
			up(&free_slots_sem);
		}
		/* reset free_slots if no TILER memory could be reserved */
		if (!i)
			ZERO(free_slots);
	}
}

void dsscomp_gralloc_exit(void)
{
	struct tiler1d_slot *slot;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend_info);
#endif

	list_for_each_entry(slot, &free_slots, q)
		tiler_free_block_area(slot->slot);
	INIT_LIST_HEAD(&free_slots);
}
