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

/* gralloc composition sync object */
struct dsscomp_gralloc_t {
	void (*cb_fn)(void *, int);
	void *cb_arg;
	struct list_head q;
	struct list_head slots;
	atomic_t refs;
};

/* queued gralloc compositions */
static LIST_HEAD(flip_queue);

static u32 ovl_use_mask[MAX_MANAGERS];

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

static void dsscomp_gralloc_cb(void *data, int status)
{
	struct dsscomp_gralloc_t *gsync = data, *gsync_;
	LIST_HEAD(done);

	mutex_lock(&mtx);
	if (status & DSS_COMPLETION_RELEASED) {
		if (atomic_dec_and_test(&gsync->refs))
			unpin_tiler_blocks(&gsync->slots);
	}

	/* get completed list items in order, if any */
	while (!list_empty(&flip_queue)) {
		gsync = list_first_entry(&flip_queue, typeof(*gsync), q);
		if (gsync->refs.counter)
			break;
		list_move_tail(&gsync->q, &done);
	}
	mutex_unlock(&mtx);

	/* call back for completed composition with mutex unlocked */
	list_for_each_entry_safe(gsync, gsync_, &done, q) {
		if (debug & DEBUG_GRALLOC_PHASES)
			dev_info(DEV(cdev), "[%p] completed flip\n", gsync);
		if (gsync->cb_fn)
			gsync->cb_fn(gsync->cb_arg, 1);
		kfree(gsync);
	}
}

/* This is just test code for now that does the setup + apply.
   It still uses userspace virtual addresses, but maps non
   TILER buffers into 1D */
int dsscomp_gralloc_queue_ioctl(struct dsscomp_setup_dispc_data *d)
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

int dsscomp_gralloc_queue(struct dsscomp_setup_dispc_data *d,
			struct tiler_pa_info **pas,
			void (*cb_fn)(void *, int), void *cb_arg)
{
	u32 i;
	int r = 0;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	dsscomp_t comp[MAX_MANAGERS];
	u32 ovl_new_use_mask[MAX_MANAGERS];
	u32 mgr_set_mask = 0;
	u32 ovl_set_mask = 0;

	u32 channels[ARRAY_SIZE(d->mgrs)], ch;
	int skip;
	struct dsscomp_gralloc_t *gsync;
	struct dss2_rect_t win = { .w = 0 };

	/* reserve tiler areas if not already done so */
	dsscomp_gralloc_init(cdev);

	dump_total_comp_info(cdev, d, "queue");
	for (i = 0; i < d->num_ovls; i++)
		dump_ovl_info(cdev, d->ovls + i);

	mutex_lock(&mtx);

	/* create sync object with 1 temporary ref */
	gsync = kzalloc(sizeof(*gsync), GFP_KERNEL);
	gsync->cb_arg = cb_arg;
	gsync->cb_fn = cb_fn;
	gsync->refs.counter = 1;
	INIT_LIST_HEAD(&gsync->slots);
	list_add_tail(&gsync->q, &flip_queue);
	if (debug & DEBUG_GRALLOC_PHASES)
		dev_info(DEV(cdev), "[%p] queuing flip\n", gsync);

	/* ignore frames while we are blanked */
	skip = blanked;
	if (skip && (debug & DEBUG_PHASES))
		dev_info(DEV(cdev), "[%p,%08x] ignored\n", gsync, d->sync_id);

	/* mark blank frame by NULL tiler pa pointer */
	if (!skip && pas == NULL)
		blanked = true;

	mutex_unlock(&mtx);

	d->num_mgrs = min(d->num_mgrs, (u16) ARRAY_SIZE(d->mgrs));
	d->num_ovls = min(d->num_ovls, (u16) ARRAY_SIZE(d->ovls));

	memset(comp, 0, sizeof(comp));
	memset(ovl_new_use_mask, 0, sizeof(ovl_new_use_mask));

	if (skip)
		goto skip_comp;

	d->mode = DSSCOMP_SETUP_DISPLAY;

	/* mark managers we are using */
	for (i = 0; i < d->num_mgrs; i++) {
		/* verify display is valid & connected, ignore if not */
		if (d->mgrs[i].ix >= cdev->num_displays)
			continue;
		dev = cdev->displays[d->mgrs[i].ix];
		if (!dev) {
			dev_warn(DEV(cdev), "failed to get display%d\n",
								d->mgrs[i].ix);
			continue;
		}
		mgr = dev->manager;
		if (!mgr) {
			dev_warn(DEV(cdev), "no manager for display%d\n",
								d->mgrs[i].ix);
			continue;
		}
		channels[i] = ch = mgr->id;
		mgr_set_mask |= 1 << ch;

		/* swap red & blue if requested */
		if (d->mgrs[i].swap_rb)
			swap_rb_in_mgr_info(d->mgrs + i);
	}

	/* create dsscomp objects for set managers (including active ones) */
	for (ch = 0; ch < MAX_MANAGERS; ch++) {
		if (!(mgr_set_mask & (1 << ch)) && !ovl_use_mask[ch])
			continue;

		mgr = cdev->mgrs[ch];

		comp[ch] = dsscomp_new(mgr);
		if (IS_ERR(comp[ch])) {
			comp[ch] = NULL;
			dev_warn(DEV(cdev), "failed to get composition on %s\n",
								mgr->name);
			continue;
		}

		/* set basic manager information for blanked managers */
		if (!(mgr_set_mask & (1 << ch))) {
			struct dss2_mgr_info mi = {
				.alpha_blending = true,
				.ix = comp[ch]->frm.mgr.ix,
			};
			dsscomp_set_mgr(comp[ch], &mi);
		}

		comp[ch]->must_apply = true;
		r = dsscomp_setup(comp[ch], d->mode, win);
		if (r)
			dev_err(DEV(cdev), "failed to setup comp (%d)\n", r);
	}

	/* configure manager data from gralloc composition */
	for (i = 0; i < d->num_mgrs; i++) {
		ch = channels[i];
		r = dsscomp_set_mgr(comp[ch], d->mgrs + i);
		if (r)
			dev_err(DEV(cdev), "failed to set mgr%d (%d)\n", ch, r);
	}

	/* NOTE: none of the dsscomp sets should fail as composition is new */
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 mgr_ix = oi->cfg.mgr_ix;

		/* verify manager index */
		if (mgr_ix >= d->num_mgrs) {
			dev_err(DEV(cdev), "invalid manager for ovl%d\n",
								oi->cfg.ix);
			continue;
		}
		ch = channels[mgr_ix];

		/* skip overlays on compositions we could not create */
		if (!comp[ch])
			continue;

		/* swap red & blue if requested */
		if (d->mgrs[mgr_ix].swap_rb)
			swap_rb_in_ovl_info(d->ovls + i);

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
				oi->cfg.enabled = false;
				break;
			}
			list_move(&slot->q, &gsync->slots);
			oi->ba = slot->phys + (oi->ba & ~PAGE_MASK);
			mutex_unlock(&mtx);
		}

		if (oi->cfg.enabled)
			ovl_new_use_mask[ch] |= 1 << oi->cfg.ix;

		r = dsscomp_set_ovl(comp[ch], oi);
		if (r)
			dev_err(DEV(cdev), "failed to set ovl%d (%d)\n",
								oi->cfg.ix, r);
		else
			ovl_set_mask |= 1 << oi->cfg.ix;
	}

	for (ch = 0; ch < MAX_MANAGERS; ch++) {
		/* disable all overlays not specifically set from prior frame */
		u32 mask = ovl_use_mask[ch] & ~ovl_set_mask;

		if (!comp[ch])
			continue;

		while (mask) {
			struct dss2_ovl_info oi = {
				.cfg.zonly = true,
				.cfg.enabled = false,
				.cfg.ix = fls(mask) - 1,
			};
			dsscomp_set_ovl(comp[ch], &oi);
			mask &= ~(1 << oi.cfg.ix);
		}

		/* associate dsscomp objects with this gralloc composition */
		comp[ch]->extra_cb = dsscomp_gralloc_cb;
		comp[ch]->extra_cb_data = gsync;
		atomic_inc(&gsync->refs);
		r = dsscomp_delayed_apply(comp[ch]);
		if (r)
			dev_err(DEV(cdev), "failed to apply comp (%d)\n", r);
		else
			ovl_use_mask[ch] = ovl_new_use_mask[ch];
	}
skip_comp:
	/* release sync object ref - this completes unapplied compositions */
	dsscomp_gralloc_cb(gsync, DSS_COMPLETION_RELEASED);

	return r;
}

#ifdef CONFIG_EARLYSUSPEND
static int blank_complete;
static DECLARE_WAIT_QUEUE_HEAD(early_suspend_wq);

static void dsscomp_early_suspend_cb(void *data, int status)
{
	blank_complete = true;
	wake_up(&early_suspend_wq);
}

static void dsscomp_early_suspend(struct early_suspend *h)
{
	struct dsscomp_setup_dispc_data d = {
		.num_mgrs = 0,
	};
	int err;

	pr_info("DSSCOMP: %s\n", __func__);

	/* use gralloc queue as we need to blank all screens */
	blank_complete = false;
	dsscomp_gralloc_queue(&d, NULL, dsscomp_early_suspend_cb, NULL);

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
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
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
