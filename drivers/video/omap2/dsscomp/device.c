/*
 * linux/drivers/video/omap2/dsscomp/device.c
 *
 * DSS Composition file device and ioctl support
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

#define DEBUG

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/syscalls.h>

#define MODULE_NAME	"dsscomp"

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"

#include <linux/debugfs.h>

static DECLARE_WAIT_QUEUE_HEAD(waitq);
static DEFINE_MUTEX(wait_mtx);

static u32 hwc_virt_to_phys(u32 arg)
{
	pmd_t *pmd;
	pte_t *ptep;

	pgd_t *pgd = pgd_offset(current->mm, arg);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pmd = pmd_offset(pgd, arg);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	ptep = pte_offset_map(pmd, arg);
	if (ptep && pte_present(*ptep))
		return (PAGE_MASK & *ptep) | (~PAGE_MASK & arg);

	return 0;
}

/*
 * ===========================================================================
 *		WAIT OPERATIONS
 * ===========================================================================
 */

static void sync_drop(struct dsscomp_sync_obj *sync)
{
	if (sync && atomic_dec_and_test(&sync->refs)) {
		if (debug & DEBUG_WAITS)
			pr_info("free sync [%p]\n", sync);

		kfree(sync);
	}
}

static int sync_setup(const char *name, const struct file_operations *fops,
				struct dsscomp_sync_obj *sync, int flags)
{
	if (!sync)
		return -ENOMEM;

	sync->refs.counter = 1;
	sync->fd = anon_inode_getfd(name, fops, sync, flags);
	return sync->fd < 0 ? sync->fd : 0;
}

static int sync_finalize(struct dsscomp_sync_obj *sync, int r)
{
	if (sync) {
		if (r < 0)
			/* delete sync object on failure */
			sys_close(sync->fd);
		else
			/* return file descriptor on success */
			r = sync->fd;
	}
	return r;
}

/* wait for programming or release of a composition */
int dsscomp_wait(struct dsscomp_sync_obj *sync, enum dsscomp_wait_phase phase,
								int timeout)
{
	mutex_lock(&wait_mtx);
	if (debug & DEBUG_WAITS)
		pr_info("wait %s on [%p]\n",
			phase == DSSCOMP_WAIT_DISPLAYED ? "display" :
			phase == DSSCOMP_WAIT_PROGRAMMED ? "program" :
			"release", sync);

	if (sync->state < phase) {
		mutex_unlock(&wait_mtx);

		timeout = wait_event_interruptible_timeout(waitq,
			sync->state >= phase, timeout);
		if (debug & DEBUG_WAITS)
			pr_info("wait over [%p]: %s %d\n", sync,
				 timeout < 0 ? "signal" :
				 timeout > 0 ? "ok" : "timeout",
				 timeout);
		if (timeout <= 0)
			return timeout ? : -ETIME;

		mutex_lock(&wait_mtx);
	}
	mutex_unlock(&wait_mtx);

	return 0;
}
EXPORT_SYMBOL(dsscomp_wait);

static void dsscomp_queue_cb(void *data, int status)
{
	struct dsscomp_sync_obj *sync = data;
	enum dsscomp_wait_phase phase =
		status == DSS_COMPLETION_PROGRAMMED ? DSSCOMP_WAIT_PROGRAMMED :
		status == DSS_COMPLETION_DISPLAYED ? DSSCOMP_WAIT_DISPLAYED :
		DSSCOMP_WAIT_RELEASED, old_phase;

	mutex_lock(&wait_mtx);
	old_phase = sync->state;
	if (old_phase < phase)
		sync->state = phase;
	mutex_unlock(&wait_mtx);

	if (status & DSS_COMPLETION_RELEASED)
		sync_drop(sync);
	if (old_phase < phase)
		wake_up_interruptible_sync(&waitq);
}

static int sync_release(struct inode *inode, struct file *filp)
{
	struct dsscomp_sync_obj *sync = filp->private_data;
	sync_drop(sync);
	return 0;
}

static long sync_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int r = 0;
	struct dsscomp_sync_obj *sync = filp->private_data;
	void __user *ptr = (void __user *)arg;

	switch (cmd) {
	case DSSCIOC_WAIT:
	{
		struct dsscomp_wait_data wd;
		r = copy_from_user(&wd, ptr, sizeof(wd)) ? :
		    dsscomp_wait(sync, wd.phase,
					usecs_to_jiffies(wd.timeout_us));
		break;
	}
	default:
		r = -EINVAL;
	}
	return r;
}

static const struct file_operations sync_fops = {
	.owner		= THIS_MODULE,
	.release	= sync_release,
	.unlocked_ioctl = sync_ioctl,
};

static long setup_mgr(struct dsscomp_dev *cdev,
					struct dsscomp_setup_mgr_data *d)
{
	int i, r;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	dsscomp_t comp;
	struct dsscomp_sync_obj *sync = NULL;

	dump_comp_info(cdev, d, "queue");
	for (i = 0; i < d->num_ovls; i++)
		dump_ovl_info(cdev, d->ovls + i);

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

	/* swap red & blue if requested */
	if (d->mgr.swap_rb) {
		swap_rb_in_mgr_info(&d->mgr);
		for (i = 0; i < d->num_ovls; i++)
			swap_rb_in_ovl_info(d->ovls + i);
	}

	r = dsscomp_set_mgr(comp, &d->mgr);

	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 addr = (u32) oi->address;

		/* convert addresses to user space */
		if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
			oi->uv = hwc_virt_to_phys(addr +
					oi->cfg.height * oi->cfg.stride);
		oi->ba = hwc_virt_to_phys(addr);

		r = r ? : dsscomp_set_ovl(comp, oi);
	}

	r = r ? : dsscomp_setup(comp, d->mode, d->win);

	/* create sync object */
	if (d->get_sync_obj) {
		sync = kzalloc(sizeof(*sync), GFP_KERNEL);
		r = sync_setup("dsscomp_sync", &sync_fops, sync, O_RDONLY);
		if (sync && (debug & DEBUG_WAITS))
			dev_info(DEV(cdev), "new sync [%p] on #%d\n", sync,
								sync->fd);
		if (r)
			sync_drop(sync);
	}

	/* drop composition if failed to create */
	if (r) {
		dsscomp_drop(comp);
		return r;
	}

	if (sync) {
		sync->refs.counter++;
		comp->extra_cb = dsscomp_queue_cb;
		comp->extra_cb_data = sync;
	}
	if (d->mode & DSSCOMP_SETUP_APPLY)
		r = dsscomp_delayed_apply(comp);

	/* delete sync object if failed to apply or create file */
	if (sync) {
		r = sync_finalize(sync, r);
		if (r < 0)
			sync_drop(sync);
	}
	return r;
}

static long query_display(struct dsscomp_dev *cdev,
					struct dsscomp_display_info *dis)
{
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	int i;

	/* get display */
	if (dis->ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[dis->ix];
	if (!dev)
		return -EINVAL;
	mgr = dev->manager;

	/* fill out display information */
	dis->channel = dev->channel;
	dis->enabled = (dev->state == OMAP_DSS_DISPLAY_SUSPENDED) ?
		dev->activate_after_resume :
		(dev->state == OMAP_DSS_DISPLAY_ACTIVE);
	dis->overlays_available = 0;
	dis->overlays_owned = 0;
#if 0
	dis->s3d_info = dev->panel.s3d_info;
#endif
	dis->state = dev->state;
	dis->timings = dev->panel.timings;

	dis->width_in_mm = DIV_ROUND_CLOSEST(dev->panel.width_in_um, 1000);
	dis->height_in_mm = DIV_ROUND_CLOSEST(dev->panel.height_in_um, 1000);

	/* find all overlays available for/owned by this display */
	for (i = 0; i < cdev->num_ovls && dis->enabled; i++) {
		if (cdev->ovls[i]->manager == mgr)
			dis->overlays_owned |= 1 << i;
		else if (!cdev->ovls[i]->info.enabled)
			dis->overlays_available |= 1 << i;
	}
	dis->overlays_available |= dis->overlays_owned;

	/* fill out manager information */
	if (mgr) {
		dis->mgr.alpha_blending = mgr->info.alpha_enabled;
		dis->mgr.default_color = mgr->info.default_color;
#if 0
		dis->mgr.interlaced = !strcmp(dev->name, "hdmi") &&
							is_hdmi_interlaced()
#else
		dis->mgr.interlaced =  0;
#endif
		dis->mgr.trans_enabled = mgr->info.trans_enabled;
		dis->mgr.trans_key = mgr->info.trans_key;
		dis->mgr.trans_key_type = mgr->info.trans_key_type;
	} else {
		/* display is disabled if it has no manager */
		memset(&dis->mgr, 0, sizeof(dis->mgr));
	}
	dis->mgr.ix = dis->ix;

	if (dis->modedb_len && dev->driver->get_modedb)
		dis->modedb_len = dev->driver->get_modedb(dev,
			(struct fb_videomode *) dis->modedb, dis->modedb_len);
	return 0;
}

static long check_ovl(struct dsscomp_dev *cdev,
					struct dsscomp_check_ovl_data *chk)
{
	/* for now return all overlays as possible */
	return (1 << cdev->num_ovls) - 1;
}

static long setup_display(struct dsscomp_dev *cdev,
				struct dsscomp_setup_display_data *dis)
{
	struct omap_dss_device *dev;

	/* get display */
	if (dis->ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[dis->ix];
	if (!dev)
		return -EINVAL;

	if (dev->driver->set_mode)
		return dev->driver->set_mode(dev,
					(struct fb_videomode *) &dis->mode);
	else
		return 0;
}

static void fill_cache(struct dsscomp_dev *cdev)
{
	unsigned long i;
	struct omap_dss_device *dssdev = NULL;

	cdev->num_ovls = min(omap_dss_get_num_overlays(), MAX_OVERLAYS);
	for (i = 0; i < cdev->num_ovls; i++)
		cdev->ovls[i] = omap_dss_get_overlay(i);

	cdev->num_mgrs = min(omap_dss_get_num_overlay_managers(), MAX_MANAGERS);
	for (i = 0; i < cdev->num_mgrs; i++)
		cdev->mgrs[i] = omap_dss_get_overlay_manager(i);

	for_each_dss_dev(dssdev) {
		const char *name = dev_name(&dssdev->dev);
		if (strncmp(name, "display", 7) ||
		    strict_strtoul(name + 7, 10, &i) ||
		    i >= MAX_DISPLAYS)
			continue;

		if (cdev->num_displays <= i)
			cdev->num_displays = i + 1;

		cdev->displays[i] = dssdev;
		dev_dbg(DEV(cdev), "display%lu=%s\n", i, dssdev->driver_name);

		cdev->state_notifiers[i].notifier_call = dsscomp_state_notifier;
		blocking_notifier_chain_register(&dssdev->state_notifiers,
						cdev->state_notifiers + i);
	}
	dev_info(DEV(cdev), "found %d displays and %d overlays\n",
				cdev->num_displays, cdev->num_ovls);
}

static long comp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int r = 0;
	struct miscdevice *dev = filp->private_data;
	struct dsscomp_dev *cdev = container_of(dev, struct dsscomp_dev, dev);
	void __user *ptr = (void __user *)arg;

	union {
		struct {
			struct dsscomp_setup_mgr_data set;
			struct dss2_ovl_info ovl[MAX_OVERLAYS];
		} m;
		struct dsscomp_setup_dispc_data dispc;
		struct dsscomp_display_info dis;
		struct dsscomp_check_ovl_data chk;
		struct dsscomp_setup_display_data sdis;
	} u;

	dsscomp_gralloc_init(cdev);

	switch (cmd) {
	case DSSCIOC_SETUP_MGR:
	{
		r = copy_from_user(&u.m.set, ptr, sizeof(u.m.set)) ? :
		    u.m.set.num_ovls >= ARRAY_SIZE(u.m.ovl) ? -EINVAL :
		    copy_from_user(&u.m.ovl,
				(void __user *)arg + sizeof(u.m.set),
				sizeof(*u.m.ovl) * u.m.set.num_ovls) ? :
		    setup_mgr(cdev, &u.m.set);
		break;
	}
	case DSSCIOC_SETUP_DISPC:
	{
		r = copy_from_user(&u.dispc, ptr, sizeof(u.dispc)) ? :
		    dsscomp_gralloc_queue_ioctl(&u.dispc);
		break;
	}
	case DSSCIOC_QUERY_DISPLAY:
	{
		struct dsscomp_display_info *dis = NULL;
		r = copy_from_user(&u.dis, ptr, sizeof(u.dis));
		if (!r) {
			/* impose a safe limit on modedb_len to prevent
			 * wrap around/overflow calculation of the alloced
			 * size that would make it smaller than
			 * struct dsscomp_display_info and cause heap
			 * corruption.
			 */
			u.dis.modedb_len = clamp_val(u.dis.modedb_len, 0, 256);

			dis = kzalloc(sizeof(*dis->modedb) * u.dis.modedb_len +
						sizeof(*dis), GFP_KERNEL);
		}
		if (dis) {
			*dis = u.dis;
			r = query_display(cdev, dis) ? :
			    copy_to_user(ptr, dis, sizeof(*dis) +
				sizeof(*dis->modedb) * dis->modedb_len);
			kfree(dis);
		} else {
			r = r ? : -ENOMEM;
		}
		break;
	}
	case DSSCIOC_CHECK_OVL:
	{
		r = copy_from_user(&u.chk, ptr, sizeof(u.chk)) ? :
		    check_ovl(cdev, &u.chk);
		break;
	}
	case DSSCIOC_SETUP_DISPLAY:
	{
		r = copy_from_user(&u.sdis, ptr, sizeof(u.sdis)) ? :
		    setup_display(cdev, &u.sdis);
	}
	default:
		r = -EINVAL;
	}
	return r;
}

/* must implement open for filp->private_data to be filled */
static int comp_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations comp_fops = {
	.owner		= THIS_MODULE,
	.open		= comp_open,
	.unlocked_ioctl = comp_ioctl,
};

static int dsscomp_debug_show(struct seq_file *s, void *unused)
{
	void (*fn)(struct seq_file *s) = s->private;
	fn(s);
	return 0;
}

static int dsscomp_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dsscomp_debug_show, inode->i_private);
}

static const struct file_operations dsscomp_debug_fops = {
	.open           = dsscomp_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int dsscomp_probe(struct platform_device *pdev)
{
	int ret;
	struct dsscomp_dev *cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		pr_err("dsscomp: failed to allocate device.\n");
		return -ENOMEM;
	}
	cdev->dev.minor = MISC_DYNAMIC_MINOR;
	cdev->dev.name = "dsscomp";
	cdev->dev.mode = 0666;
	cdev->dev.fops = &comp_fops;

	ret = misc_register(&cdev->dev);
	if (ret) {
		pr_err("dsscomp: failed to register misc device.\n");
		return ret;
	}
	cdev->dbgfs = debugfs_create_dir("dsscomp", NULL);
	if (IS_ERR_OR_NULL(cdev->dbgfs))
		dev_warn(DEV(cdev), "failed to create debug files.\n");
	else {
		debugfs_create_file("comps", S_IRUGO,
			cdev->dbgfs, dsscomp_dbg_comps, &dsscomp_debug_fops);
		debugfs_create_file("gralloc", S_IRUGO,
			cdev->dbgfs, dsscomp_dbg_gralloc, &dsscomp_debug_fops);
#ifdef CONFIG_DSSCOMP_DEBUG_LOG
		debugfs_create_file("log", S_IRUGO,
			cdev->dbgfs, dsscomp_dbg_events, &dsscomp_debug_fops);
#endif
	}

	platform_set_drvdata(pdev, cdev);

	pr_info("dsscomp: initializing.\n");

	fill_cache(cdev);

	/* initialize queues */
	dsscomp_queue_init(cdev);
	dsscomp_gralloc_init(cdev);

	return 0;
}

static int dsscomp_remove(struct platform_device *pdev)
{
	struct dsscomp_dev *cdev = platform_get_drvdata(pdev);
	misc_deregister(&cdev->dev);
	debugfs_remove_recursive(cdev->dbgfs);
	dsscomp_queue_exit();
	dsscomp_gralloc_exit();
	kfree(cdev);

	return 0;
}

static struct platform_driver dsscomp_pdriver = {
	.probe = dsscomp_probe,
	.remove = dsscomp_remove,
	.driver = { .name = MODULE_NAME, .owner = THIS_MODULE }
};

static struct platform_device dsscomp_pdev = {
	.name = MODULE_NAME,
	.id = -1
};

static int __init dsscomp_init(void)
{
	int err = platform_driver_register(&dsscomp_pdriver);
	if (err)
		return err;

	err = platform_device_register(&dsscomp_pdev);
	if (err)
		platform_driver_unregister(&dsscomp_pdriver);
	return err;
}

static void __exit dsscomp_exit(void)
{
	platform_device_unregister(&dsscomp_pdev);
	platform_driver_unregister(&dsscomp_pdriver);
}

#define DUMP_CHUNK 256
static char dump_buf[64 * 1024];
void dsscomp_kdump(void)
{
	struct seq_file s = {
		.buf = dump_buf,
		.size = sizeof(dump_buf) - 1,
	};
	int i;

	dsscomp_dbg_events(&s);
	dsscomp_dbg_comps(&s);
	dsscomp_dbg_gralloc(&s);

	for (i = 0; i < s.count; i += DUMP_CHUNK) {
		if ((s.count - i) > DUMP_CHUNK) {
			char c = s.buf[i + DUMP_CHUNK];
			s.buf[i + DUMP_CHUNK] = 0;
			pr_cont("%s", s.buf + i);
			s.buf[i + DUMP_CHUNK] = c;
		} else {
			s.buf[s.count] = 0;
			pr_cont("%s", s.buf + i);
		}
	}
}
EXPORT_SYMBOL(dsscomp_kdump);

MODULE_LICENSE("GPL v2");
module_init(dsscomp_init);
module_exit(dsscomp_exit);
