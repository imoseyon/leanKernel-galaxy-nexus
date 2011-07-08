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
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#define MODULE_NAME	"dsscomp"

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"

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

static long setup_mgr(struct dsscomp_dev *cdev,
					struct dsscomp_setup_mgr_data *d)
{
	int i, r;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	dsscomp_t comp;

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
	if (r)
		dsscomp_drop(comp);
	else if (d->mode & DSSCOMP_SETUP_APPLY)
		r = dsscomp_apply(comp);

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

	return 0;
}

static long check_ovl(struct dsscomp_dev *cdev,
					struct dsscomp_check_ovl_data *chk)
{
	/* for now return all overlays as possible */
	return (1 << cdev->num_ovls) - 1;
}

static long wait(struct dsscomp_dev *cdev, struct dsscomp_wait_data *wd)
{
	struct omap_overlay_manager *mgr;
	dsscomp_t comp;

	/* get manager */
	if (wd->ix >= cdev->num_displays || !cdev->displays[wd->ix])
		return -EINVAL;
	mgr = cdev->displays[wd->ix]->manager;
	if (!mgr)
		return -ENODEV;

	/* get composition - we don't have a handle to the composition */
	comp = ERR_PTR(-EINVAL);
	if (IS_ERR(comp))
		return 0;

	return dsscomp_wait(comp, wd->phase, usecs_to_jiffies(wd->timeout_us));
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

	struct {
		struct dsscomp_setup_mgr_data set;
		struct dss2_ovl_info ovl[MAX_OVERLAYS];
	} p;

	dsscomp_gralloc_init(cdev);

	switch (cmd) {
	case DSSCOMP_SETUP_MGR:
	{
		r = copy_from_user(&p.set, ptr, sizeof(p.set)) ? :
		    p.set.num_ovls >= ARRAY_SIZE(p.ovl) ? -EINVAL :
		    copy_from_user(&p.ovl, (void __user *)arg + sizeof(p.set),
					sizeof(*p.ovl) * p.set.num_ovls) ? :
		    setup_mgr(cdev, &p.set);
		break;
	}
	case DSSCOMP_SETUP_MGR_G:
	{
		r = copy_from_user(&p.set, ptr, sizeof(p.set)) ? :
		    p.set.num_ovls >= ARRAY_SIZE(p.ovl) ? -EINVAL :
		    copy_from_user(&p.ovl, (void __user *)arg + sizeof(p.set),
					sizeof(*p.ovl) * p.set.num_ovls) ? :
		    dsscomp_gralloc_queue_ioctl(&p.set);
		break;
	}
	case DSSCOMP_QUERY_DISPLAY:
	{
		struct dsscomp_display_info dis;
		r = copy_from_user(&dis, ptr, sizeof(dis)) ? :
		    query_display(cdev, &dis) ? :
		    copy_to_user(ptr, &dis, sizeof(dis));
		break;
	}
	case DSSCOMP_CHECK_OVL:
	{
		struct dsscomp_check_ovl_data chk;
		r = copy_from_user(&chk, ptr, sizeof(chk)) ? :
		    check_ovl(cdev, &chk);
		break;
	}
	case DSSCOMP_WAIT:
	{
		struct dsscomp_wait_data wd;
		r = copy_from_user(&wd, ptr, sizeof(wd)) ? :
		    wait(cdev, &wd);
		break;
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

MODULE_LICENSE("GPL v2");
module_init(dsscomp_init);
module_exit(dsscomp_exit);
