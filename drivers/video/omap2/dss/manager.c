/*
 * linux/drivers/video/omap2/dss/manager.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#define DSS_SUBSYS_NAME "MANAGER"

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/ratelimit.h>
#include <linux/seq_file.h>

#include <video/omapdss.h>
#include <plat/cpu.h>

#include "dss.h"
#include "dss_features.h"

static int num_managers;
static struct list_head manager_list;
static struct omap_overlay_manager *mgrs[MAX_DSS_MANAGERS];

static ssize_t manager_name_show(struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", mgr->name);
}

static ssize_t manager_display_show(struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
			mgr->device ? mgr->device->name : "<none>");
}

static ssize_t manager_display_store(struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	int r = 0;
	size_t len = size;
	struct omap_dss_device *dssdev = NULL;

	int match(struct omap_dss_device *dssdev, void *data)
	{
		const char *str = data;
		return sysfs_streq(dssdev->name, str);
	}

	if (buf[size-1] == '\n')
		--len;

	if (len > 0)
		dssdev = omap_dss_find_device((void *)buf, match);

	if (len > 0 && dssdev == NULL)
		return -EINVAL;

	if (dssdev)
		DSSDBG("display %s found\n", dssdev->name);

	if (mgr->device) {
		r = mgr->unset_device(mgr);
		if (r) {
			DSSERR("failed to unset display\n");
			goto put_device;
		}
	}

	if (dssdev) {
		r = mgr->set_device(mgr, dssdev);
		if (r) {
			DSSERR("failed to set manager\n");
			goto put_device;
		}

		r = mgr->apply(mgr);
		if (r) {
			DSSERR("failed to apply dispc config\n");
			goto put_device;
		}
	}

put_device:
	if (dssdev)
		omap_dss_put_device(dssdev);

	return r ? r : size;
}

static ssize_t manager_default_color_show(struct omap_overlay_manager *mgr,
					  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.default_color);
}

static ssize_t manager_default_color_store(struct omap_overlay_manager *mgr,
					   const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	u32 color;
	int r;

	if (sscanf(buf, "%d", &color) != 1)
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.default_color = color;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static const char *trans_key_type_str[] = {
	"gfx-destination",
	"video-source",
};

static ssize_t manager_trans_key_type_show(struct omap_overlay_manager *mgr,
					   char *buf)
{
	enum omap_dss_trans_key_type key_type;

	key_type = mgr->info.trans_key_type;
	BUG_ON(key_type >= ARRAY_SIZE(trans_key_type_str));

	return snprintf(buf, PAGE_SIZE, "%s\n", trans_key_type_str[key_type]);
}

static ssize_t manager_trans_key_type_store(struct omap_overlay_manager *mgr,
					    const char *buf, size_t size)
{
	enum omap_dss_trans_key_type key_type;
	struct omap_overlay_manager_info info;
	int r;

	for (key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
			key_type < ARRAY_SIZE(trans_key_type_str); key_type++) {
		if (sysfs_streq(buf, trans_key_type_str[key_type]))
			break;
	}

	if (key_type == ARRAY_SIZE(trans_key_type_str))
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.trans_key_type = key_type;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static ssize_t manager_trans_key_value_show(struct omap_overlay_manager *mgr,
					    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.trans_key);
}

static ssize_t manager_trans_key_value_store(struct omap_overlay_manager *mgr,
					     const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	u32 key_value;
	int r;

	if (sscanf(buf, "%d", &key_value) != 1)
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.trans_key = key_value;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static ssize_t manager_trans_key_enabled_show(struct omap_overlay_manager *mgr,
					      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.trans_enabled);
}

static ssize_t manager_trans_key_enabled_store(struct omap_overlay_manager *mgr,
					       const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	int enable;
	int r;

	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.trans_enabled = enable ? true : false;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static ssize_t manager_alpha_blending_enabled_show(
		struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.alpha_enabled);
}

static ssize_t manager_alpha_blending_enabled_store(
		struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	int enable;
	int r;

	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;

	/* if we have OMAP3 alpha compatibility, alpha blending is always on */
	if (dss_has_feature(FEAT_ALPHA_OMAP3_COMPAT) && !enable)
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.alpha_enabled = enable ? true : false;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static ssize_t manager_cpr_enable_show(struct omap_overlay_manager *mgr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.cpr_enable);
}

static ssize_t manager_cpr_enable_store(struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	int v;
	int r;
	bool enable;

	if (!dss_has_feature(FEAT_CPR))
		return -ENODEV;

	r = kstrtoint(buf, 0, &v);
	if (r)
		return r;

	enable = !!v;

	mgr->get_manager_info(mgr, &info);

	if (info.cpr_enable == enable)
		return size;

	info.cpr_enable = enable;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

static ssize_t manager_cpr_coef_show(struct omap_overlay_manager *mgr,
		char *buf)
{
	struct omap_overlay_manager_info info;

	mgr->get_manager_info(mgr, &info);

	return snprintf(buf, PAGE_SIZE,
			"%d %d %d %d %d %d %d %d %d\n",
			info.cpr_coefs.rr,
			info.cpr_coefs.rg,
			info.cpr_coefs.rb,
			info.cpr_coefs.gr,
			info.cpr_coefs.gg,
			info.cpr_coefs.gb,
			info.cpr_coefs.br,
			info.cpr_coefs.bg,
			info.cpr_coefs.bb);
}

static ssize_t manager_cpr_coef_store(struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	struct omap_dss_cpr_coefs coefs;
	int r, i;
	s16 *arr;

	if (!dss_has_feature(FEAT_CPR))
		return -ENODEV;

	if (sscanf(buf, "%hd %hd %hd %hd %hd %hd %hd %hd %hd",
				&coefs.rr, &coefs.rg, &coefs.rb,
				&coefs.gr, &coefs.gg, &coefs.gb,
				&coefs.br, &coefs.bg, &coefs.bb) != 9)
		return -EINVAL;

	arr = (s16[]){ coefs.rr, coefs.rg, coefs.rb,
		coefs.gr, coefs.gg, coefs.gb,
		coefs.br, coefs.bg, coefs.bb };

	for (i = 0; i < 9; ++i) {
		if (arr[i] < -512 || arr[i] > 511)
			return -EINVAL;
	}

	mgr->get_manager_info(mgr, &info);

	info.cpr_coefs = coefs;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}
static ssize_t manager_gamma_show(
		struct omap_overlay_manager *mgr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mgr->info.gamma);
}

static ssize_t manager_gamma_store(
		struct omap_overlay_manager *mgr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager_info info;
	int gamma_value;
	int r;

	if (sscanf(buf, "%d", &gamma_value) != 1)
		return -EINVAL;

	mgr->get_manager_info(mgr, &info);

	info.gamma = gamma_value;

	r = mgr->set_manager_info(mgr, &info);
	if (r)
		return r;

	r = mgr->apply(mgr);
	if (r)
		return r;

	return size;
}

struct manager_attribute {
	struct attribute attr;
	ssize_t (*show)(struct omap_overlay_manager *, char *);
	ssize_t	(*store)(struct omap_overlay_manager *, const char *, size_t);
};

#define MANAGER_ATTR(_name, _mode, _show, _store) \
	struct manager_attribute manager_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static MANAGER_ATTR(name, S_IRUGO, manager_name_show, NULL);
static MANAGER_ATTR(display, S_IRUGO|S_IWUSR,
		manager_display_show, manager_display_store);
static MANAGER_ATTR(default_color, S_IRUGO|S_IWUSR,
		manager_default_color_show, manager_default_color_store);
static MANAGER_ATTR(trans_key_type, S_IRUGO|S_IWUSR,
		manager_trans_key_type_show, manager_trans_key_type_store);
static MANAGER_ATTR(trans_key_value, S_IRUGO|S_IWUSR,
		manager_trans_key_value_show, manager_trans_key_value_store);
static MANAGER_ATTR(trans_key_enabled, S_IRUGO|S_IWUSR,
		manager_trans_key_enabled_show,
		manager_trans_key_enabled_store);
static MANAGER_ATTR(alpha_blending_enabled, S_IRUGO|S_IWUSR,
		manager_alpha_blending_enabled_show,
		manager_alpha_blending_enabled_store);
static MANAGER_ATTR(cpr_enable, S_IRUGO|S_IWUSR,
		manager_cpr_enable_show,
		manager_cpr_enable_store);
static MANAGER_ATTR(cpr_coef, S_IRUGO|S_IWUSR,
		manager_cpr_coef_show,
		manager_cpr_coef_store);

static MANAGER_ATTR(gamma, S_IRUGO|S_IWUSR,
			manager_gamma_show,
			manager_gamma_store);

static struct attribute *manager_sysfs_attrs[] = {
	&manager_attr_name.attr,
	&manager_attr_display.attr,
	&manager_attr_default_color.attr,
	&manager_attr_trans_key_type.attr,
	&manager_attr_trans_key_value.attr,
	&manager_attr_trans_key_enabled.attr,
	&manager_attr_alpha_blending_enabled.attr,
	&manager_attr_cpr_enable.attr,
	&manager_attr_cpr_coef.attr,
	&manager_attr_gamma.attr,
	NULL
};

static ssize_t manager_attr_show(struct kobject *kobj, struct attribute *attr,
		char *buf)
{
	struct omap_overlay_manager *manager;
	struct manager_attribute *manager_attr;

	manager = container_of(kobj, struct omap_overlay_manager, kobj);
	manager_attr = container_of(attr, struct manager_attribute, attr);

	if (!manager_attr->show)
		return -ENOENT;

	return manager_attr->show(manager, buf);
}

static ssize_t manager_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t size)
{
	struct omap_overlay_manager *manager;
	struct manager_attribute *manager_attr;

	manager = container_of(kobj, struct omap_overlay_manager, kobj);
	manager_attr = container_of(attr, struct manager_attribute, attr);

	if (!manager_attr->store)
		return -ENOENT;

	return manager_attr->store(manager, buf, size);
}

static const struct sysfs_ops manager_sysfs_ops = {
	.show = manager_attr_show,
	.store = manager_attr_store,
};

static struct kobj_type manager_ktype = {
	.sysfs_ops = &manager_sysfs_ops,
	.default_attrs = manager_sysfs_attrs,
};

struct callback_states {
	/*
	 * Keep track of callbacks at the last 3 levels of pipeline:
	 * cache, shadow registers and in DISPC registers.
	 *
	 * Note: We zero the function pointer when moving from one level to
	 * another to avoid checking for dirty and shadow_dirty fields that
	 * are not common between overlay and manager cache structures.
	 */
	struct omapdss_ovl_cb cache, shadow, dispc;
	bool dispc_displayed;
	bool shadow_enabled;
};

/*
 * We have 4 levels of cache for the dispc settings. First two are in SW and
 * the latter two in HW.
 *
 * +--------------------+
 * |overlay/manager_info|
 * +--------------------+
 *          v
 *        apply()
 *          v
 * +--------------------+
 * |     dss_cache      |
 * +--------------------+
 *          v
 *      configure()
 *          v
 * +--------------------+
 * |  shadow registers  |
 * +--------------------+
 *          v
 * VFP or lcd/digit_enable
 *          v
 * +--------------------+
 * |      registers     |
 * +--------------------+
 */

struct overlay_cache_data {
	/* If true, cache changed, but not written to shadow registers. Set
	 * in apply(), cleared when registers written. */
	bool dirty;
	/* If true, shadow registers contain changed values not yet in real
	 * registers. Set when writing to shadow registers, cleared at
	 * VSYNC/EVSYNC */
	bool shadow_dirty;

	bool enabled;

	u32 paddr;
	void __iomem *vaddr;
	u32 p_uv_addr; /* relevant for NV12 format only */
	u16 screen_width;
	u16 width;
	u16 height;
	enum omap_color_mode color_mode;
	u8 rotation;
	enum omap_dss_rotation_type rotation_type;
	bool mirror;

	u16 pos_x;
	u16 pos_y;
	u16 out_width;	/* if 0, out_width == width */
	u16 out_height;	/* if 0, out_height == height */
	u8 global_alpha;
	u8 pre_mult_alpha;

	struct callback_states cb; /* callback data for the last 3 states */
	int dispc_channel; /* overlay's channel in DISPC */

	enum omap_channel channel;
	bool replication;
	bool ilace;
	u16 min_x_decim, max_x_decim, min_y_decim, max_y_decim;

	enum omap_burst_size burst_size;
	u32 fifo_low;
	u32 fifo_high;

	bool manual_update;
	enum omap_overlay_zorder zorder;
	struct omap_dss_cconv_coefs cconv;
};

struct manager_cache_data {
	/* If true, cache changed, but not written to shadow registers. Set
	 * in apply(), cleared when registers written. */
	bool dirty;
	/* If true, shadow registers contain changed values not yet in real
	 * registers. Set when writing to shadow registers, cleared at
	 * VSYNC/EVSYNC */
	bool shadow_dirty;

	u32 default_color;

	enum omap_dss_trans_key_type trans_key_type;
	u32 trans_key;
	u8 gamma;
	bool trans_enabled;

	bool alpha_enabled;

	bool manual_upd_display;
	bool manual_update;
	bool do_manual_update;

	/* manual update region */
	u16 x, y, w, h;

	/* enlarge the update area if the update area contains scaled
	 * overlays */
	bool enlarge_update_area;

	struct callback_states cb; /* callback data for the last 3 states */

	bool cpr_enable;
	struct omap_dss_cpr_coefs cpr_coefs;
	bool skip_init;
};

static struct {
	spinlock_t lock;
	struct overlay_cache_data overlay_cache[MAX_DSS_OVERLAYS];
	struct manager_cache_data manager_cache[MAX_DSS_MANAGERS];

	bool irq_enabled;
	u32 comp_irq_enabled;
} dss_cache;

/* propagating callback info between states */
static inline void
dss_ovl_configure_cb(struct callback_states *st, int i, bool enabled)
{
	/* complete info in shadow */
	dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_ECLIPSED_SHADOW);

	/* propagate cache to shadow */
	st->shadow = st->cache;
	st->shadow_enabled = enabled;
	st->cache.fn = NULL;	/* info traveled to shadow */
}

static inline void
dss_ovl_program_cb(struct callback_states *st, int i)
{
	/* mark previous programming as completed */
	dss_ovl_cb(&st->dispc, i, st->dispc_displayed ?
				DSS_COMPLETION_RELEASED : DSS_COMPLETION_TORN);

	/* mark shadow info as programmed, not yet displayed */
	dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_PROGRAMMED);

	/* if overlay/manager is not enabled, we are done now */
	if (!st->shadow_enabled) {
		dss_ovl_cb(&st->shadow, i, DSS_COMPLETION_RELEASED);
		st->shadow.fn = NULL;
	}

	/* propagate shadow to dispc */
	st->dispc = st->shadow;
	st->shadow.fn = NULL;
	st->dispc_displayed = false;
}

static int omap_dss_set_device(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev)
{
	int i;
	int r;

	if (dssdev->manager) {
		DSSERR("display '%s' already has a manager '%s'\n",
			       dssdev->name, dssdev->manager->name);
		return -EINVAL;
	}

	if ((mgr->supported_displays & dssdev->type) == 0) {
		DSSERR("display '%s' does not support manager '%s'\n",
			       dssdev->name, mgr->name);
		return -EINVAL;
	}

	for (i = 0; i < mgr->num_overlays; i++) {
		struct omap_overlay *ovl = mgr->overlays[i];

		if (ovl->manager != mgr || !ovl->info.enabled)
			continue;

		r = dss_check_overlay(ovl, dssdev);
		if (r)
			return r;
	}

	dssdev->manager = mgr;
	mgr->device = dssdev;
	mgr->device_changed = true;

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI &&
	    !(dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE))
		omap_dispc_set_irq_type(mgr->id, OMAP_DISPC_IRQ_TYPE_VSYNC);
	else
		omap_dispc_set_irq_type(mgr->id, OMAP_DISPC_IRQ_TYPE_FRAMEDONE);

	return 0;
}

static int omap_dss_unset_device(struct omap_overlay_manager *mgr)
{
	if (!mgr->device) {
		DSSERR("failed to unset display, display not set.\n");
		return -EINVAL;
	}

	mgr->device->manager = NULL;
	mgr->device = NULL;
	mgr->device_changed = true;

	return 0;
}

static int dss_mgr_wait_for_vsync(struct omap_overlay_manager *mgr)
{
	unsigned long timeout = msecs_to_jiffies(500);
	u32 irq;
	int r;

	if (mgr->device->type == OMAP_DISPLAY_TYPE_VENC) {
		irq = DISPC_IRQ_EVSYNC_ODD;
	} else if (mgr->device->type == OMAP_DISPLAY_TYPE_HDMI) {
		irq = DISPC_IRQ_EVSYNC_EVEN;
	} else {
		if (mgr->id == OMAP_DSS_CHANNEL_LCD)
			irq = DISPC_IRQ_VSYNC;
		else
			irq = DISPC_IRQ_VSYNC2;
	}
	r = omap_dispc_wait_for_irq_interruptible_timeout(irq, timeout);
	if (!r)
		mgr->device->first_vsync = true;

	return r;
}

static int dss_mgr_wait_for_go(struct omap_overlay_manager *mgr)
{
	unsigned long timeout = msecs_to_jiffies(500);
	struct manager_cache_data *mc;
	u32 irq;
	int r;
	int i;
	struct omap_dss_device *dssdev = mgr->device;

	if (!dssdev || dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	if (dssdev->type == OMAP_DISPLAY_TYPE_VENC
			|| dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		irq = DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_EVSYNC_EVEN
						| DISPC_IRQ_FRAMEDONETV;
	} else {
		if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
			enum omap_dss_update_mode mode;
			mode = dssdev->driver->get_update_mode(dssdev);
			if (mode != OMAP_DSS_UPDATE_AUTO)
				return 0;

			irq = (dssdev->manager->id == OMAP_DSS_CHANNEL_LCD) ?
				DISPC_IRQ_FRAMEDONE
				: DISPC_IRQ_FRAMEDONE2;
		} else {
			irq = (dssdev->manager->id == OMAP_DSS_CHANNEL_LCD) ?
				DISPC_IRQ_VSYNC
				: DISPC_IRQ_VSYNC2;
		}
	}

	mc = &dss_cache.manager_cache[mgr->id];
	i = 0;
	while (1) {
		unsigned long flags;
		bool shadow_dirty, dirty;

		spin_lock_irqsave(&dss_cache.lock, flags);
		dirty = mc->dirty;
		shadow_dirty = mc->shadow_dirty;
		spin_unlock_irqrestore(&dss_cache.lock, flags);

		if (!dirty && !shadow_dirty) {
			r = 0;
			break;
		}

		/* 4 iterations is the worst case:
		 * 1 - initial iteration, dirty = true (between VFP and VSYNC)
		 * 2 - first VSYNC, dirty = true
		 * 3 - dirty = false, shadow_dirty = true
		 * 4 - shadow_dirty = false */
		if (i++ == 3) {
			DSSERR("mgr(%d)->wait_for_go() not finishing\n",
					mgr->id);
			r = 0;
			break;
		}

		r = omap_dispc_wait_for_irq_interruptible_timeout(irq, timeout);
		if (!r)
			mgr->device->first_vsync = true;
		if (r == -ERESTARTSYS)
			break;

		if (r) {
			DSSERR("mgr(%d)->wait_for_go() timeout\n", mgr->id);
			break;
		}
	}

	return r;
}

int dss_mgr_wait_for_go_ovl(struct omap_overlay *ovl)
{
	unsigned long timeout = msecs_to_jiffies(500);
	struct overlay_cache_data *oc;
	struct omap_dss_device *dssdev;
	u32 irq;
	int r;
	int i;

	if (!ovl->manager)
		return 0;

	dssdev = ovl->manager->device;

	if (!dssdev || dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	if (dssdev->type == OMAP_DISPLAY_TYPE_VENC
			|| dssdev->type == OMAP_DISPLAY_TYPE_HDMI) {
		irq = DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_EVSYNC_EVEN
						| DISPC_IRQ_FRAMEDONETV;
	} else {
		if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
			enum omap_dss_update_mode mode;
			mode = dssdev->driver->get_update_mode(dssdev);
			if (mode != OMAP_DSS_UPDATE_AUTO)
				return 0;

			irq = (dssdev->manager->id == OMAP_DSS_CHANNEL_LCD) ?
				DISPC_IRQ_FRAMEDONE
				: DISPC_IRQ_FRAMEDONE2;
		} else {
			irq = (dssdev->manager->id == OMAP_DSS_CHANNEL_LCD) ?
				DISPC_IRQ_VSYNC
				: DISPC_IRQ_VSYNC2;
		}
	}

	oc = &dss_cache.overlay_cache[ovl->id];
	i = 0;
	while (1) {
		unsigned long flags;
		bool shadow_dirty, dirty;

		spin_lock_irqsave(&dss_cache.lock, flags);
		dirty = oc->dirty;
		shadow_dirty = oc->shadow_dirty;
		spin_unlock_irqrestore(&dss_cache.lock, flags);

		if (!dirty && !shadow_dirty) {
			r = 0;
			break;
		}

		/* 4 iterations is the worst case:
		 * 1 - initial iteration, dirty = true (between VFP and VSYNC)
		 * 2 - first VSYNC, dirty = true
		 * 3 - dirty = false, shadow_dirty = true
		 * 4 - shadow_dirty = false */
		if (i++ == 3) {
			DSSERR("ovl(%d)->wait_for_go() not finishing\n",
					ovl->id);
			r = 0;
			break;
		}

		r = omap_dispc_wait_for_irq_interruptible_timeout(irq, timeout);
		if (r == -ERESTARTSYS)
			break;

		if (r) {
			DSSERR("ovl(%d)->wait_for_go() timeout\n", ovl->id);
			break;
		}
	}

	return r;
}

static int overlay_enabled(struct omap_overlay *ovl)
{
	return ovl->info.enabled && ovl->manager && ovl->manager->device;
}

/* Is rect1 a subset of rect2? */
static bool rectangle_subset(int x1, int y1, int w1, int h1,
		int x2, int y2, int w2, int h2)
{
	if (x1 < x2 || y1 < y2)
		return false;

	if (x1 + w1 > x2 + w2)
		return false;

	if (y1 + h1 > y2 + h2)
		return false;

	return true;
}

/* Do rect1 and rect2 overlap? */
static bool rectangle_intersects(int x1, int y1, int w1, int h1,
		int x2, int y2, int w2, int h2)
{
	if (x1 >= x2 + w2)
		return false;

	if (x2 >= x1 + w1)
		return false;

	if (y1 >= y2 + h2)
		return false;

	if (y2 >= y1 + h1)
		return false;

	return true;
}

static bool dispc_is_overlay_scaled(struct overlay_cache_data *oc)
{
	if (oc->out_width != 0 && oc->width != oc->out_width)
		return true;

	if (oc->out_height != 0 && oc->height != oc->out_height)
		return true;

	return false;
}

static int configure_overlay(enum omap_plane plane)
{
	struct overlay_cache_data *c;
	struct manager_cache_data *mc;
	u16 outw, outh;
	u16 x, y, w, h;
	u32 paddr;
	int r;
	u16 x_decim, y_decim;
	bool five_taps;
	u16 orig_w, orig_h, orig_outw, orig_outh;

	DSSDBGF("%d", plane);

	c = &dss_cache.overlay_cache[plane];

	if (!c->enabled) {
		dispc_enable_plane(plane, 0);
		return 0;
	}

	mc = &dss_cache.manager_cache[c->channel];

	x = c->pos_x;
	y = c->pos_y;
	w = c->width;
	h = c->height;
	outw = c->out_width == 0 ? c->width : c->out_width;
	outh = c->out_height == 0 ? c->height : c->out_height;
	paddr = c->paddr;

	orig_w = w;
	orig_h = h;
	orig_outw = outw;
	orig_outh = outh;

	if (c->manual_update && mc->do_manual_update) {
		unsigned bpp;
		unsigned scale_x_m = w, scale_x_d = outw;
		unsigned scale_y_m = h, scale_y_d = outh;

		/* If the overlay is outside the update region, disable it */
		if (!rectangle_intersects(mc->x, mc->y, mc->w, mc->h,
					x, y, outw, outh)) {
			dispc_enable_plane(plane, 0);
			return 0;
		}

		switch (c->color_mode) {
		case OMAP_DSS_COLOR_NV12:
			bpp = 8;
			break;

		case OMAP_DSS_COLOR_CLUT1:
		case OMAP_DSS_COLOR_CLUT2:
		case OMAP_DSS_COLOR_CLUT4:
		case OMAP_DSS_COLOR_CLUT8:
		case OMAP_DSS_COLOR_RGB16:
		case OMAP_DSS_COLOR_ARGB16:
		case OMAP_DSS_COLOR_YUV2:
		case OMAP_DSS_COLOR_UYVY:
		case OMAP_DSS_COLOR_RGBA16:
		case OMAP_DSS_COLOR_RGB12U:
		case OMAP_DSS_COLOR_RGBX16:
		case OMAP_DSS_COLOR_ARGB16_1555:
		case OMAP_DSS_COLOR_XRGB16_1555:
			bpp = 16;
			break;

		case OMAP_DSS_COLOR_RGB24P:
			bpp = 24;
			break;

		case OMAP_DSS_COLOR_RGB24U:
		case OMAP_DSS_COLOR_ARGB32:
		case OMAP_DSS_COLOR_RGBA32:
		case OMAP_DSS_COLOR_RGBX32:
			bpp = 32;
			break;

		default:
			BUG();
		}

		if (mc->x > c->pos_x) {
			x = 0;
			outw -= (mc->x - c->pos_x);
			paddr += (mc->x - c->pos_x) *
				scale_x_m / scale_x_d * bpp / 8;
		} else {
			x = c->pos_x - mc->x;
		}

		if (mc->y > c->pos_y) {
			y = 0;
			outh -= (mc->y - c->pos_y);
			paddr += (mc->y - c->pos_y) *
				scale_y_m / scale_y_d *
				c->screen_width * bpp / 8;
		} else {
			y = c->pos_y - mc->y;
		}

		if (mc->w < (x + outw))
			outw -= (x + outw) - (mc->w);

		if (mc->h < (y + outh))
			outh -= (y + outh) - (mc->h);

		w = w * outw / orig_outw;
		h = h * outh / orig_outh;

		/* YUV mode overlay's input width has to be even and the
		 * algorithm above may adjust the width to be odd.
		 *
		 * Here we adjust the width if needed, preferring to increase
		 * the width if the original width was bigger.
		 */
		if ((w & 1) &&
				(c->color_mode == OMAP_DSS_COLOR_YUV2 ||
				 c->color_mode == OMAP_DSS_COLOR_UYVY)) {
			if (orig_w > w)
				w += 1;
			else
				w -= 1;
		}
	}

	r = dispc_scaling_decision(w, h, outw, outh,
			       plane, c->color_mode, c->channel,
			       c->rotation, c->rotation_type,
			       c->min_x_decim, c->max_x_decim,
			       c->min_y_decim, c->max_y_decim,
			       &x_decim, &y_decim, &five_taps);
	r = r ? : dispc_setup_plane(plane,
			paddr,
			c->screen_width,
			x, y,
			w, h,
			outw, outh,
			c->color_mode,
			c->ilace, x_decim, y_decim, five_taps,
			c->rotation_type,
			c->rotation,
			c->mirror,
			c->global_alpha,
			c->pre_mult_alpha,
			c->channel,
			c->p_uv_addr);

	if (r) {
		/* this shouldn't happen */
		DSSERR("dispc_setup_plane failed for ovl %d\n", plane);
		dispc_enable_plane(plane, 0);
		return r;
	}

	dispc_enable_replication(plane, c->replication);

	dispc_set_burst_size(plane, c->burst_size);
	dispc_set_zorder(plane, c->zorder);
	dispc_enable_zorder(plane, 1);
	if (!cpu_is_omap44xx())
		dispc_setup_plane_fifo(plane, c->fifo_low, c->fifo_high);
	if (plane != OMAP_DSS_GFX)
		_dispc_setup_color_conv_coef(plane, &c->cconv);

	dispc_enable_plane(plane, 1);

	return 0;
}

static void configure_manager(enum omap_channel channel)
{
	struct manager_cache_data *c;

	DSSDBGF("%d", channel);

	c = &dss_cache.manager_cache[channel];

	dispc_set_default_color(channel, c->default_color);
	dispc_set_trans_key(channel, c->trans_key_type, c->trans_key);
	dispc_enable_trans_key(channel, c->trans_enabled);
	dispc_enable_gamma(channel, c->gamma);

	/* if we have OMAP3 alpha compatibility, alpha blending is always on */
	if (dss_has_feature(FEAT_ALPHA_OMAP3_COMPAT)) {
		/* and alpha_blending bit enables OMAP3 compatibility mode */
		dispc_enable_alpha_blending(channel, false);
		c->alpha_enabled = true;
	} else {
		dispc_enable_alpha_blending(channel, c->alpha_enabled);
	}
	if (dss_has_feature(FEAT_CPR)) {
		dispc_enable_cpr(channel, c->cpr_enable);
		dispc_set_cpr_coef(channel, &c->cpr_coefs);
	}
}

/* configure_dispc() tries to write values from cache to shadow registers.
 * It writes only to those managers/overlays that are not busy.
 * returns 0 if everything could be written to shadow registers.
 * returns 1 if not everything could be written to shadow registers. */
static int configure_dispc(void)
{
	struct overlay_cache_data *oc;
	struct manager_cache_data *mc;
	const int num_ovls = dss_feat_get_num_ovls();
	const int num_mgrs = dss_feat_get_num_mgrs();
	int i;
	int r;
	int used_ovls, j;
	bool mgr_busy[MAX_DSS_MANAGERS];
	bool mgr_go[MAX_DSS_MANAGERS];
	bool busy;

	r = 0;
	busy = false;

	for (i = 0; i < num_mgrs; i++) {
		mgr_busy[i] = dispc_go_busy(i);
		mgr_go[i] = false;
	}

	/* Commit overlay settings */
	for (i = 0; i < num_ovls; ++i) {
		oc = &dss_cache.overlay_cache[i];
		mc = &dss_cache.manager_cache[oc->channel];

		if (!oc->dirty)
			continue;

		if (oc->manual_update && !mc->do_manual_update)
			continue;

		if (mgr_busy[oc->channel]) {
			busy = true;
			continue;
		}

		r = configure_overlay(i);
		if (r)
			DSSERR("configure_overlay %d failed\n", i);

		dss_ovl_configure_cb(&oc->cb, i, oc->enabled);

		oc->dirty = false;
		oc->shadow_dirty = true;
		mgr_go[oc->channel] = true;
	}

	/* Commit manager settings */
	for (i = 0; i < num_mgrs; ++i) {
		mc = &dss_cache.manager_cache[i];

		if (!mc->dirty)
			continue;

		if (mc->manual_update && !mc->do_manual_update)
			continue;

		if (mgr_busy[i]) {
			busy = true;
			continue;
		}

		for (j = used_ovls = 0; j < num_ovls; j++) {
			oc = &dss_cache.overlay_cache[j];
			if (oc->channel == i && oc->enabled)
				used_ovls++;
		}

		configure_manager(i);

		dss_ovl_configure_cb(&mc->cb, i, used_ovls);

		mc->dirty = false;
		mc->shadow_dirty = true;
		mgr_go[i] = true;
	}

	/* set GO */
	for (i = 0; i < num_mgrs; ++i) {
		mc = &dss_cache.manager_cache[i];

		if (!mgr_go[i])
			continue;

		/* We don't need GO with manual update display. LCD iface will
		 * always be turned off after frame, and new settings will be
		 * taken in to use at next update */
		if (!mc->manual_upd_display){
			if(mc->skip_init)
				mc->skip_init = false;
			else
				dispc_go(i);
		}
	}

	if (busy)
		r = 1;
	else
		r = 0;

	return r;
}

/* Make the coordinates even. There are some strange problems with OMAP and
 * partial DSI update when the update widths are odd. */
static void make_even(u16 *x, u16 *w)
{
	u16 x1, x2;

	x1 = *x;
	x2 = *x + *w;

	x1 &= ~1;
	x2 = ALIGN(x2, 2);

	*x = x1;
	*w = x2 - x1;
}

/* Configure dispc for partial update. Return possibly modified update
 * area */
void dss_setup_partial_planes(struct omap_dss_device *dssdev,
		u16 *xi, u16 *yi, u16 *wi, u16 *hi, bool enlarge_update_area)
{
	struct overlay_cache_data *oc;
	struct manager_cache_data *mc;
	const int num_ovls = dss_feat_get_num_ovls();
	struct omap_overlay_manager *mgr;
	int i;
	u16 x, y, w, h;
	unsigned long flags;
	bool area_changed;

	x = *xi;
	y = *yi;
	w = *wi;
	h = *hi;

	DSSDBG("dispc_setup_partial_planes %d,%d %dx%d\n",
		*xi, *yi, *wi, *hi);

	mgr = dssdev->manager;

	if (!mgr) {
		DSSDBG("no manager\n");
		return;
	}

	make_even(&x, &w);

	spin_lock_irqsave(&dss_cache.lock, flags);

	/*
	 * Execute the outer loop until the inner loop has completed
	 * once without increasing the update area. This will ensure that
	 * all scaled overlays end up completely within the update area.
	 */
	do {
		area_changed = false;

		/* We need to show the whole overlay if it is scaled. So look
		 * for those, and make the update area larger if found.
		 * Also mark the overlay cache dirty */
		for (i = 0; i < num_ovls; ++i) {
			unsigned x1, y1, x2, y2;
			unsigned outw, outh;

			oc = &dss_cache.overlay_cache[i];

			if (oc->channel != mgr->id)
				continue;

			oc->dirty = true;

			if (!enlarge_update_area)
				continue;

			if (!oc->enabled)
				continue;

			if (!dispc_is_overlay_scaled(oc))
				continue;

			outw = oc->out_width == 0 ?
				oc->width : oc->out_width;
			outh = oc->out_height == 0 ?
				oc->height : oc->out_height;

			/* is the overlay outside the update region? */
			if (!rectangle_intersects(x, y, w, h,
						oc->pos_x, oc->pos_y,
						outw, outh))
				continue;

			/* if the overlay totally inside the update region? */
			if (rectangle_subset(oc->pos_x, oc->pos_y, outw, outh,
						x, y, w, h))
				continue;

			if (x > oc->pos_x)
				x1 = oc->pos_x;
			else
				x1 = x;

			if (y > oc->pos_y)
				y1 = oc->pos_y;
			else
				y1 = y;

			if ((x + w) < (oc->pos_x + outw))
				x2 = oc->pos_x + outw;
			else
				x2 = x + w;

			if ((y + h) < (oc->pos_y + outh))
				y2 = oc->pos_y + outh;
			else
				y2 = y + h;

			x = x1;
			y = y1;
			w = x2 - x1;
			h = y2 - y1;

			make_even(&x, &w);

			DSSDBG("changing upd area due to ovl(%d) "
			       "scaling %d,%d %dx%d\n",
				i, x, y, w, h);

			area_changed = true;
		}
	} while (area_changed);

	mc = &dss_cache.manager_cache[mgr->id];
	mc->do_manual_update = true;
	mc->enlarge_update_area = enlarge_update_area;
	mc->x = x;
	mc->y = y;
	mc->w = w;
	mc->h = h;

	configure_dispc();

	mc->do_manual_update = false;

	spin_unlock_irqrestore(&dss_cache.lock, flags);

	*xi = x;
	*yi = y;
	*wi = w;
	*hi = h;
}

static void schedule_completion_irq(void);

static void dss_completion_irq_handler(void *data, u32 mask)
{
	struct manager_cache_data *mc;
	struct overlay_cache_data *oc;
	const int num_ovls = ARRAY_SIZE(dss_cache.overlay_cache);
	const int num_mgrs = MAX_DSS_MANAGERS;
	const u32 masks[] = {
		DISPC_IRQ_FRAMEDONE | DISPC_IRQ_VSYNC,
		DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2,
		DISPC_IRQ_FRAMEDONETV | DISPC_IRQ_EVSYNC_EVEN |
		DISPC_IRQ_EVSYNC_ODD
	};
	int i;

	spin_lock(&dss_cache.lock);

	for (i = 0; i < num_mgrs; i++) {
		mc = &dss_cache.manager_cache[i];
		if (mask & masks[i]) {
			if (mgrs[i] && mgrs[i]->device)
				mgrs[i]->device->first_vsync = true;
			dss_ovl_cb(&mc->cb.dispc, i, DSS_COMPLETION_DISPLAYED);
			mc->cb.dispc_displayed = true;
		}
	}

	/* notify all overlays on that manager */
	for (i = 0; i < num_ovls; i++) {
		oc = &dss_cache.overlay_cache[i];
		if (mask & masks[oc->channel]) {
			dss_ovl_cb(&oc->cb.dispc, i, DSS_COMPLETION_DISPLAYED);
			oc->cb.dispc_displayed = true;
		}
	}

	schedule_completion_irq();

	spin_unlock(&dss_cache.lock);
}

static void schedule_completion_irq(void)
{
	struct manager_cache_data *mc;
	struct overlay_cache_data *oc;
	const int num_ovls = ARRAY_SIZE(dss_cache.overlay_cache);
	const int num_mgrs = MAX_DSS_MANAGERS;
	const u32 masks[] = {
		DISPC_IRQ_FRAMEDONE | DISPC_IRQ_VSYNC,
		DISPC_IRQ_FRAMEDONE2 | DISPC_IRQ_VSYNC2,
		DISPC_IRQ_FRAMEDONETV | DISPC_IRQ_EVSYNC_EVEN |
		DISPC_IRQ_EVSYNC_ODD
	};
	u32 mask = 0;
	int i;

	for (i = 0; i < num_mgrs; i++) {
		mc = &dss_cache.manager_cache[i];
		if (mc->cb.dispc.fn &&
				(mc->cb.dispc.mask & DSS_COMPLETION_DISPLAYED))
			mask |= masks[i];
	}

	/* notify all overlays on that manager */
	for (i = 0; i < num_ovls; i++) {
		oc = &dss_cache.overlay_cache[i];
		if (oc->cb.dispc.fn && oc->enabled &&
				(oc->cb.dispc.mask & DSS_COMPLETION_DISPLAYED))
			mask |= masks[oc->channel];
	}

	if (mask != dss_cache.comp_irq_enabled) {
		if (dss_cache.comp_irq_enabled)
			omap_dispc_unregister_isr(dss_completion_irq_handler,
				NULL, dss_cache.comp_irq_enabled);
		if (mask)
			omap_dispc_register_isr(dss_completion_irq_handler,
				NULL, mask);
		dss_cache.comp_irq_enabled = mask;
	}
}

void dss_start_update(struct omap_dss_device *dssdev)
{
	struct manager_cache_data *mc;
	struct overlay_cache_data *oc;
	const int num_ovls = dss_feat_get_num_ovls();
	const int num_mgrs = dss_feat_get_num_mgrs();
	struct omap_overlay_manager *mgr;
	int i;
	unsigned long flags;

	mgr = dssdev->manager;

	spin_lock_irqsave(&dss_cache.lock, flags);
	for (i = 0; i < num_ovls; ++i) {
		oc = &dss_cache.overlay_cache[i];
		if (oc->channel != mgr->id)
			continue;

		if (oc->shadow_dirty) {
			dss_ovl_program_cb(&oc->cb, i);
			oc->dispc_channel = oc->channel;
		}
		oc->shadow_dirty = false;
	}

	for (i = 0; i < num_mgrs; ++i) {
		mc = &dss_cache.manager_cache[i];
		if (mgr->id != i)
			continue;

		if (mc->shadow_dirty)
			dss_ovl_program_cb(&mc->cb, i);
		mc->shadow_dirty = false;
	}

	schedule_completion_irq();
	spin_unlock_irqrestore(&dss_cache.lock, flags);

	dssdev->manager->enable(dssdev->manager);
}

static void dss_apply_irq_handler(void *data, u32 mask)
{
	struct manager_cache_data *mc;
	struct overlay_cache_data *oc;
	const int num_ovls = dss_feat_get_num_ovls();
	const int num_mgrs = dss_feat_get_num_mgrs();
	int i, r;
	bool mgr_busy[MAX_DSS_MANAGERS];
	u32 irq_mask;

	spin_lock(&dss_cache.lock);

	for (i = 0; i < num_mgrs; i++)
		mgr_busy[i] = dispc_go_busy(i);

	for (i = 0; i < num_ovls; ++i) {
		oc = &dss_cache.overlay_cache[i];
		if (!mgr_busy[oc->channel] && oc->shadow_dirty) {
			dss_ovl_program_cb(&oc->cb, i);
			oc->dispc_channel = oc->channel;
			oc->shadow_dirty = false;
		}
	}

	for (i = 0; i < num_mgrs; ++i) {
		mc = &dss_cache.manager_cache[i];
		if (!mgr_busy[i] && mc->shadow_dirty) {
			if (mgrs[i] && mgrs[i]->device)
				mgrs[i]->device->first_vsync = true;
			dss_ovl_program_cb(&mc->cb, i);
			mc->shadow_dirty = false;
		}
	}

	schedule_completion_irq();

	r = configure_dispc();
	if (r == 1)
		goto end;

	/*
	 * FIXME Sometimes when handling an interrupt for a manager, the
	 * manager is still busy at the beginning of the interrupt handler.
	 * Later it becomes idle, so we unregister the interrupt.  This
	 * leaves the shadow_dirty flag in an incorrect true state, and also
	 * misses the 'programmed' callback.
	 *
	 * For now, we do not unregister the interrupt if any manager
	 * was busy at the first read of the GO bits.  A better fix would be
	 * to keep the first read busy state in the cache, so we do not operate
	 * on instantaneous reads of the GO bit.
	 */

	/* re-read busy flags */
	for (i = 0; i < num_mgrs; i++)
		mgr_busy[i] |= dispc_go_busy(i);

	/* keep running as long as there are busy managers, so that
	 * we can collect overlay-applied information */
	for (i = 0; i < num_mgrs; ++i) {
		if (mgr_busy[i])
			goto end;
	}

	irq_mask = DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_ODD |
			DISPC_IRQ_EVSYNC_EVEN;
	if (dss_has_feature(FEAT_MGR_LCD2))
		irq_mask |= DISPC_IRQ_VSYNC2;

	omap_dispc_unregister_isr(dss_apply_irq_handler, NULL, irq_mask);
	dss_cache.irq_enabled = false;

end:
	spin_unlock(&dss_cache.lock);
}

static int omap_dss_mgr_blank(struct omap_overlay_manager *mgr,
			bool wait_for_go)
{
	struct overlay_cache_data *oc;
	struct manager_cache_data *mc;
	unsigned long flags;
	int r, r_get, i;

	DSSDBG("omap_dss_mgr_blank(%s,wait=%d)\n", mgr->name, wait_for_go);

	r_get = r = dispc_runtime_get();
	/* still clear cache even if failed to get clocks, just don't config */

	spin_lock_irqsave(&dss_cache.lock, flags);

	/* disable overlays in overlay info structs and in cache */
	for (i = 0; i < omap_dss_get_num_overlays(); i++) {
		struct omap_overlay_info oi = { .enabled = false };
		struct omap_overlay *ovl;

		ovl = omap_dss_get_overlay(i);

		if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC) ||
		    ovl->manager != mgr)
			continue;

		oc = &dss_cache.overlay_cache[ovl->id];

		/* complete unconfigured info in cache */
		dss_ovl_cb(&oc->cb.cache, i, DSS_COMPLETION_ECLIPSED_CACHE);
		oc->cb.cache.fn = NULL;

		ovl->info = oi;
		ovl->info_dirty = false;
		oc->dirty = true;
		oc->enabled = false;
	}

	/* dirty manager */
	mc = &dss_cache.manager_cache[mgr->id];
	dss_ovl_cb(&mc->cb.cache, i, DSS_COMPLETION_ECLIPSED_CACHE);
	mc->cb.cache.fn = NULL;
	mgr->info.cb.fn = NULL;
	mc->dirty = true;
	mgr->info_dirty = false;

	/*
	 * TRICKY: Enable apply irq even if not waiting for vsync, so that
	 * DISPC programming takes place in case GO bit was on.
	 */
	if (!dss_cache.irq_enabled) {
		u32 mask;

		mask = DISPC_IRQ_VSYNC	| DISPC_IRQ_EVSYNC_ODD |
			DISPC_IRQ_EVSYNC_EVEN;
		if (dss_has_feature(FEAT_MGR_LCD2))
			mask |= DISPC_IRQ_VSYNC2;

		r = omap_dispc_register_isr(dss_apply_irq_handler, NULL, mask);
		dss_cache.irq_enabled = true;
	}

	if (!r_get) {
		r = configure_dispc();
		if (r)
			pr_info("mgr_blank while GO is set");
	}

	if (r_get || !wait_for_go) {
		/* pretend that programming has happened */
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			oc = &dss_cache.overlay_cache[i];
			if (oc->channel != mgr->id)
				continue;
			if (r && oc->dirty)
				dss_ovl_configure_cb(&oc->cb, i, false);
			if (oc->shadow_dirty) {
				dss_ovl_program_cb(&oc->cb, i);
				oc->dispc_channel = oc->channel;
				oc->shadow_dirty = false;
			} else {
				pr_warn("ovl%d-shadow is not dirty\n", i);
			}
		}

		if (r && mc->dirty)
			dss_ovl_configure_cb(&mc->cb, i, false);
		if (mc->shadow_dirty) {
			dss_ovl_program_cb(&mc->cb, i);
			mc->shadow_dirty = false;
		} else {
			pr_warn("mgr%d-shadow is not dirty\n", mgr->id);
		}
	}

	spin_unlock_irqrestore(&dss_cache.lock, flags);

	if (wait_for_go)
		mgr->wait_for_go(mgr);

	if (!r_get)
		dispc_runtime_put();

	return r;
}

int omap_dss_manager_unregister_callback(struct omap_overlay_manager *mgr,
					 struct omapdss_ovl_cb *cb)
{
	unsigned long flags;
	int r = 0;
	spin_lock_irqsave(&dss_cache.lock, flags);
	if (mgr->info_dirty &&
	    mgr->info.cb.fn == cb->fn &&
	    mgr->info.cb.data == cb->data)
		mgr->info.cb.fn = NULL;
	else
		r = -EPERM;
	spin_unlock_irqrestore(&dss_cache.lock, flags);
	return r;
}

static int omap_dss_mgr_apply(struct omap_overlay_manager *mgr)
{
	struct overlay_cache_data *oc;
	struct manager_cache_data *mc;
	struct omap_dss_device *dssdev;
	int i;
	struct omap_overlay *ovl;
	int num_planes_enabled = 0;
	bool use_fifomerge;
	unsigned long flags;
	int r;

	DSSDBG("omap_dss_mgr_apply(%s)\n", mgr->name);

	r = dispc_runtime_get();
	if (r)
		return r;

	spin_lock_irqsave(&dss_cache.lock, flags);

	if (!mgr->device || mgr->device->state != OMAP_DSS_DISPLAY_ACTIVE) {
		pr_info_ratelimited("cannot apply mgr(%s) on inactive device\n",
								mgr->name);
		r = -ENODEV;
		goto done;
	}

	/* Configure overlays */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		struct omap_dss_device *dssdev;

		ovl = omap_dss_get_overlay(i);

		if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
			continue;

		if (ovl->manager != mgr)
			continue;

		oc = &dss_cache.overlay_cache[ovl->id];
		dssdev = mgr->device;

		if (!overlay_enabled(ovl) || !dssdev) {
			ovl->info.enabled = false;
		} else if (!ovl->info_dirty) {
			if (oc->enabled)
				++num_planes_enabled;
			continue;
		} else if (dss_check_overlay(ovl, dssdev)) {
			ovl->info.enabled = false;
		}

		/* complete unconfigured info in cache */
		dss_ovl_cb(&oc->cb.cache, i,
#if 0
			   (oc->cb.cache.fn == ovl->info.cb.fn &&
			    oc->cb.cache.data == ovl->info.cb.data) ?
			   DSS_COMPLETION_CHANGED_CACHE :
#endif
			   DSS_COMPLETION_ECLIPSED_CACHE);
		oc->cb.cache = ovl->info.cb;
		ovl->info.cb.fn = NULL;

		ovl->info_dirty = false;
		if (ovl->info.enabled || oc->enabled)
			oc->dirty = true;
		oc->enabled = ovl->info.enabled;
		if (!oc->enabled)
			continue;

		oc->paddr = ovl->info.paddr;
		oc->vaddr = ovl->info.vaddr;
		oc->p_uv_addr = ovl->info.p_uv_addr;
		oc->screen_width = ovl->info.screen_width;
		oc->width = ovl->info.width;
		oc->height = ovl->info.height;
		oc->color_mode = ovl->info.color_mode;
		oc->rotation = ovl->info.rotation;
		oc->rotation_type = ovl->info.rotation_type;
		oc->mirror = ovl->info.mirror;
		oc->pos_x = ovl->info.pos_x;
		oc->pos_y = ovl->info.pos_y;
		oc->out_width = ovl->info.out_width;
		oc->out_height = ovl->info.out_height;
		oc->global_alpha = ovl->info.global_alpha;
		oc->pre_mult_alpha = ovl->info.pre_mult_alpha;
		oc->zorder = ovl->info.zorder;
		oc->min_x_decim = ovl->info.min_x_decim;
		oc->max_x_decim = ovl->info.max_x_decim;
		oc->min_y_decim = ovl->info.min_y_decim;
		oc->max_y_decim = ovl->info.max_y_decim;
		oc->cconv = ovl->info.cconv;

		oc->replication =
			dss_use_replication(dssdev, ovl->info.color_mode);

		oc->ilace = dssdev->type == OMAP_DISPLAY_TYPE_VENC;

		oc->channel = mgr->id;

		oc->manual_update =
			dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
			dssdev->driver->get_update_mode(dssdev) !=
				OMAP_DSS_UPDATE_AUTO;

		++num_planes_enabled;
	}

	/* configure manager */
	if (!(mgr->caps & OMAP_DSS_OVL_MGR_CAP_DISPC))
		goto skip_mgr;

	mc = &dss_cache.manager_cache[mgr->id];

	if (mgr->device_changed) {
		mgr->device_changed = false;
		mgr->info_dirty  = true;
	}

	if (!mgr->info_dirty)
		goto skip_mgr;

	if (!mgr->device)
		goto skip_mgr;

	dssdev = mgr->device;

	/* complete unconfigured info in cache */
	dss_ovl_cb(&mc->cb.cache, mgr->id,
#if 0
		   (mc->cb.cache.fn == mgr->info.cb.fn &&
		    mc->cb.cache.data == mgr->info.cb.data) ?
		   DSS_COMPLETION_CHANGED_CACHE :
#endif
		   DSS_COMPLETION_ECLIPSED_CACHE);
	mc->cb.cache = mgr->info.cb;
	mgr->info.cb.fn = NULL;

	mgr->info_dirty = false;
	mc->dirty = true;

	mc->default_color = mgr->info.default_color;
	mc->trans_key_type = mgr->info.trans_key_type;
	mc->trans_key = mgr->info.trans_key;
	mc->trans_enabled = mgr->info.trans_enabled;
	mc->alpha_enabled = mgr->info.alpha_enabled;
	mc->cpr_coefs = mgr->info.cpr_coefs;
	mc->cpr_enable = mgr->info.cpr_enable;
	mc->gamma = mgr->info.gamma;

	mc->manual_upd_display =
		dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE;

	mc->manual_update =
		dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
		dssdev->driver->get_update_mode(dssdev) !=
			OMAP_DSS_UPDATE_AUTO;

	mc->skip_init = dssdev->skip_init;

skip_mgr:

	/* XXX TODO: Try to get fifomerge working. The problem is that it
	 * affects both managers, not individually but at the same time. This
	 * means the change has to be well synchronized. I guess the proper way
	 * is to have a two step process for fifo merge:
	 *        fifomerge enable:
	 *             1. disable other planes, leaving one plane enabled
	 *             2. wait until the planes are disabled on HW
	 *             3. config merged fifo thresholds, enable fifomerge
	 *        fifomerge disable:
	 *             1. config unmerged fifo thresholds, disable fifomerge
	 *             2. wait until fifo changes are in HW
	 *             3. enable planes
	 */
	use_fifomerge = false;

	/* Configure overlay fifos */
	for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
		struct omap_dss_device *dssdev;
		u32 size;

		ovl = omap_dss_get_overlay(i);

		if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
			continue;

		oc = &dss_cache.overlay_cache[ovl->id];

		if (!oc->enabled)
			continue;

		dssdev = ovl->manager->device;
		if (!dssdev)
			continue;

		size = dispc_get_plane_fifo_size(ovl->id);
		if (use_fifomerge)
			size *= 3;

		switch (dssdev->type) {
		case OMAP_DISPLAY_TYPE_DPI:
		case OMAP_DISPLAY_TYPE_DBI:
		case OMAP_DISPLAY_TYPE_SDI:
		case OMAP_DISPLAY_TYPE_VENC:
		case OMAP_DISPLAY_TYPE_HDMI:
			default_get_overlay_fifo_thresholds(ovl->id, size,
					&oc->burst_size, &oc->fifo_low,
					&oc->fifo_high);
			break;
#ifdef CONFIG_OMAP2_DSS_DSI
		case OMAP_DISPLAY_TYPE_DSI:
			dsi_get_overlay_fifo_thresholds(ovl->id, size,
					&oc->burst_size, &oc->fifo_low,
					&oc->fifo_high);
			break;
#endif
		default:
			BUG();
		}
	}

	r = 0;
	if (!dss_cache.irq_enabled) {
		u32 mask;

		mask = DISPC_IRQ_VSYNC	| DISPC_IRQ_EVSYNC_ODD |
			DISPC_IRQ_EVSYNC_EVEN;
		if (dss_has_feature(FEAT_MGR_LCD2))
			mask |= DISPC_IRQ_VSYNC2;

		r = omap_dispc_register_isr(dss_apply_irq_handler, NULL, mask);
		dss_cache.irq_enabled = true;
	}
	configure_dispc();

done:
	spin_unlock_irqrestore(&dss_cache.lock, flags);

	dispc_runtime_put();

	return r;
}

#ifdef CONFIG_DEBUG_FS
static void seq_print_cb(struct seq_file *s, struct omapdss_ovl_cb *cb)
{
	if (!cb->fn) {
		seq_printf(s, "(none)\n");
		return;
	}

	seq_printf(s, "mask=%c%c%c%c [%p] %pf\n",
		   (cb->mask & DSS_COMPLETION_CHANGED) ? 'C' : '-',
		   (cb->mask & DSS_COMPLETION_PROGRAMMED) ? 'P' : '-',
		   (cb->mask & DSS_COMPLETION_DISPLAYED) ? 'D' : '-',
		   (cb->mask & DSS_COMPLETION_RELEASED) ? 'R' : '-',
		   cb->data,
		   cb->fn);
}
#endif

static void seq_print_cbs(struct omap_overlay_manager *mgr, struct seq_file *s)
{
#ifdef CONFIG_DEBUG_FS
	struct manager_cache_data *mc;
	unsigned long flags;

	spin_lock_irqsave(&dss_cache.lock, flags);

	mc = &dss_cache.manager_cache[mgr->id];

	seq_printf(s, "  DISPC pipeline:\n\n"
		      "    info:%13s ", mgr->info_dirty ? "DIRTY" : "clean");
	seq_print_cb(s, &mgr->info.cb);
	seq_printf(s, "    cache:%12s ", mc->dirty ? "DIRTY" : "clean");
	seq_print_cb(s, &mc->cb.cache);
	seq_printf(s, "    shadow:  %s %s ",
			mc->cb.shadow_enabled ? "ACT" : "off",
			mc->shadow_dirty ? "DIRTY" : "clean");
	seq_print_cb(s, &mc->cb.shadow);
	seq_printf(s, "    dispc:%12s ",
			mc->cb.dispc_displayed ? "DISPLAYED" : "");
	seq_print_cb(s, &mc->cb.dispc);
	seq_printf(s, "\n");

	spin_unlock_irqrestore(&dss_cache.lock, flags);
#endif
}

static int dss_check_manager(struct omap_overlay_manager *mgr)
{
	/* if we have OMAP3 alpha compatibility, alpha blending is always on */
	if (dss_has_feature(FEAT_ALPHA_OMAP3_COMPAT)) {
		if (!mgr->info.alpha_enabled)
			return -EINVAL;
	} else {
		/*
		 * OMAP3- supports only graphics destination transparency
		 * color key and alpha blending simultaneously.
		 * See TRM 15.4.2.4.2.2 Alpha Mode.
		 */
		if (mgr->info.alpha_enabled && mgr->info.trans_enabled &&
			mgr->info.trans_key_type != OMAP_DSS_COLOR_KEY_GFX_DST)
			return -EINVAL;
	}

	return 0;
}

int omap_dss_ovl_set_info(struct omap_overlay *ovl,
		struct omap_overlay_info *info)
{
	int r;
	struct omap_overlay_info old_info;
	unsigned long flags;

	spin_lock_irqsave(&dss_cache.lock, flags);
	old_info = ovl->info;
	ovl->info = *info;

	if (ovl->manager) {
		r = dss_check_overlay(ovl, ovl->manager->device);
		if (r) {
			ovl->info = old_info;
			spin_unlock_irqrestore(&dss_cache.lock, flags);
			return r;
		}
	}

	/* complete previous settings */
	if (ovl->info_dirty)
		dss_ovl_cb(&old_info.cb, ovl->id,
			   (info->cb.fn == old_info.cb.fn &&
			    info->cb.data == old_info.cb.data) ?
			   DSS_COMPLETION_CHANGED_SET :
			   DSS_COMPLETION_ECLIPSED_SET);

	ovl->info_dirty = true;
	spin_unlock_irqrestore(&dss_cache.lock, flags);

	return 0;
}


static int omap_dss_mgr_set_info(struct omap_overlay_manager *mgr,
		struct omap_overlay_manager_info *info)
{
	int r;
	struct omap_overlay_manager_info old_info;
	unsigned long flags;

	spin_lock_irqsave(&dss_cache.lock, flags);
	old_info = mgr->info;
	mgr->info = *info;

	r = dss_check_manager(mgr);
	if (r) {
		mgr->info = old_info;
		spin_unlock_irqrestore(&dss_cache.lock, flags);
		return r;
	}

	if (mgr->info_dirty)
		dss_ovl_cb(&old_info.cb, mgr->id, DSS_COMPLETION_ECLIPSED_SET);

	mgr->info_dirty = true;
	spin_unlock_irqrestore(&dss_cache.lock, flags);

	return 0;
}

static void omap_dss_mgr_get_info(struct omap_overlay_manager *mgr,
		struct omap_overlay_manager_info *info)
{
	*info = mgr->info;
}

static int dss_mgr_enable(struct omap_overlay_manager *mgr)
{
	dispc_enable_channel(mgr->id, mgr->device->type, 1);
	return 0;
}

static int dss_mgr_disable(struct omap_overlay_manager *mgr)
{
	dispc_enable_channel(mgr->id, mgr->device->type, 0);
	return 0;
}

static void omap_dss_add_overlay_manager(struct omap_overlay_manager *manager)
{
	++num_managers;
	list_add_tail(&manager->list, &manager_list);
	if (manager->id < ARRAY_SIZE(mgrs))
		mgrs[manager->id] = manager;
}

int dss_init_overlay_managers(struct platform_device *pdev)
{
	int i, r;

	spin_lock_init(&dss_cache.lock);

	INIT_LIST_HEAD(&manager_list);

	num_managers = 0;

	for (i = 0; i < dss_feat_get_num_mgrs(); ++i) {
		struct omap_overlay_manager *mgr;
		mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);

		BUG_ON(mgr == NULL);

		/* alpha blending always on with OMAP3 alpha compatibility */
		if (dss_has_feature(FEAT_ALPHA_OMAP3_COMPAT))
			mgr->info.alpha_enabled = true;

		switch (i) {
		case 0:
			mgr->name = "lcd";
			mgr->id = OMAP_DSS_CHANNEL_LCD;
			break;
		case 1:
			mgr->name = "tv";
			mgr->id = OMAP_DSS_CHANNEL_DIGIT;
			break;
		case 2:
			mgr->name = "lcd2";
			mgr->id = OMAP_DSS_CHANNEL_LCD2;
			break;
		}

		mgr->set_device = &omap_dss_set_device;
		mgr->unset_device = &omap_dss_unset_device;
		mgr->apply = &omap_dss_mgr_apply;
		mgr->set_manager_info = &omap_dss_mgr_set_info;
		mgr->get_manager_info = &omap_dss_mgr_get_info;
		mgr->wait_for_go = &dss_mgr_wait_for_go;
		mgr->wait_for_vsync = &dss_mgr_wait_for_vsync;
		mgr->blank = &omap_dss_mgr_blank;
		mgr->dump_cb = &seq_print_cbs;

		mgr->enable = &dss_mgr_enable;
		mgr->disable = &dss_mgr_disable;

		mgr->caps = OMAP_DSS_OVL_MGR_CAP_DISPC;
		mgr->supported_displays =
			dss_feat_get_supported_displays(mgr->id);

		dss_overlay_setup_dispc_manager(mgr);

		omap_dss_add_overlay_manager(mgr);

		r = kobject_init_and_add(&mgr->kobj, &manager_ktype,
				&pdev->dev.kobj, "manager%d", i);

		if (r) {
			DSSERR("failed to create sysfs file\n");
			continue;
		}
	}

#ifdef L4_EXAMPLE
	{
		int omap_dss_mgr_apply_l4(struct omap_overlay_manager *mgr)
		{
			DSSDBG("omap_dss_mgr_apply_l4(%s)\n", mgr->name);

			return 0;
		}

		struct omap_overlay_manager *mgr;
		mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);

		BUG_ON(mgr == NULL);

		mgr->name = "l4";
		mgr->supported_displays =
			OMAP_DISPLAY_TYPE_DBI | OMAP_DISPLAY_TYPE_DSI;

		mgr->set_device = &omap_dss_set_device;
		mgr->unset_device = &omap_dss_unset_device;
		mgr->apply = &omap_dss_mgr_apply_l4;
		mgr->set_manager_info = &omap_dss_mgr_set_info;
		mgr->get_manager_info = &omap_dss_mgr_get_info;

		dss_overlay_setup_l4_manager(mgr);

		omap_dss_add_overlay_manager(mgr);

		r = kobject_init_and_add(&mgr->kobj, &manager_ktype,
				&pdev->dev.kobj, "managerl4");

		if (r)
			DSSERR("failed to create sysfs file\n");
	}
#endif

	return 0;
}

void dss_uninit_overlay_managers(struct platform_device *pdev)
{
	struct omap_overlay_manager *mgr;

	while (!list_empty(&manager_list)) {
		mgr = list_first_entry(&manager_list,
				struct omap_overlay_manager, list);
		list_del(&mgr->list);
		kobject_del(&mgr->kobj);
		kobject_put(&mgr->kobj);
		kfree(mgr);
	}

	num_managers = 0;
}

int omap_dss_get_num_overlay_managers(void)
{
	return num_managers;
}
EXPORT_SYMBOL(omap_dss_get_num_overlay_managers);

struct omap_overlay_manager *omap_dss_get_overlay_manager(int num)
{
	int i = 0;
	struct omap_overlay_manager *mgr;

	list_for_each_entry(mgr, &manager_list, list) {
		if (i++ == num)
			return mgr;
	}

	return NULL;
}
EXPORT_SYMBOL(omap_dss_get_overlay_manager);

