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

#include <linux/kernel.h>

#include <linux/notifier.h>
#include <mach/tiler.h>

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#include "dsscomp.h"

int debug;
module_param(debug, int, 0644);

/* color formats supported - bitfield info is used for truncation logic */
static const struct color_info {
	int a_ix, a_bt;	/* bitfields */
	int r_ix, r_bt;
	int g_ix, g_bt;
	int b_ix, b_bt;
	int x_bt;
	enum omap_color_mode mode;
	const char *name;
} fmts[2][16] = { {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 1, OMAP_DSS_COLOR_CLUT1, "BITMAP1" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 2, OMAP_DSS_COLOR_CLUT2, "BITMAP2" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 4, OMAP_DSS_COLOR_CLUT4, "BITMAP4" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 8, OMAP_DSS_COLOR_CLUT8, "BITMAP8" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX16, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX32, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB16_1555,
								"xRGB15-1555" },
}, {
	{ 0,  0, 0,  0, 0,  0, 0, 0, 12, OMAP_DSS_COLOR_NV12, "NV12" },
	{ 0,  0, 12, 4, 8,  4, 4, 4, 4, OMAP_DSS_COLOR_RGBX16, "RGBx12-4444" },
	{ 0,  4, 12, 4, 8,  4, 4, 4, 0, OMAP_DSS_COLOR_RGBA16, "RGBA16-4444" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 0, 0, "invalid" },
	{ 0,  0, 8,  4, 4,  4, 0, 4, 4, OMAP_DSS_COLOR_RGB12U, "xRGB12-4444" },
	{ 12, 4, 8,  4, 4,  4, 0, 4, 0, OMAP_DSS_COLOR_ARGB16, "ARGB16-4444" },
	{ 0,  0, 11, 5, 5,  6, 0, 5, 0, OMAP_DSS_COLOR_RGB16, "RGB16-565" },
	{ 15, 1, 10, 5, 5,  5, 0, 5, 0, OMAP_DSS_COLOR_ARGB16_1555,
								"ARGB16-1555" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 8, OMAP_DSS_COLOR_RGB24U, "xRGB24-8888" },
	{ 0,  0, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_RGB24P, "RGB24-888" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_YUV2, "YUYV" },
	{ 0,  0, 0,  0, 0,  0, 0, 0, 16, OMAP_DSS_COLOR_UYVY, "UYVY" },
	{ 24, 8, 16, 8, 8,  8, 0, 8, 0, OMAP_DSS_COLOR_ARGB32, "ARGB32-8888" },
	{ 0,  8, 24, 8, 16, 8, 8, 8, 0, OMAP_DSS_COLOR_RGBA32, "RGBA32-8888" },
	{ 0,  0, 24, 8, 16, 8, 8, 8, 8, OMAP_DSS_COLOR_RGBX32, "RGBx24-8888" },
	{ 0,  0, 10, 5, 5,  5, 0, 5, 1, OMAP_DSS_COLOR_XRGB16_1555,
								"xRGB15-1555" },
} };

static const struct color_info *get_color_info(enum omap_color_mode mode)
{
	int i;
	for (i = 0; i < sizeof(fmts) / sizeof(fmts[0][0]); i++)
		if (fmts[0][i].mode == mode)
			return fmts[0] + i;
	return NULL;
}

static int color_mode_to_bpp(enum omap_color_mode color_mode)
{
	const struct color_info *ci = get_color_info(color_mode);
	BUG_ON(!ci);

	return ci->a_bt + ci->r_bt + ci->g_bt + ci->b_bt + ci->x_bt;
}

#ifdef CONFIG_DEBUG_FS
const char *dsscomp_get_color_name(enum omap_color_mode m)
{
	const struct color_info *ci = get_color_info(m);
	return ci ? ci->name : NULL;
}
#endif

union rect {
	struct {
		s32 x;
		s32 y;
		s32 w;
		s32 h;
	};
	struct {
		s32 xy[2];
		s32 wh[2];
	};
	struct dss2_rect_t r;
};

int crop_to_rect(union rect *crop, union rect *win, union rect *vis,
						int rotation, int mirror)
{
	int c, swap = rotation & 1;

	/* align crop window with display coordinates */
	if (swap)
		crop->y -= (crop->h = -crop->h);
	if (rotation & 2)
		crop->xy[!swap] -= (crop->wh[!swap] = -crop->wh[!swap]);
	if ((!mirror) ^ !(rotation & 2))
		crop->xy[swap] -= (crop->wh[swap] = -crop->wh[swap]);

	for (c = 0; c < 2; c++) {
		/* see if complete buffer is outside the vis or it is
		   fully cropped or scaled to 0 */
		if (win->wh[c] <= 0 || vis->wh[c] <= 0 ||
		    win->xy[c] + win->wh[c] <= vis->xy[c] ||
		    win->xy[c] >= vis->xy[c] + vis->wh[c] ||
		    !crop->wh[c ^ swap])
			return -ENOENT;

		/* crop left/top */
		if (win->xy[c] < vis->xy[c]) {
			/* correction term */
			int a = (vis->xy[c] - win->xy[c]) *
						crop->wh[c ^ swap] / win->wh[c];
			crop->xy[c ^ swap] += a;
			crop->wh[c ^ swap] -= a;
			win->wh[c] -= vis->xy[c] - win->xy[c];
			win->xy[c] = vis->xy[c];
		}
		/* crop right/bottom */
		if (win->xy[c] + win->wh[c] > vis->xy[c] + vis->wh[c]) {
			crop->wh[c ^ swap] = crop->wh[c ^ swap] *
				(vis->xy[c] + vis->wh[c] - win->xy[c]) /
								win->wh[c];
			win->wh[c] = vis->xy[c] + vis->wh[c] - win->xy[c];
		}

		if (!crop->wh[c ^ swap] || !win->wh[c])
			return -ENOENT;
	}

	/* realign crop window to buffer coordinates */
	if (rotation & 2)
		crop->xy[!swap] -= (crop->wh[!swap] = -crop->wh[!swap]);
	if ((!mirror) ^ !(rotation & 2))
		crop->xy[swap] -= (crop->wh[swap] = -crop->wh[swap]);
	if (swap)
		crop->y -= (crop->h = -crop->h);
	return 0;
}

int set_dss_ovl_info(struct dss2_ovl_info *oi)
{
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	struct dss2_ovl_cfg *cfg;
	union rect crop, win, vis;
	int c;

	/* check overlay number */
	if (!oi || oi->cfg.ix >= omap_dss_get_num_overlays())
		return -EINVAL;
	cfg = &oi->cfg;
	ovl = omap_dss_get_overlay(cfg->ix);

	/* just in case there are new fields, we get the current info */
	ovl->get_overlay_info(ovl, &info);

	info.enabled = cfg->enabled;
	if (!cfg->enabled)
		goto done;

	/* copied params */
	info.zorder = cfg->zorder;

	if (cfg->zonly)
		goto done;

	info.global_alpha = cfg->global_alpha;
	info.pre_mult_alpha = cfg->pre_mult_alpha;
	info.rotation = cfg->rotation;
	info.mirror = cfg->mirror;
	info.color_mode = cfg->color_mode;

	/* crop to screen */
	crop.r = cfg->crop;
	win.r = cfg->win;
	vis.x = vis.y = 0;
	vis.w = ovl->manager->device->panel.timings.x_res;
	vis.h = ovl->manager->device->panel.timings.y_res;

	if (crop_to_rect(&crop, &win, &vis, cfg->rotation, cfg->mirror) ||
								vis.w < 2) {
		info.enabled = false;
		goto done;
	}

	/* adjust crop to UV pixel boundaries */
	for (c = 0; c < (cfg->color_mode == OMAP_DSS_COLOR_NV12 ? 2 :
		(cfg->color_mode &
		 (OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) ? 1 : 0); c++) {
		/* keep the output window to avoid trembling edges */
		crop.wh[c] += crop.xy[c] & 1;	/* round down start */
		crop.xy[c] &= ~1;
		crop.wh[c] += crop.wh[c] & 1;	/* round up end */

		/*
		 * Buffer is aligned on UV pixel boundaries, so no
		 * worries about extending crop region.
		 */
	}

	info.width  = crop.w;
	info.height = crop.h;
	if (cfg->rotation & 1)
		/* DISPC uses swapped height/width for 90/270 degrees */
		swap(info.width, info.height);
	info.pos_x = win.x;
	info.pos_y = win.y;
	info.out_width = win.w;
	info.out_height = win.h;

	/* calculate addresses and cropping */
	info.paddr = oi->ba;
	info.p_uv_addr = (info.color_mode == OMAP_DSS_COLOR_NV12) ? oi->uv : 0;
	info.vaddr = NULL;

	/* check for TILER 2D buffer */
	if (info.paddr >= 0x60000000 && info.paddr < 0x78000000) {
		int bpp = 1 << ((info.paddr >> 27) & 3);
		struct tiler_view_t t;

		/* crop to top-left */

		/*
		 * DSS supports YUV422 on 32-bit mode, but its technically
		 * 2 bytes-per-pixel.
		 * Also RGB24-888 is 3 bytes-per-pixel even though no
		 * tiler pixel format matches this.
		 */
		if (cfg->color_mode &
				(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY))
			bpp = 2;
		else if (cfg->color_mode == OMAP_DSS_COLOR_RGB24P)
			bpp = 3;

		tilview_create(&t, info.paddr, cfg->width, cfg->height);
		info.paddr -= t.tsptr;
		tilview_crop(&t, 0, crop.y, cfg->width, crop.h);
		info.paddr += t.tsptr + bpp * crop.x;

		info.rotation_type = OMAP_DSS_ROT_TILER;
		info.screen_width = 0;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12) {
			tilview_create(&t, info.p_uv_addr,
					cfg->width >> 1, cfg->height >> 1);
			info.p_uv_addr -= t.tsptr;
			tilview_crop(&t, 0, crop.y >> 1, cfg->width >> 1,
								crop.h >> 1);
			info.p_uv_addr += t.tsptr + bpp * crop.x;
		}
	} else {
		/* program tiler 1D as SDMA */

		int bpp = color_mode_to_bpp(cfg->color_mode);
		info.screen_width = cfg->stride * 8 / (bpp == 12 ? 8 : bpp);
		info.paddr += crop.x * (bpp / 8) + crop.y * cfg->stride;

		/* for NV12 format also crop NV12 */
		if (info.color_mode == OMAP_DSS_COLOR_NV12)
			info.p_uv_addr += crop.x * (bpp / 8) +
				(crop.y >> 1) * cfg->stride;

		/* no rotation on DMA buffer */
		if (cfg->rotation & 3 || cfg->mirror)
			return -EINVAL;

		info.rotation_type = OMAP_DSS_ROT_DMA;
	}

	info.max_x_decim = cfg->decim.max_x ? : 255;
	info.max_y_decim = cfg->decim.max_y ? : 255;
	info.min_x_decim = cfg->decim.min_x ? : 1;
	info.min_y_decim = cfg->decim.min_y ? : 1;
#if 0
	info.pic_height = cfg->height;

	info.field = 0;
	if (cfg->ilace & OMAP_DSS_ILACE_SEQ)
		info.field |= OMAP_FLAG_IBUF;
	if (cfg->ilace & OMAP_DSS_ILACE_SWAP)
		info.field |= OMAP_FLAG_ISWAP;
	/*
	 * Ignore OMAP_DSS_ILACE as there is no real support yet for
	 * interlaced interleaved vs progressive buffers
	 */
	if (ovl->manager &&
	    ovl->manager->device &&
	    !strcmp(ovl->manager->device->name, "hdmi") &&
	    is_hdmi_interlaced())
		info.field |= OMAP_FLAG_IDEV;

	info.out_wb = 0;
#endif

	info.cconv = cfg->cconv;

done:
#if 0
	pr_debug("ovl%d: en=%d %x/%x (%dx%d|%d) => (%dx%d) @ (%d,%d) rot=%d "
		"mir=%d col=%x z=%d al=%02x prem=%d pich=%d ilace=%d\n",
		ovl->id, info.enabled, info.paddr, info.p_uv_addr, info.width,
		info.height, info.screen_width, info.out_width, info.out_height,
		info.pos_x, info.pos_y, info.rotation, info.mirror,
		info.color_mode, info.zorder, info.global_alpha,
		info.pre_mult_alpha, info.pic_height, info.field);
#else
	pr_debug("ovl%d: en=%d %x/%x (%dx%d|%d) => (%dx%d) @ (%d,%d) rot=%d "
		"mir=%d col=%x z=%d al=%02x prem=%d\n",
		ovl->id, info.enabled, info.paddr, info.p_uv_addr, info.width,
		info.height, info.screen_width, info.out_width, info.out_height,
		info.pos_x, info.pos_y, info.rotation, info.mirror,
		info.color_mode, info.zorder, info.global_alpha,
		info.pre_mult_alpha);
#endif
	/* set overlay info */
	return ovl->set_overlay_info(ovl, &info);
}

void swap_rb_in_ovl_info(struct dss2_ovl_info *oi)
{
	/* we need to swap YUV color matrix if we are swapping R and B */
	if (oi->cfg.color_mode &
	    (OMAP_DSS_COLOR_NV12 | OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) {
		swap(oi->cfg.cconv.ry, oi->cfg.cconv.by);
		swap(oi->cfg.cconv.rcr, oi->cfg.cconv.bcr);
		swap(oi->cfg.cconv.rcb, oi->cfg.cconv.bcb);
	}
}

struct omap_overlay_manager *find_dss_mgr(int display_ix)
{
	struct omap_overlay_manager *mgr;
	char name[32];
	int i;

	sprintf(name, "display%d", display_ix);

	for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
		mgr = omap_dss_get_overlay_manager(i);
		if (mgr->device && !strcmp(name, dev_name(&mgr->device->dev)))
			return mgr;
	}
	return NULL;
}

int set_dss_mgr_info(struct dss2_mgr_info *mi, struct omapdss_ovl_cb *cb)
{
	struct omap_overlay_manager_info info;
	struct omap_overlay_manager *mgr;

	if (!mi)
		return -EINVAL;
	mgr = find_dss_mgr(mi->ix);
	if (!mgr)
		return -EINVAL;

	/* just in case there are new fields, we get the current info */
	mgr->get_manager_info(mgr, &info);

	info.alpha_enabled = mi->alpha_blending;
	info.default_color = mi->default_color;
	info.trans_enabled = mi->trans_enabled && !mi->alpha_blending;
	info.trans_key = mi->trans_key;
	info.trans_key_type = mi->trans_key_type;

	info.cpr_coefs = mi->cpr_coefs;
	info.cpr_enable = mi->cpr_enabled;
	info.cb = *cb;

	return mgr->set_manager_info(mgr, &info);
}

void swap_rb_in_mgr_info(struct dss2_mgr_info *mi)
{
	const struct omap_dss_cpr_coefs c = { 256, 0, 0, 0, 256, 0, 0, 0, 256 };

	/* set default CPR */
	if (!mi->cpr_enabled)
		mi->cpr_coefs = c;
	mi->cpr_enabled = true;

	/* swap red and blue */
	swap(mi->cpr_coefs.rr, mi->cpr_coefs.br);
	swap(mi->cpr_coefs.rg, mi->cpr_coefs.bg);
	swap(mi->cpr_coefs.rb, mi->cpr_coefs.bb);
}

/*
 * ===========================================================================
 *				DEBUG METHODS
 * ===========================================================================
 */
void dump_ovl_info(struct dsscomp_dev *cdev, struct dss2_ovl_info *oi)
{
	struct dss2_ovl_cfg *c = &oi->cfg;
	const struct color_info *ci;

	if (!(debug & DEBUG_OVERLAYS) ||
	    !(debug & DEBUG_COMPOSITIONS))
		return;

	ci = get_color_info(c->color_mode);
	if (c->zonly) {
		dev_info(DEV(cdev), "ovl%d(%s z%d)\n",
			c->ix, c->enabled ? "ON" : "off", c->zorder);
		return;
	}
	dev_info(DEV(cdev), "ovl%d(%s z%d %s%s *%d%% %d*%d:%d,%d+%d,%d rot%d%s"
						" => %d,%d+%d,%d %p/%p|%d)\n",
		c->ix, c->enabled ? "ON" : "off", c->zorder,
		ci->name ? : "(none)",
		c->pre_mult_alpha ? " premult" : "",
		(c->global_alpha * 100 + 128) / 255,
		c->width, c->height, c->crop.x, c->crop.y,
		c->crop.w, c->crop.h,
		c->rotation, c->mirror ? "+mir" : "",
		c->win.x, c->win.y, c->win.w, c->win.h,
		(void *) oi->ba, (void *) oi->uv, c->stride);
}

static void print_mgr_info(struct dsscomp_dev *cdev,
			struct dss2_mgr_info *mi)
{
	printk("(dis%d(%s) alpha=%d col=%08x ilace=%d) ",
		mi->ix,
		(mi->ix < cdev->num_displays && cdev->displays[mi->ix]) ?
		cdev->displays[mi->ix]->name : "NONE",
		mi->alpha_blending, mi->default_color,
		mi->interlaced);
}

void dump_comp_info(struct dsscomp_dev *cdev, struct dsscomp_setup_mgr_data *d,
			const char *phase)
{
	if (!(debug & DEBUG_COMPOSITIONS))
		return;

	dev_info(DEV(cdev), "[%p] %s: %c%c%c ",
		 *phase == 'q' ? (void *) d->sync_id : d, phase,
		 (d->mode & DSSCOMP_SETUP_MODE_APPLY) ? 'A' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_DISPLAY) ? 'D' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_CAPTURE) ? 'C' : '-');
	print_mgr_info(cdev, &d->mgr);
	printk("n=%d\n", d->num_ovls);
}

void dump_total_comp_info(struct dsscomp_dev *cdev,
			struct dsscomp_setup_dispc_data *d,
			const char *phase)
{
	int i;

	if (!(debug & DEBUG_COMPOSITIONS))
		return;

	dev_info(DEV(cdev), "[%p] %s: %c%c%c ",
		 *phase == 'q' ? (void *) d->sync_id : d, phase,
		 (d->mode & DSSCOMP_SETUP_MODE_APPLY) ? 'A' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_DISPLAY) ? 'D' : '-',
		 (d->mode & DSSCOMP_SETUP_MODE_CAPTURE) ? 'C' : '-');

	for (i = 0; i < d->num_mgrs && i < ARRAY_SIZE(d->mgrs); i++)
		print_mgr_info(cdev, d->mgrs + i);
	printk("n=%d\n", d->num_ovls);
}
