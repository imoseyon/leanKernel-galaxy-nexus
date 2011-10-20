/*
 * tiler-main.c
 *
 * TILER driver main support functions for TI TILER hardware block.
 *
 * Authors: Lajos Molnar <molnar@ti.com>
 *          David Sin <davidsin@ti.com>
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>			/* struct cdev */
#include <linux/kdev_t.h>		/* MKDEV() */
#include <linux/fs.h>			/* register_chrdev_region() */
#include <linux/device.h>		/* struct class */
#include <linux/platform_device.h>	/* platform_device() */
#include <linux/err.h>			/* IS_ERR() */
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>		/* dma_alloc_coherent */
#include <linux/pagemap.h>		/* page_cache_release() */
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#include <mach/dmm.h>
#include "tmm.h"
#include "_tiler.h"
#include "tcm/tcm-sita.h"		/* TCM algorithm */

static bool ssptr_id = CONFIG_TILER_SSPTR_ID;
static uint granularity = CONFIG_TILER_GRANULARITY;
static uint tiler_alloc_debug;

/*
 * We can only change ssptr_id if there are no blocks allocated, so that
 * pseudo-random ids and ssptrs do not potentially clash. For now make it
 * read-only.
 */
module_param(ssptr_id, bool, 0444);
MODULE_PARM_DESC(ssptr_id, "Use ssptr as block ID");
module_param_named(grain, granularity, uint, 0644);
MODULE_PARM_DESC(grain, "Granularity (bytes)");
module_param_named(alloc_debug, tiler_alloc_debug, uint, 0644);
MODULE_PARM_DESC(alloc_debug, "Allocation debug flag");

struct tiler_dev {
	struct cdev cdev;
};
static struct dentry *dbgfs;
static struct dentry *dbg_map;

static struct tiler_ops tiler;		/* shared methods and variables */

static struct list_head blocks;		/* all tiler blocks */
static struct list_head orphan_areas;	/* orphaned 2D areas */
static struct list_head orphan_onedim;	/* orphaned 1D areas */

static s32 tiler_major;
static s32 tiler_minor;
static struct tiler_dev *tiler_device;
static struct class *tilerdev_class;
static struct mutex mtx;
static struct tcm *tcm[TILER_FORMATS];
static struct tmm *tmm[TILER_FORMATS];
static u32 *dmac_va;
static dma_addr_t dmac_pa;
static DEFINE_MUTEX(dmac_mtx);

/*
 *  TMM connectors
 *  ==========================================================================
 */
/* wrapper around tmm_pin */
static s32 pin_mem_to_area(struct tmm *tmm, struct tcm_area *area, u32 *ptr)
{
	s32 res = 0;
	struct pat_area p_area = {0};
	struct tcm_area slice, area_s;

	/* Ensure the data reaches to main memory before PAT refill */
	wmb();

	mutex_lock(&dmac_mtx);
	tcm_for_each_slice(slice, *area, area_s) {
		p_area.x0 = slice.p0.x;
		p_area.y0 = slice.p0.y;
		p_area.x1 = slice.p1.x;
		p_area.y1 = slice.p1.y;

		memcpy(dmac_va, ptr, sizeof(*ptr) * tcm_sizeof(slice));
		ptr += tcm_sizeof(slice);

		/* pin memory into DMM */
		if (tmm_pin(tmm, p_area, dmac_pa)) {
			res = -EFAULT;
			break;
		}
	}
	mutex_unlock(&dmac_mtx);

	return res;
}

/* wrapper around tmm_unpin */
static void unpin_mem_from_area(struct tmm *tmm, struct tcm_area *area)
{
	struct pat_area p_area = {0};
	struct tcm_area slice, area_s;

	mutex_lock(&dmac_mtx);
	tcm_for_each_slice(slice, *area, area_s) {
		p_area.x0 = slice.p0.x;
		p_area.y0 = slice.p0.y;
		p_area.x1 = slice.p1.x;
		p_area.y1 = slice.p1.y;

		tmm_unpin(tmm, p_area);
	}
	mutex_unlock(&dmac_mtx);
}

/*
 *  ID handling methods
 *  ==========================================================================
 */

/* check if an id is used */
static bool _m_id_in_use(u32 id)
{
	struct mem_info *mi;
	list_for_each_entry(mi, &blocks, global)
		if (mi->blk.id == id)
			return 1;
	return 0;
}

/* get an id */
static u32 _m_get_id(void)
{
	static u32 id = 0x2d7ae;

	/* ensure noone is using this id */
	while (_m_id_in_use(id)) {
		/* generate a new pseudo-random ID */

		/* Galois LSFR: 32, 22, 2, 1 */
		id = (id >> 1) ^ (u32)((0 - (id & 1u)) & 0x80200003u);
	}

	return id;
}

/*
 *  Debugfs support
 *  ==========================================================================
 */
struct tiler_debugfs_data {
	char name[17];
	void (*func)(struct seq_file *, u32 arg);
	u32 arg;
};

static void fill_map(char **map, int xdiv, int ydiv, struct tcm_area *a,
							char c, bool ovw)
{
	int x, y;
	for (y = a->p0.y / ydiv; y <= a->p1.y / ydiv; y++)
		for (x = a->p0.x / xdiv; x <= a->p1.x / xdiv; x++)
			if (map[y][x] == ' ' || ovw)
				map[y][x] = c;
}

static void fill_map_pt(char **map, int xdiv, int ydiv, struct tcm_pt *p,
									char c)
{
	map[p->y / ydiv][p->x / xdiv] = c;
}

static char read_map_pt(char **map, int xdiv, int ydiv, struct tcm_pt *p)
{
	return map[p->y / ydiv][p->x / xdiv];
}

static int map_width(int xdiv, int x0, int x1)
{
	return (x1 / xdiv) - (x0 / xdiv) + 1;
}

static void text_map(char **map, int xdiv, char *nice, int yd, int x0, int x1)
{
	char *p = map[yd] + (x0 / xdiv);
	int w = (map_width(xdiv, x0, x1) - strlen(nice)) / 2;
	if (w >= 0) {
		p += w;
		while (*nice)
			*p++ = *nice++;
	}
}

static void map_1d_info(char **map, int xdiv, int ydiv, char *nice,
							struct tcm_area *a)
{
	sprintf(nice, "%dK", tcm_sizeof(*a) * 4);
	if (a->p0.y + 1 < a->p1.y) {
		text_map(map, xdiv, nice, (a->p0.y + a->p1.y) / 2 / ydiv, 0,
							tiler.width - 1);
	} else if (a->p0.y < a->p1.y) {
		if (strlen(nice) < map_width(xdiv, a->p0.x, tiler.width - 1))
			text_map(map, xdiv, nice, a->p0.y / ydiv,
					a->p0.x + xdiv,	tiler.width - 1);
		else if (strlen(nice) < map_width(xdiv, 0, a->p1.x))
			text_map(map, xdiv, nice, a->p1.y / ydiv,
					0, a->p1.y - xdiv);
	} else if (strlen(nice) + 1 < map_width(xdiv, a->p0.x, a->p1.x)) {
		text_map(map, xdiv, nice, a->p0.y / ydiv, a->p0.x, a->p1.x);
	}
}

static void map_2d_info(char **map, int xdiv, int ydiv, char *nice,
							struct tcm_area *a)
{
	sprintf(nice, "(%d*%d)", tcm_awidth(*a), tcm_aheight(*a));
	if (strlen(nice) + 1 < map_width(xdiv, a->p0.x, a->p1.x))
		text_map(map, xdiv, nice, (a->p0.y + a->p1.y) / 2 / ydiv,
							a->p0.x, a->p1.x);
}

static void debug_allocation_map(struct seq_file *s, u32 arg)
{
	int xdiv = (arg >> 8) & 0xFF;
	int ydiv = arg & 0xFF;
	int i;
	char **map, *global_map;
	struct area_info *ai;
	struct mem_info *mi;
	struct tcm_area a, p;
	static char *m2d = "abcdefghijklmnopqrstuvwxyz"
					"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	static char *a2d = ".,:;'\"`~!^-+";
	char *m2dp = m2d, *a2dp = a2d;
	char nice[128];

	/* allocate map */
	map = kzalloc(tiler.height / ydiv * sizeof(*map), GFP_KERNEL);
	global_map = kzalloc((tiler.width / xdiv + 1) * tiler.height / ydiv,
								GFP_KERNEL);
	if (!map || !global_map) {
		printk(KERN_ERR "could not allocate map for debug print\n");
		goto error;
	}
	memset(global_map, ' ', (tiler.width / xdiv + 1) * tiler.height / ydiv);
	for (i = 0; i < tiler.height / ydiv; i++) {
		map[i] = global_map + i * (tiler.width / xdiv + 1);
		map[i][tiler.width / xdiv] = 0;
	}

	/* get all allocations */
	mutex_lock(&mtx);

	list_for_each_entry(mi, &blocks, global) {
		if (mi->area.is2d) {
			ai = mi->parent;
			fill_map(map, xdiv, ydiv, &ai->area, *a2dp, false);
			fill_map(map, xdiv, ydiv, &mi->area, *m2dp, true);
			if (!*++a2dp)
				a2dp = a2d;
			if (!*++m2dp)
				m2dp = m2d;
			map_2d_info(map, xdiv, ydiv, nice, &mi->area);
		} else {
			bool start = read_map_pt(map, xdiv, ydiv, &mi->area.p0)
									== ' ';
			bool end = read_map_pt(map, xdiv, ydiv, &mi->area.p1)
									== ' ';
			tcm_for_each_slice(a, mi->area, p)
				fill_map(map, xdiv, ydiv, &a, '=', true);
			fill_map_pt(map, xdiv, ydiv, &mi->area.p0,
							start ? '<' : 'X');
			fill_map_pt(map, xdiv, ydiv, &mi->area.p1,
							end ? '>' : 'X');
			map_1d_info(map, xdiv, ydiv, nice, &mi->area);
		}
	}

	seq_printf(s, "BEGIN TILER MAP\n");
	for (i = 0; i < tiler.height / ydiv; i++)
		seq_printf(s, "%03d:%s\n", i * ydiv, map[i]);
	seq_printf(s, "END TILER MAP\n");

	mutex_unlock(&mtx);

error:
	kfree(map);
	kfree(global_map);
}

const struct tiler_debugfs_data debugfs_maps[] = {
	{ "1x1", debug_allocation_map, 0x0101 },
	{ "2x1", debug_allocation_map, 0x0201 },
	{ "4x1", debug_allocation_map, 0x0401 },
	{ "2x2", debug_allocation_map, 0x0202 },
	{ "4x2", debug_allocation_map, 0x0402 },
	{ "4x4", debug_allocation_map, 0x0404 },
};

static int tiler_debug_show(struct seq_file *s, void *unused)
{
	struct tiler_debugfs_data *fn = s->private;
	fn->func(s, fn->arg);
	return 0;
}

static int tiler_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tiler_debug_show, inode->i_private);
}

static const struct file_operations tiler_debug_fops = {
	.open           = tiler_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*
 *  gid_info handling methods
 *  ==========================================================================
 */

/* get or create new gid_info object */
static struct gid_info *_m_get_gi(struct process_info *pi, u32 gid)
{
	struct gid_info *gi;

	/* have mutex */

	/* see if group already exist */
	list_for_each_entry(gi, &pi->groups, by_pid) {
		if (gi->gid == gid)
			goto done;
	}

	/* create new group */
	gi = kmalloc(sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return gi;

	memset(gi, 0, sizeof(*gi));
	INIT_LIST_HEAD(&gi->areas);
	INIT_LIST_HEAD(&gi->onedim);
	INIT_LIST_HEAD(&gi->reserved);
	gi->pi = pi;
	gi->gid = gid;
	list_add(&gi->by_pid, &pi->groups);
done:
	/*
	 * Once area is allocated, the group info's ref count will be
	 * decremented as the reference is no longer needed.
	 */
	gi->refs++;
	return gi;
}

/* free gid_info object if empty */
static void _m_try_free_group(struct gid_info *gi)
{
	/* have mutex */
	if (gi && list_empty(&gi->areas) && list_empty(&gi->onedim) &&
	    /* also ensure noone is still using this group */
	    !gi->refs) {
		BUG_ON(!list_empty(&gi->reserved));
		list_del(&gi->by_pid);

		/* if group is tracking kernel objects, we may free even
		   the process info */
		if (gi->pi->kernel && list_empty(&gi->pi->groups)) {
			list_del(&gi->pi->list);
			kfree(gi->pi);
		}

		kfree(gi);
	}
}

/* --- external versions --- */

static struct gid_info *get_gi(struct process_info *pi, u32 gid)
{
	struct gid_info *gi;
	mutex_lock(&mtx);
	gi = _m_get_gi(pi, gid);
	mutex_unlock(&mtx);
	return gi;
}

static void release_gi(struct gid_info *gi)
{
	mutex_lock(&mtx);
	gi->refs--;
	_m_try_free_group(gi);
	mutex_unlock(&mtx);
}

/*
 *  Area handling methods
 *  ==========================================================================
 */

/* allocate an reserved area of size, alignment and link it to gi */
/* leaves mutex locked to be able to add block to area */
static struct area_info *area_new_m(u16 width, u16 height, u16 align,
				  struct tcm *tcm, struct gid_info *gi)
{
	struct area_info *ai = kmalloc(sizeof(*ai), GFP_KERNEL);
	if (!ai)
		return NULL;

	/* set up empty area info */
	memset(ai, 0x0, sizeof(*ai));
	INIT_LIST_HEAD(&ai->blocks);

	/* reserve an allocation area */
	if (tcm_reserve_2d(tcm, width, height, align, &ai->area)) {
		kfree(ai);
		return NULL;
	}

	ai->gi = gi;
	mutex_lock(&mtx);
	list_add_tail(&ai->by_gid, &gi->areas);
	return ai;
}

/* (must have mutex) free an area */
static inline void _m_area_free(struct area_info *ai)
{
	if (ai) {
		list_del(&ai->by_gid);
		kfree(ai);
	}
}

static s32 __analize_area(enum tiler_fmt fmt, u32 width, u32 height,
			  u16 *x_area, u16 *y_area, u16 *band,
			  u16 *align)
{
	/* input: width, height is in pixels */
	/* output: x_area, y_area, band, align */

	/* slot width, height, and row size */
	u32 slot_row, min_align;
	const struct tiler_geom *g;

	/* set alignment to page size */
	*align = PAGE_SIZE;

	/* width and height must be positive */
	if (!width || !height)
		return -EINVAL;

	if (fmt == TILFMT_PAGE) {
		/* for 1D area keep the height (1), width is in tiler slots */
		*x_area = DIV_ROUND_UP(width, tiler.page);
		*y_area = *band = 1;

		if (*x_area * *y_area > tiler.width * tiler.height)
			return -ENOMEM;
		return 0;
	}

	/* format must be valid */
	g = tiler.geom(fmt);
	if (!g)
		return -EINVAL;

	/* get the # of bytes per row in 1 slot */
	slot_row = g->slot_w * g->bpp;

	/* how many slots are can be accessed via one physical page */
	*band = PAGE_SIZE / slot_row;

	/* minimum alignment is at least 1 slot */
	min_align = max(slot_row, granularity);
	*align = ALIGN(*align, min_align);

	/* adjust to slots */
	*x_area = DIV_ROUND_UP(width, g->slot_w);
	*y_area = DIV_ROUND_UP(height, g->slot_h);
	*align /= slot_row;

	if (*x_area > tiler.width || *y_area > tiler.height)
		return -ENOMEM;
	return 0;
}

void fill_virt_array(struct tiler_block_t *blk, u32 *virt_array)
{
	u32 v, p, len, size;
	u32 i = 0, offs = 0;

	if (!virt_array)
		return;

	/* get page aligned stride */
	v = tiler_vstride(blk);
	p = tiler_pstride(blk);

	/* get page aligned virtual size for the block */
	size = tiler_size(blk);
	offs = blk->phys;
	while (size) {
		/* set len to length of one row (2D), or full length if 1D */
		len = v;

		while (len && size) {
			virt_array[i++] = PAGE_ALIGN(offs);
			size -= PAGE_SIZE;
			len -= PAGE_SIZE;
			offs += PAGE_SIZE;
		}

		/* set offset to next row beginning */
		offs += p - v;
	}
}

/**
 * Find a place where a 2D block would fit into a 2D area of the
 * same height.
 *
 * @author a0194118 (3/19/2010)
 *
 * @param w	Width of the block.
 * @param align	Alignment of the block.
 * @param ai	Pointer to area info
 * @param next	Pointer to the variable where the next block
 *		will be stored.  The block should be inserted
 *		before this block.
 *
 * @return the end coordinate (x1 + 1) where a block would fit,
 *	   or 0 if it does not fit.
 *
 * (must have mutex)
 */
static u16 _m_blk_find_fit(u16 w, u16 align,
		     struct area_info *ai, struct list_head **before)
{
	int x = ai->area.p0.x + w;
	struct mem_info *mi;

	/* area blocks are sorted by x */
	list_for_each_entry(mi, &ai->blocks, by_area) {
		/* check if buffer would fit before this area */
		if (x <= mi->area.p0.x) {
			*before = &mi->by_area;
			return x;
		}
		x = ALIGN(mi->area.p1.x + 1, align) + w;
	}
	*before = &ai->blocks;

	/* check if buffer would fit after last area */
	return (x <= ai->area.p1.x + 1) ? x : 0;
}

/* (must have mutex) adds a block to an area with certain x coordinates */
static inline
struct mem_info *_m_add2area(struct mem_info *mi, struct area_info *ai,
				u16 x0, u16 w, struct list_head *before)
{
	mi->parent = ai;
	mi->area = ai->area;
	mi->area.p0.x = x0;
	mi->area.p1.x = x0 + w - 1;
	list_add_tail(&mi->by_area, before);
	ai->nblocks++;
	return mi;
}

static struct mem_info *get_2d_area(u16 w, u16 h, u16 align, u16 band,
					struct gid_info *gi, struct tcm *tcm)
{
	struct area_info *ai = NULL;
	struct mem_info *mi = NULL;
	struct list_head *before = NULL;
	u16 x = 0;   /* this holds the end of a potential area */

	/* allocate map info */

	/* see if there is available prereserved space */
	mutex_lock(&mtx);
	list_for_each_entry(mi, &gi->reserved, global) {
		if (mi->area.tcm == tcm &&
		    tcm_aheight(mi->area) == h &&
		    tcm_awidth(mi->area) == w &&
		    (mi->area.p0.x & (align - 1)) == 0) {
			/* this area is already set up */

			/* remove from reserved list */
			list_del(&mi->global);
			if (tiler_alloc_debug & 1)
				 printk(KERN_ERR "(=2d (%d-%d,%d-%d) in (%d-%d,%d-%d) prereserved)\n",
				 mi->area.p0.x, mi->area.p1.x,
				 mi->area.p0.y, mi->area.p1.y,
				 ((struct area_info *) mi->parent)->area.p0.x,
				 ((struct area_info *) mi->parent)->area.p1.x,
				 ((struct area_info *) mi->parent)->area.p0.y,
				 ((struct area_info *) mi->parent)->area.p1.y);

			goto done;
		}
	}
	mutex_unlock(&mtx);

	/* if not, reserve a block struct */
	mi = kmalloc(sizeof(*mi), GFP_KERNEL);
	if (!mi)
		return mi;
	memset(mi, 0, sizeof(*mi));

	/* see if allocation fits in one of the existing areas */
	/* this sets x, ai and before */
	mutex_lock(&mtx);
	list_for_each_entry(ai, &gi->areas, by_gid) {
		if (ai->area.tcm == tcm &&
		    tcm_aheight(ai->area) == h) {
			x = _m_blk_find_fit(w, align, ai, &before);
			if (x) {
				_m_add2area(mi, ai, x - w, w, before);

			if (tiler_alloc_debug & 1)
				printk(KERN_ERR "(+2d (%d-%d,%d-%d) in (%d-%d,%d-%d) existing)\n",
				mi->area.p0.x, mi->area.p1.x,
				mi->area.p0.y, mi->area.p1.y,
				((struct area_info *) mi->parent)->area.p0.x,
				((struct area_info *) mi->parent)->area.p1.x,
				((struct area_info *) mi->parent)->area.p0.y,
				((struct area_info *) mi->parent)->area.p1.y);

			goto done;
			}
		}
	}
	mutex_unlock(&mtx);

	/* if no area fit, reserve a new one */
	ai = area_new_m(ALIGN(w, max(band, align)), h,
		      max(band, align), tcm, gi);
	if (ai) {
		_m_add2area(mi, ai, ai->area.p0.x, w, &ai->blocks);
		if (tiler_alloc_debug & 1)
			printk(KERN_ERR "(+2d (%d-%d,%d-%d) in (%d-%d,%d-%d) new)\n",
					mi->area.p0.x, mi->area.p1.x,
					mi->area.p0.y, mi->area.p1.y,
					ai->area.p0.x, ai->area.p1.x,
					ai->area.p0.y, ai->area.p1.y);
	} else {
		/* clean up */
		kfree(mi);
		return NULL;
	}

done:
	mutex_unlock(&mtx);
	return mi;
}

/* layout reserved 2d blocks in a larger area */
/* NOTE: band, w, h, a(lign) is in slots */
static s32 lay_2d(enum tiler_fmt fmt, u16 n, u16 w, u16 h, u16 band,
		      u16 align, struct gid_info *gi,
		      struct list_head *pos)
{
	u16 x, x0, e = ALIGN(w, align), w_res = (n - 1) * e + w;
	struct mem_info *mi = NULL;
	struct area_info *ai = NULL;

	printk(KERN_INFO "packing %u %u buffers into %u width\n",
	       n, w, w_res);

	/* calculate dimensions, band, and alignment in slots */
	/* reserve an area */
	ai = area_new_m(ALIGN(w_res, max(band, align)), h,
			max(band, align), tcm[fmt], gi);
	if (!ai)
		return -ENOMEM;

	/* lay out blocks in the reserved area */
	for (n = 0, x = 0; x < w_res; x += e, n++) {
		/* reserve a block struct */
		mi = kmalloc(sizeof(*mi), GFP_KERNEL);
		if (!mi)
			break;

		memset(mi, 0, sizeof(*mi));
		x0 = ai->area.p0.x + x;
		_m_add2area(mi, ai, x0, w, &ai->blocks);
		list_add(&mi->global, pos);
	}

	mutex_unlock(&mtx);
	return n;
}

#ifdef CONFIG_TILER_ENABLE_NV12
/* layout reserved nv12 blocks in a larger area */
/* NOTE: area w(idth), w1 (8-bit block width), h(eight) are in slots */
/* p is a pointer to a packing description, which is a list of offsets in
   the area for consecutive 8-bit and 16-bit blocks */
static s32 lay_nv12(int n, u16 w, u16 w1, u16 h, struct gid_info *gi, u8 *p)
{
	u16 wh = (w1 + 1) >> 1, width, x0;
	int m;
	int a = PAGE_SIZE / tiler.geom(TILFMT_8BIT)->slot_w;

	struct mem_info *mi = NULL;
	struct area_info *ai = NULL;
	struct list_head *pos;

	/* reserve area */
	ai = area_new_m(w, h, a, TILFMT_8BIT, gi);
	if (!ai)
		return -ENOMEM;

	/* lay out blocks in the reserved area */
	for (m = 0; m < 2 * n; m++) {
		width =	(m & 1) ? wh : w1;
		x0 = ai->area.p0.x + *p++;

		/* get insertion head */
		list_for_each(pos, &ai->blocks) {
			mi = list_entry(pos, struct mem_info, by_area);
			if (mi->area.p0.x > x0)
				break;
		}

		/* reserve a block struct */
		mi = kmalloc(sizeof(*mi), GFP_KERNEL);
		if (!mi)
			break;

		memset(mi, 0, sizeof(*mi));

		_m_add2area(mi, ai, x0, width, pos);
		list_add(&mi->global, &gi->reserved);
	}

	mutex_unlock(&mtx);
	return n;
}
#endif

static void _m_unpin(struct mem_info *mi)
{
	/* release memory */
	if (mi->pa.memtype == TILER_MEM_GOT_PAGES) {
		int i;
		for (i = 0; i < mi->pa.num_pg; i++) {
			struct page *page = phys_to_page(mi->pa.mem[i]);
			if (page) {
				if (!PageReserved(page))
					SetPageDirty(page);
				page_cache_release(page);
			}
		}
	} else if (mi->pa.memtype == TILER_MEM_ALLOCED && mi->pa.mem) {
		tmm_free(tmm[tiler_fmt(mi->blk.phys)], mi->pa.mem);
		/*
		 * TRICKY: tmm module uses the same mi->pa.mem pointer which
		 * it just freed.  We need to clear ours so we don't double free
		 */
		mi->pa.mem = NULL;
	}
	kfree(mi->pa.mem);
	mi->pa.mem = NULL;
	mi->pa.num_pg = 0;
	unpin_mem_from_area(tmm[tiler_fmt(mi->blk.phys)], &mi->area);
}

/* (must have mutex) free block and any freed areas */
static s32 _m_free(struct mem_info *mi)
{
	struct area_info *ai = NULL;
	s32 res = 0;

	_m_unpin(mi);

	/* safe deletion as list may not have been assigned */
	if (mi->global.next)
		list_del(&mi->global);
	if (mi->by_area.next)
		list_del(&mi->by_area);

	/* remove block from area first if 2D */
	if (mi->area.is2d) {
		ai = mi->parent;

		/* check to see if area needs removing also */
		if (ai && !--ai->nblocks) {
				if (tiler_alloc_debug & 1)
					printk(KERN_ERR "(-2d (%d-%d,%d-%d) in (%d-%d,%d-%d) last)\n",
							   mi->area.p0.x, mi->area.p1.x,
							   mi->area.p0.y, mi->area.p1.y,
							   ai->area.p0.x, ai->area.p1.x,
							   ai->area.p0.y, ai->area.p1.y);

			res = tcm_free(&ai->area);
			list_del(&ai->by_gid);
			/* try to remove parent if it became empty */
			_m_try_free_group(ai->gi);
			kfree(ai);
			ai = NULL;
		} else if (tiler_alloc_debug & 1)
					printk(KERN_ERR "(-2d (%d-%d,%d-%d) in (%d-%d,%d-%d) remaining)\n",
						mi->area.p0.x, mi->area.p1.x,
						mi->area.p0.y, mi->area.p1.y,
						ai->area.p0.x, ai->area.p1.x,
						ai->area.p0.y, ai->area.p1.y);

	} else {
		if (tiler_alloc_debug & 1)
			printk(KERN_ERR "(-1d: %d,%d..%d,%d)\n",
			mi->area.p0.x, mi->area.p0.y,
			mi->area.p1.x, mi->area.p1.y);
		/* remove 1D area */
		res = tcm_free(&mi->area);
		/* try to remove parent if it became empty */
		_m_try_free_group(mi->parent);
	}

	kfree(mi);
	return res;
}

/* (must have mutex) returns true if block was freed */
static bool _m_chk_ref(struct mem_info *mi)
{
	/* check references */
	if (mi->refs)
		return 0;

	if (_m_free(mi))
		printk(KERN_ERR "error while removing tiler block\n");

	return 1;
}

/* (must have mutex) */
static inline bool _m_dec_ref(struct mem_info *mi)
{
	if (mi->refs-- <= 1)
		return _m_chk_ref(mi);

	return 0;
}

/* (must have mutex) */
static inline void _m_inc_ref(struct mem_info *mi)
{
	mi->refs++;
}

/* (must have mutex) returns true if block was freed */
static inline bool _m_try_free(struct mem_info *mi)
{
	if (mi->alloced) {
		mi->refs--;
		mi->alloced = false;
	}
	return _m_chk_ref(mi);
}

/* --- external methods --- */

/* find a block by key/id and lock it */
static struct mem_info *
find_n_lock(u32 key, u32 id, struct gid_info *gi) {
	struct area_info *ai = NULL;
	struct mem_info *mi = NULL;

	mutex_lock(&mtx);

	/* if group is not given, look globally */
	if (!gi) {
		list_for_each_entry(mi, &blocks, global) {
			if (mi->blk.key == key && mi->blk.id == id)
				goto done;
		}
	} else {
		/* is id is ssptr, we know if block is 1D or 2D by the address,
		   so we optimize lookup */
		if (!ssptr_id ||
		    tiler_fmt(id) == TILFMT_PAGE) {
			list_for_each_entry(mi, &gi->onedim, by_area) {
				if (mi->blk.key == key && mi->blk.id == id)
					goto done;
			}
		}

		if (!ssptr_id ||
		    tiler_fmt(id) != TILFMT_PAGE) {
			list_for_each_entry(ai, &gi->areas, by_gid) {
				list_for_each_entry(mi, &ai->blocks, by_area) {
					if (mi->blk.key == key &&
					    mi->blk.id == id)
						goto done;
				}
			}
		}
	}

	mi = NULL;
done:
	/* lock block by increasing its ref count */
	if (mi)
		mi->refs++;

	mutex_unlock(&mtx);

	return mi;
}

/* unlock a block, and optionally free it */
static void unlock_n_free(struct mem_info *mi, bool free)
{
	mutex_lock(&mtx);

	_m_dec_ref(mi);
	if (free)
		_m_try_free(mi);

	mutex_unlock(&mtx);
}

/**
 * Free all blocks in a group:
 *
 * allocated blocks, and unreferenced blocks.  Any blocks/areas still referenced
 * will move to the orphaned lists to avoid issues if a new process is created
 * with the same pid.
 *
 * (must have mutex)
 */
static void destroy_group(struct gid_info *gi)
{
	struct area_info *ai, *ai_;
	struct mem_info *mi, *mi_;
	bool ai_autofreed, need2free;

	mutex_lock(&mtx);

	/* free all allocated blocks, and remove unreferenced ones */

	/*
	 * Group info structs when they become empty on an _m_try_free.
	 * However, if the group info is already empty, we need to
	 * remove it manually
	 */
	need2free = list_empty(&gi->areas) && list_empty(&gi->onedim);
	list_for_each_entry_safe(ai, ai_, &gi->areas, by_gid) {
		ai_autofreed = true;
		list_for_each_entry_safe(mi, mi_, &ai->blocks, by_area)
			ai_autofreed &= _m_try_free(mi);

		/* save orphaned areas for later removal */
		if (!ai_autofreed) {
			need2free = true;
			ai->gi = NULL;
			list_move(&ai->by_gid, &orphan_areas);
		}
	}

	list_for_each_entry_safe(mi, mi_, &gi->onedim, by_area) {
		if (!_m_try_free(mi)) {
			need2free = true;
			/* save orphaned 1D blocks */
			mi->parent = NULL;
			list_move(&mi->by_area, &orphan_onedim);
		}
	}

	/* if group is still alive reserved list should have been
	   emptied as there should be no reference on those blocks */
	if (need2free) {
		BUG_ON(!list_empty(&gi->onedim));
		BUG_ON(!list_empty(&gi->areas));
		_m_try_free_group(gi);
	}

	mutex_unlock(&mtx);
}

/* release (reserved) blocks */
static void release_blocks(struct list_head *reserved)
{
	struct mem_info *mi, *mi_;

	mutex_lock(&mtx);

	/* find block in global list and free it */
	list_for_each_entry_safe(mi, mi_, reserved, global) {
		BUG_ON(mi->refs || mi->alloced);
		_m_free(mi);
	}
	mutex_unlock(&mtx);
}

/* add reserved blocks to a group */
static void add_reserved_blocks(struct list_head *reserved, struct gid_info *gi)
{
	mutex_lock(&mtx);
	list_splice_init(reserved, &gi->reserved);
	mutex_unlock(&mtx);
}

/* find a block by ssptr */
static struct mem_info *find_block_by_ssptr(u32 sys_addr)
{
	struct mem_info *i;
	struct tcm_pt pt;
	u32 x, y;
	enum tiler_fmt fmt;
	const struct tiler_geom *g;

	fmt = tiler_fmt(sys_addr);
	if (fmt == TILFMT_INVALID)
		return NULL;

	g = tiler.geom(fmt);

	/* convert x & y pixel coordinates to slot coordinates */
	tiler.xy(sys_addr, &x, &y);
	pt.x = x / g->slot_w;
	pt.y = y / g->slot_h;

	mutex_lock(&mtx);
	list_for_each_entry(i, &blocks, global) {
		if (tiler_fmt(i->blk.phys) == tiler_fmt(sys_addr) &&
		    tcm_is_in(pt, i->area)) {
			i->refs++;
			goto found;
		}
	}
	i = NULL;

found:
	mutex_unlock(&mtx);
	return i;
}

/* find a block by ssptr */
static void fill_block_info(struct mem_info *i, struct tiler_block_info *blk)
{
	blk->fmt = tiler_fmt(i->blk.phys);
#ifdef CONFIG_TILER_EXPOSE_SSPTR
	blk->ssptr = i->blk.phys;
#endif
	if (blk->fmt == TILFMT_PAGE) {
		blk->dim.len = i->blk.width;
		blk->group_id = ((struct gid_info *) i->parent)->gid;
	} else {
		blk->stride = tiler_vstride(&i->blk);
		blk->dim.area.width = i->blk.width;
		blk->dim.area.height = i->blk.height;
		blk->group_id = ((struct area_info *) i->parent)->gi->gid;
	}
	blk->id = i->blk.id;
	blk->key = i->blk.key;
}

/*
 *  Block operations
 *  ==========================================================================
 */
static struct mem_info *alloc_area(enum tiler_fmt fmt, u32 width, u32 height,
				   struct gid_info *gi)
{
	u16 x, y, band, align;
	struct mem_info *mi = NULL;
	const struct tiler_geom *g = tiler.geom(fmt);

	/* calculate dimensions, band, and alignment in slots */
	if (__analize_area(fmt, width, height, &x, &y, &band, &align))
		return NULL;

	if (fmt == TILFMT_PAGE)	{
		/* 1D areas don't pack */
		mi = kmalloc(sizeof(*mi), GFP_KERNEL);
		if (!mi)
			return NULL;
		memset(mi, 0x0, sizeof(*mi));

		if (tcm_reserve_1d(tcm[fmt], x * y, &mi->area)) {
			kfree(mi);
			return NULL;
		}

		if (tiler_alloc_debug & 1)
			 printk(KERN_ERR "(+1d: %d,%d..%d,%d)\n",
						mi->area.p0.x, mi->area.p0.y,
						mi->area.p1.x, mi->area.p1.y);

		mutex_lock(&mtx);
		mi->parent = gi;
		list_add(&mi->by_area, &gi->onedim);
	} else {
		mi = get_2d_area(x, y, align, band, gi, tcm[fmt]);
		if (!mi)
			return NULL;

		mutex_lock(&mtx);
	}

	list_add(&mi->global, &blocks);
	mi->alloced = true;
	mi->refs++;
	gi->refs--;
	mutex_unlock(&mtx);

	mi->blk.phys = tiler.addr(fmt,
		mi->area.p0.x * g->slot_w, mi->area.p0.y * g->slot_h);
	return mi;
}

static struct mem_info *alloc_block_area(enum tiler_fmt fmt, u32 width,
		u32 height, u32 key, u32 gid,
		struct process_info *pi)
{
	struct mem_info *mi = NULL;
	struct gid_info *gi = NULL;

	/* validate parameters */
	if (!pi)
		return ERR_PTR(-EINVAL);

	/* get group context */
	mutex_lock(&mtx);
	gi = _m_get_gi(pi, gid);
	mutex_unlock(&mtx);

	if (!gi)
		return ERR_PTR(-ENOMEM);

	/* reserve area in tiler container */
	mi = alloc_area(fmt, width, height, gi);
	if (!mi) {
		mutex_lock(&mtx);
		gi->refs--;
		_m_try_free_group(gi);
		mutex_unlock(&mtx);
		return ERR_PTR(-ENOMEM);
	}

	mi->blk.width = width;
	mi->blk.height = height;
	mi->blk.key = key;
	if (ssptr_id) {
		mi->blk.id = mi->blk.phys;
	} else {
		mutex_lock(&mtx);
		mi->blk.id = _m_get_id();
		mutex_unlock(&mtx);
	}

	return mi;
}

static s32 pin_memory(struct mem_info *mi, struct tiler_pa_info *pa)
{
	enum tiler_fmt fmt = tiler_fmt(mi->blk.phys);
	struct tcm_area area = mi->area;

	/* ensure we can pin */
	if (!tmm_can_pin(tmm[fmt]))
		return -EINVAL;

	/* ensure pages fit into area */
	if (pa->num_pg > tcm_sizeof(mi->area))
		return -ENOMEM;

	/* for 2D area, pages must fit exactly */
	if (fmt != TILFMT_PAGE &&
	    pa->num_pg != tcm_sizeof(mi->area))
		return -EINVAL;

	/* save pages used */
	mi->pa = *pa;
	pa->mem = NULL;	/* transfered array */

	/* only refill available pages for 1D */
	if (fmt == TILFMT_PAGE)
		tcm_1d_limit(&area, pa->num_pg);
	if (mi->pa.num_pg)
		return pin_mem_to_area(tmm[fmt], &area, mi->pa.mem);
	return 0;
}

void tiler_pa_free(struct tiler_pa_info *pa)
{
	if (pa)
		kfree(pa->mem);
	kfree(pa);
}
EXPORT_SYMBOL(tiler_pa_free);

/* allocate physical pages for a block */
static struct tiler_pa_info *get_new_pa(struct tmm *tmm, u32 num_pg)
{
	struct tiler_pa_info *pa = NULL;
	pa = kzalloc(sizeof(*pa), GFP_KERNEL);
	if (!pa)
		return NULL;

	pa->mem = tmm_get(tmm, num_pg);
	if (pa->mem) {
		pa->num_pg = num_pg;
		pa->memtype = TILER_MEM_ALLOCED;
		return pa;
	} else {
		kfree(pa);
		return NULL;
	}
}

static s32 alloc_block(enum tiler_fmt fmt, u32 width, u32 height,
		u32 key, u32 gid, struct process_info *pi,
		struct mem_info **info)
{
	struct mem_info *mi;
	struct tiler_pa_info *pa = NULL;
	int res;

	*info = NULL;

	/* allocate tiler container area */
	mi = alloc_block_area(fmt, width, height, key, gid, pi);
	if (IS_ERR_OR_NULL(mi))
		return mi ? -ENOMEM : PTR_ERR(mi);

	/* allocate memory */
	pa = get_new_pa(tmm[fmt], tcm_sizeof(mi->area));
	if (IS_ERR_OR_NULL(pa)) {
		res = -ENOMEM;
		goto cleanup;
	}

	/* pin memory */
	res = pin_memory(mi, pa);
	tiler_pa_free(pa);
	if (res)
		goto cleanup;

	*info = mi;
	return 0;

cleanup:
	mutex_lock(&mtx);
	_m_free(mi);
	mutex_unlock(&mtx);
	return res;
}


/* get physical pages of a user block */
struct tiler_pa_info *user_block_to_pa(u32 usr_addr, u32 num_pg)
{
	struct task_struct *curr_task = current;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma = NULL;

	struct tiler_pa_info *pa = NULL;
	struct page **pages = NULL;
	u32 *mem = NULL, write, i;
	int usr_count;

	pa = kzalloc(sizeof(*pa), GFP_KERNEL);
	if (!pa)
		return NULL;

	mem = kzalloc(num_pg * sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		kfree(pa);
		return NULL;
	}

	pages = kmalloc(num_pg * sizeof(*pages), GFP_KERNEL);
	if (!pages) {
		kfree(mem);
		kfree(pa);
		return NULL;
	}

	/*
	 * Important Note: usr_addr is mapped from user
	 * application process to current process - it must lie
	 * completely within the current virtual memory address
	 * space in order to be of use to us here.
	 */
	down_read(&mm->mmap_sem);
	vma = find_vma(mm, usr_addr + (num_pg << PAGE_SHIFT));

	if (!vma || (usr_addr < vma->vm_start)) {
		kfree(mem);
		kfree(pa);
		kfree(pages);
		up_read(&mm->mmap_sem);
		printk(KERN_ERR "Address is outside VMA: address start = %08x, "
			"user end = %08x\n",
			usr_addr, (usr_addr + (num_pg << PAGE_SHIFT)));
		return ERR_PTR(-EFAULT);
	}

	if (vma->vm_flags & (VM_WRITE | VM_MAYWRITE))
		write = 1;

	usr_count = get_user_pages(curr_task, mm, usr_addr, num_pg, write, 1,
					pages, NULL);

	if (usr_count > 0) {
		/* process user allocated buffer */
		if (usr_count != num_pg) {
			/* release the pages we did get */
			for (i = 0; i < usr_count; i++)
				page_cache_release(pages[i]);
		} else {
			/* fill in the physical address information */
			for (i = 0; i < num_pg; i++) {
				mem[i] = page_to_phys(pages[i]);
				BUG_ON(pages[i] != phys_to_page(mem[i]));
			}
		}
	} else {
		/* fallback for kernel allocated buffers */
		for (i = 0; i < num_pg; i++) {
			mem[i] = tiler_virt2phys(usr_addr);

			if (!mem[i]) {
				printk(KERN_ERR "VMA not in page table\n");
				break;
			}

			usr_addr += PAGE_SIZE;
		}
	}

	up_read(&mm->mmap_sem);

	kfree(pages);

	/* if failed to map all pages */
	if (i < num_pg) {
		kfree(mem);
		kfree(pa);
		return ERR_PTR(-EFAULT);
	}

	pa->mem = mem;
	pa->memtype = usr_count > 0 ? TILER_MEM_GOT_PAGES : TILER_MEM_USING;
	pa->num_pg = num_pg;
	return pa;
}
EXPORT_SYMBOL(user_block_to_pa);

/* allocate area from container and pin memory */
static s32 pin_any_block(enum tiler_fmt fmt, u32 width, u32 height,
		     u32 key, u32 gid, struct process_info *pi,
		     struct mem_info **info, struct tiler_pa_info *pa)
{
	s32 res = -EPERM;
	struct mem_info *mi = NULL;

	*info = NULL;

	/* check if mapping is supported by tmm */
	if (!tmm_can_pin(tmm[fmt]))
		goto done;

	/* get allocation area */
	mi = alloc_block_area(fmt, width, height, key, gid, pi);
	if (IS_ERR_OR_NULL(mi)) {
		res = mi ? PTR_ERR(mi) : -ENOMEM;
		goto done;
	}

	/* pin pages to tiler container */
	res = pin_memory(mi, pa);

	/* success */
	if (!res) {
		*info = mi;
	} else {
		mutex_lock(&mtx);
		_m_free(mi);
		mutex_unlock(&mtx);
	}
done:
	tiler_pa_free(pa);
	return res;
}

static s32 pin_block(enum tiler_fmt fmt, u32 width, u32 height,
		     u32 key, u32 gid, struct process_info *pi,
		     struct mem_info **info, u32 usr_addr)
{
	struct tiler_pa_info *pa = NULL;

	/* we only support mapping a user buffer in page mode */
	if (fmt != TILFMT_PAGE)
		return -ENOMEM;

	/* get user pages */
	pa = user_block_to_pa(usr_addr, DIV_ROUND_UP(width, PAGE_SIZE));
	if (IS_ERR_OR_NULL(pa))
		return pa ? PTR_ERR(pa) : -ENOMEM;

	return pin_any_block(fmt, width, height, key, gid, pi, info, pa);
}

s32 tiler_pin_block(tiler_blk_handle block, u32 *addr_array, u32 nents)
{
	struct tiler_pa_info *pa = NULL;
	u32 *mem = NULL;
	int res;

	pa = kzalloc(sizeof(*pa), GFP_KERNEL);
	if (!pa)
		return -ENOMEM;

	mem = kmemdup(addr_array, sizeof(*addr_array)*nents, GFP_KERNEL);
	if (!mem) {
		kfree(pa);
		return -ENOMEM;
	}

	pa->mem = mem;
	pa->memtype =  TILER_MEM_USING;
	pa->num_pg = nents;

	res = pin_memory(block, pa);
	tiler_pa_free(pa);

	return res;
}
EXPORT_SYMBOL(tiler_pin_block);

/*
 *  Driver code
 *  ==========================================================================
 */

#ifdef CONFIG_PM
static int tiler_resume(struct device *pdev)
{
	struct mem_info *mi;
	struct pat_area area = {0};

	/* clear out PAT entries and set dummy page */
	area.x1 = tiler.width - 1;
	area.y1 = tiler.height - 1;
	mutex_lock(&dmac_mtx);
	tmm_unpin(tmm[TILFMT_8BIT], area);
	mutex_unlock(&dmac_mtx);

	/* iterate over all the blocks and refresh the PAT entries */
	list_for_each_entry(mi, &blocks, global) {
		if (mi->pa.mem)
			if (pin_mem_to_area(tmm[tiler_fmt(mi->blk.phys)],
						&mi->area, mi->pa.mem))
				printk(KERN_ERR "Failed PAT restore - %08x\n",
					mi->blk.phys);
	}

	return 0;
}

static const struct dev_pm_ops tiler_pm_ops = {
	.resume = tiler_resume,
};
#endif

static struct platform_driver tiler_driver_ldm = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tiler",
#ifdef CONFIG_PM
		.pm = &tiler_pm_ops,
#endif
	},
};

static s32 __init tiler_init(void)
{
	dev_t dev  = 0;
	s32 r = -1;
	struct device *device = NULL;
	struct tcm_pt div_pt;
	struct tcm *sita = NULL;
	struct tmm *tmm_pat = NULL;
	struct pat_area area = {0};

	tiler.alloc = alloc_block;
	tiler.pin = pin_block;
	tiler.lock = find_n_lock;
	tiler.unlock_free = unlock_n_free;
	tiler.lay_2d = lay_2d;
#ifdef CONFIG_TILER_ENABLE_NV12
	tiler.lay_nv12 = lay_nv12;
#endif
	tiler.destroy_group = destroy_group;
	tiler.lock_by_ssptr = find_block_by_ssptr;
	tiler.describe = fill_block_info;
	tiler.get_gi = get_gi;
	tiler.release_gi = release_gi;
	tiler.release = release_blocks;
	tiler.add_reserved = add_reserved_blocks;
	tiler.analize = __analize_area;
	tiler_geom_init(&tiler);
	tiler_reserve_init(&tiler);

	mutex_init(&tiler.mtx);
	tiler_iface_init(&tiler);
#ifdef CONFIG_TILER_ENABLE_USERSPACE
	tiler_ioctl_init(&tiler);
#endif
#ifdef CONFIG_TILER_ENABLE_NV12
	tiler_nv12_init(&tiler);
#endif

	/* check module parameters for correctness */
	if (granularity < 1 || granularity > PAGE_SIZE ||
	    granularity & (granularity - 1))
		return -EINVAL;

	/*
	 * Array of physical pages for PAT programming, which must be a 16-byte
	 * aligned physical address.
	 */
	dmac_va = dma_alloc_coherent(NULL, tiler.width * tiler.height *
					sizeof(*dmac_va), &dmac_pa, GFP_ATOMIC);
	if (!dmac_va)
		return -ENOMEM;

	/* Allocate tiler container manager (we share 1 on OMAP4) */
	div_pt.x = tiler.width;   /* hardcoded default */
	div_pt.y = (3 * tiler.height) / 4;
	sita = sita_init(tiler.width, tiler.height, (void *)&div_pt);

	tcm[TILFMT_8BIT]  = sita;
	tcm[TILFMT_16BIT] = sita;
	tcm[TILFMT_32BIT] = sita;
	tcm[TILFMT_PAGE]  = sita;

	/* Allocate tiler memory manager (must have 1 unique TMM per TCM ) */
	tmm_pat = tmm_pat_init(0, dmac_va, dmac_pa);
	tmm[TILFMT_8BIT]  = tmm_pat;
	tmm[TILFMT_16BIT] = tmm_pat;
	tmm[TILFMT_32BIT] = tmm_pat;
	tmm[TILFMT_PAGE]  = tmm_pat;

	/* Clear out all PAT entries */
	area.x1 = tiler.width - 1;
	area.y1 = tiler.height - 1;
	tmm_unpin(tmm_pat, area);

#ifdef CONFIG_TILER_ENABLE_NV12
	tiler.nv12_packed = tcm[TILFMT_8BIT] == tcm[TILFMT_16BIT];
#endif

	tiler_device = kmalloc(sizeof(*tiler_device), GFP_KERNEL);
	if (!tiler_device || !sita || !tmm_pat) {
		r = -ENOMEM;
		goto error;
	}

	memset(tiler_device, 0x0, sizeof(*tiler_device));
	if (tiler_major) {
		dev = MKDEV(tiler_major, tiler_minor);
		r = register_chrdev_region(dev, 1, "tiler");
	} else {
		r = alloc_chrdev_region(&dev, tiler_minor, 1, "tiler");
		tiler_major = MAJOR(dev);
	}

	cdev_init(&tiler_device->cdev, tiler.fops);
	tiler_device->cdev.owner = THIS_MODULE;
	tiler_device->cdev.ops   = tiler.fops;

	r = cdev_add(&tiler_device->cdev, dev, 1);
	if (r)
		printk(KERN_ERR "cdev_add():failed\n");

	tilerdev_class = class_create(THIS_MODULE, "tiler");

	if (IS_ERR(tilerdev_class)) {
		printk(KERN_ERR "class_create():failed\n");
		goto error;
	}

	device = device_create(tilerdev_class, NULL, dev, NULL, "tiler");
	if (device == NULL)
		printk(KERN_ERR "device_create() fail\n");

	r = platform_driver_register(&tiler_driver_ldm);

	mutex_init(&mtx);
	INIT_LIST_HEAD(&blocks);
	INIT_LIST_HEAD(&orphan_areas);
	INIT_LIST_HEAD(&orphan_onedim);

	dbgfs = debugfs_create_dir("tiler", NULL);
	if (IS_ERR_OR_NULL(dbgfs))
		dev_warn(device, "failed to create debug files.\n");
	else
		dbg_map = debugfs_create_dir("map", dbgfs);
	if (!IS_ERR_OR_NULL(dbg_map)) {
		int i;
		for (i = 0; i < ARRAY_SIZE(debugfs_maps); i++)
			debugfs_create_file(debugfs_maps[i].name, S_IRUGO,
				dbg_map, (void *) (debugfs_maps + i),
				&tiler_debug_fops);
	}

error:
	/* TODO: error handling for device registration */
	if (r) {
		kfree(tiler_device);
		tcm_deinit(sita);
		tmm_deinit(tmm_pat);
		dma_free_coherent(NULL, tiler.width * tiler.height *
					sizeof(*dmac_va), dmac_va, dmac_pa);
	}

	return r;
}

static void __exit tiler_exit(void)
{
	int i, j;

	mutex_lock(&mtx);

	debugfs_remove_recursive(dbgfs);

	/* free all process data */
	tiler.cleanup();

	/* all lists should have cleared */
	BUG_ON(!list_empty(&blocks));
	BUG_ON(!list_empty(&orphan_onedim));
	BUG_ON(!list_empty(&orphan_areas));

	mutex_unlock(&mtx);

	dma_free_coherent(NULL, tiler.width * tiler.height * sizeof(*dmac_va),
							dmac_va, dmac_pa);

	/* close containers only once */
	for (i = TILFMT_MIN; i <= TILFMT_MAX; i++) {
		/* remove identical containers (tmm is unique per tcm) */
		for (j = i + 1; j <= TILFMT_MAX; j++)
			if (tcm[i] == tcm[j]) {
				tcm[j] = NULL;
				tmm[j] = NULL;
			}

		tcm_deinit(tcm[i]);
		tmm_deinit(tmm[i]);
	}

	mutex_destroy(&mtx);
	platform_driver_unregister(&tiler_driver_ldm);
	cdev_del(&tiler_device->cdev);
	kfree(tiler_device);
	device_destroy(tilerdev_class, MKDEV(tiler_major, tiler_minor));
	class_destroy(tilerdev_class);
}

tiler_blk_handle tiler_map_1d_block(struct tiler_pa_info *pa)
{
	struct mem_info *mi = NULL;
	struct tiler_pa_info *pa_tmp = kmemdup(pa, sizeof(*pa), GFP_KERNEL);
	s32 res = pin_any_block(TILFMT_PAGE, pa->num_pg << PAGE_SHIFT, 1, 0, 0,
						__get_pi(0, true), &mi, pa_tmp);
	return res ? ERR_PTR(res) : mi;
}
EXPORT_SYMBOL(tiler_map_1d_block);

void tiler_free_block_area(tiler_blk_handle block)
{
	mutex_lock(&mtx);
	_m_try_free(block);
	mutex_unlock(&mtx);
}
EXPORT_SYMBOL(tiler_free_block_area);

tiler_blk_handle tiler_alloc_block_area(enum tiler_fmt fmt, u32 width,
					u32 height, u32 *ssptr, u32 *virt_array)
{
	struct mem_info *mi;
	*ssptr = 0;

	/* if tiler is not initialized fail gracefully */
	if (!tilerdev_class)
		return NULL;

	mi = alloc_block_area(fmt, width, height, 0, 0, __get_pi(0, true));

	if (IS_ERR_OR_NULL(mi))
		goto done;

	fill_virt_array(&mi->blk, virt_array);
	*ssptr = mi->blk.phys;

done:
	return mi;
}
EXPORT_SYMBOL(tiler_alloc_block_area);

void tiler_unpin_block(tiler_blk_handle block)
{
	mutex_lock(&mtx);
	_m_unpin(block);
	mutex_unlock(&mtx);
}
EXPORT_SYMBOL(tiler_unpin_block);

s32 tiler_memsize(enum tiler_fmt fmt, u32 width, u32 height, u32 *alloc_pages,
		  u32 *virt_pages)
{
	u16 x, y, band, align;
	int res;
	struct tiler_block_t blk;

	*alloc_pages = *virt_pages = 0;

	res = tiler.analize(fmt, width, height, &x, &y, &align, &band);

	if (!res) {
		blk.height = height;
		blk.width = width;
		blk.phys = tiler.addr(fmt, 0, 0);
		*alloc_pages = x*y;
		*virt_pages = tiler_size(&blk) / PAGE_SIZE;
	}

	return res;
}
EXPORT_SYMBOL(tiler_memsize);

u32 tiler_block_vstride(tiler_blk_handle block)
{
	return tiler_vstride(&block->blk);
}
EXPORT_SYMBOL(tiler_block_vstride);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lajos Molnar <molnar@ti.com>");
MODULE_AUTHOR("David Sin <davidsin@ti.com>");
module_init(tiler_init);
module_exit(tiler_exit);
