/*
 * tiler-iface.c
 *
 * TILER driver interace functions for TI TILER hardware block.
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

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/fs.h>		/* fops */
#include <linux/uaccess.h>	/* copy_to_user */
#include <linux/slab.h>		/* kmalloc */
#include <linux/sched.h>	/* current */
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <asm/mach/map.h>	/* for ioremap_page */

#include "_tiler.h"

static bool security = CONFIG_TILER_SECURITY;

module_param(security, bool, 0644);
MODULE_PARM_DESC(security,
	"Separate allocations by different processes into different pages");

static struct list_head procs;	/* list of process info structs */
static struct tiler_ops *ops;	/* shared methods and variables */

/*
 *  process_info handling methods
 *  ==========================================================================
 */

/* get process info, and increment refs for device tracking */
struct process_info *__get_pi(pid_t pid, bool kernel)
{
	struct process_info *pi;

	/*
	 * treat all processes as the same, kernel processes are still treated
	 * differently so not to free kernel allocated areas when a user process
	 * closes the tiler driver
	 */
	if (!security)
		pid = 0;

	/* find process context */
	mutex_lock(&ops->mtx);
	list_for_each_entry(pi, &procs, list) {
		if (pi->pid == pid && pi->kernel == kernel)
			goto done;
	}

	/* create process context */
	pi = kmalloc(sizeof(*pi), GFP_KERNEL);
	if (!pi)
		goto done;
	memset(pi, 0, sizeof(*pi));

	pi->pid = pid;
	pi->kernel = kernel;
	INIT_LIST_HEAD(&pi->groups);
	INIT_LIST_HEAD(&pi->bufs);
	list_add(&pi->list, &procs);
done:
	/* increment reference count */
	if (pi && !kernel)
		pi->refs++;
	mutex_unlock(&ops->mtx);
	return pi;
}

/**
 * Free all info kept by a process: all registered buffers, allocated blocks,
 * and unreferenced blocks.  Any blocks/areas still referenced will move to the
 * orphaned lists to avoid issues if a new process is created with the same pid.
 *
 *    caller MUST already have mtx
 */
void _m_free_process_info(struct process_info *pi)
{
	struct gid_info *gi, *gi_;
#ifdef CONFIG_TILER_ENABLE_USERSPACE
	struct __buf_info *_b = NULL, *_b_ = NULL;

	if (!list_empty(&pi->bufs))
		tiler_notify_event(TILER_DEVICE_CLOSE, NULL);

	/* unregister all buffers */
	list_for_each_entry_safe(_b, _b_, &pi->bufs, by_pid)
		_m_unregister_buf(_b);
#endif
	BUG_ON(!list_empty(&pi->bufs));

	/* free all allocated blocks, and remove unreferenced ones */
	list_for_each_entry_safe(gi, gi_, &pi->groups, by_pid)
		ops->destroy_group(gi);

	BUG_ON(!list_empty(&pi->groups));
	list_del(&pi->list);
	kfree(pi);
}

static void destroy_processes(void)
{
	struct process_info *pi, *pi_;

	mutex_lock(&ops->mtx);

	list_for_each_entry_safe(pi, pi_, &procs, list)
		_m_free_process_info(pi);
	BUG_ON(!list_empty(&procs));

	mutex_unlock(&ops->mtx);
}


/* initialize tiler interface */
void tiler_iface_init(struct tiler_ops *tiler)
{
	ops = tiler;
	ops->cleanup = destroy_processes;

#ifdef CONFIG_TILER_SECURE
	security = true;
#endif
	INIT_LIST_HEAD(&procs);
}

/*
 *  Kernel APIs
 *  ==========================================================================
 */

u32 tiler_virt2phys(u32 usr)
{
	pmd_t *pmd;
	pte_t *ptep;
	pgd_t *pgd = pgd_offset(current->mm, usr);

	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pmd = pmd_offset(pgd, usr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	ptep = pte_offset_map(pmd, usr);
	if (ptep && pte_present(*ptep))
		return (*ptep & PAGE_MASK) | (~PAGE_MASK & usr);

	return 0;
}
EXPORT_SYMBOL(tiler_virt2phys);

void tiler_reservex(u32 n, enum tiler_fmt fmt, u32 width, u32 height,
		   u32 align, u32 offs, u32 gid, pid_t pid)
{
	struct process_info *pi = __get_pi(pid, true);

	if (pi)
		ops->reserve(n, fmt, width, height, align, offs, gid, pi);
}
EXPORT_SYMBOL(tiler_reservex);

void tiler_reserve(u32 n, enum tiler_fmt fmt, u32 width, u32 height,
		  u32 align, u32 offs)
{
	tiler_reservex(n, fmt, width, height, align, offs, 0, current->tgid);
}
EXPORT_SYMBOL(tiler_reserve);

#ifdef CONFIG_TILER_ENABLE_NV12
void tiler_reservex_nv12(u32 n, u32 width, u32 height, u32 align, u32 offs,
			u32 gid, pid_t pid)
{
	struct process_info *pi = __get_pi(pid, true);

	if (pi)
		ops->reserve_nv12(n, width, height, align, offs, gid, pi);
}
EXPORT_SYMBOL(tiler_reservex_nv12);

void tiler_reserve_nv12(u32 n, u32 width, u32 height, u32 align, u32 offs)
{
	tiler_reservex_nv12(n, width, height, align, offs, 0, current->tgid);
}
EXPORT_SYMBOL(tiler_reserve_nv12);
#endif

s32 tiler_allocx(struct tiler_block_t *blk, enum tiler_fmt fmt,
				u32 align, u32 offs, u32 gid, pid_t pid)
{
	struct mem_info *mi;
	struct process_info *pi;
	s32 res;

	BUG_ON(!blk || blk->phys);

	pi = __get_pi(pid, true);
	if (!pi)
		return -ENOMEM;

	res = ops->alloc(fmt, blk->width, blk->height, align, offs, blk->key,
								gid, pi, &mi);
	if (mi) {
		blk->phys = mi->blk.phys;
		blk->id = mi->blk.id;
	}
	return res;
}
EXPORT_SYMBOL(tiler_allocx);

s32 tiler_alloc(struct tiler_block_t *blk, enum tiler_fmt fmt,
		u32 align, u32 offs)
{
	return tiler_allocx(blk, fmt, align, offs, 0, current->tgid);
}
EXPORT_SYMBOL(tiler_alloc);

s32 tiler_mapx(struct tiler_block_t *blk, enum tiler_fmt fmt, u32 gid,
				pid_t pid, u32 usr_addr)
{
	struct mem_info *mi;
	struct process_info *pi;
	s32 res;

	BUG_ON(!blk || blk->phys);

	pi = __get_pi(pid, true);
	if (!pi)
		return -ENOMEM;

	res = ops->pin(fmt, blk->width, blk->height, blk->key, gid, pi, &mi,
								usr_addr);
	if (mi) {
		blk->phys = mi->blk.phys;
		blk->id = mi->blk.id;
	}
	return res;

}
EXPORT_SYMBOL(tiler_mapx);

s32 tiler_map(struct tiler_block_t *blk, enum tiler_fmt fmt, u32 usr_addr)
{
	return tiler_mapx(blk, fmt, 0, current->tgid, usr_addr);
}
EXPORT_SYMBOL(tiler_map);

s32 tiler_mmap_blk(struct tiler_block_t *blk, u32 offs, u32 size,
				struct vm_area_struct *vma, u32 voffs)
{
	u32 v, p, len;

	/* mapping must fit into vma */
	BUG_ON(vma->vm_start > vma->vm_start + voffs ||
		vma->vm_start + voffs > vma->vm_start + voffs + size ||
		vma->vm_start + voffs + size > vma->vm_end);

	/* mapping must fit into block */
	BUG_ON(offs > offs + size || offs + size > tiler_size(blk));

	v = tiler_vstride(blk);
	p = tiler_pstride(blk);

	/* remap block portion */
	len = v - (offs % v);	/* initial area to map */
	while (size) {
		/* restrict to size still needs mapping */
		if (len > size)
			len = size;

		vma->vm_pgoff = (blk->phys + offs) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, vma->vm_start + voffs, vma->vm_pgoff,
				    len, vma->vm_page_prot))
			return -EAGAIN;
		voffs += len;
		offs += len + p - v;
		size -= len;
		len = v;	/* subsequent area to map */
	}
	return 0;
}
EXPORT_SYMBOL(tiler_mmap_blk);

s32 tiler_ioremap_blk(struct tiler_block_t *blk, u32 offs, u32 size,
				u32 addr, u32 mtype)
{
	u32 v, p;
	u32 len;		/* area to map */
	const struct mem_type *type = get_mem_type(mtype);

	/* mapping must fit into address space */
	BUG_ON(addr > addr + size);

	/* mapping must fit into block */
	BUG_ON(offs > offs + size || offs + size > tiler_size(blk));

	v = tiler_vstride(blk);
	p = tiler_pstride(blk);

	/* move offset and address to end */
	offs += blk->phys + size;
	addr += size;

	len = v - (offs % v);	/* initial area to map */
	while (size) {
		while (len && size) {
			if (ioremap_page(addr - size, offs - size, type))
				return -EAGAIN;
			len  -= PAGE_SIZE;
			size -= PAGE_SIZE;
		}

		offs += p - v;
		len = v;	/* subsequent area to map */
	}
	return 0;
}
EXPORT_SYMBOL(tiler_ioremap_blk);

void tiler_free(struct tiler_block_t *blk)
{
	/* find block */
	struct mem_info *mi = ops->lock(blk->key, blk->id, NULL);
	if (mi)
		ops->unlock_free(mi, true);
	blk->phys = blk->id = 0;
}
EXPORT_SYMBOL(tiler_free);
