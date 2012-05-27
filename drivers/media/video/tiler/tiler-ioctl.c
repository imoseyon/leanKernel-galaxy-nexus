/*
 * tiler-ioctl.c
 *
 * TILER driver userspace interface functions for TI TILER hardware block.
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

static bool ssptr_lookup = true;
static bool offset_lookup = true;

module_param(ssptr_lookup, bool, 0644);
MODULE_PARM_DESC(ssptr_lookup,
	"Allow looking up a block by ssptr - This is a security risk");
module_param(offset_lookup, bool, 0644);
MODULE_PARM_DESC(offset_lookup,
	"Allow looking up a buffer by offset - This is a security risk");

static struct tiler_ops *ops;	/* shared methods and variables */
static struct blocking_notifier_head notifier;	/* notifier for events */

/*
 *  Event notification methods
 *  ==========================================================================
 */

s32 tiler_notify_event(int event, void *data)
{
	return blocking_notifier_call_chain(&notifier, event, data);
}

/*
 *  Buffer handling methods
 *  ==========================================================================
 */

/* check if an offset is used */
static bool _m_offs_in_use(u32 offs, u32 length, struct security_info *si)
{
	struct __buf_info *_b;
	/* have mutex */
	list_for_each_entry(_b, &si->bufs, by_sid)
		if (_b->buf_info.offset < offs + length &&
		    _b->buf_info.offset + _b->buf_info.length > offs)
			return 1;
	return 0;
}

/* get an offset */
static u32 _m_get_offs(struct security_info *si, u32 length)
{
	static u32 offs = 0xda7a;

	/* ensure no-one is using this offset */
	while ((offs << PAGE_SHIFT) + length < length ||
	       _m_offs_in_use(offs << PAGE_SHIFT, length, si)) {
		/* use a pseudo-random generator to get a new offset to try */

		/* Galois LSF: 20, 17 */
		offs = (offs >> 1) ^ (u32)((0 - (offs & 1u)) & 0x90000);
	}

	return offs << PAGE_SHIFT;
}

/* find and lock a block.  security_info is optional */
static struct mem_info *
_m_lock_block(u32 key, u32 id, struct security_info *si) {
	struct gid_info *gi;
	struct mem_info *mi;

	/* if security_info is given, look there first */
	if (si) {
		/* have mutex */

		/* find block in security list and free it */
		list_for_each_entry(gi, &si->groups, by_sid) {
			mi = ops->lock(key, id, gi);
			if (mi)
				return mi;
		}
	}

	/* if not found or no process_info given, find block in global list */
	return ops->lock(key, id, NULL);
}

/* register a buffer */
static s32 _m_register_buf(struct __buf_info *_b, struct security_info *si)
{
	struct mem_info *mi;
	struct tiler_buf_info *b = &_b->buf_info;
	u32 i, num = b->num_blocks, offs;

	/* check validity */
	if (num > TILER_MAX_NUM_BLOCKS || num == 0)
		return -EINVAL;

	/* find each block */
	b->length = 0;
	for (i = 0; i < num; i++) {
		mi = _m_lock_block(b->blocks[i].key, b->blocks[i].id, si);
		if (!mi) {
			/* unlock any blocks already found */
			while (i--)
				ops->unlock_free(_b->mi[i], false);
			return -EACCES;
		}
		_b->mi[i] = mi;

		/* we don't keep track of ptr and 1D stride so clear them */
		b->blocks[i].ptr = NULL;
		b->blocks[i].stride = 0;

		ops->describe(mi, b->blocks + i);
		b->length += tiler_size(&mi->blk);
	}

	/* if found all, register buffer */
	offs = _b->mi[0]->blk.phys & ~PAGE_MASK;
	b->offset = _m_get_offs(si, b->length) + offs;
	b->length -= offs;

	/* have mutex */
	list_add(&_b->by_sid, &si->bufs);

	return 0;
}

/* unregister a buffer */
void _m_unregister_buf(struct __buf_info *_b)
{
	u32 i;

	/* unregister */
	list_del(&_b->by_sid);

	/* no longer using the blocks */
	for (i = 0; i < _b->buf_info.num_blocks; i++)
		ops->unlock_free(_b->mi[i], false);

	kfree(_b);
}


/*
 *  File operations (mmap, ioctl, open, close)
 *  ==========================================================================
 */

#ifdef CONFIG_TILER_ENABLE_USERSPACE
/* mmap tiler buffer into user's virtual space */
static s32 tiler_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct __buf_info *_b;
	struct tiler_buf_info *b = NULL;
	u32 i, map_offs, map_size, blk_offs, blk_size, mapped_size;
	struct security_info *si = filp->private_data;
	u32 offs = vma->vm_pgoff << PAGE_SHIFT;
	u32 size = vma->vm_end - vma->vm_start;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* find tiler buffer to mmap */
	mutex_lock(&ops->mtx);
	list_for_each_entry(_b, &si->bufs, by_sid) {
		/* we support partial mmaping of a whole tiler buffer */
		if (offs >= (_b->buf_info.offset & PAGE_MASK) &&
		    offs + size <= PAGE_ALIGN(_b->buf_info.offset +
						_b->buf_info.length)) {
			b = &_b->buf_info;
			break;
		}
	}
	mutex_unlock(&ops->mtx);

	/* we use b to detect if we found the bufffer */
	if (!b)
		return -ENXIO;

	/* mmap relevant blocks */
	blk_offs = _b->buf_info.offset;

	/* start at the beginning of the region */
	mapped_size = 0;
	for (i = 0; i < b->num_blocks; i++, blk_offs += blk_size) {
		blk_size = tiler_size(&_b->mi[i]->blk);
		/* see if tiler block is inside the requested region */
		if (offs >= blk_offs + blk_size || offs + size < blk_offs)
			continue;
		/* get the offset and map size for this particular block */
		map_offs = max(offs, blk_offs) - blk_offs;
		map_size = min(size - mapped_size, blk_size);

		/* mmap block */
		if (tiler_mmap_blk(&_b->mi[i]->blk, map_offs, map_size, vma,
				   mapped_size))
			return -EAGAIN;

		/* update mmap region pointer */
		mapped_size += map_size;
	}
	return 0;
}

/* ioctl handler */
static long tiler_ioctl(struct file *filp, u32 cmd, unsigned long arg)
{
	s32 r;
	void __user *data = (void __user *)arg;
	struct security_info *si = filp->private_data;
	struct __buf_info *_b;
	struct tiler_buf_info buf_info = {0};
	struct tiler_block_info block_info = {0};
	struct mem_info *mi;
	u32 phys_addr;

	switch (cmd) {
	/* allocate block */
	case TILIOC_GBLK:
		if (copy_from_user(&block_info, data, sizeof(block_info)))
			return -EFAULT;

		switch (block_info.fmt) {
		case TILFMT_PAGE:
			r = ops->alloc(block_info.fmt, block_info.dim.len, 1,
					block_info.key, block_info.group_id,
					si, &mi);
			break;
		case TILFMT_8BIT:
		case TILFMT_16BIT:
		case TILFMT_32BIT:
			r = ops->alloc(block_info.fmt,
					block_info.dim.area.width,
					block_info.dim.area.height,
					block_info.key, block_info.group_id,
					si, &mi);
			break;
		default:
			return -EINVAL;
		}
		if (r)
			return r;

		/* fill out block info */
		if (mi) {
			block_info.ptr = NULL;
			ops->describe(mi, &block_info);
		}

		if (copy_to_user(data, &block_info, sizeof(block_info)))
			return -EFAULT;
		break;
	/* free/unmap block */
	case TILIOC_FBLK:
	case TILIOC_UMBLK:
		if (copy_from_user(&block_info, data, sizeof(block_info)))
			return -EFAULT;

		/* search current process first, then all processes */
		mutex_lock(&ops->mtx);
		mi = _m_lock_block(block_info.key, block_info.id, si);
		mutex_unlock(&ops->mtx);
		if (mi)
			ops->unlock_free(mi, true);

		/* free always succeeds */
		break;
	/* get physical address */
	case TILIOC_GSSP:
		down_read(&current->mm->mmap_sem);
		phys_addr = tiler_virt2phys(arg);
		up_read(&current->mm->mmap_sem);
		return phys_addr;
		break;
	/* map block */
	case TILIOC_MBLK:
		if (copy_from_user(&block_info, data, sizeof(block_info)))
			return -EFAULT;

		if (!block_info.ptr)
			return -EFAULT;

		r = ops->pin(block_info.fmt, block_info.dim.len, 1,
			      block_info.key, block_info.group_id, si,
			      &mi, (u32)block_info.ptr);
		if (r)
			return r;

		/* fill out block info */
		if (mi)
			ops->describe(mi, &block_info);

		if (copy_to_user(data, &block_info, sizeof(block_info)))
			return -EFAULT;
		break;
#ifndef CONFIG_TILER_SECURE
	/* query buffer information by offset */
	case TILIOC_QBUF:
		if (!offset_lookup)
			return -EPERM;

		if (copy_from_user(&buf_info, data, sizeof(buf_info)))
			return -EFAULT;

		/* find buffer */
		mutex_lock(&ops->mtx);
		r = -ENOENT;
		/* buffer registration is per process */
		list_for_each_entry(_b, &si->bufs, by_sid) {
			if (buf_info.offset == _b->buf_info.offset) {
				memcpy(&buf_info, &_b->buf_info,
					sizeof(buf_info));
				r = 0;
				break;
			}
		}
		mutex_unlock(&ops->mtx);

		if (r)
			return r;

		if (copy_to_user(data, &_b->buf_info, sizeof(_b->buf_info)))
			return -EFAULT;
		break;
#endif
	/* register buffer */
	case TILIOC_RBUF:
		/* save buffer information */
		_b = kmalloc(sizeof(*_b), GFP_KERNEL);
		if (!_b)
			return -ENOMEM;
		memset(_b, 0, sizeof(*_b));

		if (copy_from_user(&_b->buf_info, data, sizeof(_b->buf_info))) {
			kfree(_b);
			return -EFAULT;
		}

		mutex_lock(&ops->mtx);
		r = _m_register_buf(_b, si);
		mutex_unlock(&ops->mtx);

		if (r) {
			kfree(_b);
			return -EACCES;
		}

		/* undo registration on failure */
		if (copy_to_user(data, &_b->buf_info, sizeof(_b->buf_info))) {
			mutex_lock(&ops->mtx);
			_m_unregister_buf(_b);
			mutex_unlock(&ops->mtx);
			return -EFAULT;
		}
		break;
	/* unregister a buffer */
	case TILIOC_URBUF:
		if (copy_from_user(&buf_info, data, sizeof(buf_info)))
			return -EFAULT;

		/* find buffer */
		r = -EFAULT;
		mutex_lock(&ops->mtx);
		/* buffer registration is per process */
		list_for_each_entry(_b, &si->bufs, by_sid) {
			if (buf_info.offset == _b->buf_info.offset) {
				/* only retrieve buffer length */
				buf_info.length = _b->buf_info.length;
				_m_unregister_buf(_b);
				r = 0;
				break;
			}
		}
		mutex_unlock(&ops->mtx);

		if (r)
			return r;

		if (copy_to_user(data, &buf_info, sizeof(buf_info)))
			return -EFAULT;
		break;
	/* prereserv blocks */
	case TILIOC_PRBLK:
		if (copy_from_user(&block_info, data, sizeof(block_info)))
			return -EFAULT;

		if (block_info.fmt == TILFMT_8AND16)
#ifdef CONFIG_TILER_ENABLE_NV12
			ops->reserve_nv12(block_info.key,
					  block_info.dim.area.width,
					  block_info.dim.area.height,
					  block_info.group_id, si);
#else
			return -EINVAL;
#endif
		else
			ops->reserve(block_info.key,
				     block_info.fmt,
				     block_info.dim.area.width,
				     block_info.dim.area.height,
					PAGE_SIZE,
					0,
				     block_info.group_id, si);
		break;
	/* unreserve blocks */
	case TILIOC_URBLK:
		ops->unreserve(arg, si);
		break;
	/* query a tiler block */
	case TILIOC_QBLK:
		if (copy_from_user(&block_info, data, sizeof(block_info)))
			return -EFAULT;

		if (block_info.id) {
			/* look up by id if specified */
			mutex_lock(&ops->mtx);
			mi = _m_lock_block(block_info.key, block_info.id, si);
			mutex_unlock(&ops->mtx);
		} else
#ifndef CONFIG_TILER_SECURE
		if (ssptr_lookup) {
			/* otherwise, look up by ssptr if allowed */
			mi = ops->lock_by_ssptr(block_info.ssptr);
		} else
#endif
			return -EPERM;

		if (!mi)
			return -EFAULT;

		/* we don't keep track of ptr and 1D stride so clear them */
		block_info.ptr = NULL;
		block_info.stride = 0;

		ops->describe(mi, &block_info);
		ops->unlock_free(mi, false);

		if (copy_to_user(data, &block_info, sizeof(block_info)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* open tiler driver */
static s32 tiler_open(struct inode *ip, struct file *filp)
{
	struct security_info *si = __get_si(current->tgid, false,
						SECURE_BY_PID);
	if (!si)
		return -ENOMEM;

	filp->private_data = si;
	return 0;
}

/* close tiler driver */
static s32 tiler_release(struct inode *ip, struct file *filp)
{
	struct security_info *si = filp->private_data;

	mutex_lock(&ops->mtx);
	/* free resources if last device in this process */
	if (0 == --si->refs)
		_m_free_security_info(si);

	mutex_unlock(&ops->mtx);

	return 0;
}

/* tiler driver file operations */
static const struct file_operations tiler_fops = {
	.open = tiler_open,
	.unlocked_ioctl = tiler_ioctl,
	.release = tiler_release,
	.mmap = tiler_mmap,
};
#endif


void tiler_ioctl_init(struct tiler_ops *tiler)
{
	ops = tiler;
#ifdef CONFIG_TILER_ENABLE_USERSPACE
	ops->fops = &tiler_fops;
#endif

#ifdef CONFIG_TILER_SECURE
	offset_lookup = ssptr_lookup = false;
#endif
	BLOCKING_INIT_NOTIFIER_HEAD(&notifier);
}


s32 tiler_reg_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;
	return blocking_notifier_chain_register(&notifier, nb);
}
EXPORT_SYMBOL(tiler_reg_notifier);

s32 tiler_unreg_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;
	return blocking_notifier_chain_unregister(&notifier, nb);
}
EXPORT_SYMBOL(tiler_unreg_notifier);
