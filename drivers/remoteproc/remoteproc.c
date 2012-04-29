/*
 * Remote Processor Framework
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Mark Grosen <mgrosen@ti.com>
 * Brian Swetland <swetland@google.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Robert Tivy <rtivy@ti.com>
 * Armando Uribe De Leon <x0095078@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/remoteproc.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <plat/remoteproc.h>

/* list of available remote processors on this board */
static LIST_HEAD(rprocs);
static DEFINE_SPINLOCK(rprocs_lock);

/* debugfs parent dir */
static struct dentry *rproc_dbg;

static ssize_t rproc_format_trace_buf(struct rproc *rproc, char __user *userbuf,
					size_t count, loff_t *ppos,
					const void *src, int size)
{
	const char *buf = (const char *) src;
	ssize_t num_copied = 0;
	static int from_beg;
	loff_t pos = *ppos;
	int *w_idx;
	int i, w_pos, ret = 0;

	if (mutex_lock_interruptible(&rproc->tlock))
		return -EINTR;

	/* When src is NULL, the remoteproc is offline. */
	if (!src) {
		ret = -EIO;
		goto unlock;
	}

	if (size < 2 * sizeof(u32)) {
		ret = -EINVAL;
		goto unlock;
	}

	/* Assume write_idx is the penultimate byte in the buffer trace*/
	size = size - (sizeof(u32) * 2);
	w_idx = (int *)(buf + size);
	w_pos = *w_idx;

	if (from_beg)
		goto print_beg;

	if (pos == 0)
		*ppos = w_pos;

	for (i = w_pos; i < size && buf[i]; i++)
		;

	if (i > w_pos)
		num_copied =
			simple_read_from_buffer(userbuf, count, ppos, src, i);
		if (!num_copied) {
			from_beg = 1;
			*ppos = 0;
		} else {
			ret = num_copied;
			goto unlock;
		}
print_beg:
	for (i = 0; i < w_pos && buf[i]; i++)
		;

	if (i) {
		num_copied =
			simple_read_from_buffer(userbuf, count, ppos, src, i);
		if (!num_copied)
			from_beg = 0;
		ret = num_copied;
	}
unlock:
	mutex_unlock(&rproc->tlock);
	return ret;
}

static ssize_t rproc_name_read(struct file *filp, char __user *userbuf,
						size_t count, loff_t *ppos)
{
	struct rproc *rproc = filp->private_data;
	/* need room for the name, a newline and a terminating null */
	char buf[RPROC_MAX_NAME + 2];
	int i;

	i = snprintf(buf, RPROC_MAX_NAME + 2, "%s\n", rproc->name);

	return simple_read_from_buffer(userbuf, count, ppos, buf, i);
}

static ssize_t rproc_version_read(struct file *filp, char __user *userbuf,
						size_t count, loff_t *ppos)
{

	struct rproc *rproc = filp->private_data;
	char *pch;
	int len;
	pch = strstr(rproc->header, "version:");
	if (!pch)
		return 0;
	pch += strlen("version:") + 1;
	len = rproc->header_len - (pch - rproc->header);
	return simple_read_from_buffer(userbuf, count, ppos, pch, len);
}

static int rproc_open_generic(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define DEBUGFS_READONLY_FILE(name, v, l)				\
static ssize_t name## _rproc_read(struct file *filp,			\
		char __user *ubuf, size_t count, loff_t *ppos)		\
{									\
	struct rproc *rproc = filp->private_data;			\
	return rproc_format_trace_buf(rproc, ubuf, count, ppos, v, l);	\
}									\
									\
static const struct file_operations name ##_rproc_ops = {		\
	.read = name ##_rproc_read,					\
	.open = rproc_open_generic,					\
	.llseek	= generic_file_llseek,					\
};

#ifdef CONFIG_REMOTEPROC_CORE_DUMP

/* + 1 for the notes segment */
#define NUM_PHDR (RPROC_MAX_MEM_ENTRIES + 1)

#define CORE_STR "CORE"

/* Intermediate core-dump-file format */
struct core_rproc {
	struct rproc *rproc;
	/* ELF state */
	Elf_Half e_phnum;

	struct core {
		struct elfhdr elf;
		struct elf_phdr phdr[NUM_PHDR];
		struct {
			struct elf_note note_prstatus;
			char name[sizeof(CORE_STR)];
			struct elf_prstatus prstatus __aligned(4);
		} core_note __packed __aligned(4);
	} core __packed;

	loff_t offset;
};

/* Return the number of segments to be written to the core file */
static int rproc_core_map_count(const struct rproc *rproc)
{
	int i = 0;
	int count = 0;
	for (;; i++) {
		if (!rproc->memory_maps[i].size)
			break;
		if (!rproc->memory_maps[i].core)
			continue;
		count++;
	}

	/* The Ducati has a low number of segments */
	if (count > PN_XNUM)
		return -1;

	return count;
}

/* Copied from fs/binfmt_elf.c */
static void fill_elf_header(struct elfhdr *elf, int segs)
{
	memset(elf, 0, sizeof(*elf));

	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELFCLASS32;
	elf->e_ident[EI_DATA] = ELFDATA2LSB;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELFOSABI_NONE;

	elf->e_type = ET_CORE;
	elf->e_machine = EM_ARM;
	elf->e_version = EV_CURRENT;
	elf->e_phoff = sizeof(struct elfhdr);
	elf->e_flags = EF_ARM_EABI_VER5;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = segs;

	return;
}

static void fill_elf_segment_headers(struct core_rproc *d)
{
	int i = 0;
	int hi = 0;
	loff_t offset = d->offset;
	for (;; i++) {
		u32 size;

		size = d->rproc->memory_maps[i].size;
		if (!size)
			break;
		if (!d->rproc->memory_maps[i].core)
			continue;

		BUG_ON(hi >= d->e_phnum - 1);

		d->core.phdr[hi].p_type = PT_LOAD;
		d->core.phdr[hi].p_offset = offset;
		d->core.phdr[hi].p_vaddr = d->rproc->memory_maps[i].da;
		d->core.phdr[hi].p_paddr = d->rproc->memory_maps[i].pa;
		d->core.phdr[hi].p_filesz = size;
		d->core.phdr[hi].p_memsz = size;
		/* FIXME: get these from the Ducati */
		d->core.phdr[hi].p_flags = PF_R | PF_W | PF_X;

		pr_debug("%s: phdr type %d f_off %08x va %08x pa %08x fl %x\n",
			__func__,
			d->core.phdr[hi].p_type,
			d->core.phdr[hi].p_offset,
			d->core.phdr[hi].p_vaddr,
			d->core.phdr[hi].p_paddr,
			d->core.phdr[hi].p_flags);

		offset += size;
		hi++;
	}
}

static int setup_rproc_elf_core_dump(struct core_rproc *d)
{
	short __phnum;
	struct elf_phdr *nphdr;
	struct exc_regs *xregs = d->rproc->cdump_buf1;
	struct pt_regs *regs =
		(struct pt_regs *)&d->core.core_note.prstatus.pr_reg;

	memset(&d->core.elf, 0, sizeof(d->core.elf));

	__phnum = rproc_core_map_count(d->rproc);
	if (__phnum < 0 || __phnum > ARRAY_SIZE(d->core.phdr))
		return -EIO;
	d->e_phnum = __phnum + 1; /* + 1 for notes */

	pr_info("number of segments: %d\n", d->e_phnum);

	fill_elf_header(&d->core.elf, d->e_phnum);

	nphdr = d->core.phdr + __phnum;
	nphdr->p_type    = PT_NOTE;
	nphdr->p_offset  = 0;
	nphdr->p_vaddr   = 0;
	nphdr->p_paddr   = 0;
	nphdr->p_filesz  = 0;
	nphdr->p_memsz   = 0;
	nphdr->p_flags   = 0;
	nphdr->p_align   = 0;

	/* The notes start right after the phdr array. Adjust p_filesz
	 * accordingly if you add more notes
	 */
	nphdr->p_filesz = sizeof(d->core.core_note);
	nphdr->p_offset = offsetof(struct core, core_note);

	d->core.core_note.note_prstatus.n_namesz = sizeof(CORE_STR);
	d->core.core_note.note_prstatus.n_descsz =
		sizeof(struct elf_prstatus);
	d->core.core_note.note_prstatus.n_type = NT_PRSTATUS;
	memcpy(d->core.core_note.name, CORE_STR, sizeof(CORE_STR));

	remoteproc_fill_pt_regs(regs, xregs);

	/* We ignore the NVIC registers for now */

	d->offset = sizeof(struct core);
	d->offset = roundup(d->offset, PAGE_SIZE);
	fill_elf_segment_headers(d);
	return 0;
}

static int core_rproc_open(struct inode *inode, struct file *filp)
{
	int i;
	struct core_rproc *d;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->rproc = inode->i_private;
	filp->private_data = d;

	setup_rproc_elf_core_dump(d);

	if (0) {
		const struct rproc *rproc;
		rproc = d->rproc;
		for (i = 0; rproc->memory_maps[i].size; i++) {
			pr_info("%s: memory_map[%d] pa %08x sz %d core %d\n",
				__func__,
				i,
				rproc->memory_maps[i].pa,
				rproc->memory_maps[i].size,
				rproc->memory_maps[i].core);
		}
	}

	return 0;
}

static int core_rproc_release(struct inode *inode, struct file *filp)
{
	pr_info("%s\n", __func__);
	kfree(filp->private_data);
	return 0;
}

/* Given an offset to read from, return the index of the memory-map region to
 * read from.
 */
static int rproc_memory_map_index(const struct rproc *rproc, loff_t *off)
{
	int i = 0;
	for (;; i++) {
		int size = rproc->memory_maps[i].size;

		if (!size)
			break;
		if (!rproc->memory_maps[i].core)
			continue;
		if (*off < size)
			return i;

		*off -= size;
	}

	return -1;
}

ssize_t core_rproc_write(struct file *filp,
		const char __user *buffer, size_t count, loff_t *off)
{
	char cmd[100];
	int cmdlen;
	struct core_rproc *d = filp->private_data;
	struct rproc *rproc = d->rproc;

	cmdlen = min(sizeof(cmd) - 1, count);
	if (copy_from_user(cmd, buffer, cmdlen))
		return -EFAULT;
	cmd[cmdlen] = 0;

	if (!strncmp(cmd, "enable", 6)) {
		pr_info("remoteproc %s halt on crash ENABLED\n", rproc->name);
		rproc->halt_on_crash = true;
		goto done;
	} else if (!strncmp(cmd, "disable", 7)) {
		pr_info("remoteproc %s halt on crash DISABLED\n", rproc->name);
		rproc->halt_on_crash = false;
		/* If you disable halt-on-crashed after the remote processor
		 * has already crashed, we will let it continue crashing (so it
		 * can get handled otherwise) as well.
		 */
		if (rproc->state != RPROC_CRASHED)
			goto done;
	} else if (strncmp(cmd, "continue", 8)) {
		pr_err("%s: invalid command: expecting \"enable\"," \
				"\"disable\", or \"continue\"\n", __func__);
		return -EINVAL;
	}

	if (rproc->state == RPROC_CRASHED) {
		pr_info("remoteproc %s: resuming crash recovery\n",
			rproc->name);
		blocking_notifier_call_chain(&rproc->nbh, RPROC_ERROR, NULL);
	}

done:
	*off += count;
	return count;
}

static ssize_t core_rproc_read(struct file *filp,
			char __user *userbuf, size_t count, loff_t *ppos)
{
	const struct core_rproc *d = filp->private_data;
	const struct rproc *rproc = d->rproc;
	int index;
	loff_t pos;
	size_t remaining = count;
	ssize_t copied = 0;

	pr_debug("%s count %d off %lld\n", __func__, count, *ppos);

	/* copy the ELF and segment header first */
	if (*ppos < d->offset) {
		copied = simple_read_from_buffer(userbuf, count,
					ppos, &d->core, d->offset);
		if (copied < 0) {
			pr_err("%s: could not copy ELF header\n", __func__);
			return -EIO;
		}

		pr_debug("%s: copied %d/%lld from ELF header\n", __func__,
			copied, d->offset);
		remaining -= copied;
	}

	/* copy the data */
	while (remaining) {
		size_t remaining_in_region;
		const struct rproc_mem_entry *r;
		void __iomem *kvaddr;

		pos = *ppos - d->offset;
		index = rproc_memory_map_index(rproc, &pos);
		if (index < 0) {
			pr_info("%s: EOF at off %lld\n", __func__, *ppos);
			break;
		}

		r = &rproc->memory_maps[index];

		remaining_in_region = r->size - pos;
		if (remaining_in_region > remaining)
			remaining_in_region = remaining;

		pr_debug("%s: iomap 0x%x size %d\n", __func__, r->pa, r->size);
		kvaddr = ioremap(r->pa, r->size);
		if (!kvaddr) {
			pr_err("%s: iomap error: region %d (phys 0x%08x size %d)\n",
				__func__, index, r->pa, r->size);
			return -EIO;
		}

		pr_debug("%s: off %lld -> [%d](pa 0x%08x off %lld sz %d)\n",
			__func__,
			*ppos, index, r->pa, pos, r->size);

		if (copy_to_user(userbuf + copied, kvaddr + pos,
				remaining_in_region)) {
			pr_err("%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}

		iounmap(kvaddr);

		copied += remaining_in_region;
		*ppos += remaining_in_region;
		BUG_ON(remaining < remaining_in_region);
		remaining -= remaining_in_region;
	}

	return copied;
}

static const struct file_operations core_rproc_ops = {
	.read = core_rproc_read,
	.write = core_rproc_write,
	.open = core_rproc_open,
	.release = core_rproc_release,
	.llseek = generic_file_llseek,
};
#endif /* CONFIG_REMOTEPROC_CORE_DUMP */

static const struct file_operations rproc_name_ops = {
	.read = rproc_name_read,
	.open = rproc_open_generic,
	.llseek	= generic_file_llseek,
};

static const struct file_operations rproc_version_ops = {
	.read = rproc_version_read,
	.open = rproc_open_generic,
	.llseek	= generic_file_llseek,
};

DEBUGFS_READONLY_FILE(trace0, rproc->trace_buf0, rproc->trace_len0);
DEBUGFS_READONLY_FILE(trace1, rproc->trace_buf1, rproc->trace_len1);
DEBUGFS_READONLY_FILE(trace0_last, rproc->last_trace_buf0,
						rproc->last_trace_len0);
DEBUGFS_READONLY_FILE(trace1_last, rproc->last_trace_buf1,
						rproc->last_trace_len1);
DEBUGFS_READONLY_FILE(cdump0, rproc->cdump_buf0, rproc->cdump_len0);
DEBUGFS_READONLY_FILE(cdump1, rproc->cdump_buf1, rproc->cdump_len1);

#define DEBUGFS_ADD(name)						\
	debugfs_create_file(#name, 0444, rproc->dbg_dir,		\
			rproc, &name## _rproc_ops)

/**
 * __find_rproc_by_name - find a registered remote processor by name
 * @name: name of the remote processor
 *
 * Internal function that returns the rproc @name, or NULL if @name does
 * not exists.
 */
static struct rproc *__find_rproc_by_name(const char *name)
{
	struct rproc *rproc;
	struct list_head *tmp;

	spin_lock(&rprocs_lock);

	list_for_each(tmp, &rprocs) {
		rproc = list_entry(tmp, struct rproc, next);
		if (!strcmp(rproc->name, name))
			break;
		rproc = NULL;
	}

	spin_unlock(&rprocs_lock);

	return rproc;
}

/**
 * __rproc_da_to_pa - convert a device (virtual) address to its physical address
 * @maps: the remote processor's memory mappings array
 * @da: a device address (as seen by the remote processor)
 * @pa: pointer to the physical address result
 *
 * This function converts @da to its physical address (pa) by going through
 * @maps, looking for a mapping that contains @da, and then calculating the
 * appropriate pa.
 *
 * On success 0 is returned, and the @pa is updated with the result.
 * Otherwise, -EINVAL is returned.
 */
static int
rproc_da_to_pa(const struct rproc_mem_entry *maps, u64 da, phys_addr_t *pa)
{
	int i;
	u64 offset;

	for (i = 0; maps[i].size; i++) {
		const struct rproc_mem_entry *me = &maps[i];

		if (da >= me->da && da < (me->da + me->size)) {
			offset = da - me->da;
			pr_debug("%s: matched mem entry no. %d\n",
				__func__, i);
			*pa = me->pa + offset;
			return 0;
		}
	}

	return -EINVAL;
}

static int rproc_mmu_fault_isr(struct rproc *rproc, u64 da, u32 flags)
{
	dev_err(rproc->dev, "%s\n", __func__);
	schedule_work(&rproc->error_work);
	return -EIO;
}

static int rproc_watchdog_isr(struct rproc *rproc)
{
	dev_err(rproc->dev, "%s\n", __func__);
	schedule_work(&rproc->error_work);
	return 0;
}

static int rproc_crash(struct rproc *rproc)
{
	init_completion(&rproc->error_comp);
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	pm_runtime_dont_use_autosuspend(rproc->dev);
#endif
	if (rproc->ops->dump_registers)
		rproc->ops->dump_registers(rproc);

	if (rproc->trace_buf0 && rproc->last_trace_buf0)
		memcpy(rproc->last_trace_buf0, rproc->trace_buf0,
				rproc->last_trace_len0);
	if (rproc->trace_buf1 && rproc->last_trace_buf1)
		memcpy(rproc->last_trace_buf1, rproc->trace_buf1,
				rproc->last_trace_len1);
	rproc->state = RPROC_CRASHED;

	return 0;
}

static int _event_notify(struct rproc *rproc, int type, void *data)
{
	if (type == RPROC_ERROR) {
		mutex_lock(&rproc->lock);
		/* only notify first crash */
		if (rproc->state == RPROC_CRASHED) {
			mutex_unlock(&rproc->lock);
			return 0;
		}
		rproc_crash(rproc);
		mutex_unlock(&rproc->lock);
		/* If halt_on_crash do not notify the error */
		pr_info("remoteproc: %s has crashed\n", rproc->name);
		if (rproc->halt_on_crash) {
			/* FIXME: send uevent here */
			pr_info("remoteproc: %s: halt-on-crash enabled: "
				"deferring crash recovery\n", rproc->name);
			return 0;
		}
	}

	return blocking_notifier_call_chain(&rproc->nbh, type, data);
}

/**
 * rproc_start - power on the remote processor and let it start running
 * @rproc: the remote processor
 * @bootaddr: address of first instruction to execute (optional)
 *
 * Start a remote processor (i.e. power it on, take it out of reset, etc..)
 */
static void rproc_start(struct rproc *rproc, u64 bootaddr)
{
	struct device *dev = rproc->dev;
	int err;

	err = mutex_lock_interruptible(&rproc->lock);
	if (err) {
		dev_err(dev, "can't lock remote processor %d\n", err);
		return;
	}

	if (rproc->ops->iommu_init) {
		err = rproc->ops->iommu_init(rproc, rproc_mmu_fault_isr);
		if (err) {
			dev_err(dev, "can't configure iommu %d\n", err);
			goto unlock_mutex;
		}
	}

	if (rproc->ops->watchdog_init) {
		err = rproc->ops->watchdog_init(rproc, rproc_watchdog_isr);
		if (err) {
			dev_err(dev, "can't configure watchdog timer %d\n",
				err);
			goto wdt_error;
		}
	}

#ifdef CONFIG_REMOTEPROC_CORE_DUMP
	debugfs_create_file("core", 0400, rproc->dbg_dir,
			rproc, &core_rproc_ops);
#endif

	err = rproc->ops->start(rproc, bootaddr);
	if (err) {
		dev_err(dev, "can't start rproc %s: %d\n", rproc->name, err);
		goto start_error;
	}

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, rproc->sus_timeout);
	pm_runtime_get_noresume(rproc->dev);
	pm_runtime_set_active(rproc->dev);
	if (!rproc->secure_mode)
		pm_runtime_enable(rproc->dev);
	pm_runtime_mark_last_busy(rproc->dev);
	pm_runtime_put_autosuspend(rproc->dev);
#endif

	rproc->state = RPROC_RUNNING;

	dev_info(dev, "remote processor %s is now up\n", rproc->name);
	rproc->secure_ok = true;
	complete_all(&rproc->secure_restart);
	mutex_unlock(&rproc->lock);

	return;

	/*
	 * signal always, as we would need a notification in both the
	 * normal->secure & secure->normal mode transitions, otherwise
	 * we would have to introduce one more variable.
	 */
start_error:
	if (rproc->ops->watchdog_exit)
		rproc->ops->watchdog_exit(rproc);
wdt_error:
	if (rproc->ops->iommu_exit)
		rproc->ops->iommu_exit(rproc);
unlock_mutex:
	rproc->secure_ok = false;
	complete_all(&rproc->secure_restart);
	mutex_unlock(&rproc->lock);
}

static void rproc_reset_poolmem(struct rproc *rproc)
{
	struct rproc_mem_pool *pool = rproc->memory_pool;

	if (!pool || !pool->mem_base || !pool->mem_size) {
		pr_warn("invalid pool\n");
		return;
	}

	pool->cur_base = pool->mem_base;
	pool->cur_size = pool->mem_size;
}

static int rproc_add_mem_entry(struct rproc *rproc, struct fw_resource *rsc)
{
	struct rproc_mem_entry *me = rproc->memory_maps;
	int i = 0;
	int ret = 0;

	while (me->da || me->pa || me->size) {
		me += 1;
		i++;
		if (i == RPROC_MAX_MEM_ENTRIES) {
			ret = -ENOSPC;
			break;
		}
	}

	if (!ret) {
		me->da = rsc->da;
		me->pa = (phys_addr_t)rsc->pa;
		me->size = rsc->len;
#ifdef CONFIG_REMOTEPROC_CORE_DUMP
		/* FIXME: ION heaps are reported as RSC_CARVEOUT.  We need a
		 * better way to understand which sections are for
		 * code/stack/heap/static data, and which belong to the
		 * carveouts we don't care about in a core dump.
		 * Perhaps the ION carveout should be reported as RSC_DEVMEM.
		 */
		me->core = (rsc->type == RSC_CARVEOUT && rsc->pa != 0xba300000);
#endif
	}

	return ret;
}

static int rproc_alloc_poolmem(struct rproc *rproc, u32 size, phys_addr_t *pa)
{
	struct rproc_mem_pool *pool = rproc->memory_pool;

	*pa = 0;
	if (!pool || !pool->mem_base || !pool->mem_size) {
		pr_warn("invalid pool\n");
		return -EINVAL;
	}
	if (pool->cur_size < size) {
		pr_warn("out of carveout memory\n");
		return -ENOMEM;
	}

	*pa = pool->cur_base;
	pool->cur_base += size;
	pool->cur_size -= size;
	return 0;
}

static int rproc_check_poolmem(struct rproc *rproc, u32 size, phys_addr_t pa)
{
	struct rproc_mem_pool *pool = rproc->memory_pool;

	if (!pool || !pool->st_base || !pool->st_size) {
		pr_warn("invalid pool\n");
		return -EINVAL;
	}

	if (pa < pool->st_base || pa + size > pool->st_base + pool->st_size) {
		pr_warn("section size does not fit within carveout memory\n");
		return -ENOSPC;
	}

	return 0;
}

static int rproc_handle_resources(struct rproc *rproc, struct fw_resource *rsc,
							int len, u64 *bootaddr)
{
	struct device *dev = rproc->dev;
	phys_addr_t pa;
	u64 da;
	u64 trace_da0 = 0;
	u64 trace_da1 = 0;
	u64 cdump_da0 = 0;
	u64 cdump_da1 = 0;
	int ret = 0;

	while (len >= sizeof(*rsc) && !ret) {
		da = rsc->da;
		pa = rsc->pa;
		dev_dbg(dev, "resource: type %d, da 0x%llx, pa 0x%llx, "
			"mapped pa: 0x%x, len 0x%x, reserved 0x%x, "
			"name %s\n", rsc->type, rsc->da, rsc->pa, pa,
			rsc->len, rsc->reserved, rsc->name);

		if (rsc->reserved)
			dev_warn(dev, "nonzero reserved\n");

		switch (rsc->type) {
		case RSC_TRACE:
			if (trace_da0 && trace_da1) {
				dev_warn(dev, "skipping extra trace rsc %s\n",
						rsc->name);
				break;
			}

			/* store the da for processing at the end */
			if (!trace_da0) {
				rproc->trace_len0 = rsc->len;
				rproc->last_trace_len0 = rsc->len;
				trace_da0 = da;
			} else {
				rproc->trace_len1 = rsc->len;
				rproc->last_trace_len1 = rsc->len;
				trace_da1 = da;
			}
			break;
		case RSC_CRASHDUMP:
			if (rproc->cdump_buf0 && rproc->cdump_buf1) {
				dev_warn(dev, "skipping extra trace rsc %s\n",
						rsc->name);
				break;
			}
			/* store the da for processing at the end */
			if (!cdump_da0) {
				rproc->cdump_len0 = rsc->len;
				cdump_da0 = da;
			} else {
				rproc->cdump_len1 = rsc->len;
				cdump_da1 = da;
			}
			break;
		case RSC_BOOTADDR:
			*bootaddr = da;
			break;
		case RSC_DEVMEM:
			ret = rproc_add_mem_entry(rproc, rsc);
			if (ret) {
				dev_err(dev, "can't add mem_entry %s\n",
							rsc->name);
				break;
			}
			break;
		case RSC_CARVEOUT:
			if (!pa) {
				ret = rproc_alloc_poolmem(rproc, rsc->len, &pa);
				if (ret) {
					dev_err(dev, "can't alloc poolmem %s\n",
								rsc->name);
					break;
				}
				rsc->pa = pa;
			} else {
				ret = rproc_check_poolmem(rproc, rsc->len, pa);
				if (ret) {
					dev_err(dev, "static memory for %s "
						"doesn't belong to poolmem\n",
						rsc->name);
					break;
				}
			}
			ret = rproc_add_mem_entry(rproc, rsc);
			if (ret) {
				dev_err(dev, "can't add mem_entry %s\n",
							rsc->name);
				break;
			}
			break;
		default:
			/* we don't support much right now. so use dbg lvl */
			dev_dbg(dev, "unsupported resource type %d\n",
								rsc->type);
			break;
		}

		rsc++;
		len -= sizeof(*rsc);
	}

	if (ret)
		goto error;

	/*
	 * post-process trace buffers, as we cannot rely on the order of the
	 * trace section and the carveout sections.
	 *
	 * trace buffer memory _is_ normal memory, so we cast away the
	 * __iomem to make sparse happy
	 */

	if (mutex_lock_interruptible(&rproc->tlock))
		goto error;

	if (trace_da0) {
		ret = rproc_da_to_pa(rproc->memory_maps, trace_da0, &pa);
		if (ret)
			goto unlock;
		rproc->trace_buf0 = (__force void *)
				ioremap_nocache(pa, rproc->trace_len0);
		if (rproc->trace_buf0) {
			DEBUGFS_ADD(trace0);
			if (!rproc->last_trace_buf0) {
				rproc->last_trace_buf0 = kzalloc(sizeof(u32) *
							rproc->last_trace_len0,
							GFP_KERNEL);
				if (!rproc->last_trace_buf0) {
					ret = -ENOMEM;
					goto unlock;
				}
				DEBUGFS_ADD(trace0_last);
			}
		} else {
			dev_err(dev, "can't ioremap trace buffer0\n");
			ret = -EIO;
			goto unlock;
		}
	}
	if (trace_da1) {
		ret = rproc_da_to_pa(rproc->memory_maps, trace_da1, &pa);
		if (ret)
			goto unlock;
		rproc->trace_buf1 = (__force void *)
				ioremap_nocache(pa, rproc->trace_len1);
		if (rproc->trace_buf1) {
			DEBUGFS_ADD(trace1);
			if (!rproc->last_trace_buf1) {
				rproc->last_trace_buf1 = kzalloc(sizeof(u32) *
							rproc->last_trace_len1,
							GFP_KERNEL);
				if (!rproc->last_trace_buf1) {
					ret = -ENOMEM;
					goto unlock;
				}
				DEBUGFS_ADD(trace1_last);
			}
		} else {
			dev_err(dev, "can't ioremap trace buffer1\n");
			ret = -EIO;
			goto unlock;
		}
	}

	/*
	 * post-process crash-dump buffers, as we cannot rely on the order of
	 * the crash-dump section and the carveout sections.
	 *
	 * crash-dump memory _is_ normal memory, so we cast away the __iomem to
	 * make sparse happy
	 */
	if (cdump_da0) {
		ret = rproc_da_to_pa(rproc->memory_maps, cdump_da0, &pa);
		if (ret)
			goto unlock;
		rproc->cdump_buf0 = (__force void *)
					ioremap_nocache(pa, rproc->cdump_len0);
		if (rproc->cdump_buf0)
			DEBUGFS_ADD(cdump0);
		else {
			dev_err(dev, "can't ioremap cdump buffer0\n");
			ret = -EIO;
			goto unlock;
		}
	}
	if (cdump_da1) {
		ret = rproc_da_to_pa(rproc->memory_maps, cdump_da1, &pa);
		if (ret)
			goto unlock;
		rproc->cdump_buf1 = (__force void *)
					ioremap_nocache(pa, rproc->cdump_len1);
		if (rproc->cdump_buf1)
			DEBUGFS_ADD(cdump1);
		else {
			dev_err(dev, "can't ioremap cdump buffer1\n");
			ret = -EIO;
		}
	}

unlock:
	mutex_unlock(&rproc->tlock);

error:
	if (ret && rproc->dbg_dir) {
		debugfs_remove_recursive(rproc->dbg_dir);
		rproc->dbg_dir = NULL;
	}
	return ret;
}

static int rproc_process_fw(struct rproc *rproc, struct fw_section *section,
						int left, u64 *bootaddr)
{
	struct device *dev = rproc->dev;
	phys_addr_t pa;
	u32 len, type;
	u64 da;
	int ret = 0;
	void *ptr;
	bool copy;

	/* first section should be FW_RESOURCE section */
	if (section->type != FW_RESOURCE) {
		dev_err(dev, "first section is not FW_RESOURCE: type %u found",
			section->type);
		ret = -EINVAL;
		goto exit;
	}

	while (left > sizeof(struct fw_section)) {
		da = section->da;
		len = section->len;
		type = section->type;
		copy = true;

		dev_dbg(dev, "section: type %d da 0x%llx len 0x%x\n",
								type, da, len);

		left -= sizeof(struct fw_section);
		if (left < section->len) {
			dev_err(dev, "BIOS image is truncated\n");
			ret = -EINVAL;
			break;
		}

		/* a resource table needs special handling */
		if (section->type == FW_RESOURCE) {
			ret = rproc_handle_resources(rproc,
					(struct fw_resource *) section->content,
					len, bootaddr);
			if (ret) {
				break;
			}
		}

		if (section->type <= FW_DATA) {
			ret = rproc_da_to_pa(rproc->memory_maps, da, &pa);
			if (ret) {
				dev_err(dev, "rproc_da_to_pa failed:%d\n", ret);
				break;
			}
		} else if (rproc->secure_mode) {
			pa = da;
			if (section->type == FW_MMU)
				rproc->secure_ttb = (void *)pa;
		} else
			copy = false;

		dev_dbg(dev, "da 0x%llx pa 0x%x len 0x%x\n", da, pa, len);

		if (copy) {
			/* ioremaping normal memory, so make sparse happy */
			ptr = (__force void *) ioremap_nocache(pa, len);
			if (!ptr) {
				dev_err(dev, "can't ioremap 0x%x\n", pa);
				ret = -ENOMEM;
				break;
			}

			memcpy(ptr, section->content, len);

			/* iounmap normal memory, so make sparse happy */
			iounmap((__force void __iomem *) ptr);
		}

		section = (struct fw_section *)(section->content + len);
		left -= len;
	}

exit:
	return ret;
}

static void rproc_loader_cont(const struct firmware *fw, void *context)
{
	struct rproc *rproc = context;
	struct device *dev = rproc->dev;
	const char *fwfile = rproc->firmware;
	u64 bootaddr = 0;
	struct fw_header *image;
	struct fw_section *section;
	int left, ret;

	if (!fw) {
		dev_err(dev, "%s: failed to load %s\n", __func__, fwfile);
		goto complete_fw;
	}

	dev_info(dev, "Loaded BIOS image %s, size %d\n", fwfile, fw->size);

	/* make sure this image is sane */
	if (fw->size < sizeof(struct fw_header)) {
		dev_err(dev, "Image is too small\n");
		goto out;
	}

	image = (struct fw_header *) fw->data;

	if (memcmp(image->magic, "RPRC", 4)) {
		dev_err(dev, "Image is corrupted (bad magic)\n");
		goto out;
	}

	dev_info(dev, "BIOS image version is %d\n", image->version);

	rproc->header = kzalloc(image->header_len, GFP_KERNEL);
	if (!rproc->header) {
		dev_err(dev, "%s: kzalloc failed\n", __func__);
		goto out;
	}
	memcpy(rproc->header, image->header, image->header_len);
	rproc->header_len = image->header_len;

	/* Ensure we recognize this BIOS version: */
	if (image->version != RPROC_BIOS_VERSION) {
		dev_err(dev, "Expected BIOS version: %d!\n",
			RPROC_BIOS_VERSION);
		goto out;
	}

	/* now process the image, section by section */
	section = (struct fw_section *)(image->header + image->header_len);

	left = fw->size - sizeof(struct fw_header) - image->header_len;

	ret = rproc_process_fw(rproc, section, left, &bootaddr);
	if (ret) {
		dev_err(dev, "Failed to process the image: %d\n", ret);
		goto out;
	}

	rproc_start(rproc, bootaddr);

out:
	release_firmware(fw);
complete_fw:
	/* allow all contexts calling rproc_put() to proceed */
	complete_all(&rproc->firmware_loading_complete);
}

static int rproc_loader(struct rproc *rproc)
{
	const char *fwfile = rproc->firmware;
	struct device *dev = rproc->dev;
	int ret;

	if (!fwfile) {
		dev_err(dev, "%s: no firmware to load\n", __func__);
		return -EINVAL;
	}

	/*
	 * allow building remoteproc as built-in kernel code, without
	 * hanging the boot process
	 */
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, fwfile,
			dev, GFP_KERNEL, rproc, rproc_loader_cont);
	if (ret < 0) {
		dev_err(dev, "request_firmware_nowait failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int rproc_set_secure(const char *name, bool enable)
{
	struct rproc *rproc;
	int ret;

	rproc = __find_rproc_by_name(name);
	if (!rproc) {
		pr_err("can't find remote processor %s\n", name);
		return -ENODEV;
	}

	/*
	 * set the secure_mode here, the secure_ttb will be filled up during
	 * the reload process.
	 */
	if (mutex_lock_interruptible(&rproc->secure_lock))
		return -EINTR;
	rproc->secure_mode = enable;
	rproc->secure_ttb = NULL;
	rproc->secure_ok = false;
	init_completion(&rproc->secure_restart);

	/*
	 * restart the processor, the mode will dictate regular load or
	 * secure load
	 */
	_event_notify(rproc, RPROC_SECURE, (void *)enable);

	/* block until the restart is complete */
	if (wait_for_completion_interruptible(&rproc->secure_restart)) {
		pr_err("error waiting restart completion\n");
		ret = -EINTR;
		goto out;
	}

	ret = rproc->secure_ok ? 0 : -EACCES;
out:
	mutex_unlock(&rproc->secure_lock);

	return ret;
}
EXPORT_SYMBOL(rproc_set_secure);

int rproc_error_notify(struct rproc *rproc)
{
	return _event_notify(rproc, RPROC_ERROR, NULL);
}
EXPORT_SYMBOL_GPL(rproc_error_notify);

struct rproc *rproc_get(const char *name)
{
	struct rproc *rproc, *ret = NULL;
	struct device *dev;
	int err;

	rproc = __find_rproc_by_name(name);
	if (!rproc) {
		pr_err("can't find remote processor %s\n", name);
		return NULL;
	}

	dev = rproc->dev;

	err = mutex_lock_interruptible(&rproc->lock);
	if (err) {
		dev_err(dev, "can't lock remote processor %s\n", name);
		return NULL;
	}

	if (rproc->state == RPROC_CRASHED) {
		mutex_unlock(&rproc->lock);
		if (wait_for_completion_interruptible(&rproc->error_comp)) {
			dev_err(dev, "error waiting error completion\n");
			return NULL;
		}
		mutex_lock(&rproc->lock);
	}

	/* prevent underlying implementation from being removed */
	if (!try_module_get(rproc->owner)) {
		dev_err(dev, "%s: can't get owner\n", __func__);
		goto unlock_mutex;
	}

	/* bail if rproc is already powered up */
	if (rproc->count++) {
		ret = rproc;
		goto unlock_mutex;
	}

	/* rproc_put() calls should wait until async loader completes */
	init_completion(&rproc->firmware_loading_complete);

	dev_info(dev, "powering up %s\n", name);

	err = rproc_loader(rproc);
	if (err) {
		dev_err(dev, "failed to load rproc %s\n", rproc->name);
		complete_all(&rproc->firmware_loading_complete);
		module_put(rproc->owner);
		--rproc->count;
		goto unlock_mutex;
	}

	rproc->state = RPROC_LOADING;
	ret = rproc;

unlock_mutex:
	mutex_unlock(&rproc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rproc_get);

void rproc_put(struct rproc *rproc)
{
	struct device *dev = rproc->dev;
	int ret;

	/* make sure rproc is not loading now */
	wait_for_completion(&rproc->firmware_loading_complete);

	ret = mutex_lock_interruptible(&rproc->lock);
	if (ret) {
		dev_err(dev, "can't lock rproc %s: %d\n", rproc->name, ret);
		return;
	}

	if (!rproc->count) {
		dev_warn(dev, "asymmetric rproc_put\n");
		ret = -EINVAL;
		goto out;
	}

	/* if the remote proc is still needed, bail out */
	if (--rproc->count)
		goto out;

	if (mutex_lock_interruptible(&rproc->tlock))
		goto out;

	if (rproc->trace_buf0)
		/* iounmap normal memory, so make sparse happy */
		iounmap((__force void __iomem *) rproc->trace_buf0);
	if (rproc->trace_buf1)
		/* iounmap normal memory, so make sparse happy */
		iounmap((__force void __iomem *) rproc->trace_buf1);
	rproc->trace_buf0 = rproc->trace_buf1 = NULL;

	if (rproc->cdump_buf0)
		/* iounmap normal memory, so make sparse happy */
		iounmap((__force void __iomem *) rproc->cdump_buf0);
	if (rproc->cdump_buf1)
		/* iounmap normal memory, so make sparse happy */
		iounmap((__force void __iomem *) rproc->cdump_buf1);
	rproc->cdump_buf0 = rproc->cdump_buf1 = NULL;

	mutex_unlock(&rproc->tlock);

	rproc_reset_poolmem(rproc);
	memset(rproc->memory_maps, 0, sizeof(rproc->memory_maps));
	kfree(rproc->header);

	/*
	 * make sure rproc is really running before powering it off.
	 * this is important, because the fw loading might have failed.
	 */
	if (rproc->state == RPROC_RUNNING || rproc->state == RPROC_CRASHED) {
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
		/*
		 * Call resume, it will cancel any pending autosuspend,
		 * so that no callback is executed after the device is stopped.
		 * Device stop function takes care of shutting down the device.
		 */
		pm_runtime_get_sync(rproc->dev);
		pm_runtime_put_noidle(rproc->dev);
		if (!rproc->secure_reset)
			pm_runtime_disable(rproc->dev);

		pm_runtime_set_suspended(rproc->dev);
#endif
		ret = rproc->ops->stop(rproc);
		if (ret) {
			dev_err(dev, "can't stop rproc %s: %d\n", rproc->name,
									ret);
			goto out;
		}
		if (rproc->ops->watchdog_exit) {
			ret = rproc->ops->watchdog_exit(rproc);
			if (ret) {
				dev_err(rproc->dev, "error watchdog_exit %d\n",
					ret);
				goto out;
			}
		}
		if (rproc->ops->iommu_exit) {
			ret = rproc->ops->iommu_exit(rproc);
			if (ret) {
				dev_err(rproc->dev, "error iommu_exit %d\n",
					ret);
				goto out;
			}
		}
	}

	if (rproc->state == RPROC_CRASHED)
		complete_all(&rproc->error_comp);

	rproc->state = RPROC_OFFLINE;

	dev_info(dev, "stopped remote processor %s\n", rproc->name);

out:
	mutex_unlock(&rproc->lock);
	if (!ret)
		module_put(rproc->owner);
}
EXPORT_SYMBOL_GPL(rproc_put);

static void rproc_error_work(struct work_struct *work)
{
	struct rproc *rproc = container_of(work, struct rproc, error_work);

	dev_dbg(rproc->dev, "%s\n", __func__);
	_event_notify(rproc, RPROC_ERROR, NULL);
}

int rproc_event_register(struct rproc *rproc, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&rproc->nbh, nb);
}
EXPORT_SYMBOL_GPL(rproc_event_register);

int rproc_event_unregister(struct rproc *rproc, struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&rproc->nbh, nb);
}
EXPORT_SYMBOL_GPL(rproc_event_unregister);

void rproc_last_busy(struct rproc *rproc)
{
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	struct device *dev = rproc->dev;

	mutex_lock(&rproc->pm_lock);
	if (pm_runtime_suspended(dev) ||
			!pm_runtime_autosuspend_expiration(dev)) {
		pm_runtime_mark_last_busy(dev);
		mutex_unlock(&rproc->pm_lock);
		/*
		 * if the remote processor is suspended, we can not wake it
		 * up (that would abort system suspend), instead state that
		 * the remote processor needs to be waken up on system resume.
		 */
		mutex_lock(&rproc->lock);
		if (rproc->state == RPROC_SUSPENDED) {
			rproc->need_resume = true;
			mutex_unlock(&rproc->lock);
			return;
		}
		mutex_unlock(&rproc->lock);
		pm_runtime_get_sync(dev);
		pm_runtime_mark_last_busy(dev);
		pm_runtime_put_autosuspend(dev);
		return;
	}
	pm_runtime_mark_last_busy(dev);
	mutex_unlock(&rproc->pm_lock);
#endif
}
EXPORT_SYMBOL(rproc_last_busy);

#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
static int rproc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	mutex_lock(&rproc->lock);
	if (rproc->state != RPROC_SUSPENDED) {
		mutex_unlock(&rproc->lock);
		return 0;
	}

	if (!rproc->need_resume)
		goto unlock;

	rproc->need_resume = false;
	pm_runtime_get_sync(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
unlock:
	rproc->state = (ret) ? RPROC_CRASHED : RPROC_RUNNING;
	mutex_unlock(&rproc->lock);
	if (ret) {
		_event_notify(rproc, RPROC_ERROR, NULL);
		dev_err(dev, "Error resuming %d\n", ret);
	}
	return ret;
}

static int rproc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	mutex_lock(&rproc->lock);
	if (rproc->state != RPROC_RUNNING) {
		mutex_unlock(&rproc->lock);
		return 0;
	}

	if (pm_runtime_suspended(dev))
		goto out;
	/*
	 * If it is not runtime suspended, it means remote processor is still
	 * doing something. However we need to stop it.
	 */

	dev_dbg(dev, "%s: will be forced to suspend\n", rproc->name);

	rproc->force_suspend = true;
	ret = pm_runtime_suspend(dev);
	rproc->force_suspend = false;
	if (ret)
		goto out;
	/*
	 * As the remote processor had to be forced to suspend, it was
	 * executing some task, so it needs to be waken up on system resume
	 */
	rproc->need_resume = true;
out:
	if (!ret)
		rproc->state = RPROC_SUSPENDED;
	mutex_unlock(&rproc->lock);

	return ret;
}

static int rproc_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	int ret = 0;

	dev_dbg(dev, "Enter %s\n", __func__);

	if (rproc->ops->resume)
		ret = rproc->ops->resume(rproc);

	if (!ret)
		_event_notify(rproc, RPROC_RESUME, NULL);

	return 0;
}

static int rproc_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	int ret = 0;
	unsigned to;

	dev_dbg(dev, "Enter %s\n", __func__);

	if (rproc->state == RPROC_SUSPENDED)
		return 0;

	mutex_lock(&rproc->pm_lock);

	if (pm_runtime_autosuspend_expiration(dev) && !rproc->force_suspend) {
		ret = -EBUSY;
		goto abort;
	}

	/*
	 * Notify PROC_PRE_SUSPEND only when the suspend is not forced.
	 * Users can use pre suspend call back to cancel autosuspend, but
	 * when the suspend is forced, there is no need to notify them
	 */
	if (!rproc->force_suspend)
		ret = _event_notify(rproc, RPROC_PRE_SUSPEND, NULL);
	/*
	 * If rproc user avoids suspend, that means it is still using rproc.
	 * Lets go to abort suspend.
	 */
	if (ret) {
		dev_dbg(dev, "suspend aborted by user %d\n", ret);
		ret = -EBUSY;
		goto abort;
	}
	/* Now call machine-specific suspend function (if exist) */
	if (rproc->ops->suspend)
		ret = rproc->ops->suspend(rproc, rproc->force_suspend);
	/*
	 * If it fails with -EBUSY/EAGAIN, remote processor is still running,
	 * but rproc users were not aware of that, so lets abort suspend.
	 * If it is a different error, there is something wrong with the
	 * remote processor. Return that error to pm runtime  framework,
	 * which will disable autosuspend.
	 */
	if (ret) {
		dev_dbg(dev, "suspend aborted by remote processor %d\n", ret);
		if (ret != -EBUSY  && ret != -EAGAIN)
			dev_err(dev, "suspend error %d", ret);
		goto abort;
	}
	/* we are not interested in the returned value */
	_event_notify(rproc, RPROC_POS_SUSPEND, NULL);
	mutex_unlock(&rproc->pm_lock);

	return 0;
abort:
	pm_runtime_mark_last_busy(dev);
	to = jiffies_to_msecs(pm_runtime_autosuspend_expiration(dev) - jiffies);
	pm_schedule_suspend(dev, to);
	dev->power.timer_autosuspends = 1;
	mutex_unlock(&rproc->pm_lock);
	return ret;
}

const struct dev_pm_ops rproc_gen_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rproc_suspend, rproc_resume)
	SET_RUNTIME_PM_OPS(rproc_runtime_suspend, rproc_runtime_resume, NULL)
};
#endif
int
rproc_set_constraints(struct rproc *rproc, enum rproc_constraint type, long v)
{
	int ret;
	char *cname[] = {"scale", "latency", "bandwidth"};
	int (*func)(struct rproc *, long);

	switch (type) {
	case RPROC_CONSTRAINT_SCALE:
		func = rproc->ops->scale;
		break;
	case RPROC_CONSTRAINT_LATENCY:
		func = rproc->ops->set_lat;
		break;
	case RPROC_CONSTRAINT_BANDWIDTH:
		func = rproc->ops->set_bw;
		break;
	default:
		dev_err(rproc->dev, "invalid constraint\n");
		return -EINVAL;
	}

	if (!func) {
		dev_err(rproc->dev, "%s: no %s constraint\n",
			__func__, cname[type]);
		return -EINVAL;
	}

	mutex_lock(&rproc->lock);
	if (rproc->state == RPROC_OFFLINE) {
		pr_err("%s: rproc inactive\n", __func__);
		mutex_unlock(&rproc->lock);
		return -EPERM;
	}

	dev_dbg(rproc->dev, "set %s constraint %ld\n", cname[type], v);
	ret = func(rproc, v);
	if (ret)
		dev_err(rproc->dev, "error %s constraint\n", cname[type]);
	mutex_unlock(&rproc->lock);

	return ret;
}
EXPORT_SYMBOL(rproc_set_constraints);

int rproc_register(struct device *dev, const char *name,
				const struct rproc_ops *ops,
				const char *firmware,
				struct rproc_mem_pool *memory_pool,
				struct module *owner,
				unsigned sus_timeout)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc;

	if (!dev || !name || !ops)
		return -EINVAL;

	rproc = kzalloc(sizeof(struct rproc), GFP_KERNEL);
	if (!rproc) {
		dev_err(dev, "%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	rproc->dev = dev;
	rproc->name = name;
	rproc->ops = ops;
	rproc->firmware = firmware;
	rproc->owner = owner;
	rproc->memory_pool = memory_pool;
#ifdef CONFIG_REMOTE_PROC_AUTOSUSPEND
	rproc->sus_timeout = sus_timeout;
	mutex_init(&rproc->pm_lock);
#endif
	mutex_init(&rproc->lock);
	mutex_init(&rproc->secure_lock);
	mutex_init(&rproc->tlock);
	INIT_WORK(&rproc->error_work, rproc_error_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&rproc->nbh);

	rproc->state = RPROC_OFFLINE;

	rproc->qos_request = kzalloc(sizeof(*rproc->qos_request),
			GFP_KERNEL);
	if (!rproc->qos_request) {
		kfree(rproc);
		return -ENOMEM;
	}

	pm_qos_add_request(rproc->qos_request, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	rproc->secure_mode = false;
	rproc->secure_ttb = NULL;
	init_completion(&rproc->secure_restart);

	spin_lock(&rprocs_lock);
	list_add_tail(&rproc->next, &rprocs);
	spin_unlock(&rprocs_lock);

	platform_set_drvdata(pdev, rproc);

	dev_info(dev, "%s is available\n", name);

	if (!rproc_dbg)
		goto out;

	rproc->dbg_dir = debugfs_create_dir(dev_name(dev), rproc_dbg);
	if (!rproc->dbg_dir) {
		dev_err(dev, "can't create debugfs dir\n");
		goto out;
	}

	debugfs_create_file("name", 0444, rproc->dbg_dir, rproc,
							&rproc_name_ops);

	debugfs_create_file("version", 0444, rproc->dbg_dir, rproc,
							&rproc_version_ops);
out:
	return 0;
}
EXPORT_SYMBOL_GPL(rproc_register);

int rproc_unregister(const char *name)
{
	struct rproc *rproc;

	rproc = __find_rproc_by_name(name);
	if (!rproc) {
		pr_err("can't find remote processor %s\n", name);
		return -EINVAL;
	}

	dev_info(rproc->dev, "removing %s\n", name);

	if (rproc->dbg_dir)
		debugfs_remove_recursive(rproc->dbg_dir);

	spin_lock(&rprocs_lock);
	list_del(&rproc->next);
	spin_unlock(&rprocs_lock);

	rproc->secure_mode = false;
	rproc->secure_ttb = NULL;
	pm_qos_remove_request(rproc->qos_request);
	kfree(rproc->qos_request);
	kfree(rproc->last_trace_buf0);
	kfree(rproc->last_trace_buf1);
	kfree(rproc);

	return 0;
}
EXPORT_SYMBOL_GPL(rproc_unregister);

static int __init remoteproc_init(void)
{
	if (debugfs_initialized()) {
		rproc_dbg = debugfs_create_dir(KBUILD_MODNAME, NULL);
		if (!rproc_dbg)
			pr_err("can't create debugfs dir\n");
	}

	return 0;
}
/* must be ready in time for device_initcall users */
subsys_initcall(remoteproc_init);

static void __exit remoteproc_exit(void)
{
	if (rproc_dbg)
		debugfs_remove(rproc_dbg);
}
module_exit(remoteproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic Remote Processor Framework");
