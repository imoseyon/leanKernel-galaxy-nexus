/*
 * _tiler.h
 *
 * TI TILER driver internal shared definitions.
 *
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TILER_H
#define _TILER_H

#include <linux/kernel.h>
#include <mach/tiler.h>
#include "tcm.h"

#define TILER_FORMATS		(TILFMT_MAX - TILFMT_MIN + 1)

/* per process (thread group) info */
struct process_info {
	struct list_head list;		/* other processes */
	struct list_head groups;	/* my groups */
	struct list_head bufs;		/* my registered buffers */
	pid_t pid;			/* really: thread group ID */
	u32 refs;			/* open tiler devices, 0 for processes
					   tracked via kernel APIs */
	bool kernel;			/* tracking kernel objects */
};

struct __buf_info {
	struct list_head by_pid;		/* list of buffers per pid */
	struct tiler_buf_info buf_info;
	struct mem_info *mi[TILER_MAX_NUM_BLOCKS];	/* blocks */
};

/* per group info (within a process) */
struct gid_info {
	struct list_head by_pid;	/* other groups */
	struct list_head areas;		/* all areas in this pid/gid */
	struct list_head reserved;	/* areas pre-reserved */
	struct list_head onedim;	/* all 1D areas in this pid/gid */
	u32 gid;			/* group ID */
	int refs;			/* instances directly using this ptr */
	struct process_info *pi;	/* parent */
};

/* info for an area reserved from a container */
struct area_info {
	struct list_head by_gid;	/* areas in this pid/gid */
	struct list_head blocks;	/* blocks in this area */
	u32 nblocks;			/* # of blocks in this area */

	struct tcm_area area;		/* area details */
	struct gid_info *gi;		/* link to parent, if still alive */
};

/* info for a block */
struct mem_info {
	struct list_head global;	/* reserved / global blocks */
	struct tiler_block_t blk;	/* block info */
	struct tiler_pa_info pa;	/* pinned physical pages */
	struct tcm_area area;
	int refs;			/* number of times referenced */
	bool alloced;			/* still alloced */

	struct list_head by_area;	/* blocks in the same area / 1D */
	void *parent;			/* area info for 2D, else group info */
};

/* tiler geometry information */
struct tiler_geom {
	u32 x_shft;	/* unused X-bits (as part of bpp) */
	u32 y_shft;	/* unused Y-bits (as part of bpp) */
	u32 bpp;	/* bytes per pixel */
	u32 slot_w;	/* width of each slot (in pixels) */
	u32 slot_h;	/* height of each slot (in pixels) */
	u32 bpp_m;	/* modified bytes per pixel (=1 for page mode) */
};

/* methods and variables shared between source files */
struct tiler_ops {
	/* block operations */
	s32 (*alloc) (enum tiler_fmt fmt, u32 width, u32 height,
			u32 key,
			u32 gid, struct process_info *pi,
			struct mem_info **info);
	s32 (*pin) (enum tiler_fmt fmt, u32 width, u32 height,
			u32 key, u32 gid, struct process_info *pi,
			struct mem_info **info, u32 usr_addr);
	void (*reserve_nv12) (u32 n, u32 width, u32 height,
					u32 gid, struct process_info *pi);
	void (*reserve) (u32 n, enum tiler_fmt fmt, u32 width, u32 height,
			 u32 gid, struct process_info *pi);
	void (*unreserve) (u32 gid, struct process_info *pi);

	/* block access operations */
	struct mem_info * (*lock) (u32 key, u32 id, struct gid_info *gi);
	struct mem_info * (*lock_by_ssptr) (u32 sys_addr);
	void (*describe) (struct mem_info *i, struct tiler_block_info *blk);
	void (*unlock_free) (struct mem_info *mi, bool free);

	s32 (*lay_2d) (enum tiler_fmt fmt, u16 n, u16 w, u16 h, u16 band,
			u16 align, struct gid_info *gi,
			struct list_head *pos);
#ifdef CONFIG_TILER_ENABLE_NV12
	s32 (*lay_nv12) (int n, u16 w, u16 w1, u16 h, struct gid_info *gi,
			 u8 *p);
#endif
	/* group operations */
	struct gid_info * (*get_gi) (struct process_info *pi, u32 gid);
	void (*release_gi) (struct gid_info *gi);
	void (*destroy_group) (struct gid_info *pi);

	/* group access operations */
	void (*add_reserved) (struct list_head *reserved, struct gid_info *gi);
	void (*release) (struct list_head *reserved);

	/* area operations */
	s32 (*analize) (enum tiler_fmt fmt, u32 width, u32 height,
			u16 *x_area, u16 *y_area, u16 *band, u16 *align);

	/* process operations */
	void (*cleanup) (void);

	/* geometry operations */
	void (*xy) (u32 ssptr, u32 *x, u32 *y);
	u32 (*addr) (enum tiler_fmt fmt, u32 x, u32 y);
	const struct tiler_geom * (*geom) (enum tiler_fmt fmt);

	/* additional info */
	const struct file_operations *fops;
#ifdef CONFIG_TILER_ENABLE_NV12
	bool nv12_packed;	/* whether NV12 is packed into same container */
#endif
	u32 page;		/* page size */
	u32 width;		/* container width */
	u32 height;		/* container height */

	struct mutex mtx;	/* mutex for interfaces and ioctls */
};

void tiler_iface_init(struct tiler_ops *tiler);
void tiler_geom_init(struct tiler_ops *tiler);
void tiler_reserve_init(struct tiler_ops *tiler);
void tiler_nv12_init(struct tiler_ops *tiler);
u32 tiler_best2pack(u16 o, u16 a, u16 b, u16 w, u16 *n, u16 *_area);
void tiler_ioctl_init(struct tiler_ops *tiler);
struct process_info *__get_pi(pid_t pid, bool kernel);
void _m_unregister_buf(struct __buf_info *_b);
s32 tiler_notify_event(int event, void *data);
void _m_free_process_info(struct process_info *pi);

struct process_info *__get_pi(pid_t pid, bool kernel);

#endif
