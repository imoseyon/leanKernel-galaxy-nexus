/*
 * tiler.h
 *
 * TILER driver support functions for TI TILER hardware block.
 *
 * Authors: Lajos Molnar <molnar@ti.com>
 *          David Sin <davidsin@ti.com>
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

#ifndef TILER_H
#define TILER_H

#include <linux/mm.h>

/*
 * ----------------------------- API Definitions -----------------------------
 */

/* return true if physical address is in the tiler container */
bool is_tiler_addr(u32 phys);

enum tiler_fmt {
	TILFMT_MIN     = -2,
	TILFMT_INVALID = -2,
	TILFMT_NONE    = -1,
	TILFMT_8BIT    = 0,
	TILFMT_16BIT   = 1,
	TILFMT_32BIT   = 2,
	TILFMT_PAGE    = 3,
	TILFMT_MAX     = 3,
	TILFMT_8AND16  = 4,	/* used to mark NV12 reserve block */
};

/* tiler block info */
struct tiler_block_t {
	u32 phys;		/* system space (L3) tiler addr */
	u32 width;		/* width */
	u32 height;		/* height */
	u32 key;		/* secret key */
	u32 id;			/* unique block ID */
};

/* tiler (image/video frame) view */
struct tiler_view_t {
	u32 tsptr;		/* tiler space addr */
	u32 width;		/* width */
	u32 height;		/* height */
	u32 bpp;		/* bytes per pixel */
	s32 h_inc;		/* horizontal increment */
	s32 v_inc;		/* vertical increment */
};

/* get the tiler format for a physical address or TILFMT_INVALID */
enum tiler_fmt tiler_fmt(u32 phys);

/* get the modified (1 for page mode) bytes-per-pixel for a tiler block */
u32 tiler_bpp(const struct tiler_block_t *b);

/* get tiler block physical stride */
u32 tiler_pstride(const struct tiler_block_t *b);

/* get tiler block virtual stride */
static inline u32 tiler_vstride(const struct tiler_block_t *b)
{
	return PAGE_ALIGN((b->phys & ~PAGE_MASK) + tiler_bpp(b) * b->width);
}

/* returns the virtual size of the block (for mmap) */
static inline u32 tiler_size(const struct tiler_block_t *b)
{
	return b->height * tiler_vstride(b);
}

/* Event types */
#define TILER_DEVICE_CLOSE	0

/**
 * Registers a notifier block with TILER driver.
 *
 * @param nb		notifier_block
 *
 * @return error status
 */
s32 tiler_reg_notifier(struct notifier_block *nb);

/**
 * Un-registers a notifier block with TILER driver.
 *
 * @param nb		notifier_block
 *
 * @return error status
 */
s32 tiler_unreg_notifier(struct notifier_block *nb);

/**
 * Get the physical address for a given user va.
 *
 * @param usr	user virtual address
 *
 * @return valid pa or 0 for error
 */
u32 tiler_virt2phys(u32 usr);

/**
 * Reserves a 1D or 2D TILER block area and memory for the
 * current process with group ID 0.
 *
 * @param blk	pointer to tiler block data.  This must be set up ('phys' member
 *		must be 0) with the tiler block information. 'height' must be 1
 *		for 1D block.
 * @param fmt	TILER block format
 *
 * @return error status
 */
s32 tiler_alloc(struct tiler_block_t *blk, enum tiler_fmt fmt);

/**
 * Reserves a 1D or 2D TILER block area and memory for a set process and group
 * ID.
 *
 * @param blk	pointer to tiler block data.  This must be set up ('phys' member
 *		must be 0) with the tiler block information. 'height' must be 1
 *		for 1D block.
 * @param fmt	TILER block format
 * @param gid	group ID
 * @param pid	process ID
 *
 * @return error status
 */
s32 tiler_allocx(struct tiler_block_t *blk, enum tiler_fmt fmt,
					u32 gid, pid_t pid);

/**
 * Mmaps a portion of a tiler block to a virtual address.  Use this method in
 * your driver's mmap function to potentially combine multiple tiler blocks as
 * one virtual buffer.
 *
 * @param blk		pointer to tiler block data
 * @param offs		offset from where to map (must be page aligned)
 * @param size		size of area to map (must be page aligned)
 * @param vma		VMM memory area to map to
 * @param voffs		offset (from vm_start) in the VMM memory area to start
 *			mapping at
 *
 * @return error status
 */
s32 tiler_mmap_blk(struct tiler_block_t *blk, u32 offs, u32 size,
				struct vm_area_struct *vma, u32 voffs);

/**
 * Ioremaps a portion of a tiler block.  Use this method in your driver instead
 * of ioremap to potentially combine multiple tiler blocks as one virtual
 * buffer.
 *
 * @param blk		pointer to tiler block data
 * @param offs		offset from where to map (must be page aligned)
 * @param size		size of area to map (must be page aligned)
 * @param addr		virtual address
 * @param mtype		ioremap memory type (e.g. MT_DEVICE)
 *
 * @return error status
 */
s32 tiler_ioremap_blk(struct tiler_block_t *blk, u32 offs, u32 size, u32 addr,
							u32 mtype);

/**
 * Maps an existing buffer to a 1D or 2D TILER area for the
 * current process with group ID 0.
 *
 * Currently, only 1D area mapping is supported.
 *
 * NOTE: alignment is always PAGE_SIZE and offset is 0 as full pages are mapped
 * into tiler container.
 *
 * @param blk		pointer to tiler block data.  This must be set up
 *			('phys' member must be 0) with the tiler block
 *			information. 'height' must be 1 for 1D block.
 * @param fmt		TILER format
 * @param usr_addr	user space address of existing buffer.
 *
 * @return error status
 */
s32 tiler_map(struct tiler_block_t *blk, enum tiler_fmt fmt, u32 usr_addr);

/**
 * Maps an existing buffer to a 1D or 2D TILER area for a set process and group
 * ID.
 *
 * Currently, only 1D area mapping is supported.
 *
 * NOTE: alignment is always PAGE_SIZE and offset is 0 as full pages are mapped
 * into tiler container.
 *
 * @param blk		pointer to tiler block data.  This must be set up
 *			('phys' member must be 0) with the tiler block
 *			information. 'height' must be 1 for 1D block.
 * @param fmt		TILER format
 * @param gid		group ID
 * @param pid		process ID
 * @param usr_addr	user space address of existing buffer.
 *
 * @return error status
 */
s32 tiler_mapx(struct tiler_block_t *blk, enum tiler_fmt fmt,
					u32 gid, pid_t pid, u32 usr_addr);

/**
 * Frees TILER memory.  Since there may be multiple references for the same area
 * if duplicated by tiler_dup, the area is only actually freed if all references
 * have been freed.
 *
 * @param blk	pointer to a tiler block data as filled by tiler_alloc,
 *		tiler_map or tiler_dup.  'phys' and 'id' members will be set to
 *		0 on success.
 */
void tiler_free(struct tiler_block_t *blk);

/**
 * Reserves tiler area for n identical blocks for the current process.  Use this
 * method to get optimal placement of multiple identical tiler blocks; however,
 * it may not reserve area if tiler_alloc is equally efficient.
 *
 * @param n		number of identical set of blocks
 * @param fmt		TILER format
 * @param width		block width
 * @param height	block height (must be 1 for 1D)
 */
void tiler_reserve(u32 n, enum tiler_fmt fmt, u32 width, u32 height);

/**
 * Reserves tiler area for n identical blocks.  Use this method to get optimal
 * placement of multiple identical tiler blocks; however, it may not reserve
 * area if tiler_alloc is equally efficient.
 *
 * @param n		number of identical set of blocks
 * @param fmt		TILER bit mode
 * @param width		block width
 * @param height	block height (must be 1 for 1D)
 * @param gid		group ID
 * @param pid		process ID
 */
void tiler_reservex(u32 n, enum tiler_fmt fmt, u32 width, u32 height,
				u32 gid, pid_t pid);

/**
 * Reserves tiler area for n identical NV12 blocks for the current process.  Use
 * this method to get optimal placement of multiple identical NV12 tiler blocks;
 * however, it may not reserve area if tiler_alloc is equally efficient.
 *
 * @param n		number of identical set of blocks
 * @param width		block width (Y)
 * @param height	block height (Y)
 */
void tiler_reserve_nv12(u32 n, u32 width, u32 height);

/**
 * Reserves tiler area for n identical NV12 blocks.  Use this method to get
 * optimal placement of multiple identical NV12 tiler blocks; however, it may
 * not reserve area if tiler_alloc is equally efficient.
 *
 * @param n		number of identical set of blocks
 * @param width		block width (Y)
 * @param height	block height (Y)
 * @param gid		group ID
 * @param pid		process ID
 */
void tiler_reservex_nv12(u32 n, u32 width, u32 height, u32 gid, pid_t pid);

/**
 * Create a view based on a tiler address and width and height
 *
 * This method should only be used as a last resort, e.g. if tilview object
 * cannot be passed because of incoherence with other view 2D objects that must
 * be supported.
 *
 * @param view		Pointer to a view where the information will be stored
 * @param ssptr		MUST BE a tiler address
 * @param width		view width
 * @param height	view height
 */
void tilview_create(struct tiler_view_t *view, u32 phys, u32 width, u32 height);

/**
 * Obtains the view information for a tiler block
 *
 * @param view		Pointer to a view where the information will be stored
 * @param blk		Pointer to an existing allocated tiler block
 */
void tilview_get(struct tiler_view_t *view, struct tiler_block_t *blk);

/**
 * Crops a tiler view to a rectangular portion. Crop area must be fully within
 * the orginal tiler view: 0 <= left <= left + width <= view->width, also:
 * 0 <= top <= top + height <= view->height.
 *
 * @param view		Pointer to tiler view to be cropped
 * @param left		x of top-left corner
 * @param top		y of top-left corner
 * @param width		crop width
 * @param height	crop height
 *
 * @return error status.  The view will be reduced to the crop region if the
 *	   crop region is correct.  Otherwise, no modifications are made.
 */
s32 tilview_crop(struct tiler_view_t *view, u32 left, u32 top, u32 width,
								u32 height);

/**
 * Rotates a tiler view clockwise by a specified degree.
 *
 * @param view		Pointer to tiler view to be cropped
 * @param rotate	Degree of rotation (clockwise).  Must be a multiple of
 *			90.
 * @return error status.  View is not modified on error; otherwise, it is
 *	   updated in place.
 */
s32 tilview_rotate(struct tiler_view_t *view, s32 rotation);

/**
 * Mirrors a tiler view horizontally and/or vertically.
 *
 * @param view		Pointer to tiler view to be cropped
 * @param flip_x	Mirror horizontally (left-to-right)
 * @param flip_y	Mirror vertically (top-to-bottom)
 *
 * @return error status.  View is not modified on error; otherwise, it is
 *	   updated in place.
 */
s32 tilview_flip(struct tiler_view_t *view, bool flip_x, bool flip_y);

/*
 * -------------------- TILER hooks for ION/HWC migration --------------------
 */

/* type of tiler memory */
enum tiler_memtype {
	TILER_MEM_ALLOCED,		/* tiler allocated the memory */
	TILER_MEM_GOT_PAGES,		/* tiler used get_user_pages */
	TILER_MEM_USING,		/* tiler is using the pages */
};

/* physical pages to pin - mem must be kmalloced */
struct tiler_pa_info {
	u32 num_pg;			/* number of pages in page-list */
	u32 *mem;			/* list of phys page addresses */
	enum tiler_memtype memtype;	/* how we got physical pages */
};

typedef struct mem_info *tiler_blk_handle;

/**
 * Allocate a 1D area of container space in the Tiler
 *
 * @param pa		ptr to tiler_pa_info structure
 *
 * @return handle	Handle to tiler block information.  NULL on error.
 *
 * NOTE: this will take ownership pa->mem (will free it)
 *
 */
tiler_blk_handle tiler_map_1d_block(struct tiler_pa_info *pa);

/**
 * Allocate an area of container space in the Tiler
 *
 * @param fmt		Tiler bpp mode
 * @param width		Width in pixels
 * @param height	Height in pixels
 * @param ssptr		Value of tiler physical address of allocation
 * @param virt_array	Array of physical address for the start of each virtual
			page
 *
 * @return handle	Handle to tiler block information.  NULL on error.
 *
 * NOTE: For 1D allocations, specify the full size in the width field, and
 *       specify a height of 1.
 */
tiler_blk_handle tiler_alloc_block_area(enum tiler_fmt fmt, u32 width,
					u32 height, u32 *ssptr,
					u32 *virt_array);

/**
 * Free a reserved area in the Tiler
 *
 * @param handle	Handle to tiler block information
 *
 */
void tiler_free_block_area(tiler_blk_handle block);

/**
 * Pins a set of physical pages into the Tiler using the area defined in a
 * handle
 *
 * @param handle	Handle to tiler block information
 * @param addr_array	Array of addresses
 * @param nents		Number of addresses in array
 *
 * @return error status.
 */
s32 tiler_pin_block(tiler_blk_handle handle, u32 *addr_array, u32 nents);

/**
 * Unpins a set of physical pages from the Tiler
 *
 * @param handle	Handle to tiler block information
 *
 */
void tiler_unpin_block(tiler_blk_handle handle);

/**
 * Gives memory requirements for a given container allocation
 *
 * @param fmt		Tiler bpp mode
 * @param width		Width in pixels
 * @param height	Height in pixels
 * @param alloc_pages	Number of pages required to back tiler container
 * @param virt_pages    Number of pages required to back the virtual address space
 *
 * @return 0 for success.  Non zero for error
 */
s32 tiler_memsize(enum tiler_fmt fmt, u32 width, u32 height, u32 *alloc_pages,
		  u32 *virt_pages);

/**
 * Returns virtual stride of a tiler block
 *
 * @param handle	Handle to tiler block allocation
 *
 * @return Size of virtual stride
 */
u32 tiler_block_vstride(tiler_blk_handle handle);

struct tiler_pa_info *user_block_to_pa(u32 usr_addr, u32 num_pg);
void tiler_pa_free(struct tiler_pa_info *pa);

/*
 * ---------------------------- IOCTL Definitions ----------------------------
 */

/* ioctls */
#define TILIOC_GBLK  _IOWR('z', 100, struct tiler_block_info)
#define TILIOC_FBLK   _IOW('z', 101, struct tiler_block_info)
#define TILIOC_GSSP  _IOWR('z', 102, u32)
#define TILIOC_MBLK  _IOWR('z', 103, struct tiler_block_info)
#define TILIOC_UMBLK  _IOW('z', 104, struct tiler_block_info)
#define TILIOC_QBUF  _IOWR('z', 105, struct tiler_buf_info)
#define TILIOC_RBUF  _IOWR('z', 106, struct tiler_buf_info)
#define TILIOC_URBUF _IOWR('z', 107, struct tiler_buf_info)
#define TILIOC_QBLK  _IOWR('z', 108, struct tiler_block_info)
#define TILIOC_PRBLK  _IOW('z', 109, struct tiler_block_info)
#define TILIOC_URBLK  _IOW('z', 110, u32)

struct area {
	u16 width;
	u16 height;
};

/* userspace tiler block info */
struct tiler_block_info {
	enum tiler_fmt fmt;
	union {
		struct area area;
		u32 len;
	} dim;
	u32 stride;	/* stride is not maintained for 1D blocks */
	void *ptr;	/* userspace address for mapping existing buffer */
	u32 id;
	u32 key;
	u32 group_id;
	u32 ssptr;	/* physical address, may not exposed by default */
};

#define TILER_MAX_NUM_BLOCKS 16

/* userspace tiler buffer info */
struct tiler_buf_info {
	u32 num_blocks;
	struct tiler_block_info blocks[TILER_MAX_NUM_BLOCKS];
	u32 offset;
	u32 length;	/* also used as number of buffers for reservation */
};

#endif
