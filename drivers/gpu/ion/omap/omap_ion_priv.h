/*
 * include/linux/omap/omap_ion_priv.h
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_OMAP_ION_PRIV_H
#define _LINUX_OMAP_ION_PRIV_H

#include <linux/types.h>

int omap_tiler_alloc(struct ion_heap *heap,
		     struct ion_client *client,
		     struct omap_ion_tiler_alloc_data *data);
struct ion_heap *omap_tiler_heap_create(struct ion_platform_heap *heap_data);
void omap_tiler_heap_destroy(struct ion_heap *heap);

#endif /* _LINUX_OMAP_ION_PRIV_H */
