/*
 * Copyright (c) 2006-2010 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/mman.h>
#include "scxlnx_util.h"

void *internal_kmalloc(size_t nSize, int nPriority)
{
	void *pResult;
	struct SCXLNX_DEVICE *pDevice = SCXLNXGetDevice();

	pResult = kmalloc(nSize, nPriority);

	if (pResult != NULL)
		atomic_inc(
			&pDevice->sDeviceStats.stat_memories_allocated);

	return pResult;
}

void internal_kfree(void *pMemory)
{
	struct SCXLNX_DEVICE *pDevice = SCXLNXGetDevice();

	if (pMemory != NULL)
		atomic_dec(
			&pDevice->sDeviceStats.stat_memories_allocated);
	return kfree(pMemory);
}

