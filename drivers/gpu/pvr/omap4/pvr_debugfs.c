/*
 * linux/drivers/gpu/pvr/omap4/pvr_debugfs.c
 *
 * Copyright (C) 2011 Texas Instruments Inc.
 * Author: Gilles-Arnaud Bleu-Laine <gilles@ti.com>
 *
 * Some code and ideas taken from drm_debugfs.c
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

#include <linux/debugfs.h>
#include "services_headers.h"
#include "syslocal.h"

#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
extern struct platform_device *gpsPVRLDMDev;
#endif

u32 sgx_freq_index;
u32 curr_sgx_freq_index;
bool sgx_freq_locked;
u32 freq_transition_counter;
SYS_SPECIFIC_DATA *gpsSysSpecData;

void pvr_ov_freq_scaling(IMG_INT32 freq_index);

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	if (option == &sgx_freq_index)
		sgx_freq_index = curr_sgx_freq_index;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	if (option == &sgx_freq_index)
		pvr_ov_freq_scaling((int)val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pvr_dbg_option_fops, option_get, option_set, "%llu\n");

/*
 * Remove debugfs entries created by pvr_debugfs_create_files().
 */
void pvr_debugfs_remove_files(struct dentry *debug_root)
{
	debugfs_remove(debug_root);
}

/*
 * Initialize debugfs files for a device
 */
int pvr_debugfs_create_files()
{
	int ret;
	struct dentry *ent;
	struct dentry *debug_root;

	debug_root = debugfs_create_dir("sgx", NULL);
	if (IS_ERR(debug_root))
		return PTR_ERR(debug_root);

	ent = debugfs_create_file("sgx_freq_index", S_IRWXUGO, debug_root,
							  &sgx_freq_index, &pvr_dbg_option_fops);
	if (IS_ERR(ent)) {
		printk("Failed to create sgx debug entry");
		ret = PTR_ERR(ent);
		goto fail;
	}

	return 0;

fail:
	pvr_debugfs_remove_files(debug_root);
	return ret;
}

/**
 * pvr_ov_freq_scaling - Set sgx frequency and prevent its transition
 *
 * Intended to be set by sysfs
 */
void pvr_ov_freq_scaling(IMG_INT32 freq_index)
{
#define UNLOCK -1
#define MAX_FREQ_INDEX 2

	struct gpu_platform_data *pdata;

	if (freq_index <= UNLOCK) {
		sgx_freq_locked = false;
		/*
		 * The frequency should transition to the last requested value
		 * received while it was being locked
		 */
		if (freq_transition_counter)
			freq_index = MAX_FREQ_INDEX;
		else
			freq_index = 0;
	}
	else {
		sgx_freq_locked = true;
	}

	if (freq_index > MAX_FREQ_INDEX)
		freq_index = MAX_FREQ_INDEX;

	pdata = (struct gpu_platform_data *)gpsPVRLDMDev->dev.platform_data;
	curr_sgx_freq_index = freq_index;

	/* Request frequency scaling */
	pdata->device_scale(&gpsPVRLDMDev->dev,
			    &gpsPVRLDMDev->dev,
			    gpsSysSpecData->pui32SGXFreqList[freq_index]);

#undef MAX_FREQ_INDEX
#undef UNLOCK
}
