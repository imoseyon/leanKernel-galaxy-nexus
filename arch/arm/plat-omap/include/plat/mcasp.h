/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __OMAP_PLAT_MCASP_H__
#define __OMAP_PLAT_MCASP_H__

#include <linux/platform_device.h>

/* The SPDIF bit clock is derived from the McASP functional clock.
 * The McASP has two programmable clock dividers (aclkxdiv and
 * ahclkxdiv) that are configured via the registers MCASP_ACLKXCTL
 * and MCASP_AHCLKXCTL. For SPDIF the bit clock frequency should be
 * 128 * sample rate freq. Therefore...
 *
 * McASP functional clock = aclkxdiv * ahclkxdiv * 128 * sample rate
 *
 * For each sample rate supported the user must define the aclkxdiv
 * and ahclkxdiv values that are passed to the McASP driver via the
 * following structure. The McASP functional clock frequency can be
 * configured also, and this is pass to the McASP driver via the
 * omap_mcasp_platform_data structure below.
 */
struct omap_mcasp_configs {
	unsigned int sampling_rate;
	u16 aclkxdiv;
	u16 ahclkxdiv;
};

struct omap_mcasp_platform_data {
	unsigned long mcasp_fclk_rate;
	struct omap_mcasp_configs *mcasp_configs;
	unsigned int num_configs;
};

void omap_init_mcasp(struct omap_mcasp_platform_data *pdata);

#endif
