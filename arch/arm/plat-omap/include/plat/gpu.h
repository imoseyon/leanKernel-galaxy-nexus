/*
 *  arch/arm/plat-omap/include/plat/gpu.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef OMAP_GPU_H
#define OMAP_GPU_H

#include <plat/omap-pm.h>
#include <linux/platform_device.h>

struct gpu_platform_data {
	void (*set_min_bus_tput)(struct device *dev, u8 agent_id,
						unsigned long r);
	int (*device_enable) (struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle) (struct platform_device *pdev);
};

#endif
