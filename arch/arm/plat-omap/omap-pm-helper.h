/*
 * OMAP PM interface helpers
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __OMAP_PM_HELPER_INTERFACE_H__
#define __OMAP_PM_HELPER_INTERFACE_H__

#ifdef CONFIG_OMAP_PM
int omap_pm_set_min_bus_tput_helper(struct device *dev, u8 agent_id, long r);
int omap_pm_set_max_dev_wakeup_lat_helper(struct device *req_dev,
		struct device *dev, long t);
int __init omap_pm_if_init_helper(void);

#else
static inline int omap_pm_set_min_bus_tput_helper(struct device *dev,
		u8 agent_id, long r)
{
	return 0;
}

static inline int omap_pm_set_max_dev_wakeup_lat_helper(struct device *req_dev,
		struct device *dev, long t)
{
	return 0;
}

static inline int omap_pm_if_init_helper(void)
{
	return 0;
}
#endif	/* CONFIG_OMAP_PM */

#endif	/* __OMAP_PM_HELPER_INTERFACE_H__ */
