/*
 * ALSA SoC McASP Audio Layer for TI OMAP processor
 *
 * MCASP related definitions
 *
 * Author: Jon Hunter <jon-hunter@ti.com>,
 *         Dan Milea <dan.milea@ti.com>,
 *
 * Based upon McASP driver written for TI DaVinci
 *
 * Copyright:   (C) 2011  Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef OMAP_MCASP_H
#define OMAP_MCASP_H

#include <linux/io.h>
#include <plat/mcasp.h>

#define OMAP44XX_MCASP_CFG_BASE		0x49028000
#define OMAP44XX_MCASP_DAT_BASE		0x4902A000

struct omap_mcasp {
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;
	struct clk *fclk;
	int irq;
	unsigned int stream_rate;
	struct pm_qos_request_list *pm_qos;
};

#endif	/* OMAP_MCASP_H */
