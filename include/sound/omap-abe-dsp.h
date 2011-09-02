/*
 * omap-aess  --  OMAP4 ABE DSP
 *
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _OMAP4_ABE_DSP_H
#define _OMAP4_ABE_DSP_H

struct omap4_abe_dsp_pdata {
	bool (*was_context_lost)(struct device *dev);
	int (*device_scale)(struct device *req_dev,
			    struct device *target_dev,
			    unsigned long rate);
};

#endif
