/*
 * arch/arm/mach-omap2/resetreason.c
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

#include <linux/kernel.h>
#include <linux/string.h>
#include "prm-regbits-44xx.h"
#include "prcm44xx.h"
#include "prm44xx.h"
#include "prminst44xx.h"
#include "resetreason.h"

static char resetreason[1024];

static struct {
	const char *str;
	u32 mask;
} resetreason_flags[] = {
	{ "C2C ",			OMAP4430_C2C_RST_MASK },
	{ "IcePick ",			OMAP4430_ICEPICK_RST_MASK },
	{ "Voltage Manager ",		OMAP4430_VDD_MPU_VOLT_MGR_RST_MASK |
					OMAP4430_VDD_IVA_VOLT_MGR_RST_MASK |
					OMAP4430_VDD_CORE_VOLT_MGR_RST_MASK },
	{ "external warm ",		OMAP4430_EXTERNAL_WARM_RST_MASK },
	{ "MPU Watchdog Timer ",	OMAP4430_MPU_WDT_RST_MASK },
	{ "warm software ",		OMAP4430_GLOBAL_WARM_SW_RST_MASK },
	{ "cold ",			OMAP4430_GLOBAL_COLD_RST_MASK },
};

const char *omap4_get_resetreason(void)
{
	return resetreason;
}

static int __init resetreason_init(void)
{
	int i;
	u32 reasons =
		omap4_prminst_read_inst_reg(OMAP4430_PRM_PARTITION,
					    OMAP4430_PRM_DEVICE_INST,
					    OMAP4_PRM_RSTST_OFFSET);
	char buf[128];

	strlcpy(resetreason, "Last reset was ", sizeof(resetreason));

	for (i = 0; i < ARRAY_SIZE(resetreason_flags); i++)
		if (reasons & resetreason_flags[i].mask)
			strlcat(resetreason, resetreason_flags[i].str,
				sizeof(resetreason));

	snprintf(buf, sizeof(buf), "reset (PRM_RSTST=0x%x)\n", reasons);

	strlcat(resetreason, buf, sizeof(resetreason));

	pr_info("%s\n", resetreason);

	omap4_prminst_write_inst_reg(reasons, OMAP4430_PRM_PARTITION,
				     OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_RSTST_OFFSET);

	return 0;
}

postcore_initcall(resetreason_init);
