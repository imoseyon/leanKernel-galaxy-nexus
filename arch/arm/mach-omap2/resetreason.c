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
#include "prm-regbits-44xx.h"
#include "prcm44xx.h"
#include "prm44xx.h"
#include "prminst44xx.h"

static int __init resetreason_init(void)
{
	u32 reasons =
		omap4_prminst_read_inst_reg(OMAP4430_PRM_PARTITION,
					    OMAP4430_PRM_DEVICE_INST,
					    OMAP4_PRM_RSTST_OFFSET);

	pr_info("Last reset was ");

	if (reasons & OMAP4430_C2C_RST_MASK)
		pr_cont("C2C ");

	if (reasons & OMAP4430_ICEPICK_RST_MASK)
		pr_cont("IcePick ");

	if (reasons & (OMAP4430_VDD_MPU_VOLT_MGR_RST_MASK |
		       OMAP4430_VDD_IVA_VOLT_MGR_RST_MASK |
		       OMAP4430_VDD_CORE_VOLT_MGR_RST_MASK))
		pr_cont("Voltage Manager ");

	if (reasons & OMAP4430_EXTERNAL_WARM_RST_MASK)
		pr_cont("external warm ");

	if (reasons & OMAP4430_MPU_WDT_RST_MASK)
		pr_cont("MPU Watchdog Timer ");

	if (reasons & OMAP4430_GLOBAL_WARM_SW_RST_MASK)
		pr_cont("warm software ");
	else if (reasons & OMAP4430_GLOBAL_COLD_RST_MASK)
		pr_cont("cold ");


	pr_cont("reset (PRM_RSTST=0x%x)\n", reasons);

	omap4_prminst_write_inst_reg(reasons, OMAP4430_PRM_PARTITION,
				     OMAP4430_PRM_DEVICE_INST, OMAP4_PRM_RSTST_OFFSET);

	return 0;
}

arch_initcall(resetreason_init);
