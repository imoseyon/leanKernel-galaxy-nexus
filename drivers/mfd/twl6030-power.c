/*
 * Handling for Resource Mapping for TWL6030 Family of chips
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>

#define VREG_GRP		0

/**
 * struct twl6030_resource_map - describe the resource mapping for TWL6030
 * @name:	name of the resource
 * @res_id:	resource ID
 * @base_addr:	base address
 * @group:	which device group can control this resource?
 */
struct twl6030_resource_map {
	char *name;
	u8 res_id;
	u8 base_addr;
	u8 group;
};

/* list of all s/w modifiable resources in TWL6030 */
static __initdata struct twl6030_resource_map twl6030_res_map[] = {
	{.res_id = RES_V1V29,.name = "V1V29",.base_addr = 0x40,.group = DEV_GRP_P1,},
	{.res_id = RES_V1V8,.name = "V1V8",.base_addr = 0x46,.group = DEV_GRP_P1,},
	{.res_id = RES_V2V1,.name = "V2V1",.base_addr = 0x4c,.group = DEV_GRP_P1,},
	{.res_id = RES_VDD1,.name = "CORE1",.base_addr = 0x52,.group = DEV_GRP_P1,},
	{.res_id = RES_VDD2,.name = "CORE2",.base_addr = 0x58,.group = DEV_GRP_P1,},
	{.res_id = RES_VDD3,.name = "CORE3",.base_addr = 0x5e,.group = DEV_GRP_P1,},
	{.res_id = RES_VMEM,.name = "VMEM",.base_addr = 0x64,.group = DEV_GRP_P1,},
	/* VANA cannot be modified */
	{.res_id = RES_VUAX1,.name = "VUAX1",.base_addr = 0x84,.group = DEV_GRP_P1,},
	{.res_id = RES_VAUX2,.name = "VAUX2",.base_addr = 0x88,.group = DEV_GRP_P1,},
	{.res_id = RES_VAUX3,.name = "VAUX3",.base_addr = 0x8c,.group = DEV_GRP_P1,},
	{.res_id = RES_VCXIO,.name = "VCXIO",.base_addr = 0x90,.group = DEV_GRP_P1,},
	{.res_id = RES_VDAC,.name = "VDAC",.base_addr = 0x94,.group = DEV_GRP_P1,},
	{.res_id = RES_VMMC1,.name = "VMMC",.base_addr = 0x98,.group = DEV_GRP_P1,},
	{.res_id = RES_VPP,.name = "VPP",.base_addr = 0x9c,.group = DEV_GRP_P1,},
	/* VRTC cannot be modified */
	{.res_id = RES_VUSBCP,.name = "VUSB",.base_addr = 0xa0,.group = DEV_GRP_P1,},
	{.res_id = RES_VSIM,.name = "VSIM",.base_addr = 0xa4,.group = DEV_GRP_P1,},
	{.res_id = RES_REGEN,.name = "REGEN1",.base_addr = 0xad,.group = DEV_GRP_P1,},
	{.res_id = RES_REGEN2,.name = "REGEN2",.base_addr = 0xb0,.group = DEV_GRP_P1,},
	{.res_id = RES_SYSEN,.name = "SYSEN",.base_addr = 0xb3,.group = DEV_GRP_P1,},
	/* NRES_PWRON cannot be modified */
	/* 32KCLKAO cannot be modified */
	{.res_id = RES_32KCLKG,.name = "32KCLKG",.base_addr = 0xbc,.group = DEV_GRP_P1,},
	{.res_id = RES_32KCLKAUDIO,.name = "32KCLKAUDIO",.base_addr = 0xbf,.group = DEV_GRP_P1,},
	/* BIAS cannot be modified */
	/* VBATMIN_HI cannot be modified */
	/* RC6MHZ cannot be modified */
	/* TEMP cannot be modified */
};

/* Actual power groups that TWL understands */
#define P3_GRP_6030	BIT(2)		/* secondary processor, modem, etc */
#define P2_GRP_6030	BIT(1)		/* "peripherals" */
#define P1_GRP_6030	BIT(0)		/* CPU/Linux */

static __init void twl6030_program_map(void)
{
	struct twl6030_resource_map *res = twl6030_res_map;
	int r, i;

	for (i = 0; i < ARRAY_SIZE(twl6030_res_map); i++) {
		u8 grp = 0;

		/* map back from generic device id to TWL6030 ID */
		grp |= (res->group & DEV_GRP_P1) ? P1_GRP_6030 : 0;
		grp |= (res->group & DEV_GRP_P2) ? P2_GRP_6030 : 0;
		grp |= (res->group & DEV_GRP_P3) ? P3_GRP_6030 : 0;

		r = twl_i2c_write_u8(TWL6030_MODULE_ID0, res->group,
				     res->base_addr);
		if (r)
			pr_err("%s: Error(%d) programming map %s {addr=0x%02x},"
			       "grp=0x%02X\n", __func__, r, res->name,
			       res->base_addr, res->group);
		res++;
	}
}

static __init void twl6030_update_map(struct twl4030_resconfig *res_list)
{
	int i, res_idx = 0;
	struct twl6030_resource_map *res;

	while (res_list->resource != TWL4030_RESCONFIG_UNDEF) {
		res = twl6030_res_map;
		for (i = 0; i < ARRAY_SIZE(twl6030_res_map); i++) {
			if (res->res_id == res_list->resource) {
				res->group = res_list->devgroup &
				    (DEV_GRP_P1 | DEV_GRP_P2 | DEV_GRP_P3);
				break;
			}
			res++;
		}

		if (i == ARRAY_SIZE(twl6030_res_map)) {
			pr_err("%s: in platform_data resource index %d, cannot"
			       " find match for resource 0x%02x. NO Update!\n",
			       __func__, res_idx, res_list->resource);
		}
		res_list++;
		res_idx++;
	}
}

/**
 * twl6030_power_init() - Update the power map to reflect connectivity of board
 * @power_data:	power resource map to update (OPTIONAL) - use this if a resource
 *		is used by other devices other than APP (DEV_GRP_P1)
 */
void __init twl6030_power_init(struct twl4030_power_data *power_data)
{
	if (power_data && !power_data->resource_config) {
		pr_err("%s: power data from platform without resources!\n",
		       __func__);
		return;
	}

	if (power_data)
		twl6030_update_map(power_data->resource_config);

	twl6030_program_map();

	return;
}
