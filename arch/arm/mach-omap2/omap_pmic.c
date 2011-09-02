/*
 * Registration hooks for PMICs used with OMAP
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "voltage.h"

#include "pm.h"

/**
 * omap_pmic_data_init() - trigger point for all PMIC initializers
 */
void __init omap_pmic_data_init(void)
{
	omap_twl_init();
	omap_tps6236x_init();
}

/**
 * omap_pmic_register_data() - Register the PMIC information to OMAP mapping
 * @omap_pmic_maps:	array ending with a empty element representing the maps
 * @desc:		description for this PMIC.
 */
int __init omap_pmic_register_data(struct omap_pmic_map *omap_pmic_maps,
				   struct omap_pmic_description *desc)
{
	struct voltagedomain *voltdm;
	struct omap_pmic_map *map;
	int r;

	if (!omap_pmic_maps)
		return 0;

	map = omap_pmic_maps;

	while (map->name) {
		if (!omap_chip_is(map->omap_chip))
			goto next;

		/* The base PMIC is the one controlling core voltdm */
		if (desc && !strcmp(map->name, "core"))
			omap_pm_set_pmic_lp_time(desc->pmic_lp_tstart,
						 desc->pmic_lp_tshut);

		voltdm = voltdm_lookup(map->name);
		if (IS_ERR_OR_NULL(voltdm)) {
			pr_err("%s: unable to find map %s\n", __func__,
				map->name);
			goto next;
		}
		if (IS_ERR_OR_NULL(map->pmic_data)) {
			pr_warning("%s: domain[%s] has no pmic data\n",
					__func__, map->name);
			goto next;
		}

		r = omap_voltage_register_pmic(voltdm, map->pmic_data);
		if (r) {
			pr_warning("%s: domain[%s] register returned %d\n",
					__func__, map->name, r);
			goto next;
		}
		if (map->special_action) {
			r = map->special_action(voltdm);
			WARN(r, "%s: domain[%s] action returned %d\n", __func__,
				map->name, r);
		}
next:
		map++;
	}

	return 0;
}

int __init omap_pmic_update(struct omap_pmic_map *tmp_map, char *name,
				u32 old_chip_id, u32 new_chip_id)
{
	while (tmp_map->name != NULL) {
		if (!strcmp(tmp_map->name, name) &&
				(tmp_map->omap_chip.oc & new_chip_id)) {
			WARN(1, "%s: this map already exists:%s-%x\n",
					__func__, name, new_chip_id);
			return -1;
		}
		if (!strcmp(tmp_map->name, name) &&
				(tmp_map->omap_chip.oc & old_chip_id))
			break;
		tmp_map++;
	}
	if (tmp_map->name != NULL) {
		tmp_map->omap_chip.oc = new_chip_id;
		return 0;
	}
	return -ENOENT;
}
