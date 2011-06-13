/*
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
 */

#ifndef _MACH_OMAP2_BOARD_TUNA_H_
#define _MACH_OMAP2_BOARD_TUNA_H_

#define TUNA_REV_PRE_LUNCHBOX	0x5
#define TUNA_REV_LUNCHBOX	0x0
#define TUNA_REV_MASK		0xf

#define TUNA_TYPE_TORO		0x10
#define TUNA_TYPE_MAGURO	0x00
#define TUNA_TYPE_MASK		0x10

int omap4_tuna_get_revision(void);
int omap4_tuna_get_type(void);
bool omap4_tuna_final_gpios(void);
void omap4_tuna_display_init(void);
void omap4_tuna_input_init(void);
void omap4_tuna_power_init(void);
void omap4_tuna_sensors_init(void);
int tuna_wlan_init(void);
int omap_hsi_init(void);

extern struct mmc_platform_data tuna_wifi_data;

#endif
