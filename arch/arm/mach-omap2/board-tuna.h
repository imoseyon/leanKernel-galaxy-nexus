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

#define TUNA_REV_MASK		0xf
#define TUNA_REV_03		0x3
#define TUNA_REV_SAMPLE_4	0x3

#define TUNA_TYPE_TORO		0x10
#define TUNA_TYPE_MAGURO	0x00
#define TUNA_TYPE_MASK		0x10

#define TUNA_GPIO_HDMI_HPD	63

int omap4_tuna_get_revision(void);
int omap4_tuna_get_type(void);
bool omap4_tuna_final_gpios(void);
void omap4_tuna_display_init(void);
void omap4_tuna_input_init(void);
void omap4_tuna_jack_init(void);
void omap4_tuna_nfc_init(void);
void omap4_tuna_power_init(void);
void omap4_tuna_sensors_init(void);
void omap4_tuna_pogo_init(void);
int omap4_tuna_connector_init(void);
int tuna_wlan_init(void);
int omap_hsi_init(void);
void omap4_tuna_emif_init(void);
void omap4_ehci_init(void);
void modem_toro_init(void);

void tuna_otg_pogo_charger(bool on);

extern struct mmc_platform_data tuna_wifi_data;
extern struct class *sec_class;

#endif
