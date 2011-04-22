/* Display panel support for Samsung Tuna Board.
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
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/omapfb.h>

#include <plat/display.h>
#include <plat/panel-generic-dpi.h>
#include <plat/vram.h>

#define TUNA_FB_RAM_SIZE		SZ_8M /* ~1280*720*4 * 2 */

static struct panel_generic_dpi_data tuna_lcd_panel = {
	.name			= "samsung_ams452gn05",
	.platform_enable        = NULL,
	.platform_disable       = NULL,
};

static struct omap_dss_device tuna_lcd_device = {
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.data			= &tuna_lcd_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *tuna_dss_devices[] = {
	&tuna_lcd_device,
};

static struct omap_dss_board_info tuna_dss_data = {
	.num_devices	= ARRAY_SIZE(tuna_dss_devices),
	.devices	= tuna_dss_devices,
	.default_device	= &tuna_lcd_device,
};

static struct omapfb_platform_data tuna_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = TUNA_FB_RAM_SIZE,
			},
		},
	},
};

void __init omap4_tuna_display_init(void)
{
	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	omap_display_init(&tuna_dss_data);
}
