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

#include "board-tuna.h"
#include "mux.h"

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

#define MUX_DISPLAY_OUT OMAP_PIN_OUTPUT | OMAP_MUX_MODE5
void __init omap4_tuna_display_init(void)
{
	if (omap4_tuna_get_revision() == TUNA_REV_PRE_LUNCHBOX) {
		/* dispc2_data23 - dispc2_data0 */
		omap_mux_init_signal("usbb2_ulpitll_stp", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dir", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_nxt", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat0", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat1", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat2", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu6", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu5", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat3", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat4", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat5", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat6", MUX_DISPLAY_OUT);
		omap_mux_init_signal("usbb2_ulpitll_dat7", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu3", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu4", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu11", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu12", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu13", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu14", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu15", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu16", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu17", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu18", MUX_DISPLAY_OUT);
		omap_mux_init_signal("dpm_emu19", MUX_DISPLAY_OUT);
		/* dispc2_hsync */
		omap_mux_init_signal("dpm_emu7", MUX_DISPLAY_OUT);
		/* dispc2_pclk */
		omap_mux_init_signal("dpm_emu8", MUX_DISPLAY_OUT);
		/* dispc2_vsync */
		omap_mux_init_signal("dpm_emu9", MUX_DISPLAY_OUT);
		/* dispc2_de */
		omap_mux_init_signal("dpm_emu10", MUX_DISPLAY_OUT);
	}

	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	omap_display_init(&tuna_dss_data);
}
