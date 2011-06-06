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
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/omapfb.h>
#include <linux/regulator/consumer.h>

#include <linux/platform_data/panel-s6e8aa0.h>

#include <plat/vram.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include "board-tuna.h"
#include "control.h"
#include "mux.h"

#define TUNA_FB_RAM_SIZE		SZ_16M /* ~1280*720*4 * 2 */

#define TUNA_GPIO_MLCD_RST_LUNCHBOX	35
#define TUNA_GPIO_MLCD_RST		23

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

struct regulator *tuna_oled_reg;

static void tuna_oled_set_power(bool enable)
{
	if (IS_ERR_OR_NULL(tuna_oled_reg)) {
		tuna_oled_reg = regulator_get(NULL, "vlcd");
		if (IS_ERR_OR_NULL(tuna_oled_reg)) {
			pr_err("Can't get vlcd for display!\n");
			return;
		}
	}

	if (enable)
		regulator_enable(tuna_oled_reg);
	else
		regulator_disable(tuna_oled_reg);
}

static struct panel_s6e8aa0_data tuna_oled_data = {
	.reset_gpio	= TUNA_GPIO_MLCD_RST,
	.set_power	= tuna_oled_set_power,
};

/* width: 58mm */
/* height: 102mm */
static struct omap_dss_device tuna_oled_device = {
	.name			= "lcd",
	.driver_name		= "s6e8aa0",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &tuna_oled_data,
	.phy.dsi		= {
		.type		= OMAP_DSS_DSI_TYPE_VIDEO_MODE,
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.data3_lane	= 4,
		.data3_pol	= 0,
		.data4_lane	= 5,
		.data4_pol	= 0,
	},
	.clocks = {
		.dispc		= {
			.channel = {
				.lck_div	= 1,	/* LCD */
				.pck_div	= 2,	/* PCD */
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},
		.dsi		= {
			.regn		= 19,	/* DSI_PLL_REGN */
			.regm		= 240,	/* DSI_PLL_REGM */

			.regm_dispc	= 6,	/* PLL_CLK1 (M4) */
			.regm_dsi	= 6,	/* PLL_CLK2 (M5) */
			.lp_clk_div	= 14,	/* LPDIV */

			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},

	.channel		= OMAP_DSS_CHANNEL_LCD,
};


static struct omap_dss_device *tuna_dss_devices[] = {
	&tuna_oled_device,
};

static struct omap_dss_board_info tuna_dss_data = {
	.num_devices	= ARRAY_SIZE(tuna_dss_devices),
	.devices	= tuna_dss_devices,
	.default_device	= &tuna_oled_device,
};

static struct omap_dss_device *prelunchbox_dss_devices[] = {
	&tuna_lcd_device,
};

static struct omap_dss_board_info prelunchbox_dss_data = {
	.num_devices	= ARRAY_SIZE(prelunchbox_dss_devices),
	.devices	= prelunchbox_dss_devices,
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
	struct omap_dss_board_info *dss_data;

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

		dss_data = &prelunchbox_dss_data;
	} else {
		omap4_ctrl_pad_writel(0x1F000000, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
		if (!omap4_tuna_final_gpios())
			tuna_oled_data.reset_gpio = TUNA_GPIO_MLCD_RST_LUNCHBOX;
		omap_mux_init_gpio(tuna_oled_data.reset_gpio, OMAP_PIN_OUTPUT);
		dss_data = &tuna_dss_data;
	}

	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	omap_display_init(dss_data);
}
