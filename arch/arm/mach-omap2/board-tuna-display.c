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

static const u8 tuna_oled_cmd_init_pre[] = {
	0xF0,
	0x5A,
	0x5A,
};

static const u8 tuna_oled_cmd_sleep_out[] = {
	0x11,
};

static const u8 tuna_oled_cmd_init_panel[] = {
	0xF8, /* Panel Condition Set */
	0x3D, /* DOTC[0:1], GTCON[2:4], SS, DOTC_H[6:7] */
	0x35, /* FLTE[0:7] */
	0x00,
	0x00,
	0x00,
	0x8D,
	0x00,
	0x4C, /* SCTE[0:7] */
	0x6E,
	0x10,
	0x27,
	0x7D, /* INTE[0:7] */
	0x3F, /* INWE[0:7] */
	0x10,
	0x00,
	0x00,
	0x20,
	0x04, /* E_FLWE_H[0:7] */
	0x08, /* E_SCTE[0:7] */
	0x6E, /* E_SCWE[0:7] */
	0x00,
	0x00,
	0x00,
	0x02,
	0x08,
	0x08,
	0x23,
	0x23,
	0xC0,
	0xC8, /* CLK2_CON[0:2], CLK1_CON[3:5], CLK2_DC, CLK1_DC */
	0x08, /* INT2_CON[0:2], INT1_CON[3:5], INT2_DC, INT1_DC */
	0x48, /* BICTLB_CON[0:2], BICTL_CON[3:5], BICTLB_DC, BICTL_DC */
	0xC1,
	0x00,
	0xC3, /* EM_FLM_CON[0:2], ACL_FLM_CON[3:5], EM_FLM_DC, ACL_FLM_DC */
	0xFF, /* EM_CLK1B_CON[0:2], EM_CLK1_CON[3:5], EM_CLK1B_DC, EM_CLK1_DC */
	0xFF, /* EM_CLK2B_CON[0:2], EM_CLK2_CON[3:5], EM_CLK2B_DC, EM_CLK2_DC */
	0xC8, /* EM_INT2_CON[0:2], EM_INT1_CON[3:5], EM_INT2_DC, EM_INT1_DC */
};

static const u8 tuna_oled_cmd_init_display[] = {
	0xF2, /* Display Condition set */
	0x80, /* Display area */
	0x03, /* VBP : 3 HsYNC */
	0x0D, /* VFP : 13HSYNC */
};

static const struct s6e8aa0_sequence_entry tuna_oled_seq_display_set[] = {
	{
		.cmd = tuna_oled_cmd_init_pre,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_pre),
	},
	{
		.cmd = tuna_oled_cmd_sleep_out,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_sleep_out),
	},
	{
		.cmd = tuna_oled_cmd_init_panel,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_panel),
	},
	{
		.cmd = tuna_oled_cmd_init_display,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_display),
	},
};

static const u8 tuna_oled_cmd_gamma_ltps_update[] = {
	0xF7,
	0x03, /* Gamma/LTPS update */
};

static const u8 tuna_oled_cmd_init_post0[] = {
	0xF6,
	0x00,
	0x02,
	0x00,
};

static const u8 tuna_oled_cmd_init_post1[] = {
	0xB6,
	0x0C,
	0x02,
	0x03,
	0x32,
	0xFF,
	0x44,
	0x44,
	0xC0,
	0x00,
};

static const u8 tuna_oled_cmd_init_post2[] = {
	0xD9,
	0x14,
	0x40,
	0x0C,
	0xCB,
	0xCE,
	0x6E,
	0xC4,
	0x07, /* COLUMN_CHOP, FRAME_CHOP, LINE_CHOP, CHOP_EN */
	0x40,
	0x40,
	0xD0, /* ELVSS -4.9V */
	0x00,
	0x60,
	0x19,
};

static const u8 tuna_oled_cmd_display_on[] = {
	0x29,
};

static const struct s6e8aa0_sequence_entry tuna_oled_seq_etc_set[] = {
	{
		.cmd = tuna_oled_cmd_gamma_ltps_update,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_gamma_ltps_update),
	},
	{
		.cmd = tuna_oled_cmd_init_post0,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_post0),
	},
	{
		.cmd = tuna_oled_cmd_init_post1,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_post1),
	},
	{
		.cmd = tuna_oled_cmd_init_post2,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_post2),
		.msleep = 120,
	},
	{
		.cmd = tuna_oled_cmd_display_on,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_display_on),
	},
};

static const struct s6e8aa0_gamma_entry tuna_oled_gamma_table[] = {
	{       BV_0, { 4500000, 4500000, 4500000, }, },
	{ 0x00000001, { 4350000, 4350000, 4350000, }, },
	{ 0x0001F8F0, { 4320166, 4338185, 4200000, }, },
	{ 0x0002F71F, { 4305148, 4332238, 4102500, }, },
	{ 0x00038485, { 4296793, 4328930, 4065000, }, },
	{ 0x00053C55, { 4270807, 4318639, 3982500, }, },
	{ 0x00060EEF, { 4258364, 4313711, 3960000, }, },
	{ 0x00075444, { 4239141, 4306099, 3930000, }, },
	{ 0x00095733, { 4208717, 4294050, 3892500, }, },
	{ 0x000EC060, { 4126874, 4261640, 3810000, }, },
	{ 0x00129EAB, { 4068363, 4238469, 3757500, }, },
	{ 0x00179214, { 3993478, 4208813, 3720000, }, },
	{ 0x001A47A0, { 3952500, 4192586, 3712500, }, },
	{ 0x0025E12C, { 3802500, 4123103, 3682500, }, },
	{ 0x002D413D, { 3756465, 4078926, 3673450, }, },
	{ 0x0035D13F, { 3741586, 4027637, 3659844, }, },
	{ 0x00400000, { 3726401, 3966643, 3645758, }, },
	{ 0x004C1BF8, { 3710894, 3894110, 3631166, }, },
	{ 0x005A827A, { 3695052, 3855649, 3616042, }, },
	{ 0x006BA27E, { 3678859, 3823316, 3600360, }, },
	{ 0x00800000, { 3662298, 3794488, 3584091, }, },
	{ 0x009837F0, { 3645351, 3767841, 3567202, }, },
	{ 0x00B504F3, { 3627999, 3742607, 3549662, }, },
	{ 0x00D744FD, { 3610220, 3718295, 3531434, }, },
	{ 0x01000000, { 3591992, 3694568, 3512481, }, },
	{ 0x01306FE1, { 3573291, 3671183, 3492761, }, },
	{ 0x016A09E6, { 3554089, 3647957, 3472232, }, },
	{ 0x01AE89FA, { 3534358, 3624743, 3450847, }, },
	{ 0x02000000, { 3514065, 3601422, 3428554, }, },
	{ 0x0260DFC1, { 3493177, 3577893, 3405301, }, },
	{ 0x02D413CD, { 3471654, 3554067, 3381029, }, },
	{ 0x035D13F3, { 3449455, 3529864, 3355676, }, },
	{ 0x04000000, { 3426534, 3505211, 3329174, }, },
	{ 0x04C1BF83, { 3402839, 3480034, 3301451, }, },
	{ 0x05A8279A, { 3378312, 3454264, 3272429, }, },
	{ 0x06BA27E6, { 3352890, 3427831, 3242021, }, },
	{ 0x08000000, { 3326501, 3400661, 3210138, }, },
	{ 0x09837F05, { 3299062, 3372679, 3176678, }, },
	{ 0x0B504F33, { 3270483, 3343804, 3141535, }, },
	{ 0x0D744FCD, { 3240658, 3313948, 3104592, }, },
	{ 0x10000000, { 3209466, 3283015, 3065722, }, },
	{ 0x1306FE0A, { 3176767, 3250899, 3024787, }, },
	{ 0x16A09E66, { 3142400, 3217481, 2981638, }, },
	{ 0x1AE89F99, { 3106171, 3182624, 2936111, }, },
	{ 0x20000000, { 3067855, 3146174, 2888031, }, },
	{ 0x260DFC14, { 3027178, 3107951, 2837203, }, },
	{ 0x2D413CCD, { 2983809, 3067744, 2783420, }, },
	{ 0x35D13F32, { 2937340, 3025303, 2726454, }, },
	{ 0x40000000, { 2887259, 2980328, 2666057, }, },
	{ 0x4C1BF828, { 2832912, 2932454, 2601961, }, },
	{ 0x5A82799A, { 2773444, 2881230, 2533878, }, },
	{ 0x6BA27E65, { 2707706, 2826086, 2461495, }, },
	{ 0x80000000, { 2639098, 2766291, 2384478, }, },
	{ 0x9837F051, { 2560291, 2700878, 2302469, }, },
	{ 0xB504F333, { 2472698, 2628531, 2215093, }, },
	{ 0xD744FCCA, { 2375331, 2547389, 2121959, }, },
	{ 0xFFFFFFFF, { 2266945, 2454682, 2022667, }, },
};

static struct s6e8aa0_factory_calibration_info tuna_oled_factory_info_old = {
	.regs = {
		[1][0][6] = 0x090,
		[1][1][6] = 0x081,
		[1][2][6] = 0x0c5,
	},
	.brightness = {
		[1][6] = BV_255,	/* 300 cd/m2 */
	},
	.color_adj = {
		/* Convert from 8500K to D65, assuming:
		 * Rx 0.66950, Ry 0.33100
		 * Gx 0.18800, Gy 0.74350
		 * Bx 0.14142, By 0.04258
		 */
		.mult = {
			2318372099U,
			2117262806U,
			1729744557U,
		},
		.rshift = 31,
	},
};

static struct s6e8aa0_factory_calibration_info tuna_oled_factory_info_m2t1 = {
	.regs = {
		[1][0][6] = 0x090,
		[1][1][6] = 0x081,
		[1][2][6] = 0x0c5,
	},
	.brightness = {
		[1][6] = BV_255,	/* 300 cd/m2 */
	},
};

static struct s6e8aa0_factory_calibration_info tuna_oled_factory_info_8500k = {
	.regs = {
		[1][0][0] = 0x0f,
		[1][1][0] = 0x0f,
		[1][2][0] = 0x0f,

		[1][0][1] = 0xcc,
		[1][1][1] = 0x9c,
		[1][2][1] = 0xd7,

		[1][0][2] = 0xc1,
		[1][1][2] = 0xba,
		[1][2][2] = 0xc1,

		[1][0][3] = 0xcf,
		[1][1][3] = 0xcd,
		[1][2][3] = 0xcf,

		[1][0][4] = 0xaa,
		[1][1][4] = 0xaa,
		[1][2][4] = 0xa7,

		[1][0][5] = 0xbe,
		[1][1][5] = 0xbe,
		[1][2][5] = 0xba,

		[1][0][6] = 0x090,
		[1][1][6] = 0x081,
		[1][2][6] = 0x0c5,
	},
	.brightness = {
		[1][4] = 403193777,	/* 28.16275996 cd/m2 */
		[1][6] = BV_255,	/* 300 cd/m2 */
	},
	.color_adj = {
		/* Convert from 8500K to D65, assuming:
		 * Rx 0.66950, Ry 0.33100
		 * Gx 0.18800, Gy 0.74350
		 * Bx 0.14142, By 0.04258
		 */
		.mult = {
			2318372099U,
			2117262806U,
			1729744557U,
		},
		.rshift = 31,
	},
};

static struct panel_s6e8aa0_data tuna_oled_data = {
	.reset_gpio	= TUNA_GPIO_MLCD_RST,
	.set_power	= tuna_oled_set_power,
	.seq_display_set = tuna_oled_seq_display_set,
	.seq_display_set_size = ARRAY_SIZE(tuna_oled_seq_display_set),
	.seq_etc_set = tuna_oled_seq_etc_set,
	.seq_etc_set_size = ARRAY_SIZE(tuna_oled_seq_etc_set),
	.gamma_table = tuna_oled_gamma_table,
	.gamma_table_size = ARRAY_SIZE(tuna_oled_gamma_table),
	.factory_info = &tuna_oled_factory_info_8500k,
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
		omap4_ctrl_pad_writel(0x1FF80000, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
		if (!omap4_tuna_final_gpios())
			tuna_oled_data.reset_gpio = TUNA_GPIO_MLCD_RST_LUNCHBOX;
		omap_mux_init_gpio(tuna_oled_data.reset_gpio, OMAP_PIN_OUTPUT);
		dss_data = &tuna_dss_data;
	}

	if (omap4_tuna_get_revision() ==
	    (omap4_tuna_get_type() == TUNA_TYPE_MAGURO ? 2 : 1)) {
		/*
		 * Older devices were not calibrated the same way as newer
		 * devices. These values are probably not correct, but the older
		 * devices tested look closer to the newer devices with these
		 * values than they do using the same register values as the
		 * newer devices.
		 */
		tuna_oled_data.factory_info = &tuna_oled_factory_info_m2t1;
	} else if (omap4_tuna_get_revision() <= 1) {
		tuna_oled_data.factory_info = &tuna_oled_factory_info_old;
	}
	pr_info("Using %ps\n", tuna_oled_data.factory_info);

	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	omap_display_init(dss_data);
}
