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
	{          1, { 4294200, 4407600, 4210200, }, },
	{ 0x00000400, { 3969486, 4038030, 3955093, }, },
	{ 0x000004C2, { 3964456, 4032059, 3949872, }, },
	{ 0x000005A8, { 3959356, 4026019, 3944574, }, },
	{ 0x000006BA, { 3954160, 4019879, 3939171, }, },
	{ 0x00000800, { 3948872, 4013646, 3933668, }, },
	{ 0x00000983, { 3943502, 4007331, 3928075, }, },
	{ 0x00000B50, { 3938029, 4000909, 3922368, }, },
	{ 0x00000D74, { 3932461, 3994392, 3916558, }, },
	{ 0x00001000, { 3926792, 3987772, 3910636, }, },
	{ 0x00001307, { 3921022, 3981052, 3904605, }, },
	{ 0x000016A1, { 3915146, 3974224, 3898455, }, },
	{ 0x00001AE9, { 3909163, 3967289, 3892189, }, },
	{ 0x00002000, { 3903070, 3960245, 3885801, }, },
	{ 0x0000260E, { 3896860, 3953083, 3879284, }, },
	{ 0x00002D41, { 3890532, 3945805, 3872637, }, },
	{ 0x000035D1, { 3884081, 3938403, 3865854, }, },
	{ 0x00004000, { 3877504, 3930876, 3858930, }, },
	{ 0x00004C1C, { 3870797, 3923221, 3851863, }, },
	{ 0x00005A82, { 3863956, 3915434, 3844649, }, },
	{ 0x00006BA2, { 3856976, 3907510, 3837279, }, },
	{ 0x00008000, { 3849853, 3899444, 3829750, }, },
	{ 0x00009838, { 3842582, 3891234, 3822056, }, },
	{ 0x0000B505, { 3835159, 3882874, 3814193, }, },
	{ 0x0000D745, { 3827577, 3874360, 3806153, }, },
	{ 0x00010000, { 3819832, 3865687, 3797931, }, },
	{ 0x00013070, { 3811918, 3856849, 3789519, }, },
	{ 0x00016A0A, { 3803829, 3847842, 3780912, }, },
	{ 0x0001AE8A, { 3795559, 3838659, 3772102, }, },
	{ 0x00020000, { 3787101, 3829295, 3763080, }, },
	{ 0x000260E0, { 3778447, 3819742, 3753839, }, },
	{ 0x0002D414, { 3769592, 3809996, 3744372, }, },
	{ 0x00035D14, { 3760527, 3800049, 3734667, }, },
	{ 0x00040000, { 3751244, 3789893, 3724717, }, },
	{ 0x0004C1C0, { 3741734, 3779522, 3714512, }, },
	{ 0x0005A828, { 3731990, 3768927, 3704040, }, },
	{ 0x0006BA28, { 3722000, 3758099, 3693292, }, },
	{ 0x00080000, { 3711756, 3747030, 3682254, }, },
	{ 0x0009837F, { 3701247, 3735711, 3670915, }, },
	{ 0x000B504F, { 3690462, 3724131, 3659262, }, },
	{ 0x000D7450, { 3679388, 3712280, 3647281, }, },
	{ 0x00100000, { 3668014, 3700147, 3634957, }, },
	{ 0x001306FE, { 3656325, 3687721, 3622274, }, },
	{ 0x0016A09E, { 3644309, 3674988, 3609216, }, },
	{ 0x001AE8A0, { 3631950, 3661936, 3595765, }, },
	{ 0x00200000, { 3619231, 3648550, 3581902, }, },
	{ 0x00260DFC, { 3606137, 3634817, 3567607, }, },
	{ 0x002D413D, { 3592649, 3620719, 3552859, }, },
	{ 0x0035D13F, { 3578748, 3606240, 3537634, }, },
	{ 0x00400000, { 3564413, 3591361, 3521908, }, },
	{ 0x004C1BF8, { 3549622, 3576065, 3505654, }, },
	{ 0x005A827A, { 3534351, 3560329, 3488845, }, },
	{ 0x006BA27E, { 3518576, 3544131, 3471449, }, },
	{ 0x00800000, { 3502268, 3527448, 3453434, }, },
	{ 0x009837F0, { 3485399, 3510255, 3434765, }, },
	{ 0x00B504F3, { 3467936, 3492523, 3415404, }, },
	{ 0x00D744FD, { 3449847, 3474223, 3395308, }, },
	{ 0x01000000, { 3431093, 3455322, 3374435, }, },
	{ 0x01306FE1, { 3411635, 3435786, 3352735, }, },
	{ 0x016A09E6, { 3391431, 3415578, 3330156, }, },
	{ 0x01AE89FA, { 3370432, 3394655, 3306641, }, },
	{ 0x02000000, { 3348587, 3372974, 3282127, }, },
	{ 0x0260DFC1, { 3325842, 3350485, 3256547, }, },
	{ 0x02D413CD, { 3302134, 3327135, 3229824, }, },
	{ 0x035D13F3, { 3277397, 3302865, 3201879, }, },
	{ 0x04000000, { 3251558, 3277611, 3172620, }, },
	{ 0x04C1BF83, { 3224535, 3251302, 3141948, }, },
	{ 0x05A8279A, { 3196240, 3223858, 3109753, }, },
	{ 0x06BA27E6, { 3166574, 3195192, 3075914, }, },
	{ 0x08000000, { 3135426, 3165207, 3040295, }, },
	{ 0x09837F05, { 3102676, 3133793, 3002744, }, },
	{ 0x0B504F33, { 3068187, 3100829, 2963094, }, },
	{ 0x0D744FCD, { 3031806, 3066175, 2921155, }, },
	{ 0x10000000, { 2993361, 3029675, 2876712, }, },
	{ 0x1306FE0A, { 2952659, 2991153, 2829527, }, },
	{ 0x16A09E66, { 2909480, 2950402, 2779324, }, },
	{ 0x1AE89F99, { 2863575, 2907191, 2725793, }, },
	{ 0x20000000, { 2814655, 2861246, 2668579, }, },
	{ 0x260DFC14, { 2762394, 2812251, 2607272, }, },
	{ 0x2D413CCD, { 2706412, 2759834, 2541403, }, },
	{ 0x35D13F32, { 2646266, 2703554, 2470425, }, },
	{ 0x40000000, { 2581441, 2642883, 2393706, }, },
	{ 0x4C1BF828, { 2511332, 2577183, 2310504, }, },
	{ 0x5A82799A, { 2435220, 2505675, 2219951, }, },
	{ 0x6BA27E65, { 2352250, 2427391, 2121028, }, },
	{ 0x80000000, { 2261395, 2341114, 2012536, }, },
	{ 0x9837F051, { 2161415, 2245288, 1893066, }, },
	{ 0xB504F333, { 2050800, 2137874, 1760986, }, },
	{ 0xD744FCCA, { 1927706, 2016150, 1614437, }, },
	{ 0xFFFFFFFF, { 1789879, 1876363, 1451415, }, },
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
	.factory_v255_regs = {
		0x090,
		0x081,
		0x0c5,
	},
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

	if (omap4_tuna_get_revision() !=
	    (omap4_tuna_get_type() == TUNA_TYPE_MAGURO ? 2 : 1)) {
		/*
		 * Older devices were not calibrated the same way as newer
		 * devices. These values are probably not correct, but the older
		 * devices tested look closer to the newer devices with these
		 * values than they do using the same register values as the
		 * newer devices.
		 */

		/* Convert from 8500K to D65, assuming:
		 * Rx 0.66950, Ry 0.33100
		 * Gx 0.18800, Gy 0.74350
		 * Bx 0.14142, By 0.04258
		 */
		tuna_oled_data.color_adj.mult[0] = 2318372099U;
		tuna_oled_data.color_adj.mult[1] = 2117262806U;
		tuna_oled_data.color_adj.mult[2] = 1729744557U;
		tuna_oled_data.color_adj.rshift = 31;
	}

	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	omap_display_init(dss_data);
}
