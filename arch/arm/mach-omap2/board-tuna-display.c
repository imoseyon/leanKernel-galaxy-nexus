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

#define TUNA_GPIO_MLCD_RST		23

/* 4.65" Panel ID Info (D1h 1st Para) */
#define M3		0xA1
#define SM2		0x12
#define SM2A2		0xA2

static unsigned int panel_id;
struct regulator *tuna_oled_reg;
struct regulator *tuna_oled_reg_iovcc;


static void tuna_oled_set_power(bool enable)
{
	if (IS_ERR_OR_NULL(tuna_oled_reg)) {
		tuna_oled_reg = regulator_get(NULL, "vlcd");
		if (IS_ERR_OR_NULL(tuna_oled_reg)) {
			pr_err("Can't get vlcd for display!\n");
			return;
		}
	}

	if (omap4_tuna_get_revision() >= 5) {
		if (IS_ERR_OR_NULL(tuna_oled_reg_iovcc)) {
			tuna_oled_reg_iovcc = regulator_get(NULL, "vlcd-iovcc");
			if (IS_ERR_OR_NULL(tuna_oled_reg_iovcc)) {
				pr_err("Can't get vlcd for display!\n");
				return;
			}
		}

		if (enable) {
			regulator_enable(tuna_oled_reg_iovcc);
			regulator_enable(tuna_oled_reg);
		} else {
			regulator_disable(tuna_oled_reg);
			regulator_disable(tuna_oled_reg_iovcc);
		}
	} else {
		if (enable)
			regulator_enable(tuna_oled_reg);
		else
			regulator_disable(tuna_oled_reg);
	}
}

static const struct s6e8aa0_acl_parameters tuna_oled_acl[] = {
	{
		.cd = 40,
		.acl_val = 43,
		.regs = {
			0xC1, /* ACL Control2 Register */
			0x47,
			0x53,
			0x13,
			0x53,
			0x00,
			0x00,
			0x02,
			0xCF,
			0x00,
			0x00,
			0x04,
			0xFF,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x01,
			0x07,
			0x0D,
			0x14,
			0x1A,
			0x20,
			0x26,
			0x2C,
			0x33,
			0x39,
			0x3F,
		},
	},
	{
		.cd = 300,
		.acl_val = 45,
		.regs = {
			0xC1, /* ACL Control2 Register */
			0x47,
			0x53,
			0x13,
			0x53,
			0x00,
			0x00,
			0x02,
			0xCF,
			0x00,
			0x00,
			0x04,
			0xFF,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x01,
			0x07,
			0x0E,
			0x14,
			0x1B,
			0x21,
			0x27,
			0x2E,
			0x34,
			0x3B,
			0x41,
		},
	},
};

static const struct s6e8aa0_elvss_parameters tuna_oled_elvss[] = {
	{
		.cd = 100,
		.elvss_val = 0x11,
	},
	{
		.cd = 160,
		.elvss_val = 0x0D,
	},
	{
		.cd = 200,
		.elvss_val = 0x08,
	},
	{
		.cd = 300,
		.elvss_val = 0x00,
	},
};

static const u8 tuna_oled_cmd_init_pre0[] = {
	0xF0,
	0x5A,
	0x5A,
};

static const u8 tuna_oled_cmd_init_pre1[] = {
	0xF1,
	0x5A,
	0x5A,
};

static const u8 tuna_oled_cmd_sleep_out[] = {
	0x11,
};

static const u8 tuna_oled_cmd_init_panel_m3[] = {
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

static const u8 tuna_oled_cmd_init_panel_sm2[] = {
	0xF8, /* Panel Condition Set */
	0x3D, /* DOTC[0:1], GTCON[2:4], SS, DOTC_H[6:7] */
	0x31, /* FLTE[0:7] */
	0x00,
	0x00,
	0x00,
	0x8C, /* FLWE */
	0x00,
	0x39, /* SCTE[0:7] */
	0x77, /* SCWE */
	0x08, /* INTE */
	0x25,
	0x77, /* INTE[0:7] */
	0x3C, /* INWE[0:7] */
	0x00, /* EMPS */
	0x00,
	0x00,
	0x20,
	0x04, /* E_FLWE_H[0:7] */
	0x08, /* E_SCTE[0:7] */
	0x68, /* E_SCWE[0:7] */
	0x00,
	0x00,
	0x00,
	0x02,
	0x07,
	0x07,
	0x21,
	0x21,
	0xC0,
	0xC8, /* CLK2_CON[0:2], CLK1_CON[3:5], CLK2_DC, CLK1_DC */
	0x08, /* INT2_CON[0:2], INT1_CON[3:5], INT2_DC, INT1_DC */
	0x48, /* BICTLB_CON[0:2], BICTL_CON[3:5], BICTLB_DC, BICTL_DC */
	0xC1,
	0x00,
	0xC1, /* EM_FLM_CON[0:2], ACL_FLM_CON[3:5], EM_FLM_DC, ACL_FLM_DC */
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

static const struct s6e8aa0_sequence_entry tuna_oled_seq_display_set_m3[] = {
	{
		.cmd = tuna_oled_cmd_init_pre0,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_pre0),
	},
	{
		.cmd = tuna_oled_cmd_sleep_out,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_sleep_out),
		.msleep = 5,
	},
	{
		.cmd = tuna_oled_cmd_init_panel_m3,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_panel_m3),
	},
	{
		.cmd = tuna_oled_cmd_init_display,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_display),
	},
};

static const struct s6e8aa0_sequence_entry tuna_oled_seq_display_set_sm2[] = {
	{
		.cmd = tuna_oled_cmd_init_pre0,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_pre0),
	},
	{
		.cmd = tuna_oled_cmd_init_pre1,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_pre1),
	},
	{
		.cmd = tuna_oled_cmd_sleep_out,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_sleep_out),
		.msleep = 5,
	},
	{
		.cmd = tuna_oled_cmd_init_panel_sm2,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_panel_sm2),
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

static const u8 tuna_oled_cmd_init_post2_m3[] = {
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
	0x40, /* ELVSS_CON : 0 */
	0xD0, /* ELVSS -4.9V */
	0x00,
	0x60,
	0x19,
};

static const u8 tuna_oled_cmd_power_ctrl_m3[] = {
	0xF4, /* Power Control */
	0xCF,
	0x0A,
	0x0F, /* Vreg1 : 4.5V(default) */
	0x10, /* VGH : 5.2v(default) */
	0x19, /* VGL : -7.0v(default) */
	0x33,
	0x02,
};

static const u8 tuna_oled_cmd_init_post2_sm2[] = {
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
	0x41, /* ELVSS_CON : 1 */
	0xD0, /* ELVSS -4.9V */
	0x00,
	0x60,
	0x19,
};

static const u8 tuna_oled_cmd_power_ctrl_sm2[] = {
	0xF4, /* Power Control */
	0xCF,
	0x0A,
	0x12, /* Vreg1 : 4.6V */
	0x10, /* VGH : 5.2v(default) */
	0x1E, /* VGL : -8.0v */
	0x33,
	0x02,
};

static const u8 tuna_oled_cmd_display_on[] = {
	0x29,
};

static const struct s6e8aa0_sequence_entry tuna_oled_seq_etc_set_m3[] = {
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
		.cmd = tuna_oled_cmd_init_post2_m3,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_post2_m3),
	},
	{
		.cmd = tuna_oled_cmd_power_ctrl_m3,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_power_ctrl_m3),
		.msleep = 120,
	},
	{
		.cmd = tuna_oled_cmd_display_on,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_display_on),
	},
};

static const struct s6e8aa0_sequence_entry tuna_oled_seq_etc_set_sm2[] = {
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
		.cmd = tuna_oled_cmd_init_post2_sm2,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_init_post2_sm2),
	},
	{
		.cmd = tuna_oled_cmd_power_ctrl_sm2,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_power_ctrl_sm2),
		.msleep = 120,
	},
	{
		.cmd = tuna_oled_cmd_display_on,
		.cmd_len = ARRAY_SIZE(tuna_oled_cmd_display_on),
	},
};

static const struct s6e8aa0_gamma_entry tuna_oled_gamma_table_m3[] = {
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

static const struct s6e8aa0_gamma_entry tuna_oled_gamma_table_sm2[] = {
	{       BV_0, { 4600000, 4600000, 4600000, }, },
	{ 0x00000001, { 4561667, 4561667, 4561667, }, },
	{ 0x000004C2, { 4102930, 4561654, 4561115, }, },
	{ 0x000005A8, { 4093308, 4561651, 3799195, }, },
	{ 0x000006BA, { 4083466, 4561645, 3793888, }, },
	{ 0x00000800, { 4073413, 4561639, 3788484, }, },
	{ 0x00000983, { 4063166, 4561630, 3782992, }, },
	{ 0x00000B50, { 4052685, 4561618, 3777391, }, },
	{ 0x00000D74, { 4041989, 4561602, 3771689, }, },
	{ 0x00001000, { 4031064, 4561582, 3765880, }, },
	{ 0x00001307, { 4019915, 4561555, 3759964, }, },
	{ 0x000016A1, { 4008527, 4561519, 3753935, }, },
	{ 0x00001AE9, { 3996905, 4561472, 3747792, }, },
	{ 0x00002000, { 3985042, 4561410, 3741533, }, },
	{ 0x0000260E, { 3972926, 4561328, 3735148, }, },
	{ 0x00002D41, { 3960557, 4561219, 3728639, }, },
	{ 0x000035D1, { 3947926, 4561076, 3721998, }, },
	{ 0x00004000, { 3935029, 4560888, 3715222, }, },
	{ 0x00004C1C, { 3921862, 4560639, 3708307, }, },
	{ 0x00005A82, { 3908420, 4560310, 3701250, }, },
	{ 0x00006BA2, { 3894694, 4559877, 3694045, }, },
	{ 0x00008000, { 3880678, 4559305, 3686685, }, },
	{ 0x00009838, { 3866369, 4558550, 3679168, }, },
	{ 0x0000B505, { 3851759, 4557554, 3671487, }, },
	{ 0x0000D745, { 3836842, 4556240, 3663637, }, },
	{ 0x00010000, { 3821612, 4554507, 3655612, }, },
	{ 0x00013070, { 3806062, 4552219, 3647405, }, },
	{ 0x00016A0A, { 3790185, 4549200, 3639010, }, },
	{ 0x0001AE8A, { 3773975, 4545217, 3630420, }, },
	{ 0x00020000, { 3757424, 4539961, 3621628, }, },
	{ 0x000260E0, { 3740525, 4533026, 3612625, }, },
	{ 0x0002D414, { 3723271, 4523876, 3603405, }, },
	{ 0x00035D14, { 3705654, 4511801, 3593959, }, },
	{ 0x00040000, { 3687668, 4495869, 3584277, }, },
	{ 0x0004C1C0, { 3669303, 4474845, 3574351, }, },
	{ 0x0005A828, { 3650553, 4447106, 3564170, }, },
	{ 0x0006BA28, { 3631408, 4410503, 3553725, }, },
	{ 0x00080000, { 3611862, 4362204, 3543003, }, },
	{ 0x0009837F, { 3591904, 4298475, 3531993, }, },
	{ 0x000B504F, { 3571527, 4214383, 3520683, }, },
	{ 0x000D7450, { 3557593, 4103423, 3509060, }, },
	{ 0x00100000, { 3544015, 4010592, 3497110, }, },
	{ 0x001306FE, { 3530633, 3962306, 3484817, }, },
	{ 0x0016A09E, { 3517328, 3926468, 3472166, }, },
	{ 0x001AE8A0, { 3504007, 3896492, 3459141, }, },
	{ 0x00200000, { 3490597, 3869832, 3445722, }, },
	{ 0x00260DFC, { 3477036, 3845220, 3431893, }, },
	{ 0x002D413D, { 3463269, 3821929, 3417630, }, },
	{ 0x0035D13F, { 3449251, 3799494, 3402914, }, },
	{ 0x00400000, { 3434937, 3777603, 3387721, }, },
	{ 0x004C1BF8, { 3420287, 3756027, 3372025, }, },
	{ 0x005A827A, { 3405262, 3734596, 3355799, }, },
	{ 0x006BA27E, { 3389824, 3713176, 3339016, }, },
	{ 0x00800000, { 3373936, 3691658, 3321643, }, },
	{ 0x009837F0, { 3357560, 3669948, 3303646, }, },
	{ 0x00B504F3, { 3340656, 3647968, 3284991, }, },
	{ 0x00D744FD, { 3323186, 3625646, 3265636, }, },
	{ 0x01000000, { 3305106, 3602915, 3245540, }, },
	{ 0x01306FE1, { 3286372, 3579714, 3224657, }, },
	{ 0x016A09E6, { 3266937, 3555983, 3202935, }, },
	{ 0x01AE89FA, { 3246751, 3531662, 3180321, }, },
	{ 0x02000000, { 3225759, 3506692, 3156754, }, },
	{ 0x0260DFC1, { 3203902, 3481011, 3132167, }, },
	{ 0x02D413CD, { 3181115, 3454554, 3106490, }, },
	{ 0x035D13F3, { 3157329, 3427255, 3079643, }, },
	{ 0x04000000, { 3132467, 3399041, 3051538, }, },
	{ 0x04C1BF83, { 3106444, 3369833, 3022078, }, },
	{ 0x05A8279A, { 3079166, 3339546, 2991155, }, },
	{ 0x06BA27E6, { 3050528, 3308086, 2958650, }, },
	{ 0x08000000, { 3020414, 3275348, 2924429, }, },
	{ 0x09837F05, { 2988694, 3241215, 2888342, }, },
	{ 0x0B504F33, { 2955220, 3205555, 2850219, }, },
	{ 0x0D744FCD, { 2919827, 3168219, 2809871, }, },
	{ 0x10000000, { 2882325, 3129034, 2767081, }, },
	{ 0x1306FE0A, { 2842499, 3087803, 2721603, }, },
	{ 0x16A09E66, { 2800102, 3044294, 2673154, }, },
	{ 0x1AE89F99, { 2754846, 2998238, 2621408, }, },
	{ 0x20000000, { 2706399, 2949314, 2565989, }, },
	{ 0x260DFC14, { 2654372, 2897137, 2506458, }, },
	{ 0x2D413CCD, { 2598304, 2841239, 2442298, }, },
	{ 0x35D13F32, { 2537647, 2781048, 2372900, }, },
	{ 0x40000000, { 2471743, 2715846, 2297536, }, },
	{ 0x4C1BF828, { 2399793, 2644720, 2215328, }, },
	{ 0x5A82799A, { 2320814, 2566484, 2125212, }, },
	{ 0x6BA27E65, { 2233581, 2479554, 2025874, }, },
	{ 0x80000000, { 2136547, 2381755, 1915679, }, },
	{ 0x9837F051, { 2027719, 2269975, 1792556, }, },
	{ 0xB504F333, { 1904479, 2139541, 1653843, }, },
	{ 0xD744FCCA, { 1763299, 1982960, 1496041, }, },
	{ 0xFFFFFFFF, { 1599291, 1787064, 1314455, }, },
};

static const struct s6e8aa0_gamma_entry tuna_oled_gamma_table_sm2a2[] = {
	{       BV_0, { 4600000, 4600000, 4600000, }, },
	{ 0x00000001, { 4561667, 4561667, 4561667, }, },
	{ 0x000004C2, { 4320569, 4507119, 4172023, }, },
	{ 0x000005A8, { 4313803, 4504412, 4164881, }, },
	{ 0x000006BA, { 4306834, 4501566, 4157595, }, },
	{ 0x00000800, { 4299666, 4498576, 4150172, }, },
	{ 0x00000983, { 4292309, 4495443, 4142625, }, },
	{ 0x00000B50, { 4284732, 4492149, 4134926, }, },
	{ 0x00000D74, { 4276946, 4488692, 4127090, }, },
	{ 0x00001000, { 4268937, 4485062, 4119106, }, },
	{ 0x00001307, { 4260707, 4481254, 4110980, }, },
	{ 0x000016A1, { 4252243, 4477254, 4102702, }, },
	{ 0x00001AE9, { 4243544, 4473058, 4094275, }, },
	{ 0x00002000, { 4234603, 4468654, 4085695, }, },
	{ 0x0000260E, { 4225408, 4464030, 4076956, }, },
	{ 0x00002D41, { 4215956, 4459176, 4068057, }, },
	{ 0x000035D1, { 4206236, 4454081, 4058993, }, },
	{ 0x00004000, { 4196243, 4448731, 4049762, }, },
	{ 0x00004C1C, { 4185969, 4443116, 4040363, }, },
	{ 0x00005A82, { 4175408, 4437223, 4030792, }, },
	{ 0x00006BA2, { 4164549, 4431036, 4021044, }, },
	{ 0x00008000, { 4153383, 4424541, 4011116, }, },
	{ 0x00009838, { 4141905, 4417724, 4001007, }, },
	{ 0x0000B505, { 4130104, 4410567, 3990713, }, },
	{ 0x0000D745, { 4117971, 4403055, 3980229, }, },
	{ 0x00010000, { 4105497, 4395170, 3969553, }, },
	{ 0x00013070, { 4092672, 4386892, 3958681, }, },
	{ 0x00016A0A, { 4079487, 4378203, 3947609, }, },
	{ 0x0001AE8A, { 4065931, 4369081, 3936334, }, },
	{ 0x00020000, { 4051994, 4359507, 3924852, }, },
	{ 0x000260E0, { 4037665, 4349456, 3913159, }, },
	{ 0x0002D414, { 4022934, 4338906, 3901251, }, },
	{ 0x00035D14, { 4007788, 4327831, 3889125, }, },
	{ 0x00040000, { 3992216, 4316205, 3876776, }, },
	{ 0x0004C1C0, { 3976207, 4304001, 3864200, }, },
	{ 0x0005A828, { 3959747, 4291191, 3851393, }, },
	{ 0x0006BA28, { 3942825, 4277744, 3838351, }, },
	{ 0x00080000, { 3925427, 4263628, 3825070, }, },
	{ 0x0009837F, { 3907540, 4248811, 3811545, }, },
	{ 0x000B504F, { 3889150, 4233257, 3797772, }, },
	{ 0x000D7450, { 3873393, 4216929, 3783745, }, },
	{ 0x00100000, { 3857665, 4199790, 3769461, }, },
	{ 0x001306FE, { 3841894, 4181799, 3754915, }, },
	{ 0x0016A09E, { 3826021, 4162913, 3740102, }, },
	{ 0x001AE8A0, { 3809994, 4143088, 3725017, }, },
	{ 0x00200000, { 3793770, 4122278, 3709654, }, },
	{ 0x00260DFC, { 3777308, 4100433, 3694010, }, },
	{ 0x002D413D, { 3760573, 4077502, 3677663, }, },
	{ 0x0035D13F, { 3743530, 4053431, 3660904, }, },
	{ 0x00400000, { 3726147, 4028163, 3643715, }, },
	{ 0x004C1BF8, { 3708392, 4001639, 3626075, }, },
	{ 0x005A827A, { 3690235, 3973797, 3607960, }, },
	{ 0x006BA27E, { 3671644, 3953685, 3589346, }, },
	{ 0x00800000, { 3652589, 3933053, 3570207, }, },
	{ 0x009837F0, { 3633036, 3911876, 3550514, }, },
	{ 0x00B504F3, { 3612951, 3890122, 3530237, }, },
	{ 0x00D744FD, { 3592301, 3867762, 3509342, }, },
	{ 0x01000000, { 3571046, 3844761, 3487793, }, },
	{ 0x01306FE1, { 3549148, 3821082, 3465552, }, },
	{ 0x016A09E6, { 3526565, 3796686, 3442574, }, },
	{ 0x01AE89FA, { 3503250, 3771528, 3418813, }, },
	{ 0x02000000, { 3479155, 3745560, 3394218, }, },
	{ 0x0260DFC1, { 3454224, 3718731, 3368732, }, },
	{ 0x02D413CD, { 3428401, 3690982, 3342292, }, },
	{ 0x035D13F3, { 3401619, 3662250, 3314830, }, },
	{ 0x04000000, { 3373808, 3632465, 3286268, }, },
	{ 0x04C1BF83, { 3344890, 3601547, 3256522, }, },
	{ 0x05A8279A, { 3314777, 3569411, 3225495, }, },
	{ 0x06BA27E6, { 3283370, 3535958, 3193081, }, },
	{ 0x08000000, { 3250562, 3501080, 3159159, }, },
	{ 0x09837F05, { 3216227, 3464651, 3123590, }, },
	{ 0x0B504F33, { 3180227, 3426533, 3086220, }, },
	{ 0x0D744FCD, { 3142401, 3386564, 3046868, }, },
	{ 0x10000000, { 3102567, 3344561, 3005329, }, },
	{ 0x1306FE0A, { 3060513, 3300311, 2961362, }, },
	{ 0x16A09E66, { 3015995, 3253568, 2914685, }, },
	{ 0x1AE89F99, { 2968726, 3204040, 2864970, }, },
	{ 0x20000000, { 2918367, 3151384, 2811819, }, },
	{ 0x260DFC14, { 2864513, 3095191, 2754761, }, },
	{ 0x2D413CCD, { 2806678, 3034964, 2693215, }, },
	{ 0x35D13F32, { 2744269, 2970096, 2626472, }, },
	{ 0x40000000, { 2676551, 2899833, 2553637, }, },
	{ 0x4C1BF828, { 2602603, 2823224, 2473575, }, },
	{ 0x5A82799A, { 2521246, 2739043, 2384804, }, },
	{ 0x6BA27E65, { 2430943, 2645676, 2285353, }, },
	{ 0x80000000, { 2329631, 2540937, 2172509, }, },
	{ 0x9837F051, { 2214459, 2421765, 2042412, }, },
	{ 0xB504F333, { 2081337, 2283682, 1889300, }, },
	{ 0xD744FCCA, { 1924091, 2119771, 1704041, }, },
	{ 0xFFFFFFFF, { 1732795, 1918520, 1470915, }, },
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
		 *
		 * These values are adjusted down by x 0.9333 to bring
		 * maximum brightness down from 300 cd/m2 to 280.
		 */
		.mult = {
			2163736680U,
			1976041377U,
			1614370595U,
		},
		.rshift = 31,
	},
};

static struct s6e8aa0_factory_calibration_info tuna_oled_factory_info_6500k = {
	.regs = {
		[1][0][0] = 0x7c, /* sRGB Gamma 300cd, 6500K */
		[1][1][0] = 0x3c,
		[1][2][0] = 0x87,

		[1][0][1] = 0xbb,
		[1][1][1] = 0xd4,
		[1][2][1] = 0xa8,

		[1][0][2] = 0xac,
		[1][1][2] = 0xc0,
		[1][2][2] = 0xa1,

		[1][0][3] = 0xb8,
		[1][1][3] = 0xc8,
		[1][2][3] = 0xb2,

		[1][0][4] = 0x8c,
		[1][1][4] = 0x9f,
		[1][2][4] = 0x84,

		[1][0][5] = 0xa7,
		[1][1][5] = 0xb2,
		[1][2][5] = 0xa3,

		[1][0][6] = 0x0d8,
		[1][1][6] = 0x0bd,
		[1][2][6] = 0x0f7,
	},
	.brightness = {
		[1][1] = BV_15,		/* 1.43 cd/m2 */
		[1][2] = BV_35,		/* 5.04 cd/m2 */
		[1][3] = BV_59,		/* 13.12 cd/m2 */
		[1][4] = BV_87,		/* 28.59 cd/m2 */
		[1][5] = BV_171,	/* 122.17 cd/m2 */
		[1][6] = BV_255,	/* 300 cd/m2 */
	},
	.color_adj = {
		/*
		 * These values are adjusted down by x 0.9333 to bring
		 * maximum brightness down from 300 cd/m2 to 280.
		 */
		.mult = {
			2004318071U,
			2004318071U,
			2004318071U,
		},
		.rshift = 31,
	},
};

static struct s6e8aa0_factory_calibration_info tuna_oled_factory_info_sm2a2 = {
	.regs = {
		[1][0][0] = 0x52, /* sRGB Gamma 300cd, 6500K, A2 Line */
		[1][1][0] = 0x24,
		[1][2][0] = 0x5d,

		[1][0][1] = 0xba,
		[1][1][1] = 0xcd,
		[1][2][1] = 0xb3,

		[1][0][2] = 0xad,
		[1][1][2] = 0xc0,
		[1][2][2] = 0xb1,

		[1][0][3] = 0xbf,
		[1][1][3] = 0xc7,
		[1][2][3] = 0xbc,

		[1][0][4] = 0x90,
		[1][1][4] = 0x97,
		[1][2][4] = 0x8a,

		[1][0][5] = 0xaa,
		[1][1][5] = 0xae,
		[1][2][5] = 0xa5,

		[1][0][6] = 0x0c2,
		[1][1][6] = 0x0a8,
		[1][2][6] = 0x0d7,
	},
	.brightness = {
		[1][1] = BV_15,		/* 1.43 cd/m2 */
		[1][2] = BV_35,		/* 5.04 cd/m2 */
		[1][3] = BV_59,		/* 13.12 cd/m2 */
		[1][4] = BV_87,		/* 28.59 cd/m2 */
		[1][5] = BV_171,	/* 122.17 cd/m2 */
		[1][6] = BV_255,	/* 300 cd/m2 */
	},
	.color_adj = {
		/*
		 * These values are adjusted down by x 0.9333 to bring
		 * maximum brightness down from 300 cd/m2 to 280.
		 */
		.mult = {
			2004318071U,
			2004318071U,
			2004318071U,
		},
		.rshift = 31,
	},
};

static struct panel_s6e8aa0_data tuna_oled_data_m3 = {
	.reset_gpio	= TUNA_GPIO_MLCD_RST,
	.set_power	= tuna_oled_set_power,
	.seq_display_set = tuna_oled_seq_display_set_m3,
	.seq_display_set_size = ARRAY_SIZE(tuna_oled_seq_display_set_m3),
	.seq_etc_set = tuna_oled_seq_etc_set_m3,
	.seq_etc_set_size = ARRAY_SIZE(tuna_oled_seq_etc_set_m3),
	.gamma_table = tuna_oled_gamma_table_m3,
	.gamma_table_size = ARRAY_SIZE(tuna_oled_gamma_table_m3),
	.factory_info = &tuna_oled_factory_info_8500k,
	.acl_table = tuna_oled_acl,
	.acl_table_size = ARRAY_SIZE(tuna_oled_acl),
	.acl_average = 6, /* use 20 frame Y average accumulation count */
};

static struct panel_s6e8aa0_data tuna_oled_data_sm2 = {
	.reset_gpio	= TUNA_GPIO_MLCD_RST,
	.set_power	= tuna_oled_set_power,
	.seq_display_set = tuna_oled_seq_display_set_sm2,
	.seq_display_set_size = ARRAY_SIZE(tuna_oled_seq_display_set_sm2),
	.seq_etc_set = tuna_oled_seq_etc_set_sm2,
	.seq_etc_set_size = ARRAY_SIZE(tuna_oled_seq_etc_set_sm2),
	.gamma_table = tuna_oled_gamma_table_sm2,
	.gamma_table_size = ARRAY_SIZE(tuna_oled_gamma_table_sm2),
	.factory_info = &tuna_oled_factory_info_6500k,
	.acl_table = tuna_oled_acl,
	.acl_table_size = ARRAY_SIZE(tuna_oled_acl),
	.acl_average = 6, /* use 20 frame Y average accumulation count */
	.elvss_table = tuna_oled_elvss,
	.elvss_table_size = ARRAY_SIZE(tuna_oled_elvss),
};

static struct panel_s6e8aa0_data tuna_oled_data_sm2a2 = {
	.reset_gpio	= TUNA_GPIO_MLCD_RST,
	.set_power	= tuna_oled_set_power,
	.seq_display_set = tuna_oled_seq_display_set_sm2,
	.seq_display_set_size = ARRAY_SIZE(tuna_oled_seq_display_set_sm2),
	.seq_etc_set = tuna_oled_seq_etc_set_sm2,
	.seq_etc_set_size = ARRAY_SIZE(tuna_oled_seq_etc_set_sm2),
	.gamma_table = tuna_oled_gamma_table_sm2a2,
	.gamma_table_size = ARRAY_SIZE(tuna_oled_gamma_table_sm2a2),
	.factory_info = &tuna_oled_factory_info_sm2a2,
	.acl_table = tuna_oled_acl,
	.acl_table_size = ARRAY_SIZE(tuna_oled_acl),
	.acl_average = 6, /* use 20 frame Y average accumulation count */
	.elvss_table = tuna_oled_elvss,
	.elvss_table_size = ARRAY_SIZE(tuna_oled_elvss),
};

static struct omap_dss_device tuna_oled_device = {
	.name			= "lcd",
	.driver_name		= "s6e8aa0",
	.type			= OMAP_DISPLAY_TYPE_DSI,
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
	.panel = {
		.width_in_um	= 58000,
		.height_in_um	= 102000,
	},
	.clocks = {
		.dispc		= {
			.channel = {
				.lck_div	= 1,	/* LCD */
				.pck_div	= 2,	/* PCD */
				.lcd_clk_src
					= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},
		.dsi		= {
			.regn		= 19,	/* DSI_PLL_REGN */
			.regm		= 236,	/* DSI_PLL_REGM */

			.regm_dispc	= 6,	/* PLL_CLK1 (M4) */
			.regm_dsi	= 6,	/* PLL_CLK2 (M5) */
			.lp_clk_div	= 8,	/* LPDIV */
			.offset_ddr_clk	= 122,	/* DDR PRE & DDR POST
						 * offset increase
						 */

			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},

	.channel		= OMAP_DSS_CHANNEL_LCD,
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	.skip_init              = true,
#else
	.skip_init              = false,
#endif
};

static void tuna_hdmi_mux_init(void)
{
	u32 r;

	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

}

static struct omap_dss_device tuna_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
			.max_pixclk_khz = 75000,
		},
	},
	.hpd_gpio = TUNA_GPIO_HDMI_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *tuna_dss_devices[] = {
	&tuna_oled_device,
	&tuna_hdmi_device,
};

static struct omap_dss_board_info tuna_dss_data = {
	.num_devices	= ARRAY_SIZE(tuna_dss_devices),
	.devices	= tuna_dss_devices,
	.default_device	= &tuna_oled_device,
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
	struct panel_s6e8aa0_data *panel;

	if (omap4_tuna_get_revision() ==
	    (omap4_tuna_get_type() == TUNA_TYPE_MAGURO ? 2 : 1)) {
		/*
		 * Older devices were not calibrated the same way as newer
		 * devices. These values are probably not correct, but the older
		 * devices tested look closer to the newer devices with these
		 * values than they do using the same register values as the
		 * newer devices.
		 */
		tuna_oled_data_m3.factory_info = &tuna_oled_factory_info_m2t1;
	} else if (omap4_tuna_get_revision() <= 1) {
		tuna_oled_data_m3.factory_info = &tuna_oled_factory_info_old;
	}

	switch (panel_id) {
	case SM2:
		panel = &tuna_oled_data_sm2;
		break;
	case SM2A2:
		panel = &tuna_oled_data_sm2a2;
		break;
	default:
		panel = &tuna_oled_data_m3;
		break;
	}

	tuna_oled_device.data = panel;

	omap4_ctrl_pad_writel(0x1FF80000,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	omap_mux_init_gpio(panel->reset_gpio, OMAP_PIN_OUTPUT);

	pr_info("Using %ps\n", panel->factory_info);

	omap_vram_set_sdram_vram(TUNA_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&tuna_fb_pdata);
	tuna_hdmi_mux_init();
	omap_display_init(&tuna_dss_data);
}

static int __init get_panel_id(char *str)
{
	long value;
	int ret;

	ret = strict_strtol(str, 0, &value);
	if (ret < 0)
		return ret;

	panel_id = (unsigned int)value;
	return 0;
}
__setup("mms_ts.panel_id=", get_panel_id);

