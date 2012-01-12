/*
 * Samsung s6e8aa0 panel support
 *
 * Copyright 2011 Google, Inc.
 * Author: Erik Gilling <konkers@google.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>


#include <video/omapdss.h>

#include <linux/platform_data/panel-s6e8aa0.h>

#include "../dss/dss.h"

/* DSI Command Virtual channel */
#define CMD_VC_CHANNEL 1

#define V1_ADJ_MAX 140
#define V255_ADJ_MAX 430
#define NUM_GAMMA_REGS	24
#define NUM_DY_REGS (32)

enum {
	V1,
	V15,
	V35,
	V59,
	V87,
	V171,
	V255,
	V_COUNT,
};

#define DRIVER_NAME "s6e8aa0_i2c"
#define DEVICE_NAME "s6e8aa0_i2c"

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h);

static struct omap_video_timings s6e8aa0_timings = {
	.x_res = 720,
	.y_res = 1280,
	.pixel_clock = 80842,
	.hfp = 158,
	.hsw = 2,
	.hbp = 160,
	.vfp = 13,
	.vsw = 1,
	.vbp = 2,
};

static const struct s6e8aa0_gamma_adj_points default_gamma_adj_points = {
	.v1 = BV_1,
	.v15 = BV_15,
	.v35 = BV_35,
	.v59 = BV_59,
	.v87 = BV_87,
	.v171 = BV_171,
};

static u32 s6e8aa0_srgb_dyi_to_b[NUM_DY_REGS] = {
	0x0027c8ac, /*   2   */
	0x004f9159, /*   4   */
	0x00775a06, /*   6   */
	0x009f22b3, /*   8   */
	0x00f0f18e, /*  12   */
	0x0153936c, /*  16   */
	0x02569c13, /*  24   */
	0x03b2977b, /*  32   */
	0x060e496b, /*  42.5 */
	0x0803965b, /*  49.5 */
	0x0bf23e3b, /*  61   */
	0x10048613, /*  70.8 */
	0x14041743, /*  79   */
	0x180da7c6, /*  86.4 */
	0x1c05aec3, /*  93   */
	0x200e2a06, /*  99.2 */
	0x280bf0ad, /* 110.2 */
	0x301505aa, /* 120   */
	0x38108cb8, /* 128.9 */
	0x400a5f93, /* 137   */
	0x5008b39d, /* 151.7 */
	0x6010dbad, /* 164.8 */
	0x700a375a, /* 176.6 */
	0x801a9901, /* 187.6 */
	0x901075cd, /* 197.7 */
	0xa01b515c, /* 207.2 */
	0xb00de3e4, /* 216.1 */
	0xc01c269c, /* 224.7 */
	0xd01871ca, /* 232.8 */
	0xe010831c, /* 240.5 */
	0xf02ca6b8, /* 247.9 */
	0xffffffff, /* 255.1 */
};

struct s6e8aa0_gamma_reg_offsets {
	s16 v[2][3][7];
};

struct s6e8aa0_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;
	struct backlight_device *bldev;
	struct dentry *debug_dir;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned int bl;
	const struct s6e8aa0_gamma_adj_points *gamma_adj_points;
	const u32 *dyi_to_b;
	struct s6e8aa0_gamma_reg_offsets gamma_reg_offsets;
	struct s6e8aa0_gamma_entry *brightness_table;
	int brightness_table_size;
	u32 brightness_limit[3];
	unsigned long hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long hw_guard_wait;	/* max guard time in jiffies */

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

	bool cabc_broken;
	unsigned cabc_mode;

	bool force_update;
	struct omap_video_timings *timings;

	struct panel_s6e8aa0_data *pdata;

	unsigned int acl_cur;
	bool acl_enable;
	u8 acl_average;
	unsigned int elvss_cur_i;
	u8 panel_id[3];
};

const u8 s6e8aa0_mtp_unlock[] = {
	0xF1,
	0x5A,
	0x5A,
};

const u8 s6e8aa0_mtp_lock[] = {
	0xF1,
	0xA5,
	0xA5,
};

#ifdef CONFIG_COLOR_CONTROL
struct omap_dss_device * lcd_dev;

int hacky_v1_offset[3] = {0, 0, 0};
#endif

static int s6e8aa0_write_reg(struct omap_dss_device *dssdev, u8 reg, u8 val)
{
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;

	return dsi_vc_dcs_write(dssdev, 1, buf, 2);
}

static int s6e8aa0_write_block(struct omap_dss_device *dssdev, const u8 *data, int len)
{
	// XXX: dsi_vc_dsc_write should take a const u8 *
	return dsi_vc_dcs_write(dssdev, 1, (u8 *)data, len);
}

static int s6e8aa0_write_block_nosync(struct omap_dss_device *dssdev,
				      const u8 *data, int len)
{
	return dsi_vc_dcs_write_nosync(dssdev, 1, (u8 *)data, len);
}

static int s6e8aa0_read_block(struct omap_dss_device *dssdev,
			      u8 cmd, u8 *data, int len)
{
	return dsi_vc_dcs_read(dssdev, 1, cmd, data, len);
}

static void s6e8aa0_write_sequence(struct omap_dss_device *dssdev,
	const struct s6e8aa0_sequence_entry *seq, int seq_len)
{
	while (seq_len--) {
		if (seq->cmd_len)
			s6e8aa0_write_block(dssdev, seq->cmd, seq->cmd_len);
		if (seq->msleep)
			msleep(seq->msleep);
		seq++;
	}
}

/***********************
*** DUMMY FUNCTIONS ****
***********************/

static int s6e8aa0_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return 0;
}

static u8 s6e8aa0_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int s6e8aa0_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static bool s6e8aa0_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;
}

static void s6e8aa0_get_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void s6e8aa0_set_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int s6e8aa0_check_timings(struct omap_dss_device *dssdev,
			     struct omap_video_timings *timings)
{
	return 0;
}

static void s6e8aa0_get_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->rotate == 0 || s6->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static int s6e8aa0_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static int s6e8aa0_hw_reset(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(s6->pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(s6->pdata->reset_gpio, 1);
	msleep(40);

	return 0;
}

static u32 s6e8aa0_table_lookup(u32 b, int c,
				const struct s6e8aa0_gamma_entry *table,
				int table_size)
{
	int i;
	u32 ret;
	u32 bl = 0;
	u32 bh = 0;
	u32 vl = 0;
	u32 vh;
	u64 tmp;

	if (!table_size)
		return b;

	for (i = 0; i < table_size; i++) {
		bl = bh;
		bh = table[i].brightness;
		if (bh >= b)
			break;
	}
	vh = table[i].v[c];
	if (i == 0 || (b - bl) == 0) {
		ret = vl = vh;
	} else {
		vl = table[i - 1].v[c];
		tmp = (u64)vh * (b - bl) + (u64)vl * (bh - b);
		do_div(tmp, bh - bl);
		ret = tmp;
	}
	pr_debug("%s: looking for c %d, %08x, "
		 "found %08x:%08x, v %7d:%7d, ret %7d\n",
		 __func__, c, b, bl, bh, vl, vh, ret);
	return ret;
}

static u32 s6e8aa0_raw_gamma_lookup(struct s6e8aa0_data *s6, u32 b, int c)
{
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	return s6e8aa0_table_lookup(b, c, pdata->gamma_table,
				    pdata->gamma_table_size);
}

static u32 s6e8aa0_gamma_lookup(struct s6e8aa0_data *s6,
				u8 brightness, u32 val, int c)
{
	u32 b;
	u32 ret;
	u64 tmp;

	tmp = val;
	tmp *= brightness;
	do_div(tmp, 255);

	b = s6e8aa0_table_lookup(tmp, c, s6->brightness_table,
				 s6->brightness_table_size);

	ret = s6e8aa0_raw_gamma_lookup(s6, b, c);

	pr_debug("%s: looking for %3d %08x c %d, %08x, got %7d\n",
		 __func__, brightness, val, c, b, ret);

	return ret;
}

/*
 * V1    =  V0 -          V0 * (   5 +   v1_adj ) / 600
 * V15   =  V1 - (V1 -  V35) * (  20 +  v15_adj ) / 320
 * V35   =  V1 - (V1 -  V59) * (  65 +  v35_adj ) / 320
 * V59   =  V1 - (V1 -  V87) * (  65 +  v59_adj ) / 320
 * V87   =  V1 - (V1 - V171) * (  65 +  v87_adj ) / 320
 * V171  =  V1 - (V1 - V255) * (  65 + v171_adj ) / 320
 * V255  =  V0 -          V0 * ( 100 + v255_adj ) / 600
 *
 * v_n_adj = v_n_reg + v_n_offset
 */

static u32 v1adj_to_v1(u8 v1_adj, u32 v0)
{
	return DIV_ROUND_CLOSEST((600 - 5 - v1_adj) * v0, 600);
}

static u32 v1_to_v1adj(u32 v1, u32 v0)
{
	return  600 - 5 - DIV_ROUND_CLOSEST(600 * v1, v0);
}

static u32 vnadj_to_vn(int n, u8 v_n_adj, u32 v1, u32 v_next)
{
	int base = (n == V15) ? 20 : 65;
	return v1 - DIV_ROUND_CLOSEST((v1 - v_next) * (base + v_n_adj), 320);
}

static u32 vn_to_vnadj(int n, u32 v_n, u32 v1, u32 v_next)
{
	int base = (n == V15) ? 20 : 65;
	return DIV_ROUND_CLOSEST(320 * (v1 - v_n), v1 - v_next) - base;
}

static u32 v255adj_to_v255(u16 v255_adj, u32 v0)
{
	return DIV_ROUND_CLOSEST((600 - 100 - v255_adj) * v0, 600);
}

static u32 v255_to_v255adj(u32 v255, u32 v0)
{
	return 600 - 100 - DIV_ROUND_CLOSEST(600 * v255, v0);
}

static int gamma_reg_index(int c, int i)
{
	return 3 * i + c;
}

static int gamma_reg_index_v255_h(int c)
{
	return 3 * V255 + 2 * c;
}

static int gamma_reg_index_v255_l(int c)
{
	return gamma_reg_index_v255_h(c) + 1;
}

static void s6e8aa0_setup_dy_regs(struct s6e8aa0_data *s6, int c,
				  u32 v0, u32 v[V_COUNT], u8 dy[NUM_DY_REGS])
{
	static const struct {
		int base;
		u32 scale;
	} output_table[256] = {
		[0]   = {                                 },
		[1]   = { V1,                             },
		[2]   = { V15,  0x100000000ULL * 47 / 52, },
		[3]   = { V15,  0x100000000ULL * 42 / 52, },
		[4]   = { V15,  0x100000000ULL * 37 / 52, },
		[5]   = { V15,  0x100000000ULL * 32 / 52, },
		[6]   = { V15,  0x100000000ULL * 27 / 52, },
		[7]   = { V15,  0x100000000ULL * 23 / 52, },
		[8]   = { V15,  0x100000000ULL * 19 / 52, },
		[9]   = { V15,  0x100000000ULL * 15 / 52, },
		[10]  = { V15,  0x100000000ULL * 12 / 52, },
		[11]  = { V15,  0x100000000ULL *  9 / 52, },
		[12]  = { V15,  0x100000000ULL *  6 / 52, },
		[13]  = { V15,  0x100000000ULL *  4 / 52, },
		[14]  = { V15,  0x100000000ULL *  2 / 52, },
		[15]  = { V15,                            },
		[16]  = { V35,  0x100000000ULL * 66 / 70, },
		[17]  = { V35,  0x100000000ULL * 62 / 70, },
		[18]  = { V35,  0x100000000ULL * 58 / 70, },
		[19]  = { V35,  0x100000000ULL * 54 / 70, },
		[20]  = { V35,  0x100000000ULL * 50 / 70, },
		[21]  = { V35,  0x100000000ULL * 46 / 70, },
		[22]  = { V35,  0x100000000ULL * 42 / 70, },
		[23]  = { V35,  0x100000000ULL * 38 / 70, },
		[24]  = { V35,  0x100000000ULL * 34 / 70, },
		[25]  = { V35,  0x100000000ULL * 30 / 70, },
		[26]  = { V35,  0x100000000ULL * 27 / 70, },
		[27]  = { V35,  0x100000000ULL * 24 / 70, },
		[28]  = { V35,  0x100000000ULL * 21 / 70, },
		[29]  = { V35,  0x100000000ULL * 18 / 70, },
		[30]  = { V35,  0x100000000ULL * 15 / 70, },
		[31]  = { V35,  0x100000000ULL * 12 / 70, },
		[32]  = { V35,  0x100000000ULL *  9 / 70, },
		[33]  = { V35,  0x100000000ULL *  6 / 70, },
		[34]  = { V35,  0x100000000ULL *  3 / 70, },
		[35]  = { V35,                            },
		[36]  = { V59,  0x100000000ULL * 23 / 24, },
		[37]  = { V59,  0x100000000ULL * 22 / 24, },
		[38]  = { V59,  0x100000000ULL * 21 / 24, },
		[39]  = { V59,  0x100000000ULL * 20 / 24, },
		[40]  = { V59,  0x100000000ULL * 19 / 24, },
		[41]  = { V59,  0x100000000ULL * 18 / 24, },
		[42]  = { V59,  0x100000000ULL * 17 / 24, },
		[43]  = { V59,  0x100000000ULL * 16 / 24, },
		[44]  = { V59,  0x100000000ULL * 15 / 24, },
		[45]  = { V59,  0x100000000ULL * 14 / 24, },
		[46]  = { V59,  0x100000000ULL * 13 / 24, },
		[47]  = { V59,  0x100000000ULL * 12 / 24, },
		[48]  = { V59,  0x100000000ULL * 11 / 24, },
		[49]  = { V59,  0x100000000ULL * 10 / 24, },
		[50]  = { V59,  0x100000000ULL *  9 / 24, },
		[51]  = { V59,  0x100000000ULL *  8 / 24, },
		[52]  = { V59,  0x100000000ULL *  7 / 24, },
		[53]  = { V59,  0x100000000ULL *  6 / 24, },
		[54]  = { V59,  0x100000000ULL *  5 / 24, },
		[55]  = { V59,  0x100000000ULL *  4 / 24, },
		[56]  = { V59,  0x100000000ULL *  3 / 24, },
		[57]  = { V59,  0x100000000ULL *  2 / 24, },
		[58]  = { V59,  0x100000000ULL *  1 / 24, },
		[59]  = { V59,                            },
		[60]  = { V87,  0x100000000ULL * 27 / 28, },
		[61]  = { V87,  0x100000000ULL * 26 / 28, },
		[62]  = { V87,  0x100000000ULL * 25 / 28, },
		[63]  = { V87,  0x100000000ULL * 24 / 28, },
		[64]  = { V87,  0x100000000ULL * 23 / 28, },
		[65]  = { V87,  0x100000000ULL * 22 / 28, },
		[66]  = { V87,  0x100000000ULL * 21 / 28, },
		[67]  = { V87,  0x100000000ULL * 20 / 28, },
		[68]  = { V87,  0x100000000ULL * 19 / 28, },
		[69]  = { V87,  0x100000000ULL * 18 / 28, },
		[70]  = { V87,  0x100000000ULL * 17 / 28, },
		[71]  = { V87,  0x100000000ULL * 16 / 28, },
		[72]  = { V87,  0x100000000ULL * 15 / 28, },
		[73]  = { V87,  0x100000000ULL * 14 / 28, },
		[74]  = { V87,  0x100000000ULL * 13 / 28, },
		[75]  = { V87,  0x100000000ULL * 12 / 28, },
		[76]  = { V87,  0x100000000ULL * 11 / 28, },
		[77]  = { V87,  0x100000000ULL * 10 / 28, },
		[78]  = { V87,  0x100000000ULL *  9 / 28, },
		[79]  = { V87,  0x100000000ULL *  8 / 28, },
		[80]  = { V87,  0x100000000ULL *  7 / 28, },
		[81]  = { V87,  0x100000000ULL *  6 / 28, },
		[82]  = { V87,  0x100000000ULL *  5 / 28, },
		[83]  = { V87,  0x100000000ULL *  4 / 28, },
		[84]  = { V87,  0x100000000ULL *  3 / 28, },
		[85]  = { V87,  0x100000000ULL *  2 / 28, },
		[86]  = { V87,  0x100000000ULL *  1 / 28, },
		[87]  = { V87,                            },
		[88]  = { V171, 0x100000000ULL * 83 / 84, },
		[89]  = { V171, 0x100000000ULL * 82 / 84, },
		[90]  = { V171, 0x100000000ULL * 81 / 84, },
		[91]  = { V171, 0x100000000ULL * 80 / 84, },
		[92]  = { V171, 0x100000000ULL * 79 / 84, },
		[93]  = { V171, 0x100000000ULL * 78 / 84, },
		[94]  = { V171, 0x100000000ULL * 77 / 84, },
		[95]  = { V171, 0x100000000ULL * 76 / 84, },
		[96]  = { V171, 0x100000000ULL * 75 / 84, },
		[97]  = { V171, 0x100000000ULL * 74 / 84, },
		[98]  = { V171, 0x100000000ULL * 73 / 84, },
		[99]  = { V171, 0x100000000ULL * 72 / 84, },
		[100] = { V171, 0x100000000ULL * 71 / 84, },
		[101] = { V171, 0x100000000ULL * 70 / 84, },
		[102] = { V171, 0x100000000ULL * 69 / 84, },
		[103] = { V171, 0x100000000ULL * 68 / 84, },
		[104] = { V171, 0x100000000ULL * 67 / 84, },
		[105] = { V171, 0x100000000ULL * 66 / 84, },
		[106] = { V171, 0x100000000ULL * 65 / 84, },
		[107] = { V171, 0x100000000ULL * 64 / 84, },
		[108] = { V171, 0x100000000ULL * 63 / 84, },
		[109] = { V171, 0x100000000ULL * 62 / 84, },
		[110] = { V171, 0x100000000ULL * 61 / 84, },
		[111] = { V171, 0x100000000ULL * 60 / 84, },
		[112] = { V171, 0x100000000ULL * 59 / 84, },
		[113] = { V171, 0x100000000ULL * 58 / 84, },
		[114] = { V171, 0x100000000ULL * 57 / 84, },
		[115] = { V171, 0x100000000ULL * 56 / 84, },
		[116] = { V171, 0x100000000ULL * 55 / 84, },
		[117] = { V171, 0x100000000ULL * 54 / 84, },
		[118] = { V171, 0x100000000ULL * 53 / 84, },
		[119] = { V171, 0x100000000ULL * 52 / 84, },
		[120] = { V171, 0x100000000ULL * 51 / 84, },
		[121] = { V171, 0x100000000ULL * 50 / 84, },
		[122] = { V171, 0x100000000ULL * 49 / 84, },
		[123] = { V171, 0x100000000ULL * 48 / 84, },
		[124] = { V171, 0x100000000ULL * 47 / 84, },
		[125] = { V171, 0x100000000ULL * 46 / 84, },
		[126] = { V171, 0x100000000ULL * 45 / 84, },
		[127] = { V171, 0x100000000ULL * 44 / 84, },
		[128] = { V171, 0x100000000ULL * 43 / 84, },
		[129] = { V171, 0x100000000ULL * 42 / 84, },
		[130] = { V171, 0x100000000ULL * 41 / 84, },
		[131] = { V171, 0x100000000ULL * 40 / 84, },
		[132] = { V171, 0x100000000ULL * 39 / 84, },
		[133] = { V171, 0x100000000ULL * 38 / 84, },
		[134] = { V171, 0x100000000ULL * 37 / 84, },
		[135] = { V171, 0x100000000ULL * 36 / 84, },
		[136] = { V171, 0x100000000ULL * 35 / 84, },
		[137] = { V171, 0x100000000ULL * 34 / 84, },
		[138] = { V171, 0x100000000ULL * 33 / 84, },
		[139] = { V171, 0x100000000ULL * 32 / 84, },
		[140] = { V171, 0x100000000ULL * 31 / 84, },
		[141] = { V171, 0x100000000ULL * 30 / 84, },
		[142] = { V171, 0x100000000ULL * 29 / 84, },
		[143] = { V171, 0x100000000ULL * 28 / 84, },
		[144] = { V171, 0x100000000ULL * 27 / 84, },
		[145] = { V171, 0x100000000ULL * 26 / 84, },
		[146] = { V171, 0x100000000ULL * 25 / 84, },
		[147] = { V171, 0x100000000ULL * 24 / 84, },
		[148] = { V171, 0x100000000ULL * 23 / 84, },
		[149] = { V171, 0x100000000ULL * 22 / 84, },
		[150] = { V171, 0x100000000ULL * 21 / 84, },
		[151] = { V171, 0x100000000ULL * 20 / 84, },
		[152] = { V171, 0x100000000ULL * 19 / 84, },
		[153] = { V171, 0x100000000ULL * 18 / 84, },
		[154] = { V171, 0x100000000ULL * 17 / 84, },
		[155] = { V171, 0x100000000ULL * 16 / 84, },
		[156] = { V171, 0x100000000ULL * 15 / 84, },
		[157] = { V171, 0x100000000ULL * 14 / 84, },
		[158] = { V171, 0x100000000ULL * 13 / 84, },
		[159] = { V171, 0x100000000ULL * 12 / 84, },
		[160] = { V171, 0x100000000ULL * 11 / 84, },
		[161] = { V171, 0x100000000ULL * 10 / 84, },
		[162] = { V171, 0x100000000ULL *  9 / 84, },
		[163] = { V171, 0x100000000ULL *  8 / 84, },
		[164] = { V171, 0x100000000ULL *  7 / 84, },
		[165] = { V171, 0x100000000ULL *  6 / 84, },
		[166] = { V171, 0x100000000ULL *  5 / 84, },
		[167] = { V171, 0x100000000ULL *  4 / 84, },
		[168] = { V171, 0x100000000ULL *  3 / 84, },
		[169] = { V171, 0x100000000ULL *  2 / 84, },
		[170] = { V171, 0x100000000ULL *  1 / 84, },
		[171] = { V171,                           },
		[172] = { V255, 0x100000000ULL * 83 / 84, },
		[173] = { V255, 0x100000000ULL * 82 / 84, },
		[174] = { V255, 0x100000000ULL * 81 / 84, },
		[175] = { V255, 0x100000000ULL * 80 / 84, },
		[176] = { V255, 0x100000000ULL * 79 / 84, },
		[177] = { V255, 0x100000000ULL * 78 / 84, },
		[178] = { V255, 0x100000000ULL * 77 / 84, },
		[179] = { V255, 0x100000000ULL * 76 / 84, },
		[180] = { V255, 0x100000000ULL * 75 / 84, },
		[181] = { V255, 0x100000000ULL * 74 / 84, },
		[182] = { V255, 0x100000000ULL * 73 / 84, },
		[183] = { V255, 0x100000000ULL * 72 / 84, },
		[184] = { V255, 0x100000000ULL * 71 / 84, },
		[185] = { V255, 0x100000000ULL * 70 / 84, },
		[186] = { V255, 0x100000000ULL * 69 / 84, },
		[187] = { V255, 0x100000000ULL * 68 / 84, },
		[188] = { V255, 0x100000000ULL * 67 / 84, },
		[189] = { V255, 0x100000000ULL * 66 / 84, },
		[190] = { V255, 0x100000000ULL * 65 / 84, },
		[191] = { V255, 0x100000000ULL * 64 / 84, },
		[192] = { V255, 0x100000000ULL * 63 / 84, },
		[193] = { V255, 0x100000000ULL * 62 / 84, },
		[194] = { V255, 0x100000000ULL * 61 / 84, },
		[195] = { V255, 0x100000000ULL * 60 / 84, },
		[196] = { V255, 0x100000000ULL * 59 / 84, },
		[197] = { V255, 0x100000000ULL * 58 / 84, },
		[198] = { V255, 0x100000000ULL * 57 / 84, },
		[199] = { V255, 0x100000000ULL * 56 / 84, },
		[200] = { V255, 0x100000000ULL * 55 / 84, },
		[201] = { V255, 0x100000000ULL * 54 / 84, },
		[202] = { V255, 0x100000000ULL * 53 / 84, },
		[203] = { V255, 0x100000000ULL * 52 / 84, },
		[204] = { V255, 0x100000000ULL * 51 / 84, },
		[205] = { V255, 0x100000000ULL * 50 / 84, },
		[206] = { V255, 0x100000000ULL * 49 / 84, },
		[207] = { V255, 0x100000000ULL * 48 / 84, },
		[208] = { V255, 0x100000000ULL * 47 / 84, },
		[209] = { V255, 0x100000000ULL * 46 / 84, },
		[210] = { V255, 0x100000000ULL * 45 / 84, },
		[211] = { V255, 0x100000000ULL * 44 / 84, },
		[212] = { V255, 0x100000000ULL * 43 / 84, },
		[213] = { V255, 0x100000000ULL * 42 / 84, },
		[214] = { V255, 0x100000000ULL * 41 / 84, },
		[215] = { V255, 0x100000000ULL * 40 / 84, },
		[216] = { V255, 0x100000000ULL * 39 / 84, },
		[217] = { V255, 0x100000000ULL * 38 / 84, },
		[218] = { V255, 0x100000000ULL * 37 / 84, },
		[219] = { V255, 0x100000000ULL * 36 / 84, },
		[220] = { V255, 0x100000000ULL * 35 / 84, },
		[221] = { V255, 0x100000000ULL * 34 / 84, },
		[222] = { V255, 0x100000000ULL * 33 / 84, },
		[223] = { V255, 0x100000000ULL * 32 / 84, },
		[224] = { V255, 0x100000000ULL * 31 / 84, },
		[225] = { V255, 0x100000000ULL * 30 / 84, },
		[226] = { V255, 0x100000000ULL * 29 / 84, },
		[227] = { V255, 0x100000000ULL * 28 / 84, },
		[228] = { V255, 0x100000000ULL * 27 / 84, },
		[229] = { V255, 0x100000000ULL * 26 / 84, },
		[230] = { V255, 0x100000000ULL * 25 / 84, },
		[231] = { V255, 0x100000000ULL * 24 / 84, },
		[232] = { V255, 0x100000000ULL * 23 / 84, },
		[233] = { V255, 0x100000000ULL * 22 / 84, },
		[234] = { V255, 0x100000000ULL * 21 / 84, },
		[235] = { V255, 0x100000000ULL * 20 / 84, },
		[236] = { V255, 0x100000000ULL * 19 / 84, },
		[237] = { V255, 0x100000000ULL * 18 / 84, },
		[238] = { V255, 0x100000000ULL * 17 / 84, },
		[239] = { V255, 0x100000000ULL * 16 / 84, },
		[240] = { V255, 0x100000000ULL * 15 / 84, },
		[241] = { V255, 0x100000000ULL * 14 / 84, },
		[242] = { V255, 0x100000000ULL * 13 / 84, },
		[243] = { V255, 0x100000000ULL * 12 / 84, },
		[244] = { V255, 0x100000000ULL * 11 / 84, },
		[245] = { V255, 0x100000000ULL * 10 / 84, },
		[246] = { V255, 0x100000000ULL *  9 / 84, },
		[247] = { V255, 0x100000000ULL *  8 / 84, },
		[248] = { V255, 0x100000000ULL *  7 / 84, },
		[249] = { V255, 0x100000000ULL *  6 / 84, },
		[250] = { V255, 0x100000000ULL *  5 / 84, },
		[251] = { V255, 0x100000000ULL *  4 / 84, },
		[252] = { V255, 0x100000000ULL *  3 / 84, },
		[253] = { V255, 0x100000000ULL *  2 / 84, },
		[254] = { V255, 0x100000000ULL *  1 / 84, },
		[255] = { V255,                           },
	};

	u8 brightness = s6->bl;
	int last_y = 0;
	int i;
	int j = 0;
	u32 vh = v0;
	u32 vl = vh;
	u32 vt;
	u32 vb;
	u64 tmp;
	int y;
	u32 scale;

	for (i = 0; i < NUM_DY_REGS; i++) {
		vt = s6e8aa0_gamma_lookup(s6, brightness, s6->dyi_to_b[i], c);
		while (vl > vt && j < ARRAY_SIZE(output_table) - 1) {
			j++;
			vh = vl;
			vl = v[output_table[j].base];
			scale = output_table[j].scale;
			if (scale) {
				vb = v[output_table[j].base - 1];
				tmp = vb - vl;
				tmp *= scale;
				tmp >>= 32;
				vl += tmp;
			}
		}
		y = j * 4;
		if (vh > vl && vt >= vl)
			y -= DIV_ROUND_CLOSEST(4 * (vt - vl), vh - vl);
		pr_debug("%s: dy%d %d, v %d (vh %d @ %d, vl %d @ %d)\n",
			 __func__, i, y, vt, vh, j * 4 - 4, vl, j * 4);
		if (y < last_y)
			y = last_y;
		dy[i] = y - last_y;
		last_y = y;
	}
}

static void s6e8aa0_setup_gamma_regs(struct s6e8aa0_data *s6, u8 gamma_regs[],
				     u8 dy_regs[3][NUM_DY_REGS + 1])
{
	int c, i;
	u8 brightness = s6->bl;
	const struct s6e8aa0_gamma_adj_points *bv = s6->gamma_adj_points;

	for (c = 0; c < 3; c++) {
		u32 adj;
		u32 adj_min;
		u32 adj_max;
		s16 offset;
		u32 v0 = s6e8aa0_gamma_lookup(s6, brightness, BV_0, c);
		u32 v[V_COUNT];

		v[V1] = s6e8aa0_gamma_lookup(s6, brightness, bv->v1, c);
		offset = s6->gamma_reg_offsets.v[1][c][V1];
		adj_max = min(V1_ADJ_MAX, V1_ADJ_MAX - offset);
		adj_min = max(0, 0 - offset);
		adj = v1_to_v1adj(v[V1], v0) - offset;
		if (adj < adj_min || adj > adj_max) {
			pr_debug("%s: bad adj value %d, v0 %d, v1 %d, c %d\n",
				__func__, adj, v0, v[V1], c);
			adj = clamp_t(int, adj, adj_min, adj_max);
		}
#ifdef CONFIG_COLOR_CONTROL
		gamma_regs[gamma_reg_index(c, V1)] = ((adj + hacky_v1_offset[c]) > 0 && (adj <=255)) ? (adj + hacky_v1_offset[c]) : adj;
#else
		gamma_regs[gamma_reg_index(c, V1)] = adj;
#endif
		v[V1] = v1adj_to_v1(adj + offset, v0);

		v[V255] = s6e8aa0_gamma_lookup(s6, brightness, BV_255, c);
		offset = s6->gamma_reg_offsets.v[1][c][V255];
		adj_max = min(V255_ADJ_MAX, V255_ADJ_MAX - offset);
		adj_min = max(0, 0 - offset);
		adj = v255_to_v255adj(v[V255], v0) - offset;
		if (adj < adj_min || adj > adj_max) {
			pr_debug("%s: bad adj value %d, v0 %d, v255 %d, c %d\n",
				__func__, adj, v0, v[V255], c);
			adj = clamp_t(int, adj, adj_min, adj_max);
		}
		gamma_regs[3 * V255 + 2 * c] = adj >> 8;
		gamma_regs[3 * V255 + 2 * c + 1] = (adj & 0xff);
		gamma_regs[gamma_reg_index_v255_h(c)] = adj >> 8;
		gamma_regs[gamma_reg_index_v255_l(c)] = adj;
		v[V255] = v255adj_to_v255(adj + offset, v0);

		v[V15] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v15, c);
		v[V35] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v35, c);
		v[V59] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v59, c);
		v[V87] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v87, c);
		v[V171] = s6e8aa0_gamma_lookup(s6, brightness, bv->v171, c);

		for (i = V171; i >= V15; i--) {
			offset = s6->gamma_reg_offsets.v[1][c][i];
			adj_max = min(255, 255 - offset);
			adj_min = max(0, 0 - offset);
			if (v[V1] <= v[i + 1] || v[V1] <= v[i]) {
				adj = -1;
			} else {
				adj = vn_to_vnadj(i, v[i], v[V1], v[i + 1]);
				adj -= offset;
			}
			if (adj < adj_min || adj > adj_max) {
				pr_debug("%s: bad adj value %d, "
					"vh %d, v %d, c %d\n",
					__func__, adj, v[i + 1], v[i], c);
				adj = clamp_t(int, adj, adj_min, adj_max);
			}
			gamma_regs[gamma_reg_index(c, i)] = adj;
			v[i] = vnadj_to_vn(i, adj + offset, v[V1], v[i + 1]);
		}

		s6e8aa0_setup_dy_regs(s6, c, v0, v, dy_regs[c] + 1);
	}
}

static void s6e8aa0_update_acl_set(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	int i;
	unsigned int cd;
	unsigned int max_cd = 0;
	const struct s6e8aa0_acl_parameters *acl;

	/* Quietly return if you don't have a table */
	if (!pdata->acl_table_size)
		return;

	max_cd = pdata->acl_table[pdata->acl_table_size - 1].cd;

	cd = s6->bl * max_cd / 255;
	if (cd > max_cd)
		cd = max_cd;

	if (s6->acl_enable) {
		for (i = 0; i < pdata->acl_table_size; i++)
			if (cd <= pdata->acl_table[i].cd)
				break;

		if (i == pdata->acl_table_size)
			i = pdata->acl_table_size - 1;

		acl = &pdata->acl_table[i];
		if (s6->acl_cur != acl->acl_val) {
			s6e8aa0_write_block_nosync(dssdev, acl->regs,
				sizeof(acl->regs));
			s6e8aa0_write_reg(dssdev, 0xC0,
				0x01 | (s6->acl_average << 4)); /* ACL ON */

			s6->acl_cur = acl->acl_val;
		}
	} else {
		if (s6->acl_cur != 0) {
			s6->acl_cur = 0;
			s6e8aa0_write_reg(dssdev, 0xC0, 0x00); /* ACL OFF */
		}
	}
	pr_debug("%s : cur_acl=%d, %d\n", __func__, s6->acl_cur,
		s6->acl_enable);
	return;
}

static void s6e8aa0_update_elvss(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	u8 elvss_cmd[3];
	u8 elvss;
	u8 limit = 0x9F;
	unsigned int i;
	unsigned int cd;
	unsigned int max_cd = 0;

	if (!pdata->elvss_table_size)
		return;

	elvss_cmd[0] = 0xB1;
	elvss_cmd[1] = 0x04;

	max_cd = pdata->elvss_table[pdata->elvss_table_size - 1].cd;
	cd = s6->bl * max_cd / 255;

	for (i = 0; i < pdata->elvss_table_size - 1; i++)
		if (cd <= pdata->elvss_table[i].cd)
			break;

	if (i == s6->elvss_cur_i)
		return;

	s6->elvss_cur_i = i;

	elvss = s6->panel_id[2] & 0x1F; /* ELVSS Pulse 0-4bits */
	elvss += pdata->elvss_table[i].elvss_val;

	if (elvss > limit)
		elvss = limit;

	elvss_cmd[2] = elvss;

	s6e8aa0_write_block(dssdev, elvss_cmd, sizeof(elvss_cmd));
	pr_debug("%s - brightness : %d, cd : %d, elvss : %02x\n",
					__func__, s6->bl, cd, elvss);
	return;
}

static int s6e8aa0_update_brightness(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	u8 gamma_regs[NUM_GAMMA_REGS + 2];
	u8 dy_regs[3][NUM_DY_REGS + 1];

	gamma_regs[0] = 0xFA;
	gamma_regs[1] = 0x01;
	dy_regs[0][0] = 0xb8;
	dy_regs[1][0] = 0xb9;
	dy_regs[2][0] = 0xba;

	s6e8aa0_setup_gamma_regs(s6, gamma_regs + 2, dy_regs);
	s6e8aa0_write_block_nosync(dssdev, gamma_regs, sizeof(gamma_regs));
	s6e8aa0_write_block_nosync(dssdev, dy_regs[0], sizeof(dy_regs[0]));
	s6e8aa0_write_block_nosync(dssdev, dy_regs[1], sizeof(dy_regs[1]));
	s6e8aa0_write_block_nosync(dssdev, dy_regs[2], sizeof(dy_regs[2]));
	s6e8aa0_write_reg(dssdev, 0xF7, 0x01);

	s6e8aa0_update_acl_set(dssdev);
	s6e8aa0_update_elvss(dssdev);
	return 0;
}

#ifdef CONFIG_COLOR_CONTROL
void colorcontrol_update(int * v1_offsets)
{
    int i;

    for (i = 0; i < 3; i++)
	{
	    hacky_v1_offset[i] = v1_offsets[i];
	}

    if (lcd_dev->state == OMAP_DSS_DISPLAY_ACTIVE)
	s6e8aa0_update_brightness(lcd_dev);

    return;
}
EXPORT_SYMBOL(colorcontrol_update);
#endif

static u64 s6e8aa0_voltage_lookup(struct s6e8aa0_data *s6, int c, u32 v)
{
	int i;
	u32 vh = ~0, vl = ~0;
	u32 bl = 0, bh = 0;
	u64 ret;
	struct panel_s6e8aa0_data *pdata = s6->pdata;

	for (i = 0; i < pdata->gamma_table_size; i++) {
		vh = vl;
		vl = pdata->gamma_table[i].v[c];
		bh = bl;
		bl = pdata->gamma_table[i].brightness;
		if (vl <= v)
			break;
	}
	if (i == 0 || (v - vl) == 0) {
		ret = bl;
	} else {
		ret = (u64)bh * (s32)(v - vl) + (u64)bl * (vh - v);
		do_div(ret, vh - vl);
	}
	pr_debug("%s: looking for %7d c %d, "
		"found %7d:%7d, b %08x:%08x, ret %08llx\n",
		__func__, v, c, vl, vh, bl, bh, ret);
	return ret;
}

static u64 s6e8aa0_limit_brightness(u64 bc[3], u64 bcmax)
{
	int c;
	int shift;

	for (c = 0; c < 3; c++)
		if (bc[c] > bcmax)
			bcmax = bc[c];

	if (bcmax != 0xffffffff) {
		u64 tmp;
		pr_warn("s6e8aa: factory calibration info is out of range: scale to 0x%llx\n",
			bcmax);
		shift = fls(bcmax >> 32);
		tmp = (bcmax << (32 - shift)) - 1;
		do_div(tmp, 0xffffffff);
		tmp++;
		pr_warn("s6e8aa: factory calibration info is out of range: scale to 0x%llx, shift %d\n",
			tmp, shift);
		for (c = 0; c < 3; c++) {
			bc[c] <<= 32 - shift;
			do_div(bc[c], tmp);
		}
	}
	return bcmax;
}

static void s6e8aa0_apply_color_adj(
	const struct s6e8aa0_factory_calibration_info *fi, u64 bc[3])
{
	int c;
	int shift = fi->color_adj.rshift;

	if (!shift)
		return;

	for (c = 0; c < 3; c++) {
		u64 b = bc[c];
		u32 bh = b >> 32;
		u32 bl = b;
		u64 m = fi->color_adj.mult[c];
		/*
		 * Calculate ((b * m) >> shift).
		 * If b is greater than 2^32, The 64 by 32 to 64 bit (b * m)
		 * multiplication can overflow, even if the end result fits,
		 * so we split it into two 32 by 32 to 64 bit operations.
		 */
		bc[c] = ((bh * m) << (32 - shift)) + ((bl * m) >> shift);
	}
}

static int s6e8aa0_cmp_gamma_entry(const void *pa, const void *pb)
{
	u32 a = ((const struct s6e8aa0_gamma_entry *)pa)->brightness;
	u32 b = ((const struct s6e8aa0_gamma_entry *)pb)->brightness;
	if (a > b)
		return 1;
	if (a < b)
		return -1;
	return 0;
}

static void s6e8aa0_adjust_brightness_from_mtp(struct s6e8aa0_data *s6)
{
	int b, c, i;
	u32 v[2][3][V_COUNT];
	u64 bc[3];
	u64 bcmax;
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	const struct s6e8aa0_gamma_reg_offsets *offset = &s6->gamma_reg_offsets;
	struct s6e8aa0_factory_calibration_info *fi = pdata->factory_info;
	struct s6e8aa0_gamma_entry *brightness_table;
	int brightness_table_size = 1;

	for (b = 0; b < 2; b++)
		for (i = 0; i < V_COUNT; i++)
			if (fi->brightness[b][i])
				brightness_table_size++;

	brightness_table = kmalloc(sizeof(*brightness_table) *
				   brightness_table_size, GFP_KERNEL);
	if (!brightness_table) {
		dev_err(&s6->dssdev->dev,
			"Failed to allocate brightness table\n");
		return;
	}
	brightness_table->brightness = 0;
	for (c = 0; c < 3; c++)
		brightness_table->v[c] = 0;
	s6->brightness_table = brightness_table;
	s6->brightness_table_size = brightness_table_size;
	brightness_table++;

	for (b = 0; b < 2; b++) {
		for (c = 0; c < 3; c++) {
			u32 v0 = s6e8aa0_raw_gamma_lookup(s6, BV_0, c);
			v[b][c][V1] = v1adj_to_v1(fi->regs[b][c][V1] +
						  offset->v[b][c][V1], v0);
			v[b][c][V255] = v255adj_to_v255(fi->regs[b][c][V255] +
						     offset->v[b][c][V255], v0);
			for (i = V171; i >= V15; i--)
				v[b][c][i] = vnadj_to_vn(i, fi->regs[b][c][i] +
						   offset->v[b][c][i],
						   v[b][c][V1], v[b][c][i + 1]);
		}
	}

	for (b = 0; b < 2; b++)
		for (i = 0; i < V_COUNT; i++)
			pr_debug("%s: b %d, p %d, R %7dv, G %7dv, B %7dv\n",
				 __func__, b, i,
				 v[b][0][i], v[b][1][i], v[b][2][i]);

	bcmax = 0xffffffff;
	for (b = 0; b < 2; b++) {
		for (i = 0; i < V_COUNT; i++) {
			if (!fi->brightness[b][i])
				continue;

			for (c = 0; c < 3; c++)
				bc[c] = s6e8aa0_voltage_lookup(s6, c,
							       v[b][c][i]);

			s6e8aa0_apply_color_adj(fi, bc);
			bcmax = s6e8aa0_limit_brightness(bc, bcmax);
		}
	}

	for (b = 0; b < 2; b++) {
		for (i = 0; i < V_COUNT; i++) {
			if (!fi->brightness[b][i])
				continue;

			for (c = 0; c < 3; c++) {
				bc[c] = s6e8aa0_voltage_lookup(s6, c,
							       v[b][c][i]);
				pr_debug("s6e8aa: c%d, %d, b-%08llx, before scaling\n",
					 c, i, bc[c]);
			}

			s6e8aa0_apply_color_adj(fi, bc);
			for (c = 0; c < 3; c++) {
				pr_debug("s6e8aa: c%d, %d, b-%08llx, after color adj\n",
					 c, i, bc[c]);
			}

			s6e8aa0_limit_brightness(bc, bcmax);

			brightness_table->brightness = fi->brightness[b][i];
			pr_info("s6e8aa: d/b %d, p %d, b-%08x\n",
				b, i, fi->brightness[b][i]);
			for (c = 0; c < 3; c++) {
				if (bc[c] > s6->brightness_limit[c])
					s6->brightness_limit[c] = bc[c];
				brightness_table->v[c] = bc[c];
				pr_info("s6e8aa: c%d, %d, b-%08llx, got v %d, factory wants %d\n",
					c, i, bc[c],
					s6e8aa0_raw_gamma_lookup(s6, bc[c], c),
					v[b][c][i]);
			}
			brightness_table++;
		}
	}
	sort(s6->brightness_table + 1, s6->brightness_table_size - 1,
	     sizeof(*s6->brightness_table), s6e8aa0_cmp_gamma_entry, NULL);
}

static s16 s9_to_s16(s16 v)
{
	return (s16)(v << 7) >> 7;
}

static int mtp_reg_index(int c, int i)
{
	return c * (V_COUNT + 1) + i;
}

static void s6e8aa0_read_id_info(struct s6e8aa0_data *s6)
{
	struct omap_dss_device *dssdev = s6->dssdev;
	int ret;
	u8 cmd = 0xD1;

	dsi_vc_set_max_rx_packet_size(dssdev, 1, 3);
	ret = s6e8aa0_read_block(dssdev, cmd, s6->panel_id,
					ARRAY_SIZE(s6->panel_id));
	dsi_vc_set_max_rx_packet_size(dssdev, 1, 1);
	if (ret < 0) {
		pr_err("%s: Failed to read id data\n", __func__);
		return;
	}
}

static void s6e8aa0_read_mtp_info(struct s6e8aa0_data *s6, int b)
{
	int ret;
	int c, i;
	u8 mtp_data[24];
	u8 cmd = b ? 0xD3 : 0xD4;
	struct omap_dss_device *dssdev = s6->dssdev;

	s6e8aa0_write_block(dssdev, s6e8aa0_mtp_unlock,
			    ARRAY_SIZE(s6e8aa0_mtp_unlock));
	dsi_vc_set_max_rx_packet_size(dssdev, 1, 24);
	ret = s6e8aa0_read_block(dssdev, cmd, mtp_data, ARRAY_SIZE(mtp_data));
	dsi_vc_set_max_rx_packet_size(dssdev, 1, 1);
	s6e8aa0_write_block(dssdev, s6e8aa0_mtp_lock,
			    ARRAY_SIZE(s6e8aa0_mtp_lock));
	if (ret < 0) {
		pr_err("%s: Failed to read mtp data\n", __func__);
		return;
	}
	for (c = 0; c < 3; c++) {
		for (i = 0; i < V255; i++)
			s6->gamma_reg_offsets.v[b][c][i] =
				(s8)mtp_data[mtp_reg_index(c, i)];

		s6->gamma_reg_offsets.v[b][c][V255] =
			s9_to_s16(mtp_data[mtp_reg_index(c, V255)] << 8 |
				  mtp_data[mtp_reg_index(c, V255 + 1)]);
	}
}

static int s6e8aa0_set_brightness(struct backlight_device *bd)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bd->dev);
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int bl = bd->props.brightness;
	int ret = 0;

	if (bl == s6->bl)
		return 0;

	s6->bl = bl;
	mutex_lock(&s6->lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);
		ret = s6e8aa0_update_brightness(dssdev);
		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&s6->lock);
	return ret;
}

static int s6e8aa0_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static const struct backlight_ops s6e8aa0_backlight_ops  = {
	.get_brightness = s6e8aa0_get_brightness,
	.update_status = s6e8aa0_set_brightness,
};

static void seq_print_gamma_regs(struct seq_file *m, const u8 gamma_regs[])
{
	struct s6e8aa0_data *s6 = m->private;
	int c, i;
	const int adj_points[] = { 1, 15, 35, 59, 87, 171, 255 };
	const char color[] = { 'R', 'G', 'B' };
	u8 brightness = s6->bl;
	const struct s6e8aa0_gamma_adj_points *bv = s6->gamma_adj_points;
	const struct s6e8aa0_gamma_reg_offsets *offset = &s6->gamma_reg_offsets;

	for (c = 0; c < 3; c++) {
		u32 adj[V_COUNT];
		u32 vt[V_COUNT];
		u32 v[V_COUNT];
		u32 v0 = s6e8aa0_gamma_lookup(s6, brightness, BV_0, c);

		vt[V1] = s6e8aa0_gamma_lookup(s6, brightness, bv->v1, c);
		vt[V15] = s6e8aa0_gamma_lookup(s6, brightness, bv->v15, c);
		vt[V35] = s6e8aa0_gamma_lookup(s6, brightness, bv->v35, c);
		vt[V59] = s6e8aa0_gamma_lookup(s6, brightness, bv->v59, c);
		vt[V87] = s6e8aa0_gamma_lookup(s6, brightness, bv->v87, c);
		vt[V171] = s6e8aa0_gamma_lookup(s6, brightness, bv->v171, c);
		vt[V255] = s6e8aa0_gamma_lookup(s6, brightness, BV_255, c);

		adj[V1] = gamma_regs[gamma_reg_index(c, V1)];
		v[V1] = v1adj_to_v1(adj[V1] + offset->v[1][c][V1], v0);

		adj[V255] = gamma_regs[gamma_reg_index_v255_h(c)] << 8 |
			    gamma_regs[gamma_reg_index_v255_l(c)];
		v[V255] = v255adj_to_v255(adj[V255] + offset->v[1][c][V255],
					  v0);

		for (i = V171; i >= V15; i--) {
			adj[i] = gamma_regs[gamma_reg_index(c, i)];
			v[i] = vnadj_to_vn(i, adj[i] + offset->v[1][c][i],
					   v[V1], v[i + 1]);
		}
		seq_printf(m, "%c                   v0   %7d\n",
			   color[c], v0);
		for (i = 0; i < V_COUNT; i++) {
			seq_printf(m, "%c adj %3d (%02x) %+4d "
				   "v%-3d %7d - %7d %+8d\n",
				   color[c], adj[i], adj[i], offset->v[1][c][i],
				   adj_points[i], v[i], vt[i], v[i] - vt[i]);
		}
	}
}

static void seq_print_dy_regs(struct seq_file *m,
			      u8 dy_regs[3][NUM_DY_REGS + 1])
{
	int i, c;
	u16 y[3] = {};
	seq_printf(m, "    R  y (rv) G  y (rv) B  y (rv)\n");
	for (i = 0; i < NUM_DY_REGS; i++) {
		seq_printf(m, "%-2d:", i);
		for (c = 0; c < 3; c++) {
			y[c] += dy_regs[c][i + 1];
			seq_printf(m, " %4d (%02x)", y[c], dy_regs[c][i + 1]);
		}
		seq_printf(m, "\n");
	}
}

static int s6e8aa0_current_gamma_show(struct seq_file *m, void *unused)
{
	struct s6e8aa0_data *s6 = m->private;
	u8 gamma_regs[NUM_GAMMA_REGS];
	u8 dy_regs[3][NUM_DY_REGS + 1];

	mutex_lock(&s6->lock);
	s6e8aa0_setup_gamma_regs(s6, gamma_regs, dy_regs);
	seq_printf(m, "brightness %3d:\n", s6->bl);
	seq_print_gamma_regs(m, gamma_regs);
	seq_printf(m, "\n");
	seq_print_dy_regs(m, dy_regs);
	mutex_unlock(&s6->lock);
	return 0;
}

static int s6e8aa0_current_gamma_open(struct inode *inode, struct file *file)
{
	return single_open(file, s6e8aa0_current_gamma_show, inode->i_private);
}

static int s6e8aa0_gamma_correction_show(struct seq_file *m, void *unused)
{
	struct s6e8aa0_data *s6 = m->private;
	const struct s6e8aa0_gamma_entry *bte;
	int n, c;

	mutex_lock(&s6->lock);
	n = s6->brightness_table_size;
	bte = s6->brightness_table;
	while (n--) {
		seq_printf(m, "0x%08x", bte->brightness);
		for (c = 0; c < 3; c++)
			seq_printf(m, " 0x%08x", bte->v[c]);
		seq_printf(m, "\n");
		bte++;
	}
	seq_printf(m, "\n");
	seq_printf(m, "0x%08x", BV_255);
	for (c = 0; c < 3; c++)
		seq_printf(m, " 0x%08x", s6->brightness_limit[c]);
	seq_printf(m, "\n");
	mutex_unlock(&s6->lock);
	return 0;
}

static int s6e8aa0_gamma_correction_open(struct inode *inode, struct file *file)
{
	return single_open(file, s6e8aa0_gamma_correction_show,
			   inode->i_private);
}

static ssize_t s6e8aa0_gamma_correction_write(struct file *file,
				       const char __user *buf,
				       size_t size, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct s6e8aa0_data *s6 = m->private;
	struct omap_dss_device *dssdev = s6->dssdev;
	char sbuf[80];
	u32 val[4] = {
	};
	u32 last_val[4];
	struct s6e8aa0_gamma_entry *bt = NULL;
	struct s6e8aa0_gamma_entry *new_bt;
	int bt_size = 0;

	int ret;
	size_t used;
	size_t sbuf_len = sizeof(sbuf) - 1;
	size_t rem = size;
	int c;
	int i;

	while (rem && val[0] != BV_255) {
		if (sbuf_len > rem)
			sbuf_len = rem;
		if (copy_from_user(sbuf, buf, sbuf_len)) {
			ret = -EFAULT;
			goto err;
		}
		sbuf[sbuf_len] = '\0';

		for (i = 0; i < ARRAY_SIZE(val); i++)
			last_val[i] = val[i];
		ret = sscanf(sbuf, "%i %i %i %i\n%n",
			     &val[0], &val[1], &val[2], &val[3], &used);
		if (ret < 4 || !used)
			break;

		buf += used;
		rem -= used;

		if (!bt_size) {
			for (i = 0; i < ARRAY_SIZE(val); i++) {
				if (val[i] != 0) {
					pr_info("%s: invalid start value %d: "
						"0x%08x != 0\n",
						__func__, i, val[i]);
					ret = -EINVAL;
					goto err;
				}
			}
		} else {
			for (i = 0; i < ARRAY_SIZE(val); i++) {
				if (val[i] <= last_val[i]) {
					pr_info("%s: invalid value %d: "
						"0x%08x <= 0x%08x\n", __func__,
						i, val[i], last_val[i]);
					ret = -EINVAL;
					goto err;
				}
			}
			for (c = 0; c < 3; c++) {
				if (val[c + 1] > s6->brightness_limit[c]) {
					pr_info("%s: invalid value %d: "
						"0x%08x > 0x%08x\n", __func__,
						c, val[c + 1],
						s6->brightness_limit[c]);
					ret = -EOVERFLOW;
					goto err;
				}
			}
		}

		new_bt = krealloc(bt, (bt_size + 1) * sizeof(*bt), GFP_KERNEL);
		if (!new_bt) {
			ret = -ENOMEM;
			goto err;
		}
		bt = new_bt;
		bt[bt_size].brightness = val[0];
		for (c = 0; c < 3; c++)
			bt[bt_size].v[c] = val[c + 1];
		bt_size++;
	}
	if (val[0] != BV_255) {
		ret = -EINVAL;
		goto err;
	}

	ret = size - rem;
	mutex_lock(&s6->lock);
	pr_debug("%s: got new brightness_table size %d\n", __func__, bt_size);
	swap(bt, s6->brightness_table);
	s6->brightness_table_size = bt_size;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);
		s6e8aa0_update_brightness(dssdev);
		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&s6->lock);

err:
	kfree(bt);
	return ret;
}

static ssize_t acl_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(dev);
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	snprintf(buf, PAGE_SIZE, "%d\n", s6->acl_enable);

	return strlen(buf);
}

static ssize_t acl_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(dev);
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	long value;
	bool enable;
	int rc;

	rc = strict_strtol(buf, 0, &value);

	if (rc < 0)
		return rc;

	enable = value;

	mutex_lock(&s6->lock);
	if (s6->acl_enable != enable) {
		dsi_bus_lock(dssdev);

		s6->acl_enable = enable;
		s6e8aa0_update_acl_set(dssdev);

		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&s6->lock);
	return size;
}

static DEVICE_ATTR(acl_set, S_IRUGO|S_IWUSR,
		acl_enable_show, acl_enable_store);


static ssize_t acl_average_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(dev);
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	snprintf(buf, PAGE_SIZE, "%d\n", s6->acl_average);

	return strlen(buf);
}

static ssize_t acl_average_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(dev);
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	long value;
	int rc;

	rc = strict_strtol(buf, 0, &value);

	if (rc < 0)
		return rc;

	if (value < 0 || value > 7)
		return -EINVAL;

	mutex_lock(&s6->lock);
	if (s6->acl_average != value) {
		dsi_bus_lock(dssdev);

		s6->acl_average = value;
		s6->acl_cur = 0;
		s6e8aa0_update_acl_set(dssdev);

		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&s6->lock);
	return size;
}

static DEVICE_ATTR(acl_average, S_IRUGO|S_IWUSR,
		acl_average_show, acl_average_store);

static struct attribute *s6e8aa0_bl_attributes[] = {
	&dev_attr_acl_set.attr,
	&dev_attr_acl_average.attr,
	NULL
};

static const struct attribute_group s6e8aa0_bl_attr_group = {
	.attrs = s6e8aa0_bl_attributes,
};

static const struct file_operations s6e8aa0_current_gamma_fops = {
	.open = s6e8aa0_current_gamma_open,
	.read = seq_read,
	.release = single_release,
};

static const struct file_operations s6e8aa0_gamma_correction_fops = {
	.open = s6e8aa0_gamma_correction_open,
	.read = seq_read,
	.write = s6e8aa0_gamma_correction_write,
	.release = single_release,
};

static int s6e8aa0_probe(struct omap_dss_device *dssdev)
{
	int ret = 0;
	struct backlight_properties props = {
		.brightness = 255,
		.max_brightness = 255,
		.type = BACKLIGHT_RAW,
	};
	struct s6e8aa0_data *s6 = NULL;

	dev_dbg(&dssdev->dev, "s6e8aa0_probe\n");

	if (dssdev->data == NULL) {
		dev_err(&dssdev->dev, "no platform data!\n");
		return -EINVAL;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = s6e8aa0_timings;

	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	s6 = kzalloc(sizeof(*s6), GFP_KERNEL);
	if (!s6)
		return -ENOMEM;

	s6->dssdev = dssdev;
	s6->pdata = dssdev->data;

	s6->bl = props.brightness;

	if (!s6->pdata->seq_display_set || !s6->pdata->seq_etc_set
		|| !s6->pdata->gamma_table) {
		dev_err(&dssdev->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err;
	}
	s6->gamma_adj_points =
		s6->pdata->gamma_adj_points ?: &default_gamma_adj_points;
	s6->dyi_to_b = s6e8aa0_srgb_dyi_to_b;

	ret = gpio_request(s6->pdata->reset_gpio, "s6e8aa0_reset");
	if (ret < 0) {
		dev_err(&dssdev->dev, "gpio_request %d failed!\n", s6->pdata->reset_gpio);
		goto err;
	}
	gpio_direction_output(s6->pdata->reset_gpio, 1);

	mutex_init(&s6->lock);

	atomic_set(&s6->do_update, 0);

	dev_set_drvdata(&dssdev->dev, s6);

	/* Register DSI backlight  control */
	s6->bldev = backlight_device_register("s6e8aa0", &dssdev->dev, dssdev,
					      &s6e8aa0_backlight_ops, &props);
	if (IS_ERR(s6->bldev)) {
		ret = PTR_ERR(s6->bldev);
		goto err_backlight_device_register;
	}

	s6->debug_dir = debugfs_create_dir("s6e8aa0", NULL);
	if (!s6->debug_dir) {
		dev_err(&dssdev->dev, "failed to create debug dir\n");
	} else {
		debugfs_create_file("current_gamma", S_IRUGO,
			s6->debug_dir, s6, &s6e8aa0_current_gamma_fops);
		debugfs_create_file("gamma_correction", S_IRUGO | S_IWUSR,
			s6->debug_dir, s6, &s6e8aa0_gamma_correction_fops);
	}

	s6->acl_enable = true;
	s6->acl_cur = 0;
	s6->acl_average = s6->pdata->acl_average;
	s6->elvss_cur_i = ~0;

	ret = sysfs_create_group(&s6->bldev->dev.kobj, &s6e8aa0_bl_attr_group);
	if (ret < 0) {
		dev_err(&dssdev->dev, "failed to add sysfs entries\n");
		goto err_backlight_device_register;
	}

	if (cpu_is_omap44xx())
		s6->force_update = true;

#ifdef CONFIG_COLOR_CONTROL
	lcd_dev = dssdev;
#endif

	dev_dbg(&dssdev->dev, "s6e8aa0_probe\n");
	return ret;

err_backlight_device_register:
	mutex_destroy(&s6->lock);
	gpio_free(s6->pdata->reset_gpio);
err:
	kfree(s6);

	return ret;
}

static void s6e8aa0_remove(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	sysfs_remove_group(&s6->bldev->dev.kobj, &s6e8aa0_bl_attr_group);
	debugfs_remove_recursive(s6->debug_dir);
	backlight_device_unregister(s6->bldev);
	mutex_destroy(&s6->lock);
	gpio_free(s6->pdata->reset_gpio);
	kfree(s6);
}

/**
 * s6e8aa0_config - Configure S6E8AA0
 *
 * Initial configuration for S6E8AA0 configuration registers, PLL...
 */
static void s6e8aa0_config(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	if (!s6->brightness_table) {
		s6e8aa0_read_id_info(s6);
		s6e8aa0_read_mtp_info(s6, 0);
		s6e8aa0_read_mtp_info(s6, 1);
		s6e8aa0_adjust_brightness_from_mtp(s6);
	}

	s6e8aa0_write_sequence(dssdev, pdata->seq_display_set,
			       pdata->seq_display_set_size);

	s6->acl_cur = 0; /* make sure acl table and elvss value gets written */
	s6->elvss_cur_i = ~0;
	s6e8aa0_update_brightness(dssdev);

	s6e8aa0_write_sequence(dssdev, pdata->seq_etc_set,
			       pdata->seq_etc_set_size);
}

static int s6e8aa0_power_on(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret = 0;

	/* At power on the first vsync has not been received yet*/
	dssdev->first_vsync = false;

	if (s6->enabled != 1) {
		if (s6->pdata->set_power)
			s6->pdata->set_power(true);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		/* reset s6e8aa0 bridge */
		if(!dssdev->skip_init){
			s6e8aa0_hw_reset(dssdev);

			/* XXX */
			msleep(100);
			s6e8aa0_config(dssdev);

			dsi_video_mode_enable(dssdev, 0x3E); /* DSI_DT_PXLSTREAM_24BPP_PACKED; */
		}

		s6->enabled = 1;
	}

	if(dssdev->skip_init)
		dssdev->skip_init = false;

err:
	return ret;
}

static void s6e8aa0_power_off(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(s6->pdata->reset_gpio, 0);
	msleep(10);

	s6->enabled = 0;
	omapdss_dsi_display_disable(dssdev, 0, 0);

	if (s6->pdata->set_power)
		s6->pdata->set_power(false);

}

static int s6e8aa0_start(struct omap_dss_device *dssdev)
{
	int r = 0;
	unsigned long pclk;

	dsi_bus_lock(dssdev);

	r = s6e8aa0_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dssdev->manager->enable(dssdev->manager);
	}

	/* fixup pclk based on pll config */
	pclk = dispc_pclk_rate(dssdev->channel);
	if (pclk)
		dssdev->panel.timings.pixel_clock = (pclk + 500) / 1000;

	return r;
}

static void s6e8aa0_stop(struct omap_dss_device *dssdev)
{
	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	s6e8aa0_power_off(dssdev);

	dsi_bus_unlock(dssdev);
}

static void s6e8aa0_disable(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&s6->lock);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	mutex_unlock(&s6->lock);
}

static int s6e8aa0_enable(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret;

	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&s6->lock);
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		ret = -EINVAL;
		goto out;
	}

	ret = s6e8aa0_start(dssdev);
out:
	mutex_unlock(&s6->lock);
	return ret;
}

static void s6e8aa0_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(dssdev);
}

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int r;
	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&s6->lock);

	dsi_bus_lock(dssdev);

	if (!s6->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	/* We use VC(0) for VideoPort Data and VC(1) for commands */
	r = omap_dsi_update(dssdev, 0, x, y, w, h, s6e8aa0_framedone_cb, dssdev);
	if (r)
		goto err;

	dsi_bus_unlock(dssdev);
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&s6->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&s6->lock);
	return r;
}

static int s6e8aa0_sync(struct omap_dss_device *dssdev)
{
	/* TODO? */
	return 0;
}

static int s6e8aa0_set_update_mode(struct omap_dss_device *dssdev,
			       enum omap_dss_update_mode mode)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode s6e8aa0_get_update_mode(struct omap_dss_device
						     *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->force_update)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

#ifdef CONFIG_PM
static int s6e8aa0_resume(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret;

	dev_dbg(&dssdev->dev, "resume\n");

	mutex_lock(&s6->lock);
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		ret = -EINVAL;
		goto out;
	}

	ret = s6e8aa0_start(dssdev);
out:
	mutex_unlock(&s6->lock);
	return ret;
}

static int s6e8aa0_suspend(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret = 0;

	dev_dbg(&dssdev->dev, "suspend\n");

	mutex_lock(&s6->lock);
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		ret = -EINVAL;
		goto out;
	}

	s6e8aa0_stop(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
out:
	mutex_unlock(&s6->lock);
	return ret;
}
#endif

static struct omap_dss_driver s6e8aa0_driver = {
	.probe = s6e8aa0_probe,
	.remove = s6e8aa0_remove,

	.enable = s6e8aa0_enable,
	.disable = s6e8aa0_disable,
#ifdef CONFIG_PM
	.suspend = s6e8aa0_suspend,
	.resume = s6e8aa0_resume,
#endif

	.set_update_mode = s6e8aa0_set_update_mode,
	.get_update_mode = s6e8aa0_get_update_mode,

	.update = s6e8aa0_update,
	.sync = s6e8aa0_sync,

	.get_resolution = s6e8aa0_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	/* dummy entry start */
	.enable_te = s6e8aa0_enable_te,
	.set_rotate = s6e8aa0_rotate,
	.get_rotate = s6e8aa0_get_rotate,
	.set_mirror = s6e8aa0_mirror,
	.get_mirror = s6e8aa0_get_mirror,
	/* dummy entry end */

	.get_timings = s6e8aa0_get_timings,
	.set_timings = s6e8aa0_set_timings,
	.check_timings = s6e8aa0_check_timings,

	.driver = {
		   .name = "s6e8aa0",
		   .owner = THIS_MODULE,
		   },
};

static int __init s6e8aa0_init(void)
{
	omap_dss_register_driver(&s6e8aa0_driver);
	return 0;
}

static void __exit s6e8aa0_exit(void)
{
	omap_dss_unregister_driver(&s6e8aa0_driver);
}

module_init(s6e8aa0_init);
module_exit(s6e8aa0_exit);
