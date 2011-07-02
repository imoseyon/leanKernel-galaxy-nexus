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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>


#include <video/omapdss.h>

#include <linux/platform_data/panel-s6e8aa0.h>

/* DSI Command Virtual channel */
#define CMD_VC_CHANNEL 1

#define V1_ADJ_MAX 140
#define V255_ADJ_MAX 430
#define NUM_GAMMA_REGS	24

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
	.v0 = BV_0,
	.v1 = BV_1,
	.v15 = BV_15,
	.v35 = BV_35,
	.v59 = BV_59,
	.v87 = BV_87,
	.v171 = BV_171,
	.v255 = BV_255,
};

struct s6e8aa0_gamma_reg_offsets {
	s16 v[3][7];
};

struct s6e8aa0_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;
	struct backlight_device *bldev;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned int bl;
	const struct s6e8aa0_gamma_adj_points *gamma_adj_points;
	struct s6e8aa0_gamma_reg_offsets gamma_reg_offsets;
	u32 color_mult[3];
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
	int ret;
	msleep(10);  // XxX: why do we have to wait

	ret = dsi_vc_dcs_write(dssdev, 1, (u8 *)data, len);
	msleep(10);  // XxX: why do we have to wait
	return ret;
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

static u32 s6e8aa0_gamma_lookup(struct s6e8aa0_data *s6,
				u8 brightness, u32 val, int c)
{
	int i;
	u32 bl = 0;
	u32 bh = 0;
	u32 vl = 0;
	u32 vh;
	u32 b;
	u32 ret;
	u64 tmp;
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	const struct s6e8aa0_gamma_adj_points *bv = s6->gamma_adj_points;

	if (!val) {
		b = 0;
	} else {
		tmp = bv->v255 - bv->v0;
		tmp *= brightness;
		do_div(tmp, 255);

		tmp *= s6->color_mult[c];
		do_div(tmp, 0xffffffff);

		tmp *= (val - bv->v0);
		do_div(tmp, bv->v255 - bv->v0);
		b = tmp + bv->v0;
	}

	for (i = 0; i < pdata->gamma_table_size; i++) {
		bl = bh;
		bh = pdata->gamma_table[i].brightness;
		if (bh >= b)
			break;
	}
	vh = pdata->gamma_table[i].v[c];
	if (i == 0 || (b - bl) == 0) {
		ret = vl = vh;
	} else {
		vl = pdata->gamma_table[i - 1].v[c];
		tmp = (u64)vh * (b - bl) + (u64)vl * (bh - b);
		do_div(tmp, bh - bl);
		ret = tmp;
	}

	pr_debug("%s: looking for %3d %08x c %d, %08x, "
		"found %08x:%08x, v %7d:%7d, ret %7d\n",
		__func__, brightness, val, c, b, bl, bh, vl, vh, ret);

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

static void s6e8aa0_setup_gamma_regs(struct s6e8aa0_data *s6, u8 gamma_regs[])
{
	int c, i;
	u8 brightness = s6->bl;
	const struct s6e8aa0_gamma_adj_points *bv = s6->gamma_adj_points;

	for (c = 0; c < 3; c++) {
		u32 adj;
		u32 v0 = s6e8aa0_gamma_lookup(s6, brightness, BV_0, c);
		u32 v[V_COUNT];

		v[V1] = s6e8aa0_gamma_lookup(s6, brightness, bv->v1, c);
		adj = v1_to_v1adj(v[V1], v0);
		adj -= s6->gamma_reg_offsets.v[c][V1];
		if (adj > V1_ADJ_MAX) {
			pr_debug("%s: bad adj value %d, v0 %d, v1 %d, c %d\n",
				__func__, adj, v0, v[V1], c);
			if ((int)adj < 0)
				adj = 0;
			else
				adj = V1_ADJ_MAX;
		}
		gamma_regs[gamma_reg_index(c, V1)] = adj;

		v[V255] = s6e8aa0_gamma_lookup(s6, brightness, bv->v255, c);
		adj = v255_to_v255adj(v[V255], v0);
		adj -= s6->gamma_reg_offsets.v[c][V255];
		if (adj > V255_ADJ_MAX) {
			pr_debug("%s: bad adj value %d, v0 %d, v255 %d, c %d\n",
				__func__, adj, v0, v[V255], c);
			if ((int)adj < 0)
				adj = 0;
			else
				adj = V255_ADJ_MAX;
		}
		gamma_regs[3 * V255 + 2 * c] = adj >> 8;
		gamma_regs[3 * V255 + 2 * c + 1] = (adj & 0xff);
		gamma_regs[gamma_reg_index_v255_h(c)] = adj >> 8;
		gamma_regs[gamma_reg_index_v255_l(c)] = adj;

		v[V15] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v15, c);
		v[V35] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v35, c);
		v[V59] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v59, c);
		v[V87] = s6e8aa0_gamma_lookup(s6, brightness,  bv->v87, c);
		v[V171] = s6e8aa0_gamma_lookup(s6, brightness, bv->v171, c);

		for (i = V171; i >= V15; i--) {
			if (v[V1] <= v[i + 1]) {
				adj = -1;
			} else {
				adj = vn_to_vnadj(i, v[i], v[V1], v[i + 1]);
				adj -= s6->gamma_reg_offsets.v[c][i];
			}
			if (adj > 255) {
				pr_debug("%s: bad adj value %d, "
					"vh %d, v %d, c %d\n",
					__func__, adj, v[i + 1], v[i], c);
				if ((int)adj < 0)
					adj = 0;
				else
					adj = 255;
			}
			gamma_regs[gamma_reg_index(c, i)] = adj;
		}
	}
}

static int s6e8aa0_update_brightness(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret = 0;
	u8 gamma_regs[NUM_GAMMA_REGS + 2];

	gamma_regs[0] = 0xFA;
	gamma_regs[1] = 0x01;

	s6e8aa0_setup_gamma_regs(s6, gamma_regs + 2);
	s6e8aa0_write_block(dssdev, gamma_regs, sizeof(gamma_regs));
	s6e8aa0_write_reg(dssdev, 0xF7, 0x01);

	return ret;
}

static u64 s6e8aa0_voltage_lookup(struct s6e8aa0_data *s6, int c, u32 v)
{
	int i;
	u32 vh = ~0, vl = ~0;
	u32 bl, bh = 0;
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

static void s6e8aa0_adjust_brightness_from_mtp(struct s6e8aa0_data *s6)
{
	int c;
	u32 v255[3];
	u64 bc[3];
	u64 bcmax;
	int shift;
	struct panel_s6e8aa0_data *pdata = s6->pdata;
	const struct s6e8aa0_gamma_reg_offsets *offset = &s6->gamma_reg_offsets;
	const u16 *factory_v255_regs = pdata->factory_v255_regs;

	for (c = 0; c < 3; c++) {
		u32 v0 = s6e8aa0_gamma_lookup(s6, 255, BV_0, c);
		v255[c] = v255adj_to_v255(factory_v255_regs[c] +
					  offset->v[c][V255], v0);
		bc[c] = s6e8aa0_voltage_lookup(s6, c, v255[c]);
	}

	shift = pdata->color_adj.rshift;
	if (shift)
		for (c = 0; c < 3; c++)
			bc[c] = bc[c] * pdata->color_adj.mult[c] >> shift;

	bcmax = 0xffffffff;
	for (c = 0; c < 3; c++)
		if (bc[c] > bcmax)
			bcmax = bc[c];

	if (bcmax != 0xffffffff) {
		pr_warn("s6e8aa: factory calibration info is out of range: "
			"scale to 0x%llx\n", bcmax);
		bcmax += 1;
		shift = fls(bcmax >> 32);
		for (c = 0; c < 3; c++) {
			bc[c] <<= 32 - shift;
			do_div(bc[c], bcmax >> shift);
		}
	}

	for (c = 0; c < 3; c++) {
		s6->color_mult[c] = bc[c];
		pr_info("s6e8aa: c%d, b-%08llx, got v %d, factory wants %d\n",
			c, bc[c], s6e8aa0_gamma_lookup(s6, 255, BV_255, c),
			v255[c]);
	}
}

static s16 s9_to_s16(s16 v)
{
	return (s16)(v << 7) >> 7;
}

static int mtp_reg_index(int c, int i)
{
	return c * (V_COUNT + 1) + i;
}

static void s6e8aa0_read_mtp_info(struct s6e8aa0_data *s6)
{
	int ret;
	int c, i;
	u8 mtp_data[24];
	struct omap_dss_device *dssdev = s6->dssdev;

	s6e8aa0_write_block(dssdev, s6e8aa0_mtp_unlock,
			    ARRAY_SIZE(s6e8aa0_mtp_unlock));
	dsi_vc_set_max_rx_packet_size(dssdev, 1, 24);
	ret = s6e8aa0_read_block(dssdev, 0xD3, mtp_data, ARRAY_SIZE(mtp_data));
	dsi_vc_set_max_rx_packet_size(dssdev, 1, 1);
	s6e8aa0_write_block(dssdev, s6e8aa0_mtp_lock,
			    ARRAY_SIZE(s6e8aa0_mtp_lock));
	if (ret < 0) {
		pr_err("%s: Failed to read mtp data\n", __func__);
		return;
	}
	for (c = 0; c < 3; c++) {
		for (i = 0; i < V255; i++)
			s6->gamma_reg_offsets.v[c][i] =
				(s8)mtp_data[mtp_reg_index(c, i)];

		s6->gamma_reg_offsets.v[c][V255] =
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

	if (cpu_is_omap44xx())
		s6->force_update = true;

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
	if (!s6->color_mult[0]) {
		s6e8aa0_read_mtp_info(s6);
		s6e8aa0_adjust_brightness_from_mtp(s6);
	}

	s6e8aa0_write_sequence(dssdev, pdata->seq_display_set,
			       pdata->seq_display_set_size);

	s6e8aa0_update_brightness(dssdev);

	s6e8aa0_write_sequence(dssdev, pdata->seq_etc_set,
			       pdata->seq_etc_set_size);
}

static int s6e8aa0_power_on(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret = 0;

	if (s6->enabled != 1) {
		if (s6->pdata->set_power)
			s6->pdata->set_power(true);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		/* reset s6e8aa0 bridge */
		s6e8aa0_hw_reset(dssdev);

		/* XXX */
		msleep(100);
		s6e8aa0_config(dssdev);

		dsi_video_mode_enable(dssdev, 0x3E); /* DSI_DT_PXLSTREAM_24BPP_PACKED; */

		s6->enabled = 1;
	}

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
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&s6->lock);

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

	mutex_unlock(&s6->lock);

	return r;
}

static void s6e8aa0_stop(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&s6->lock);

	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock(dssdev);

	s6e8aa0_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	mutex_unlock(&s6->lock);
}

static void s6e8aa0_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int s6e8aa0_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return s6e8aa0_start(dssdev);
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
	dev_dbg(&dssdev->dev, "resume\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return s6e8aa0_start(dssdev);
}

static int s6e8aa0_suspend(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "suspend\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
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
