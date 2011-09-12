/*
 * hdmi.c
 *
 * HDMI interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Yong Zhi
 *	Mythri pk <mythripk@ti.com>
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

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/omapfb.h>

#include "dss.h"
#include "dss_features.h"

#define HDMI_WP			0x0
#define HDMI_CORE_SYS		0x400
#define HDMI_CORE_AV		0x900
#define HDMI_PLLCTRL		0x200
#define HDMI_PHY		0x300

/* HDMI EDID Length move this */
#define HDMI_EDID_MAX_LENGTH			256
#define EDID_TIMING_DESCRIPTOR_SIZE		0x12
#define EDID_DESCRIPTOR_BLOCK0_ADDRESS		0x36
#define EDID_DESCRIPTOR_BLOCK1_ADDRESS		0x80
#define EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR	4
#define EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR	4

#define OMAP_HDMI_TIMINGS_NB			34

#define GPIO_HDMI_HPD		63

static struct {
	struct mutex lock;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
	struct hdmi_ip_data hdmi_data;
	int code;
	int mode;
	u8 edid[HDMI_EDID_MAX_LENGTH];
	u8 edid_set;

	bool custom_set;
	enum hdmi_deep_color_mode deep_color;
	struct hdmi_config cfg;
	struct regulator *hdmi_reg;

	struct clk *sys_clk;
	struct clk *hdmi_clk;

	int runtime_count;
} hdmi;

/*
 * Logic for the below structure :
 * user enters the CEA or VESA timings by specifying the HDMI/DVI code.
 * There is a correspondence between CEA/VESA timing and code, please
 * refer to section 6.3 in HDMI 1.3 specification for timing code.
 *
 * In the below structure, cea_vesa_timings corresponds to all OMAP4
 * supported CEA and VESA timing values.code_cea corresponds to the CEA
 * code, It is used to get the timing from cea_vesa_timing array.Similarly
 * with code_vesa. Code_index is used for back mapping, that is once EDID
 * is read from the TV, EDID is parsed to find the timing values and then
 * map it to corresponding CEA or VESA index.
 */

struct fb_videomode cea_timings[] = {
	/* 640x480 at 60.00 Hz */
	[1] = { NULL, 60,
		640, 480, 39682, 48, 16, 33, 10, 96, 2,
		0, FB_VMODE_NONINTERLACED, },
	/* 720x480 at 60.00 Hz */
	[2] = { NULL, 60,
		720, 480, 37000, 60, 16, 30, 9, 62, 6,
		0, FB_VMODE_NONINTERLACED, },
	/* 720x480 at 60.00 Hz */
	[3] = { NULL, 60,
		720, 480, 37000, 60, 16, 30, 9, 62, 6,
		0, FB_VMODE_NONINTERLACED, },
	/* 1280x720 at 60.00 Hz */
	[4] = { NULL, 60,
		1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1920x540 at 60.05 Hz */
	[5] = { NULL, 60,
		1920, 1080, 13468, 148, 88, 15, 2, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED, },
	/* 1440x240 at 60.11 Hz */
	[6] = { NULL, 60,
		1440, 480, 37000, 114, 38, 15, 4, 124, 3,
		0, FB_VMODE_INTERLACED, },
	/* 1440x240 at 60.11 Hz */
	[7] = { NULL, 60,
		1440, 480, 37000, 114, 38, 15, 4, 124, 3,
		0, FB_VMODE_INTERLACED, },
	/* 1920x1080 at 60.00 Hz */
	[16] = { NULL, 60,
		1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 720x576 at 50.00 Hz */
	[17] = { NULL, 50,
		720, 576, 37037, 68, 12, 39, 5, 64, 5,
		0, FB_VMODE_NONINTERLACED, },
	/* 720x576 at 50.00 Hz */
	[18] = { NULL, 50,
		720, 576, 37037, 68, 12, 39, 5, 64, 5,
		0, FB_VMODE_NONINTERLACED, },
	/* 1280x720 at 50.00 Hz */
	[19] = { NULL, 50,
		1280, 720, 13468, 220, 440, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1920x540 at 50.04 Hz */
	[20] = { NULL, 50,
		1920, 1080, 13468, 148, 528, 15, 2, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED, },
	/* 1440x288 at 50.08 Hz */
	[21] = { NULL, 50,
		1440, 576, 37037, 138, 24, 19, 2, 126, 3,
		0, FB_VMODE_INTERLACED, },
	/* 1440x288 at 50.08 Hz */
	[22] = { NULL, 50,
		1440, 576, 37037, 138, 24, 19, 2, 126, 3,
		0, FB_VMODE_INTERLACED, },
	/* 1440x576 at 50.00 Hz */
	[29] = { NULL, 50,
		1440, 576, 18518, 136, 24, 39, 5, 128, 5,
		0, FB_VMODE_NONINTERLACED, },
	/* 1440x576 at 50.00 Hz */
	[30] = { NULL, 50,
		1440, 576, 18518, 136, 24, 39, 5, 128, 5,
		0, FB_VMODE_NONINTERLACED, },
	/* 1920x1080 at 50.00 Hz */
	[31] = { NULL, 50,
		1920, 1080, 6734, 148, 528, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1920x1080 at 24.00 Hz */
	[32] = { NULL, 24,
		1920, 1080, 13468, 148, 638, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 2880x480 at 60.00 Hz */
	[35] = { NULL, 60,
		2880, 480, 9250, 240, 64, 30, 9, 248, 6,
		0, FB_VMODE_NONINTERLACED, },
	/* 2880x480 at 60.00 Hz */
	[36] = { NULL, 60,
		2880, 480, 9250, 240, 64, 30, 9, 248, 6,
		0, FB_VMODE_NONINTERLACED, },
	/* 2880x576 at 50.00 Hz */
	[37] = { NULL, 50,
		2880, 576, 9259, 272, 48, 39, 5, 256, 5,
		0, FB_VMODE_NONINTERLACED, },
	/* 2880x576 at 50.00 Hz */
	[38] = { NULL, 50,
		2880, 576, 9259, 272, 48, 39, 5, 256, 5,
		0, FB_VMODE_NONINTERLACED, },
};
struct fb_videomode vesa_timings[] = {
	/* 640x480 at 60.05 Hz */
	[4] = { NULL, 60,
		640, 480, 39721, 48, 16, 31, 11, 96, 2,
		0, FB_VMODE_NONINTERLACED, },
	/* 800x600 at 60.32 Hz */
	[9] = { NULL, 60,
		800, 600, 25000, 88, 40, 23, 1, 128, 4,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 848x480 at 60.00 Hz */
	[14] = { NULL, 60,
		848, 480, 29629, 112, 16, 23, 6, 112, 8,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1024x768 at 60.00 Hz */
	[16] = { NULL, 60,
		1024, 768, 15384, 160, 24, 29, 3, 136, 6,
		0, FB_VMODE_NONINTERLACED, },
	/* 1280x768 at 59.99 Hz */
	[22] = { NULL, 59,
		1280, 768, 14652, 80, 48, 12, 3, 32, 7,
		FB_SYNC_HOR_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1280x768 at 59.87 Hz */
	[23] = { NULL, 59,
		1280, 768, 12578, 192, 64, 20, 3, 128, 7,
		FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1280x800 at 67.08 Hz */
	[27] = { NULL, 67,
		1280, 800, 12578, 80, 48, 14, 3, 32, 6,
		FB_SYNC_HOR_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1280x800 at 59.81 Hz */
	[28] = { NULL, 59,
		1280, 800, 11976, 200, 72, 22, 3, 128, 6,
		FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1280x960 at 60.00 Hz */
	[32] = { NULL, 60,
		1280, 960, 9259, 312, 96, 36, 1, 112, 3,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1280x1024 at 60.02 Hz */
	[35] = { NULL, 60,
		1280, 1024, 9259, 248, 48, 38, 1, 112, 3,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1360x768 at 60.02 Hz */
	[39] = { NULL, 60,
		1360, 768, 11695, 256, 64, 18, 3, 112, 6,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1400x1050 at 59.95 Hz */
	[41] = { NULL, 59,
		1400, 1050, 9900, 80, 48, 23, 3, 32, 4,
		FB_SYNC_HOR_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1400x1050 at 59.98 Hz */
	[42] = { NULL, 59,
		1400, 1050, 8213, 232, 88, 32, 3, 144, 4,
		FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1440x900 at 59.89 Hz */
	[47] = { NULL, 59,
		1440, 900, 9389, 232, 80, 25, 3, 152, 6,
		FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1680x1050 at 59.88 Hz */
	[57] = { NULL, 59,
		1680, 1050, 8403, 80, 48, 21, 3, 32, 6,
		FB_SYNC_HOR_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1680x1050 at 59.95 Hz */
	[58] = { NULL, 59,
		1680, 1050, 6837, 280, 104, 30, 3, 176, 6,
		FB_SYNC_VERT_HIGH_ACT, FB_VMODE_NONINTERLACED, },
	/* 1366x768 at 59.79 Hz */
	[81] = { NULL, 59,
		1366, 768, 11695, 213, 70, 24, 3, 143, 3,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1920x1080 at 60.22 Hz */
	[82] = { NULL, 60,
		1920, 1080, 6734, 80, 148, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
	/* 1280x720 at 60.00 Hz */
	[84] = { NULL, 60,
		1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED, },
};

static const u8 edid_header[8] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0};

static int hdmi_runtime_get(void)
{
	int r;

	DSSDBG("hdmi_runtime_get\n");

	if (hdmi.runtime_count++ == 0) {
		r = dss_runtime_get();
		if (r)
			goto err_get_dss;

		r = dispc_runtime_get();
		if (r)
			goto err_get_dispc;

		clk_enable(hdmi.sys_clk);
		clk_enable(hdmi.hdmi_clk);

		r = pm_runtime_get_sync(&hdmi.pdev->dev);
		WARN_ON(r);
		if (r < 0)
			goto err_runtime_get;
	}

	return 0;

err_runtime_get:
	clk_disable(hdmi.sys_clk);
	clk_disable(hdmi.hdmi_clk);
	dispc_runtime_put();
err_get_dispc:
	dss_runtime_put();
err_get_dss:
	return r;
}

static void hdmi_runtime_put(void)
{
	int r;

	DSSDBG("hdmi_runtime_put\n");

	if (--hdmi.runtime_count == 0) {
		r = pm_runtime_put_sync(&hdmi.pdev->dev);
		WARN_ON(r);

		clk_disable(hdmi.sys_clk);
		clk_disable(hdmi.hdmi_clk);

		dispc_runtime_put();
		dss_runtime_put();
	}
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	return 0;
}

static int relaxed_fb_mode_is_equal(const struct fb_videomode *mode1,
				    const struct fb_videomode *mode2)
{
	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->pixclock     <= mode2->pixclock + 1 &&
		mode1->pixclock     >= mode2->pixclock - 1 &&
		mode1->hsync_len + mode1->left_margin + mode1->right_margin ==
		mode2->hsync_len + mode2->left_margin + mode2->right_margin &&
		mode1->vsync_len + mode1->upper_margin + mode1->lower_margin ==
		mode2->vsync_len + mode2->upper_margin + mode2->lower_margin &&
		(mode1->vmode & FB_VMODE_INTERLACED) ==
		(mode2->vmode & FB_VMODE_INTERLACED));
}

static int hdmi_set_timings(const struct fb_videomode *vm, bool check_only)
{
	int i = 0;
	DSSDBG("hdmi_get_code\n");

	if (!vm->xres || !vm->yres || !vm->pixclock)
		goto fail;

	for (i = 0; i < ARRAY_SIZE(cea_timings); i++) {
		if (relaxed_fb_mode_is_equal(cea_timings + i, vm)) {
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_HDMI;
			hdmi.cfg.timings = cea_timings[hdmi.cfg.cm.code];
			goto done;
		}
	}

	for (i = 0; i < ARRAY_SIZE(vesa_timings); i++) {
		if (relaxed_fb_mode_is_equal(vesa_timings + i, vm)) {
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_DVI;
			hdmi.cfg.timings = vesa_timings[hdmi.cfg.cm.code];
			goto done;
		}
	}
#if 0
	for (i = 0; i < sizeof(cea_modes); i++) {
		if (relaxed_fb_mode_is_equal(cea_modes + i, vm)) {
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_HDMI;
			hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}

	for (i = 0; i < 34; i++) {
		if (relaxed_fb_mode_is_equal(vesa_modes + i, vm)) {
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_DVI;
			hdmi.cfg.timings = vesa_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}
#endif
fail:
	if (check_only)
		return 0;
	hdmi.cfg.cm.code = 1;
	hdmi.cfg.cm.mode = HDMI_HDMI;
	hdmi.cfg.timings = cea_timings[hdmi.cfg.cm.code];

	i = -1;
done:

	DSSDBG("%s-%d\n", hdmi.cfg.cm.mode ? "CEA" : "VESA", hdmi.cfg.cm.code);
	return i >= 0;
}

void hdmi_get_monspecs(struct fb_monspecs *specs)
{
	int i, j;
	char *edid = (char *) hdmi.edid;

	memset(specs, 0x0, sizeof(*specs));
	if (!hdmi.edid_set)
		return;

	fb_edid_to_monspecs(edid, specs);
	if (specs->modedb == NULL)
		return;

	for (i = 1; i <= edid[0x7e] && i * 128 < HDMI_EDID_MAX_LENGTH; i++) {
		if (edid[i * 128] == 0x2)
			fb_edid_add_monspecs(edid + i * 128, specs);
	}

	/* filter out resolutions we don't support */
	for (i = j = 0; i < specs->modedb_len; i++) {
		if (hdmi_set_timings(&specs->modedb[i], true))
			specs->modedb[j++] = specs->modedb[i];
	}
	specs->modedb_len = j;
}

u8 *hdmi_read_edid(struct omap_video_timings *dp)
{
	int ret = 0, i;

	if (hdmi.edid_set)
		return hdmi.edid;

	memset(hdmi.edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = read_ti_4xxx_edid(&hdmi.hdmi_data, hdmi.edid,
						HDMI_EDID_MAX_LENGTH);

	for (i = 0; i < 256; i += 16)
		pr_info("edid[%03x] = %02x %02x %02x %02x %02x %02x %02x %02x "
			 "%02x %02x %02x %02x %02x %02x %02x %02x\n", i,
			hdmi.edid[i], hdmi.edid[i + 1], hdmi.edid[i + 2],
			hdmi.edid[i + 3], hdmi.edid[i + 4], hdmi.edid[i + 5],
			hdmi.edid[i + 6], hdmi.edid[i + 7], hdmi.edid[i + 8],
			hdmi.edid[i + 9], hdmi.edid[i + 10], hdmi.edid[i + 11],
			hdmi.edid[i + 12], hdmi.edid[i + 13], hdmi.edid[i + 14],
			hdmi.edid[i + 15]);

	if (ret) {
		DSSWARN("failed to read E-EDID\n");
		return NULL;
	}

	if (memcmp(hdmi.edid, edid_header, sizeof(edid_header)))
		return NULL;

	hdmi.edid_set = true;
	return hdmi.edid;
}

static void hdmi_compute_pll(struct omap_dss_device *dssdev, int phy,
		struct hdmi_pll_info *pi)
{
	unsigned long clkin, refclk;
	u32 mf;

	clkin = clk_get_rate(hdmi.sys_clk) / 10000;
	/*
	 * Input clock is predivided by N + 1
	 * out put of which is reference clk
	 */
	pi->regn = dssdev->clocks.hdmi.regn;
	refclk = clkin / (pi->regn + 1);

	/*
	 * multiplier is pixel_clk/ref_clk
	 * Multiplying by 100 to avoid fractional part removal
	 */
	pi->regm = (phy * 100 / (refclk)) / 100;
	pi->regm2 = dssdev->clocks.hdmi.regm2;

	/*
	 * fractional multiplier is remainder of the difference between
	 * multiplier and actual phy(required pixel clock thus should be
	 * multiplied by 2^18(262144) divided by the reference clock
	 */
	mf = (phy - pi->regm * refclk) * 262144;
	pi->regmf = mf / (refclk);

	/*
	 * Dcofreq should be set to 1 if required pixel clock
	 * is greater than 1000MHz
	 */
	pi->dcofreq = phy > 1000 * 100;
	pi->regsd = ((pi->regm * clkin / 10) / ((pi->regn + 1) * 250) + 5) / 10;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int r;
	struct hdmi_pll_info pll_data;
	struct omap_video_timings *p;
	unsigned long phy;

	r = hdmi_runtime_get();
	if (r)
		return r;

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

	p = &dssdev->panel.timings;

	DSSDBG("hdmi_power_on x_res= %d y_res = %d\n",
		dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);

	if (!hdmi.custom_set)
		hdmi_set_timings(&vesa_timings[4], false);

	omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);

	phy = p->pixel_clock;

	switch (hdmi.deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		phy = (p->pixel_clock * 125) / 100 ;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_30BIT;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR "36 bit deep color not supported");
			goto err;
		}

		phy = (p->pixel_clock * 150) / 100;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_36BIT;
		break;
	case HDMI_DEEP_COLOR_24BIT:
	default:
		phy = p->pixel_clock;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_24BIT;
		break;
	}

	hdmi_compute_pll(dssdev, phy, &pll_data);

	/* config the PLL and PHY hdmi_set_pll_pwrfirst */
	r = hdmi_ti_4xxx_pll_program(&hdmi.hdmi_data, &pll_data);
	if (r) {
		DSSDBG("Failed to lock PLL\n");
		goto err;
	}

	r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data);
	if (r) {
		DSSDBG("Failed to start PHY\n");
		goto err;
	}

	hdmi.cfg.cm.mode = hdmi.mode;
	hdmi.cfg.cm.code = hdmi.code;
	hdmi_ti_4xxx_basic_configure(&hdmi.hdmi_data, &hdmi.cfg);

	/* Make selection of HDMI in DSS */
	dss_select_hdmi_venc_clk_source(DSS_HDMI_M_PCLK);

	/* Select the dispc clock source as PRCM clock, to ensure that it is not
	 * DSI PLL source as the clock selected by DSI PLL might not be
	 * sufficient for the resolution selected / that can be changed
	 * dynamically by user. This can be moved to single location , say
	 * Boardfile.
	 */
	dss_select_dispc_clk_source(dssdev->clocks.dispc.dispc_fclk_src);

	/* bypass TV gamma table */
	dispc_enable_gamma_table(0);

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 1);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 1);

	return 0;
err:
	hdmi_runtime_put();
	return -EIO;
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);
	hdmi_ti_4xxx_phy_off(&hdmi.hdmi_data);
	hdmi_ti_4xxx_set_pll_pwr(&hdmi.hdmi_data, HDMI_PLLPWRCMD_ALLOFF);
	hdmi_runtime_put();
	hdmi.deep_color = HDMI_DEEP_COLOR_24BIT;
}

int omapdss_hdmi_get_pixel_clock(void)
{
	return PICOS2KHZ(hdmi.cfg.timings.pixclock);
}

int omapdss_hdmi_get_mode(void)
{
	return hdmi.mode;
}

void omapdss_hdmi_set_deepcolor(int val)
{
	hdmi.deep_color = val;
}

int omapdss_hdmi_get_deepcolor(void)
{
	return hdmi.deep_color;
}

int hdmi_get_current_hpd()
{
	return gpio_get_value(GPIO_HDMI_HPD);
}

static irqreturn_t hpd_irq_handler(int irq, void *ptr)
{
	int hpd = hdmi_get_current_hpd();
	pr_info("hpd %d\n", hpd);

	hdmi_panel_hpd_handler(hpd);

	return IRQ_HANDLED;
}

int omapdss_hdmi_display_check_timing(struct omap_dss_device *dssdev,
					struct omap_video_timings *timings)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(timings, &t);

	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}
	if (!hdmi_set_timings(&t, true))
		return -EINVAL;
	return 0;
}

int omapdss_hdmi_display_set_mode(struct omap_dss_device *dssdev,
				  struct fb_videomode *vm)
{
	int r1, r2;
	/* turn the hdmi off and on to get new timings to use */
	omapdss_hdmi_display_disable(dssdev);
	r1 = hdmi_set_timings(vm, false) ? 0 : -EINVAL;
	hdmi.custom_set = 1;
	hdmi.code = hdmi.cfg.cm.code;
	hdmi.mode = hdmi.cfg.cm.mode;
	r2 = omapdss_hdmi_display_enable(dssdev);
	return r1 ? : r2;
}

void omapdss_hdmi_display_set_timing(struct omap_dss_device *dssdev)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(&dssdev->panel.timings, &t);
	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}

	omapdss_hdmi_display_set_mode(dssdev, &t);
}

int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSDBG("ENTER hdmi_display_enable\n");

	mutex_lock(&hdmi.lock);

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			DSSERR("failed to enable GPIO's\n");
			goto err1;
		}
	}

	hdmi.hdmi_reg = regulator_get(NULL, "hdmi_vref");
	if (IS_ERR_OR_NULL(hdmi.hdmi_reg)) {
		DSSERR("Failed to get hdmi_vref regulator\n");
		r = PTR_ERR(hdmi.hdmi_reg) ? : -ENODEV;
		goto err2;
	}

	r = regulator_enable(hdmi.hdmi_reg);
	if (r) {
		DSSERR("failed to enable hdmi_vref regulator\n");
		goto err3;
	}

	r = hdmi_power_on(dssdev);
	if (r) {
		DSSERR("failed to power on device\n");
		goto err4;
	}

	mutex_unlock(&hdmi.lock);
	return 0;

err4:
	regulator_disable(hdmi.hdmi_reg);
err3:
	regulator_put(hdmi.hdmi_reg);
err2:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
err1:
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&hdmi.lock);
	return r;
}

void omapdss_hdmi_display_disable(struct omap_dss_device *dssdev)
{
	DSSDBG("Enter hdmi_display_disable\n");

	mutex_lock(&hdmi.lock);

	hdmi_power_off(dssdev);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		/* clear EDID and mode on disable only */
		hdmi.edid_set = false;
		hdmi.custom_set = 0;
		pr_info("hdmi: clearing EDID info\n");
	}

	regulator_disable(hdmi.hdmi_reg);

	regulator_put(hdmi.hdmi_reg);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omap_dss_stop_device(dssdev);

	mutex_unlock(&hdmi.lock);
}

static int hdmi_get_clocks(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "sys_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.sys_clk = clk;

	clk = clk_get(&pdev->dev, "hdmi_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get hdmi_clk\n");
		clk_put(hdmi.sys_clk);
		return PTR_ERR(clk);
	}

	hdmi.hdmi_clk = clk;

	return 0;
}

static void hdmi_put_clocks(void)
{
	if (hdmi.sys_clk)
		clk_put(hdmi.sys_clk);
	if (hdmi.hdmi_clk)
		clk_put(hdmi.hdmi_clk);
}

/* HDMI HW IP initialisation */
static int omapdss_hdmihw_probe(struct platform_device *pdev)
{
	struct resource *hdmi_mem;
	int r;

	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;

	mutex_init(&hdmi.lock);

	hdmi_mem = platform_get_resource(hdmi.pdev, IORESOURCE_MEM, 0);
	if (!hdmi_mem) {
		DSSERR("can't get IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	/* Base address taken from platform */
	hdmi.hdmi_data.base_wp = ioremap(hdmi_mem->start,
						resource_size(hdmi_mem));
	if (!hdmi.hdmi_data.base_wp) {
		DSSERR("can't ioremap WP\n");
		return -ENOMEM;
	}

	r = hdmi_get_clocks(pdev);
	if (r) {
		iounmap(hdmi.hdmi_data.base_wp);
		return r;
	}

	pm_runtime_enable(&pdev->dev);

	r = request_irq(gpio_to_irq(GPIO_HDMI_HPD), hpd_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"hpd", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %d failed\n",
			gpio_to_irq(GPIO_HDMI_HPD));
		return -EINVAL;
	}

	hdmi.hdmi_data.hdmi_core_sys_offset = HDMI_CORE_SYS;
	hdmi.hdmi_data.hdmi_core_av_offset = HDMI_CORE_AV;
	hdmi.hdmi_data.hdmi_pll_offset = HDMI_PLLCTRL;
	hdmi.hdmi_data.hdmi_phy_offset = HDMI_PHY;

	hdmi_panel_init();

	return 0;
}

static int omapdss_hdmihw_remove(struct platform_device *pdev)
{
	hdmi_panel_exit();

	pm_runtime_disable(&pdev->dev);

	hdmi_put_clocks();

	iounmap(hdmi.hdmi_data.base_wp);

	return 0;
}

static struct platform_driver omapdss_hdmihw_driver = {
	.probe          = omapdss_hdmihw_probe,
	.remove         = omapdss_hdmihw_remove,
	.driver         = {
		.name   = "omapdss_hdmi",
		.owner  = THIS_MODULE,
	},
};

int hdmi_init_platform_driver(void)
{
	return platform_driver_register(&omapdss_hdmihw_driver);
}

void hdmi_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omapdss_hdmihw_driver);
}

void hdmi_dump_regs(struct seq_file *s)
{
	if (hdmi_runtime_get())
		return;

	hdmi_ti_4xxx_dump_regs(&hdmi.hdmi_data, s);

	hdmi_runtime_put();
}
