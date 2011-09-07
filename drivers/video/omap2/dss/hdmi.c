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
#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)
#include <sound/soc.h>
#include <sound/pcm_params.h>
#endif

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

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)
static void hdmi_wp_audio_config_format(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_format *aud_fmt)
{
	u32 r;

	DSSDBG("Enter hdmi_wp_audio_config_format\n");

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG);
	r = FLD_MOD(r, aud_fmt->stereo_channels, 26, 24);
	r = FLD_MOD(r, aud_fmt->active_chnnls_msk, 23, 16);
	r = FLD_MOD(r, aud_fmt->en_sig_blk_strt_end, 5, 5);
	r = FLD_MOD(r, aud_fmt->type, 4, 4);
	r = FLD_MOD(r, aud_fmt->justification, 3, 3);
	r = FLD_MOD(r, aud_fmt->sample_order, 2, 2);
	r = FLD_MOD(r, aud_fmt->samples_per_word, 1, 1);
	r = FLD_MOD(r, aud_fmt->sample_size, 0, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG, r);
}

static void hdmi_wp_audio_config_dma(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_dma *aud_dma)
{
	u32 r;

	DSSDBG("Enter hdmi_wp_audio_config_dma\n");

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG2);
	r = FLD_MOD(r, aud_dma->transfer_size, 15, 8);
	r = FLD_MOD(r, aud_dma->block_size, 7, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG2, r);

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CTRL);
	r = FLD_MOD(r, aud_dma->mode, 9, 9);
	r = FLD_MOD(r, aud_dma->fifo_threshold, 8, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CTRL, r);
}

static void hdmi_core_audio_config(struct hdmi_ip_data *ip_data,
					struct hdmi_core_audio_config *cfg)
{
	u32 r;

	/* audio clock recovery parameters */
	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_ACR_CTRL);
	r = FLD_MOD(r, cfg->use_mclk, 2, 2);
	r = FLD_MOD(r, cfg->en_acr_pkt, 1, 1);
	r = FLD_MOD(r, cfg->cts_mode, 0, 0);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_ACR_CTRL, r);

	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_N_SVAL1, cfg->n, 7, 0);
	REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_N_SVAL2, cfg->n >> 8, 7, 0);
	REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_N_SVAL3, cfg->n >> 16, 7, 0);

	if (cfg->cts_mode == HDMI_AUDIO_CTS_MODE_SW) {
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL1, cfg->cts, 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL2, cfg->cts >> 8, 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL3, cfg->cts >> 16, 7, 0);
	} else {
		/*
		 * HDMI IP uses this configuration to divide the MCLK to
		 * update CTS value.
		 */
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_FREQ_SVAL, cfg->mclk_mode, 2, 0);

		/* Configure clock for audio packets */
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_AUD_PAR_BUSCLK_1,
				cfg->aud_par_busclk, 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_AUD_PAR_BUSCLK_2,
				(cfg->aud_par_busclk >> 8), 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_AUD_PAR_BUSCLK_3,
				(cfg->aud_par_busclk >> 16), 7, 0);
	}

	/* Override of SPDIF sample frequency with value in I2S_CHST4 */
	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_SPDIF_CTRL,
						cfg->fs_override, 1, 1);

	/* I2S parameters */
	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_CHST4,
						cfg->freq_sample, 3, 0);

	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_IN_CTRL);
	r = FLD_MOD(r, cfg->i2s_cfg.en_high_bitrate_aud, 7, 7);
	r = FLD_MOD(r, cfg->i2s_cfg.sck_edge_mode, 6, 6);
	r = FLD_MOD(r, cfg->i2s_cfg.cbit_order, 5, 5);
	r = FLD_MOD(r, cfg->i2s_cfg.vbit, 4, 4);
	r = FLD_MOD(r, cfg->i2s_cfg.ws_polarity, 3, 3);
	r = FLD_MOD(r, cfg->i2s_cfg.justification, 2, 2);
	r = FLD_MOD(r, cfg->i2s_cfg.direction, 1, 1);
	r = FLD_MOD(r, cfg->i2s_cfg.shift, 0, 0);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_IN_CTRL, r);

	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_CHST5);
	r = FLD_MOD(r, cfg->freq_sample, 7, 4);
	r = FLD_MOD(r, cfg->i2s_cfg.word_length, 3, 1);
	r = FLD_MOD(r, cfg->i2s_cfg.word_max_length, 0, 0);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_CHST5, r);

	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_I2S_IN_LEN,
					cfg->i2s_cfg.in_length_bits, 3, 0);

	/* Audio channels and mode parameters */
	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_HDMI_CTRL,
							cfg->layout, 2, 1);
	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_MODE);
	r = FLD_MOD(r, cfg->i2s_cfg.active_sds, 7, 4);
	r = FLD_MOD(r, cfg->en_dsd_audio, 3, 3);
	r = FLD_MOD(r, cfg->en_parallel_aud_input, 2, 2);
	r = FLD_MOD(r, cfg->en_spdif, 1, 1);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_MODE, r);
}

static void hdmi_core_audio_infoframe_config(struct hdmi_ip_data *ip_data,
		struct hdmi_core_infoframe_audio *info_aud)
{
	u8 val;
	u8 sum = 0, checksum = 0;

	/*
	 * Set audio info frame type, version and length as
	 * described in HDMI 1.4a Section 8.2.2 specification.
	 * Checksum calculation is defined in Section 5.3.5.
	 */
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUDIO_TYPE, 0x84);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUDIO_VERS, 0x01);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUDIO_LEN, 0x0a);
	sum += 0x84 + 0x001 + 0x00a;

	val = (info_aud->db1_coding_type << 4)
			| (info_aud->db1_channel_count - 1);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(0), val);
	sum += val;

	val = (info_aud->db2_sample_freq << 2) | info_aud->db2_sample_size;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(1), val);
	sum += val;

	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(2), 0x00);

	val = info_aud->db4_channel_alloc;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(3), val);
	sum += val;

	val = (info_aud->db5_downmix_inh << 7) | (info_aud->db5_lsv << 3);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(4), val);
	sum += val;

	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(5), 0x00);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(6), 0x00);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(7), 0x00);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(8), 0x00);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AUD_DBYTE(9), 0x00);

	checksum = 0x100 - sum;
	hdmi_write_reg(hdmi_av_base(ip_data),
					HDMI_CORE_AV_AUDIO_CHSUM, checksum);

	/*
	 * TODO: Add MPEG and SPD enable and repeat cfg when EDID parsing
	 * is available.
	 */
}

static int hdmi_config_audio_acr(struct hdmi_ip_data *ip_data,
				u32 sample_freq, u32 *n, u32 *cts)
{
	u32 r;
	u32 deep_color = 0;
	u32 pclk = hdmi.cfg.timings.timings.pixel_clock;

	if (n == NULL || cts == NULL)
		return -EINVAL;
	/*
	 * Obtain current deep color configuration. This needed
	 * to calculate the TMDS clock based on the pixel clock.
	 */
	r = REG_GET(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG, 1, 0);
	switch (r) {
	case 1: /* No deep color selected */
		deep_color = 100;
		break;
	case 2: /* 10-bit deep color selected */
		deep_color = 125;
		break;
	case 3: /* 12-bit deep color selected */
		deep_color = 150;
		break;
	default:
		return -EINVAL;
	}

	switch (sample_freq) {
	case 32000:
		if ((deep_color == 125) && ((pclk == 54054)
				|| (pclk == 74250)))
			*n = 8192;
		else
			*n = 4096;
		break;
	case 44100:
		*n = 6272;
		break;
	case 48000:
		if ((deep_color == 125) && ((pclk == 54054)
				|| (pclk == 74250)))
			*n = 8192;
		else
			*n = 6144;
		break;
	default:
		*n = 0;
		return -EINVAL;
	}

	/* Calculate CTS. See HDMI 1.3a or 1.4a specifications */
	*cts = pclk * (*n / 128) * deep_color / (sample_freq / 10);

	return 0;
}

static int hdmi_audio_hw_params(struct hdmi_ip_data *ip_data,
					struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct hdmi_audio_format audio_format;
	struct hdmi_audio_dma audio_dma;
	struct hdmi_core_audio_config core_cfg;
	struct hdmi_core_infoframe_audio aud_if_cfg;
	int err, n, cts;
	enum hdmi_core_audio_sample_freq sample_freq;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		core_cfg.i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_20BITS;
		core_cfg.i2s_cfg.word_length = HDMI_AUDIO_I2S_CHST_WORD_16_BITS;
		core_cfg.i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_16;
		core_cfg.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_dma.transfer_size = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		core_cfg.i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_24BITS;
		core_cfg.i2s_cfg.word_length = HDMI_AUDIO_I2S_CHST_WORD_24_BITS;
		core_cfg.i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_24;
		audio_format.samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format.sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		core_cfg.i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		audio_dma.transfer_size = 0x20;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 32000:
		sample_freq = HDMI_AUDIO_FS_32000;
		break;
	case 44100:
		sample_freq = HDMI_AUDIO_FS_44100;
		break;
	case 48000:
		sample_freq = HDMI_AUDIO_FS_48000;
		break;
	default:
		return -EINVAL;
	}

	err = hdmi_config_audio_acr(ip_data, params_rate(params), &n, &cts);
	if (err < 0)
		return err;

	/* Audio wrapper config */
	audio_format.stereo_channels = HDMI_AUDIO_STEREO_ONECHANNEL;
	audio_format.active_chnnls_msk = 0x03;
	audio_format.type = HDMI_AUDIO_TYPE_LPCM;
	audio_format.sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;
	/* Disable start/stop signals of IEC 60958 blocks */
	audio_format.en_sig_blk_strt_end = HDMI_AUDIO_BLOCK_SIG_STARTEND_OFF;

	audio_dma.block_size = 0xC0;
	audio_dma.mode = HDMI_AUDIO_TRANSF_DMA;
	audio_dma.fifo_threshold = 0x20; /* in number of samples */

	hdmi_wp_audio_config_dma(ip_data, &audio_dma);
	hdmi_wp_audio_config_format(ip_data, &audio_format);

	/*
	 * I2S config
	 */
	core_cfg.i2s_cfg.en_high_bitrate_aud = false;
	/* Only used with high bitrate audio */
	core_cfg.i2s_cfg.cbit_order = false;
	/* Serial data and word select should change on sck rising edge */
	core_cfg.i2s_cfg.sck_edge_mode = HDMI_AUDIO_I2S_SCK_EDGE_RISING;
	core_cfg.i2s_cfg.vbit = HDMI_AUDIO_I2S_VBIT_FOR_PCM;
	/* Set I2S word select polarity */
	core_cfg.i2s_cfg.ws_polarity = HDMI_AUDIO_I2S_WS_POLARITY_LOW_IS_LEFT;
	core_cfg.i2s_cfg.direction = HDMI_AUDIO_I2S_MSB_SHIFTED_FIRST;
	/* Set serial data to word select shift. See Phillips spec. */
	core_cfg.i2s_cfg.shift = HDMI_AUDIO_I2S_FIRST_BIT_SHIFT;
	/* Enable one of the four available serial data channels */
	core_cfg.i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN;

	/* Core audio config */
	core_cfg.freq_sample = sample_freq;
	core_cfg.n = n;
	core_cfg.cts = cts;
	if (dss_has_feature(FEAT_HDMI_CTS_SWMODE)) {
		core_cfg.aud_par_busclk = 0;
		core_cfg.cts_mode = HDMI_AUDIO_CTS_MODE_SW;
		core_cfg.use_mclk = false;
	} else {
		core_cfg.aud_par_busclk = (((128 * 31) - 1) << 8);
		core_cfg.cts_mode = HDMI_AUDIO_CTS_MODE_HW;
		core_cfg.use_mclk = true;
		core_cfg.mclk_mode = HDMI_AUDIO_MCLK_128FS;
	}
	core_cfg.layout = HDMI_AUDIO_LAYOUT_2CH;
	core_cfg.en_spdif = false;
	/* Use sample frequency from channel status word */
	core_cfg.fs_override = true;
	/* Enable ACR packets */
	core_cfg.en_acr_pkt = true;
	/* Disable direct streaming digital audio */
	core_cfg.en_dsd_audio = false;
	/* Use parallel audio interface */
	core_cfg.en_parallel_aud_input = true;

	hdmi_core_audio_config(ip_data, &core_cfg);

	/*
	 * Configure packet
	 * info frame audio see doc CEA861-D page 74
	 */
	aud_if_cfg.db1_coding_type = HDMI_INFOFRAME_AUDIO_DB1CT_FROM_STREAM;
	aud_if_cfg.db1_channel_count = 2;
	aud_if_cfg.db2_sample_freq = HDMI_INFOFRAME_AUDIO_DB2SF_FROM_STREAM;
	aud_if_cfg.db2_sample_size = HDMI_INFOFRAME_AUDIO_DB2SS_FROM_STREAM;
	aud_if_cfg.db4_channel_alloc = 0x00;
	aud_if_cfg.db5_downmix_inh = false;
	aud_if_cfg.db5_lsv = 0;

	hdmi_core_audio_infoframe_config(ip_data, &aud_if_cfg);
	return 0;
}

static int hdmi_audio_trigger(struct hdmi_ip_data *ip_data,
				struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	int err = 0;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		REG_FLD_MOD(hdmi_av_base(ip_data),
					HDMI_CORE_AV_AUD_MODE, 1, 0, 0);
		REG_FLD_MOD(hdmi_wp_base(ip_data),
					HDMI_WP_AUDIO_CTRL, 1, 31, 31);
		REG_FLD_MOD(hdmi_wp_base(ip_data),
					HDMI_WP_AUDIO_CTRL, 1, 30, 30);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		REG_FLD_MOD(hdmi_av_base(ip_data),
					HDMI_CORE_AV_AUD_MODE, 0, 0, 0);
		REG_FLD_MOD(hdmi_wp_base(ip_data),
					HDMI_WP_AUDIO_CTRL, 0, 30, 30);
		REG_FLD_MOD(hdmi_wp_base(ip_data),
					HDMI_WP_AUDIO_CTRL, 0, 31, 31);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static int hdmi_audio_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	if (!hdmi.mode) {
		pr_err("Current video settings do not support audio.\n");
		return -EIO;
	}
	return 0;
}

static struct snd_soc_codec_driver hdmi_audio_codec_drv = {
};

static struct snd_soc_dai_ops hdmi_audio_codec_ops = {
	.hw_params = hdmi_audio_hw_params,
	.trigger = hdmi_audio_trigger,
	.startup = hdmi_audio_startup,
};

static struct snd_soc_dai_driver hdmi_codec_dai_drv = {
		.name = "hdmi-audio-codec",
		.playback = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &hdmi_audio_codec_ops,
};
#endif

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

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)

	/* Register ASoC codec DAI */
	r = snd_soc_register_codec(&pdev->dev, &hdmi_audio_codec_drv,
					&hdmi_codec_dai_drv, 1);
	if (r) {
		DSSERR("can't register ASoC HDMI audio codec\n");
		return r;
	}
#endif
	return 0;
}

static int omapdss_hdmihw_remove(struct platform_device *pdev)
{
	hdmi_panel_exit();

#if defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI) || \
	defined(CONFIG_SND_OMAP_SOC_OMAP4_HDMI_MODULE)
	snd_soc_unregister_codec(&pdev->dev);
#endif

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
