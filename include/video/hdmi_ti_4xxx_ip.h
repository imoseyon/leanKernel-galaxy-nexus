/*
 * hdmi_ti_4xxx_ip.h
 *
 * HDMI driver definition for TI OMAP4 processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _HDMI_TI_4xxx_
#define _HDMI_TI_4xxx_

enum hdmi_pll_pwr {
	HDMI_PLLPWRCMD_ALLOFF = 0,
	HDMI_PLLPWRCMD_PLLONLY = 1,
	HDMI_PLLPWRCMD_BOTHON_ALLCLKS = 2,
	HDMI_PLLPWRCMD_BOTHON_NOPHYCLK = 3
};

enum hdmi_core_hdmi_dvi {
	HDMI_DVI = 0,
	HDMI_HDMI = 1
};

struct hdmi_ip_data {
	void __iomem *base_wp;	/* HDMI wrapper */
	unsigned long	hdmi_core_sys_offset;
	unsigned long hdmi_core_av_offset;
	unsigned long hdmi_pll_offset;
	unsigned long hdmi_phy_offset;
};

struct hdmi_video_timings {
	u16 x_res;
	u16 y_res;
	/* Unit: KHz */
	u32 pixel_clock;
	u16 hsw;
	u16 hfp;
	u16 hbp;
	u16 vsw;
	u16 vfp;
	u16 vbp;
};

/* HDMI timing structure */
struct hdmi_timings {
	struct hdmi_video_timings timings;
	int vsync_pol;
	int hsync_pol;
};

struct hdmi_cm {
	int	code;
	int	mode;
};

struct hdmi_config {
	struct hdmi_timings timings;
	u16	interlace;
	struct hdmi_cm cm;
};

/* HDMI PLL structure */
struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};

int hdmi_ti_4xxx_phy_init(struct hdmi_ip_data *ip_data);
void hdmi_ti_4xxx_phy_off(struct hdmi_ip_data *ip_data);
int read_ti_4xxx_edid(struct hdmi_ip_data *ip_data, u8 *pedid, u16 max_length);
void hdmi_ti_4xxx_wp_video_start(struct hdmi_ip_data *ip_data, bool start);
int hdmi_ti_4xxx_pll_program(struct hdmi_ip_data *ip_data,
			struct hdmi_pll_info *fmt);
int hdmi_ti_4xxx_set_pll_pwr(struct hdmi_ip_data *ip_data, enum hdmi_pll_pwr val);
void hdmi_ti_4xxx_basic_configure(struct hdmi_ip_data *ip_data,
			struct hdmi_config *cfg);
#endif
