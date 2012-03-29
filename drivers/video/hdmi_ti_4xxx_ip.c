/*
 * hdmi_ti_4xxx_ip.c
 *
 * HDMI TI81xx, TI38xx, TI OMAP4 etc IP driver Library
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/omapfb.h>

#include "hdmi_ti_4xxx_ip.h"

static inline void hdmi_write_reg(void __iomem *base_addr,
				const struct hdmi_reg idx, u32 val)
{
	__raw_writel(val, base_addr + idx.idx);
}

static inline u32 hdmi_read_reg(void __iomem *base_addr,
				const struct hdmi_reg idx)
{
	return __raw_readl(base_addr + idx.idx);
}

static inline void __iomem *hdmi_wp_base(struct hdmi_ip_data *ip_data)
{
	return (void __iomem *) (ip_data->base_wp);
}

static inline void __iomem *hdmi_phy_base(struct hdmi_ip_data *ip_data)
{
	return (void __iomem *) (ip_data->base_wp + ip_data->hdmi_phy_offset);
}

static inline void __iomem *hdmi_pll_base(struct hdmi_ip_data *ip_data)
{
	return (void __iomem *)	(ip_data->base_wp + ip_data->hdmi_pll_offset);
}

static inline void __iomem *hdmi_av_base(struct hdmi_ip_data *ip_data)
{
	return (void __iomem *)
			(ip_data->base_wp + ip_data->hdmi_core_av_offset);
}

static inline void __iomem *hdmi_core_sys_base(struct hdmi_ip_data *ip_data)
{
	return (void __iomem *)
			(ip_data->base_wp + ip_data->hdmi_core_sys_offset);
}

static inline int hdmi_wait_for_bit_change(void __iomem *base_addr,
				const struct hdmi_reg idx,
				int b2, int b1, u32 val)
{
	u32 t = 0;
	while (val != REG_GET(base_addr, idx, b2, b1)) {
		udelay(1);
		if (t++ > 10000)
			return !val;
	}
	return val;
}

static int hdmi_pll_init(struct hdmi_ip_data *ip_data,
		enum hdmi_clk_refsel refsel, int dcofreq,
		struct hdmi_pll_info *fmt, u16 sd)
{
	u32 r;

	/* PLL start always use manual mode */
	REG_FLD_MOD(hdmi_pll_base(ip_data), PLLCTRL_PLL_CONTROL, 0x0, 0, 0);

	r = hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG1);
	r = FLD_MOD(r, fmt->regm, 20, 9); /* CFG1_PLL_REGM */
	r = FLD_MOD(r, fmt->regn, 8, 1);  /* CFG1_PLL_REGN */

	hdmi_write_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG1, r);

	r = hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG2);

	r = FLD_MOD(r, 0x0, 12, 12); /* PLL_HIGHFREQ divide by 2 */
	r = FLD_MOD(r, 0x1, 13, 13); /* PLL_REFEN */
	r = FLD_MOD(r, 0x0, 14, 14); /* PHY_CLKINEN de-assert during locking */

	if (dcofreq) {
		/* divider programming for frequency beyond 1000Mhz */
		REG_FLD_MOD(hdmi_pll_base(ip_data), PLLCTRL_CFG3, sd, 17, 10);
		r = FLD_MOD(r, 0x4, 3, 1); /* 1000MHz and 2000MHz */
	} else {
		r = FLD_MOD(r, 0x2, 3, 1); /* 500MHz and 1000MHz */
	}

	hdmi_write_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG2, r);

	r = hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG4);
	r = FLD_MOD(r, fmt->regm2, 24, 18);
	r = FLD_MOD(r, fmt->regmf, 17, 0);

	hdmi_write_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG4, r);

	/* go now */
	REG_FLD_MOD(hdmi_pll_base(ip_data), PLLCTRL_PLL_GO, 0x1, 0, 0);

	/* wait for bit change */
	if (hdmi_wait_for_bit_change(hdmi_pll_base(ip_data), PLLCTRL_PLL_GO,
							0, 0, 1) != 1) {
		pr_err("PLL GO bit not set\n");
		return -ETIMEDOUT;
	}

	/* Wait till the lock bit is set in PLL status */
	if (hdmi_wait_for_bit_change(hdmi_pll_base(ip_data),
				PLLCTRL_PLL_STATUS, 1, 1, 1) != 1) {
		pr_err("cannot lock PLL\n");
		pr_err("CFG1 0x%x\n",
			hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG1));
		pr_err("CFG2 0x%x\n",
			hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG2));
		pr_err("CFG4 0x%x\n",
			hdmi_read_reg(hdmi_pll_base(ip_data), PLLCTRL_CFG4));
		return -ETIMEDOUT;
	}

	pr_debug("PLL locked!\n");

	return 0;
}
static int hdmi_wait_for_audio_stop(struct hdmi_ip_data *ip_data)
{
	int count = 0;
	/* wait for audio to stop before powering off the phy*/
	while (REG_GET(hdmi_wp_base(ip_data),
		       HDMI_WP_AUDIO_CTRL, 31, 31) != 0) {
		msleep(100);
		if (count++ > 100) {
			pr_err("Audio is not turned off "
			       "even after 10 seconds\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/* PHY_PWR_CMD */
static int hdmi_set_phy_pwr(struct hdmi_ip_data *ip_data,
				enum hdmi_phy_pwr val, bool set_mode)
{
	/* FIXME audio driver should have already stopped, but not yet */
	if (val == HDMI_PHYPWRCMD_OFF && !set_mode)
		hdmi_wait_for_audio_stop(ip_data);

	/* Command for power control of HDMI PHY */
	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_PWR_CTRL, val, 7, 6);

	/* Status of the power control of HDMI PHY */
	if (hdmi_wait_for_bit_change(hdmi_wp_base(ip_data),
				HDMI_WP_PWR_CTRL, 5, 4, val) != val) {
		pr_err("Failed to set PHY power mode to %d\n", val);
		return -ETIMEDOUT;
	}

	return 0;
}

/* PLL_PWR_CMD */
int hdmi_ti_4xxx_set_pll_pwr(struct hdmi_ip_data *ip_data, enum hdmi_pll_pwr val)
{
	/* Command for power control of HDMI PLL */
	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_PWR_CTRL, val, 3, 2);

	/* wait till PHY_PWR_STATUS is set */
if (hdmi_wait_for_bit_change(hdmi_wp_base(ip_data), HDMI_WP_PWR_CTRL,
						1, 0, val) != val) {
		pr_err("Failed to set PLL_PWR_STATUS\n");
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL(hdmi_ti_4xxx_set_pll_pwr);

static int hdmi_pll_reset(struct hdmi_ip_data *ip_data)
{
	/* SYSRESET  controlled by power FSM */
	REG_FLD_MOD(hdmi_pll_base(ip_data), PLLCTRL_PLL_CONTROL, 0x0, 3, 3);

	/* READ 0x0 reset is in progress */
	if (hdmi_wait_for_bit_change(hdmi_pll_base(ip_data),
				PLLCTRL_PLL_STATUS, 0, 0, 1) != 1) {
		pr_err("Failed to sysreset PLL\n");
		return -ETIMEDOUT;
	}

	return 0;
}

int hdmi_ti_4xxx_pll_program(struct hdmi_ip_data *ip_data,
				struct hdmi_pll_info *fmt)
{
	u16 r = 0;
	enum hdmi_clk_refsel refsel;

	r = hdmi_ti_4xxx_set_pll_pwr(ip_data, HDMI_PLLPWRCMD_ALLOFF);
	if (r)
		return r;

	r = hdmi_ti_4xxx_set_pll_pwr(ip_data, HDMI_PLLPWRCMD_BOTHON_ALLCLKS);
	if (r)
		return r;

	r = hdmi_pll_reset(ip_data);
	if (r)
		return r;

	refsel = HDMI_REFSEL_SYSCLK;

	r = hdmi_pll_init(ip_data, refsel, fmt->dcofreq, fmt, fmt->regsd);
	if (r)
		return r;

	return 0;
}

int hdmi_ti_4xxx_phy_init(struct hdmi_ip_data *ip_data)
{
	u16 r = 0;

	r = hdmi_set_phy_pwr(ip_data, HDMI_PHYPWRCMD_LDOON, false);
	if (r)
		return r;

	r = hdmi_set_phy_pwr(ip_data, HDMI_PHYPWRCMD_TXON, false);
	if (r)
		return r;

	/*
	 * Read address 0 in order to get the SCP reset done completed
	 * Dummy access performed to make sure reset is done
	 */
	hdmi_read_reg(hdmi_phy_base(ip_data), HDMI_TXPHY_TX_CTRL);

	/*
	 * Write to phy address 0 to configure the clock
	 * use HFBITCLK write HDMI_TXPHY_TX_CONTROL_FREQOUT field
	 */
	REG_FLD_MOD(hdmi_phy_base(ip_data), HDMI_TXPHY_TX_CTRL, 0x1, 31, 30);

	/* Write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(hdmi_phy_base(ip_data),
					HDMI_TXPHY_DIGITAL_CTRL, 0xF0000000);

	/* Write to phy address 3 to change the polarity control */
	REG_FLD_MOD(hdmi_phy_base(ip_data),
					HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 27, 27);

	return 0;
}

void hdmi_ti_4xxx_phy_off(struct hdmi_ip_data *ip_data, bool set_mode)
{
	hdmi_set_phy_pwr(ip_data, HDMI_PHYPWRCMD_OFF, set_mode);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_phy_init);
EXPORT_SYMBOL(hdmi_ti_4xxx_phy_off);

static int hdmi_core_ddc_edid(struct hdmi_ip_data *ip_data,
						u8 *pedid, int ext)
{
	u32 i, j;
	char checksum = 0;
	u32 offset = 0;

	/* Turn on CLK for DDC */
	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_DPD, 0x7, 2, 0);

	/*
	 * SW HACK : Without the Delay DDC(i2c bus) reads 0 values /
	 * right shifted values( The behavior is not consistent and seen only
	 * with some TV's)
	 */
	msleep(300);

	if (!ext) {
		/* Clk SCL Devices */
		REG_FLD_MOD(hdmi_core_sys_base(ip_data),
						HDMI_CORE_DDC_CMD, 0xA, 3, 0);

		/* HDMI_CORE_DDC_STATUS_IN_PROG */
		if (hdmi_wait_for_bit_change(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_STATUS, 4, 4, 0) != 0) {
			pr_err("Failed to program DDC\n");
			return -ETIMEDOUT;
		}

		/* Clear FIFO */
		REG_FLD_MOD(hdmi_core_sys_base(ip_data)
						, HDMI_CORE_DDC_CMD, 0x9, 3, 0);

		/* HDMI_CORE_DDC_STATUS_IN_PROG */
		if (hdmi_wait_for_bit_change(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_STATUS, 4, 4, 0) != 0) {
			pr_err("Failed to program DDC\n");
			return -ETIMEDOUT;
		}

	} else {
		if (ext % 2 != 0)
			offset = 0x80;
	}

	/* Load Segment Address Register */
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_SEGM, ext/2, 7, 0);

	/* Load Slave Address Register */
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_ADDR, 0xA0 >> 1, 7, 1);

	/* Load Offset Address Register */
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_OFFSET, offset, 7, 0);

	/* Load Byte Count */
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_COUNT1, 0x80, 7, 0);
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_COUNT2, 0x0, 1, 0);

	/* Set DDC_CMD */
	if (ext)
		REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_CMD, 0x4, 3, 0);
	else
		REG_FLD_MOD(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_CMD, 0x2, 3, 0);

	/* HDMI_CORE_DDC_STATUS_BUS_LOW */
	if (REG_GET(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_STATUS, 6, 6) == 1) {
		pr_err("I2C Bus Low?\n");
		return -EIO;
	}
	/* HDMI_CORE_DDC_STATUS_NO_ACK */
	if (REG_GET(hdmi_core_sys_base(ip_data),
					HDMI_CORE_DDC_STATUS, 5, 5) == 1) {
		pr_err("I2C No Ack\n");
		return -EIO;
	}

	i = ext * 128;
	j = 0;
	while (((REG_GET(hdmi_core_sys_base(ip_data),
			HDMI_CORE_DDC_STATUS, 4, 4) == 1) ||
			(REG_GET(hdmi_core_sys_base(ip_data),
			HDMI_CORE_DDC_STATUS, 2, 2) == 0)) && j < 128) {

		if (REG_GET(hdmi_core_sys_base(ip_data)
					, HDMI_CORE_DDC_STATUS, 2, 2) == 0) {
			/* FIFO not empty */
			pedid[i++] = REG_GET(hdmi_core_sys_base(ip_data),
						HDMI_CORE_DDC_DATA, 7, 0);
			j++;
		}
	}

	for (j = 0; j < 128; j++)
		checksum += pedid[j];

	if (checksum != 0) {
		pr_err("E-EDID checksum failed!!\n");
		return -EIO;
	}

	return 0;
}

int read_ti_4xxx_edid(struct hdmi_ip_data *ip_data, u8 *pedid, u16 max_length)
{
	int r = 0, n = 0, i = 0;
	int max_ext_blocks = (max_length / 128) - 1;

	r = hdmi_core_ddc_edid(ip_data, pedid, 0);
	if (r) {
		return r;
	} else {
		n = pedid[0x7e];

		/*
		 * README: need to comply with max_length set by the caller.
		 * Better implementation should be to allocate necessary
		 * memory to store EDID according to nb_block field found
		 * in first block
		 */
		if (n > max_ext_blocks)
			n = max_ext_blocks;

		for (i = 1; i <= n; i++) {
			r = hdmi_core_ddc_edid(ip_data, pedid, i);
			if (r)
				return r;
		}
	}
	return 0;
}
EXPORT_SYMBOL(read_ti_4xxx_edid);

static void hdmi_core_init(enum hdmi_deep_color_mode deep_color,
			struct hdmi_core_video_config *video_cfg,
			struct hdmi_core_infoframe_avi *avi_cfg,
			struct hdmi_core_packet_enable_repeat *repeat_cfg)
{
	pr_debug("Enter hdmi_core_init\n");

	/* video core */
	switch (deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		video_cfg->ip_bus_width = HDMI_INPUT_10BIT;
		video_cfg->op_dither_truc = HDMI_OUTPUTTRUNCATION_10BIT;
		video_cfg->deep_color_pkt = HDMI_DEEPCOLORPACKECTENABLE;
		video_cfg->pkt_mode = HDMI_PACKETMODE30BITPERPIXEL;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		video_cfg->ip_bus_width = HDMI_INPUT_12BIT;
		video_cfg->op_dither_truc = HDMI_OUTPUTTRUNCATION_12BIT;
		video_cfg->deep_color_pkt = HDMI_DEEPCOLORPACKECTENABLE;
		video_cfg->pkt_mode = HDMI_PACKETMODE36BITPERPIXEL;
		break;
	case HDMI_DEEP_COLOR_24BIT:
	default:
		video_cfg->ip_bus_width = HDMI_INPUT_8BIT;
		video_cfg->op_dither_truc = HDMI_OUTPUTTRUNCATION_8BIT;
		video_cfg->deep_color_pkt = HDMI_DEEPCOLORPACKECTDISABLE;
		video_cfg->pkt_mode = HDMI_PACKETMODERESERVEDVALUE;
		break;
	}

	video_cfg->hdmi_dvi = HDMI_DVI;
	video_cfg->tclk_sel_clkmult = HDMI_FPLL10IDCK;

	/* info frame */
	avi_cfg->db1_format = 0;
	avi_cfg->db1_active_info = 0;
	avi_cfg->db1_bar_info_dv = 0;
	avi_cfg->db1_scan_info = 0;
	avi_cfg->db2_colorimetry = 0;
	avi_cfg->db2_aspect_ratio = 0;
	avi_cfg->db2_active_fmt_ar = 0;
	avi_cfg->db3_itc = 0;
	avi_cfg->db3_ec = 0;
	avi_cfg->db3_q_range = 0;
	avi_cfg->db3_nup_scaling = 0;
	avi_cfg->db4_videocode = 0;
	avi_cfg->db5_pixel_repeat = 0;
	avi_cfg->db6_7_line_eoftop = 0 ;
	avi_cfg->db8_9_line_sofbottom = 0;
	avi_cfg->db10_11_pixel_eofleft = 0;
	avi_cfg->db12_13_pixel_sofright = 0;

	/* packet enable and repeat */
	repeat_cfg->audio_pkt = 0;
	repeat_cfg->audio_pkt_repeat = 0;
	repeat_cfg->avi_infoframe = 0;
	repeat_cfg->avi_infoframe_repeat = 0;
	repeat_cfg->gen_cntrl_pkt = 0;
	repeat_cfg->gen_cntrl_pkt_repeat = 0;
	repeat_cfg->generic_pkt = 0;
	repeat_cfg->generic_pkt_repeat = 0;
}

static void hdmi_core_powerdown_disable(struct hdmi_ip_data *ip_data)
{
	pr_debug("Enter hdmi_core_powerdown_disable\n");
	REG_FLD_MOD(hdmi_core_sys_base(ip_data), HDMI_CORE_CTRL1, 0x0, 0, 0);
}

static void hdmi_core_swreset_release(struct hdmi_ip_data *ip_data)
{
	pr_debug("Enter hdmi_core_swreset_release\n");
	REG_FLD_MOD(hdmi_core_sys_base(ip_data), HDMI_CORE_SYS_SRST, 0x0, 0, 0);
}

static void hdmi_core_swreset_assert(struct hdmi_ip_data *ip_data)
{
	pr_debug("Enter hdmi_core_swreset_assert\n");
	REG_FLD_MOD(hdmi_core_sys_base(ip_data), HDMI_CORE_SYS_SRST, 0x1, 0, 0);
}

/* HDMI_CORE_VIDEO_CONFIG */
static void hdmi_core_video_config(struct hdmi_ip_data *ip_data,
				struct hdmi_core_video_config *cfg)
{
	u32 r = 0;

	/* sys_ctrl1 default configuration not tunable */
	r = hdmi_read_reg(hdmi_core_sys_base(ip_data), HDMI_CORE_CTRL1);
	r = FLD_MOD(r, HDMI_CORE_CTRL1_VEN_FOLLOWVSYNC, 5, 5);
	r = FLD_MOD(r, HDMI_CORE_CTRL1_HEN_FOLLOWHSYNC, 4, 4);
	r = FLD_MOD(r, HDMI_CORE_CTRL1_BSEL_24BITBUS, 2, 2);
	r = FLD_MOD(r, HDMI_CORE_CTRL1_EDGE_RISINGEDGE, 1, 1);
	/* PD bit has to be written to recieve the interrupts */
	r = FLD_MOD(r, HDMI_CORE_CTRL1_POWER_DOWN, 0, 0);
	hdmi_write_reg(hdmi_core_sys_base(ip_data), HDMI_CORE_CTRL1, r);

	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
			HDMI_CORE_SYS_VID_ACEN, cfg->ip_bus_width, 7, 6);

	/* Vid_Mode */
	r = hdmi_read_reg(hdmi_core_sys_base(ip_data), HDMI_CORE_SYS_VID_MODE);

	/* dither truncation configuration */
	if (cfg->op_dither_truc > HDMI_OUTPUTTRUNCATION_12BIT) {
		r = FLD_MOD(r, cfg->op_dither_truc - 3, 7, 6);
		r = FLD_MOD(r, 1, 5, 5);
	} else {
		r = FLD_MOD(r, cfg->op_dither_truc, 7, 6);
		r = FLD_MOD(r, 0, 5, 5);
	}
	hdmi_write_reg(hdmi_core_sys_base(ip_data), HDMI_CORE_SYS_VID_MODE, r);

	/* HDMI_Ctrl */
	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_HDMI_CTRL);
	r = FLD_MOD(r, cfg->deep_color_pkt, 6, 6);
	r = FLD_MOD(r, cfg->pkt_mode, 5, 3);
	r = FLD_MOD(r, cfg->hdmi_dvi, 0, 0);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_HDMI_CTRL, r);

	/* TMDS_CTRL */
	REG_FLD_MOD(hdmi_core_sys_base(ip_data),
			HDMI_CORE_SYS_TMDS_CTRL, cfg->tclk_sel_clkmult, 6, 5);
}

static void hdmi_core_aux_infoframe_avi_config(struct hdmi_ip_data *ip_data,
		struct hdmi_core_infoframe_avi info_avi)
{
	u32 val;
	char sum = 0, checksum = 0;

	sum += 0x82 + 0x002 + 0x00D;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_TYPE, 0x082);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_VERS, 0x002);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_LEN, 0x00D);

	val = (info_avi.db1_format << 5) |
		(info_avi.db1_active_info << 4) |
		(info_avi.db1_bar_info_dv << 2) |
		(info_avi.db1_scan_info);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(0), val);
	sum += val;

	val = (info_avi.db2_colorimetry << 6) |
		(info_avi.db2_aspect_ratio << 4) |
		(info_avi.db2_active_fmt_ar);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(1), val);
	sum += val;

	val = (info_avi.db3_itc << 7) |
		(info_avi.db3_ec << 4) |
		(info_avi.db3_q_range << 2) |
		(info_avi.db3_nup_scaling);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(2), val);
	sum += val;

	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(3),
					info_avi.db4_videocode);
	sum += info_avi.db4_videocode;

	val = info_avi.db5_pixel_repeat;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(4), val);
	sum += val;

	val = info_avi.db6_7_line_eoftop & 0x00FF;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(5), val);
	sum += val;

	val = ((info_avi.db6_7_line_eoftop >> 8) & 0x00FF);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(6), val);
	sum += val;

	val = info_avi.db8_9_line_sofbottom & 0x00FF;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(7), val);
	sum += val;

	val = ((info_avi.db8_9_line_sofbottom >> 8) & 0x00FF);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(8), val);
	sum += val;

	val = info_avi.db10_11_pixel_eofleft & 0x00FF;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(9), val);
	sum += val;

	val = ((info_avi.db10_11_pixel_eofleft >> 8) & 0x00FF);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(10), val);
	sum += val;

	val = info_avi.db12_13_pixel_sofright & 0x00FF;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(11), val);
	sum += val;

	val = ((info_avi.db12_13_pixel_sofright >> 8) & 0x00FF);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_DBYTE(12), val);
	sum += val;

	checksum = 0x100 - sum;
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_AVI_CHSUM, checksum);
}

static void hdmi_core_av_packet_config(struct hdmi_ip_data *ip_data,
		struct hdmi_core_packet_enable_repeat repeat_cfg)
{
	/* enable/repeat the infoframe */
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_PB_CTRL1,
		(repeat_cfg.audio_pkt << 5) |
		(repeat_cfg.audio_pkt_repeat << 4) |
		(repeat_cfg.avi_infoframe << 1) |
		(repeat_cfg.avi_infoframe_repeat));

	/* enable/repeat the packet */
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_PB_CTRL2,
		(repeat_cfg.gen_cntrl_pkt << 3) |
		(repeat_cfg.gen_cntrl_pkt_repeat << 2) |
		(repeat_cfg.generic_pkt << 1) |
		(repeat_cfg.generic_pkt_repeat));
}

static void hdmi_wp_init(struct omap_video_timings *timings,
			struct hdmi_video_format *video_fmt,
			struct hdmi_video_interface *video_int)
{
	pr_debug("Enter hdmi_wp_init\n");

	timings->hbp = 0;
	timings->hfp = 0;
	timings->hsw = 0;
	timings->vbp = 0;
	timings->vfp = 0;
	timings->vsw = 0;

	video_fmt->packing_mode = HDMI_PACK_10b_RGB_YUV444;
	video_fmt->y_res = 0;
	video_fmt->x_res = 0;

	video_int->vsp = 0;
	video_int->hsp = 0;

	video_int->interlacing = 0;
	video_int->tm = 0; /* HDMI_TIMING_SLAVE */

}

void hdmi_ti_4xxx_wp_video_start(struct hdmi_ip_data *ip_data, bool start)
{
	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG, start, 31, 31);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_wp_video_start);

int hdmi_ti_4xxx_wp_get_video_state(struct hdmi_ip_data *ip_data)
{
	u32 status = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG);

	return (status & 0x80000000) ? 1 : 0;
}

int hdmi_ti_4xxx_set_wait_soft_reset(struct hdmi_ip_data *ip_data)
{
	u8 count = 0;

	/* reset W1 */
	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_SYSCONFIG, 0x1, 0, 0);

	/* wait till SOFTRESET == 0 */
	while (hdmi_wait_for_bit_change(hdmi_wp_base(ip_data),
					HDMI_WP_SYSCONFIG, 0, 0, 0) != 0) {
		if (count++ > 10) {
			pr_err("SYSCONFIG[SOFTRESET] bit not set to 0\n");
			return -ETIMEDOUT;
		}
	}

	/* Make madule smart and wakeup capable*/
	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_SYSCONFIG, 0x3, 3, 2);

	return 0;
}


static void hdmi_wp_video_init_format(struct hdmi_video_format *video_fmt,
	struct omap_video_timings *timings, struct hdmi_config *param)
{
	pr_debug("Enter hdmi_wp_video_init_format\n");

	video_fmt->y_res = param->timings.yres;
	video_fmt->x_res = param->timings.xres;

	omapfb_fb2dss_timings(&param->timings, timings);
}

static void hdmi_wp_video_config_format(struct hdmi_ip_data *ip_data,
		struct hdmi_video_format *video_fmt)
{
	u32 l = 0;

	REG_FLD_MOD(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG,
			video_fmt->packing_mode, 10, 8);

	l |= FLD_VAL(video_fmt->y_res, 31, 16);
	l |= FLD_VAL(video_fmt->x_res, 15, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_SIZE, l);
}

static void hdmi_wp_video_config_interface(struct hdmi_ip_data *ip_data,
		struct hdmi_video_interface *video_int)
{
	u32 r;
	pr_debug("Enter hdmi_wp_video_config_interface\n");

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG);
	r = FLD_MOD(r, video_int->vsp, 7, 7);
	r = FLD_MOD(r, video_int->hsp, 6, 6);
	r = FLD_MOD(r, video_int->interlacing, 3, 3);
	r = FLD_MOD(r, video_int->tm, 1, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_CFG, r);
}

static void hdmi_wp_video_config_timing(struct hdmi_ip_data *ip_data,
		struct omap_video_timings *timings)
{
	u32 timing_h = 0;
	u32 timing_v = 0;

	pr_debug("Enter hdmi_wp_video_config_timing\n");

	timing_h |= FLD_VAL(timings->hbp, 31, 20);
	timing_h |= FLD_VAL(timings->hfp, 19, 8);
	timing_h |= FLD_VAL(timings->hsw, 7, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_TIMING_H, timing_h);

	timing_v |= FLD_VAL(timings->vbp, 31, 20);
	timing_v |= FLD_VAL(timings->vfp, 19, 8);
	timing_v |= FLD_VAL(timings->vsw, 7, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_VIDEO_TIMING_V, timing_v);
}

static void hdmi_wp_core_interrupt_set(struct hdmi_ip_data *ip_data, u32 val)
{
	u32	irqStatus;
	irqStatus = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_IRQENABLE_SET);
	pr_debug("[HDMI] WP_IRQENABLE_SET..currently reads as:%x\n", irqStatus);
	irqStatus = irqStatus | val;
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_IRQENABLE_SET, irqStatus);
	pr_debug("[HDMI]WP_IRQENABLE_SET..changed to :%x\n", irqStatus);
}

void hdmi_ti_4xxx_basic_configure(struct hdmi_ip_data *ip_data,
			struct hdmi_config *cfg)
{
	/* HDMI */
	struct omap_video_timings video_timing;
	struct hdmi_video_format video_format;
	struct hdmi_video_interface video_interface;
	/* HDMI core */
	struct hdmi_core_infoframe_avi avi_cfg;
	struct hdmi_core_video_config v_core_cfg;
	struct hdmi_core_packet_enable_repeat repeat_cfg;

	hdmi_wp_init(&video_timing, &video_format,
		&video_interface);

	hdmi_core_init(cfg->deep_color, &v_core_cfg,
		&avi_cfg,
		&repeat_cfg);

	hdmi_wp_core_interrupt_set(ip_data, HDMI_WP_IRQENABLE_CORE |
				HDMI_WP_AUDIO_FIFO_UNDERFLOW);

	hdmi_wp_video_init_format(&video_format, &video_timing, cfg);

	hdmi_wp_video_config_timing(ip_data, &video_timing);

	/* video config */
	video_format.packing_mode = HDMI_PACK_24b_RGB_YUV444_YUV422;

	hdmi_wp_video_config_format(ip_data, &video_format);

	video_interface.vsp = !!(cfg->timings.sync & FB_SYNC_VERT_HIGH_ACT);
	video_interface.hsp = !!(cfg->timings.sync & FB_SYNC_HOR_HIGH_ACT);
	video_interface.interlacing = cfg->timings.vmode & FB_VMODE_INTERLACED;
	video_interface.tm = 1 ; /* HDMI_TIMING_MASTER_24BIT */

	hdmi_wp_video_config_interface(ip_data, &video_interface);

	/*
	 * configure core video part
	 * set software reset in the core
	 */
	hdmi_core_swreset_assert(ip_data);

	/* power down off */
	hdmi_core_powerdown_disable(ip_data);

	v_core_cfg.pkt_mode = HDMI_PACKETMODE24BITPERPIXEL;
	v_core_cfg.hdmi_dvi = cfg->cm.mode;

	hdmi_core_video_config(ip_data, &v_core_cfg);

	/* release software reset in the core */
	hdmi_core_swreset_release(ip_data);

	/*
	 * configure packet
	 * info frame video see doc CEA861-D page 65
	 */
	avi_cfg.db1_format = HDMI_INFOFRAME_AVI_DB1Y_RGB;
	avi_cfg.db1_active_info =
		HDMI_INFOFRAME_AVI_DB1A_ACTIVE_FORMAT_OFF;
	avi_cfg.db1_bar_info_dv = HDMI_INFOFRAME_AVI_DB1B_NO;
	avi_cfg.db1_scan_info = HDMI_INFOFRAME_AVI_DB1S_0;
	avi_cfg.db2_colorimetry = HDMI_INFOFRAME_AVI_DB2C_NO;
	avi_cfg.db2_aspect_ratio = HDMI_INFOFRAME_AVI_DB2M_NO;
	if (cfg->cm.mode == HDMI_HDMI && cfg->cm.code < CEA_MODEDB_SIZE) {
		if (cea_modes[cfg->cm.code].flag & FB_FLAG_RATIO_16_9)
			avi_cfg.db2_aspect_ratio = HDMI_INFOFRAME_AVI_DB2M_169;
		else if (cea_modes[cfg->cm.code].flag & FB_FLAG_RATIO_4_3)
			avi_cfg.db2_aspect_ratio = HDMI_INFOFRAME_AVI_DB2M_43;
	}
	avi_cfg.db2_active_fmt_ar = HDMI_INFOFRAME_AVI_DB2R_SAME;
	avi_cfg.db3_itc = HDMI_INFOFRAME_AVI_DB3ITC_NO;
	avi_cfg.db3_ec = HDMI_INFOFRAME_AVI_DB3EC_XVYUV601;
	avi_cfg.db3_q_range = HDMI_INFOFRAME_AVI_DB3Q_DEFAULT;
	avi_cfg.db3_nup_scaling = HDMI_INFOFRAME_AVI_DB3SC_NO;
	avi_cfg.db4_videocode = cfg->cm.code;
	avi_cfg.db5_pixel_repeat = HDMI_INFOFRAME_AVI_DB5PR_NO;
	avi_cfg.db6_7_line_eoftop = 0;
	avi_cfg.db8_9_line_sofbottom = 0;
	avi_cfg.db10_11_pixel_eofleft = 0;
	avi_cfg.db12_13_pixel_sofright = 0;

	hdmi_core_aux_infoframe_avi_config(ip_data, avi_cfg);

	/* enable/repeat the infoframe */
	repeat_cfg.avi_infoframe = HDMI_PACKETENABLE;
	repeat_cfg.avi_infoframe_repeat = HDMI_PACKETREPEATON;
	/* wakeup */
	repeat_cfg.audio_pkt = HDMI_PACKETENABLE;
	repeat_cfg.audio_pkt_repeat = HDMI_PACKETREPEATON;
	hdmi_core_av_packet_config(ip_data, repeat_cfg);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_basic_configure);

u32 hdmi_ti_4xxx_irq_handler(struct hdmi_ip_data *ip_data)
{
	u32 val, sys_stat = 0, core_state = 0;
	u32 intr2 = 0, intr3 = 0, r = 0;
	void __iomem *wp_base = hdmi_wp_base(ip_data);
	void __iomem *core_base = hdmi_core_sys_base(ip_data);

	pr_debug("Enter hdmi_ti_4xxx_irq_handler\n");

	val = hdmi_read_reg(wp_base, HDMI_WP_IRQSTATUS);
	if (val & HDMI_WP_IRQSTATUS_CORE) {
		core_state = hdmi_read_reg(core_base, HDMI_CORE_SYS_INTR_STATE);
		if (core_state & 0x1) {
			sys_stat = hdmi_read_reg(core_base,
						 HDMI_CORE_SYS_SYS_STAT);
			intr2 = hdmi_read_reg(core_base, HDMI_CORE_SYS_INTR2);
			intr3 = hdmi_read_reg(core_base, HDMI_CORE_SYS_INTR3);

			pr_debug("HDMI_CORE_SYS_SYS_STAT = 0x%x\n", sys_stat);
			pr_debug("HDMI_CORE_SYS_INTR2 = 0x%x\n", intr2);
			pr_debug("HDMI_CORE_SYS_INTR3 = 0x%x\n", intr3);

			hdmi_write_reg(core_base, HDMI_CORE_SYS_INTR2, intr2);
			hdmi_write_reg(core_base, HDMI_CORE_SYS_INTR3, intr3);

			hdmi_read_reg(core_base, HDMI_CORE_SYS_INTR2);
			hdmi_read_reg(core_base, HDMI_CORE_SYS_INTR3);
		}
	}

	if (val & HDMI_WP_AUDIO_FIFO_UNDERFLOW)
		pr_err("HDMI_WP_AUDIO_FIFO_UNDERFLOW\n");

	pr_debug("HDMI_WP_IRQSTATUS = 0x%x\n", val);
	pr_debug("HDMI_CORE_SYS_INTR_STATE = 0x%x\n", core_state);

	if (intr2 & HDMI_CORE_SYSTEM_INTR2__BCAP)
		r |= HDMI_BCAP;

	if (intr3 & HDMI_CORE_SYSTEM_INTR3__RI_ERR)
		r |= HDMI_RI_ERR;

	/* Ack other interrupts if any */
	hdmi_write_reg(wp_base, HDMI_WP_IRQSTATUS, val);
	/* flush posted write */
	hdmi_read_reg(wp_base, HDMI_WP_IRQSTATUS);
	return r;
}
EXPORT_SYMBOL(hdmi_ti_4xxx_irq_handler);

void hdmi_ti_4xxx_dump_regs(struct hdmi_ip_data *ip_data, struct seq_file *s)
{
#define DUMPREG(g, r) seq_printf(s, "%-35s %08x\n", #r, hdmi_read_reg(g, r))

	void __iomem *wp_base = hdmi_wp_base(ip_data);
	void __iomem *core_sys_base = hdmi_core_sys_base(ip_data);
	void __iomem *phy_base = hdmi_phy_base(ip_data);
	void __iomem *pll_base = hdmi_pll_base(ip_data);
	void __iomem *av_base = hdmi_av_base(ip_data);

	/* wrapper registers */
	DUMPREG(wp_base, HDMI_WP_REVISION);
	DUMPREG(wp_base, HDMI_WP_SYSCONFIG);
	DUMPREG(wp_base, HDMI_WP_IRQSTATUS_RAW);
	DUMPREG(wp_base, HDMI_WP_IRQSTATUS);
	DUMPREG(wp_base, HDMI_WP_PWR_CTRL);
	DUMPREG(wp_base, HDMI_WP_IRQENABLE_SET);
	DUMPREG(wp_base, HDMI_WP_VIDEO_SIZE);
	DUMPREG(wp_base, HDMI_WP_VIDEO_TIMING_H);
	DUMPREG(wp_base, HDMI_WP_VIDEO_TIMING_V);
	DUMPREG(wp_base, HDMI_WP_WP_CLK);

	DUMPREG(core_sys_base, HDMI_CORE_SYS_VND_IDL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DEV_IDL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DEV_IDH);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DEV_REV);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_SRST);
	DUMPREG(core_sys_base, HDMI_CORE_CTRL1);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_SYS_STAT);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_VID_ACEN);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_VID_MODE);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_INTR_STATE);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_INTR1);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_INTR2);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_INTR3);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_INTR4);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_UMASK1);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_TMDS_CTRL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_DLY);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_CTRL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_TOP);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_CNTL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_CNTH);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_LINL);
	DUMPREG(core_sys_base, HDMI_CORE_SYS_DE_LINH_1);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_CMD);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_STATUS);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_ADDR);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_OFFSET);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_COUNT1);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_COUNT2);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_DATA);
	DUMPREG(core_sys_base, HDMI_CORE_DDC_SEGM);

	DUMPREG(av_base, HDMI_CORE_AV_HDMI_CTRL);
	DUMPREG(av_base, HDMI_CORE_AV_AVI_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_DBYTE);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_AUD_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_DBYTE);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_GEN_DBYTE);
	DUMPREG(av_base, HDMI_CORE_AV_GEN_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_GEN2_DBYTE);
	DUMPREG(av_base, HDMI_CORE_AV_GEN2_DBYTE_NELEMS);
	DUMPREG(av_base, HDMI_CORE_AV_ACR_CTRL);
	DUMPREG(av_base, HDMI_CORE_AV_FREQ_SVAL);
	DUMPREG(av_base, HDMI_CORE_AV_N_SVAL1);
	DUMPREG(av_base, HDMI_CORE_AV_N_SVAL2);
	DUMPREG(av_base, HDMI_CORE_AV_N_SVAL3);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_SVAL1);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_SVAL2);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_SVAL3);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_HVAL1);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_HVAL2);
	DUMPREG(av_base, HDMI_CORE_AV_CTS_HVAL3);
	DUMPREG(av_base, HDMI_CORE_AV_AUD_MODE);
	DUMPREG(av_base, HDMI_CORE_AV_SPDIF_CTRL);
	DUMPREG(av_base, HDMI_CORE_AV_HW_SPDIF_FS);
	DUMPREG(av_base, HDMI_CORE_AV_SWAP_I2S);
	DUMPREG(av_base, HDMI_CORE_AV_SPDIF_ERTH);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_IN_MAP);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_IN_CTRL);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_CHST0);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_CHST1);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_CHST2);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_CHST4);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_CHST5);
	DUMPREG(av_base, HDMI_CORE_AV_ASRC);
	DUMPREG(av_base, HDMI_CORE_AV_I2S_IN_LEN);
	DUMPREG(av_base, HDMI_CORE_AV_AUDO_TXSTAT);
	DUMPREG(av_base, HDMI_CORE_AV_AUD_PAR_BUSCLK_1);
	DUMPREG(av_base, HDMI_CORE_AV_AUD_PAR_BUSCLK_2);
	DUMPREG(av_base, HDMI_CORE_AV_AUD_PAR_BUSCLK_3);
	DUMPREG(av_base, HDMI_CORE_AV_TEST_TXCTRL);

	DUMPREG(av_base, HDMI_CORE_AV_DPD);
	DUMPREG(av_base, HDMI_CORE_AV_PB_CTRL1);
	DUMPREG(av_base, HDMI_CORE_AV_PB_CTRL2);
	DUMPREG(av_base, HDMI_CORE_AV_AVI_TYPE);
	DUMPREG(av_base, HDMI_CORE_AV_AVI_VERS);
	DUMPREG(av_base, HDMI_CORE_AV_AVI_LEN);
	DUMPREG(av_base, HDMI_CORE_AV_AVI_CHSUM);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_TYPE);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_VERS);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_LEN);
	DUMPREG(av_base, HDMI_CORE_AV_SPD_CHSUM);
	DUMPREG(av_base, HDMI_CORE_AV_AUDIO_TYPE);
	DUMPREG(av_base, HDMI_CORE_AV_AUDIO_VERS);
	DUMPREG(av_base, HDMI_CORE_AV_AUDIO_LEN);
	DUMPREG(av_base, HDMI_CORE_AV_AUDIO_CHSUM);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_TYPE);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_VERS);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_LEN);
	DUMPREG(av_base, HDMI_CORE_AV_MPEG_CHSUM);
	DUMPREG(av_base, HDMI_CORE_AV_CP_BYTE1);
	DUMPREG(av_base, HDMI_CORE_AV_CEC_ADDR_ID);

	DUMPREG(pll_base, PLLCTRL_PLL_CONTROL);
	DUMPREG(pll_base, PLLCTRL_PLL_STATUS);
	DUMPREG(pll_base, PLLCTRL_PLL_GO);
	DUMPREG(pll_base, PLLCTRL_CFG1);
	DUMPREG(pll_base, PLLCTRL_CFG2);
	DUMPREG(pll_base, PLLCTRL_CFG3);
	DUMPREG(pll_base, PLLCTRL_CFG4);

	DUMPREG(phy_base, HDMI_TXPHY_TX_CTRL);
	DUMPREG(phy_base, HDMI_TXPHY_DIGITAL_CTRL);
	DUMPREG(phy_base, HDMI_TXPHY_POWER_CTRL);
	DUMPREG(phy_base, HDMI_TXPHY_PAD_CFG_CTRL);

#undef DUMPREG
}
EXPORT_SYMBOL(hdmi_ti_4xxx_dump_regs);

int hdmi_ti_4xxx_config_audio_acr(struct hdmi_ip_data *ip_data,
				u32 sample_freq, u32 *n, u32 *cts, u32 pclk)
{
	u32 r;
	u32 deep_color = 0;


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
EXPORT_SYMBOL(hdmi_ti_4xxx_config_audio_acr);


void hdmi_ti_4xxx_wp_audio_config_format(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_format *aud_fmt)
{
	u32 r, reset_wp;

	pr_debug("Enter hdmi_wp_audio_config_format\n");

	reset_wp = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CTRL);
	/* Reset HDMI wrapper */
	if (reset_wp & 0x80000000)
		REG_FLD_MOD(hdmi_wp_base(ip_data),
		HDMI_WP_AUDIO_CTRL, 0, 31, 31);

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

	if (r & 0x80000000)
		REG_FLD_MOD(hdmi_wp_base(ip_data),
		HDMI_WP_AUDIO_CTRL, 1, 31, 31);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_wp_audio_config_format);

void hdmi_ti_4xxx_wp_audio_config_dma(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_dma *aud_dma)
{
	u32 r;

	pr_debug("Enter hdmi_wp_audio_config_dma\n");

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG2);
	r = FLD_MOD(r, aud_dma->transfer_size, 15, 8);
	r = FLD_MOD(r, aud_dma->block_size, 7, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CFG2, r);

	r = hdmi_read_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CTRL);
	r = FLD_MOD(r, aud_dma->mode, 9, 9);
	r = FLD_MOD(r, aud_dma->fifo_threshold, 8, 0);
	hdmi_write_reg(hdmi_wp_base(ip_data), HDMI_WP_AUDIO_CTRL, r);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_wp_audio_config_dma);


void hdmi_ti_4xxx_core_audio_config(struct hdmi_ip_data *ip_data,
					struct hdmi_core_audio_config *cfg)
{
	u32 r;

	/* audio clock recovery parameters */
	r = hdmi_read_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_ACR_CTRL);
	/*
	 * MCLK_EN: use TCLK for ACR packets. For devices that use
	 * the MCLK, this is the first part of the MCLK initialization
	 */
	r = FLD_MOD(r, 0, 2, 2);
	r = FLD_MOD(r, cfg->en_acr_pkt, 1, 1);
	r = FLD_MOD(r, cfg->cts_mode, 0, 0);
	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_ACR_CTRL, r);

	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_N_SVAL1, cfg->n, 7, 0);
	REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_N_SVAL2, cfg->n >> 8, 7, 0);
	REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_N_SVAL3, cfg->n >> 16, 7, 0);

	if (cfg->use_mclk)
		REG_FLD_MOD(hdmi_av_base(ip_data),
			HDMI_CORE_AV_FREQ_SVAL, cfg->mclk_mode, 2, 0);

	if (cfg->cts_mode == HDMI_AUDIO_CTS_MODE_SW) {
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL1, cfg->cts, 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL2, cfg->cts >> 8, 7, 0);
		REG_FLD_MOD(hdmi_av_base(ip_data),
				HDMI_CORE_AV_CTS_SVAL3, cfg->cts >> 16, 7, 0);
	} else {
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

	/* For devices using MCLK, this completes its initialization. */
	REG_FLD_MOD(hdmi_av_base(ip_data), HDMI_CORE_AV_ACR_CTRL,
							cfg->use_mclk, 2, 2);

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

	hdmi_write_reg(hdmi_av_base(ip_data), HDMI_CORE_AV_SWAP_I2S, 0x29);

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
EXPORT_SYMBOL(hdmi_ti_4xxx_core_audio_config);

void hdmi_ti_4xxx_core_audio_infoframe_config(struct hdmi_ip_data *ip_data,
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
EXPORT_SYMBOL(hdmi_ti_4xxx_core_audio_infoframe_config);

void hdmi_ti_4xxx_audio_transfer_en(struct hdmi_ip_data *ip_data,
						bool enable)
{
	REG_FLD_MOD(hdmi_wp_base(ip_data),
			HDMI_WP_AUDIO_CTRL, enable, 30, 30);
	REG_FLD_MOD(hdmi_av_base(ip_data),
			HDMI_CORE_AV_AUD_MODE, enable, 0, 0);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_audio_transfer_en);


void hdmi_ti_4xxx_wp_audio_enable(struct hdmi_ip_data *ip_data, bool enable)
{
	REG_FLD_MOD(hdmi_wp_base(ip_data),
			HDMI_WP_AUDIO_CTRL, enable, 31, 31);
}
EXPORT_SYMBOL(hdmi_ti_4xxx_wp_audio_enable);

int hdmi_ti_4xx_check_aksv_data(struct hdmi_ip_data *ip_data)
{
	u32 aksv_data[5];
	int i, j, ret;
	int one = 0, zero = 0;
	/* check if HDCP AKSV registers are populated.
	 * If not load the keys and reset the wrapper.
	 */
	for (i = 0; i < 5; i++) {
		aksv_data[i] = hdmi_read_reg(hdmi_core_sys_base(ip_data),
					     HDMI_CORE_AKSV(i));
		/* Count number of zero / one */
		for (j = 0; j < 8; j++)
			(aksv_data[i] & (0x01 << j)) ? one++ : zero++;
		pr_debug("%x ", aksv_data[i] & 0xFF);
	}

	ret = (one == zero) ? HDMI_AKSV_VALID :
		(one == 0) ? HDMI_AKSV_ZERO : HDMI_AKSV_ERROR;

	return ret;

}
EXPORT_SYMBOL(hdmi_ti_4xx_check_aksv_data);

static int __init hdmi_ti_4xxx_init(void)
{
	return 0;
}

static void __exit hdmi_ti_4xxx_exit(void)
{

}

module_init(hdmi_ti_4xxx_init);
module_exit(hdmi_ti_4xxx_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("hdmi_ti_4xxx_ip module");
MODULE_LICENSE("GPL");
