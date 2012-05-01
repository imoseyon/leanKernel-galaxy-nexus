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

#define HDMI_HPD_LOW		0x10
#define HDMI_HPD_HIGH		0x20
#define HDMI_BCAP		0x40
#define HDMI_RI_ERR		0x80
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

enum hdmi_deep_color_mode {
	HDMI_DEEP_COLOR_24BIT = 0,
	HDMI_DEEP_COLOR_30BIT = 1,
	HDMI_DEEP_COLOR_36BIT = 2
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
	struct fb_videomode timings;
	struct hdmi_cm cm;
	enum hdmi_deep_color_mode	deep_color;
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

struct hdmi_core_audio_i2s_config {
	u8 word_max_length;
	u8 word_length;
	u8 in_length_bits;
	u8 justification;
	u8 en_high_bitrate_aud;
	u8 sck_edge_mode;
	u8 cbit_order;
	u8 vbit;
	u8 ws_polarity;
	u8 direction;
	u8 shift;
	u8 active_sds;
};


enum hdmi_audio_i2s_config {
	HDMI_AUDIO_I2S_WS_POLARITY_LOW_IS_LEFT = 0,
	HDMI_AUDIO_I2S_WS_POLARIT_YLOW_IS_RIGHT = 1,
	HDMI_AUDIO_I2S_MSB_SHIFTED_FIRST = 0,
	HDMI_AUDIO_I2S_LSB_SHIFTED_FIRST = 1,
	HDMI_AUDIO_I2S_MAX_WORD_20BITS = 0,
	HDMI_AUDIO_I2S_MAX_WORD_24BITS = 1,
	HDMI_AUDIO_I2S_CHST_WORD_NOT_SPECIFIED = 0,
	HDMI_AUDIO_I2S_CHST_WORD_16_BITS = 1,
	HDMI_AUDIO_I2S_CHST_WORD_17_BITS = 6,
	HDMI_AUDIO_I2S_CHST_WORD_18_BITS = 2,
	HDMI_AUDIO_I2S_CHST_WORD_19_BITS = 4,
	HDMI_AUDIO_I2S_CHST_WORD_20_BITS_20MAX = 5,
	HDMI_AUDIO_I2S_CHST_WORD_20_BITS_24MAX = 1,
	HDMI_AUDIO_I2S_CHST_WORD_21_BITS = 6,
	HDMI_AUDIO_I2S_CHST_WORD_22_BITS = 2,
	HDMI_AUDIO_I2S_CHST_WORD_23_BITS = 4,
	HDMI_AUDIO_I2S_CHST_WORD_24_BITS = 5,
	HDMI_AUDIO_I2S_SCK_EDGE_FALLING = 0,
	HDMI_AUDIO_I2S_SCK_EDGE_RISING = 1,
	HDMI_AUDIO_I2S_VBIT_FOR_PCM = 0,
	HDMI_AUDIO_I2S_VBIT_FOR_COMPRESSED = 1,
	HDMI_AUDIO_I2S_INPUT_LENGTH_NA = 0,
	HDMI_AUDIO_I2S_INPUT_LENGTH_16 = 2,
	HDMI_AUDIO_I2S_INPUT_LENGTH_17 = 12,
	HDMI_AUDIO_I2S_INPUT_LENGTH_18 = 4,
	HDMI_AUDIO_I2S_INPUT_LENGTH_19 = 8,
	HDMI_AUDIO_I2S_INPUT_LENGTH_20 = 10,
	HDMI_AUDIO_I2S_INPUT_LENGTH_21 = 13,
	HDMI_AUDIO_I2S_INPUT_LENGTH_22 = 5,
	HDMI_AUDIO_I2S_INPUT_LENGTH_23 = 9,
	HDMI_AUDIO_I2S_INPUT_LENGTH_24 = 11,
	HDMI_AUDIO_I2S_FIRST_BIT_SHIFT = 0,
	HDMI_AUDIO_I2S_FIRST_BIT_NO_SHIFT = 1,
	HDMI_AUDIO_I2S_SD0_EN = 1,
	HDMI_AUDIO_I2S_SD1_EN = 1 << 1,
	HDMI_AUDIO_I2S_SD2_EN = 1 << 2,
	HDMI_AUDIO_I2S_SD3_EN = 1 << 3,
};

enum hdmi_audio_mclk_mode {
	HDMI_AUDIO_MCLK_128FS = 0,
	HDMI_AUDIO_MCLK_256FS = 1,
	HDMI_AUDIO_MCLK_384FS = 2,
	HDMI_AUDIO_MCLK_512FS = 3,
	HDMI_AUDIO_MCLK_768FS = 4,
	HDMI_AUDIO_MCLK_1024FS = 5,
	HDMI_AUDIO_MCLK_1152FS = 6,
	HDMI_AUDIO_MCLK_192FS = 7
};


enum hdmi_core_audio_sample_freq {
	HDMI_AUDIO_FS_32000 = 0x3,
	HDMI_AUDIO_FS_44100 = 0x0,
	HDMI_AUDIO_FS_48000 = 0x2,
	HDMI_AUDIO_FS_88200 = 0x8,
	HDMI_AUDIO_FS_96000 = 0xA,
	HDMI_AUDIO_FS_176400 = 0xC,
	HDMI_AUDIO_FS_192000 = 0xE,
	HDMI_AUDIO_FS_NOT_INDICATED = 0x1
};

enum hdmi_core_audio_layout {
	HDMI_AUDIO_LAYOUT_2CH = 0,
	HDMI_AUDIO_LAYOUT_8CH = 1
};

enum hdmi_core_cts_mode {
	HDMI_AUDIO_CTS_MODE_HW = 0,
	HDMI_AUDIO_CTS_MODE_SW = 1
};

enum hdmi_stereo_channels {
	HDMI_AUDIO_STEREO_NOCHANNELS = 0,
	HDMI_AUDIO_STEREO_ONECHANNEL = 1,
	HDMI_AUDIO_STEREO_TWOCHANNELS = 2,
	HDMI_AUDIO_STEREO_THREECHANNELS = 3,
	HDMI_AUDIO_STEREO_FOURCHANNELS = 4
};

enum hdmi_audio_type {
	HDMI_AUDIO_TYPE_LPCM = 0,
	HDMI_AUDIO_TYPE_IEC = 1
};

enum hdmi_audio_justify {
	HDMI_AUDIO_JUSTIFY_LEFT = 0,
	HDMI_AUDIO_JUSTIFY_RIGHT = 1
};

enum hdmi_audio_sample_order {
	HDMI_AUDIO_SAMPLE_RIGHT_FIRST = 0,
	HDMI_AUDIO_SAMPLE_LEFT_FIRST = 1
};

enum hdmi_audio_samples_perword {
	HDMI_AUDIO_ONEWORD_ONESAMPLE = 0,
	HDMI_AUDIO_ONEWORD_TWOSAMPLES = 1
};

enum hdmi_audio_sample_size {
	HDMI_AUDIO_SAMPLE_16BITS = 0,
	HDMI_AUDIO_SAMPLE_24BITS = 1
};

enum hdmi_audio_transf_mode {
	HDMI_AUDIO_TRANSF_DMA = 0,
	HDMI_AUDIO_TRANSF_IRQ = 1
};

enum hdmi_audio_blk_strt_end_sig {
	HDMI_AUDIO_BLOCK_SIG_STARTEND_ON = 0,
	HDMI_AUDIO_BLOCK_SIG_STARTEND_OFF = 1
};


struct hdmi_core_audio_config {
	struct hdmi_core_audio_i2s_config	i2s_cfg;
	enum hdmi_core_audio_sample_freq	freq_sample;
	bool					fs_override;
	u32					n;
	u32					cts;
	u32					aud_par_busclk;
	enum hdmi_core_audio_layout		layout;
	enum hdmi_core_cts_mode			cts_mode;
	bool					use_mclk;
	enum hdmi_audio_mclk_mode		mclk_mode;
	bool					en_acr_pkt;
	bool					en_dsd_audio;
	bool					en_parallel_aud_input;
	bool					en_spdif;
};



struct hdmi_audio_format {
	enum hdmi_stereo_channels		stereo_channels;
	u8					active_chnnls_msk;
	enum hdmi_audio_type			type;
	enum hdmi_audio_justify			justification;
	enum hdmi_audio_sample_order		sample_order;
	enum hdmi_audio_samples_perword		samples_per_word;
	enum hdmi_audio_sample_size		sample_size;
	enum hdmi_audio_blk_strt_end_sig	en_sig_blk_strt_end;
};

struct hdmi_audio_dma {
	u8				transfer_size;
	u8				block_size;
	enum hdmi_audio_transf_mode	mode;
	u16				fifo_threshold;
};

/*
 * Refer to section 8.2 in HDMI 1.3 specification for
 * details about infoframe databytes
 */
struct hdmi_core_infoframe_audio {
	u8 db1_coding_type;
	u8 db1_channel_count;
	u8 db2_sample_freq;
	u8 db2_sample_size;
	u8 db4_channel_alloc;
	bool db5_downmix_inh;
	u8 db5_lsv;	/* Level shift values for downmix */
};



/* INFOFRAME_AVI_ and INFOFRAME_AUDIO_ definitions */
enum hdmi_core_infoframe {
	HDMI_INFOFRAME_AVI_DB1Y_RGB = 0,
	HDMI_INFOFRAME_AVI_DB1Y_YUV422 = 1,
	HDMI_INFOFRAME_AVI_DB1Y_YUV444 = 2,
	HDMI_INFOFRAME_AVI_DB1A_ACTIVE_FORMAT_OFF = 0,
	HDMI_INFOFRAME_AVI_DB1A_ACTIVE_FORMAT_ON =  1,
	HDMI_INFOFRAME_AVI_DB1B_NO = 0,
	HDMI_INFOFRAME_AVI_DB1B_VERT = 1,
	HDMI_INFOFRAME_AVI_DB1B_HORI = 2,
	HDMI_INFOFRAME_AVI_DB1B_VERTHORI = 3,
	HDMI_INFOFRAME_AVI_DB1S_0 = 0,
	HDMI_INFOFRAME_AVI_DB1S_1 = 1,
	HDMI_INFOFRAME_AVI_DB1S_2 = 2,
	HDMI_INFOFRAME_AVI_DB2C_NO = 0,
	HDMI_INFOFRAME_AVI_DB2C_ITU601 = 1,
	HDMI_INFOFRAME_AVI_DB2C_ITU709 = 2,
	HDMI_INFOFRAME_AVI_DB2C_EC_EXTENDED = 3,
	HDMI_INFOFRAME_AVI_DB2M_NO = 0,
	HDMI_INFOFRAME_AVI_DB2M_43 = 1,
	HDMI_INFOFRAME_AVI_DB2M_169 = 2,
	HDMI_INFOFRAME_AVI_DB2R_SAME = 8,
	HDMI_INFOFRAME_AVI_DB2R_43 = 9,
	HDMI_INFOFRAME_AVI_DB2R_169 = 10,
	HDMI_INFOFRAME_AVI_DB2R_149 = 11,
	HDMI_INFOFRAME_AVI_DB3ITC_NO = 0,
	HDMI_INFOFRAME_AVI_DB3ITC_YES = 1,
	HDMI_INFOFRAME_AVI_DB3EC_XVYUV601 = 0,
	HDMI_INFOFRAME_AVI_DB3EC_XVYUV709 = 1,
	HDMI_INFOFRAME_AVI_DB3Q_DEFAULT = 0,
	HDMI_INFOFRAME_AVI_DB3Q_LR = 1,
	HDMI_INFOFRAME_AVI_DB3Q_FR = 2,
	HDMI_INFOFRAME_AVI_DB3SC_NO = 0,
	HDMI_INFOFRAME_AVI_DB3SC_HORI = 1,
	HDMI_INFOFRAME_AVI_DB3SC_VERT = 2,
	HDMI_INFOFRAME_AVI_DB3SC_HORIVERT = 3,
	HDMI_INFOFRAME_AVI_DB5PR_NO = 0,
	HDMI_INFOFRAME_AVI_DB5PR_2 = 1,
	HDMI_INFOFRAME_AVI_DB5PR_3 = 2,
	HDMI_INFOFRAME_AVI_DB5PR_4 = 3,
	HDMI_INFOFRAME_AVI_DB5PR_5 = 4,
	HDMI_INFOFRAME_AVI_DB5PR_6 = 5,
	HDMI_INFOFRAME_AVI_DB5PR_7 = 6,
	HDMI_INFOFRAME_AVI_DB5PR_8 = 7,
	HDMI_INFOFRAME_AVI_DB5PR_9 = 8,
	HDMI_INFOFRAME_AVI_DB5PR_10 = 9,
	HDMI_INFOFRAME_AUDIO_DB1CT_FROM_STREAM = 0,
	HDMI_INFOFRAME_AUDIO_DB1CT_IEC60958 = 1,
	HDMI_INFOFRAME_AUDIO_DB1CT_AC3 = 2,
	HDMI_INFOFRAME_AUDIO_DB1CT_MPEG1 = 3,
	HDMI_INFOFRAME_AUDIO_DB1CT_MP3 = 4,
	HDMI_INFOFRAME_AUDIO_DB1CT_MPEG2_MULTICH = 5,
	HDMI_INFOFRAME_AUDIO_DB1CT_AAC = 6,
	HDMI_INFOFRAME_AUDIO_DB1CT_DTS = 7,
	HDMI_INFOFRAME_AUDIO_DB1CT_ATRAC = 8,
	HDMI_INFOFRAME_AUDIO_DB1CT_ONEBIT = 9,
	HDMI_INFOFRAME_AUDIO_DB1CT_DOLBY_DIGITAL_PLUS = 10,
	HDMI_INFOFRAME_AUDIO_DB1CT_DTS_HD = 11,
	HDMI_INFOFRAME_AUDIO_DB1CT_MAT = 12,
	HDMI_INFOFRAME_AUDIO_DB1CT_DST = 13,
	HDMI_INFOFRAME_AUDIO_DB1CT_WMA_PRO = 14,
	HDMI_INFOFRAME_AUDIO_DB2SF_FROM_STREAM = 0,
	HDMI_INFOFRAME_AUDIO_DB2SF_32000 = 1,
	HDMI_INFOFRAME_AUDIO_DB2SF_44100 = 2,
	HDMI_INFOFRAME_AUDIO_DB2SF_48000 = 3,
	HDMI_INFOFRAME_AUDIO_DB2SF_88200 = 4,
	HDMI_INFOFRAME_AUDIO_DB2SF_96000 = 5,
	HDMI_INFOFRAME_AUDIO_DB2SF_176400 = 6,
	HDMI_INFOFRAME_AUDIO_DB2SF_192000 = 7,
	HDMI_INFOFRAME_AUDIO_DB2SS_FROM_STREAM = 0,
	HDMI_INFOFRAME_AUDIO_DB2SS_16BIT = 1,
	HDMI_INFOFRAME_AUDIO_DB2SS_20BIT = 2,
	HDMI_INFOFRAME_AUDIO_DB2SS_24BIT = 3,
	HDMI_INFOFRAME_AUDIO_DB5_DM_INH_PERMITTED = 0,
	HDMI_INFOFRAME_AUDIO_DB5_DM_INH_PROHIBITED = 1
};

enum hdmi_aksv_err {
	HDMI_AKSV_ZERO = 0,
	HDMI_AKSV_ERROR = 1,
	HDMI_AKSV_VALID = 2
};

int hdmi_ti_4xxx_phy_init(struct hdmi_ip_data *ip_data);
void hdmi_ti_4xxx_phy_off(struct hdmi_ip_data *ip_data, bool set_mode);
int read_ti_4xxx_edid(struct hdmi_ip_data *ip_data, u8 *pedid, u16 max_length);
void hdmi_ti_4xxx_wp_video_start(struct hdmi_ip_data *ip_data, bool start);
int hdmi_ti_4xxx_pll_program(struct hdmi_ip_data *ip_data,
			struct hdmi_pll_info *fmt);
int hdmi_ti_4xxx_set_pll_pwr(struct hdmi_ip_data *ip_data, enum hdmi_pll_pwr val);
void hdmi_ti_4xxx_basic_configure(struct hdmi_ip_data *ip_data,
			struct hdmi_config *cfg);
int hdmi_ti_4xxx_rxdet(struct hdmi_ip_data *ip_data);
int hdmi_ti_4xxx_wp_get_video_state(struct hdmi_ip_data *ip_data);
u32 hdmi_ti_4xxx_irq_handler(struct hdmi_ip_data *ip_data);
void hdmi_ti_4xxx_dump_regs(struct hdmi_ip_data *ip_data, struct seq_file *s);
int hdmi_ti_4xxx_config_audio_acr(struct hdmi_ip_data *ip_data,
			       u32 sample_freq, u32 *n, u32 *cts, u32 pclk);
void hdmi_ti_4xxx_wp_audio_config_dma(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_dma *aud_dma);

void hdmi_ti_4xxx_wp_audio_config_format(struct hdmi_ip_data *ip_data,
					struct hdmi_audio_format *aud_fmt);
void hdmi_ti_4xxx_core_audio_config(struct hdmi_ip_data *ip_data,
					struct hdmi_core_audio_config *cfg);
void hdmi_ti_4xxx_core_audio_infoframe_config(struct hdmi_ip_data *ip_data,
		struct hdmi_core_infoframe_audio *info_aud);
void hdmi_ti_4xxx_audio_transfer_en(struct hdmi_ip_data *ip_data,
						bool idle);
void hdmi_ti_4xxx_wp_audio_enable(struct hdmi_ip_data *ip_data, bool idle);

int hdmi_ti_4xxx_set_wait_soft_reset(struct hdmi_ip_data *ip_data);
int hdmi_ti_4xx_check_aksv_data(struct hdmi_ip_data *ip_data);
#endif
