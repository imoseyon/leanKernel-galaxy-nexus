/*
 * ALSA SoC HMDI codec driver
 *
 * Author: Ricardo Neri <ricardo.neri@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <plat/omap_hwmod.h>
#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>

#include "../../../drivers/video/omap2/dss/dss_features.h"
#include "../../../drivers/video/omap2/dss/dss.h"

#define HDMI_WP		0x0
#define HDMI_CORE_SYS	0x400
#define HDMI_CORE_AV	0x900
#define HDMI_PLLCTRL	0x200
#define HDMI_PHY	0x300

/* hdmi configuration params */
struct hdmi_params {
	int format;
	int sample_freq;
	int channels_nr;
};


/* codec private data */
struct hdmi_codec_data {
	struct hdmi_audio_format audio_fmt;
	struct hdmi_audio_dma audio_dma;
	struct hdmi_core_audio_config audio_core_cfg;
	struct hdmi_core_infoframe_audio aud_if_cfg;
	struct hdmi_ip_data ip_data;
	struct omap_hwmod *oh;
	struct omap_dss_device *dssdev;
	struct notifier_block notifier;
	struct hdmi_params params;
	struct delayed_work delayed_work;
	struct workqueue_struct *workqueue;
	int active;
} hdmi_data;


static int hdmi_audio_set_configuration(struct hdmi_codec_data *priv)
{
	struct hdmi_audio_format *audio_format = &priv->audio_fmt;
	struct hdmi_audio_dma *audio_dma = &priv->audio_dma;
	struct hdmi_core_audio_config *core_cfg = &priv->audio_core_cfg;
	struct hdmi_core_infoframe_audio *aud_if_cfg = &priv->aud_if_cfg;
	int err, n, cts, channel_alloc;
	enum hdmi_core_audio_sample_freq sample_freq;
	u32 pclk = omapdss_hdmi_get_pixel_clock();

	switch (priv->params.format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		core_cfg->i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_20BITS;
		core_cfg->i2s_cfg.word_length =
					HDMI_AUDIO_I2S_CHST_WORD_16_BITS;
		core_cfg->i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_16;
		core_cfg->i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_format->samples_per_word = HDMI_AUDIO_ONEWORD_TWOSAMPLES;
		audio_format->sample_size = HDMI_AUDIO_SAMPLE_16BITS;
		audio_format->justification = HDMI_AUDIO_JUSTIFY_LEFT;
		audio_dma->transfer_size = 0x10;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		core_cfg->i2s_cfg.word_max_length =
			HDMI_AUDIO_I2S_MAX_WORD_24BITS;
		core_cfg->i2s_cfg.word_length =
					HDMI_AUDIO_I2S_CHST_WORD_24_BITS;
		core_cfg->i2s_cfg.in_length_bits =
			HDMI_AUDIO_I2S_INPUT_LENGTH_24;
		audio_format->samples_per_word = HDMI_AUDIO_ONEWORD_ONESAMPLE;
		audio_format->sample_size = HDMI_AUDIO_SAMPLE_24BITS;
		audio_format->justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		core_cfg->i2s_cfg.justification = HDMI_AUDIO_JUSTIFY_RIGHT;
		audio_dma->transfer_size = 0x20;
		break;
	default:
		return -EINVAL;
	}


	switch (priv->params.sample_freq) {
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

	err = hdmi_ti_4xxx_config_audio_acr(&priv->ip_data,
			priv->params.sample_freq, &n, &cts, pclk);
	if (err < 0)
		return err;

	/* Audio wrapper config */
	audio_format->type = HDMI_AUDIO_TYPE_LPCM;
	audio_format->sample_order = HDMI_AUDIO_SAMPLE_LEFT_FIRST;
	/* Disable start/stop signals of IEC 60958 blocks */
	audio_format->en_sig_blk_strt_end = HDMI_AUDIO_BLOCK_SIG_STARTEND_OFF;

	audio_dma->block_size = 0xC0;
	audio_dma->mode = HDMI_AUDIO_TRANSF_DMA;
	audio_dma->fifo_threshold = 0x20; /* in number of samples */

	hdmi_ti_4xxx_wp_audio_config_dma(&priv->ip_data, audio_dma);
	hdmi_ti_4xxx_wp_audio_config_format(&priv->ip_data, audio_format);

	/*
	 * I2S config
	 */
	core_cfg->i2s_cfg.en_high_bitrate_aud = false;
	/* Only used with high bitrate audio */
	core_cfg->i2s_cfg.cbit_order = false;
	/* Serial data and word select should change on sck rising edge */
	core_cfg->i2s_cfg.sck_edge_mode = HDMI_AUDIO_I2S_SCK_EDGE_RISING;
	core_cfg->i2s_cfg.vbit = HDMI_AUDIO_I2S_VBIT_FOR_PCM;
	/* Set I2S word select polarity */
	core_cfg->i2s_cfg.ws_polarity = HDMI_AUDIO_I2S_WS_POLARITY_LOW_IS_LEFT;
	core_cfg->i2s_cfg.direction = HDMI_AUDIO_I2S_MSB_SHIFTED_FIRST;
	/* Set serial data to word select shift. See Phillips spec. */
	core_cfg->i2s_cfg.shift = HDMI_AUDIO_I2S_FIRST_BIT_SHIFT;

	/* Core audio config */
	core_cfg->freq_sample = sample_freq;
	core_cfg->n = n;
	core_cfg->cts = cts;
	if (dss_has_feature(FEAT_HDMI_CTS_SWMODE)) {
		core_cfg->aud_par_busclk = 0;
		core_cfg->cts_mode = HDMI_AUDIO_CTS_MODE_SW;
		core_cfg->use_mclk = cpu_is_omap446x();
	} else {
		core_cfg->aud_par_busclk = (((128 * 31) - 1) << 8);
		core_cfg->cts_mode = HDMI_AUDIO_CTS_MODE_HW;
		core_cfg->use_mclk = true;
		core_cfg->mclk_mode = HDMI_AUDIO_MCLK_128FS;
	}
	core_cfg->en_spdif = false;
	/* Use sample frequency from channel status word */
	core_cfg->fs_override = true;
	/* Enable ACR packets */
	core_cfg->en_acr_pkt = true;
	/* Disable direct streaming digital audio */
	core_cfg->en_dsd_audio = false;
	/* Use parallel audio interface */
	core_cfg->en_parallel_aud_input = true;

	/* Number of channels */

	switch (priv->params.channels_nr) {
	case 2:
		core_cfg->layout = HDMI_AUDIO_LAYOUT_2CH;
		channel_alloc = 0x0;
		audio_format->stereo_channels = HDMI_AUDIO_STEREO_ONECHANNEL;
		audio_format->active_chnnls_msk = 0x03;
		/* Enable one of the four available serial data channels */
		core_cfg->i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN;
		break;
	case 6:
		core_cfg->layout = HDMI_AUDIO_LAYOUT_8CH;
		channel_alloc = 0xB;
		audio_format->stereo_channels = HDMI_AUDIO_STEREO_FOURCHANNELS;
		audio_format->active_chnnls_msk = 0x3f;
		/* Enable all of the four available serial data channels */
		core_cfg->i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN |
				HDMI_AUDIO_I2S_SD1_EN | HDMI_AUDIO_I2S_SD2_EN |
				HDMI_AUDIO_I2S_SD3_EN;
		break;
	case 8:
		core_cfg->layout = HDMI_AUDIO_LAYOUT_8CH;
		channel_alloc = 0x13;
		audio_format->stereo_channels = HDMI_AUDIO_STEREO_FOURCHANNELS;
		audio_format->active_chnnls_msk = 0xff;
		/* Enable all of the four available serial data channels */
		core_cfg->i2s_cfg.active_sds = HDMI_AUDIO_I2S_SD0_EN |
				HDMI_AUDIO_I2S_SD1_EN | HDMI_AUDIO_I2S_SD2_EN |
				HDMI_AUDIO_I2S_SD3_EN;
		break;
	default:
		pr_err("Unsupported number of channels\n");
		return -EINVAL;
	}

	hdmi_ti_4xxx_core_audio_config(&priv->ip_data, core_cfg);
	hdmi_ti_4xxx_wp_audio_config_format(&priv->ip_data, audio_format);

	/*
	 * Configure packet
	 * info frame audio see doc CEA861-D page 74
	 */
	aud_if_cfg->db1_coding_type = HDMI_INFOFRAME_AUDIO_DB1CT_FROM_STREAM;
	aud_if_cfg->db1_channel_count = priv->params.channels_nr;
	aud_if_cfg->db2_sample_freq = HDMI_INFOFRAME_AUDIO_DB2SF_FROM_STREAM;
	aud_if_cfg->db2_sample_size = HDMI_INFOFRAME_AUDIO_DB2SS_FROM_STREAM;
	aud_if_cfg->db4_channel_alloc = channel_alloc;
	aud_if_cfg->db5_downmix_inh = false;
	aud_if_cfg->db5_lsv = 0;

	hdmi_ti_4xxx_core_audio_infoframe_config(&priv->ip_data, aud_if_cfg);
	return 0;

}

int hdmi_audio_notifier_callback(struct notifier_block *nb,
				unsigned long arg, void *ptr)
{
	enum omap_dss_display_state state = arg;

	if (state == OMAP_DSS_DISPLAY_ACTIVE) {
		/* this happens just after hdmi_power_on */
		hdmi_audio_set_configuration(&hdmi_data);
		if (hdmi_data.active) {
			omap_hwmod_set_slave_idlemode(hdmi_data.oh,
							HWMOD_IDLEMODE_NO);
			hdmi_ti_4xxx_wp_audio_enable(&hdmi_data.ip_data, 1);
			queue_delayed_work(hdmi_data.workqueue,
				&hdmi_data.delayed_work,
				msecs_to_jiffies(1));
		}
	} else {
		cancel_delayed_work(&hdmi_data.delayed_work);
	}
	return 0;
}

static void hdmi_audio_work(struct work_struct *work)
{
	hdmi_ti_4xxx_audio_transfer_en(&hdmi_data.ip_data, 1);
}

int hdmi_audio_match(struct omap_dss_device *dssdev, void *arg)
{
	return sysfs_streq(dssdev->name , "hdmi");
}

static int hdmi_audio_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct hdmi_codec_data *priv = snd_soc_codec_get_drvdata(codec);

	priv->params.format = params_format(params);
	priv->params.sample_freq = params_rate(params);
	priv->params.channels_nr = params_channels(params);
	return hdmi_audio_set_configuration(priv);
}

static int hdmi_audio_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct hdmi_codec_data *priv = snd_soc_codec_get_drvdata(codec);
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * switch to no-idle to avoid DSS_L3_ICLK clock
		 * to be shutdown during audio activity (as per TRM)
		 */
		omap_hwmod_set_slave_idlemode(priv->oh,
			HWMOD_IDLEMODE_NO);
		hdmi_ti_4xxx_wp_audio_enable(&priv->ip_data, 1);
		queue_delayed_work(priv->workqueue, &priv->delayed_work,
				msecs_to_jiffies(1));

		priv->active = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		cancel_delayed_work(&hdmi_data.delayed_work);
		priv->active = 0;
		hdmi_ti_4xxx_audio_transfer_en(&priv->ip_data, 0);
		hdmi_ti_4xxx_wp_audio_enable(&priv->ip_data, 0);
		/*
		 * switch back to smart-idle & wakeup capable
		 * after audio activity stops
		 */
		omap_hwmod_set_slave_idlemode(priv->oh,
			HWMOD_IDLEMODE_SMART_WKUP);
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

static int hdmi_audio_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	if (!omapdss_hdmi_get_mode()) {
		pr_err("Current video settings do not support audio.\n");
		return -EIO;
	}
	return 0;
}
static int hdmi_probe(struct snd_soc_codec *codec)
{
	struct platform_device *pdev = to_platform_device(codec->dev);
	struct resource *hdmi_rsrc;
	int ret = 0;

	snd_soc_codec_set_drvdata(codec, &hdmi_data);

	hdmi_rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!hdmi_rsrc) {
		dev_err(&pdev->dev, "Cannot obtain IORESOURCE_MEM HDMI\n");
		ret = -EINVAL;
		goto res_err;
	}


	hdmi_data.oh = omap_hwmod_lookup("dss_hdmi");

	if (!hdmi_data.oh) {
		dev_err(&pdev->dev, "can't find omap_hwmod for hdmi\n");
		ret = -ENODEV;
		goto res_err;
	}

	/* Base address taken from platform */
	hdmi_data.ip_data.base_wp = ioremap(hdmi_rsrc->start,
					resource_size(hdmi_rsrc));

	if (!hdmi_data.ip_data.base_wp) {
		dev_err(&pdev->dev, "can't ioremap WP\n");
		ret = -ENOMEM;
		goto res_err;
	}

	hdmi_data.ip_data.hdmi_core_sys_offset = HDMI_CORE_SYS;
	hdmi_data.ip_data.hdmi_core_av_offset = HDMI_CORE_AV;
	hdmi_data.ip_data.hdmi_pll_offset = HDMI_PLLCTRL;
	hdmi_data.ip_data.hdmi_phy_offset = HDMI_PHY;

	hdmi_data.dssdev = omap_dss_find_device(NULL, hdmi_audio_match);

	if (!hdmi_data.dssdev) {
		dev_err(&pdev->dev, "can't find HDMI device\n");
		ret = -ENODEV;
		goto dssdev_err;
	}

	hdmi_data.notifier.notifier_call = hdmi_audio_notifier_callback;
	blocking_notifier_chain_register(&hdmi_data.dssdev->state_notifiers,
			&hdmi_data.notifier);

	hdmi_data.workqueue = create_singlethread_workqueue("hdmi-codec");

	INIT_DELAYED_WORK(&hdmi_data.delayed_work, hdmi_audio_work);

	return 0;

dssdev_err:
	iounmap(hdmi_data.ip_data.base_wp);
res_err:
	return ret;

}

static int hdmi_remove(struct snd_soc_codec *codec)
{
	struct hdmi_codec_data *priv = snd_soc_codec_get_drvdata(codec);

	blocking_notifier_chain_unregister(&priv->dssdev->state_notifiers,
						&priv->notifier);
	iounmap(priv->ip_data.base_wp);
	kfree(priv);
	return 0;
}


static struct snd_soc_codec_driver hdmi_audio_codec_drv = {
	.probe = hdmi_probe,
	.remove = hdmi_remove,
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
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &hdmi_audio_codec_ops,
};

static __devinit int hdmi_codec_probe(struct platform_device *pdev)
{
	int r;

	/* Register ASoC codec DAI */
	r = snd_soc_register_codec(&pdev->dev, &hdmi_audio_codec_drv,
					&hdmi_codec_dai_drv, 1);
	if (r) {
		dev_err(&pdev->dev, "can't register ASoC HDMI audio codec\n");
		return r;
	}

	return 0;
}

static int __devexit hdmi_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}


static struct platform_driver hdmi_codec_driver = {
	.probe          = hdmi_codec_probe,
	.remove         = __devexit_p(hdmi_codec_remove),
	.driver         = {
		.name   = "omap-hdmi-codec",
		.owner  = THIS_MODULE,
	},
};


static int __init hdmi_codec_init(void)
{
	return platform_driver_register(&hdmi_codec_driver);
}
module_init(hdmi_codec_init);

static void __exit hdmi_codec_exit(void)
{
	platform_driver_unregister(&hdmi_codec_driver);
}
module_exit(hdmi_codec_exit);


MODULE_AUTHOR("Ricardo Neri <ricardo.neri@ti.com>");
MODULE_DESCRIPTION("ASoC HDMI codec driver");
MODULE_LICENSE("GPL");
