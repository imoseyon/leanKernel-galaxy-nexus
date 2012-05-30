/*
 * omap4-hdmi-card.c
 *
 * OMAP ALSA SoC machine driver for TI OMAP4 HDMI
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/delay.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>
#include <video/omapdss.h>

#define DRV_NAME "omap4-hdmi-audio"

static struct omap_overlay_manager *omap4_hdmi_get_overlay_mgr(void)
{
	int i;
	struct omap_overlay_manager *mgr = NULL;

	/* Find DSS HDMI device */
	for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
		mgr = omap_dss_get_overlay_manager(i);
		if (mgr && mgr->device
			&& mgr->device->type == OMAP_DISPLAY_TYPE_HDMI)
			break;
	}

	if (i == omap_dss_get_num_overlay_managers())
		mgr = NULL;

	return mgr;
}

static int omap4_hdmi_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int count = 0;
	struct omap_overlay_manager *mgr;
	struct device *dev = substream->pcm->card->dev;

	mgr = omap4_hdmi_get_overlay_mgr();
	if (mgr == NULL) {
		dev_err(dev, "HDMI display device not found!\n");
		return -ENODEV;
	}

	/* Make sure HDMI is power-on to avoid L3 interconnect errors */
	while (mgr->device->state != OMAP_DSS_DISPLAY_ACTIVE) {
		msleep(50);
		if (mgr->device->state == OMAP_DSS_DISPLAY_ACTIVE)
			break;
		else if (count > 5)
			return -EIO;
		dev_err(dev, "HDMI display is not active!\n");
		count++;
	}

	return 0;
}

static struct snd_soc_ops omap4_hdmi_dai_ops = {
	.hw_params = omap4_hdmi_dai_hw_params,
};

static struct snd_soc_dai_link omap4_hdmi_dai = {
	.name = "HDMI",
	.stream_name = "HDMI",
	.cpu_dai_name = "hdmi-audio-dai",
	.platform_name = "omap-pcm-audio",
	.codec_name = "omap-hdmi-codec",
	.codec_dai_name = "hdmi-audio-codec",
	.ops = &omap4_hdmi_dai_ops,
};

static int hdmi_max_chan_ctl_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;

	return 0;
}

static int hdmi_max_chan_ctl_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct omap_overlay_manager *mgr;
	struct fb_monspecs *monspecs;
	int i;
	int max = 0;

	mgr = omap4_hdmi_get_overlay_mgr();
	if (mgr == NULL) {
		/* HDMI audio not supported */
		goto out;
	}

	monspecs = &mgr->device->panel.monspecs;
	for (i = 0; i < monspecs->audiodb_len; i++) {
		if (monspecs->audiodb[i].format != FB_AUDIO_LPCM)
			continue;

		if (max < monspecs->audiodb[i].channel_count)
			max = monspecs->audiodb[i].channel_count;
	}

out:
	ucontrol->value.integer.value[0] = max;

	return 0;
}

static struct snd_kcontrol_new hdmi_max_chan_ctl = {
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.name = "Maximum LPCM channels",
	.info = hdmi_max_chan_ctl_info,
	.get = hdmi_max_chan_ctl_get,
};

static struct snd_soc_card snd_soc_omap4_hdmi = {
	.name = "OMAP4HDMI",
	.dai_link = &omap4_hdmi_dai,
	.num_links = 1,

	.controls = &hdmi_max_chan_ctl,
	.num_controls = 1,
};

static __devinit int omap4_hdmi_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_omap4_hdmi;
	int ret;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		card->dev = NULL;
		return ret;
	}
	return 0;
}

static int __devexit omap4_hdmi_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	card->dev = NULL;
	return 0;
}

static struct platform_driver omap4_hdmi_driver = {
	.driver = {
		.name = "omap4-hdmi-audio",
		.owner = THIS_MODULE,
	},
	.probe = omap4_hdmi_probe,
	.remove = __devexit_p(omap4_hdmi_remove),
};

static int __init omap4_hdmi_init(void)
{
	return platform_driver_register(&omap4_hdmi_driver);
}
module_init(omap4_hdmi_init);

static void __exit omap4_hdmi_exit(void)
{
	platform_driver_unregister(&omap4_hdmi_driver);
}
module_exit(omap4_hdmi_exit);

MODULE_AUTHOR("Ricardo Neri <ricardo.neri@ti.com>");
MODULE_DESCRIPTION("OMAP4 HDMI machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
