/*
 * ALSA SoC McASP Audio Layer for TI OMAP processor
 *
 * Multi-channel Audio Serial Port Driver
 *
 * Author: Jon Hunter <jon-hunter@ti.com>,
 *         Dan Milea <dan.milea@ti.com>,
 *
 * Based upon McASP driver written for TI DaVinci
 *
 * Copyright:   (C) 2011  Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <plat/omap_hwmod.h>
#include <plat/clock.h>
#include <plat/dma.h>
#include <plat/dma-44xx.h>

#include "omap-pcm.h"
#include "omap-mcasp.h"

/*
 * McASP register definitions
 */
#define OMAP_MCASP_PID_REG		0x00
#define OMAP_MCASP_SYSCONFIG_REG	0x04

#define OMAP_MCASP_PFUNC_REG		0x10
#define OMAP_MCASP_PDIR_REG		0x14
#define OMAP_MCASP_PDOUT_REG		0x18
#define OMAP_MCASP_PDIN_REG		0x1c
#define OMAP_MCASP_PDSET_REG		0x1c
#define OMAP_MCASP_PDCLR_REG		0x20

#define OMAP_MCASP_GBLCTL_REG		0x44
#define OMAP_MCASP_AMUTE_REG		0x48

#define OMAP_MCASP_TXDITCTL_REG		0x50

#define OMAP_MCASP_TXMASK_REG		0xa4
#define OMAP_MCASP_TXFMT_REG		0xa8
#define OMAP_MCASP_TXFMCTL_REG		0xac

#define OMAP_MCASP_ACLKXCTL_REG		0xb0
#define OMAP_MCASP_AHCLKXCTL_REG	0xb4
#define OMAP_MCASP_TXTDM_REG		0xb8
#define OMAP_MCASP_EVTCTLX_REG		0xbc

#define OMAP_MCASP_TXSTAT_REG		0xc0
#define OMAP_MCASP_TXSTAT_MASK		0x1ff

#define OMAP_MCASP_TXTDMSLOT_REG	0xc4
#define OMAP_MCASP_TXCLKCHK_REG		0xc8
#define OMAP_MCASP_TXEVTCTL_REG		0xcc

/* Left(even TDM Slot) Channel Status Register File */
#define OMAP_MCASP_DITCSRA_REG	0x100
/* Right(odd TDM slot) Channel Status Register File */
#define OMAP_MCASP_DITCSRB_REG	0x118
/* Left(even TDM slot) User Data Register File */
#define OMAP_MCASP_DITUDRA_REG	0x130
/* Right(odd TDM Slot) User Data Register File */
#define OMAP_MCASP_DITUDRB_REG	0x148

/* Serializer n Control Register */
#define OMAP_MCASP_XRSRCTL0_REG	0x180

/* Transmit Buffer for Serializer */
#define OMAP_MCASP_TXBUF0_REG	0x200

/*
 * OMAP_MCASP_PFUNC_REG - Pin Function / GPIO Enable Register Bits
 */
#define AXR0		BIT(0)
#define PFUNC_AMUTE	BIT(25)
#define ACLKX		BIT(26)
#define AHCLKX		BIT(27)
#define AFSX		BIT(28)

/*
 * OMAP_MCASP_PDIR_REG - Pin Direction Register Bits
 */
#define AXR0		BIT(0)
#define PDIR_AMUTE	BIT(25)
#define ACLKX		BIT(26)
#define AHCLKX		BIT(27)
#define AFSX		BIT(28)

/*
 * OMAP_MCASP_TXDITCTL_REG - Transmit DIT Control Register Bits
 */
#define DITEN	BIT(0)	/* Transmit DIT mode enable/disable */
#define VA	BIT(2)
#define VB	BIT(3)

/*
 * OMAP_MCASP_TXFMT_REG - Transmit Bitstream Format Register Bits
 */
#define TXROT(val)	(val)
#define TXROT_MASK	TXROT(0x7)
#define TXSEL		BIT(3)
#define TXSSZ(val)	(val<<4)
#define TXSSZ_MASK	TXSSZ(0xf<<4)
#define TXPAD(val)	(val<<13)
#define TXORD		BIT(15)
#define FSXDLY(val)	(val<<16)

#define ROTATE_24	0x6
#define SLOTSIZE_32	0xf

/*
 * OMAP_MCASP_TXFMCTL_REG -  Transmit Frame Control Register Bits
 */
#define FSXPOL		BIT(0)
#define AFSXE		BIT(1)
#define FSXDUR		BIT(4)
#define FSXMOD(val)	(val<<7)

/*
 * OMAP_MCASP_ACLKXCTL_REG - Transmit Clock Control Register Bits
 */
#define ACLKXDIV(val)	(val)
#define ACLKXE		BIT(5)
#define TX_ASYNC	BIT(6)

/*
 * OMAP_MCASP_AHCLKXCTL_REG - High Frequency Transmit Clock Control
 *     Register Bits
 */
#define AHCLKXDIV(val)	(val)
#define AHCLKXE		BIT(15)

/*
 * OMAP_MCASP_XRSRCTL_BASE_REG -  Serializer Control Register Bits
 */
#define MODE(val)	(val)
#define TXSTATE		BIT(4)

/*
 * OMAP_MCASP_TXTDMSLOT_REG - Transmit TDM Slot Register configuration
 */
#define TXTDMS(n)	(1<<n)

/*
 * OMAP_MCASP_GBLCTL_REG -  Global Control Register Bits
 */
#define TXCLKRST	BIT(8)	/* Transmitter Clock Divider Reset */
#define TXHCLKRST	BIT(9)	/* Transmitter High Frequency Clock Divider*/
#define TXSERCLR	BIT(10)	/* Transmit Serializer Clear */
#define TXSMRST		BIT(11)	/* Transmitter State Machine Reset */
#define TXFSRST		BIT(12)	/* Frame Sync Generator Reset */

/*
 * OMAP_MCASP_AMUTE_REG -  Mute Control Register Bits
 */
#define MUTENA(val)	(val)
#define MUTEINPOL	BIT(2)
#define MUTEINENA	BIT(3)
#define MUTEIN		BIT(4)
#define MUTEX		BIT(6)
#define MUTEFSX		BIT(8)
#define MUTEBADCLKX	BIT(10)
#define MUTETXDMAERR	BIT(12)

/*
 * OMAP_MCASP_TXEVTCTL_REG - Transmitter DMA Event Control Register bits
 */
#define TXDATADMADIS	BIT(0)

/*
 * Stream DMA parameters
 */
static struct omap_pcm_dma_data omap_mcasp_dai_dma_params[] = {
	{
		.name = "Audio playback",
		.dma_req = OMAP44XX_DMA_MCASP1_AXEVT,
		.data_type = OMAP_DMA_DATA_TYPE_S16,
		.sync_mode = OMAP_DMA_SYNC_ELEMENT,
		.port_addr = OMAP44XX_MCASP_DAT_BASE + OMAP_MCASP_TXBUF0_REG,
	},
};

static inline void mcasp_set_bits(void __iomem *reg, u32 val)
{
	__raw_writel(__raw_readl(reg) | val, reg);
}

static inline void mcasp_clr_bits(void __iomem *reg, u32 val)
{
	__raw_writel((__raw_readl(reg) & ~(val)), reg);
}

static inline void mcasp_mod_bits(void __iomem *reg, u32 val, u32 mask)
{
	__raw_writel((__raw_readl(reg) & ~mask) | val, reg);
}

static inline void mcasp_set_reg(void __iomem *reg, u32 val)
{
	__raw_writel(val, reg);
}

static inline u32 mcasp_get_reg(void __iomem *reg)
{
	return (unsigned int)__raw_readl(reg);
}

static inline void mcasp_set_ctl_reg(void __iomem *regs, u32 val)
{
	int i = 0;

	mcasp_set_bits(regs, val);

	/* programming GBLCTL needs to read back from GBLCTL and verfiy */
	/* loop count is to avoid the lock-up */
	for (i = 0; i < 1000; i++) {
		if ((mcasp_get_reg(regs) & val) == val)
			break;
	}

	if (i == 1000 && ((mcasp_get_reg(regs) & val) != val))
		printk(KERN_ERR "GBLCTL write error\n");
}

static int mcasp_lookup_clkdiv(struct omap_mcasp *mcasp, unsigned int rate,
						u32 *ahclkxdiv, u32 *aclkxdiv)
{
	unsigned int i, rate_bit;

	rate_bit = snd_pcm_rate_to_rate_bit(rate);

	for (i = 0; i < mcasp->num_configs; i++) {
		if (mcasp->configs[i].sampling_rate == rate_bit) {
			*aclkxdiv = mcasp->configs[i].aclkxdiv;
			*ahclkxdiv = mcasp->configs[i].ahclkxdiv;
			return 0;
		}
	}

	return -EINVAL;
}

static void mcasp_start_tx(struct omap_mcasp *mcasp)
{
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXHCLKRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXCLKRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXSERCLR);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXSMRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXFSRST);
	mcasp_clr_bits(mcasp->base + OMAP_MCASP_TXEVTCTL_REG, TXDATADMADIS);
}

static void omap_mcasp_start(struct omap_mcasp *mcasp, int stream)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		mcasp_start_tx(mcasp);
}

static void mcasp_stop_tx(struct omap_mcasp *mcasp)
{
	mcasp_set_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, 0);
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG,
			OMAP_MCASP_TXSTAT_MASK);
}

static void omap_mcasp_stop(struct omap_mcasp *mcasp, int stream)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		mcasp_stop_tx(mcasp);
}

static int omap_mcasp_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct platform_device *pdev;
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);

	pdev = to_platform_device(mcasp->dev);

	if (!mcasp->active++)
		pm_runtime_get_sync(&pdev->dev);

	mcasp_set_reg(mcasp->base + OMAP_MCASP_SYSCONFIG_REG, 0x1);

	return 0;
}

static void omap_mcasp_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct platform_device *pdev;
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);

	pdev = to_platform_device(mcasp->dev);

	mcasp_set_reg(mcasp->base + OMAP_MCASP_SYSCONFIG_REG, 0x2);

	if (!--mcasp->active)
		pm_runtime_put_sync(&pdev->dev);
}

/* S/PDIF */
static int omap_hw_dit_param(struct omap_mcasp *mcasp, unsigned int rate)
{
	u32 aclkxdiv, ahclkxdiv;

	/* To support a particular sampling rate we need to ensure that
	 * the McASP functional clock is at the expected frequency. So
	 * make sure the current McASP functional clock frequency is what
	 * we expect.
	 */
	if (clk_get_rate(mcasp->fclk) != mcasp->fclk_rate) {
		dev_err(mcasp->dev, "%s: %s (%ld) != %ld\n",
		       __func__, mcasp->fclk->name, clk_get_rate(mcasp->fclk),
		       mcasp->fclk_rate);
		return -EINVAL;
	}

	/* Set TX frame synch : DIT Mode, 1 bit width, internal, rising edge */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXFMCTL_REG,
						AFSXE | FSXMOD(0x180));

	/* Set the TX clock controls : div = 1 and internal */
	mcasp_set_bits(mcasp->base + OMAP_MCASP_ACLKXCTL_REG,
						ACLKXE | TX_ASYNC);

	/* The SPDIF bit clock is derived from the McASP functional clock.
	 * The McASP has two programmable clock dividers (aclkxdiv and
	 * ahclkxdiv) that are configured via the registers MCASP_ACLKXCTL
	 * and MCASP_AHCLKXCTL. For SPDIF the bit clock frequency should be
	 * 128 * sample rate freq. The dividers are defined as part of
	 * platform data as they are dependent upon the functional clock
	 * setting. Lookup the appropriate dividers for the sampling
	 * frequency that we are playing.
	 */
	if (mcasp_lookup_clkdiv(mcasp, rate, &ahclkxdiv, &aclkxdiv)) {
		dev_err(mcasp->dev,
			"%s: No valid McASP config for sampling rate (%d)!\n",
			__func__, rate);
		return -EINVAL;
	}

	mcasp_set_bits(mcasp->base + OMAP_MCASP_AHCLKXCTL_REG,
					AHCLKXDIV(ahclkxdiv));
	mcasp_set_bits(mcasp->base + OMAP_MCASP_ACLKXCTL_REG,
					AHCLKXDIV(aclkxdiv));

	/* Configure McASP formatter */
	mcasp_mod_bits(mcasp->base + OMAP_MCASP_TXFMT_REG,
					TXSSZ(SLOTSIZE_32), TXSSZ_MASK);
	mcasp_mod_bits(mcasp->base + OMAP_MCASP_TXFMT_REG, TXROT(ROTATE_24),
							TXROT_MASK);
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXMASK_REG, 0xFFFF);

	/* Set the TX tdm : for all the slots */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXTDM_REG, 0xFFFFFFFF);

	/* configure the serializer for transmit mode operation */
	mcasp_set_bits(mcasp->base + OMAP_MCASP_XRSRCTL0_REG, MODE(1));

	/* All PINS as McASP */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_PFUNC_REG, 0);

	mcasp_set_bits(mcasp->base + OMAP_MCASP_PDIR_REG, AXR0);

	/* Enable the DIT */
	mcasp_set_bits(mcasp->base + OMAP_MCASP_TXDITCTL_REG, DITEN);

	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG, 0xFF);

	return 0;
}

static int omap_mcasp_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	mcasp_stop_tx(mcasp);

	if ((params_format(params)) != SNDRV_PCM_FORMAT_S16_LE) {
		printk(KERN_WARNING "omap-mcasp: unsupported PCM format");
		return -EINVAL;
	}

	if (omap_hw_dit_param(mcasp, params_rate(params)) < 0)
		return -EPERM;

	snd_soc_dai_set_dma_data(dai, substream,
				 &omap_mcasp_dai_dma_params[stream]);

	return 0;
}

static int omap_mcasp_trigger(struct snd_pcm_substream *substream,
				     int cmd, struct snd_soc_dai *cpu_dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		omap_mcasp_start(mcasp, substream->stream);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		omap_mcasp_stop(mcasp, substream->stream);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static struct snd_soc_dai_ops omap_mcasp_dai_ops = {
	.startup	= omap_mcasp_startup,
	.shutdown	= omap_mcasp_shutdown,
	.trigger	= omap_mcasp_trigger,
	.hw_params	= omap_mcasp_hw_params,

};

static struct snd_soc_dai_driver omap_mcasp_dai[] = {
	{
		.name		= "omap-mcasp-dai",
		.playback	= {
			.channels_min	= 1,
			.channels_max	= 384,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops		= &omap_mcasp_dai_ops,
	},
};

static __devinit int omap_mcasp_probe(struct platform_device *pdev)
{
	struct omap_mcasp_platform_data *pdata;
	struct omap_mcasp *mcasp;
	struct omap_hwmod *oh;
	int i, ret = 0;

	oh = omap_hwmod_lookup("omap-mcasp-dai");
	if (oh == NULL) {
		dev_err(&pdev->dev, "no hwmod device found\n");
		return -ENODEV;
	}

	mcasp = kzalloc(sizeof(struct omap_mcasp), GFP_KERNEL);
	if (!mcasp)
		return	-ENOMEM;

	mcasp->base = omap_hwmod_get_mpu_rt_va(oh);
	if (!mcasp->base) {
		ret = -ENODEV;
		goto err;
	}

	mcasp->fclk = clk_get(&pdev->dev, "mcasp_fck");
	if (!mcasp->fclk) {
		ret = -ENODEV;
		goto err;
	}

	pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, mcasp);

	mcasp->dev = &pdev->dev;
	mcasp->fclk_rate = pdata->mcasp_fclk_rate;
	mcasp->configs = pdata->mcasp_configs;
	mcasp->num_configs = pdata->num_configs;

	for (i = 0; i < mcasp->num_configs; i++)
		omap_mcasp_dai[0].playback.rates |=
				mcasp->configs[i].sampling_rate;

	ret = snd_soc_register_dai(&pdev->dev, omap_mcasp_dai);

	if (ret < 0)
		goto err;

	pm_runtime_enable(&pdev->dev);

	return 0;
err:
	kfree(mcasp);

	return ret;
}

static __devexit int omap_mcasp_remove(struct platform_device *pdev)
{
	struct omap_mcasp *mcasp = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	clk_put(mcasp->fclk);

	kfree(mcasp);

	return 0;
}

static struct platform_driver omap_mcasp_driver = {
	.probe		= omap_mcasp_probe,
	.remove		= omap_mcasp_remove,
	.driver		= {
		.name	= "omap-mcasp-dai",
		.owner	= THIS_MODULE,
	},
};

static int __init omap_mcasp_init(void)
{
	return platform_driver_register(&omap_mcasp_driver);
}
module_init(omap_mcasp_init);

static void __exit omap_mcasp_exit(void)
{
	platform_driver_unregister(&omap_mcasp_driver);
}
module_exit(omap_mcasp_exit);

MODULE_AUTHOR("Jon Hunter <jon-hunter@ti.com>");
MODULE_DESCRIPTION("TI OMAP McASP SoC Interface");
MODULE_LICENSE("GPL");
