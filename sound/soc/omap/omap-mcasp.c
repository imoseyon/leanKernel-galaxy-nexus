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
#include <linux/interrupt.h>
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
 * OMAP_MCASP_EVTCTLX_REG - Transmitter Interrupt Control Register bits
 */
#define EVTCTLX_XUNDRN		BIT(0)

/*
 * OMAP_MCASP_TXSTAT_REG - Transmit Status Register Bits
 */
#define TXSTAT_XUNDRN	(0x1 << 0)
#define TXSTAT_XSYNCERR	(0x1 << 1)
#define TXSTAT_XCKFAIL	(0x1 << 2)
#define TXSTAT_XDMSLOT	(0x1 << 3)
#define TXSTAT_XLAST	(0x1 << 4)
#define TXSTAT_XDATA	(0x1 << 5)
#define TXSTAT_XSTAFRM	(0x1 << 6)
#define TXSTAT_XDMAERR	(0x1 << 7)
#define TXSTAT_XERR	(0x1 << 8)

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

#define MCASP_ALLOWED_PPM	100

/*
 * OMAP_MCASP_DITCSRA_REG/OMAP_MCASP_DITCSRB_REG
 */
#define OMAP_MCASP_DITCSR_44100HZ	(0x0 << 24)
#define OMAP_MCASP_DITCSR_48000HZ	(0x2 << 24)
#define OMAP_MCASP_DITCSR_32000HZ	(0x3 << 24)
#define OMAP_MCASP_DITCSR_22050HZ	(0x4 << 24)
#define OMAP_MCASP_DITCSR_24000HZ	(0x6 << 24)
#define OMAP_MCASP_DITCSR_88200HZ	(0x8 << 24)
#define OMAP_MCASP_DITCSR_96000HZ	(0xA << 24)
#define OMAP_MCASP_DITCSR_176400HZ	(0xC << 24)
#define OMAP_MCASP_DITCSR_192000HZ	(0xE << 24)

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

static int mcasp_compute_clock_dividers(long fclk_rate, int tgt_sample_rate,
			int *out_div_lo, int *out_div_hi)
{
	/* Given a particular functional clock rate and a target audio sample
	 * rate, determine the proper values for the ACLKXCTL and AHCLKXCTL, the
	 * dividers which produce the high frequency transmit master clock and
	 * the transmit clock.
	 */
	long divisor;
	unsigned long ppm;
	int sample_rate, i;
	BUG_ON(!out_div_lo);
	BUG_ON(!out_div_hi);

	/* A single S/PDIF frame requires 128 clocks */
	divisor = DIV_ROUND_CLOSEST(fclk_rate, tgt_sample_rate << 7);
	if (!divisor)
		return -EINVAL;

	sample_rate = (fclk_rate >> 7) / divisor;

	/* ppm calculation in two steps to avoid overflow */
	ppm = abs(tgt_sample_rate - sample_rate);
	ppm = (1000000 * ppm) / tgt_sample_rate;

	if (ppm > MCASP_ALLOWED_PPM)
		return -EINVAL;

	/* At this point, divisor holds the product of the two divider values we
	 * need to use for ACLKXCTL and AHCLKXCTL.  ACLKXCTL holds a 5 bit
	 * divider [1, 32], while AHCLKXCTL holds a 12 bit divider [1, 4096].
	 * We need to make sure that we can factor divisor into two integers
	 * which will fit into these divider registers.  Find the largest 5-bit
	 * + 1 value which divides divisor and use that as our smaller divider.
	 * After removing this factor from divisor, if the result is <= 4096,
	 * then we have succeeded and will be able to produce the target sample
	 * rate.
	 */
	for (i = 32; (i > 1) && (divisor % i); --i)
		; /* no body */

	/* Make sure to subtract one, registers hold the value of the divider
	 * minus one (IOW, to divide by 5, the register gets programmed with the
	 * value 4. */
	*out_div_lo = i - 1;
	*out_div_hi = (divisor / i) - 1;

	return (*out_div_hi <= 4096) ? 0 : -EINVAL;
}

static int omap_mcasp_start(struct omap_mcasp *mcasp)
{
	int i;
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXHCLKRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXCLKRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXSERCLR);

	/* Wait until the DMA has loaded the first sample into TXBUF before we
	 * let the TX state machine and frame sync generator out of reset. */
	i = 0;
	while (1) {
		u32 reg = mcasp_get_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG);
		if (!(reg & TXSTAT_XDATA))
			break;

		if (++i > 1000) {
			printk(KERN_ERR "Timeout waiting for DMA to load first"
					" sample of audio.\n");
			return -ETIMEDOUT;
		}

		udelay(1);
	}

	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXSMRST);
	mcasp_set_ctl_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, TXFSRST);
	mcasp_clr_bits(mcasp->base + OMAP_MCASP_TXEVTCTL_REG, TXDATADMADIS);

	/* enable IRQ sources */
	mcasp_set_bits(mcasp->base + OMAP_MCASP_EVTCTLX_REG, EVTCTLX_XUNDRN);

	return 0;
}

static void omap_mcasp_stop(struct omap_mcasp *mcasp)
{
	/* disable IRQ sources */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_EVTCTLX_REG, 0);

	mcasp_set_reg(mcasp->base + OMAP_MCASP_GBLCTL_REG, 0);
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG,
			OMAP_MCASP_TXSTAT_MASK);
}

/* S/PDIF */
static int omap_mcasp_setup(struct omap_mcasp *mcasp, unsigned int rate)
{
	u32 aclkxdiv, ahclkxdiv, ditcsr;
	int res;

	/* Set TX frame synch : DIT Mode, 1 bit width, internal, rising edge */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXFMCTL_REG,
						AFSXE | FSXMOD(0x180));

	/* Set the TX clock controls : div = 1 and internal */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_ACLKXCTL_REG,
						ACLKXE | TX_ASYNC);

	/* Set the HS TX clock controls : div = 1 and internal */
	mcasp_set_reg(mcasp->base + OMAP_MCASP_AHCLKXCTL_REG, AHCLKXE);

	/* The SPDIF bit clock is derived from the McASP functional clock.
	 * The McASP has two programmable clock dividers (aclkxdiv and
	 * ahclkxdiv) that are configured via the registers MCASP_ACLKXCTL
	 * and MCASP_AHCLKXCTL. For SPDIF the bit clock frequency should be
	 * 128 * sample rate freq. The dividers are defined as part of
	 * platform data as they are dependent upon the functional clock
	 * setting. Lookup the appropriate dividers for the sampling
	 * frequency that we are playing.
	 */
	res = mcasp_compute_clock_dividers(clk_get_rate(mcasp->fclk),
				rate,
				&aclkxdiv,
				&ahclkxdiv);
	if (res) {
		dev_err(mcasp->dev,
			"%s: No valid McASP config for sampling rate (%d)!\n",
			__func__, rate);
		return res;
	}

	switch (rate) {
	case 22050:
		ditcsr = OMAP_MCASP_DITCSR_22050HZ;
		break;
	case 24000:
		ditcsr = OMAP_MCASP_DITCSR_24000HZ;
		break;
	case 32000:
		ditcsr = OMAP_MCASP_DITCSR_32000HZ;
		break;
	case 44100:
		ditcsr = OMAP_MCASP_DITCSR_44100HZ;
		break;
	case 48000:
		ditcsr = OMAP_MCASP_DITCSR_48000HZ;
		break;
	case 88200:
		ditcsr = OMAP_MCASP_DITCSR_88200HZ;
		break;
	case 96000:
		ditcsr = OMAP_MCASP_DITCSR_96000HZ;
		break;
	case 176400:
		ditcsr = OMAP_MCASP_DITCSR_176400HZ;
		break;
	case 192000:
		ditcsr = OMAP_MCASP_DITCSR_192000HZ;
		break;
	default:
		dev_err(mcasp->dev, "%s: Invalid sampling rate: %d\n",
			__func__, rate);
		return -EINVAL;
	}
	mcasp_set_reg(mcasp->base + OMAP_MCASP_DITCSRA_REG, ditcsr);
	mcasp_set_reg(mcasp->base + OMAP_MCASP_DITCSRB_REG, ditcsr);
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

static irqreturn_t omap_mcasp_irq_handler(int irq, void *data)
{
	struct omap_mcasp *mcasp = data;
	u32 txstat;

	txstat = mcasp_get_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG);
	if (txstat & TXSTAT_XUNDRN) {
		dev_err(mcasp->dev, "%s: Underrun (0x%08x)\n", __func__,
			txstat);

		/* Try to recover from this state */
		spin_lock(&mcasp->lock);
		if (likely(mcasp->stream_rate)) {
			dev_err(mcasp->dev, "%s: Trying to recover\n",
				__func__);
			omap_mcasp_stop(mcasp);
			omap_mcasp_setup(mcasp, mcasp->stream_rate);
			omap_mcasp_start(mcasp);
		}
		spin_unlock(&mcasp->lock);
	}

	mcasp_set_reg(mcasp->base + OMAP_MCASP_TXSTAT_REG, txstat);

	return IRQ_HANDLED;
}

static int omap_mcasp_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);

	/* HACK: Only allow C2 state */
	pm_qos_add_request(mcasp->pm_qos, PM_QOS_CPU_DMA_LATENCY, 1150);

	pm_runtime_get_sync(mcasp->dev);

	return 0;
}

static void omap_mcasp_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);

	pm_runtime_put_sync(mcasp->dev);

	/* HACK: remove qos */
	pm_qos_remove_request(mcasp->pm_qos);
}

static int omap_mcasp_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	omap_mcasp_stop(mcasp);

	if (omap_mcasp_setup(mcasp, params_rate(params)) < 0)
		return -EPERM;

	snd_soc_dai_set_dma_data(dai, substream,
				 &omap_mcasp_dai_dma_params[stream]);

	return 0;
}

static int omap_mcasp_trigger(struct snd_pcm_substream *substream,
				     int cmd, struct snd_soc_dai *cpu_dai)
{
	struct omap_mcasp *mcasp = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mcasp->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mcasp->stream_rate = substream->runtime->rate;
		ret = omap_mcasp_start(mcasp);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mcasp->stream_rate = 0;
		omap_mcasp_stop(mcasp);
		break;

	default:
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&mcasp->lock, flags);

	return ret;
}

static struct snd_soc_dai_ops omap_mcasp_dai_ops = {
	.startup	= omap_mcasp_startup,
	.shutdown	= omap_mcasp_shutdown,
	.trigger	= omap_mcasp_trigger,
	.hw_params	= omap_mcasp_hw_params,

};

#define MCASP_RATES	(SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
			 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | \
			 SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)

static struct snd_soc_dai_driver omap_mcasp_dai[] = {
	{
		.name		= "omap-mcasp-dai",
		.playback	= {
			.channels_min	= 1,
			.channels_max	= 384,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE,
			.rates		= MCASP_RATES,
		},
		.ops		= &omap_mcasp_dai_ops,
	},
};

static __devinit int omap_mcasp_probe(struct platform_device *pdev)
{
	struct omap_mcasp *mcasp;
	struct resource *res;
	long fclk_rate;
	int ret = 0;

	mcasp = kzalloc(sizeof(struct omap_mcasp), GFP_KERNEL);
	if (!mcasp)
		return -ENOMEM;

	spin_lock_init(&mcasp->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no resource\n");
		ret = -ENODEV;
		goto err_res;
	}

	mcasp->base = ioremap(res->start, resource_size(res));
	if (!mcasp->base) {
		ret = -ENOMEM;
		goto err_res;
	}

	mcasp->irq = platform_get_irq(pdev, 0);
	if (mcasp->irq < 0) {
		ret = mcasp->irq;
		goto err_irq;
	}

	ret = request_threaded_irq(mcasp->irq, NULL, omap_mcasp_irq_handler,
				0, "McASP", mcasp);
	if (ret) {
		dev_err(mcasp->dev, "IRQ request failed\n");
		goto err_irq;
	}

	mcasp->fclk = clk_get(&pdev->dev, "mcasp_fck");
	if (!mcasp->fclk) {
		ret = -ENODEV;
		goto err_clk;
	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	fclk_rate = clk_get_rate(mcasp->fclk);

	platform_set_drvdata(pdev, mcasp);
	mcasp->dev = &pdev->dev;

	ret = snd_soc_register_dai(&pdev->dev, omap_mcasp_dai);
	if (ret < 0)
		goto err_dai;

	/* HACK: qos */
	mcasp->pm_qos = kzalloc(sizeof(struct pm_qos_request_list), GFP_KERNEL);
	if (!mcasp->pm_qos) {
		ret = -ENOMEM;
		goto err_dai;
	}

	pm_runtime_put_sync(&pdev->dev);

	return 0;

err_dai:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_clk:
	free_irq(mcasp->irq, (void *)mcasp);
err_irq:
	iounmap(mcasp->base);
err_res:
	kfree(mcasp);
	return ret;
}

static __devexit int omap_mcasp_remove(struct platform_device *pdev)
{
	struct omap_mcasp *mcasp = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_put(mcasp->fclk);
	free_irq(mcasp->irq, (void *)mcasp);
	iounmap(mcasp->base);
	/* HACK: qos */
	kfree(mcasp->pm_qos);
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
