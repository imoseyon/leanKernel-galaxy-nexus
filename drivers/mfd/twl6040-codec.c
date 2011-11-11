/*
 * MFD driver for twl6040 codec submodule
 *
 * Authors:     Jorge Eduardo Candelaria <jorge.candelaria@ti.com>
 *              Misael Lopez Cruz <misael.lopez@ti.com>
 *
 * Copyright:   (C) 20010 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/core.h>
#include <linux/mfd/twl6040-codec.h>

int twl6040_reg_read(struct twl6040 *twl6040, unsigned int reg)
{
	int ret;
	u8 val;

	mutex_lock(&twl6040->io_mutex);
	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, reg);
	if (ret < 0) {
		mutex_unlock(&twl6040->io_mutex);
		return ret;
	}
	mutex_unlock(&twl6040->io_mutex);

	return val;
}
EXPORT_SYMBOL(twl6040_reg_read);

int twl6040_reg_write(struct twl6040 *twl6040, unsigned int reg, u8 val)
{
	int ret;

	mutex_lock(&twl6040->io_mutex);
	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, reg);
	mutex_unlock(&twl6040->io_mutex);

	return ret;
}
EXPORT_SYMBOL(twl6040_reg_write);

int twl6040_set_bits(struct twl6040 *twl6040, unsigned int reg, u8 mask)
{
	int ret;
	u8 val;

	mutex_lock(&twl6040->io_mutex);
	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, reg);
	if (ret)
		goto out;

	val |= mask;
	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, reg);
out:
	mutex_unlock(&twl6040->io_mutex);
	return ret;
}
EXPORT_SYMBOL(twl6040_set_bits);

int twl6040_clear_bits(struct twl6040 *twl6040, unsigned int reg, u8 mask)
{
	int ret;
	u8 val;

	mutex_lock(&twl6040->io_mutex);
	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, reg);
	if (ret)
		goto out;

	val &= ~mask;
	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, reg);
out:
	mutex_unlock(&twl6040->io_mutex);
	return ret;
}
EXPORT_SYMBOL(twl6040_clear_bits);

/* twl6040 codec manual power-up sequence */
static int twl6040_power_up(struct twl6040 *twl6040)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;
	int ret;

	ncpctl = twl6040_reg_read(twl6040, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_reg_read(twl6040, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_reg_read(twl6040, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_reg_read(twl6040, TWL6040_REG_ACCCTL);

	/* enable reference system */
	ldoctl |= TWL6040_REFENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		return ret;
	msleep(10);

	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto osc_err;
	udelay(10);

	/* enable high-side ldo */
	ldoctl |= TWL6040_HSLDOENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto hsldo_err;
	udelay(244);

	/* enable negative charge pump */
	ncpctl |= TWL6040_NCPENA | TWL6040_NCPOPEN;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_NCPCTL, ncpctl);
	if (ret)
		goto ncp_err;
	udelay(488);

	/* enable low-side ldo */
	ldoctl |= TWL6040_LSLDOENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto lsldo_err;
	udelay(244);

	/* enable low-power pll */
	lppllctl |= TWL6040_LPLLENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
	if (ret)
		goto lppll_err;

	/* reset state machine */
	accctl |= TWL6040_RESETSPLIT;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_ACCCTL, accctl);
	if (ret)
		goto rst_err;
	mdelay(5);
	accctl &= ~TWL6040_RESETSPLIT;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_ACCCTL, accctl);
	if (ret)
		goto rst_err;

	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto rst_err;

	return 0;

rst_err:
	lppllctl &= ~TWL6040_LPLLENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
lppll_err:
	ldoctl &= ~TWL6040_LSLDOENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
lsldo_err:
	ncpctl &= ~(TWL6040_NCPENA | TWL6040_NCPOPEN);
	twl6040_reg_write(twl6040, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
ncp_err:
	ldoctl &= ~TWL6040_HSLDOENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
hsldo_err:
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
osc_err:
	ldoctl &= ~TWL6040_REFENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);

	return ret;
}

/* twl6040 codec manual power-down sequence */
static int twl6040_power_down(struct twl6040 *twl6040)
{
	u8 ncpctl, ldoctl, lppllctl, accctl;
	int ret;

	ncpctl = twl6040_reg_read(twl6040, TWL6040_REG_NCPCTL);
	ldoctl = twl6040_reg_read(twl6040, TWL6040_REG_LDOCTL);
	lppllctl = twl6040_reg_read(twl6040, TWL6040_REG_LPPLLCTL);
	accctl = twl6040_reg_read(twl6040, TWL6040_REG_ACCCTL);

	/* enable internal oscillator */
	ldoctl |= TWL6040_OSCENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		return ret;
	udelay(10);

	/* disable low-power pll */
	lppllctl &= ~TWL6040_LPLLENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
	if (ret)
		goto lppll_err;

	/* disable low-side ldo */
	ldoctl &= ~TWL6040_LSLDOENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto lsldo_err;
	udelay(244);

	/* disable negative charge pump */
	ncpctl &= ~(TWL6040_NCPENA | TWL6040_NCPOPEN);
	ret = twl6040_reg_write(twl6040, TWL6040_REG_NCPCTL, ncpctl);
	if (ret)
		goto ncp_err;
	udelay(488);

	/* disable high-side ldo */
	ldoctl &= ~TWL6040_HSLDOENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto hsldo_err;
	udelay(244);

	/* disable internal oscillator */
	ldoctl &= ~TWL6040_OSCENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto osc_err;

	/* disable reference system */
	ldoctl &= ~TWL6040_REFENA;
	ret = twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	if (ret)
		goto ref_err;
	msleep(10);

	return 0;

ref_err:
	ldoctl |= TWL6040_OSCENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	udelay(10);
osc_err:
	ldoctl |= TWL6040_HSLDOENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
hsldo_err:
	ncpctl |= TWL6040_NCPENA | TWL6040_NCPOPEN;
	twl6040_reg_write(twl6040, TWL6040_REG_NCPCTL, ncpctl);
	udelay(488);
ncp_err:
	ldoctl |= TWL6040_LSLDOENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	udelay(244);
lsldo_err:
	lppllctl |= TWL6040_LPLLENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
lppll_err:
	lppllctl |= TWL6040_LPLLENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
	accctl |= TWL6040_RESETSPLIT;
	twl6040_reg_write(twl6040, TWL6040_REG_ACCCTL, accctl);
	mdelay(5);
	accctl &= ~TWL6040_RESETSPLIT;
	twl6040_reg_write(twl6040, TWL6040_REG_ACCCTL, accctl);
	ldoctl &= ~TWL6040_OSCENA;
	twl6040_reg_write(twl6040, TWL6040_REG_LDOCTL, ldoctl);
	msleep(10);

	return ret;
}

static irqreturn_t twl6040_naudint_handler(int irq, void *data)
{
	struct twl6040 *twl6040 = data;
	u8 intid;

	intid = twl6040_reg_read(twl6040, TWL6040_REG_INTID);

	if (intid & TWL6040_READYINT)
		complete(&twl6040->ready);

	return IRQ_HANDLED;
}

static int twl6040_power_up_completion(struct twl6040 *twl6040,
				       int naudint)
{
	int time_left;
	int round = 0;
	int ret = 0;
	int retry = 0;
	u8 intid;
	u8 ncpctl;
	u8 ldoctl;
	u8 lppllctl;
	u8 ncpctl_exp;
	u8 ldoctl_exp;
	u8 lppllctl_exp;

	/* NCPCTL expected value: NCP enabled */
	ncpctl_exp = (TWL6040_TSHUTENA | TWL6040_NCPENA);

	/* LDOCTL expected value: HS/LS LDOs and Reference enabled */
	ldoctl_exp = (TWL6040_REFENA | TWL6040_HSLDOENA | TWL6040_LSLDOENA);

	/* LPPLLCTL expected value: Low-Power PLL enabled */
	lppllctl_exp = TWL6040_LPLLENA;

	do {
		gpio_set_value(twl6040->audpwron, 1);
		time_left = wait_for_completion_timeout(&twl6040->ready,
							msecs_to_jiffies(700));
		if (!time_left) {
			intid = twl6040_reg_read(twl6040, TWL6040_REG_INTID);
			if (!(intid & TWL6040_READYINT)) {
				dev_err(twl6040->dev,
					"timeout waiting for READYINT\n");
				return -ETIMEDOUT;
			}
		}
		/*
		 * Power on seemingly completed.
		 * Look for clues that the twl6040 might be still booting.
		 */

		retry = 0;
		ncpctl = twl6040_reg_read(twl6040, TWL6040_REG_NCPCTL);
		if (ncpctl != ncpctl_exp)
			retry++;

		ldoctl = twl6040_reg_read(twl6040, TWL6040_REG_LDOCTL);
		if (ldoctl != ldoctl_exp)
			retry++;

		lppllctl = twl6040_reg_read(twl6040, TWL6040_REG_LPPLLCTL);
		if (lppllctl != lppllctl_exp)
			retry++;

		if (retry) {
			dev_err(twl6040->dev,
				"NCPCTL: 0x%02x (should be 0x%02x)\n"
				"LDOCTL: 0x%02x (should be 0x%02x)\n"
				"LPLLCTL: 0x%02x (should be 0x%02x)\n",
				ncpctl, ncpctl_exp,
				ldoctl, ldoctl_exp,
				lppllctl, lppllctl_exp);
			round++;
			gpio_set_value(twl6040->audpwron, 0);
			usleep_range(1000, 1500);
			continue;
		}
	} while (round && (round < 3));

	if (round >= 3) {
		dev_err(twl6040->dev,
			"Automatic power on failed, reverting to manual\n");
		twl6040->audpwron = -EINVAL;
		ret = twl6040_power_up(twl6040);
		if (ret)
			dev_err(twl6040->dev, "Manual power-up failed\n");
	}

	return ret;
}

static int twl6040_power(struct twl6040 *twl6040, int enable)
{
	struct twl4030_codec_data *pdata = dev_get_platdata(twl6040->dev);
	int audpwron = twl6040->audpwron;
	int naudint = twl6040->irq;
	int ret = 0;

	if (enable) {
		/* enable 32kHz external clock */
		if (pdata->set_ext_clk32k) {
			ret = pdata->set_ext_clk32k(true);
			if (ret) {
				dev_err(twl6040->dev,
					"failed to enable CLK32K %d\n", ret);
				return ret;
			}
		}

		/* disable internal 32kHz oscillator */
		twl6040_clear_bits(twl6040, TWL6040_REG_ACCCTL,
				TWL6040_CLK32KSEL);

		if (gpio_is_valid(audpwron)) {
			/* wait for power-up completion */
			ret = twl6040_power_up_completion(twl6040, naudint);
			if (ret) {
				dev_err(twl6040->dev,
					"automatic power-down failed\n");
				return ret;
			}
		} else {
			/* use manual power-up sequence */
			ret = twl6040_power_up(twl6040);
			if (ret) {
				dev_err(twl6040->dev,
					"manual power-up failed\n");
				return ret;
			}
		}
		twl6040->pll = TWL6040_LPPLL_ID;
		twl6040->sysclk = 19200000;
	} else {
		if (gpio_is_valid(audpwron)) {
			/* use AUDPWRON line */
			gpio_set_value(audpwron, 0);

			/* power-down sequence latency */
			udelay(500);
		} else {
			/* use manual power-down sequence */
			ret = twl6040_power_down(twl6040);
			if (ret) {
				dev_err(twl6040->dev,
					"manual power-down failed\n");
				return ret;
			}
		}

		/* enable internal 32kHz oscillator */
		twl6040_set_bits(twl6040, TWL6040_REG_ACCCTL,
				TWL6040_CLK32KSEL);

		/* disable 32kHz external clock */
		if (pdata->set_ext_clk32k) {
			ret = pdata->set_ext_clk32k(false);
			if (ret)
				dev_err(twl6040->dev,
					"failed to disable CLK32K %d\n", ret);
		}

		twl6040->pll = TWL6040_NOPLL_ID;
		twl6040->sysclk = 0;
	}

	twl6040->powered = enable;

	return ret;
}

int twl6040_enable(struct twl6040 *twl6040)
{
	int ret = 0;

	mutex_lock(&twl6040->mutex);
	if (!twl6040->power_count++)
		ret = twl6040_power(twl6040, 1);
	mutex_unlock(&twl6040->mutex);

	return ret;
}
EXPORT_SYMBOL(twl6040_enable);

int twl6040_disable(struct twl6040 *twl6040)
{
	int ret = 0;

	mutex_lock(&twl6040->mutex);
	WARN(!twl6040->power_count, "TWL6040 is already disabled");
	if (!--twl6040->power_count)
		ret = twl6040_power(twl6040, 0);
	mutex_unlock(&twl6040->mutex);

	return ret;
}
EXPORT_SYMBOL(twl6040_disable);

int twl6040_is_enabled(struct twl6040 *twl6040)
{
	return twl6040->power_count;
}
EXPORT_SYMBOL(twl6040_is_enabled);

int twl6040_set_pll(struct twl6040 *twl6040, enum twl6040_pll_id id,
		    unsigned int freq_in, unsigned int freq_out)
{
	u8 hppllctl, lppllctl;
	int ret = 0;

	mutex_lock(&twl6040->mutex);

	hppllctl = twl6040_reg_read(twl6040, TWL6040_REG_HPPLLCTL);
	lppllctl = twl6040_reg_read(twl6040, TWL6040_REG_LPPLLCTL);

	switch (id) {
	case TWL6040_LPPLL_ID:
		/* lppll divider */
		switch (freq_out) {
		case 17640000:
			lppllctl |= TWL6040_LPLLFIN;
			break;
		case 19200000:
			lppllctl &= ~TWL6040_LPLLFIN;
			break;
		default:
			dev_err(twl6040->dev,
				"freq_out %d not supported\n", freq_out);
			ret = -EINVAL;
			goto pll_out;
		}
		twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);

		switch (freq_in) {
		case 32768:
			lppllctl |= TWL6040_LPLLENA;
			twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL,
					  lppllctl);
			mdelay(5);
			lppllctl &= ~TWL6040_HPLLSEL;
			twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL,
					  lppllctl);
			hppllctl &= ~TWL6040_HPLLENA;
			twl6040_reg_write(twl6040, TWL6040_REG_HPPLLCTL,
					  hppllctl);
			break;
		default:
			dev_err(twl6040->dev,
				"freq_in %d not supported\n", freq_in);
			ret = -EINVAL;
			goto pll_out;
		}

		twl6040->pll = TWL6040_LPPLL_ID;
		break;
	case TWL6040_HPPLL_ID:
		/* high-performance pll can provide only 19.2 MHz */
		if (freq_out != 19200000) {
			dev_err(twl6040->dev,
				"freq_out %d not supported\n", freq_out);
			ret = -EINVAL;
			goto pll_out;
		}

		hppllctl &= ~TWL6040_MCLK_MSK;

		switch (freq_in) {
		case 12000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_12000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 19200000:
			/* mclk input, pll disabled */
			hppllctl |= TWL6040_MCLK_19200KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		case 26000000:
			/* mclk input, pll enabled */
			hppllctl |= TWL6040_MCLK_26000KHZ |
				    TWL6040_HPLLSQRBP |
				    TWL6040_HPLLENA;
			break;
		case 38400000:
			/* clk slicer, pll disabled */
			hppllctl |= TWL6040_MCLK_38400KHZ |
				    TWL6040_HPLLSQRENA |
				    TWL6040_HPLLBP;
			break;
		default:
			dev_err(twl6040->dev,
				"freq_in %d not supported\n", freq_in);
			ret = -EINVAL;
			goto pll_out;
		}

		twl6040_reg_write(twl6040, TWL6040_REG_HPPLLCTL, hppllctl);
		udelay(500);
		lppllctl |= TWL6040_HPLLSEL;
		twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);
		lppllctl &= ~TWL6040_LPLLENA;
		twl6040_reg_write(twl6040, TWL6040_REG_LPPLLCTL, lppllctl);

		twl6040->pll = TWL6040_HPPLL_ID;
		break;
	default:
		dev_err(twl6040->dev, "unknown pll id %d\n", id);
		ret = -EINVAL;
		goto pll_out;
	}

	twl6040->sysclk = freq_out;

pll_out:
	mutex_unlock(&twl6040->mutex);
	return ret;
}
EXPORT_SYMBOL(twl6040_set_pll);

enum twl6040_pll_id twl6040_get_pll(struct twl6040 *twl6040)
{
	return twl6040->pll;
}
EXPORT_SYMBOL(twl6040_get_pll);

unsigned int twl6040_get_sysclk(struct twl6040 *twl6040)
{
	return twl6040->sysclk;
}
EXPORT_SYMBOL(twl6040_get_sysclk);

int twl6040_get_icrev(struct twl6040 *twl6040)
{
	return twl6040->icrev;
}
EXPORT_SYMBOL(twl6040_get_icrev);

static int __devinit twl6040_probe(struct platform_device *pdev)
{
	struct twl4030_codec_data *pdata = pdev->dev.platform_data;
	struct twl6040 *twl6040;
	struct mfd_cell *cell = NULL;
	unsigned int naudint;
	int audpwron;
	int ret, children = 0;
	u8 accctl;

	if(!pdata) {
		dev_err(&pdev->dev, "Platform data is missing\n");
		return -EINVAL;
	}

	twl6040 = kzalloc(sizeof(struct twl6040), GFP_KERNEL);
	if (!twl6040)
		return -ENOMEM;

	platform_set_drvdata(pdev, twl6040);

	twl6040->dev = &pdev->dev;
	mutex_init(&twl6040->mutex);
	mutex_init(&twl6040->io_mutex);

	twl6040->icrev = twl6040_reg_read(twl6040, TWL6040_REG_ASICREV);
	if (twl6040->icrev < 0) {
		ret = twl6040->icrev;
		goto gpio1_err;
	}

	if (pdata && (twl6040_get_icrev(twl6040) > TWL6040_REV_1_0))
		audpwron = pdata->audpwron_gpio;
	else
		audpwron = -EINVAL;

	if (pdata)
		naudint = pdata->naudint_irq;
	else
		naudint = 0;

	twl6040->audpwron = audpwron;
	twl6040->powered = 0;
	twl6040->irq = naudint;
	twl6040->irq_base = pdata->irq_base;
	init_completion(&twl6040->ready);

	if (gpio_is_valid(audpwron)) {
		ret = gpio_request(audpwron, "audpwron");
		if (ret)
			goto gpio1_err;

		ret = gpio_direction_output(audpwron, 0);
		if (ret)
			goto gpio2_err;
	}

	if (naudint) {
		/* codec interrupt */
		ret = twl6040_irq_init(twl6040);
		if (ret)
			goto gpio2_err;

		ret = twl6040_request_irq(twl6040, TWL6040_IRQ_READY,
				  twl6040_naudint_handler, "twl6040_irq_ready",
				  twl6040);
		if (ret) {
			dev_err(twl6040->dev, "READY IRQ request failed: %d\n",
				ret);
			goto irq_err;
		}
	}

	/* dual-access registers controlled by I2C only */
	accctl = twl6040_reg_read(twl6040, TWL6040_REG_ACCCTL);
	twl6040_reg_write(twl6040, TWL6040_REG_ACCCTL, accctl | TWL6040_I2CSEL);

	if (pdata->get_ext_clk32k) {
		ret = pdata->get_ext_clk32k();
		if (ret) {
			dev_err(twl6040->dev,
				"failed to get external 32kHz clock %d\n",
				ret);
			goto clk32k_err;
		}
	}

	if (pdata->audio) {
		cell = &twl6040->cells[children];
		cell->name = "twl6040-codec";
		cell->platform_data = pdata->audio;
		cell->pdata_size = sizeof(*pdata->audio);
		children++;
	}

	if (pdata->vibra) {
		cell = &twl6040->cells[children];
		cell->name = "twl6040-vibra";
		cell->platform_data = pdata->vibra;
		cell->pdata_size = sizeof(*pdata->vibra);
		children++;
	}

	if (children) {
		ret = mfd_add_devices(&pdev->dev, pdev->id, twl6040->cells,
				      children, NULL, 0);
		if (ret)
			goto mfd_err;
	} else {
		dev_err(&pdev->dev, "No platform data found for children\n");
		ret = -ENODEV;
		goto mfd_err;
	}

	return 0;

mfd_err:
	if (pdata->put_ext_clk32k)
		pdata->put_ext_clk32k();
clk32k_err:
	if (naudint)
		twl6040_free_irq(twl6040, TWL6040_IRQ_READY, twl6040);
irq_err:
	if (naudint)
		twl6040_irq_exit(twl6040);
gpio2_err:
	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);
gpio1_err:
	platform_set_drvdata(pdev, NULL);
	kfree(twl6040);
	return ret;
}

static int __devexit twl6040_remove(struct platform_device *pdev)
{
	struct twl6040 *twl6040 = platform_get_drvdata(pdev);
	struct twl4030_codec_data *pdata = dev_get_platdata(twl6040->dev);
	int audpwron = twl6040->audpwron;
	int naudint = twl6040->irq;

	twl6040_disable(twl6040);

	twl6040_free_irq(twl6040, TWL6040_IRQ_READY, twl6040);

	if (gpio_is_valid(audpwron))
		gpio_free(audpwron);

	if (naudint)
		twl6040_irq_exit(twl6040);

	mfd_remove_devices(&pdev->dev);

	if (pdata->put_ext_clk32k)
		pdata->put_ext_clk32k();

	platform_set_drvdata(pdev, NULL);
	kfree(twl6040);

	return 0;
}

static struct platform_driver twl6040_driver = {
	.probe		= twl6040_probe,
	.remove		= __devexit_p(twl6040_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "twl6040-audio",
	},
};

static int __devinit twl6040_init(void)
{
	return platform_driver_register(&twl6040_driver);
}
module_init(twl6040_init);

static void __devexit twl6040_exit(void)
{
	platform_driver_unregister(&twl6040_driver);
}

module_exit(twl6040_exit);

MODULE_DESCRIPTION("TWL6040 MFD");
MODULE_AUTHOR("Jorge Eduardo Candelaria <jorge.candelaria@ti.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6040-audio");
