/*
 * MFD driver for twl6040 codec submodule
 *
 * Authors:     Jorge Eduardo Candelaria <jorge.candelaria@ti.com>
 *              Misael Lopez Cruz <misael.lopez@ti.com>
 *
 * Copyright:   (C) 2011 Texas Instruments, Inc.
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

#ifndef __TWL6040_CODEC_H__
#define __TWL6040_CODEC_H__

#include <linux/interrupt.h>
#include <linux/mfd/core.h>

#define TWL6040_NO_SUPPLY		0
#define TWL6040_VIO_SUPPLY		1
#define TWL6040_VDD_SUPPLY		2


#define TWL6040_REG_ASICID		0x01
#define TWL6040_REG_ASICREV		0x02
#define TWL6040_REG_INTID		0x03
#define TWL6040_REG_INTMR		0x04
#define TWL6040_REG_NCPCTL		0x05
#define TWL6040_REG_LDOCTL		0x06
#define TWL6040_REG_HPPLLCTL		0x07
#define TWL6040_REG_LPPLLCTL		0x08
#define TWL6040_REG_LPPLLDIV		0x09
#define TWL6040_REG_AMICBCTL		0x0A
#define TWL6040_REG_DMICBCTL		0x0B
#define TWL6040_REG_MICLCTL		0x0C
#define TWL6040_REG_MICRCTL		0x0D
#define TWL6040_REG_MICGAIN		0x0E
#define TWL6040_REG_LINEGAIN		0x0F
#define TWL6040_REG_HSLCTL		0x10
#define TWL6040_REG_HSRCTL		0x11
#define TWL6040_REG_HSGAIN		0x12
#define TWL6040_REG_EARCTL		0x13
#define TWL6040_REG_HFLCTL		0x14
#define TWL6040_REG_HFLGAIN		0x15
#define TWL6040_REG_HFRCTL		0x16
#define TWL6040_REG_HFRGAIN		0x17
#define TWL6040_REG_VIBCTLL		0x18
#define TWL6040_REG_VIBDATL		0x19
#define TWL6040_REG_VIBCTLR		0x1A
#define TWL6040_REG_VIBDATR		0x1B
#define TWL6040_REG_HKCTL1		0x1C
#define TWL6040_REG_HKCTL2		0x1D
#define TWL6040_REG_GPOCTL		0x1E
#define TWL6040_REG_ALB			0x1F
#define TWL6040_REG_DLB			0x20
#define TWL6040_REG_TRIM1		0x28
#define TWL6040_REG_TRIM2		0x29
#define TWL6040_REG_TRIM3		0x2A
#define TWL6040_REG_HSOTRIM		0x2B
#define TWL6040_REG_HFOTRIM		0x2C
#define TWL6040_REG_ACCCTL		0x2D
#define TWL6040_REG_STATUS		0x2E

#define TWL6040_CACHEREGNUM		(TWL6040_REG_STATUS + 1)

#define TWL6040_VIOREGNUM		18
#define TWL6040_VDDREGNUM		21

/* ASICREV (0x02) values */

#define TWL6040_REV_1_0			0x00
#define TWL6040_REV_1_1			0x01
#define TWL6040_REV_1_3			0x02

/* INTID (0x03) fields */

#define TWL6040_THINT			0x01
#define TWL6040_PLUGINT			0x02
#define TWL6040_UNPLUGINT		0x04
#define TWL6040_HOOKINT			0x08
#define TWL6040_HFINT			0x10
#define TWL6040_VIBINT			0x20
#define TWL6040_READYINT		0x40

/* INTMR (0x04) fields */

#define TWL6040_THMSK			0x01
#define TWL6040_PLUGMSK			0x02
#define TWL6040_HOOKMSK			0x08
#define TWL6040_HFMSK			0x10
#define TWL6040_VIBMSK			0x20
#define TWL6040_READYMSK		0x40
#define TWL6040_ALLINT_MSK		0x7B

/* NCPCTL (0x05) fields */

#define TWL6040_NCPENA			0x01
#define TWL6040_NCPOPEN			0x40
#define TWL6040_TSHUTENA		0x80

/* LDOCTL (0x06) fields */

#define TWL6040_LSLDOENA		0x01
#define TWL6040_HSLDOENA		0x04
#define TWL6040_REFENA			0x40
#define TWL6040_OSCENA			0x80

/* HPPLLCTL (0x07) fields */

#define TWL6040_HPLLENA			0x01
#define TWL6040_HPLLRST			0x02
#define TWL6040_HPLLBP			0x04
#define TWL6040_HPLLSQRENA		0x08
#define TWL6040_HPLLSQRBP		0x10
#define TWL6040_MCLK_12000KHZ		(0 << 5)
#define TWL6040_MCLK_19200KHZ		(1 << 5)
#define TWL6040_MCLK_26000KHZ		(2 << 5)
#define TWL6040_MCLK_38400KHZ		(3 << 5)
#define TWL6040_MCLK_MSK		0x60

/* LPPLLCTL (0x08) fields */

#define TWL6040_LPLLENA			0x01
#define TWL6040_LPLLRST			0x02
#define TWL6040_LPLLSEL			0x04
#define TWL6040_LPLLFIN			0x08
#define TWL6040_HPLLSEL			0x10

/* HSLCTL (0x10) fields */

#define TWL6040_HSDACMODEL		0x02
#define TWL6040_HSDRVMODEL		0x08

/* HSRCTL (0x11) fields */

#define TWL6040_HSDACMODER		0x02
#define TWL6040_HSDRVMODER		0x08

/* VIBCTLL (0x18) fields */

#define TWL6040_VIBCTRLLN		0x10
#define TWL6040_VIBCTRLLP		0x04
#define TWL6040_VIBENAL			0x01

/* VIBCTLL (0x19) fields */

#define TWL6040_VIBCTRLRN		0x10
#define TWL6040_VIBCTRLRP		0x04
#define TWL6040_VIBENAR			0x01

/* GPOCTL (0x1E) fields */

#define TWL6040_GPO1			0x01
#define TWL6040_GPO2			0x02
#define TWL6040_GPO3			0x03

/* HSOTRIM (0x2B) fields */

#define TWL6040_HSLO			0x0F
#define TWL6040_HSRO			0xF0
#define TWL6040_HSLO_OFFSET		0
#define TWL6040_HSRO_OFFSET		4

/* HFOTRIM (0x2C) fields */

#define TWL6040_HFLO			0x0F
#define TWL6040_HFRO			0xF0
#define TWL6040_HFLO_OFFSET		0
#define TWL6040_HFRO_OFFSET		4

/* ACCCTL (0x2D) fields */

#define TWL6040_I2CSEL			0x01
#define TWL6040_RESETSPLIT		0x04
#define TWL6040_INTCLRMODE		0x08
#define TWL6040_CLK32KSEL		0x40

/* STATUS (0x2E) fields */

#define TWL6040_PLUGCOMP		0x02

#define TWL6040_CELLS			2

#define TWL6040_IRQ_TH			0
#define TWL6040_IRQ_PLUG		1
#define TWL6040_IRQ_HOOK		2
#define TWL6040_IRQ_HF			3
#define TWL6040_IRQ_VIB			4
#define TWL6040_IRQ_READY		5

enum twl6040_pll_id {
	TWL6040_NOPLL_ID,
	TWL6040_LPPLL_ID,
	TWL6040_HPPLL_ID,
};

struct twl6040 {
	struct device *dev;
	struct mutex mutex;
	struct mutex io_mutex;
	struct mutex irq_mutex;
	struct mfd_cell cells[TWL6040_CELLS];
	struct completion ready;

	int audpwron;
	int powered;
	int power_count;

	enum twl6040_pll_id pll;
	unsigned int sysclk;
	int icrev;

	unsigned int irq;
	unsigned int irq_base;
	u8 irq_masks_cur;
	u8 irq_masks_cache;
};

static inline int twl6040_request_irq(struct twl6040 *twl6040, int irq,
				      irq_handler_t handler, const char *name,
				      void *data)
{
	if (!twl6040->irq_base)
		return -EINVAL;

	return request_threaded_irq(twl6040->irq_base + irq, NULL, handler,
				    0, name, data);
}

static inline void twl6040_free_irq(struct twl6040 *twl6040, int irq,
				    void *data)
{
	if (!twl6040->irq_base)
		return;

	free_irq(twl6040->irq_base + irq, data);
}

int twl6040_reg_read(struct twl6040 *twl6040, unsigned int reg);
int twl6040_reg_write(struct twl6040 *twl6040, unsigned int reg,
		      u8 val);
int twl6040_set_bits(struct twl6040 *twl6040, unsigned int reg,
		     u8 mask);
int twl6040_clear_bits(struct twl6040 *twl6040, unsigned int reg,
		       u8 mask);
int twl6040_enable(struct twl6040 *twl6040);
int twl6040_disable(struct twl6040 *twl6040);
int twl6040_is_enabled(struct twl6040 *twl6040);
int twl6040_set_pll(struct twl6040 *twl6040, enum twl6040_pll_id id,
		    unsigned int freq_in, unsigned int freq_out);
enum twl6040_pll_id twl6040_get_pll(struct twl6040 *twl6040);
unsigned int twl6040_get_sysclk(struct twl6040 *twl6040);
int twl6040_get_icrev(struct twl6040 *twl6040);
int twl6040_irq_init(struct twl6040 *twl6040);
void twl6040_irq_exit(struct twl6040 *twl6040);

#endif  /* End of __TWL6040_CODEC_H__ */
