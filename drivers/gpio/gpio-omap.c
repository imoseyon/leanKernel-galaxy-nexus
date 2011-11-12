/*
 * Support functions for OMAP GPIO
 *
 * Copyright (C) 2003-2005 Nokia Corporation
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/bitops.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <asm/mach/irq.h>
#include <plat/omap-pm.h>

#include "../mux.h"

static LIST_HEAD(omap_gpio_list);

struct gpio_regs {
	u32 irqenable1;
	u32 irqenable2;
	u32 wake_en;
	u32 ctrl;
	u32 oe;
	u32 leveldetect0;
	u32 leveldetect1;
	u32 risingdetect;
	u32 fallingdetect;
	u32 dataout;
	u32 debounce;
	u32 debounce_en;
	u32 edge_falling;
	u32 edge_rising;

	u32 ew_leveldetect0;
	u32 ew_leveldetect1;

	u32 pad_set_wakeupenable;
};

struct gpio_bank {
	struct list_head node;
	unsigned long pbase;
	void __iomem *base;
	u16 irq;
	u16 virtual_irq_start;
	u32 suspend_wakeup;
	u32 saved_wakeup;
	u32 non_wakeup_gpios;
	u32 enabled_non_wakeup_gpios;
	struct gpio_regs context;
	u32 saved_datain;
	u32 saved_fallingdetect;
	u32 saved_risingdetect;
	u32 level_mask;
	u32 toggle_mask;
	spinlock_t lock;
	struct gpio_chip chip;
	struct clk *dbck;
	u32 mod_usage;
	u32 dbck_enable_mask;
	struct device *dev;
	bool is_mpuio;
	bool dbck_flag;
	bool loses_context;
	bool suspend_support;
	bool saved_context;
	int stride;
	u32 width;
	u16 id;

	u32 type_leveldetect0;
	u32 type_leveldetect1;
	u32 type_risingedge;
	u32 type_fallingedge;

	void (*set_dataout)(struct gpio_bank *bank, int gpio, int enable);
	struct omap_gpio_reg_offs *regs;

	struct omap_mux *mux[32];
};

static void omap_gpio_mod_init(struct gpio_bank *bank);

#define GPIO_INDEX(bank, gpio) (gpio % bank->width)
#define GPIO_BIT(bank, gpio) (1 << GPIO_INDEX(bank, gpio))
#define GPIO_MOD_CTRL_BIT	BIT(0)
static void _set_gpio_waken(struct gpio_bank *bank, int gpio)
{
	if (bank->regs->wkup_set != bank->regs->wkup_clear) {
		__raw_writel((1 << gpio), bank->base + bank->regs->wkup_set);
	} else {
		u32 val =  __raw_readl(bank->base + bank->regs->wkup_set);
		val |= 1 << gpio;
		__raw_writel(val, bank->base + bank->regs->wkup_set);
	}
}
static void _clear_gpio_waken(struct gpio_bank *bank, int gpio)
{
	if (bank->regs->wkup_set != bank->regs->wkup_clear) {
		__raw_writel((1 << gpio), bank->base + bank->regs->wkup_clear);
	} else {
		u32 val =  __raw_readl(bank->base + bank->regs->wkup_clear);
		val &= ~(1 << gpio);
		__raw_writel(val, bank->base + bank->regs->wkup_clear);
	}

}

static void _set_gpio_direction(struct gpio_bank *bank, int gpio, int is_input)
{
	void __iomem *reg = bank->base;
	u32 l;

	reg += bank->regs->direction;
	l = __raw_readl(reg);
	if (is_input)
		l |= 1 << gpio;
	else
		l &= ~(1 << gpio);
	__raw_writel(l, reg);
}


/* set data out value using dedicate set/clear register */
static void _set_gpio_dataout_reg(struct gpio_bank *bank, int gpio, int enable)
{
	void __iomem *reg = bank->base;
	u32 l = GPIO_BIT(bank, gpio);

	if (enable)
		reg += bank->regs->set_dataout;
	else
		reg += bank->regs->clr_dataout;

	__raw_writel(l, reg);
}

/* set data out value using mask register */
static void _set_gpio_dataout_mask(struct gpio_bank *bank, int gpio, int enable)
{
	void __iomem *reg = bank->base + bank->regs->dataout;
	u32 gpio_bit = GPIO_BIT(bank, gpio);
	u32 l;

	l = __raw_readl(reg);
	if (enable)
		l |= gpio_bit;
	else
		l &= ~gpio_bit;
	__raw_writel(l, reg);
}

static int _get_gpio_datain(struct gpio_bank *bank, int gpio)
{
	void __iomem *reg = bank->base + bank->regs->datain;

	return (__raw_readl(reg) & GPIO_BIT(bank, gpio)) != 0;
}

static int _get_gpio_dataout(struct gpio_bank *bank, int gpio)
{
	void __iomem *reg = bank->base + bank->regs->dataout;

	return (__raw_readl(reg) & GPIO_BIT(bank, gpio)) != 0;
}

#define MOD_REG_BIT(reg, bit_mask, set)	\
do {	\
	int l = __raw_readl(base + reg); \
	if (set) l |= bit_mask; \
	else l &= ~bit_mask; \
	__raw_writel(l, base + reg); \
} while(0)

/**
 * _set_gpio_debounce - low level gpio debounce time
 * @bank: the gpio bank we're acting upon
 * @gpio: the gpio number on this @gpio
 * @debounce: debounce time to use
 *
 * OMAP's debounce time is in 31us steps so we need
 * to convert and round up to the closest unit.
 */
static void _set_gpio_debounce(struct gpio_bank *bank, unsigned gpio,
		unsigned debounce)
{
	void __iomem		*reg;
	u32			val;
	u32			l;

	if (!bank->dbck_flag)
		return;

	if (debounce < 32)
		debounce = 0x01;
	else if (debounce > 7936)
		debounce = 0xff;
	else
		debounce = (debounce / 0x1f) - 1;

	l = GPIO_BIT(bank, gpio);

	reg = bank->base + bank->regs->debounce;
	__raw_writel(debounce, reg);

	reg = bank->base + bank->regs->debounce_en;
	val = __raw_readl(reg);

	if (debounce) {
		val |= l;
		clk_enable(bank->dbck);
	} else {
		val &= ~l;
		clk_disable(bank->dbck);
	}
	bank->dbck_enable_mask = val;

	__raw_writel(val, reg);
}

static inline void set_gpio_trigger(struct gpio_bank *bank, int gpio,
						int trigger)
{
	void __iomem *base = bank->base;
	u32 gpio_bit = 1 << gpio;

	MOD_REG_BIT(bank->regs->leveldetect0, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_LOW);
	MOD_REG_BIT(bank->regs->leveldetect1, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_HIGH);
	MOD_REG_BIT(bank->regs->risingdetect, gpio_bit,
			trigger & IRQ_TYPE_EDGE_RISING);
	MOD_REG_BIT(bank->regs->fallingdetect, gpio_bit,
			trigger & IRQ_TYPE_EDGE_FALLING);

	if (likely(!(bank->non_wakeup_gpios & gpio_bit))) {
		if (cpu_is_omap44xx()) {
			MOD_REG_BIT(bank->regs->wkup_status, gpio_bit,
				trigger != 0);
		} else {
			/*
			 * GPIO wakeup request can only be generated on edge
			 * transitions
			 */
			if (trigger & IRQ_TYPE_EDGE_BOTH)
				_set_gpio_waken(bank, gpio);
			else
				_clear_gpio_waken(bank, gpio);
		}
	}
	/* This part needs to be executed always for OMAP{34xx, 44xx} */
	if (cpu_is_omap34xx() || cpu_is_omap44xx() ||
			(bank->non_wakeup_gpios & gpio_bit)) {
		/*
		 * Log the edge gpio and manually trigger the IRQ
		 * after resume if the input level changes
		 * to avoid irq lost during PER RET/OFF mode
		 * Applies for omap2 non-wakeup gpio and all omap3 gpios
		 */
		if (trigger & IRQ_TYPE_EDGE_BOTH)
			bank->enabled_non_wakeup_gpios |= gpio_bit;
		else
			bank->enabled_non_wakeup_gpios &= ~gpio_bit;
	}

	bank->level_mask =
		__raw_readl(bank->base + bank->regs->leveldetect0) |
		__raw_readl(bank->base + bank->regs->leveldetect1);
}

#ifdef CONFIG_ARCH_OMAP1
/*
 * This only applies to chips that can't do both rising and falling edge
 * detection at once.  For all other chips, this function is a noop.
 */
static void _toggle_gpio_edge_triggering(struct gpio_bank *bank, int gpio)
{
	void __iomem *reg = bank->base;
	u32 l = 0;

	if (!bank->regs->irqctrl)
		return;

	reg += bank->regs->irqctrl;

	l = __raw_readl(reg);
	if ((l >> gpio) & 1)
		l &= ~(1 << gpio);
	else
		l |= 1 << gpio;

	__raw_writel(l, reg);
}
#endif

static int _set_gpio_triggering(struct gpio_bank *bank, int gpio, int trigger)
{
	void __iomem *reg = bank->base;
	u32 l = 0;

	if (bank->regs->leveldetect0 && bank->regs->wkup_status) {
		set_gpio_trigger(bank, gpio, trigger);
	} else if (bank->regs->irqctrl) {
		reg += bank->regs->irqctrl;

		l = __raw_readl(reg);
		if ((trigger & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_BOTH)
			bank->toggle_mask |= 1 << gpio;
		if (trigger & IRQ_TYPE_EDGE_RISING)
			l |= 1 << gpio;
		else if (trigger & IRQ_TYPE_EDGE_FALLING)
			l &= ~(1 << gpio);
		else
			return -EINVAL;

		__raw_writel(l, reg);

	} else if (bank->regs->edgectrl1) {
		if (gpio & 0x08)
			reg += bank->regs->edgectrl2;
		else
			reg += bank->regs->edgectrl1;

		gpio &= 0x07;
		l = __raw_readl(reg);
		l &= ~(3 << (gpio << 1));
		if (trigger & IRQ_TYPE_EDGE_RISING)
			l |= 2 << (gpio << 1);
		if (trigger & IRQ_TYPE_EDGE_FALLING)
			l |= 1 << (gpio << 1);

		if (trigger)
			/* Enable wake-up during idle for dynamic tick */
			_set_gpio_waken(bank, gpio);
		else
			_clear_gpio_waken(bank, gpio);

		__raw_writel(l, reg);
	}

	return 0;
}

static int gpio_irq_type(struct irq_data *d, unsigned type)
{
	struct gpio_bank *bank;
	unsigned gpio;
	int retval;
	unsigned long flags;

	if (!cpu_class_is_omap2() && d->irq > IH_MPUIO_BASE)
		gpio = OMAP_MPUIO(d->irq - IH_MPUIO_BASE);
	else
		gpio = d->irq - IH_GPIO_BASE;

	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	bank = irq_data_get_irq_chip_data(d);

	/* OMAP1 allows only edge triggering */
	if (!bank->regs->leveldetect0 && (type &
			(IRQ_TYPE_LEVEL_LOW|IRQ_TYPE_LEVEL_HIGH)))
		return -EINVAL;

	spin_lock_irqsave(&bank->lock, flags);

	retval = _set_gpio_triggering(bank, GPIO_INDEX(bank, gpio), type);

	bank->type_leveldetect0 &= ~GPIO_BIT(bank, gpio);
	bank->type_leveldetect1 &= ~GPIO_BIT(bank, gpio);
	bank->type_fallingedge &= ~GPIO_BIT(bank, gpio);
	bank->type_risingedge &= ~GPIO_BIT(bank, gpio);

	if (type & IRQ_TYPE_LEVEL_LOW)
		bank->type_leveldetect0 |= GPIO_BIT(bank, gpio);
	if (type & IRQ_TYPE_LEVEL_HIGH)
		bank->type_leveldetect1 |= GPIO_BIT(bank, gpio);
	if (type & IRQ_TYPE_EDGE_FALLING)
		bank->type_fallingedge |= GPIO_BIT(bank, gpio);
	if (type & IRQ_TYPE_EDGE_RISING)
		bank->type_risingedge |= GPIO_BIT(bank, gpio);

	spin_unlock_irqrestore(&bank->lock, flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	return retval;
}

static void _clear_gpio_irqbank(struct gpio_bank *bank, int gpio_mask)
{
	void __iomem *reg = bank->base;

	reg += bank->regs->irqstatus;
	__raw_writel(gpio_mask, reg);

	/* Workaround for clearing DSP GPIO interrupts to allow retention */
	if (bank->regs->irqstatus2) {
		reg = bank->base + bank->regs->irqstatus2;
		__raw_writel(gpio_mask, reg);
	}

	/* Flush posted write for the irq status to avoid spurious interrupts */
	__raw_readl(reg);
}

static inline void _clear_gpio_irqstatus(struct gpio_bank *bank, int gpio)
{
	_clear_gpio_irqbank(bank, GPIO_BIT(bank, gpio));
}

static u32 _get_gpio_irqbank_mask(struct gpio_bank *bank)
{
	void __iomem *reg = bank->base;
	u32 l;
	u32 mask = (1 << bank->width) - 1;

	reg += bank->regs->irqenable;
	l = __raw_readl(reg);
	if (bank->regs->irqenable_inv)
		l = ~l;
	l &= mask;
	return l;
}

static void _enable_gpio_irqbank(struct gpio_bank *bank, int gpio_mask)
{
	void __iomem *reg = bank->base;
	u32 l;

	if (bank->regs->set_irqenable) {
		reg += bank->regs->set_irqenable;
		l = gpio_mask;
	} else {
		reg += bank->regs->irqenable;
		l = __raw_readl(reg);
		if (bank->regs->irqenable_inv)
			l &= ~gpio_mask;
		else
			l |= gpio_mask;
	}

	__raw_writel(l, reg);
}

static void _disable_gpio_irqbank(struct gpio_bank *bank, int gpio_mask)
{
	void __iomem *reg = bank->base;
	u32 l;

	if (bank->regs->clr_irqenable) {
		reg += bank->regs->clr_irqenable;
		l = gpio_mask;
	} else {
		reg += bank->regs->irqenable;
		l = __raw_readl(reg);
		if (bank->regs->irqenable_inv)
			l |= gpio_mask;
		else
			l &= ~gpio_mask;
	}

	__raw_writel(l, reg);
}

static inline void _set_gpio_irqenable(struct gpio_bank *bank, int gpio, int enable)
{
	_enable_gpio_irqbank(bank, GPIO_BIT(bank, gpio));
}

/*
 * Note that ENAWAKEUP needs to be enabled in GPIO_SYSCONFIG register.
 * 1510 does not seem to have a wake-up register. If JTAG is connected
 * to the target, system will wake up always on GPIO events. While
 * system is running all registered GPIO interrupts need to have wake-up
 * enabled. When system is suspended, only selected GPIO interrupts need
 * to have wake-up enabled.
 */
static int _set_gpio_wakeup(struct gpio_bank *bank, int gpio, int enable)
{
	u32 gpio_bit = GPIO_BIT(bank, gpio);
	unsigned long flags;

	if (bank->non_wakeup_gpios & gpio_bit) {
		dev_err(bank->dev,
			"Unable to modify wakeup on non-wakeup GPIO%d\n", gpio);
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lock, flags);
	if (enable)
		bank->suspend_wakeup |= gpio_bit;
	else
		bank->suspend_wakeup &= ~gpio_bit;

	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static void _reset_gpio(struct gpio_bank *bank, int gpio)
{
	_set_gpio_direction(bank, GPIO_INDEX(bank, gpio), 1);
	_set_gpio_irqenable(bank, gpio, 0);
	_clear_gpio_irqstatus(bank, gpio);
	_set_gpio_triggering(bank, GPIO_INDEX(bank, gpio), IRQ_TYPE_NONE);
}

/* Use disable_irq_wake() and enable_irq_wake() functions from drivers */
static int gpio_wake_enable(struct irq_data *d, unsigned int enable)
{
	unsigned int gpio = d->irq - IH_GPIO_BASE;
	struct gpio_bank *bank;
	int retval;

	bank = irq_data_get_irq_chip_data(d);
	retval = _set_gpio_wakeup(bank, gpio, enable);

	return retval;
}

static int omap_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	/*
	 * If this is the first gpio_request for the bank,
	 * enable the bank module.
	 */
	if (!bank->mod_usage) {
		if (pm_runtime_get_sync(bank->dev) < 0) {
			dev_err(bank->dev, "%s: GPIO bank %d "
					"pm_runtime_get_sync failed\n",
					__func__, bank->id);
			spin_unlock_irqrestore(&bank->lock, flags);
			return -EINVAL;
		}

		/* Initialize the gpio bank registers to init time value */
		omap_gpio_mod_init(bank);
	}

	/* Set trigger to none. You need to enable the desired trigger with
	 * request_irq() or set_irq_type().
	 */
	_set_gpio_triggering(bank, offset, IRQ_TYPE_NONE);

	if (bank->regs->pinctrl) {
		void __iomem *reg = bank->base + bank->regs->pinctrl;

		/* Claim the pin for MPU */
		__raw_writel(__raw_readl(reg) | (1 << offset), reg);
	}

	if (bank->regs->ctrl && !bank->mod_usage) {
		void __iomem *reg = bank->base + bank->regs->ctrl;
		u32 ctrl;

		ctrl = __raw_readl(reg);
		/* Module is enabled, clocks are not gated */
		ctrl &= ~GPIO_MOD_CTRL_BIT;
		__raw_writel(ctrl, reg);
	}

	bank->mod_usage |= 1 << offset;

	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static void omap_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);

	if (bank->regs->wkup_clear)
		/* Disable wake-up during idle for dynamic tick */
		_clear_gpio_waken(bank, offset);

	bank->mod_usage &= ~(1 << offset);

	if (bank->regs->ctrl && !bank->mod_usage) {
		void __iomem *reg = bank->base + bank->regs->ctrl;
		u32 ctrl;

		ctrl = __raw_readl(reg);
		/* Module is disabled, clocks are gated */
		ctrl |= GPIO_MOD_CTRL_BIT;
		__raw_writel(ctrl, reg);
	}

	_reset_gpio(bank, bank->chip.base + offset);

	/*
	 * If this is the last gpio to be freed in the bank,
	 * disable the bank module.
	 */
	if (!bank->mod_usage) {
		if (pm_runtime_put_sync_suspend(bank->dev)) {
			dev_err(bank->dev, "%s: GPIO bank %d "
					"pm_runtime_put_sync_suspend failed\n",
					__func__, bank->id);
		}
	}
	spin_unlock_irqrestore(&bank->lock, flags);
}

/*
 * We need to unmask the GPIO bank interrupt as soon as possible to
 * avoid missing GPIO interrupts for other lines in the bank.
 * Then we need to mask-read-clear-unmask the triggered GPIO lines
 * in the bank to avoid missing nested interrupts for a GPIO line.
 * If we wait to unmask individual GPIO lines in the bank after the
 * line's interrupt handler has been run, we may miss some nested
 * interrupts.
 */
static void gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *isr_reg = NULL;
	u32 isr;
	unsigned int gpio_irq, gpio_index;
	struct gpio_bank *bank;
	u32 retrigger = 0;
	int unmasked = 0;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	bank = irq_get_handler_data(irq);

	pm_runtime_get_sync(bank->dev);

	isr_reg = bank->base + bank->regs->irqstatus;

	if (WARN_ON(!isr_reg))
		goto exit;

	while(1) {
		u32 isr_saved, level_mask = 0;
		u32 enabled;

		enabled = _get_gpio_irqbank_mask(bank);

		if (bank->width == 32)
			isr = __raw_readl(isr_reg) & enabled;
		else if (bank->width == 16)
			isr = (__raw_readw(isr_reg) & enabled) & 0x0000ffff;
		isr_saved = isr;

		if (bank->regs->leveldetect0)
			level_mask = bank->level_mask & enabled;

		/* clear edge sensitive interrupts before handler(s) are
		called so that we don't miss any interrupt occurred while
		executing them */
		_disable_gpio_irqbank(bank, isr_saved & ~level_mask);
		_clear_gpio_irqbank(bank, isr_saved & ~level_mask);
		_enable_gpio_irqbank(bank, isr_saved & ~level_mask);

		/* if there is only edge sensitive GPIO pin interrupts
		configured, we could unmask GPIO bank interrupt immediately */
		if (!level_mask && !unmasked) {
			unmasked = 1;
			chained_irq_exit(chip, desc);
		}

		isr |= retrigger;
		retrigger = 0;
		if (!isr)
			break;

		gpio_irq = bank->virtual_irq_start;
		for (; isr != 0; isr >>= 1, gpio_irq++) {
			gpio_index = GPIO_INDEX(bank, irq_to_gpio(gpio_irq));

			if (!(isr & 1))
				continue;

#ifdef CONFIG_ARCH_OMAP1
			/*
			 * Some chips can't respond to both rising and falling
			 * at the same time.  If this irq was requested with
			 * both flags, we need to flip the ICR data for the IRQ
			 * to respond to the IRQ for the opposite direction.
			 * This will be indicated in the bank toggle_mask.
			 */
			if (bank->toggle_mask & (1 << gpio_index))
				_toggle_gpio_edge_triggering(bank, gpio_index);
#endif

			generic_handle_irq(gpio_irq);
		}
	}
	/* if bank has any level sensitive GPIO pin interrupt
	configured, we must unmask the bank interrupt only after
	handler(s) are executed in order to avoid spurious bank
	interrupt */
exit:
	if (!unmasked)
		chained_irq_exit(chip, desc);

	pm_runtime_put_sync_suspend(bank->dev);
}

static void gpio_irq_shutdown(struct irq_data *d)
{
	unsigned int gpio = d->irq - IH_GPIO_BASE;
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	_reset_gpio(bank, gpio);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void gpio_ack_irq(struct irq_data *d)
{
	unsigned int gpio = d->irq - IH_GPIO_BASE;
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);

	_clear_gpio_irqstatus(bank, gpio);
}

static void gpio_mask_irq(struct irq_data *d)
{
	unsigned int gpio = d->irq - IH_GPIO_BASE;
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_irqenable(bank, gpio, 0);
	_set_gpio_triggering(bank, GPIO_INDEX(bank, gpio), IRQ_TYPE_NONE);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void gpio_unmask_irq(struct irq_data *d)
{
	unsigned int gpio = d->irq - IH_GPIO_BASE;
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);
	unsigned int irq_mask = GPIO_BIT(bank, gpio);
	u32 trigger = irqd_get_trigger_type(d);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	if (trigger)
		_set_gpio_triggering(bank, GPIO_INDEX(bank, gpio), trigger);

	/* For level-triggered GPIOs, the clearing must be done after
	 * the HW source is cleared, thus after the handler has run */
	if (bank->level_mask & irq_mask) {
		_set_gpio_irqenable(bank, gpio, 0);
		_clear_gpio_irqstatus(bank, gpio);
	}

	_set_gpio_irqenable(bank, gpio, 1);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static struct irq_chip gpio_irq_chip = {
	.name		= "GPIO",
	.irq_shutdown	= gpio_irq_shutdown,
	.irq_ack	= gpio_ack_irq,
	.irq_mask	= gpio_mask_irq,
	.irq_unmask	= gpio_unmask_irq,
	.irq_set_type	= gpio_irq_type,
	.irq_set_wake	= gpio_wake_enable,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};

/*---------------------------------------------------------------------*/
static int omap_mpuio_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank	*bank = platform_get_drvdata(pdev);
	void __iomem		*mask_reg = bank->base +
					OMAP_MPUIO_GPIO_MASKIT / bank->stride;
	unsigned long		flags;

	spin_lock_irqsave(&bank->lock, flags);
	bank->saved_wakeup = __raw_readl(mask_reg);
	__raw_writel(0xffff & ~bank->suspend_wakeup, mask_reg);
	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static int omap_mpuio_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank	*bank = platform_get_drvdata(pdev);
	void __iomem		*mask_reg = bank->base +
					OMAP_MPUIO_GPIO_MASKIT / bank->stride;
	unsigned long		flags;

	spin_lock_irqsave(&bank->lock, flags);
	__raw_writel(bank->saved_wakeup, mask_reg);
	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static const struct dev_pm_ops omap_mpuio_dev_pm_ops = {
	.suspend_noirq = omap_mpuio_suspend_noirq,
	.resume_noirq = omap_mpuio_resume_noirq,
};

/* use platform_driver for this. */
static struct platform_driver omap_mpuio_driver = {
	.driver		= {
		.name	= "mpuio",
		.pm	= &omap_mpuio_dev_pm_ops,
	},
};

static struct platform_device omap_mpuio_device = {
	.name		= "mpuio",
	.id		= -1,
	.dev = {
		.driver = &omap_mpuio_driver.driver,
	}
	/* could list the /proc/iomem resources */
};

static inline void mpuio_init(struct gpio_bank *bank)
{
	platform_set_drvdata(&omap_mpuio_device, bank);

	if (platform_driver_register(&omap_mpuio_driver) == 0)
		(void) platform_device_register(&omap_mpuio_device);
}

/*---------------------------------------------------------------------*/

static int gpio_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_direction(bank, offset, 1);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static int gpio_is_input(struct gpio_bank *bank, int mask)
{
	void __iomem *reg = bank->base + bank->regs->direction;

	return __raw_readl(reg) & mask;
}

static int gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	void __iomem *reg;
	int gpio;
	u32 mask;

	gpio = chip->base + offset;
	bank = container_of(chip, struct gpio_bank, chip);
	reg = bank->base;
	mask = GPIO_BIT(bank, gpio);

	if (gpio_is_input(bank, mask))
		return _get_gpio_datain(bank, gpio);
	else
		return _get_gpio_dataout(bank, gpio);
}

static int gpio_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	bank->set_dataout(bank, offset, value);
	_set_gpio_direction(bank, offset, 0);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static int gpio_debounce(struct gpio_chip *chip, unsigned offset,
		unsigned debounce)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);

	if (!bank->dbck) {
		bank->dbck = clk_get(bank->dev, "dbclk");
		if (IS_ERR(bank->dbck))
			dev_err(bank->dev, "Could not get gpio dbck\n");
	}

	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_debounce(bank, offset, debounce);
	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static void gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	bank->set_dataout(bank, offset, value);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static int gpio_2irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;

	bank = container_of(chip, struct gpio_bank, chip);
	return bank->virtual_irq_start + offset;
}

/*---------------------------------------------------------------------*/

static void __init omap_gpio_show_rev(struct gpio_bank *bank)
{
	static bool called;
	u32 rev;

	if (called || bank->regs->revision == USHRT_MAX)
		return;

	rev = __raw_readw(bank->base + bank->regs->revision);
	pr_info("OMAP GPIO hardware version %d.%d\n",
		(rev >> 4) & 0x0f, rev & 0x0f);

	called = true;
}

/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static void omap_gpio_mod_init(struct gpio_bank *bank)
{
	int i;

	if (bank->width == 32) {
		u32 clr_all = 0;		/* clear all the bits */
		u32 set_all = 0xFFFFFFFF;	/* set all the bits */

		if (bank->is_mpuio) {
			__raw_writel(set_all, bank->base +
						bank->regs->irqenable);

			if (bank->suspend_support)
				mpuio_init(bank);

			return;
		}

		if (bank->regs->ctrl)
			/* Initialize interface clk ungated, module enabled */
			 __raw_writel(clr_all, bank->base + bank->regs->ctrl);

		if (bank->regs->clr_irqenable) {
			__raw_writel(set_all, bank->base +
						bank->regs->clr_irqenable);
		} else if (bank->regs->irqenable) {
			u32 i;

			if (bank->regs->irqenable_inv)
				i = set_all;
			else
				i = clr_all;

			__raw_writel(i, bank->base + bank->regs->irqenable);
		}

		if (bank->regs->irqstatus) {
			u32 i;

			if (bank->regs->irqenable_inv)
				i = clr_all;
			else
				i = set_all;

			__raw_writel(i, bank->base + bank->regs->irqstatus);
		}

		if (bank->regs->debounce_en)
			__raw_writel(clr_all, bank->base +
						bank->regs->debounce_en);

	} else if (bank->width == 16) {
		u16 clr_all = 0;	/* clear all the bits */
		u16 set_all = 0xFFFF;	/* set all the bits */

		if (bank->is_mpuio) {
			__raw_writew(set_all, bank->base +
						bank->regs->irqenable);

			if (bank->suspend_support)
				mpuio_init(bank);

			return;
		}

		if (bank->regs->irqenable) {
			u16 i;

			if (bank->regs->irqenable_inv)
				i = set_all;
			else
				i = clr_all;

			__raw_writew(i, bank->base + bank->regs->irqenable);
		}

		if (bank->regs->irqstatus) {
			u32 i;

			if (bank->regs->irqenable_inv)
				i = clr_all;
			else
				i = set_all;

			__raw_writew(i, bank->base + bank->regs->irqstatus);
		}

		if (bank->regs->sysconfig) {
			/* set wakeup-enable and smart-idle */
			__raw_writew(0x14, bank->base + bank->regs->sysconfig);

			/*
			 * Enable system clock for GPIO module.
			 * The CAM_CLK_CTRL *is* really the right place.
			 */
			omap_writel(omap_readl(ULPD_CAM_CLK_CTRL) | 0x04,
						ULPD_CAM_CLK_CTRL);
		}
	}

	for (i = 0; i < bank->width; i++) {
		int gpio = irq_to_gpio(bank->virtual_irq_start + i);
		bank->mux[i] = omap_mux_get_gpio(gpio);
	}
}

static __init void
omap_mpuio_alloc_gc(struct gpio_bank *bank, unsigned int irq_start,
		    unsigned int num)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;

	gc = irq_alloc_generic_chip("MPUIO", 1, irq_start, bank->base,
				    handle_simple_irq);
	if (!gc) {
		dev_err(bank->dev, "Memory alloc failed for gc\n");
		return;
	}

	ct = gc->chip_types;

	/* NOTE: No ack required, reading IRQ status clears it. */
	ct->chip.irq_mask = irq_gc_mask_set_bit;
	ct->chip.irq_unmask = irq_gc_mask_clr_bit;
	ct->chip.irq_set_type = gpio_irq_type;

	if (bank->suspend_support)
		ct->chip.irq_set_wake = gpio_wake_enable,

	ct->regs.mask = OMAP_MPUIO_GPIO_INT / bank->stride;
	irq_setup_generic_chip(gc, IRQ_MSK(num), IRQ_GC_INIT_MASK_CACHE,
			       IRQ_NOREQUEST | IRQ_NOPROBE, 0);
}

static void __devinit omap_gpio_chip_init(struct gpio_bank *bank)
{
	int j;
	static int gpio;

	bank->mod_usage = 0;
	/*
	 * REVISIT eventually switch from OMAP-specific gpio structs
	 * over to the generic ones
	 */
	bank->chip.request = omap_gpio_request;
	bank->chip.free = omap_gpio_free;
	bank->chip.direction_input = gpio_input;
	bank->chip.get = gpio_get;
	bank->chip.direction_output = gpio_output;
	bank->chip.set_debounce = gpio_debounce;
	bank->chip.set = gpio_set;
	bank->chip.to_irq = gpio_2irq;
	if (bank->is_mpuio) {
		bank->chip.label = "mpuio";
		if (bank->suspend_support)
			bank->chip.dev = &omap_mpuio_device.dev;
		bank->chip.base = OMAP_MPUIO(0);
	} else {
		bank->chip.label = "gpio";
		bank->chip.base = gpio;
		gpio += bank->width;
	}
	bank->chip.ngpio = bank->width;

	gpiochip_add(&bank->chip);

	for (j = bank->virtual_irq_start;
		     j < bank->virtual_irq_start + bank->width; j++) {
		irq_set_lockdep_class(j, &gpio_lock_class);
		irq_set_chip_data(j, bank);
		if (bank->is_mpuio) {
			omap_mpuio_alloc_gc(bank, j, bank->width);
		} else {
			irq_set_chip(j, &gpio_irq_chip);
			irq_set_handler(j, handle_simple_irq);
			set_irq_flags(j, IRQF_VALID);
		}
	}
	irq_set_chained_handler(bank->irq, gpio_irq_handler);
	irq_set_handler_data(bank->irq, bank);
}

static int __devinit omap_gpio_probe(struct platform_device *pdev)
{
	struct omap_gpio_platform_data *pdata;
	struct resource *res;
	struct gpio_bank *bank;
	int ret = 0;

	if (!pdev->dev.platform_data) {
		ret = -EINVAL;
		goto err_exit;
	}

	bank = kzalloc(sizeof(struct gpio_bank), GFP_KERNEL);
	if (!bank) {
		dev_err(&pdev->dev, "Memory alloc failed for gpio_bank\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(!res)) {
		dev_err(&pdev->dev, "GPIO Bank %i Invalid IRQ resource\n",
				pdev->id);
		ret = -ENODEV;
		goto err_free;
	}

	bank->irq = res->start;
	bank->id = pdev->id;

	pdata = pdev->dev.platform_data;
	bank->virtual_irq_start = pdata->virtual_irq_start;
	bank->dev = &pdev->dev;
	bank->dbck_flag = pdata->dbck_flag;
	bank->stride = pdata->bank_stride;
	bank->width = pdata->bank_width;
	bank->is_mpuio = pdata->is_mpuio;
	bank->suspend_support = pdata->suspend_support;
	bank->non_wakeup_gpios = pdata->non_wakeup_gpios;
	bank->loses_context = pdata->loses_context;
	bank->regs = pdata->regs;
	bank->saved_context = 0;
	if (bank->regs->set_dataout && bank->regs->clr_dataout)
		bank->set_dataout = _set_gpio_dataout_reg;
	else
		bank->set_dataout = _set_gpio_dataout_mask;

	spin_lock_init(&bank->lock);

	/* Static mapping, never released */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(&pdev->dev, "GPIO Bank %i Invalid mem resource\n",
				pdev->id);
		ret = -ENODEV;
		goto err_free;
	}

	bank->base = ioremap(res->start, resource_size(res));
	if (!bank->base) {
		dev_err(&pdev->dev, "Could not ioremap gpio bank%i\n",
				pdev->id);
		ret = -ENOMEM;
		goto err_free;
	}

	platform_set_drvdata(pdev, bank);

	pm_runtime_enable(bank->dev);
	pm_runtime_irq_safe(bank->dev);
	if (pm_runtime_get_sync(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_get_sync "
				"failed\n", __func__, bank->id);
		iounmap(bank->base);
		return -EINVAL;
	}

	omap_gpio_mod_init(bank);
	omap_gpio_chip_init(bank);
	omap_gpio_show_rev(bank);

	if (pm_runtime_put_sync(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_put_sync "
				"failed\n", __func__, bank->id);
		iounmap(bank->base);
		return -EINVAL;
	}

	list_add_tail(&bank->node, &omap_gpio_list);

	return ret;

err_free:
	kfree(bank);
err_exit:
	return ret;
}

static int omap_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank *bank = platform_get_drvdata(pdev);
	void __iomem *wake_status;
	void __iomem *wake_clear;
	void __iomem *wake_set;
	unsigned long flags;

	if (!bank->suspend_support)
		return 0;

	wake_status = bank->base + bank->regs->wkup_status;
	wake_clear = bank->base + bank->regs->wkup_clear;
	wake_set = bank->base + bank->regs->wkup_set;

	pm_runtime_get_sync(dev);

	spin_lock_irqsave(&bank->lock, flags);
	bank->saved_wakeup = __raw_readl(wake_status);
	__raw_writel(0xffffffff, wake_clear);
	__raw_writel(bank->suspend_wakeup, wake_set);
	spin_unlock_irqrestore(&bank->lock, flags);

	pm_runtime_put_sync(dev);

	return 0;
}

static int omap_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank *bank = platform_get_drvdata(pdev);
	void __iomem *wake_clear;
	void __iomem *wake_set;
	unsigned long flags;

	if (!bank->suspend_support)
		return 0;

	wake_clear = bank->base + bank->regs->wkup_clear;
	wake_set = bank->base + bank->regs->wkup_set;

	pm_runtime_get_sync(dev);

	spin_lock_irqsave(&bank->lock, flags);
	__raw_writel(0xffffffff, wake_clear);
	__raw_writel(bank->saved_wakeup, wake_set);
	spin_unlock_irqrestore(&bank->lock, flags);

	pm_runtime_put_sync(dev);

	return 0;
}

#ifdef CONFIG_ARCH_OMAP2PLUS
static void omap_gpio_save_context(struct gpio_bank *bank);
static void omap_gpio_restore_context(struct gpio_bank *bank);

static void omap2_gpio_set_wakeupenables(struct gpio_bank *bank)
{
	unsigned long pad_wakeup;
	int i;

	bank->context.pad_set_wakeupenable = 0;

	pad_wakeup = __raw_readl(bank->base + bank->regs->irqenable);

	for_each_set_bit(i, &pad_wakeup, bank->width) {
		if (!omap_mux_get_wakeupenable(bank->mux[i])) {
			bank->context.pad_set_wakeupenable |= BIT(i);
			omap_mux_set_wakeupenable(bank->mux[i]);
		}
	}
}

static void omap2_gpio_clear_wakeupenables(struct gpio_bank *bank)
{
	unsigned long pad_wakeup;
	int i;

	pad_wakeup = bank->context.pad_set_wakeupenable;

	for_each_set_bit(i, &pad_wakeup, bank->width)
		omap_mux_clear_wakeupenable(bank->mux[i]);
}

#endif

static int omap_gpio_pm_runtime_suspend(struct device *dev)
{
#ifdef CONFIG_ARCH_OMAP2PLUS
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank *bank = platform_get_drvdata(pdev);
	u32 l1 = 0, l2 = 0;
	int j;

	/* If going to OFF, remove triggering for all
	 * non-wakeup GPIOs.  Otherwise spurious IRQs will be
	 * generated.  See OMAP2420 Errata item 1.101. */
	if (!(bank->enabled_non_wakeup_gpios))
		goto save_gpio_ctx;

	bank->saved_datain = __raw_readl(bank->base +
						bank->regs->datain);
	l1 = __raw_readl(bank->base + bank->regs->fallingdetect);
	l2 = __raw_readl(bank->base + bank->regs->risingdetect);

	bank->saved_fallingdetect = l1;
	bank->saved_risingdetect = l2;
	l1 &= ~bank->enabled_non_wakeup_gpios;
	l2 &= ~bank->enabled_non_wakeup_gpios;

	__raw_writel(l1, bank->base + bank->regs->fallingdetect);
	__raw_writel(l2, bank->base + bank->regs->risingdetect);

save_gpio_ctx:
	omap_gpio_save_context(bank);
	for (j = 0; j < hweight_long(bank->dbck_enable_mask); j++)
		clk_disable(bank->dbck);

#endif
	return 0;
}

static int omap_gpio_pm_runtime_resume(struct device *dev)
{
#ifdef CONFIG_ARCH_OMAP2PLUS
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_bank *bank = platform_get_drvdata(pdev);
	u32 l = 0, gen, gen0, gen1;
	int j;

	for (j = 0; j < hweight_long(bank->dbck_enable_mask); j++)
		clk_enable(bank->dbck);

	if (omap_pm_was_context_lost(dev))
		omap_gpio_restore_context(bank);

	if (!(bank->enabled_non_wakeup_gpios))
		return 0;

	__raw_writel(bank->saved_fallingdetect,
			bank->base + bank->regs->fallingdetect);
	__raw_writel(bank->saved_risingdetect,
			bank->base + bank->regs->risingdetect);
	l = __raw_readl(bank->base + bank->regs->datain);

	/* Check if any of the non-wakeup interrupt GPIOs have changed
	 * state.  If so, generate an IRQ by software.  This is
	 * horribly racy, but it's the best we can do to work around
	 * this silicon bug. */
	l ^= bank->saved_datain;
	l &= bank->enabled_non_wakeup_gpios;

	/*
	 * No need to generate IRQs for the rising edge for gpio IRQs
	 * configured with falling edge only; and vice versa.
	 */
	gen0 = l & bank->saved_fallingdetect;
	gen0 &= bank->saved_datain;
	gen1 = l & bank->saved_risingdetect;
	gen1 &= ~(bank->saved_datain);

	/* FIXME: Consider GPIO IRQs with level detections properly! */
	gen = l & (~(bank->saved_fallingdetect) &
			~(bank->saved_risingdetect));
	/* Consider all GPIO IRQs needed to be updated */
	gen |= gen0 | gen1;

	if (gen) {
		u32 old0, old1;

		old0 = __raw_readl(bank->base +
					bank->regs->leveldetect0);
		old1 = __raw_readl(bank->base +
					bank->regs->leveldetect1);

		__raw_writel(old0, bank->base +
					bank->regs->leveldetect0);
		__raw_writel(old1, bank->base +
					bank->regs->leveldetect1);
		if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
			old0 |= gen;
			old1 |= gen;
		}

		if (cpu_is_omap44xx()) {
			old0 |= l;
			old1 |= l;
		}
		__raw_writel(old0, bank->base +
					bank->regs->leveldetect0);
		__raw_writel(old1, bank->base +
					bank->regs->leveldetect1);
	}
#endif
	return 0;
}

#ifdef CONFIG_ARCH_OMAP2PLUS
static int omap2_gpio_set_edge_wakeup(struct gpio_bank *bank, bool suspend)
{
	int ret = 0;
	u32 wkup_status = 0;
	u32 datain;
	u32 mask;
	u32 active;

	if (pm_runtime_get_sync(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_get_sync "
				"failed\n", __func__, bank->id);
		return -EINVAL;
	}

	bank->context.ew_leveldetect0 = __raw_readl(bank->base +
			bank->regs->leveldetect0);
	bank->context.ew_leveldetect1 = __raw_readl(bank->base +
			bank->regs->leveldetect1);
	wkup_status = __raw_readl(bank->base +
			bank->regs->wkup_status);
	bank->context.edge_falling = __raw_readl(bank->base +
			bank->regs->fallingdetect);
	bank->context.edge_rising = __raw_readl(bank->base +
			bank->regs->risingdetect);

	/*
	 * Set edge trigger for all gpio's that are
	 * expected to produce wakeup from low power.
	 * even if they are set for level detection only.
	 */
	__raw_writel(bank->context.edge_falling |
			(bank->context.ew_leveldetect0 & wkup_status),
		(bank->base + bank->regs->fallingdetect));
	__raw_writel(bank->context.edge_rising |
			(bank->context.ew_leveldetect1 & wkup_status),
		(bank->base + bank->regs->risingdetect));
	__raw_writel(0, bank->base + bank->regs->leveldetect0);
	__raw_writel(0, bank->base + bank->regs->leveldetect1);

	/*
	 * If a level interrupt is pending it will be lost since
	 * we just cleared it's enable bit.  Detect and abort,
	 * the interrupt will be delivered when
	 * omap2_gpio_restore_edge_wakeup restores the level
	 * interrupt mask.
	 */
	datain = __raw_readl(bank->base + bank->regs->datain);
	if (suspend)
		mask = bank->suspend_wakeup;
	else
		mask = wkup_status;

	active = (datain & bank->type_leveldetect1 & mask) |
		 (~datain & bank->type_leveldetect0 & mask);

	if (active) {
		if (suspend)
			pr_info("%s: aborted suspend due to gpio %d\n",
				__func__, bank->id * bank->width + __ffs(active));
		ret = -EBUSY;
	}

	if (pm_runtime_put_sync_suspend(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_put_sync "
				"failed\n", __func__, bank->id);
		return -EINVAL;
	}

	return ret;
}

static void omap2_gpio_restore_edge_wakeup(struct gpio_bank *bank)
{
	if (pm_runtime_get_sync(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_get_sync "
				"failed\n", __func__, bank->id);
		return;
	}

	__raw_writel(bank->context.edge_falling,
			(bank->base + bank->regs->fallingdetect));
	__raw_writel(bank->context.edge_rising,
			(bank->base + bank->regs->risingdetect));
	__raw_writel(bank->context.ew_leveldetect0,
			(bank->base + bank->regs->leveldetect0));
	__raw_writel(bank->context.ew_leveldetect1,
			(bank->base + bank->regs->leveldetect1));

	if (pm_runtime_put_sync_suspend(bank->dev) < 0) {
		dev_err(bank->dev, "%s: GPIO bank %d pm_runtime_put_sync "
				"failed\n", __func__, bank->id);
		return;
	}
}

int omap2_gpio_prepare_for_idle(int off_mode, bool suspend)
{
	int ret = 0;
	struct gpio_bank *bank;

	list_for_each_entry(bank, &omap_gpio_list, node) {
		omap2_gpio_set_wakeupenables(bank);

		if (omap2_gpio_set_edge_wakeup(bank, suspend))
			ret = -EBUSY;

		if (bank->mod_usage && bank->loses_context && off_mode)
			if (pm_runtime_put_sync_suspend(bank->dev) < 0)
				dev_err(bank->dev, "%s: GPIO bank %d "
						"pm_runtime_put_sync failed\n",
						__func__, bank->id);
	}

	if (ret)
		omap2_gpio_resume_after_idle(off_mode);

	return ret;
}

void omap2_gpio_resume_after_idle(int off_mode)
{
	struct gpio_bank *bank;

	list_for_each_entry(bank, &omap_gpio_list, node) {
		if (bank->mod_usage && bank->loses_context && off_mode)
			if (pm_runtime_get_sync(bank->dev) < 0)
				dev_err(bank->dev, "%s: GPIO bank %d "
						"pm_runtime_get_sync failed\n",
						__func__, bank->id);

		omap2_gpio_restore_edge_wakeup(bank);

		omap2_gpio_clear_wakeupenables(bank);
	}

}
void omap_gpio_save_context(struct gpio_bank *bank)
{
	bank->context.irqenable1 =
			__raw_readl(bank->base + bank->regs->irqenable);
	bank->context.irqenable2 =
			__raw_readl(bank->base + bank->regs->irqenable2);
	bank->context.wake_en =
			__raw_readl(bank->base + bank->regs->wkup_set);
	bank->context.ctrl = __raw_readl(bank->base + bank->regs->ctrl);
	bank->context.oe = __raw_readl(bank->base + bank->regs->direction);
	bank->context.leveldetect0 =
			__raw_readl(bank->base + bank->regs->leveldetect0);
	bank->context.leveldetect1 =
			__raw_readl(bank->base + bank->regs->leveldetect1);
	bank->context.risingdetect =
			__raw_readl(bank->base + bank->regs->risingdetect);
	bank->context.fallingdetect =
			__raw_readl(bank->base + bank->regs->fallingdetect);
	bank->context.dataout = __raw_readl(bank->base + bank->regs->dataout);
	if (bank->dbck_enable_mask) {
		bank->context.debounce = __raw_readl(bank->base +
						     bank->regs->debounce);
		bank->context.debounce_en = __raw_readl(bank->base +
						bank->regs->debounce_en);
	}
	bank->saved_context = 1;
}

void omap_gpio_restore_context(struct gpio_bank *bank)
{
	if(!bank->saved_context)
		return;
	__raw_writel(bank->context.wake_en,
				bank->base + bank->regs->wkup_set);
	__raw_writel(bank->context.ctrl, bank->base + bank->regs->ctrl);
	__raw_writel(bank->context.leveldetect0,
				bank->base + bank->regs->leveldetect0);
	__raw_writel(bank->context.leveldetect1,
				bank->base + bank->regs->leveldetect1);
	__raw_writel(bank->context.risingdetect,
				bank->base + bank->regs->risingdetect);
	__raw_writel(bank->context.fallingdetect,
				bank->base + bank->regs->fallingdetect);
	if (bank->regs->set_dataout && bank->regs->clr_dataout)
		__raw_writel(bank->context.dataout,
				bank->base + bank->regs->set_dataout);
	else
		__raw_writel(bank->context.dataout,
				bank->base + bank->regs->dataout);
	__raw_writel(bank->context.oe, bank->base + bank->regs->direction);
	if (bank->dbck_enable_mask) {
		__raw_writel(bank->context.debounce, bank->base +
			     bank->regs->debounce);
		__raw_writel(bank->context.debounce_en,
			     bank->base + bank->regs->debounce_en);
	}
	__raw_writel(bank->context.irqenable1,
				bank->base + bank->regs->irqenable);
	__raw_writel(bank->context.irqenable2,
				bank->base + bank->regs->irqenable2);
	bank->saved_context = 0;
}
#endif

static const struct dev_pm_ops gpio_pm_ops = {
	.runtime_suspend	= omap_gpio_pm_runtime_suspend,
	.runtime_resume		= omap_gpio_pm_runtime_resume,
	.suspend		= omap_gpio_suspend,
	.resume			= omap_gpio_resume,
};

static struct platform_driver omap_gpio_driver = {
	.probe		= omap_gpio_probe,
	.driver		= {
		.name	= "omap_gpio",
		.pm	= &gpio_pm_ops,
	},
};

/*
 * gpio driver register needs to be done before
 * machine_init functions access gpio APIs.
 * Hence omap_gpio_drv_reg() is a postcore_initcall.
 */
static int __init omap_gpio_drv_reg(void)
{
	return platform_driver_register(&omap_gpio_driver);
}
postcore_initcall(omap_gpio_drv_reg);

