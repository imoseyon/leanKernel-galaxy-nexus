/* Power support for Samsung Tuna Board.
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/max17040_battery.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>

/* These will be different on lunchbox hardware */
#define GPIO_CHARGING_N		11
#define GPIO_TA_NCONNECTED	12
#define GPIO_CHARGE_N		13

static struct gpio charger_gpios[] = {
	{ .gpio = GPIO_CHARGING_N, .flags = GPIOF_IN, .label = "charging_n" },
	{ .gpio = GPIO_TA_NCONNECTED, .flags = GPIOF_IN, .label = "charger_n" },
	{ .gpio = GPIO_CHARGE_N, .flags = GPIOF_OUT_INIT_LOW, .label = "charge_n " },
};

static int charger_init(struct device *dev)
{
	return gpio_request_array(charger_gpios, ARRAY_SIZE(charger_gpios));
}

static void charger_exit(struct device *dev)
{
	gpio_free_array(charger_gpios, ARRAY_SIZE(charger_gpios));
}

static void charger_set_charge(int state)
{
	gpio_set_value(GPIO_CHARGE_N, !state);
}

static int charger_is_ac_online(void)
{
	return !gpio_get_value(GPIO_TA_NCONNECTED);
}

static int charger_is_charging(void)
{
	return !gpio_get_value(GPIO_CHARGING_N);
}

static const __initdata struct resource charger_resources[] = {
	{
		.name = "ac",
		.start = OMAP_GPIO_IRQ(GPIO_TA_NCONNECTED),
		.end = OMAP_GPIO_IRQ(GPIO_TA_NCONNECTED),
		.flags = IORESOURCE_IRQ |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	},
	{
		.name = "usb",
		.start = OMAP_GPIO_IRQ(GPIO_TA_NCONNECTED),
		.end = OMAP_GPIO_IRQ(GPIO_TA_NCONNECTED),
		.flags = IORESOURCE_IRQ |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	}
};

static const __initdata struct pda_power_pdata charger_pdata = {
	.init = charger_init,
	.exit = charger_exit,
	.is_ac_online = charger_is_ac_online,
	.set_charge = charger_set_charge,
	.wait_for_status = 500,
	.wait_for_charger = 500,
};

static struct max17040_platform_data max17043_pdata = {
	.charger_online = charger_is_ac_online,
	.charger_enable = charger_is_charging,
};

static const __initdata struct i2c_board_info max17043_i2c[] = {
	{
		I2C_BOARD_INFO("max17040", (0x6C >> 1)),
		.platform_data = &max17043_pdata,
	}
};

void __init omap4_tuna_power_init(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_resndata(NULL, "pda-power", -1,
		charger_resources, ARRAY_SIZE(charger_resources),
		&charger_pdata, sizeof(charger_pdata));

	i2c_register_board_info(4, max17043_i2c, ARRAY_SIZE(max17043_i2c));
}
