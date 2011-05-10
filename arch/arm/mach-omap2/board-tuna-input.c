/*
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/keyreset.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <asm/mach-types.h>
#include <plat/omap4-keypad.h>

#include "board-tuna.h"

static const int tuna_keymap[] = {
	KEY(1, 1, KEY_VOLUMEDOWN),
	KEY(2, 1, KEY_VOLUMEUP),
};

static struct matrix_keymap_data tuna_keymap_data = {
	.keymap			= tuna_keymap,
	.keymap_size		= ARRAY_SIZE(tuna_keymap),
};

static struct omap4_keypad_platform_data tuna_keypad_data = {
	.keymap_data		= &tuna_keymap_data,
	.rows			= 3,
	.cols			= 2,
};

static struct gpio_event_direct_entry tuna_gpio_keypad_keys_map[] = {
	{
		.code	= KEY_POWER,
		.gpio	= 3,
	},
};

static struct gpio_event_input_info tuna_gpio_keypad_keys_info = {
	.info.func = gpio_event_input_func,
	.type = EV_KEY,
	.keymap = tuna_gpio_keypad_keys_map,
	.keymap_size = ARRAY_SIZE(tuna_gpio_keypad_keys_map),
	.info.no_suspend = false,
	.flags = GPIOEDF_ACTIVE_HIGH,
};

static struct gpio_event_info *tuna_gpio_keypad_info[] = {
	&tuna_gpio_keypad_keys_info.info,
};

static struct gpio_event_platform_data tuna_gpio_keypad_data = {
	.name = "tuna-gpio-keypad",
	.info = tuna_gpio_keypad_info,
	.info_count = ARRAY_SIZE(tuna_gpio_keypad_info)
};

static struct platform_device tuna_gpio_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data	= &tuna_gpio_keypad_data,
	},
};

static struct mxt_platform_data atmel_mxt_ts_pdata = {
	.x_line		= 19,
	.y_line		= 11,
	.x_size		= 1024,
	.y_size		= 1024,
	.blen		= 0x21,
	.threshold	= 0x28,
	.voltage	= 2800000,              /* 2.8V */
	.orient		= MXT_DIAGONAL,
	.irqflags	= IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata tuna_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4a),
		.platform_data = &atmel_mxt_ts_pdata,
		.irq = OMAP_GPIO_IRQ(46),
	},
};

void __init omap4_tuna_input_init(void)
{
	gpio_request(46, "tsp_int_n");
	gpio_direction_input(46);

	gpio_request(54, "tsp_en");
	gpio_direction_output(54, 1);

	i2c_register_board_info(3, tuna_i2c3_boardinfo,
		ARRAY_SIZE(tuna_i2c3_boardinfo));

	omap4_keyboard_init(&tuna_keypad_data);
	platform_device_register(&tuna_gpio_keypad_device);
}
