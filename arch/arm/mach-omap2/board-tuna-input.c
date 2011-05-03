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
#include <linux/keyreset.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
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

void __init omap4_tuna_input_init(void)
{
	omap4_keyboard_init(&tuna_keypad_data);
	platform_device_register(&tuna_gpio_keypad_device);
}
