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
#include <linux/platform_data/mms_ts.h>
#include <asm/mach-types.h>
#include <plat/omap4-keypad.h>

#include "board-tuna.h"
#include "mux.h"

#define GPIO_TOUCH_EN		19
#define GPIO_LUNCHBOX_TOUCH_EN	54
#define GPIO_TOUCH_IRQ		46

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

static struct gpio_event_direct_entry tuna_gpio_keypad_keys_map_high[] = {
	{
		.code	= KEY_POWER,
		.gpio	= 3,
	},
};

static struct gpio_event_input_info tuna_gpio_keypad_keys_info_high = {
	.info.func = gpio_event_input_func,
	.type = EV_KEY,
	.keymap = tuna_gpio_keypad_keys_map_high,
	.keymap_size = ARRAY_SIZE(tuna_gpio_keypad_keys_map_high),
	.flags = GPIOEDF_ACTIVE_HIGH,
};

static struct gpio_event_direct_entry tuna_gpio_keypad_keys_map_low[] = {
	{
		.code	= KEY_VOLUMEDOWN,
		.gpio	= 8,
	},
	{
		.code	= KEY_VOLUMEUP,
		.gpio	= 30,
	},
};

static struct gpio_event_input_info tuna_gpio_keypad_keys_info_low = {
	.info.func = gpio_event_input_func,
	.type = EV_KEY,
	.keymap = tuna_gpio_keypad_keys_map_low,
	.keymap_size = ARRAY_SIZE(tuna_gpio_keypad_keys_map_low),
};

static struct gpio_event_info *tuna_gpio_keypad_info[] = {
	&tuna_gpio_keypad_keys_info_high.info,
	&tuna_gpio_keypad_keys_info_low.info,
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

static struct i2c_board_info __initdata tuna_i2c3_boardinfo_pre_lunchbox[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4a),
		.platform_data = &atmel_mxt_ts_pdata,
		.irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
	},
};

static struct mms_ts_platform_data mms_ts_pdata = {
	.max_x		= 720,
	.max_y		= 1280,
};

static struct i2c_board_info __initdata tuna_i2c3_boardinfo_lunchbox[] = {
	{
		I2C_BOARD_INFO("mms_ts", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &mms_ts_pdata,
		.irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
	},
};

void __init omap4_tuna_input_init(void)
{
	gpio_request(GPIO_TOUCH_IRQ, "tsp_int_n");
	gpio_direction_input(GPIO_TOUCH_IRQ);
	omap_mux_init_gpio(GPIO_TOUCH_IRQ, OMAP_PIN_INPUT_PULLUP);

	if (omap4_tuna_final_gpios()) {
		gpio_request(GPIO_TOUCH_EN, "tsp_en");
		gpio_direction_output(GPIO_TOUCH_EN, 1);
		omap_mux_init_gpio(GPIO_TOUCH_EN, OMAP_PIN_OUTPUT);
	} else {
		gpio_request(GPIO_LUNCHBOX_TOUCH_EN, "tsp_en");
		gpio_direction_output(GPIO_LUNCHBOX_TOUCH_EN, 1);
		omap_mux_init_gpio(GPIO_LUNCHBOX_TOUCH_EN, OMAP_PIN_OUTPUT);
	}

	if (omap4_tuna_get_revision() == TUNA_REV_PRE_LUNCHBOX) {
		i2c_register_board_info(3, tuna_i2c3_boardinfo_pre_lunchbox,
			ARRAY_SIZE(tuna_i2c3_boardinfo_pre_lunchbox));

		omap_mux_init_signal("kpd_row1.kpd_row1", OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("kpd_row2.kpd_row2", OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("kpd_col1.kpd_col1", OMAP_PIN_OUTPUT);
		omap4_keyboard_init(&tuna_keypad_data);
		tuna_gpio_keypad_data.info_count = 1;
	} else {
		i2c_register_board_info(3, tuna_i2c3_boardinfo_lunchbox,
			ARRAY_SIZE(tuna_i2c3_boardinfo_lunchbox));

		omap_mux_init_gpio(8, OMAP_PIN_INPUT);
		omap_mux_init_gpio(30, OMAP_PIN_INPUT);
	}

	platform_device_register(&tuna_gpio_keypad_device);
}
