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
#define GPIO_TOUCH_IRQ		46

/* touch is on i2c3 */
#define GPIO_TOUCH_SCL	130
#define GPIO_TOUCH_SDA	131

static int mms_ts_panel_id;

static struct gpio_event_direct_entry tuna_gpio_keypad_keys_map_high[] = {
	{
		.code	= KEY_POWER,
		.gpio	= 3,
	},
};

static struct gpio_event_input_info tuna_gpio_keypad_keys_info_high = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.type = EV_KEY,
	.keymap = tuna_gpio_keypad_keys_map_high,
	.keymap_size = ARRAY_SIZE(tuna_gpio_keypad_keys_map_high),
	.flags = GPIOEDF_ACTIVE_HIGH,
	.debounce_time.tv64 = 2 * NSEC_PER_MSEC,
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
	.info.no_suspend = true,
	.type = EV_KEY,
	.keymap = tuna_gpio_keypad_keys_map_low,
	.keymap_size = ARRAY_SIZE(tuna_gpio_keypad_keys_map_low),
	.debounce_time.tv64 = 2 * NSEC_PER_MSEC,
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

static int melfas_mux_fw_flash(bool to_gpios)
{
	/* TOUCH_EN is always an output */
	if (to_gpios) {
		gpio_direction_output(GPIO_TOUCH_IRQ, 0);
		omap_mux_set_gpio(
			OMAP_PIN_INPUT | OMAP_MUX_MODE3,
			GPIO_TOUCH_IRQ);

		gpio_direction_output(GPIO_TOUCH_SCL, 0);
		omap_mux_set_gpio(OMAP_PIN_INPUT | OMAP_MUX_MODE3,
				  GPIO_TOUCH_SCL);

		gpio_direction_output(GPIO_TOUCH_SDA, 0);
		omap_mux_set_gpio(OMAP_PIN_INPUT | OMAP_MUX_MODE3,
				  GPIO_TOUCH_SDA);
	} else {
		gpio_direction_output(GPIO_TOUCH_IRQ, 1);
		gpio_direction_input(GPIO_TOUCH_IRQ);
		omap_mux_set_gpio(
			OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE3,
			GPIO_TOUCH_IRQ);

		gpio_direction_output(GPIO_TOUCH_SCL, 1);
		gpio_direction_input(GPIO_TOUCH_SCL);
		omap_mux_set_gpio(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
				  GPIO_TOUCH_SCL);

		gpio_direction_output(GPIO_TOUCH_SDA, 1);
		gpio_direction_input(GPIO_TOUCH_SDA);
		omap_mux_set_gpio(OMAP_PIN_INPUT | OMAP_MUX_MODE0,
				  GPIO_TOUCH_SDA);
	}

	return 0;
}

static int __init mms_ts_panel_id_setup(char *str)
{
	mms_ts_panel_id = simple_strtol(str, NULL, 0);
	return 1;
}
__setup("mms_ts.panel_id=", mms_ts_panel_id_setup);

static struct mms_ts_platform_data mms_ts_pdata = {
	.max_x		= 720,
	.max_y		= 1280,
	.mux_fw_flash	= melfas_mux_fw_flash,
	.gpio_resetb	= GPIO_TOUCH_IRQ,
	.gpio_vdd_en	= GPIO_TOUCH_EN,
	.gpio_scl	= GPIO_TOUCH_SCL,
	.gpio_sda	= GPIO_TOUCH_SDA,
};

static struct i2c_board_info __initdata tuna_i2c3_boardinfo_final[] = {
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
	omap_mux_init_gpio(GPIO_TOUCH_IRQ,
			   OMAP_PIN_INPUT_PULLUP);

	gpio_request(GPIO_TOUCH_EN, "tsp_en");
	gpio_direction_output(GPIO_TOUCH_EN, 1);
	omap_mux_init_gpio(GPIO_TOUCH_EN, OMAP_PIN_OUTPUT);
	gpio_request(GPIO_TOUCH_SCL, "ap_i2c3_scl");
	gpio_request(GPIO_TOUCH_SDA, "ap_i2c3_sda");

	/* 0x12 == FPCB 3.2
	 * 0xa1 == FPCB 3.1
	 */
	if (mms_ts_panel_id == 0x12 || mms_ts_panel_id == 0xA2)
		mms_ts_pdata.fw_name = "mms144_ts_rev32.fw";
	else
		mms_ts_pdata.fw_name = "mms144_ts_rev31.fw";

	i2c_register_board_info(3, tuna_i2c3_boardinfo_final,
		ARRAY_SIZE(tuna_i2c3_boardinfo_final));

	omap_mux_init_gpio(8, OMAP_PIN_INPUT);
	omap_mux_init_gpio(30, OMAP_PIN_INPUT);

	platform_device_register(&tuna_gpio_keypad_device);
}
