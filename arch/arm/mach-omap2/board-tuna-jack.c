/* Board support file for Samsung Tuna Board.
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-omap4panda.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/i2c/twl6030-madc.h>
#include <linux/sec_jack.h>

#include "mux.h"
#include "board-tuna.h"

#define GPIO_EAR_MICBIAS_EN	49
#define GPIO_DET_35		0
#define GPIO_EAR_SEND_END	1

#define ADC_CHANNEL_JACK	2

static void sec_jack_set_micbias_state(bool on)
{
	gpio_set_value(GPIO_EAR_MICBIAS_EN, on);
}

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc < 50, unstable zone, default to 3pole if it stays
		* in this range for a half second (20ms delays, 25 samples)
		*/
		.adc_high = 50,
		.delay_ms = 20,
		.check_count = 25,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 50 < adc <= 490, unstable zone, default to 3pole if it stays
		* in this range for a second (10ms delays, 100 samples)
		*/
		.adc_high = 490,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 490 < adc <= 900, unstable zone, default to 4pole if it
		* stays in this range for a second (10ms delays, 100 samples)
		*/
		.adc_high = 900,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 900 < adc <= 1500, 4 pole zone, default to 4pole if it
		* stays in this range for 200ms (20ms delays, 10 samples)
		*/
		.adc_high = 1500,
		.delay_ms = 20,
		.check_count = 10,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 1500, unstable zone, default to 3pole if it stays
		* in this range for a second (10ms delays, 100 samples)
		*/
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <= 93, stable zone */
		.code           = KEY_MEDIA,
		.adc_low        = 0,
		.adc_high       = 93,
	},
	{
		/* 94 <= adc <= 167, stable zone */
		.code           = KEY_PREVIOUSSONG,
		.adc_low        = 94,
		.adc_high       = 167,
	},
	{
		/* 168 <= adc <= 370, stable zone */
		.code           = KEY_NEXTSONG,
		.adc_low        = 168,
		.adc_high       = 370,
	},
};

static int sec_jack_get_adc_value(void)
{
	int value;

	value =  twl6030_get_madc_conversion(ADC_CHANNEL_JACK);
	return (int)(1800*value) / 1024;
}

struct sec_jack_platform_data sec_jack_pdata = {
	.set_micbias_state = sec_jack_set_micbias_state,
	.get_adc_value = sec_jack_get_adc_value,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.buttons_zones = sec_jack_buttons_zones,
	.num_buttons_zones = ARRAY_SIZE(sec_jack_buttons_zones),
	.det_gpio = GPIO_DET_35,
	.send_end_gpio = GPIO_EAR_SEND_END,
};

static struct platform_device sec_device_jack = {
	.name                   = "sec_jack",
	.id                     = 1, /* will be used also for gpio_event id */
	.dev.platform_data      = &sec_jack_pdata,
};

void __init omap4_tuna_jack_init(void)
{
	omap_mux_init_signal("sim_io.gpio_wk0", OMAP_PIN_INPUT);
	gpio_request(GPIO_DET_35, "det_35_en");
	gpio_direction_input(GPIO_DET_35);

	omap_mux_init_signal("sim_clk.gpio_wk1", OMAP_PIN_INPUT);
	gpio_request(GPIO_EAR_SEND_END, "ear_send_end");
	gpio_direction_input(GPIO_EAR_SEND_END);

	omap_mux_init_signal("gpmc_a25.gpio_49", OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
	gpio_request(GPIO_EAR_MICBIAS_EN, "ear_micbias_en");
	gpio_direction_output(GPIO_EAR_MICBIAS_EN, 0);

	gpio_free(GPIO_DET_35);
	gpio_free(GPIO_EAR_SEND_END);
	platform_device_register(&sec_device_jack);
}
