/* Sensor support for Samsung Tuna Board.
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/gp2a.h>
#include <linux/i2c/twl6030-madc.h>

#include "mux.h"
#include "board-tuna.h"

#define GPIO_GYRO_INT		45
#define GPIO_ACC_INT		122
#define GPIO_MAG_INT		176
#define GPIO_PS_ON		25
#define GPIO_PS_VOUT		21
#define GPIO_MSENSE_IRQ		157

#define GP2A_LIGHT_ADC_CHANNEL	4

static int gp2a_light_adc_value(void)
{
	return twl6030_get_madc_conversion(GP2A_LIGHT_ADC_CHANNEL);
}

static void gp2a_power(bool on)
{
	/* this controls the power supply rail to the gp2a IC */
	gpio_set_value(GPIO_PS_ON, on);
}

static void gp2a_gpio_init(void)
{
	int ret = gpio_request(GPIO_PS_ON, "gp2a_power_supply_on");
	if (ret) {
		pr_err("%s Failed to request gpio gp2a power supply\n",
			__func__);
		return;
	}
	/* set power pin to output, initially powered off*/
	ret = gpio_direction_output(GPIO_PS_ON, 0);
	if (ret) {
		pr_err("%s Failed in gpio_direction_output, value 0 with error %d\n",
			__func__, ret);
	}
}

static s8 orientation_back[] = {
	-1,  0,  0,
	 0,  1,  0,
	 0,  0, -1,
};

static s8 orientation_back_right_90[] = {
	 0, -1,  0,
	-1,  0,  0,
	 0,  0, -1,
};

static s8 orientation_back_left_90[] = {
	 0,  1,  0,
	 1,  0,  0,
	 0,  0, -1,
};

static s8 orientation_back_180[] = {
	 1,  0,  0,
	 0, -1,  0,
	 0,  0, -1,
};

/*
 * A correction matrix for YAS530
 * which takes care of soft iron effect in TORO
 */
static s32 compass_correction_matrix_toro[] = {
	1072,	-51,	-22,
	-30,	910,	-4,
	-23,	-63,	1024,
};

static void rotcpy(s8 dst[3 * 3], const s8 src[3 * 3])
{
	memcpy(dst, src, 3 * 3);
}

static struct mpu_platform_data mpu_data = {
	.int_config  = 0x10,
	.orientation = {  1,  0,  0,
			  0,  1,  0,
			  0,  0,  1 },
	/* accel */
	.accel = {
		.irq = OMAP_GPIO_IRQ(GPIO_ACC_INT),
		.adapt_num   = 4,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x18,
		.orientation = {  1,  0,  0,
				  0,  1,  0,
				  0,  0,  1 },
	},
	/* compass */
	.compass = {
		.irq = OMAP_GPIO_IRQ(GPIO_MAG_INT),
		.adapt_num   = 4,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x2E,
		.orientation = {  1,  0,  0,
				  0,  1,  0,
				  0,  0,  1 },
	},
};

static struct gp2a_platform_data gp2a_pdata = {
	.power = gp2a_power,
	.p_out = GPIO_PS_VOUT,
	.light_adc_value = gp2a_light_adc_value,
};

static struct i2c_board_info __initdata tuna_sensors_i2c4_boardinfo[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq = OMAP_GPIO_IRQ(GPIO_GYRO_INT),
		.platform_data = &mpu_data,
	},
	{
		I2C_BOARD_INFO("bma250", 0x18),
		.irq = OMAP_GPIO_IRQ(GPIO_ACC_INT),
		.platform_data = &mpu_data.accel,
	},
	{
		I2C_BOARD_INFO("yas530", 0x2e),
		.irq = OMAP_GPIO_IRQ(GPIO_MAG_INT),
		.platform_data = &mpu_data.compass,
	},
	{
		I2C_BOARD_INFO("gp2a", 0x44),
		.platform_data = &gp2a_pdata,
	},
	{
		I2C_BOARD_INFO("bmp180", 0x77),
	},
};

static void omap4_tuna_fixup_orientations_maguro(int revision)
{
	if (revision >= 3) {
		rotcpy(mpu_data.orientation, orientation_back_right_90);
		rotcpy(mpu_data.accel.orientation, orientation_back_left_90);
	} else if (revision >= 2) {
		rotcpy(mpu_data.orientation, orientation_back_right_90);
		rotcpy(mpu_data.accel.orientation, orientation_back_180);
	} else if (revision == 1) {
		rotcpy(mpu_data.accel.orientation, orientation_back_left_90);
	}
}

static void omap4_tuna_fixup_orientations_toro(int revision)
{
	if (revision >= 2) {
		rotcpy(mpu_data.orientation, orientation_back_left_90);
		rotcpy(mpu_data.accel.orientation, orientation_back);
		rotcpy(mpu_data.compass.orientation, orientation_back_180);
	} else if (revision >= 1) {
		rotcpy(mpu_data.orientation, orientation_back_left_90);
		rotcpy(mpu_data.accel.orientation, orientation_back_180);
		rotcpy(mpu_data.compass.orientation, orientation_back_left_90);
	}
}

void __init omap4_tuna_sensors_init(void)
{
	omap_mux_init_gpio(GPIO_GYRO_INT, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_ACC_INT, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_MAG_INT, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_PS_ON, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_PS_VOUT, OMAP_WAKEUP_EN | OMAP_PIN_INPUT);

	gpio_request(GPIO_GYRO_INT, "GYRO_INT");
	gpio_direction_input(GPIO_GYRO_INT);
	gpio_request(GPIO_ACC_INT, "ACC_INT");
	gpio_direction_input(GPIO_ACC_INT);
	gpio_request(GPIO_MAG_INT, "MAG_INT");
	gpio_direction_input(GPIO_MAG_INT);
	gpio_request(GPIO_MSENSE_IRQ, "MSENSE_IRQ");
	gpio_direction_output(GPIO_MSENSE_IRQ, 1);
	/* optical sensor */
	gp2a_gpio_init();

	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO) {
		omap4_tuna_fixup_orientations_maguro(omap4_tuna_get_revision());
	} else if (omap4_tuna_get_type() == TUNA_TYPE_TORO) {
		omap4_tuna_fixup_orientations_toro(omap4_tuna_get_revision());
		mpu_data.compass.private_data = compass_correction_matrix_toro;
	}

	i2c_register_board_info(4, tuna_sensors_i2c4_boardinfo,
				ARRAY_SIZE(tuna_sensors_i2c4_boardinfo));
}
