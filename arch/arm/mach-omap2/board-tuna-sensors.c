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

#include "mux.h"
#include "board-tuna.h"

#define GPIO_GYRO_INT		45
#define GPIO_ACC_INT		122
#define GPIO_MAG_INT		176

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
		.orientation = {  0,  1,  0,
				  1,  0,  0,
				  0,  0, -1 },
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
};


void __init omap4_tuna_sensors_init(void)
{
	omap_mux_init_gpio(GPIO_GYRO_INT, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_ACC_INT, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_MAG_INT, OMAP_PIN_INPUT);

	gpio_request(GPIO_GYRO_INT, "GYRO_INT");
	gpio_direction_input(GPIO_GYRO_INT);
	gpio_request(GPIO_ACC_INT, "ACC_INT");
	gpio_direction_input(GPIO_ACC_INT);
	gpio_request(GPIO_MAG_INT, "MAG_INT");
	gpio_direction_input(GPIO_MAG_INT);

	i2c_register_board_info(4, tuna_sensors_i2c4_boardinfo,
				ARRAY_SIZE(tuna_sensors_i2c4_boardinfo));
}
