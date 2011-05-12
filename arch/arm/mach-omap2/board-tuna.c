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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/display.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include "timer-gp.h"

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-tuna.h"

#define GPIO_WIFI_PMENA		43
#define GPIO_WIFI_IRQ		53

static int tuna_hw_rev;

static struct gpio tuna_hw_rev_gpios[] = {
	{76, GPIOF_IN, "hw_rev0"},
	{75, GPIOF_IN, "hw_rev1"},
	{74, GPIOF_IN, "hw_rev2"},
	{73, GPIOF_IN, "hw_rev3"},
	{170, GPIOF_IN, "hw_rev4"},
};

static void omap4_tuna_init_hw_rev(void)
{
	int ret;
	int i;

	ret = gpio_request_array(tuna_hw_rev_gpios,
		ARRAY_SIZE(tuna_hw_rev_gpios));

	BUG_ON(ret);

	for (i = 0; i < ARRAY_SIZE(tuna_hw_rev_gpios); i++)
		tuna_hw_rev |= gpio_get_value(tuna_hw_rev_gpios[i].gpio) << i;

	pr_info("Tuna HW revision: %02x\n", tuna_hw_rev);
}

int omap4_tuna_get_revision(void)
{
	return tuna_hw_rev & TUNA_REV_MASK;
}

int omap4_tuna_get_type(void)
{
	return tuna_hw_rev & TUNA_TYPE_MASK;
}

/* wl127x BT, FM, GPS connectivity chip */
static int wl1271_gpios[] = {46, -1, -1};
static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
		.platform_data	= &wl1271_gpios,
	},
};

static struct platform_device *tuna_devices[] __initdata = {
	&wl1271_device,
};

static void __init tuna_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	.mode			= MUSB_PERIPHERAL,
#else
	.mode			= MUSB_OTG,
#endif
	.power			= 100,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.nonremovable	= true,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask       = MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
#if 0
	{
		.name		= "wl1271",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

static struct regulator_consumer_supply tuna_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};

static struct regulator_consumer_supply tuna_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data tuna_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &tuna_vmmc5_supply,
};

static struct fixed_voltage_config tuna_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &tuna_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &tuna_vwlan,
	},
};

struct wl12xx_platform_data tuna_wlan_data  __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	/* PANDA ref clock is 38.4 MHz */
	.board_ref_clock = 2,
};

static struct regulator_init_data tuna_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data tuna_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = tuna_vmmc_supply,
};

static struct regulator_init_data tuna_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data tuna_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_platform_data tuna_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &tuna_vmmc,
	.vpp		= &tuna_vpp,
	.vana		= &tuna_vana,
	.vcxio		= &tuna_vcxio,
	.vdac		= &tuna_vdac,
	.vusb		= &tuna_vusb,
	.vaux2		= &tuna_vaux2,
	.vaux3		= &tuna_vaux3,
	.clk32kg	= &tuna_clk32kg,
	.usb		= &omap4_usbphy_data,
};

static struct i2c_board_info __initdata tuna_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &tuna_twldata,
	},
};

static int __init tuna_i2c_init(void)
{
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, tuna_i2c1_boardinfo,
			      ARRAY_SIZE(tuna_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, NULL, 0);
	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* hwrev */
	OMAP4_MUX(CSI21_DY4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(USBB2_HSIC_STROBE, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* hsmmc d0-d7 */
	OMAP4_MUX(GPMC_AD0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD2, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD4, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD5, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD6, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(GPMC_AD7, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	/* hsmmc cmd */
	OMAP4_MUX(GPMC_NWE, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	/* hsmmc clk */
	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux board_wkup_mux[] __initdata = {
	/* power button */
	OMAP4_MUX(SIM_CD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	OMAP_MUX_STATIC("uart2_cts.uart2_cts",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rts.uart2_rts",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rx.uart2_rx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_tx.uart2_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial3_pads[] __initdata = {
	OMAP_MUX_STATIC("uart3_cts_rctx.uart3_cts_rctx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rts_sd.uart3_rts_sd",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rx_irrx.uart3_rx_irrx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_tx_irtx.uart3_tx_irtx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial4_pads[] __initdata = {
	OMAP_MUX_STATIC("uart4_rx.uart4_rx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart4_tx.uart4_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_board_data serial2_data = {
	.id             = 1,
	.pads           = serial2_pads,
	.pads_cnt       = ARRAY_SIZE(serial2_pads),
};

static struct omap_board_data serial3_data = {
	.id             = 2,
	.pads           = serial3_pads,
	.pads_cnt       = ARRAY_SIZE(serial3_pads),
};

static struct omap_board_data serial4_data = {
	.id             = 3,
	.pads           = serial4_pads,
	.pads_cnt       = ARRAY_SIZE(serial4_pads),
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;
	bdata.flags     = 0;
	bdata.pads      = NULL;
	bdata.pads_cnt  = 0;
	bdata.id        = 0;
	/* pass dummy data for UART1 */
	omap_serial_init_port(&bdata);

	omap_serial_init_port(&serial2_data);
	omap_serial_init_port(&serial3_data);
	omap_serial_init_port(&serial4_data);
}
#else
#define board_mux	NULL

static inline void board_serial_init(void)
{
	omap_serial_init();
}
#endif

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	if (wl12xx_set_platform_data(&tuna_wlan_data))
		pr_err("error setting wl12xx data\n");

	tuna_i2c_init();
	platform_add_devices(tuna_devices, ARRAY_SIZE(tuna_devices));
	platform_device_register(&omap_vwlan_device);
	board_serial_init();
	omap2_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);
	omap4_tuna_android_usb_init();
	omap4_tuna_display_init();
	omap4_tuna_input_init();
	omap4_tuna_power_init();
}

static void __init tuna_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(TUNA, "Tuna")
	/* Maintainer: Google, Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= tuna_map_io,
	.init_early	= tuna_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= tuna_init,
	.timer		= &omap_timer,
MACHINE_END
