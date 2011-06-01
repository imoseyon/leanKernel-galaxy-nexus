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

#define GPIO_WIFI_PMENA		104
#define GPIO_WIFI_IRQ		16

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
	u32 r;

	/* Disable weak driver pulldown on usbb2_hsic_strobe */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);
	r &= ~OMAP4_USBB2_HSIC_STROBE_WD_MASK;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);

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

bool omap4_tuna_final_gpios(void)
{
	int type = omap4_tuna_get_type();
	int rev = omap4_tuna_get_revision();

	if (type == TUNA_TYPE_TORO ||
	    (rev != TUNA_REV_PRE_LUNCHBOX && rev != TUNA_REV_LUNCHBOX))
		return true;

	return false;
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
		.mmc		= 1,
		.nonremovable	= true,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask       = MMC_VDD_165_195,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply tuna_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
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
	.microvolts = 2000000, /* 2.0V */
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

static struct regulator_consumer_supply tuna_vaux3_supplies[] = {
	{
		.supply = "vlcd",
	},
};

static struct regulator_init_data tuna_vaux3 = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tuna_vaux3_supplies),
	.consumer_supplies = &tuna_vaux3_supplies,
};

static struct regulator_init_data tuna_vmmc = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 2,
	.consumer_supplies = tuna_vmmc_supply,
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

static struct regulator_consumer_supply tuna_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
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
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vcxio_supply),
	.consumer_supplies	= tuna_vcxio_supply,

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
#define board_wkup_mux	NULL

static inline void board_serial_init(void)
{
	omap_serial_init();
}
#endif

#define HSMMC2_MUX	(OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP)
#define HSMMC1_MUX	OMAP_PIN_INPUT_PULLUP
#define HSMMC5_MUX	OMAP_PIN_INPUT_PULLUP

static void tuna_wlan_init(void)
{
	/* WLAN SDIO: MMC5 CMD */
	omap_mux_init_signal("sdmmc5_cmd", HSMMC5_MUX);
	/* WLAN SDIO: MMC5 CLK */
	omap_mux_init_signal("sdmmc5_clk", HSMMC5_MUX);
	/* WLAN SDIO: MMC5 DAT[0-3] */
	omap_mux_init_signal("sdmmc5_dat0", HSMMC5_MUX);
	omap_mux_init_signal("sdmmc5_dat1", HSMMC5_MUX);
	omap_mux_init_signal("sdmmc5_dat2", HSMMC5_MUX);
	omap_mux_init_signal("sdmmc5_dat3", HSMMC5_MUX);
	/* WLAN OOB - BCM4330 - GPIO 16 or GPIO 2 */
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT);
	omap_mux_init_signal("sim_reset", OMAP_MUX_MODE3 | OMAP_PIN_INPUT);
	/* WLAN PMENA - GPIO 104 */
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);
}

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	if (omap4_tuna_final_gpios()) {
		/* hsmmc d0-d7 */
		omap_mux_init_signal("sdmmc1_dat0.sdmmc1_dat0", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat1.sdmmc1_dat1", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat2.sdmmc1_dat2", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat3.sdmmc1_dat3", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat4.sdmmc1_dat4", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat5.sdmmc1_dat5", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat6.sdmmc1_dat6", HSMMC1_MUX);
		omap_mux_init_signal("sdmmc1_dat7.sdmmc1_dat7", HSMMC1_MUX);
		/* hsmmc cmd */
		omap_mux_init_signal("sdmmc1_cmd.sdmmc1_cmd", HSMMC1_MUX);
		/* hsmmc clk */
		omap_mux_init_signal("sdmmc1_clk.sdmmc1_clk", HSMMC1_MUX);
	} else {
		/* hsmmc d0-d7 */
		omap_mux_init_signal("gpmc_ad0", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad1", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad2", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad3", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad4", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad5", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad6", HSMMC2_MUX);
		omap_mux_init_signal("gpmc_ad7", HSMMC2_MUX);
		/* hsmmc cmd */
		omap_mux_init_signal("gpmc_nwe", HSMMC2_MUX);
		/* hsmmc clk */
		omap_mux_init_signal("gpmc_noe", HSMMC2_MUX);

		mmc[0].mmc = 2;
	}

	if (omap4_tuna_get_revision() != TUNA_REV_PRE_LUNCHBOX) {
		gpio_request(158, "emmc_en");
		gpio_direction_output(158, 1);
		omap_mux_init_gpio(158, OMAP_PIN_OUTPUT);
	}

	if (wl12xx_set_platform_data(&tuna_wlan_data))
		pr_err("error setting wl12xx data\n");

	tuna_wlan_init();
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
