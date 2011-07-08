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
#include <linux/ion.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/omap_ion.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/reboot.h>
#include <linux/memblock.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/lte_modem_bootloader.h>
#include <plat/mcspi.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/cpu.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <mach/id.h>
#include "timer-gp.h"

#include "omap4-sar-layout.h"
#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-tuna.h"

#define TUNA_RAMCONSOLE_START	(PLAT_PHYS_OFFSET + SZ_512M)
#define TUNA_RAMCONSOLE_SIZE	SZ_2M

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

/* For LTE(CMC221) */
#define OMAP_GPIO_LTE_ACTIVE	47
#define OMAP_GPIO_CMC2AP_INT1	61

#define GPIO_AUD_PWRON		127
#define GPIO_AUD_PWRON_TORO_V1	20
#define GPIO_MICBIAS_EN	48

/* GPS GPIO Setting */
#define GPIO_AP_AGPS_TSYNC	18
#define GPIO_GPS_nRST	136
#define GPIO_GPS_PWR_EN	137
#define GPIO_GPS_UART_SEL	164

#define REBOOT_FLAG_RECOVERY	0x52564352
#define REBOOT_FLAG_FASTBOOT	0x54534146
#define REBOOT_FLAG_NORMAL	0x4D524F4E

static int tuna_hw_rev;

static struct gpio tuna_hw_rev_gpios[] = {
	{76, GPIOF_IN, "hw_rev0"},
	{75, GPIOF_IN, "hw_rev1"},
	{74, GPIOF_IN, "hw_rev2"},
	{73, GPIOF_IN, "hw_rev3"},
	{170, GPIOF_IN, "hw_rev4"},
};

static const char const *omap4_tuna_hw_name_maguro[] = {
	[0x00] = "Toro Lunchbox #1",
	[0x01] = "Maguro 1st Sample",
	[0x02] = "Maguro 2nd Sample",
	[0x05] = "Toro Pre-Lunchbox",
};

static const char const *omap4_tuna_hw_name_toro[] = {
	[0x00] = "Toro Lunchbox #2",
	[0x01] = "Toro 1st Sample",
	[0x02] = "Toro 2nd Sample",
};

int omap4_tuna_get_revision(void)
{
	return tuna_hw_rev & TUNA_REV_MASK;
}

int omap4_tuna_get_type(void)
{
	return tuna_hw_rev & TUNA_TYPE_MASK;
}


static const char *omap4_tuna_hw_rev_name(void) {
	const char *ret;
	const char **names;
	int num;
	int rev;

	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO) {
		names = omap4_tuna_hw_name_maguro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_maguro);
		ret = "Maguro unknown";
	} else {
		names = omap4_tuna_hw_name_toro;
		num = ARRAY_SIZE(omap4_tuna_hw_name_toro);
		ret = "Toro unknown";
	}

	rev = omap4_tuna_get_revision();
	if (rev >= num || !names[rev])
		return ret;

	return names[rev];
}

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

	pr_info("Tuna HW revision: %02x (%s), cpu %s\n", tuna_hw_rev,
		omap4_tuna_hw_rev_name(),
		cpu_is_omap443x() ? "OMAP4430" : "OMAP4460");
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

static struct resource ramconsole_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start	= TUNA_RAMCONSOLE_START,
		.end	= TUNA_RAMCONSOLE_START + TUNA_RAMCONSOLE_SIZE - 1,
	},
};

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
};

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

static void __init tuna_bt_init(void)
{
	/* BT_EN - GPIO 104 */
	omap_mux_init_signal("gpmc_ncs6.gpio_103", OMAP_PIN_OUTPUT);
	/*BT_nRST - GPIO 42 */
	omap_mux_init_signal("gpmc_a18.gpio_42", OMAP_PIN_OUTPUT);
	/* BT_WAKE - GPIO 27 */
	omap_mux_init_signal("dpm_emu16.gpio_27", OMAP_PIN_OUTPUT);
	/* BT_HOST_WAKE  - GPIO 177 */
	omap_mux_init_signal("kpd_row5.gpio_177", OMAP_PIN_INPUT);
}

static struct twl4030_madc_platform_data twl6030_madc = {
	.irq_line = -1,
};

static struct platform_device twl6030_madc_device = {
	.name   = "twl6030_madc",
	.id = -1,
	.dev	= {
		.platform_data	= &twl6030_madc,
	},
};

#define PHYS_ADDR_DUCATI_MEM (0x80000000 + SZ_1G - (SZ_1M * 104))

static struct ion_platform_data tuna_ion_data = {
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_DUCATI_MEM - SZ_256M - SZ_64M,
			.size = SZ_64M,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM - SZ_256M,
			.size = SZ_256M,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_LARGE_SURFACES,
			.name = "large_surfaces",
			.base = 0x80000000 + SZ_512M + SZ_2M,
			.size = SZ_32M,
		},
	},
};

static struct platform_device tuna_ion_device = {
	.name = "ion-omap4",
	.id = -1,
	.dev = {
		.platform_data = &tuna_ion_data,
	},
};

static struct platform_device *tuna_devices[] __initdata = {
	&ramconsole_device,
	&wl1271_device,
	&bcm4330_bluetooth_device,
	&twl6030_madc_device,
	&tuna_ion_device,
};

static void tuna_gsd4t_gps_gpio(void)
{
	/* AP_AGPS_TSYNC - GPIO 18 */
	omap_mux_init_signal("dpm_emu7.gpio_18", OMAP_PIN_OUTPUT);
	/* GPS_nRST - GPIO 136 */
	omap_mux_init_signal("mcspi1_simo.gpio_136", OMAP_PIN_OUTPUT);
	/* GPS_PWR_EN - GPIO 137 */
	omap_mux_init_signal("mcspi1_cs0.gpio_137", OMAP_PIN_OUTPUT);
	/* GPS_UART_SEL - GPIO 164 */
	omap_mux_init_signal("usbb2_ulpitll_dat3.gpio_164", OMAP_PIN_OUTPUT);

	if (omap4_tuna_get_revision() >= TUNA_REV_03) {
		/* GPS_UART_CTS - GPIO 139 */
		omap_mux_init_signal("mcspi1_cs2.gpio_139",
					OMAP_MUX_MODE1 | OMAP_PIN_INPUT);
		/* GPS_UART_RTS - GPIO 140 */
		omap_mux_init_signal("mcspi1_cs3.gpio_140",
					OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT);
	}
}

static void tuna_gsd4t_gps_init(void)
{
	struct device *gps_dev;

	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev)) {
		pr_err("Failed to create device(gps)!\n");
		goto err;
	}
	tuna_gsd4t_gps_gpio();

	gpio_request(GPIO_AP_AGPS_TSYNC, "AP_AGPS_TSYNC");
	gpio_direction_output(GPIO_AP_AGPS_TSYNC, 0);

	gpio_request(GPIO_GPS_nRST, "GPS_nRST");
	gpio_direction_output(GPIO_GPS_nRST, 1);

	gpio_request(GPIO_GPS_PWR_EN, "GPS_PWR_EN");
	gpio_direction_output(GPIO_GPS_PWR_EN, 0);

	gpio_request(GPIO_GPS_UART_SEL , "GPS_UART_SEL");
	gpio_direction_output(GPIO_GPS_UART_SEL , 0);

	gpio_export(GPIO_GPS_nRST, 1);
	gpio_export(GPIO_GPS_PWR_EN, 1);

	gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_nRST);
	gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN);

err:
	return;
}

static int __init sec_common_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	return 0;
}

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
		.nonremovable	= false,
		.mmc_data	= &tuna_wifi_data,
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
	.consumer_supplies = tuna_vaux3_supplies,
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

static struct regulator_consumer_supply tuna_vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "tuna_otg"),
};

static struct regulator_init_data tuna_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vusb_supply),
	.consumer_supplies	= tuna_vusb_supply,
};

static struct regulator_init_data tuna_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
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

	/* children */
	.codec		= &twl6040_codec,
	.madc		= &twl6030_madc,
};

static void tuna_audio_init(void)
{
	unsigned int aud_pwron;

	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);

	/* aud_pwron */
	if (omap4_tuna_get_type() == TUNA_TYPE_TORO &&
	    omap4_tuna_get_revision() >= 1)
		aud_pwron = GPIO_AUD_PWRON_TORO_V1;
	else
		aud_pwron = GPIO_AUD_PWRON;
	omap_mux_init_gpio(aud_pwron, OMAP_PIN_OUTPUT);
	twl6040_codec.audpwron_gpio = aud_pwron;

	omap_mux_init_gpio(GPIO_MICBIAS_EN, OMAP_PIN_OUTPUT);
	gpio_request(GPIO_MICBIAS_EN, "MICBIAS_EN");
	gpio_direction_output(GPIO_MICBIAS_EN, 1);
}

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
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT_PULLUP);

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
	/* camera gpios */
	OMAP4_MUX(MCSPI1_SOMI, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), /* gpio_135 */
	OMAP4_MUX(KPD_COL0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), /* gpio_173 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), /* gpio_43 */
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

#else
#define board_mux	NULL
#define board_wkup_mux	NULL
#endif

static inline void __init board_serial_init(void)
{
	omap_serial_init();
}

/*SPI for LTE modem bootloader*/
#define LTE_MODEM_SPI_BUS_NUM 4
#define LTE_MODEM_SPI_CS  0
#define LTE_MODEM_SPI_MAX_HZ 1500000

struct lte_modem_bootloader_platform_data lte_modem_bootloader_pdata = {
	.name = "lte_modem_int",
	.gpio_lte2ap_status = OMAP_GPIO_CMC2AP_INT1,
};

static struct omap2_mcspi_device_config lte_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info tuna_lte_modem[] __initdata = {
	{
		.modalias = "lte_modem_spi",
		.controller_data = &lte_mcspi_config,
		.platform_data = &lte_modem_bootloader_pdata,
		.max_speed_hz = LTE_MODEM_SPI_MAX_HZ,
		.bus_num = LTE_MODEM_SPI_BUS_NUM,
		.chip_select = LTE_MODEM_SPI_CS,
		.mode = SPI_MODE_0,
	},
};

static int tuna_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	void __iomem *sar_base;
	unsigned int flag = REBOOT_FLAG_NORMAL;

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);

	if (code == SYS_RESTART) {
		if (_cmd) {
			if (!strcmp(_cmd, "recovery"))
				flag = REBOOT_FLAG_RECOVERY;
			else if (!strcmp(_cmd, "bootloader"))
				flag = REBOOT_FLAG_FASTBOOT;
		}
	}

	/* The Samsung LOKE bootloader will look for the boot flag at a fixed
	 * offset from the end of the 1st SAR bank.
	 */
	writel(flag, sar_base + SAR_BANK2_OFFSET - 0xC);

	return NOTIFY_DONE;
}

static struct notifier_block tuna_reboot_notifier = {
	.notifier_call = tuna_notifier_call,
};

static ssize_t tuna_soc_family_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "OMAP%04x\n", GET_OMAP_TYPE);
}

static ssize_t tuna_soc_revision_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "ES%d.%d\n", (GET_OMAP_REVISION() >> 4) & 0xf,
		       GET_OMAP_REVISION() & 0xf);
}

static ssize_t tuna_soc_die_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_die_id(&oid);
	return sprintf(buf, "%08X-%08X-%08X-%08X\n", oid.id_3, oid.id_2,
			oid.id_1, oid.id_0);
}

static ssize_t tuna_soc_prod_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_production_id(&oid);
	return sprintf(buf, "%08X-%08X\n", oid.id_1, oid.id_0);
}

static const char *omap_types[] = {
	[OMAP2_DEVICE_TYPE_TEST]	= "TST",
	[OMAP2_DEVICE_TYPE_EMU]		= "EMU",
	[OMAP2_DEVICE_TYPE_SEC]		= "HS",
	[OMAP2_DEVICE_TYPE_GP]		= "GP",
	[OMAP2_DEVICE_TYPE_BAD]		= "BAD",
};

static ssize_t tuna_soc_type_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap_types[omap_type()]);
}

#define TUNA_SOC_ATTR_RO(_name, _show) \
	struct kobj_attribute tuna_soc_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static TUNA_SOC_ATTR_RO(family, tuna_soc_family_show);
static TUNA_SOC_ATTR_RO(revision, tuna_soc_revision_show);
static TUNA_SOC_ATTR_RO(type, tuna_soc_type_show);
static TUNA_SOC_ATTR_RO(die_id, tuna_soc_die_id_show);
static TUNA_SOC_ATTR_RO(production_id, tuna_soc_prod_id_show);

static struct attribute *tuna_soc_prop_attrs[] = {
	&tuna_soc_prop_attr_family.attr,
	&tuna_soc_prop_attr_revision.attr,
	&tuna_soc_prop_attr_type.attr,
	&tuna_soc_prop_attr_die_id.attr,
	&tuna_soc_prop_attr_production_id.attr,
	NULL,
};

static struct attribute_group tuna_soc_prop_attr_group = {
	.attrs = tuna_soc_prop_attrs,
};

static void __init omap4_tuna_create_board_props(void)
{
	struct kobject *board_props_kobj;
	struct kobject *soc_kobj;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_obj;

	soc_kobj = kobject_create_and_add("soc", board_props_kobj);
	if (!soc_kobj)
		goto err_soc_obj;

	ret = sysfs_create_group(soc_kobj, &tuna_soc_prop_attr_group);
	if (ret)
		goto err_sysfs_create;
	return;

err_sysfs_create:
	kobject_put(soc_kobj);
err_soc_obj:
	kobject_put(board_props_kobj);
err_board_obj:
	if (!board_props_kobj || !soc_kobj || ret)
		pr_err("failed to create board_properties\n");
}

#define HSMMC2_MUX	(OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP)
#define HSMMC1_MUX	OMAP_PIN_INPUT_PULLUP

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	omap4_tuna_emif_init();

	register_reboot_notifier(&tuna_reboot_notifier);

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
		omap_mux_init_gpio(158, OMAP_PIN_INPUT_PULLUP);
	}

	sec_common_init();

	if (TUNA_TYPE_TORO == omap4_tuna_get_type()) {
		omap_mux_init_signal("gpmc_wait0",
				OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN);
		gpio_request(OMAP_GPIO_CMC2AP_INT1, "gpio_61");
		gpio_direction_input(OMAP_GPIO_CMC2AP_INT1);

		omap_mux_init_signal("mcspi4_clk", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_simo", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_somi", OMAP_MUX_MODE0);
		omap_mux_init_signal("mcspi4_cs0", OMAP_MUX_MODE0);
	}

	tuna_wlan_init();
	tuna_audio_init();
	tuna_i2c_init();
	tuna_bt_init();
	tuna_gsd4t_gps_init();
	platform_add_devices(tuna_devices, ARRAY_SIZE(tuna_devices));
	board_serial_init();
	omap2_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);
	omap4_tuna_create_board_props();
	if (TUNA_TYPE_TORO == omap4_tuna_get_type()) {
		spi_register_board_info(tuna_lte_modem,
				ARRAY_SIZE(tuna_lte_modem));
	}
	omap4_tuna_display_init();
	omap4_tuna_input_init();
	omap4_tuna_nfc_init();
	omap4_tuna_power_init();
	omap4_tuna_jack_init();
	omap4_tuna_sensors_init();
	omap4_tuna_connector_init();
#ifdef CONFIG_OMAP_HSI_DEVICE
	if (TUNA_TYPE_MAGURO == omap4_tuna_get_type())
		omap_hsi_init();
#endif
#ifdef CONFIG_USB_EHCI_HCD_OMAP
	if (TUNA_TYPE_TORO == omap4_tuna_get_type())
		omap4_ehci_init();
#endif
}

static void __init tuna_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init tuna_reserve(void)
{
	int i;
	int ret;

	omap_reserve();
	memblock_remove(TUNA_RAMCONSOLE_START, TUNA_RAMCONSOLE_SIZE);

	for (i = 0; i < tuna_ion_data.nr; i++)
		if (tuna_ion_data.heaps[i].type == ION_HEAP_TYPE_CARVEOUT ||
		    tuna_ion_data.heaps[i].type == OMAP_ION_HEAP_TYPE_TILER) {
			ret = memblock_remove(tuna_ion_data.heaps[i].base,
					      tuna_ion_data.heaps[i].size);
			if (ret)
				pr_err("memblock remove of %x@%lx failed\n",
				       tuna_ion_data.heaps[i].size,
				       tuna_ion_data.heaps[i].base);
		}
}

MACHINE_START(TUNA, "Tuna")
	/* Maintainer: Google, Inc */
	.boot_params	= 0x80000100,
	.reserve	= tuna_reserve,
	.map_io		= tuna_map_io,
	.init_early	= tuna_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= tuna_init,
	.timer		= &omap_timer,
MACHINE_END
