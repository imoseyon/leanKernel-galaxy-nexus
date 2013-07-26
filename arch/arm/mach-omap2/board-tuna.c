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
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_data/lte_modem_bootloader.h>
#include <linux/platform_data/ram_console.h>
#include <plat/mcspi.h>
#include <linux/i2c-gpio.h>
#include <linux/earlysuspend.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board-tuna-bluetooth.h>
#include <plat/omap-serial.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/cpu.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/omap-serial.h>

#include <mach/omap_fiq_debugger.h>

#include <mach/id.h>
#include "timer-gp.h"

#include "omap4-sar-layout.h"
#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-tuna.h"
#include "resetreason.h"
#include <mach/dmm.h>

#define TUNA_RAMCONSOLE_START	(PLAT_PHYS_OFFSET + SZ_512M)
#define TUNA_RAMCONSOLE_SIZE	SZ_2M

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

/* For LTE(CMC221) */
#define OMAP_GPIO_LTE_ACTIVE	47
#define OMAP_GPIO_CMC2AP_INT1	61

#define GPIO_AUD_PWRON		127
#define GPIO_AUD_PWRON_TORO_V1	20

/* GPS GPIO Setting */
#define GPIO_AP_AGPS_TSYNC	18
#define GPIO_GPS_nRST	136
#define GPIO_GPS_PWR_EN	137
#define GPIO_GPS_UART_SEL	164

#define GPIO_MHL_SCL_18V	99
#define GPIO_MHL_SDA_18V	98

#define REBOOT_FLAG_RECOVERY	0x52564352
#define REBOOT_FLAG_FASTBOOT	0x54534146
#define REBOOT_FLAG_NORMAL	0x4D524F4E
#define REBOOT_FLAG_POWER_OFF	0x46464F50

#define UART_NUM_FOR_GPS	0

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
	[0x03] = "Maguro 4th Sample",
	[0x05] = "Maguro 5th Sample",
	[0x07] = "Maguro 6th Sample",
	[0x08] = "Maguro 7th Sample",
	[0x09] = "Maguro 8th Sample",
};

static const char const *omap4_tuna_hw_name_toro[] = {
	[0x00] = "Toro Lunchbox #2",
	[0x01] = "Toro 1st Sample",
	[0x02] = "Toro 2nd Sample",
	[0x03] = "Toro 4th Sample",
	[0x05] = "Toro 5th Sample",
	[0x06] = "Toro 8th Sample",
	[0x08] = "Toro 8th Sample",
	[0x09] = "Toro 8-1th Sample",
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

/*
 * Store a handy board information string which we can use elsewhere like
 * like in panic situation
 */
static char omap4_tuna_bd_info_string[255];
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

	snprintf(omap4_tuna_bd_info_string,
		ARRAY_SIZE(omap4_tuna_bd_info_string),
		"Tuna HW revision: %02x (%s), cpu %s ES%d.%d ",
		tuna_hw_rev,
		omap4_tuna_hw_rev_name(),
		cpu_is_omap443x() ? "OMAP4430" : "OMAP4460",
		(GET_OMAP_REVISION() >> 4) & 0xf,
		GET_OMAP_REVISION() & 0xf);

	pr_info("%s\n", omap4_tuna_bd_info_string);
	mach_panic_string = omap4_tuna_bd_info_string;
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

static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
	.dev		= {
		.platform_data = &ramconsole_pdata,
	},
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
	omap_mux_init_signal("kpd_row5.gpio_177", OMAP_WAKEUP_EN |
						OMAP_PIN_INPUT_PULLDOWN);

	platform_device_register(&bcm4330_bluetooth_device);
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


static struct i2c_gpio_platform_data tuna_gpio_i2c5_pdata = {
	.sda_pin = GPIO_MHL_SDA_18V,
	.scl_pin = GPIO_MHL_SCL_18V,
	.udelay = 3,
	.timeout = 0,
};

static struct platform_device tuna_gpio_i2c5_device = {
	.name = "i2c-gpio",
	.id = 5,
	.dev = {
		.platform_data = &tuna_gpio_i2c5_pdata,
	}
};

#define PHYS_ADDR_SMC_SIZE			(SZ_1M * 3)
#define PHYS_ADDR_DUCATI_SIZE			(SZ_1M * 105)
#define OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE	(SZ_1M * 90)
#define OMAP_TUNA_ION_HEAP_TILER_SIZE		(SZ_1M * 81)
#define OMAP_TUNA_ION_HEAP_NONSECURE_TILER_SIZE	(SZ_1M * 15)

#define PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#define PHYS_ADDR_DUCATI_MEM	(PHYS_ADDR_SMC_MEM - \
				 PHYS_ADDR_DUCATI_SIZE - \
				 OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE)

static struct ion_platform_data tuna_ion_data = {
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id   = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_SMC_MEM -
					OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE,
			.size = OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id   = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP_TUNA_ION_HEAP_TILER_SIZE,
			.size = OMAP_TUNA_ION_HEAP_TILER_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id   = OMAP_ION_HEAP_NONSECURE_TILER,
			.name = "nonsecure_tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP_TUNA_ION_HEAP_TILER_SIZE -
					OMAP_TUNA_ION_HEAP_NONSECURE_TILER_SIZE,
			.size = OMAP_TUNA_ION_HEAP_NONSECURE_TILER_SIZE,
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

static struct platform_device tuna_spdif_dit_device = {
	.name		= "spdif-dit",
	.id		= 0,
};

static struct platform_device *tuna_devices[] __initdata = {
	&ramconsole_device,
	&wl1271_device,
	&twl6030_madc_device,
	&tuna_ion_device,
	&tuna_gpio_i2c5_device,
	&tuna_spdif_dit_device,
};

/*
 * The UART1 is for GPS, and CSR GPS chip should control uart1 rts level
 * for gps firmware download.
 */
static int uart1_rts_ctrl_write(struct file *file, const char __user *buffer,
						size_t count, loff_t *offs)
{
	char buf[10] = {0,};

	if (omap4_tuna_get_revision() < TUNA_REV_SAMPLE_4)
		return -ENXIO;
	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (!strncmp(buf, "1", 1)) {
		omap_rts_mux_write(OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7,
							UART_NUM_FOR_GPS);
	} else if (!strncmp(buf, "0", 1)) {
		omap_rts_mux_write(OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
							UART_NUM_FOR_GPS);
	}

	return count;
}

static const struct file_operations uart1_rts_ctrl_proc_fops = {
	.write	= uart1_rts_ctrl_write,
};

static int __init tuna_gps_rts_ctrl_init(void)
{
	struct proc_dir_entry *p_entry;

	p_entry = proc_create("mcspi1_cs3_ctrl", 0666, NULL,
						&uart1_rts_ctrl_proc_fops);

	if (!p_entry)
		return -ENOMEM;

	return 0;
}

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

	tuna_gps_rts_ctrl_init();

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
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 500,
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

static struct regulator_init_data tuna_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
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
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply tuna_vaux3_supplies[] = {
	{
		.supply = "vlcd",
	},
};

static struct regulator_init_data tuna_vaux3 = {
	.constraints = {
		.min_uV			= 3100000,
		.max_uV			= 3100000,
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
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply tuna_vusim_supplies[] = {
	{
		.supply = "vlcd-iovcc",
	},
};

static struct regulator_init_data tuna_vusim = {
	.constraints = {
		.min_uV			= 2200000,
		.max_uV 		= 2200000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(tuna_vusim_supplies),
	.consumer_supplies = tuna_vusim_supplies,
};

static struct regulator_init_data tuna_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
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
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vcxio_supply),
	.consumer_supplies	= tuna_vcxio_supply,

};

static struct regulator_consumer_supply tuna_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
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
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vdac_supply),
	.consumer_supplies	= tuna_vdac_supply,
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
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tuna_vusb_supply),
	.consumer_supplies	= tuna_vusb_supply,
};

/* clk32kg is a twl6030 32khz clock modeled as a regulator, used by GPS */
static struct regulator_init_data tuna_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct regulator_consumer_supply tuna_clk32kaudio_supply[] = {
	{
		.supply = "clk32kaudio",
	},
	{
		.supply = "twl6040_clk32k",
	}
};

static struct regulator_init_data tuna_clk32kaudio = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.boot_on                = true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tuna_clk32kaudio_supply),
	.consumer_supplies      = tuna_clk32kaudio_supply,
};


static struct regulator_init_data tuna_vdd3 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

/*
 * VMEM is unused. Register it to regulator framework and let it
 * be in disabled state.
 */
static struct regulator_init_data tuna_vmem = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data tuna_v2v1 = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step   = 0x0f,
	.hs_right_step  = 0x0f,
	.hf_left_step   = 0x1d,
	.hf_right_step  = 0x1d,
	.ep_step	= 0x0f,
};

static struct regulator *twl6040_clk32kreg;

static int tuna_twl6040_get_ext_clk32k(void)
{
	int ret = 0;

	twl6040_clk32kreg = regulator_get(NULL, "twl6040_clk32k");
	if (IS_ERR(twl6040_clk32kreg)) {
		ret = PTR_ERR(twl6040_clk32kreg);
		pr_err("failed to get CLK32K %d\n", ret);
	}

	return ret;
}

static void tuna_twl6040_put_ext_clk32k(void)
{
	regulator_put(twl6040_clk32kreg);
}

static int tuna_twl6040_set_ext_clk32k(bool on)
{
	int ret;

	if (IS_ERR_OR_NULL(twl6040_clk32kreg))
		return -EINVAL;

	if (on)
		ret = regulator_enable(twl6040_clk32kreg);
	else
		ret = regulator_disable(twl6040_clk32kreg);

	if (ret)
		pr_err("failed to enable TWL6040 CLK32K %d\n", ret);

	return ret;
}

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.get_ext_clk32k	= tuna_twl6040_get_ext_clk32k,
	.put_ext_clk32k	= tuna_twl6040_put_ext_clk32k,
	.set_ext_clk32k	= tuna_twl6040_set_ext_clk32k,
};

static struct twl4030_platform_data tuna_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &tuna_vmmc,
	.vpp		= &tuna_vpp,
	.vusim		= &tuna_vusim,
	.vana		= &tuna_vana,
	.vcxio		= &tuna_vcxio,
	.vdac		= &tuna_vdac,
	.vusb		= &tuna_vusb,
	.vaux1		= &tuna_vaux1,
	.vaux2		= &tuna_vaux2,
	.vaux3		= &tuna_vaux3,
	.clk32kg	= &tuna_clk32kg,
	.clk32kaudio	= &tuna_clk32kaudio,

	/* children */
	.codec		= &twl6040_codec,
	.madc		= &twl6030_madc,

	/* SMPS */
	.vdd3		= &tuna_vdd3,
	.vmem		= &tuna_vmem,
	.v2v1		= &tuna_v2v1,
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

	omap_mux_init_signal("gpmc_a24.gpio_48", OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
	omap_mux_init_signal("kpd_col3.gpio_171", OMAP_PIN_OUTPUT | OMAP_MUX_MODE3);
}

static struct i2c_board_info __initdata tuna_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &tuna_twldata,
	},
};

static struct i2c_board_info __initdata tuna_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("ducati", 0x20),
		.irq = OMAP44XX_IRQ_I2C2,
		.ext_master = true,
	},
};

static struct i2c_board_info __initdata tuna_i2c4_boardinfo[] = {
	{
		I2C_BOARD_INFO("an30259a", 0x30),
	},
};

static int __init tuna_i2c_init(void)
{
	u32 r;

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT_PULLUP);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, tuna_i2c1_boardinfo,
			      ARRAY_SIZE(tuna_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, tuna_i2c2_boardinfo,
                              ARRAY_SIZE(tuna_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);

	/* Disable internal pullup on i2c.4 line:
	 * as external 2.2K is already present
	 */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0);
	r |= (1 << OMAP4_I2C4_SDA_PULLUPRESX_SHIFT);
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0);

	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* camera gpios */
	OMAP4_MUX(MCSPI1_SOMI,
		OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), /* gpio_135 */
	OMAP4_MUX(KPD_COL0,
		OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), /* gpio_173 */
	OMAP4_MUX(GPMC_A19,
		OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), /* gpio_43 */
	/* hwrev */
	OMAP4_MUX(CSI21_DY4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DY3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(CSI21_DX3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	OMAP4_MUX(USBB2_HSIC_STROBE, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* fRom */
	OMAP4_MUX(USBB2_ULPITLL_DAT4,
		  OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* mcpsi3_somi */
	OMAP4_MUX(USBB2_ULPITLL_DAT5,
		  OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN), /* mcpsi3_cs0 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6,
		  OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* mcpsi3_simo */
	OMAP4_MUX(USBB2_ULPITLL_DAT7,
		  OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* mcpsi3_clk */
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

/* sample4+ adds gps rts/cts lines */
static struct omap_device_pad tuna_uart1_pads_sample4[] __initdata = {
	{
		.name	= "mcspi1_cs3.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs2.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
	{
		.name   = "uart3_cts_rctx.uart1_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name   = "mcspi1_cs1.uart1_rx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad tuna_uart1_pads[] __initdata = {
	{
		.name   = "uart3_cts_rctx.uart1_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name   = "mcspi1_cs1.uart1_rx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
		.idle   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad tuna_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tuna_uart3_pads[] __initdata = {
	{
		.name   = "uart3_tx_irtx.uart3_tx_irtx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart3_rx_irrx.uart3_rx_irrx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tuna_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_uart_port_info tuna_uart2_info __initdata = {
	.use_dma	= 0,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.auto_sus_timeout = 0,
	.wake_peer	= bcm_bt_lpm_exit_lpm_locked,
	.rts_mux_driver_control = 1,
};

static inline void __init board_serial_init(void)
{
	struct omap_device_pad *uart1_pads;
	int uart1_pads_sz;

	if (omap4_tuna_get_revision() >= TUNA_REV_SAMPLE_4) {
		uart1_pads = tuna_uart1_pads_sample4;
		uart1_pads_sz = ARRAY_SIZE(tuna_uart1_pads_sample4);
	} else {
		uart1_pads = tuna_uart1_pads;
		uart1_pads_sz = ARRAY_SIZE(tuna_uart1_pads);
	}

	omap_serial_init_port_pads(0, uart1_pads, uart1_pads_sz, NULL);
	omap_serial_init_port_pads(1, tuna_uart2_pads,
		ARRAY_SIZE(tuna_uart2_pads), &tuna_uart2_info);
	omap_serial_init_port_pads(3, tuna_uart4_pads,
				   ARRAY_SIZE(tuna_uart4_pads), NULL);
}

/* fiq_debugger initializes really early but OMAP resource mgmt
 * is not yet ready @ arch_init, so init the serial debugger later */
static int __init board_serial_debug_init(void)
{
	return omap_serial_debug_init(2, false, true,
			tuna_uart3_pads, ARRAY_SIZE(tuna_uart3_pads));
}
device_initcall(board_serial_debug_init);

/* SPI flash memory in camera module */
#define F_ROM_SPI_BUS_NUM	3
#define F_ROM_SPI_CS		0
#define F_ROM_SPI_SPEED_HZ	24000000

static const struct flash_platform_data w25q80_pdata = {
	.name = "w25q80",
	.type = "w25q80",
};

static struct omap2_mcspi_device_config f_rom_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
	.swap_datalines	= 1,
};

static struct spi_board_info tuna_f_rom[] __initdata = {
	{
		.modalias = "m25p80",
		.controller_data = &f_rom_mcspi_config,
		.platform_data = &w25q80_pdata,
		.bus_num = F_ROM_SPI_BUS_NUM,
		.chip_select = F_ROM_SPI_CS,
		.max_speed_hz = F_ROM_SPI_SPEED_HZ,
		.mode = SPI_MODE_0,
	},
};

static void tuna_from_init(void)
{
	int err;

	if (tuna_hw_rev >= 0x07)
		f_rom_mcspi_config.swap_datalines = 0;

	err = spi_register_board_info(tuna_f_rom, ARRAY_SIZE(tuna_f_rom));
	if (err)
		pr_err("failed to register SPI F-ROM\n");
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
	} else if (code == SYS_POWER_OFF) {
		flag = REBOOT_FLAG_POWER_OFF;
	}

	/* The Samsung LOKE bootloader will look for the boot flag at a fixed
	 * offset from the end of the 1st SAR bank.
	 */
	writel(flag, sar_base + SAR_BANK2_OFFSET - 0xC);

#if defined(CONFIG_DSSCOMP) && defined(CONFIG_EARLYSUSPEND)
	/*
	 * HACK: Blank screen to avoid screen artifacts due to removal of
	 * DSS/panel drivers shutdown in reboot path.
	 */
	{
		extern void dsscomp_early_suspend(struct early_suspend *h);

		dsscomp_early_suspend(NULL);
	}
#endif

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

static ssize_t tuna_soc_msv_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%08X\n", omap_ctrl_readl(0x013c));
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

#define TUNA_ATTR_RO(_type, _name, _show) \
	struct kobj_attribute tuna_##_type##_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static TUNA_ATTR_RO(soc, family, tuna_soc_family_show);
static TUNA_ATTR_RO(soc, revision, tuna_soc_revision_show);
static TUNA_ATTR_RO(soc, type, tuna_soc_type_show);
static TUNA_ATTR_RO(soc, die_id, tuna_soc_die_id_show);
static TUNA_ATTR_RO(soc, production_id, tuna_soc_prod_id_show);
static TUNA_ATTR_RO(soc, msv, tuna_soc_msv_show);

static struct attribute *tuna_soc_prop_attrs[] = {
	&tuna_soc_prop_attr_family.attr,
	&tuna_soc_prop_attr_revision.attr,
	&tuna_soc_prop_attr_type.attr,
	&tuna_soc_prop_attr_die_id.attr,
	&tuna_soc_prop_attr_production_id.attr,
	&tuna_soc_prop_attr_msv.attr,
	NULL,
};

static struct attribute_group tuna_soc_prop_attr_group = {
	.attrs = tuna_soc_prop_attrs,
};

static ssize_t tuna_board_revision_show(struct kobject *kobj,
	 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s (0x%02x)\n", omap4_tuna_hw_rev_name(),
		tuna_hw_rev);
}

static TUNA_ATTR_RO(board, revision, tuna_board_revision_show);
static struct attribute *tuna_board_prop_attrs[] = {
	&tuna_board_prop_attr_revision.attr,
	NULL,
};

static struct attribute_group tuna_board_prop_attr_group = {
	.attrs = tuna_board_prop_attrs,
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

	ret = sysfs_create_group(board_props_kobj, &tuna_board_prop_attr_group);
	if (ret)
		goto err_board_sysfs_create;

	ret = sysfs_create_group(soc_kobj, &tuna_soc_prop_attr_group);
	if (ret)
		goto err_soc_sysfs_create;

	return;

err_soc_sysfs_create:
	sysfs_remove_group(board_props_kobj, &tuna_board_prop_attr_group);
err_board_sysfs_create:
	kobject_put(soc_kobj);
err_soc_obj:
	kobject_put(board_props_kobj);
err_board_obj:
	if (!board_props_kobj || !soc_kobj || ret)
		pr_err("failed to create board_properties\n");
}

#define HSMMC2_MUX	(OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP)
#define HSMMC1_MUX	OMAP_PIN_INPUT_PULLUP

static void __init omap4_tuna_led_init(void)
{
	i2c_register_board_info(4, tuna_i2c4_boardinfo,
				ARRAY_SIZE(tuna_i2c4_boardinfo));
}

/* always reboot into charger mode on "power-off". Let the bootloader
 * figure out if we should truly power-off or not.
 */
static void tuna_power_off(void)
{
	printk(KERN_EMERG "Rebooting into bootloader for power-off.\n");
	arm_pm_restart('c', NULL);
}

static void __init tuna_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);

	omap4_tuna_init_hw_rev();

	omap4_tuna_emif_init();

	pm_power_off = tuna_power_off;

	register_reboot_notifier(&tuna_reboot_notifier);

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

	gpio_request(158, "emmc_en");
	gpio_direction_output(158, 1);
	omap_mux_init_gpio(158, OMAP_PIN_INPUT_PULLUP);

	omap_mux_init_gpio(GPIO_MHL_SDA_18V, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(GPIO_MHL_SCL_18V, OMAP_PIN_INPUT_PULLUP);

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
	tuna_gsd4t_gps_init();
	ramconsole_pdata.bootinfo = omap4_get_resetreason();
	platform_add_devices(tuna_devices, ARRAY_SIZE(tuna_devices));
	board_serial_init();
	tuna_bt_init();
	omap2_hsmmc_init(mmc);
	usb_musb_init(&musb_board_data);
	omap4_tuna_create_board_props();
	if (TUNA_TYPE_TORO == omap4_tuna_get_type()) {
		spi_register_board_info(tuna_lte_modem,
				ARRAY_SIZE(tuna_lte_modem));
	}
	tuna_from_init();
	omap_dmm_init();
	omap4_tuna_display_init();
	omap4_tuna_input_init();
	omap4_tuna_nfc_init();
	omap4_tuna_power_init();
	omap4_tuna_jack_init();
	omap4_tuna_sensors_init();
	omap4_tuna_led_init();
	omap4_tuna_connector_init();
	omap4_tuna_pogo_init();
#ifdef CONFIG_OMAP_HSI_DEVICE
	if (TUNA_TYPE_MAGURO == omap4_tuna_get_type())
		omap_hsi_init();
#endif
#ifdef CONFIG_USB_EHCI_HCD_OMAP
	if (TUNA_TYPE_TORO == omap4_tuna_get_type()) {
#ifdef CONFIG_SEC_MODEM
		modem_toro_init();
#endif
		omap4_ehci_init();
	}
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

	/* do the static reservations first */
	memblock_remove(TUNA_RAMCONSOLE_START, TUNA_RAMCONSOLE_SIZE);
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);

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

	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP_TUNA_ION_HEAP_SECURE_INPUT_SIZE);
	omap_reserve();
}

#define PHOENIX_LAST_TURNOFF_STS	0x22

static int __init tuna_print_last_turnon_sts(void)
{
	u8 turnon_sts;
	int err;

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &turnon_sts,
			PHOENIX_LAST_TURNOFF_STS);

	if (!err) {
		pr_info("PHOENIX_LAST_TURNOFF_STS: 0x%02x\n", turnon_sts);
		twl_i2c_write_u8(TWL6030_MODULE_ID0, turnon_sts,
				 PHOENIX_LAST_TURNOFF_STS);
	}

	return 0;
}
device_initcall(tuna_print_last_turnon_sts);

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
