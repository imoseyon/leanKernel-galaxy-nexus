/* linux/arch/arm/mach-xxxx/board-tuna-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
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
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/omap4-common.h>
#include <linux/platform_data/modem.h>
#include "board-tuna.h"
#include "mux.h"

#define OMAP_GPIO_MIPI_HSI_CP_ON	53
#define OMAP_GPIO_MIPI_HSI_RESET_REQ_N	50
#define OMAP_GPIO_MIPI_HSI_CP_RST	15
#define OMAP_GPIO_MIPI_HSI_PDA_ACTIVE	119
#define OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE	120
#define OMAP_GPIO_MIPI_HSI_CP_DUMP_INT	95
#define OMAP_GPIO_MIPI_HSI_GPS_UART_SEL	164

#define OMAP_GPIO_DPRAM_VIA_RST		15
#define OMAP_GPIO_DPRAM_PDA_ACTIVE	119
#define OMAP_GPIO_DPRAM_PHONE_ACTIVE	120
#define OMAP_GPIO_DPRAM_PWRHOLD_OFF	53
#define OMAP_GPIO_DPRAM_INT_N		62

#define OMAP_GPIO_CMC_SPI_CLK_ACK	178
#define OMAP_GPIO_CMC_SPI_CLK_REQ	164
#define OMAP_GPIO_CMC_SPI_WAKEUP_INT	134
#define OMAP_GPIO_LTE_ACTIVE		47
#define OMAP_GPIO_CMC2AP_INT1		61
#define OMAP_GPIO_CMC2AP_INT2		160
#define OMAP_GPIO_AP2CMC_INT1		18
#define OMAP_GPIO_AP2CMC_INT2		28
#define OMAP_GPIO_221_PMIC_PWRON	41
#define OMAP_GPIO_CMC_RST		50
#define OMAP_GPIO_221_PMIC_PWRHOLD_OFF	163

#define DPRAM_START_ADDRESS	0x04000000
#define DPRAM_SIZE              0x4000
#define DPRAM_END_ADDRESS	(DPRAM_START_ADDRESS + DPRAM_SIZE - 1)

#define GPMC_CONTROL_BASE_ADDR	0x50000000
#define GPMC_CONFIG1_1		(GPMC_CONTROL_BASE_ADDR + 0x90)
#define GPMC_CONFIG2_1		(GPMC_CONTROL_BASE_ADDR + 0x94)
#define GPMC_CONFIG3_1		(GPMC_CONTROL_BASE_ADDR + 0x98)
#define GPMC_CONFIG4_1		(GPMC_CONTROL_BASE_ADDR + 0x9C)
#define GPMC_CONFIG5_1		(GPMC_CONTROL_BASE_ADDR + 0xA0)
#define GPMC_CONFIG6_1		(GPMC_CONTROL_BASE_ADDR + 0xA4)
#define GPMC_CONFIG7_1		(GPMC_CONTROL_BASE_ADDR + 0xA8)

#define DPRAM_GPMC_CONFIG1	0x00001201
#define DPRAM_GPMC_CONFIG2	0x000f1200
#define DPRAM_GPMC_CONFIG3	0x44040400
#define DPRAM_GPMC_CONFIG4	0x0e05f155
#define DPRAM_GPMC_CONFIG5	0x000e1016
#define DPRAM_GPMC_CONFIG6	0x060603c3
#define DPRAM_GPMC_CONFIG7	0x00000F44

/* umts target platform data */
static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "umts_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_MIPI,
	},
	[1] = {
		.name = "umts_rfs0",
		.id = 0x41,
		.format = IPC_RFS,
		.io_type = IODEV_MISC,
		.link = LINKDEV_MIPI,
	},
	[2] = {
		.name = "rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_MIPI,
	},
	[3] = {
		.name = "umts_boot0",
		.id = 0x0,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_MIPI,
	},
	[4] = {
		.name = "rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_MIPI,
	},
	[5] = {
		.name = "rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_MIPI,
	},
	[6] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.link = LINKDEV_MIPI,
	},
	[7] = {
		.name = "umts_ramdump0",
		.id = 0x0,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.link = LINKDEV_MIPI,
	},
	[8] = {
		.name = "umts_boot1",
		.id = 0x1,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_MIPI,
	},
};

static struct modem_data umts_modem_data = {
	.name = "xmm6260",

	.gpio_cp_on = OMAP_GPIO_MIPI_HSI_CP_ON,
	.gpio_reset_req_n = OMAP_GPIO_MIPI_HSI_RESET_REQ_N,
	.gpio_cp_reset = OMAP_GPIO_MIPI_HSI_CP_RST,
	.gpio_pda_active = OMAP_GPIO_MIPI_HSI_PDA_ACTIVE,
	.gpio_phone_active = OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE,
	.gpio_cp_dump_int = OMAP_GPIO_MIPI_HSI_CP_DUMP_INT,
	.gpio_flm_uart_sel = OMAP_GPIO_MIPI_HSI_GPS_UART_SEL,
	.gpio_cp_warm_reset = 0,

	.modem_type = IMC_XMM6260,
	.link_type = LINKDEV_MIPI,
	.modem_net = UMTS_NETWORK,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs = umts_io_devices,
};

static void umts_modem_cfg_gpio(void)
{
	unsigned gpio_reset_req_n = umts_modem_data.gpio_reset_req_n;
	unsigned gpio_cp_on = umts_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = umts_modem_data.gpio_cp_reset;
	unsigned gpio_pda_active = umts_modem_data.gpio_pda_active;
	unsigned gpio_phone_active = umts_modem_data.gpio_phone_active;
	unsigned gpio_cp_dump_int = umts_modem_data.gpio_cp_dump_int;
	unsigned gpio_flm_uart_sel = umts_modem_data.gpio_flm_uart_sel;

	/* gpio mux setting */
	omap_mux_init_signal("gpmc_ncs0.gpio_50", OMAP_PIN_OUTPUT |
				OMAP_OFF_EN | OMAP_OFFOUT_VAL);
	omap_mux_init_signal("gpmc_ncs3.gpio_53", OMAP_PIN_OUTPUT |
				OMAP_OFF_EN);
	omap_mux_init_signal("dpm_emu4.gpio_15", OMAP_PIN_OUTPUT |
				OMAP_OFF_EN | OMAP_OFFOUT_VAL);
	omap_mux_init_signal("abe_dmic_clk1.gpio_119", OMAP_PIN_OUTPUT |
				OMAP_OFF_EN);
	omap_mux_init_signal("abe_dmic_din1.gpio_120", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("usbb1_ulpitll_dat7.gpio_95", OMAP_PIN_INPUT);
	omap_mux_init_signal("usbb2_ulpitll_dat3.gpio_164", OMAP_PIN_OUTPUT |
				OMAP_OFF_EN | OMAP_OFFOUT_VAL);
	omap_mux_init_signal("uart3_cts_rctx.uart1_tx", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi1_cs1.uart1_rx", OMAP_PIN_INPUT);

	if (gpio_reset_req_n) {
		gpio_request(gpio_reset_req_n, "RESET_REQ_N");
		gpio_direction_output(gpio_reset_req_n, 0);
	}

	if (gpio_cp_on) {
		gpio_request(gpio_cp_on, "CP_ON");
		gpio_direction_output(gpio_cp_on, 0);
	}

	if (gpio_cp_rst) {
		gpio_request(gpio_cp_rst, "CP_RST");
		gpio_direction_output(gpio_cp_rst, 0);
	}

	if (gpio_pda_active) {
		gpio_request(gpio_pda_active, "PDA_ACTIVE");
		gpio_direction_output(gpio_pda_active, 0);
	}

	if (gpio_phone_active) {
		gpio_request(gpio_phone_active, "PHONE_ACTIVE");
		gpio_direction_input(gpio_phone_active);
	}

	if (gpio_cp_dump_int) {
		gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
		gpio_direction_input(gpio_cp_dump_int);
	}

	if (gpio_flm_uart_sel) {
		gpio_request(gpio_flm_uart_sel, "GPS_UART_SEL");
		gpio_direction_output(gpio_flm_uart_sel, 1);
	}

	if (gpio_phone_active)
		irq_set_irq_type(
			OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),
						     IRQ_TYPE_LEVEL_HIGH);

	pr_debug("umts_modem_cfg_gpio done\n");
}

/* To get modem state, register phone active irq using resource */
static struct resource umts_modem_res[] = {
	[0] = {
		.name = "umts_phone_active",
		.start = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),
		.end = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),
		.flags = IORESOURCE_IRQ,
	},
};

/* if use more than one modem device, then set id num */
static struct platform_device umts_modem = {
	.name = "modem_if",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

static struct modem_io_t cdma_io_devices[] = {
	[0] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.link = LINKDEV_DPRAM,
	},
	[1] = {
		.name = "cdma_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[2] = {
		.name = "cdma_boot0",
		.id = 0x1,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[3] = {
		.name = "cdma_rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[4] = {
		.name = "cdma_rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[5] = {
		.name = "cdma_rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[6] = {
		.name = "cdma_rmnet3",
		.id = 0x2D,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[7] = {
		.name = "cdma_rmnet4",
		.id = 0x27,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[8] = {
		.name = "cdma_rmnet5", /* DM Port io-device */
		.id = 0x3A,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[9] = {
		.name = "cdma_ramdump0",
		.id = 0x1,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[10] = {
		.name = "cdma_rmnet6", /* AT CMD io-device */
		.id = 0x31,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
};

/* cdma target platform data */
static struct modem_data cdma_modem_data = {
	.name = "cbp7.1",

	/*ToDo: always power on vbat 3.3v it is not cennected GPIO*/
	.gpio_cp_on = 0,
	.gpio_reset_req_n =  0,
	.gpio_cp_reset = OMAP_GPIO_DPRAM_VIA_RST,
	.gpio_pda_active = OMAP_GPIO_DPRAM_PDA_ACTIVE,
	.gpio_phone_active = OMAP_GPIO_DPRAM_PHONE_ACTIVE,
	.gpio_cp_dump_int = 0, /*ToDo:*/
	.gpio_cp_warm_reset = 0,
	.gpio_cp_off = OMAP_GPIO_DPRAM_PWRHOLD_OFF,

	.modem_type = VIA_CBP71,
	.link_type = LINKDEV_DPRAM,
	.modem_net = CDMA_NETWORK,

	.num_iodevs = ARRAY_SIZE(cdma_io_devices),
	.iodevs = cdma_io_devices,
};

static void dpram_cfg_gpio(void)
{
	int nRetry = 0;
	int resetdone;

	omap_mux_init_signal("gpmc_ad0", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad1", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad2", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad3", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad4", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad5", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad6", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad7", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad8", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad9", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad10", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad11", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad12", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad13", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad14", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);
	omap_mux_init_signal("gpmc_ad15", OMAP_PULL_ENA|OMAP_PULL_UP|OMAP_INPUT_EN);

	omap_mux_init_signal("gpmc_nadv_ale", 0);
	omap_mux_init_signal("gpmc_nwe", 0);
	omap_mux_init_signal("gpmc_nbe0_cle", 0);
	omap_mux_init_signal("gpmc_ncs1", 0);
	omap_mux_init_signal("gpmc_nbe1", 0);

	omap_mux_init_signal("gpmc_wait1.gpio_62", OMAP_WAKEUP_EN | OMAP_INPUT_EN);
	omap_mux_init_signal("dpm_emu3", OMAP_MUX_MODE3);
	omap_mux_init_signal("gpmc_ncs3.gpio_53", OMAP_PIN_OUTPUT);

	gpio_request(OMAP_GPIO_DPRAM_INT_N, "dpram_int");
	gpio_direction_input(OMAP_GPIO_DPRAM_INT_N);
	irq_set_irq_type(OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_INT_N),
				IRQ_TYPE_LEVEL_LOW);

	/*dpram platform init setting*/
	__raw_writel(0x02, OMAP4_GPMC_IO_ADDRESS((OMAP44XX_GPMC_BASE + 0x10)));

	while (nRetry < 100) {
		msleep(20);
		resetdone = __raw_readl(OMAP4_GPMC_IO_ADDRESS(OMAP44XX_GPMC_BASE
					+ 0x14));
		if (resetdone == 0x1)
			break;

		nRetry++;
	  }

	__raw_writel(0x10, OMAP4_GPMC_IO_ADDRESS((OMAP44XX_GPMC_BASE + 0x10)));

	__raw_writel((u32)DPRAM_GPMC_CONFIG1,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG1_1));
	__raw_writel((u32)DPRAM_GPMC_CONFIG2,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG2_1));
	__raw_writel((u32)DPRAM_GPMC_CONFIG3,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG3_1));
	__raw_writel((u32)DPRAM_GPMC_CONFIG4,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG4_1));
	__raw_writel((u32)DPRAM_GPMC_CONFIG5,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG5_1));
	__raw_writel((u32)DPRAM_GPMC_CONFIG6,
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG6_1));

	__raw_writel((u32)0xF04, OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG7_1));
	msleep(50);
	__raw_writel((u32)(0xF04 | 0x040),
			OMAP4_GPMC_IO_ADDRESS(GPMC_CONFIG7_1));
}

static int dpram_cfg_gpmc_clk(void)
{
	struct clk *dpram_gpmc_ck;
	struct clk *dpram_gpmc_ick;

	dpram_gpmc_ck = clk_get(NULL, "gpmc_ck");
	if (IS_ERR(dpram_gpmc_ck)) {
		pr_err("Could not get GPMC clock gpmc_ck\n");
		return -ENOENT;
	}
	clk_enable(dpram_gpmc_ck);

	dpram_gpmc_ick = clk_get(NULL, "gpmc_ick");
	if (IS_ERR(dpram_gpmc_ick)) {
		clk_disable(dpram_gpmc_ck);
		pr_err("Could not get GPMC clock gpmc_ick\n");
		return -ENOENT;
	}
	clk_enable(dpram_gpmc_ick);

	return 0;
}

static void cdma_modem_cfg_gpio(void)
{
	unsigned gpio_cp_rst = cdma_modem_data.gpio_cp_reset;
	unsigned gpio_pda_active = cdma_modem_data.gpio_pda_active;
	unsigned gpio_phone_active = cdma_modem_data.gpio_phone_active;
	unsigned gpio_cp_off = cdma_modem_data.gpio_cp_off;

	dpram_cfg_gpio();
	if (dpram_cfg_gpmc_clk()) {
		pr_err("fail to enable GPMC clock\n");
		return;
	}

	omap_mux_init_signal("abe_dmic_din1.gpio_120", OMAP_PIN_INPUT);
	omap_mux_init_signal("abe_dmic_clk1.gpio_119", OMAP_PIN_OUTPUT |
				OMAP_PIN_OFF_OUTPUT_LOW);

	/* gpio mux setting */
	if (gpio_cp_rst) {
		gpio_request(gpio_cp_rst, "CP_RST");
		gpio_direction_output(gpio_cp_rst, 0);
	}

	if (gpio_pda_active) {
		gpio_request(gpio_pda_active, "PDA_ACTIVE");
		gpio_direction_output(gpio_pda_active, 0);
	}

	if (gpio_phone_active) {
		gpio_request(gpio_phone_active, "PHONE_ACTIVE");
		gpio_direction_input(gpio_phone_active);
	}

	if (gpio_cp_off) {
		gpio_request(gpio_cp_off, "VIA_OFF");
		gpio_direction_output(gpio_cp_off, 1);
	}

	if (gpio_phone_active)
		irq_set_irq_type(
			OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_PHONE_ACTIVE),
						     IRQ_TYPE_LEVEL_HIGH);
}

static struct resource cdma_modem_res[] = {
	[0] = {
		.name = "cdma_phone_active",
		.start = OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_PHONE_ACTIVE),
		.end = OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_PHONE_ACTIVE),
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.name = "cdma_dpram_int",
		.start = OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_INT_N),
		.end = OMAP_GPIO_IRQ(OMAP_GPIO_DPRAM_INT_N),
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "cdma_dpram",
		.start = DPRAM_START_ADDRESS,
		.end = DPRAM_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device cdma_modem = {
	.name = "modem_if",
	.id = 1,
	.num_resources = ARRAY_SIZE(cdma_modem_res),
	.resource = cdma_modem_res,
	.dev = {
		.platform_data = &cdma_modem_data,
	},
};

/* lte target platform data */
static struct modem_io_t lte_io_devices[] = {
	[0] = {
		.name = "lte_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_USB,
	},
	[1] = {
		.name = "lte_rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_USB,
	},
	[2] = {
		.name = "lte_rfs0",
		.id = 0x0,
		.format = IPC_RFS,
		.io_type = IODEV_MISC,
		.link = LINKDEV_USB,
	},
	[3] = {
		.name = "lte_boot0",
		.id = 0x0,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_USB,
	},
	[4] = {
		.name = "lte_rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_USB,
	},
	[5] = {
		.name = "lte_rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_USB,
	},
	[6] = {
		.name = "lte_rmnet3",
		.id = 0x2D,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_USB,
	},
	[7] = {
		.name = "lte_multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.link = LINKDEV_USB,
	},
	[8] = {
		.name = "lte_rmnet4", /* DM Port io-device */
		.id = 0x3F,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_USB,
	},
	[9] = {
		.name = "lte_ramdump0",
		.id = 0x0,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.link = LINKDEV_USB,
	},
};

/*
Prime vs P4 usage
CMC2AP_INT1 vs CMC2AP_STATUS
AP2CMC_INT1 vs AP2CMC_STATUS
CMC2AP_INT2 vs CMC2AP_WAKEUP
AP2CMC_INT2 vs AP2CMC_WAKEUP
*/
static struct modem_data lte_modem_data = {
	.name = "cmc221",

	.gpio_cp_on = OMAP_GPIO_221_PMIC_PWRON,
	.gpio_reset_req_n = 0,
	.gpio_cp_reset = OMAP_GPIO_CMC_RST,
	.gpio_pda_active = 0,/*NOT YET CONNECTED*/
	.gpio_phone_active = OMAP_GPIO_LTE_ACTIVE,
	.gpio_cp_dump_int = OMAP_GPIO_LTE_ACTIVE,/*TO BE CHECKED*/

	.gpio_cp_warm_reset = 0,
#ifdef CONFIG_LTE_MODEM_CMC221
	.gpio_cp_off = OMAP_GPIO_221_PMIC_PWRHOLD_OFF,
	.gpio_slave_wakeup = OMAP_GPIO_AP2CMC_INT2,
	.gpio_host_wakeup = OMAP_GPIO_CMC2AP_INT2,
	.gpio_host_active = OMAP_GPIO_AP2CMC_INT1,
#endif

	.modem_type = SEC_CMC221,
	.link_type = LINKDEV_USB,
	.modem_net = LTE_NETWORK,

	.num_iodevs = ARRAY_SIZE(lte_io_devices),
	.iodevs = lte_io_devices,
};

static void omap_lte_mux_init(void)
{
	pr_debug("[MODEM_IF] %s IN!\n", __func__);

	omap_mux_init_signal("gpmc_a17.gpio_41", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("usbb2_ulpitll_dat2.gpio_163", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs0.gpio_50", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dpm_emu7.gpio_18", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("usbb2_ulpitll_nxt.gpio_160",
			OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_signal("dpm_emu17.gpio_28", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_a23.gpio_47", OMAP_PIN_INPUT_PULLDOWN);
}

static void lte_modem_cfg_gpio(void)
{
	unsigned gpio_cp_on = lte_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = lte_modem_data.gpio_cp_reset;
	unsigned gpio_phone_active = lte_modem_data.gpio_phone_active;
#ifdef CONFIG_LTE_MODEM_CMC221
	unsigned gpio_cp_off = lte_modem_data.gpio_cp_off;
	unsigned gpio_slave_wakeup = lte_modem_data.gpio_slave_wakeup;
	unsigned gpio_host_wakeup = lte_modem_data.gpio_host_wakeup;
	unsigned gpio_host_active = lte_modem_data.gpio_host_active;
#endif

	omap_lte_mux_init();
	if (gpio_cp_on) {
		gpio_request(gpio_cp_on, "LTE_ON");
		gpio_direction_output(gpio_cp_on, 0);
	}

	if (gpio_cp_rst) {
		gpio_request(gpio_cp_rst, "LTE_RST");
		gpio_direction_output(gpio_cp_rst, 0);
	}

	if (gpio_phone_active) {
		gpio_request(gpio_phone_active, "LTE_ACTIVE");
		gpio_direction_input(gpio_phone_active);
	}

#ifdef CONFIG_LTE_MODEM_CMC221
	if (gpio_cp_off) {
		gpio_request(gpio_cp_off, "LTE_OFF");
		gpio_direction_output(gpio_cp_off, 1);
	}

	if (gpio_slave_wakeup) {
		gpio_request(gpio_slave_wakeup, "LTE_SLAVE_WAKEUP");
		gpio_direction_output(gpio_slave_wakeup, 0);
	}

	if (gpio_host_wakeup) {
		gpio_request(gpio_host_wakeup, "LTE_HOST_WAKEUP");
		gpio_direction_input(gpio_host_wakeup);
	}

	if (gpio_host_active) {
		gpio_request(gpio_host_active, "LTE_HOST_ACTIVE");
		gpio_direction_output(gpio_host_active, 1);
	}
#endif
}

static struct resource lte_modem_res[] = {
	[0] = {
		.name = "lte_phone_active",
		/* phone active irq */
		.start = OMAP_GPIO_IRQ(OMAP_GPIO_LTE_ACTIVE),
		.end = OMAP_GPIO_IRQ(OMAP_GPIO_LTE_ACTIVE),
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.name = "lte_host_wakeup",
		/* host wakeup irq */
		.start = OMAP_GPIO_IRQ(OMAP_GPIO_CMC2AP_INT2),
		.end = OMAP_GPIO_IRQ(OMAP_GPIO_CMC2AP_INT2),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device lte_modem_wake = {
	.name = "modem_lte_wake",
	.id = -1,
};

static struct platform_device lte_modem = {
	.name = "modem_if",
	.id = 2,
	.num_resources = ARRAY_SIZE(lte_modem_res),
	.resource = lte_modem_res,
	.dev = {
		.platform_data = &lte_modem_data,
	},
};

/* lte_modem_wake must be registered before the ehci driver */
void __init modem_toro_init(void)
{
	lte_modem_wake.dev.platform_data = &lte_modem_data;
	platform_device_register(&lte_modem_wake);
}

static int __init init_modem(void)
{
	pr_debug("[MODEM_IF] init_modem\n");

	switch (omap4_tuna_get_type()) {
	case TUNA_TYPE_MAGURO:	/* HSPA */
		/* umts gpios configuration */
		umts_modem_cfg_gpio();
		platform_device_register(&umts_modem);
		break;

	case TUNA_TYPE_TORO:	/* LTE */
		/* cdma gpios configuration */
		cdma_modem_cfg_gpio();
		platform_device_register(&cdma_modem);

		/* lte gpios configuration */
		lte_modem_cfg_gpio();
		platform_device_register(&lte_modem);
		break;

	default:
		break;
	}
	return 0;
}
late_initcall(init_modem);
