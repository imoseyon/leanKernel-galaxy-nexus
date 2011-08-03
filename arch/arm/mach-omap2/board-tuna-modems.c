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

#include <mach/omap4-common.h>
#include <linux/platform_data/dpram.h>
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
#define OMAP_GPMC_DPRAM_PAD_CONFIG	0x00000118

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

#define OMAP2_PA_MODEMIF		0x4000000
#define DP_ACCESS_ENBLE			0x2

#define DP_HEAD_SIZE			0x2
#define DP_TAIL_SIZE			0x2

#define DP_FMT_OUT_BUFF_SIZE		2044
#define DP_RAW_OUT_BUFF_SIZE		6128
#define DP_FMT_IN_BUFF_SIZE		2044
#define DP_RAW_IN_BUFF_SIZE		6128

#define MBXCP2AP_OFFSET			0x3FFC
#define MBXAP2CP_OFFSET			0x3FFE
#define MBXCP2AP_SIZE			0x2
#define MBXAP2CP_SIZE			0x2

#define DP_CTL_START_ADDRESS	OMAP2_PA_MODEMIF
#define DP_CTL_END_ADDRESS	(OMAP2_PA_MODEMIF + DPRAM_MAGIC_CODE_SIZE +\
					DP_ACCESS_ENBLE - 1)
#define DP_FMT_OUT_START_ADDRESS	(DP_CTL_END_ADDRESS + 1)
#define DP_FMT_OUT_END_ADDRESS	(DP_FMT_OUT_START_ADDRESS + DP_HEAD_SIZE +\
					DP_TAIL_SIZE + DP_FMT_OUT_BUFF_SIZE - 1)

#define DP_RAW_OUT_START_ADDRESS	(DP_FMT_OUT_END_ADDRESS + 1)
#define DP_RAW_OUT_END_ADDRESS	(DP_RAW_OUT_START_ADDRESS + DP_HEAD_SIZE +\
					DP_TAIL_SIZE + DP_RAW_OUT_BUFF_SIZE - 1)

#define DP_FMT_IN_START_ADDRESS	(DP_RAW_OUT_END_ADDRESS + 1)
#define DP_FMT_IN_END_ADDRESS	(DP_FMT_IN_START_ADDRESS + DP_HEAD_SIZE +\
					DP_TAIL_SIZE + DP_FMT_IN_BUFF_SIZE - 1)

#define DP_RAW_IN_START_ADDRESS	(DP_FMT_IN_END_ADDRESS + 1)
#define DP_RAW_IN_END_ADDRESS	(DP_RAW_IN_START_ADDRESS + DP_HEAD_SIZE +\
					DP_TAIL_SIZE + DP_RAW_IN_BUFF_SIZE - 1)

#define DP_MBX_START_ADDRESS	(OMAP2_PA_MODEMIF + MBXCP2AP_OFFSET)
#define DP_MBX_END_ADDRESS	(DP_MBX_START_ADDRESS + MBXCP2AP_SIZE +\
					MBXAP2CP_SIZE - 1)

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
	omap_mux_init_signal("gpmc_ncs0.gpio_50", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs3.gpio_53", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dpm_emu4.gpio_15", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_dmic_clk1.gpio_119", OMAP_PIN_OUTPUT |
				OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("abe_dmic_din1.gpio_120", OMAP_PIN_INPUT);
	omap_mux_init_signal("usbb1_ulpitll_dat7.gpio_95", OMAP_PIN_INPUT);
	omap_mux_init_signal("usbb2_ulpitll_dat3.gpio_164", OMAP_PIN_OUTPUT);
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
		gpio_direction_output(gpio_reset_req_n, 1);
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
	[6] = {
		.name = "cdma_rmnet4",
		.id = 0x27,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[7] = {
		.name = "cdma_rmnet5", /* DM Port io-device */
		.id = 0x3A,
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

	.modem_type = VIA_CBP71,
	.link_type = LINKDEV_DPRAM,
	.modem_net = CDMA_NETWORK,

	.num_iodevs = ARRAY_SIZE(cdma_io_devices),
	.iodevs = cdma_io_devices,
};

static void dpram_mux_setting(u16 offset, int option, u32 val)
{
	void __iomem *gpio_reg_addr;
	u32 reg_val;

	gpio_reg_addr = (void __iomem *)(OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE
					+ offset);
	reg_val = omap_readl((u32)gpio_reg_addr);

	reg_val &= option ? 0xFFFF0000 : 0x0000FFFF;

	reg_val |= val;
	omap_writel(reg_val, (u32)gpio_reg_addr);
}

static void dpram_cfg_gpio(void)
{
	int nRetry = 0;
	int resetdone;

	/*dpram_init_HW_setting*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD0_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD0*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD0_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD1*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD2_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD2*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD2_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD3*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD4_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD4*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD4_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD5*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD6_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD6*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD6_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD7*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD8_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD8*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD8_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD9*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD10_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD10*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD10_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD11*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD12_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD12*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD12_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD13*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD14_OFFSET,
			1, OMAP_GPMC_DPRAM_PAD_CONFIG); /*AD14*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_AD14_OFFSET,
			0, (OMAP_GPMC_DPRAM_PAD_CONFIG << 16)); /*AD15*/

	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_NADV_ALE_OFFSET, 1, 0x0);
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_NWE_OFFSET, 0, 0x0);
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_NCS0_OFFSET, 0, 0x0);
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_NBE1_OFFSET, 1, 0x0);
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_NWE_OFFSET, 1, 0x0);

	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_GPMC_WAIT1_OFFSET,
					1, 0x00000103); /*0x008c*/
	dpram_mux_setting(OMAP4_CTRL_MODULE_PAD_DPM_EMU3_OFFSET,
					0, 0x00030000); /*0x01b4*/

	gpio_request(GPIO_DPRAM_INT_N, "dpram_int");
	gpio_direction_input(GPIO_DPRAM_INT_N);
	irq_set_irq_type(OMAP_GPIO_IRQ(GPIO_DPRAM_INT_N),
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

	dpram_cfg_gpio();
	if (dpram_cfg_gpmc_clk()) {
		pr_err("fail to enable GPMC clock\n");
		return;
	}

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
		.start = 0,  /* dpram int */
		.end = 0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "cdma_dpram_ctl",
		.start = DP_CTL_START_ADDRESS,
		.end = DP_CTL_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name = "cdma_dpram_fmt_out",
		.start = DP_FMT_OUT_START_ADDRESS,
		.end = DP_FMT_OUT_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name = "cdma_dpram_raw_out",
		.start = DP_RAW_OUT_START_ADDRESS,
		.end = DP_RAW_OUT_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[5] = {
		.name = "cdma_dpram_fmt_in",
		.start = DP_FMT_IN_START_ADDRESS,
		.end = DP_FMT_IN_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[6] = {
		.name = "cdma_dpram_raw_in",
		.start = DP_RAW_IN_START_ADDRESS,
		.end = DP_RAW_IN_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	[7] = {
		.name = "cdma_dpram_mbx",
		.start = DP_MBX_START_ADDRESS,
		.end = DP_MBX_END_ADDRESS,
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

	omap_mux_init_gpio(OMAP_GPIO_221_PMIC_PWRON, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_GPIO_221_PMIC_PWRHOLD_OFF , OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_GPIO_CMC_RST, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_GPIO_AP2CMC_INT1, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_GPIO_CMC2AP_INT2,
			OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(OMAP_GPIO_AP2CMC_INT2, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_GPIO_LTE_ACTIVE, OMAP_PIN_INPUT);
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
		gpio_direction_output(gpio_cp_off, 0);
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
		gpio_direction_output(gpio_host_active, 0);
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

static struct platform_device lte_modem = {
	.name = "modem_if",
	.id = 2,
	.num_resources = ARRAY_SIZE(lte_modem_res),
	.resource = lte_modem_res,
	.dev = {
		.platform_data = &lte_modem_data,
	},
};

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
