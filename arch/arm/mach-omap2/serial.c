/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Major rework for PM support by Kevin Hilman
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <plat/omap-serial.h>
#include <plat/common.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/dma.h>
#include <plat/omap_device.h>
#include <plat/omap-pm.h>

#include "prm2xxx_3xxx.h"
#include "pm.h"
#include "cm2xxx_3xxx.h"
#include "prm-regbits-34xx.h"
#include "control.h"
#include "mux.h"

#define MAX_UART_HWMOD_NAME_LEN		16

static int omap_uart_con_id __initdata = -1;

static struct omap_uart_port_info omap_serial_default_info[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	},
};

static int uart_idle_hwmod(struct omap_device *od)
{
	omap_hwmod_idle(od->hwmods[0]);

	return 0;
}

static int uart_enable_hwmod(struct omap_device *od)
{
	omap_hwmod_enable(od->hwmods[0]);

	return 0;
}

static struct omap_device_pm_latency omap_uart_latency[] = {
	{
		.deactivate_func = uart_idle_hwmod,
		.activate_func	 = uart_enable_hwmod,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

#ifdef CONFIG_OMAP_MUX
static struct omap_device_pad default_uart1_pads[] __initdata = {
	{
		.name	= "uart1_cts.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rts.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad default_uart2_pads[] __initdata = {
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
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad default_uart3_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rts_sd.uart3_rts_sd",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad default_omap36xx_uart4_pads[] __initdata = {
	{
		.name   = "gpmc_wait2.uart4_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "gpmc_wait3.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE2,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE2,
	},
};

static struct omap_device_pad default_omap4_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};
#else
static struct omap_device_pad default_uart1_pads[] __initdata = {};
static struct omap_device_pad default_uart2_pads[] __initdata = {};
static struct omap_device_pad default_uart3_pads[] __initdata = {};
static struct omap_device_pad default_omap36xx_uart4_pads[] __initdata = {};
static struct omap_device_pad default_omap4_uart4_pads[] __initdata = {};
#endif

static __init void omap_serial_fill_default_pads(struct omap_board_data *bdata)
{
	BUG_ON(!cpu_is_omap44xx() && !cpu_is_omap34xx());

	switch (bdata->id) {
	case 0:
		bdata->pads = default_uart1_pads;
		bdata->pads_cnt = ARRAY_SIZE(default_uart1_pads);
		break;
	case 1:
		bdata->pads = default_uart2_pads;
		bdata->pads_cnt = ARRAY_SIZE(default_uart2_pads);
		break;
	case 2:
		bdata->pads = default_uart3_pads;
		bdata->pads_cnt = ARRAY_SIZE(default_uart3_pads);
		break;
	case 3:
		if (cpu_is_omap44xx()) {
			bdata->pads = default_omap4_uart4_pads;
			bdata->pads_cnt =
				ARRAY_SIZE(default_omap4_uart4_pads);
		} else {
			bdata->pads = default_omap36xx_uart4_pads;
			bdata->pads_cnt =
				ARRAY_SIZE(default_omap36xx_uart4_pads);
		}
		break;
	default:
		break;
	}
}

/* TBD: Will be removed once we have irq-chaing mechanism */
static bool omap_uart_chk_wakeup(struct platform_device *pdev)
{
	struct omap_uart_port_info *up = pdev->dev.platform_data;
	struct omap_device *od;
	u32 wkst = 0;
	bool ret = false;

	od = to_omap_device(pdev);
	if (omap_hwmod_pad_get_wakeup_status(od->hwmods[0]))
		ret = true;

	if (up->wk_st && up->wk_en && up->wk_mask) {
		/* Check for normal UART wakeup (and clear it) */
		wkst = __raw_readl(up->wk_st) & up->wk_mask;
		if (wkst) {
			__raw_writel(wkst, up->wk_st);
			ret = true;
		}
	}

	return ret;
}

static void omap_uart_wakeup_enable(struct platform_device *pdev, bool enable)
{
	struct omap_device *od;

	od = to_omap_device(pdev);
	if (enable)
		omap_hwmod_enable_wakeup(od->hwmods[0]);
	else
		omap_hwmod_disable_wakeup(od->hwmods[0]);
}

static void omap_uart_idle_init(struct omap_uart_port_info *uart,
				unsigned short num)
{
	if (cpu_is_omap44xx()) {
		uart->wer |= OMAP4_UART_WER_MOD_WKUP;
	} else if (cpu_is_omap34xx()) {
		u32 mod = num > 1 ? OMAP3430_PER_MOD : CORE_MOD;
		u32 wk_mask = 0;

		uart->wer |= OMAP2_UART_WER_MOD_WKUP;
		uart->wk_en = OMAP34XX_PRM_REGADDR(mod, PM_WKEN1);
		uart->wk_st = OMAP34XX_PRM_REGADDR(mod, PM_WKST1);
		switch (num) {
		case 0:
			wk_mask = OMAP3430_ST_UART1_MASK;
			break;
		case 1:
			wk_mask = OMAP3430_ST_UART2_MASK;
			break;
		case 2:
			wk_mask = OMAP3430_ST_UART3_MASK;
			break;
		case 3:
			wk_mask = OMAP3630_ST_UART4_MASK;
			break;
		}
		uart->wk_mask = wk_mask;
	} else if (cpu_is_omap24xx()) {
		u32 wk_mask = 0;
		u32 wk_en = PM_WKEN1, wk_st = PM_WKST1;

		switch (num) {
		case 0:
			wk_mask = OMAP24XX_ST_UART1_MASK;
			break;
		case 1:
			wk_mask = OMAP24XX_ST_UART2_MASK;
			break;
		case 2:
			wk_en = OMAP24XX_PM_WKEN2;
			wk_st = OMAP24XX_PM_WKST2;
			wk_mask = OMAP24XX_ST_UART3_MASK;
			break;
		}
		uart->wk_mask = wk_mask;
		if (cpu_is_omap2430()) {
			uart->wk_en = OMAP2430_PRM_REGADDR(CORE_MOD, wk_en);
			uart->wk_st = OMAP2430_PRM_REGADDR(CORE_MOD, wk_st);
		} else if (cpu_is_omap2420()) {
			uart->wk_en = OMAP2420_PRM_REGADDR(CORE_MOD, wk_en);
			uart->wk_st = OMAP2420_PRM_REGADDR(CORE_MOD, wk_st);
		}
	} else {
		uart->wk_en = NULL;
		uart->wk_st = NULL;
		uart->wk_mask = 0;
	}
}

char *cmdline_find_option(char *str)
{
	extern char *saved_command_line;

	return strstr(saved_command_line, str);
}

struct omap_hwmod *omap_uart_hwmod_lookup(int num)
{
	struct omap_hwmod *oh;
	char oh_name[MAX_UART_HWMOD_NAME_LEN];

	snprintf(oh_name, MAX_UART_HWMOD_NAME_LEN, "uart%d", num + 1);
	oh = omap_hwmod_lookup(oh_name);
	WARN(IS_ERR(oh), "Could not lookup hmwod info for %s\n",
					oh_name);
	return oh;
}

void omap_rts_mux_write(u16 val, int num)
{
	struct omap_hwmod *oh;
	int i;

	oh = omap_uart_hwmod_lookup(num);
	if (!oh)
		return;

	for (i = 0; i < oh->mux->nr_pads ; i++) {
		if (strstr(oh->mux->pads[i].name, "rts")) {
			omap_mux_write(oh->mux->pads[i].partition,
					val,
					oh->mux->pads[i].mux[0].reg_offset);
			break;
		}
	}
}

static int __init omap_serial_early_init(void)
{
	int i = 0;
	char omap_tty_name[MAX_UART_HWMOD_NAME_LEN];
	struct omap_hwmod *oh;

	for (i = 0; i < OMAP_MAX_HSUART_PORTS; i++) {
		snprintf(omap_tty_name, MAX_UART_HWMOD_NAME_LEN,
			"%s%d", OMAP_SERIAL_NAME, i);
		if (cmdline_find_option(omap_tty_name)) {
			omap_uart_con_id = i;
			oh = omap_uart_hwmod_lookup(i);
			oh->flags |= HWMOD_INIT_NO_IDLE | HWMOD_INIT_NO_RESET;
			return 0;
		}
	}
	return 0;
}
core_initcall(omap_serial_early_init);

void __init omap_serial_init_port_pads(int id, struct omap_device_pad *pads,
	int size, struct omap_uart_port_info *info)
{
	struct omap_board_data bdata;

	bdata.id = id;
	bdata.flags = 0;
	bdata.pads = pads;
	bdata.pads_cnt = size;

	if (!bdata.pads)
		omap_serial_fill_default_pads(&bdata);

	omap_serial_init_port(&bdata, info);
}

/**
 * omap_serial_init_port() - initialize single serial port
 * @bdata: port specific board data pointer
 * @info: platform specific data pointer
 *
 * This function initialies serial driver for given port only.
 * Platforms can call this function instead of omap_serial_init()
 * if they don't plan to use all available UARTs as serial ports.
 *
 * Don't mix calls to omap_serial_init_port() and omap_serial_init(),
 * use only one of the two.
 */
void __init omap_serial_init_port(struct omap_board_data *bdata,
			struct omap_uart_port_info *info)
{
	struct omap_hwmod *oh;
	struct omap_device *od;
	struct omap_uart_port_info *pdata;
	char *name = DRIVER_NAME;

	if (WARN_ON(!bdata))
		return;
	if (WARN_ON(bdata->id < 0))
		return;
	if (WARN_ON(bdata->id >= OMAP_MAX_HSUART_PORTS))
		return;

	oh = omap_uart_hwmod_lookup(bdata->id);
	if (!oh)
		return;

	if (info == NULL)
		info = omap_serial_default_info;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Memory allocation for UART pdata failed\n");
		return;
	}

	if (cpu_is_omap34xx() || cpu_is_omap44xx())
		pdata->errata |= UART_ERRATA_i202_MDR1_ACCESS;

	omap_uart_idle_init(pdata, bdata->id);

	pdata->uartclk = OMAP24XX_BASE_BAUD * 16;
	pdata->flags = UPF_BOOT_AUTOCONF;
	pdata->enable_wakeup = omap_uart_wakeup_enable;
	pdata->use_dma = info->use_dma;
	pdata->chk_wakeup = omap_uart_chk_wakeup;
	pdata->dma_rx_buf_size = info->dma_rx_buf_size;
	pdata->dma_rx_poll_rate = info->dma_rx_poll_rate;
	pdata->dma_rx_timeout = info->dma_rx_timeout;
	pdata->auto_sus_timeout = info->auto_sus_timeout;
	pdata->wake_peer = info->wake_peer;
	pdata->rts_mux_driver_control = info->rts_mux_driver_control;
	if (bdata->id == omap_uart_con_id) {
		pdata->console_uart = true;
#ifdef CONFIG_DEBUG_LL
		pdata->auto_sus_timeout = -1;
#endif
	}

	if (pdata->use_dma &&
			cpu_is_omap44xx() && omap_rev() > OMAP4430_REV_ES1_0)
		pdata->errata |= OMAP4_UART_ERRATA_i659_TX_THR;

	od = omap_device_build(name, bdata->id, oh, pdata,
				sizeof(*pdata), omap_uart_latency,
				ARRAY_SIZE(omap_uart_latency), false);
	WARN(IS_ERR(od), "Could not build omap_device for %s: %s.\n",
	     name, oh->name);

	oh->mux = omap_hwmod_mux_init(bdata->pads, bdata->pads_cnt);

	if (((cpu_is_omap34xx() || cpu_is_omap44xx()) && bdata->pads) ||
	    (pdata->wk_en && pdata->wk_mask)) {
		device_init_wakeup(&od->pdev.dev, true);
	}

	kfree(pdata);
}

/**
 * omap_serial_board_init() - initialize all supported serial ports
 * @platform_data: platform specific data pointer
 *
 * Initializes all available UARTs as serial ports. Platforms
 * can call this function when they want to have default behaviour
 * for serial ports (e.g initialize them all as serial ports).
 */
void __init omap_serial_board_init(struct omap_uart_port_info *platform_data)
{
	struct omap_board_data bdata;
	u8 i;

	for (i = 0; i < OMAP_MAX_HSUART_PORTS; i++) {
		bdata.id = i;
		bdata.flags = 0;
		bdata.pads = NULL;
		bdata.pads_cnt = 0;

		if (cpu_is_omap44xx() || cpu_is_omap34xx())
			omap_serial_fill_default_pads(&bdata);

		if (platform_data == NULL)
			omap_serial_init_port(&bdata, NULL);
		else
			omap_serial_init_port(&bdata, &platform_data[i]);
	}
}

/**
 * omap_serial_init() - initialize all supported serial ports
 *
 * Initializes all available UARTs.
 * Platforms can call this function when they want to have default behaviour
 * for serial ports (e.g initialize them all as serial ports).
 */
void __init omap_serial_init(void)
{
	omap_serial_board_init(NULL);
}
