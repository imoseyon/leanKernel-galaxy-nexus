/*
 * Serial Debugger Interface for Omap
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stacktrace.h>
#include <linux/uaccess.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/omap-serial.h>

#include <asm/fiq_debugger.h>

#include <mach/omap_fiq_debugger.h>
#include <mach/system.h>

#include "mux.h"

struct omap_fiq_debugger {
	struct fiq_debugger_pdata pdata;
	struct platform_device *pdev;
	void __iomem *debug_port_base;
	bool suspended;
	spinlock_t lock;
	bool have_state;

	/* uart state */
	unsigned char lcr;
	unsigned char fcr;
	unsigned char efr;
	unsigned char dll;
	unsigned char dlh;
	unsigned char mcr;
	unsigned char ier;
	unsigned char wer;
};

static struct omap_fiq_debugger *dbgs[OMAP_MAX_HSUART_PORTS];

static inline struct omap_fiq_debugger *get_dbg(struct platform_device *pdev)
{
	struct fiq_debugger_pdata *pdata = dev_get_platdata(&pdev->dev);
	return container_of(pdata, struct omap_fiq_debugger, pdata);
}

static inline void omap_write(struct omap_fiq_debugger *dbg,
			       unsigned int val, unsigned int off)
{
	__raw_writel(val, dbg->debug_port_base + off * 4);
}

static inline unsigned int omap_read(struct omap_fiq_debugger *dbg,
				      unsigned int off)
{
	return __raw_readl(dbg->debug_port_base + off * 4);
}

static void debug_omap_port_enable(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
}

static void debug_omap_port_disable(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);
	unsigned long flags;

	spin_lock_irqsave(&dbg->lock, flags);
	if (!dbg->suspended) {
		pm_runtime_mark_last_busy(&pdev->dev);
		pm_runtime_put_autosuspend(&pdev->dev);
	} else {
		pm_runtime_put_sync_suspend(&pdev->dev);
	}
	spin_unlock_irqrestore(&dbg->lock, flags);
}

static int debug_omap_port_resume(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);

	dbg->suspended = false;
	barrier();
	return 0;
}

static int debug_omap_port_suspend(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);
	unsigned long flags;

	/* this will force the device to be idle'd now, in case it was
	 * autosuspended but timer has not yet run out.
	 */
	spin_lock_irqsave(&dbg->lock, flags);
	dbg->suspended = true;
	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_put_sync_suspend(&pdev->dev);
	spin_unlock_irqrestore(&dbg->lock, flags);

	return 0;
}

/* mostly copied from drivers/tty/serial/omap-serial.c */
static void omap_write_mdr1(struct omap_fiq_debugger *dbg, u8 mdr1)
{
	u8 timeout = 255;

	if (!(cpu_is_omap34xx() || cpu_is_omap44xx())) {
		omap_write(dbg, UART_OMAP_MDR1_DISABLE, UART_OMAP_MDR1);
		return;
	}

	omap_write(dbg, mdr1, UART_OMAP_MDR1);
	udelay(2);
	omap_write(dbg, dbg->fcr | UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR,
		   UART_FCR);

	/*
	 * Wait for FIFO to empty: when empty, RX_FIFO_E bit is 0 and
	 * TX_FIFO_E bit is 1.
	 */
	while (UART_LSR_THRE !=
	       (omap_read(dbg, UART_LSR) & (UART_LSR_THRE | UART_LSR_DR))) {
		timeout--;
		if (!timeout) {
			/* Should *never* happen. we warn and carry on */
			dev_crit(&dbg->pdev->dev, "Errata i202: timedout %x\n",
				 omap_read(dbg, UART_LSR));
			break;
		}
		udelay(1);
	}
}

/* assume the bootloader programmed us correctly */
static void debug_port_read_state(struct omap_fiq_debugger *dbg)
{
	/* assume we're in operational mode when we are called */
	dbg->lcr = omap_read(dbg, UART_LCR);

	/* config mode A */
	omap_write(dbg, UART_LCR_CONF_MODE_A, UART_LCR);
	dbg->mcr = omap_read(dbg, UART_MCR);

	/* config mode B */
	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR);
	dbg->efr = omap_read(dbg, UART_EFR);
	dbg->dll = omap_read(dbg, UART_DLL);
	dbg->dlh = omap_read(dbg, UART_DLM);

	/* back to operational */
	omap_write(dbg, dbg->lcr, UART_LCR);

	pr_debug("%s: lcr=%02x mcr=%02x efr=%02x dll=%02x dlh=%02x\n",
		 __func__, dbg->lcr, dbg->mcr, dbg->efr, dbg->dll, dbg->dlh);
}

static void debug_port_restore(struct omap_fiq_debugger *dbg)
{
	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR); /* Config B mode */
	omap_write(dbg, dbg->efr | UART_EFR_ECB, UART_EFR);
	omap_write(dbg, 0x0, UART_LCR); /* Operational mode */
	omap_write(dbg, 0x0, UART_IER);
	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR); /* Config B mode */
	omap_write(dbg, 0, UART_DLL);
	omap_write(dbg, 0, UART_DLM);
	omap_write(dbg, UART_LCR_CONF_MODE_A, UART_LCR);
	omap_write(dbg, dbg->mcr | UART_MCR_TCRTLR, UART_MCR);
	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR);
	omap_write(dbg, 0, UART_TI752_TLR);
	omap_write(dbg, 0, UART_SCR);
	omap_write(dbg, dbg->efr, UART_EFR);
	omap_write(dbg, UART_LCR_CONF_MODE_A, UART_LCR);
	omap_write(dbg, dbg->fcr, UART_FCR);
	omap_write(dbg, dbg->mcr, UART_MCR);
	omap_write_mdr1(dbg, UART_OMAP_MDR1_DISABLE);

	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR);
	omap_write(dbg, dbg->efr | UART_EFR_ECB, UART_EFR);
	omap_write(dbg, dbg->dll, UART_DLL);
	omap_write(dbg, dbg->dlh, UART_DLM);
	omap_write(dbg, 0, UART_LCR);
	omap_write(dbg, dbg->ier, UART_IER);
	omap_write(dbg, UART_LCR_CONF_MODE_B, UART_LCR);
	omap_write(dbg, dbg->efr, UART_EFR);

	/* will put us back to operational mode */
	omap_write(dbg, dbg->lcr, UART_LCR);
	omap_write_mdr1(dbg, UART_OMAP_MDR1_16X_MODE);

	omap_write(dbg, dbg->wer, UART_OMAP_WER);
}

u32 omap_debug_uart_resume_idle(void)
{
	int i;
	u32 ret = 0;

	for (i = 0; i < OMAP_MAX_HSUART_PORTS; i++) {
		struct omap_fiq_debugger *dbg = dbgs[i];
		struct omap_device *od;

		if (!dbg || !dbg->pdev)
			continue;

		od = to_omap_device(dbg->pdev);
		if (omap_hwmod_pad_get_wakeup_status(od->hwmods[0])) {
			/*
			 * poke the uart and let it stay on long enough
			 * to process any further data. It's ok to use
			 * autosuspend here since this is on the resume path
			 * during the wakeup. We'll still go through a full
			 * resume cycle, so if we go back to suspend
			 * the suspended flag will properly get reset.
			 */
			pm_runtime_get_sync(&dbg->pdev->dev);
			pm_runtime_mark_last_busy(&dbg->pdev->dev);
			pm_runtime_put_autosuspend(&dbg->pdev->dev);
			dev_dbg(&dbg->pdev->dev, "woke up from IO pad\n");
			ret++;
		}
	}

	return ret;
}

static int debug_port_init(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);

	dbg->ier = UART_IER_RLSI | UART_IER_RDI;
	dbg->fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01 |
		UART_FCR_T_TRIG_01;
	dbg->wer = 0;

	device_init_wakeup(&pdev->dev, true);

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, DEFAULT_AUTOSUSPEND_DELAY);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_irq_safe(&pdev->dev);

	omap_hwmod_idle(to_omap_device(pdev)->hwmods[0]);
	debug_omap_port_enable(pdev);
	debug_omap_port_disable(pdev);

	debug_omap_port_enable(pdev);

	if (device_may_wakeup(&pdev->dev))
		omap_hwmod_enable_wakeup(to_omap_device(pdev)->hwmods[0]);

	debug_port_read_state(dbg);
	debug_port_restore(dbg);

	dbg->have_state = true;


	debug_omap_port_disable(pdev);
	return 0;
}

static int debug_getc(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);
	unsigned int lsr;
	int ret = FIQ_DEBUGGER_NO_CHAR;

	lsr = omap_read(dbg, UART_LSR);
	if (lsr & UART_LSR_BI) {
		/* need to read RHR to clear the BI condition */
		omap_read(dbg, UART_RX);
		ret = FIQ_DEBUGGER_BREAK;
	} else if (lsr & UART_LSR_DR) {
		ret = omap_read(dbg, UART_RX);
	}

	return ret;
}

static void debug_putc(struct platform_device *pdev, unsigned int c)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);

	while (!(omap_read(dbg, UART_LSR) & UART_LSR_THRE))
		cpu_relax();

	omap_write(dbg, c, UART_TX);
}

static void debug_flush(struct platform_device *pdev)
{
	struct omap_fiq_debugger *dbg = get_dbg(pdev);

	while (!(omap_read(dbg, UART_LSR) & UART_LSR_TEMT))
		cpu_relax();
}

static int uart_idle_hwmod(struct omap_device *od)
{
	omap_hwmod_idle(od->hwmods[0]);

	return 0;
}

static int uart_enable_hwmod(struct omap_device *od)
{
	struct platform_device *pdev = &od->pdev;
	struct omap_fiq_debugger *dbg = get_dbg(pdev);

	omap_hwmod_enable(od->hwmods[0]);
	if (omap_pm_was_context_lost(&pdev->dev) && dbg->have_state) {
		debug_port_restore(dbg);
		dev_dbg(&pdev->dev, "restoring lost context!\n");
	}

	return 0;
}

static struct omap_device_pm_latency omap_uart_latency[] = {
	{
		.deactivate_func = uart_idle_hwmod,
		.activate_func	 = uart_enable_hwmod,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

extern struct omap_hwmod *omap_uart_hwmod_lookup(int num);

int __init omap_serial_debug_init(int id, bool is_fiq, bool is_high_prio_irq,
				  struct omap_device_pad *pads, int num_pads)
{
	struct omap_fiq_debugger *dbg;
	struct omap_hwmod *oh;
	struct omap_device *od;
	int ret;

	if (id >= OMAP_MAX_HSUART_PORTS)
		return -EINVAL;
	if (dbgs[id])
		return -EBUSY;

	oh = omap_uart_hwmod_lookup(id);
	if (!oh)
		return -ENODEV;

	oh->mpu_irqs[0].name = "uart_irq";
	oh->mux = omap_hwmod_mux_init(pads, num_pads);

	dbg = kzalloc(sizeof(struct omap_fiq_debugger), GFP_KERNEL);
	if (!dbg) {
		pr_err("Failed to allocate for fiq debugger\n");
		return -ENOMEM;
	}

	dbg->debug_port_base = ioremap(oh->slaves[0]->addr[0].pa_start,
				       PAGE_SIZE);
	if (!dbg->debug_port_base) {
		pr_err("Failed to ioremap for fiq debugger\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	spin_lock_init(&dbg->lock);

	dbg->pdata.uart_init = debug_port_init;
	dbg->pdata.uart_getc = debug_getc;
	dbg->pdata.uart_putc = debug_putc;
	dbg->pdata.uart_flush = debug_flush;
	dbg->pdata.uart_enable = debug_omap_port_enable;
	dbg->pdata.uart_disable = debug_omap_port_disable;
	dbg->pdata.uart_dev_suspend = debug_omap_port_suspend;
	dbg->pdata.uart_dev_resume = debug_omap_port_resume;

	od = omap_device_build("fiq_debugger", id,
			       oh, dbg, sizeof(*dbg), omap_uart_latency,
			       ARRAY_SIZE(omap_uart_latency), false);
	if (IS_ERR(od)) {
		pr_err("Could not build omap_device for fiq_debugger: %s\n",
		       oh->name);
		ret = PTR_ERR(od);
		goto err_dev_build;
	}

	dbg->pdev = &od->pdev;
	dbgs[id] = dbg;

	return 0;

err_dev_build:
	iounmap(dbg->debug_port_base);
err_ioremap:
	kfree(dbg);
	return ret;
}
