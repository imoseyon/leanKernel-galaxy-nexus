/*
 * Driver for OMAP-UART controller.
 * Based on drivers/serial/8250.c
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * Authors:
 *	Govindraj R	<govindraj.raja@ti.com>
 *	Thara Gopinath	<thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Note: This driver is made separate from 8250 driver as we cannot
 * over load 8250 driver with omap platform specific configuration for
 * features like DMA, it makes easier to implement features like DMA and
 * hardware flow control and software flow control configuration with
 * this driver as required for the omap-platform.
 */

#if defined(CONFIG_SERIAL_OMAP_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

#include <plat/dma.h>
#include <plat/dmtimer.h>
#include <plat/omap-serial.h>
#include <plat/omap_device.h>
#include <plat/serial.h>
#include <plat/omap-pm.h>

#define UART_OMAP_IIR_ID		0x3e
#define UART_OMAP_IIR_RX_TIMEOUT	0xc

#define UART_OMAP_TXFIFO_LVL		(0x68/4)

static struct uart_omap_port *ui[OMAP_MAX_HSUART_PORTS];

/* Forward declaration of functions */
static void uart_tx_dma_callback(int lch, u16 ch_status, void *data);
static void serial_omap_rxdma_poll(unsigned long uart_no);
static int serial_omap_start_rxdma(struct uart_omap_port *up);
static void omap_uart_mdr1_errataset(struct uart_omap_port *up, u8 mdr1);

static inline unsigned int serial_in(struct uart_omap_port *up, int offset)
{
	offset <<= up->port.regshift;
	return readw(up->port.membase + offset);
}

static inline void serial_out(struct uart_omap_port *up, int offset, int value)
{
	offset <<= up->port.regshift;
	writew(value, up->port.membase + offset);
}

static inline void serial_omap_clear_fifos(struct uart_omap_port *up)
{
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
		       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);
}

/*
 * serial_omap_get_divisor - calculate divisor value
 * @port: uart port info
 * @baud: baudrate for which divisor needs to be calculated.
 *
 * We have written our own function to get the divisor so as to support
 * 13x mode. 3Mbps Baudrate as an different divisor.
 * Reference OMAP TRM Chapter 17:
 * Table 17-1. UART Mode Baud Rates, Divisor Values, and Error Rates
 * referring to oversampling - divisor value
 * baudrate 460,800 to 3,686,400 all have divisor 13
 * except 3,000,000 which has divisor value 16
 */
static unsigned int
serial_omap_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int divisor;

	if (baud > OMAP_MODE13X_SPEED && baud != 3000000)
		divisor = 13;
	else
		divisor = 16;
	return port->uartclk/(baud * divisor);
}

static inline void serial_omap_port_disable(struct uart_omap_port *up)
{
	if (up->suspended) {
		/*
		 * If the port has been suspended by system-wide suspend,
		 * put it back to low power mode immediately.
		 */
		pm_runtime_put_sync_suspend(&up->pdev->dev);
	} else {
		pm_runtime_mark_last_busy(&up->pdev->dev);
		pm_runtime_put_autosuspend(&up->pdev->dev);
	}
}

static inline void serial_omap_port_enable(struct uart_omap_port *up)
{
	pm_runtime_get_sync(&up->pdev->dev);
}

/* TBD: Should be removed once we irq-chaining mechanism in place */
u32 omap_uart_resume_idle()
{
	int i;
	u32 ret = 0;

	for (i = 0; i < OMAP_MAX_HSUART_PORTS; i++) {
		struct uart_omap_port *up = ui[i];

		if (!up)
			continue;

		if (up->chk_wakeup(up->pdev)) {
			serial_omap_port_enable(up);
			serial_omap_port_disable(up);
			ret++;
		}
	}
	return ret;
}

int omap_uart_enable(u8 uart_num)
{
	if (uart_num > OMAP_MAX_HSUART_PORTS)
		return -ENODEV;

	if (!ui[uart_num - 1])
		return -ENODEV;

	pm_runtime_get_sync(&ui[uart_num - 1]->pdev->dev);

	return 0;
}

int omap_uart_disable(u8 uart_num)
{
	if (uart_num > OMAP_MAX_HSUART_PORTS)
		return -ENODEV;

	if (!ui[uart_num - 1])
		return -ENODEV;

	pm_runtime_put_sync_suspend(&ui[uart_num - 1]->pdev->dev);

	return 0;
}

int omap_uart_wake(u8 uart_num)
{
	if (uart_num > OMAP_MAX_HSUART_PORTS)
		return -ENODEV;

	if (!ui[uart_num - 1])
		return -ENODEV;

	serial_omap_port_enable(ui[uart_num - 1]);
	serial_omap_port_disable(ui[uart_num - 1]);

	return 0;
}

static void serial_omap_stop_rxdma(struct uart_omap_port *up)
{
	if (up->uart_dma.rx_dma_used) {
		del_timer(&up->uart_dma.rx_timer);
		omap_stop_dma(up->uart_dma.rx_dma_channel);
		omap_free_dma(up->uart_dma.rx_dma_channel);
		up->uart_dma.rx_dma_channel = OMAP_UART_DMA_CH_FREE;
		up->uart_dma.rx_dma_used = false;
		serial_omap_port_disable(up);
	}
}

static void serial_omap_enable_ms(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_enable_ms+%d\n", up->pdev->id);

	serial_omap_port_enable(up);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
	serial_omap_port_disable(up);
}

static void serial_omap_stop_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	if (up->use_dma &&
		up->uart_dma.tx_dma_channel != OMAP_UART_DMA_CH_FREE) {
		/*
		 * Check if dma is still active. If yes do nothing,
		 * return. Else stop dma
		 */
		if (omap_get_dma_active_status(up->uart_dma.tx_dma_channel))
			return;
		omap_stop_dma(up->uart_dma.tx_dma_channel);
		omap_free_dma(up->uart_dma.tx_dma_channel);
		up->uart_dma.tx_dma_channel = OMAP_UART_DMA_CH_FREE;
		serial_omap_port_disable(up);
	}

	serial_omap_port_enable(up);
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}

	serial_omap_port_disable(up);
}

static void serial_omap_stop_rx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	serial_omap_port_enable(up);
	if (up->use_dma)
		serial_omap_stop_rxdma(up);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
	serial_omap_port_disable(up);
}

static inline void receive_chars(struct uart_omap_port *up, int *status)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned int flag;
	unsigned char ch, lsr = *status;
	int max_count = 256;

	do {
		if (likely(lsr & UART_LSR_DR))
			ch = serial_in(up, UART_RX);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE) {
				up->port.icount.parity++;
			} else if (lsr & UART_LSR_FE) {
				up->port.icount.frame++;
			}

			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_OMAP_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				lsr |= up->lsr_break_flag;
			}
#endif
			if (lsr & UART_LSR_BI)
				flag = TTY_BREAK;
			else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;
		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);
ignore_char:
		lsr = serial_in(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
}

static void transmit_chars(struct uart_omap_port *up, u8 tx_fifo_lvl)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count, i;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_omap_stop_tx(&up->port);
		return;
	}
	count = up->port.fifosize - tx_fifo_lvl;
	for (i = 0; i < count; i++) {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_omap_stop_tx(&up->port);
}

static inline void serial_omap_enable_ier_thri(struct uart_omap_port *up)
{
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serial_omap_start_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	struct circ_buf *xmit;
	unsigned int start;
	int ret = 0;

	if (!up->use_dma) {
		serial_omap_port_enable(up);
		serial_omap_enable_ier_thri(up);
		serial_omap_port_disable(up);
		return;
	}

	if (up->uart_dma.tx_dma_used)
		return;

	xmit = &up->port.state->xmit;

	if (up->uart_dma.tx_dma_channel == OMAP_UART_DMA_CH_FREE) {
		serial_omap_port_enable(up);
		ret = omap_request_dma(up->uart_dma.uart_dma_tx,
				"UART Tx DMA",
				(void *)uart_tx_dma_callback, up,
				&(up->uart_dma.tx_dma_channel));

		if (ret < 0) {
			serial_omap_enable_ier_thri(up);
			return;
		}
	}
	spin_lock(&(up->uart_dma.tx_lock));
	up->uart_dma.tx_dma_used = true;
	spin_unlock(&(up->uart_dma.tx_lock));

	start = up->uart_dma.tx_buf_dma_phys +
				(xmit->tail & (UART_XMIT_SIZE - 1));

	up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
	/*
	 * It is a circular buffer. See if the buffer has wounded back.
	 * If yes it will have to be transferred in two separate dma
	 * transfers
	 */
	if (start + up->uart_dma.tx_buf_size >=
			up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
		up->uart_dma.tx_buf_size =
			(up->uart_dma.tx_buf_dma_phys +
			UART_XMIT_SIZE) - start;

	omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
				OMAP_DMA_AMODE_CONSTANT,
				up->uart_dma.uart_base, 0, 0);
	omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC, start, 0, 0);
	omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
				OMAP_DMA_DATA_TYPE_S8,
				up->uart_dma.tx_buf_size, 1,
				OMAP_DMA_SYNC_ELEMENT,
				up->uart_dma.uart_dma_tx, 0);
	/* FIXME: Cache maintenance needed here? */
	omap_start_dma(up->uart_dma.tx_dma_channel);
}

static unsigned int check_modem_status(struct uart_omap_port *up)
{
	unsigned int status;

	status = serial_in(up, UART_MSR);
	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if ((status & UART_MSR_ANY_DELTA) == 0)
		return status;

	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change
				(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change
				(&up->port, status & UART_MSR_CTS);
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

/**
 * serial_omap_irq() - This handles the interrupt from one port
 * @irq: uart port irq number
 * @dev_id: uart port info
 */
static inline irqreturn_t serial_omap_irq(int irq, void *dev_id)
{
	struct uart_omap_port *up = dev_id;
	unsigned int iir, lsr;
	unsigned int int_id;
	unsigned long flags;
	int ret = IRQ_HANDLED;
	u8 tx_fifo_lvl;

	serial_omap_port_enable(up);
	iir = serial_in(up, UART_IIR);
	if (iir & UART_IIR_NO_INT) {
		serial_omap_port_disable(up);
		return IRQ_NONE;
	}

	int_id = iir & UART_OMAP_IIR_ID;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	if (int_id == UART_IIR_RDI || int_id == UART_OMAP_IIR_RX_TIMEOUT ||
	    int_id == UART_IIR_RLSI) {
		if (!up->use_dma) {
			if (lsr & UART_LSR_DR)
				receive_chars(up, &lsr);
		} else {
			up->ier &= ~(UART_IER_RDI | UART_IER_RLSI);
			serial_out(up, UART_IER, up->ier);
			if ((serial_omap_start_rxdma(up) != 0) &&
					(lsr & UART_LSR_DR))
				receive_chars(up, &lsr);
		}
	}

	check_modem_status(up);
	if (int_id == UART_IIR_THRI) {
		tx_fifo_lvl = serial_in(up, UART_OMAP_TXFIFO_LVL);
		if (lsr & UART_LSR_THRE || tx_fifo_lvl < up->port.fifosize)
			transmit_chars(up, tx_fifo_lvl);
		else
			ret = IRQ_NONE;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);
	serial_omap_port_disable(up);

	up->port_activity = jiffies;
	return ret;
}

static unsigned int serial_omap_tx_empty(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags = 0;
	unsigned int ret = 0;

	serial_omap_port_enable(up);
	dev_dbg(up->port.dev, "serial_omap_tx_empty+%d\n", up->pdev->id);
	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);
	serial_omap_port_disable(up);
	return ret;
}

static unsigned int serial_omap_get_mctrl(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char status;
	unsigned int ret = 0;

	serial_omap_port_enable(up);
	status = check_modem_status(up);
	serial_omap_port_disable(up);

	dev_dbg(up->port.dev, "serial_omap_get_mctrl+%d\n", up->pdev->id);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_omap_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char mcr = 0;

	dev_dbg(up->port.dev, "serial_omap_set_mctrl+%d\n", up->pdev->id);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	serial_omap_port_enable(up);
	up->mcr = serial_in(up, UART_MCR);
	up->mcr |= mcr;
	serial_out(up, UART_MCR, up->mcr);
	serial_omap_port_disable(up);
}

static void serial_omap_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags = 0;

	dev_dbg(up->port.dev, "serial_omap_break_ctl+%d\n", up->pdev->id);
	serial_omap_port_enable(up);
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
	serial_omap_port_disable(up);
}

static int serial_omap_startup(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags = 0;

	enable_irq(up->port.irq);

	dev_dbg(up->port.dev, "serial_omap_startup+%d\n", up->pdev->id);

	serial_omap_port_enable(up);
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_omap_clear_fifos(up);
	/* For Hardware flow control */
	serial_out(up, UART_MCR, UART_MCR_RTS);
	up->mcr = serial_in(up, UART_MCR);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	spin_lock_irqsave(&up->port.lock, flags);
	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	up->port.mctrl |= TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	up->msr_saved_flags = 0;
	if (up->use_dma) {
		free_page((unsigned long)up->port.state->xmit.buf);
		up->port.state->xmit.buf = dma_alloc_coherent(NULL,
			UART_XMIT_SIZE,
			(dma_addr_t *)&(up->uart_dma.tx_buf_dma_phys),
			0);
		init_timer(&(up->uart_dma.rx_timer));
		up->uart_dma.rx_timer.function = serial_omap_rxdma_poll;
		up->uart_dma.rx_timer.data = up->pdev->id;
		/* Currently the buffer size is 4KB. Can increase it */
		up->uart_dma.rx_buf = dma_alloc_coherent(NULL,
			up->uart_dma.rx_buf_size,
			(dma_addr_t *)&(up->uart_dma.rx_buf_dma_phys), 0);
	}
	/*
	 * Finally, enable interrupts. Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);

	/* Enable module level wake up */
	serial_out(up, UART_OMAP_WER, up->wer);

	serial_omap_port_disable(up);
	up->port_activity = jiffies;
	return 0;
}

static void serial_omap_shutdown(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags = 0;

	dev_dbg(up->port.dev, "serial_omap_shutdown+%d\n", up->pdev->id);

	serial_omap_port_enable(up);
	/*
	 * Disable interrupts & wakeup events from this port
	 */
	up->ier = 0;
	serial_out(up, UART_OMAP_WER, 0);
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_omap_clear_fifos(up);

	/*
	 * Read data port to reset things, and then free the irq
	 */
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	if (up->use_dma) {
		dma_free_coherent(up->port.dev,
			UART_XMIT_SIZE,	up->port.state->xmit.buf,
			up->uart_dma.tx_buf_dma_phys);
		up->port.state->xmit.buf = NULL;
		serial_omap_stop_rx(port);
		dma_free_coherent(up->port.dev,
			up->uart_dma.rx_buf_size, up->uart_dma.rx_buf,
			up->uart_dma.rx_buf_dma_phys);
		up->uart_dma.rx_buf = NULL;
	}
	serial_omap_port_disable(up);
	disable_irq(up->port.irq);
}

static inline void
serial_omap_configure_xonxoff
		(struct uart_omap_port *up, struct ktermios *termios)
{
	up->lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr & ~UART_EFR_ECB);

	serial_out(up, UART_XON1, termios->c_cc[VSTART]);
	serial_out(up, UART_XOFF1, termios->c_cc[VSTOP]);

	/* clear SW control mode bits */
	up->efr &= OMAP_UART_SW_CLR;

	/*
	 * IXON Flag:
	 * Flow control for OMAP.TX
	 * OMAP.RX should listen for XON/XOFF
	 */
	if (termios->c_iflag & IXON)
		up->efr |= OMAP_UART_SW_RX;

	/*
	 * IXOFF Flag:
	 * Flow control for OMAP.RX
	 * OMAP.TX should send XON/XOFF
	 */
	if (termios->c_iflag & IXOFF)
		up->efr |= OMAP_UART_SW_TX;

	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);

	up->mcr = serial_in(up, UART_MCR);

	/*
	 * IXANY Flag:
	 * Enable any character to restart output.
	 * Operation resumes after receiving any
	 * character after recognition of the XOFF character
	 */
	if (termios->c_iflag & IXANY)
		up->mcr |= UART_MCR_XONANY;

	serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_TI752_TCR, OMAP_UART_TCR_TRIG);
	/* Enable special char function UARTi.EFR_REG[5] and
	 * load the new software flow control mode IXON or IXOFF
	 * and restore the UARTi.EFR_REG[4] ENHANCED_EN value.
	 */
	serial_out(up, UART_EFR, up->efr | UART_EFR_SCD);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);

	serial_out(up, UART_MCR, up->mcr & ~UART_MCR_TCRTLR);
	serial_out(up, UART_LCR, up->lcr);
}

static void
serial_omap_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char cval = 0;
	unsigned char efr = 0;
	unsigned long flags = 0;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/13);
	quot = serial_omap_get_divisor(port, baud);

	up->dll = quot & 0xff;
	up->dlh = quot >> 8;
	up->mdr1 = UART_OMAP_MDR1_DISABLE;

	up->fcr = UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_01 |
			UART_FCR_ENABLE_FIFO;
	if (up->use_dma)
		up->fcr |= UART_FCR_DMA_SELECT;

	/*
	 * Ok, we're now changing the port state. Do it with
	 * interrupts disabled.
	 */
	serial_omap_port_enable(up);
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * Modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;

	/* FIFOs and DMA Settings */

	/* FCR can be changed only when the
	 * baud clock is not running
	 * DLL_REG and DLH_REG set to 0.
	 */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	up->mcr = serial_in(up, UART_MCR);
	serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);
	/* FIFO ENABLE, DMA MODE */
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	if (up->use_dma) {
		if (up->errata & OMAP4_UART_ERRATA_i659_TX_THR) {
			serial_out(up, UART_MDR3, SET_DMA_TX_THRESHOLD);
			serial_out(up, UART_TX_DMA_THRESHOLD, TX_FIFO_THR_LVL);
		}

		serial_out(up, UART_TI752_TLR, 0);
		serial_out(up, UART_OMAP_SCR,
			(UART_FCR_TRIGGER_4 | UART_FCR_TRIGGER_8));
	}

	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr);

	/* Protocol, Baud Rate, and Interrupt Settings */
	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		omap_uart_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	up->efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, 0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	serial_out(up, UART_DLL, up->dll);	/* LS of divisor */
	serial_out(up, UART_DLM, up->dlh);	/* MS of divisor */

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, cval);

	if (baud > 230400 && baud != 3000000)
		up->mdr1 = UART_OMAP_MDR1_13X_MODE;
	else
		up->mdr1 = UART_OMAP_MDR1_16X_MODE;

	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		omap_uart_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);

	/* Hardware Flow Control Configuration */

	if (termios->c_cflag & CRTSCTS) {
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
		up->mcr = serial_in(up, UART_MCR);
		serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);

		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
		up->efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
		serial_out(up, UART_TI752_TCR, OMAP_UART_TCR_TRIG);

		up->efr |= (UART_EFR_CTS | UART_EFR_RTS);
		serial_out(up, UART_EFR, up->efr); /* Enable AUTORTS and AUTOCTS */
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
		up->mcr |= UART_MCR_RTS;
		serial_out(up, UART_MCR, up->mcr);
		serial_out(up, UART_LCR, cval);
	} else {
		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
		up->efr = serial_in(up, UART_EFR);
		up->efr &= ~(UART_EFR_CTS | UART_EFR_RTS);
		serial_out(up, UART_EFR, up->efr); /* Disable AUTORTS and AUTOCTS */
		serial_out(up, UART_LCR, cval);
	}

	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	/* Software Flow Control Configuration */
	serial_omap_configure_xonxoff(up, termios);

	/* Now we are ready for RX data: enable rts line */
	if (up->rts_mux_driver_control && up->rts_pullup_in_suspend) {
		omap_rts_mux_write(0, up->port.line);
		up->rts_pullup_in_suspend = 0;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);
	serial_omap_port_disable(up);
	dev_dbg(up->port.dev, "serial_omap_set_termios+%d\n", up->pdev->id);
}

static void
serial_omap_pm(struct uart_port *port, unsigned int state,
	       unsigned int oldstate)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char efr;
	unsigned char lcr;

	dev_dbg(up->port.dev, "serial_omap_pm+%d\n", up->pdev->id);

	serial_omap_port_enable(up);
	lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_IER, (state != 0) ? UART_IERX_SLEEP : 0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, lcr);
	if (state)
		pm_runtime_put_sync(&up->pdev->dev);
	else
		serial_omap_port_disable(up);
}

static void serial_omap_wake_peer(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	if (up->wake_peer)
		up->wake_peer(port);
}

static void serial_omap_release_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_release_port+\n");
}

static int serial_omap_request_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_request_port+\n");
	return 0;
}

static void serial_omap_config_port(struct uart_port *port, int flags)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_config_port+%d\n",
							up->pdev->id);
	up->port.type = PORT_OMAP;
}

static int
serial_omap_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	dev_dbg(port->dev, "serial_omap_verify_port+\n");
	return -EINVAL;
}

static const char *
serial_omap_type(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	dev_dbg(up->port.dev, "serial_omap_type+%d\n", up->pdev->id);
	return up->name;
}

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

static inline void wait_for_xmitr(struct uart_omap_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);

			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;

			udelay(1);
		}
	}
}

#ifdef CONFIG_CONSOLE_POLL

static void serial_omap_poll_put_char(struct uart_port *port, unsigned char ch)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	serial_omap_port_enable(up);
	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
	serial_omap_port_disable(up);
}

static int serial_omap_poll_get_char(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned int status;

	serial_omap_port_enable(up);
	status = serial_in(up, UART_LSR);
	if (!(status & UART_LSR_DR))
		return NO_POLL_CHAR;

	status = serial_in(up, UART_RX);
	serial_omap_port_disable(up);
	return status;
}

#endif /* CONFIG_CONSOLE_POLL */

#ifdef CONFIG_SERIAL_OMAP_CONSOLE
static struct uart_omap_port *serial_omap_console_ports[4];

static struct uart_driver serial_omap_reg;

static void serial_omap_console_putchar(struct uart_port *port, int ch)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

static void
serial_omap_console_write(struct console *co, const char *s,
		unsigned int count)
{
	struct uart_omap_port *up = serial_omap_console_ports[co->index];
	unsigned long flags;
	unsigned int ier;
	int console_lock = 0, locked = 1;

	if (console_trylock())
		console_lock = 1;

	/*
	 * If console_lock is not available and we are in suspending
	 * state then we can avoid the console usage scenario
	 * as this may introduce recursive prints.
	 * Basically this scenario occurs during boot while
	 * printing debug bootlogs.
	 */

	if (!console_lock &&
		up->pdev->dev.power.runtime_status == RPM_SUSPENDING)
		return;

	local_irq_save(flags);
	if (up->port.sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&up->port.lock);
	else
		spin_lock(&up->port.lock);

	serial_omap_port_enable(up);

	/*
	 * First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_omap_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
	/*
	 * The receive handling will happen properly because the
	 * receive ready bit will still be set; it is not cleared
	 * on read.  However, modem control will not, we must
	 * call it if we have saved something in the saved flags
	 * while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);

	if (console_lock)
		console_unlock();

	serial_omap_port_disable(up);
	if (locked)
		spin_unlock(&up->port.lock);
	local_irq_restore(flags);
}

static int __init
serial_omap_console_setup(struct console *co, char *options)
{
	struct uart_omap_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (serial_omap_console_ports[co->index] == NULL)
		return -ENODEV;
	up = serial_omap_console_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_omap_console = {
	.name		= OMAP_SERIAL_NAME,
	.write		= serial_omap_console_write,
	.device		= uart_console_device,
	.setup		= serial_omap_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_omap_reg,
};

static void serial_omap_add_console_port(struct uart_omap_port *up)
{
	serial_omap_console_ports[up->pdev->id] = up;
}

#define OMAP_CONSOLE	(&serial_omap_console)

#else

#define OMAP_CONSOLE	NULL

static inline void serial_omap_add_console_port(struct uart_omap_port *up)
{}

#endif

static struct uart_ops serial_omap_pops = {
	.tx_empty	= serial_omap_tx_empty,
	.set_mctrl	= serial_omap_set_mctrl,
	.get_mctrl	= serial_omap_get_mctrl,
	.stop_tx	= serial_omap_stop_tx,
	.start_tx	= serial_omap_start_tx,
	.stop_rx	= serial_omap_stop_rx,
	.enable_ms	= serial_omap_enable_ms,
	.break_ctl	= serial_omap_break_ctl,
	.startup	= serial_omap_startup,
	.shutdown	= serial_omap_shutdown,
	.set_termios	= serial_omap_set_termios,
	.pm		= serial_omap_pm,
	.wake_peer	= serial_omap_wake_peer,
	.type		= serial_omap_type,
	.release_port	= serial_omap_release_port,
	.request_port	= serial_omap_request_port,
	.config_port	= serial_omap_config_port,
	.verify_port	= serial_omap_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char  = serial_omap_poll_put_char,
	.poll_get_char  = serial_omap_poll_get_char,
#endif
};

static struct uart_driver serial_omap_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "OMAP-SERIAL",
	.dev_name	= OMAP_SERIAL_NAME,
	.nr		= OMAP_MAX_HSUART_PORTS,
	.cons		= OMAP_CONSOLE,
};

static int serial_omap_suspend(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	if (up) {
		disable_irq(up->port.irq);
		if (up->rts_mux_driver_control) {
			up->rts_pullup_in_suspend = 1;
			omap_rts_mux_write(MUX_PULL_UP, up->port.line);
		}
		up->suspended = true;
		uart_suspend_port(&serial_omap_reg, &up->port);
		serial_omap_pm(&up->port, 3, 0);
	}
	return 0;
}

static int serial_omap_resume(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	if (up) {
		uart_resume_port(&serial_omap_reg, &up->port);
		up->suspended = false;
		enable_irq(up->port.irq);
	}

	return 0;
}

static void serial_omap_rxdma_poll(unsigned long uart_no)
{
	struct uart_omap_port *up = ui[uart_no];
	unsigned int curr_dma_pos, curr_transmitted_size;
	int ret = 0;

	curr_dma_pos = omap_get_dma_dst_pos(up->uart_dma.rx_dma_channel);
	if ((curr_dma_pos == up->uart_dma.prev_rx_dma_pos) ||
			     (curr_dma_pos == 0)) {
		if (jiffies_to_msecs(jiffies - up->port_activity) <
						up->uart_dma.rx_timeout) {
			mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_poll_rate));
		} else {
			serial_omap_stop_rxdma(up);
			up->ier |= (UART_IER_RDI | UART_IER_RLSI);
			serial_out(up, UART_IER, up->ier);
		}
		return;
	}

	curr_transmitted_size = curr_dma_pos -
					up->uart_dma.prev_rx_dma_pos;
	up->port.icount.rx += curr_transmitted_size;
	tty_insert_flip_string(up->port.state->port.tty,
			up->uart_dma.rx_buf +
			(up->uart_dma.prev_rx_dma_pos -
			up->uart_dma.rx_buf_dma_phys),
			curr_transmitted_size);
	tty_flip_buffer_push(up->port.state->port.tty);
	up->uart_dma.prev_rx_dma_pos = curr_dma_pos;
	if (up->uart_dma.rx_buf_size +
			up->uart_dma.rx_buf_dma_phys == curr_dma_pos) {
		ret = serial_omap_start_rxdma(up);
		if (ret < 0) {
			serial_omap_stop_rxdma(up);
			up->ier |= (UART_IER_RDI | UART_IER_RLSI);
			serial_out(up, UART_IER, up->ier);
		}
	} else  {
		mod_timer(&up->uart_dma.rx_timer, jiffies +
			usecs_to_jiffies(up->uart_dma.rx_poll_rate));
	}
	up->port_activity = jiffies;
}

static void uart_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	return;
}

static int serial_omap_start_rxdma(struct uart_omap_port *up)
{
	int ret = 0;

	if (up->uart_dma.rx_dma_channel == -1) {
		serial_omap_port_enable(up);
		ret = omap_request_dma(up->uart_dma.uart_dma_rx,
				"UART Rx DMA",
				(void *)uart_rx_dma_callback, up,
				&(up->uart_dma.rx_dma_channel));
		if (ret < 0)
			return ret;

		omap_set_dma_src_params(up->uart_dma.rx_dma_channel, 0,
				OMAP_DMA_AMODE_CONSTANT,
				up->uart_dma.uart_base, 0, 0);
		omap_set_dma_dest_params(up->uart_dma.rx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC,
				up->uart_dma.rx_buf_dma_phys, 0, 0);
		omap_set_dma_transfer_params(up->uart_dma.rx_dma_channel,
				OMAP_DMA_DATA_TYPE_S8,
				up->uart_dma.rx_buf_size, 1,
				OMAP_DMA_SYNC_ELEMENT,
				up->uart_dma.uart_dma_rx, 0);
	}
	up->uart_dma.prev_rx_dma_pos = up->uart_dma.rx_buf_dma_phys;
	/* FIXME: Cache maintenance needed here? */
	omap_start_dma(up->uart_dma.rx_dma_channel);
	mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_poll_rate));
	up->uart_dma.rx_dma_used = true;
	return ret;
}

static void serial_omap_continue_tx(struct uart_omap_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	unsigned int start = up->uart_dma.tx_buf_dma_phys
			+ (xmit->tail & (UART_XMIT_SIZE - 1));

	if (uart_circ_empty(xmit))
		return;

	up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
	/*
	 * It is a circular buffer. See if the buffer has wounded back.
	 * If yes it will have to be transferred in two separate dma
	 * transfers
	 */
	if (start + up->uart_dma.tx_buf_size >=
			up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
		up->uart_dma.tx_buf_size =
			(up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE) - start;
	omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
				OMAP_DMA_AMODE_CONSTANT,
				up->uart_dma.uart_base, 0, 0);
	omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC, start, 0, 0);
	omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
				OMAP_DMA_DATA_TYPE_S8,
				up->uart_dma.tx_buf_size, 1,
				OMAP_DMA_SYNC_ELEMENT,
				up->uart_dma.uart_dma_tx, 0);
	/* FIXME: Cache maintenance needed here? */
	omap_start_dma(up->uart_dma.tx_dma_channel);
}

static void uart_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct uart_omap_port *up = (struct uart_omap_port *)data;
	struct circ_buf *xmit = &up->port.state->xmit;

	xmit->tail = (xmit->tail + up->uart_dma.tx_buf_size) & \
			(UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->uart_dma.tx_buf_size;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit)) {
		spin_lock(&(up->uart_dma.tx_lock));
		serial_omap_stop_tx(&up->port);
		up->uart_dma.tx_dma_used = false;
		spin_unlock(&(up->uart_dma.tx_lock));
	} else {
		omap_stop_dma(up->uart_dma.tx_dma_channel);
		serial_omap_continue_tx(up);
	}
	up->port_activity = jiffies;
	return;
}

static int serial_omap_probe(struct platform_device *pdev)
{
	struct uart_omap_port	*up = NULL;
	struct resource		*mem, *irq, *dma_tx, *dma_rx;
	struct omap_uart_port_info *omap_up_info = pdev->dev.platform_data;
	struct omap_device *od;
	int ret = -ENOSPC;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	if (!request_mem_region(mem->start, (mem->end - mem->start) + 1,
				     pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		return -EBUSY;
	}

	dma_rx = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (!dma_rx) {
		ret = -EINVAL;
		goto do_release_region;
	}

	dma_tx = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx");
	if (!dma_tx) {
		ret = -EINVAL;
		goto do_release_region;
	}

	up = kzalloc(sizeof(*up), GFP_KERNEL);
	if (up == NULL) {
		ret = -ENOMEM;
		goto do_release_region;
	}
	sprintf(up->name, "OMAP UART%d", pdev->id);
	up->pdev = pdev;
	up->port.dev = &pdev->dev;
	up->port.type = PORT_OMAP;
	up->port.iotype = UPIO_MEM;
	up->port.irq = irq->start;

	up->port.regshift = 2;
	up->port.fifosize = 64;
	up->port.ops = &serial_omap_pops;
	up->port.line = pdev->id;

	up->port.mapbase = mem->start;
	up->port.membase = ioremap(mem->start, mem->end - mem->start);

	if (!up->port.membase) {
		dev_err(&pdev->dev, "can't ioremap UART\n");
		ret = -ENOMEM;
		goto do_free;
	}

	up->port.flags = omap_up_info->flags;
	up->port.uartclk = omap_up_info->uartclk;
	up->uart_dma.uart_base = mem->start;
	up->errata = omap_up_info->errata;
	up->enable_wakeup = omap_up_info->enable_wakeup;
	up->wer = omap_up_info->wer;
	up->chk_wakeup = omap_up_info->chk_wakeup;
	up->wake_peer = omap_up_info->wake_peer;
	up->rts_mux_driver_control = omap_up_info->rts_mux_driver_control;
	up->rts_pullup_in_suspend = 0;

	if (omap_up_info->use_dma) {
		up->uart_dma.uart_dma_tx = dma_tx->start;
		up->uart_dma.uart_dma_rx = dma_rx->start;
		up->use_dma = 1;
		up->uart_dma.rx_buf_size = omap_up_info->dma_rx_buf_size;
		up->uart_dma.rx_timeout = omap_up_info->dma_rx_timeout;
		up->uart_dma.rx_poll_rate = omap_up_info->dma_rx_poll_rate;
		spin_lock_init(&(up->uart_dma.tx_lock));
		spin_lock_init(&(up->uart_dma.rx_lock));
		up->uart_dma.tx_dma_channel = OMAP_UART_DMA_CH_FREE;
		up->uart_dma.rx_dma_channel = OMAP_UART_DMA_CH_FREE;
	}

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
			omap_up_info->auto_sus_timeout);

	if (device_may_wakeup(&pdev->dev))
		pm_runtime_enable(&pdev->dev);

	pm_runtime_irq_safe(&pdev->dev);
	if (omap_up_info->console_uart) {
		od = to_omap_device(up->pdev);
		omap_hwmod_idle(od->hwmods[0]);
		serial_omap_port_enable(up);
		serial_omap_port_disable(up);
	}

	ui[pdev->id] = up;
	serial_omap_add_console_port(up);

	ret = request_irq(up->port.irq, serial_omap_irq, up->port.irqflags,
				up->name, up);
	if (ret)
		goto do_iounmap;
	disable_irq(up->port.irq);

	ret = uart_add_one_port(&serial_omap_reg, &up->port);
	if (ret != 0)
		goto do_free_irq;

	dev_set_drvdata(&pdev->dev, up);
	platform_set_drvdata(pdev, up);

	return 0;

do_free_irq:
	free_irq(up->port.irq, up);
do_iounmap:
	iounmap(up->port.membase);
do_free:
	kfree(up);
do_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	dev_err(&pdev->dev, "[UART%d]: failure [%s]: %d\n",
				pdev->id, __func__, ret);
	return ret;
}

static int serial_omap_remove(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	if (up) {
		pm_runtime_disable(&up->pdev->dev);
		free_irq(up->port.irq, up);
		uart_remove_one_port(&serial_omap_reg, &up->port);
		iounmap(up->port.membase);
		kfree(up);
	}
	return 0;
}

/*
 * Work Around for Errata i202 (3430 - 1.12, 3630 - 1.6)
 * The access to uart register after MDR1 Access
 * causes UART to corrupt data.
 *
 * Need a delay =
 * 5 L4 clock cycles + 5 UART functional clock cycle (@48MHz = ~0.2uS)
 * give 10 times as much
 */
static void omap_uart_mdr1_errataset(struct uart_omap_port *up, u8 mdr1)
{
	u8 timeout = 255;

	serial_out(up, UART_OMAP_MDR1, mdr1);
	udelay(2);
	serial_out(up, UART_FCR, up->fcr | UART_FCR_CLEAR_XMIT |
			UART_FCR_CLEAR_RCVR);
	/*
	 * Wait for FIFO to empty: when empty, RX_FIFO_E bit is 0 and
	 * TX_FIFO_E bit is 1.
	 */
	while (UART_LSR_THRE != (serial_in(up, UART_LSR) &
				(UART_LSR_THRE | UART_LSR_DR))) {
		timeout--;
		if (!timeout) {
			/* Should *never* happen. we warn and carry on */
			dev_crit(&up->pdev->dev, "Errata i202: timedout %x\n",
						serial_in(up, UART_LSR));
			break;
		}
		udelay(1);
	}
}

static void omap_uart_restore_context(struct uart_omap_port *up)
{
	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		omap_uart_mdr1_errataset(up, UART_OMAP_MDR1_DISABLE);
	else
		serial_out(up, UART_OMAP_MDR1, UART_OMAP_MDR1_DISABLE);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, 0x0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_DLL, up->dll);
	serial_out(up, UART_DLM, up->dlh);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, up->lcr);
	/* Enable module level wake up */
	serial_out(up, UART_OMAP_WER, up->wer);
	if (up->use_dma) {
		if (up->errata & OMAP4_UART_ERRATA_i659_TX_THR) {
			serial_out(up, UART_MDR3, SET_DMA_TX_THRESHOLD);
			serial_out(up, UART_TX_DMA_THRESHOLD, TX_FIFO_THR_LVL);
		}

		serial_out(up, UART_TI752_TLR, 0);
		serial_out(up, UART_OMAP_SCR,
			(UART_FCR_TRIGGER_4 | UART_FCR_TRIGGER_8));
	}

	/* UART 16x mode */
	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		omap_uart_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);
}

static int omap_serial_runtime_suspend(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	if (!up)
		goto done;

	if (up->rts_mux_driver_control) {
		omap_rts_mux_write(MUX_PULL_UP, up->port.line);
		/* wait a few bytes to allow current transmission to complete */
		udelay(300);
	}
	if (device_may_wakeup(dev))
		up->enable_wakeup(up->pdev, true);
	else
		up->enable_wakeup(up->pdev, false);
done:
	return 0;
}

static int omap_serial_runtime_resume(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);
	struct omap_device *od;

	if (up) {
		if (omap_pm_was_context_lost(dev))
			omap_uart_restore_context(up);

		if (up->use_dma) {
			/* NO TX_DMA WAKEUP SO KEEP IN NO IDLE MODE */
			od = to_omap_device(up->pdev);
			omap_hwmod_set_slave_idlemode(od->hwmods[0],
						HWMOD_IDLEMODE_NO);
		}
		if (up->rts_mux_driver_control && (!up->rts_pullup_in_suspend))
			omap_rts_mux_write(0, up->port.line);
	}

	return 0;
}

static const struct dev_pm_ops omap_serial_dev_pm_ops = {
	.suspend = serial_omap_suspend,
	.resume	= serial_omap_resume,
	.runtime_suspend = omap_serial_runtime_suspend,
	.runtime_resume = omap_serial_runtime_resume,
};

static struct platform_driver serial_omap_driver = {
	.probe          = serial_omap_probe,
	.remove         = serial_omap_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.pm = &omap_serial_dev_pm_ops,
	},
};

static int __init serial_omap_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_omap_reg);
	if (ret != 0)
		return ret;
	ret = platform_driver_register(&serial_omap_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_omap_reg);
	return ret;
}

static void __exit serial_omap_exit(void)
{
	platform_driver_unregister(&serial_omap_driver);
	uart_unregister_driver(&serial_omap_reg);
}

module_init(serial_omap_init);
module_exit(serial_omap_exit);

MODULE_DESCRIPTION("OMAP High Speed UART driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
