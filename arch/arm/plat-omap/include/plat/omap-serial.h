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
 */

#ifndef __OMAP_SERIAL_H__
#define __OMAP_SERIAL_H__

#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <plat/mux.h>

#define DRIVER_NAME	"omap_uart"

/*
 * Use tty device name as ttyO, [O -> OMAP]
 * in bootargs we specify as console=ttyO0 if uart1
 * is used as console uart.
 */
#define OMAP_SERIAL_NAME	"ttyO"

#define OMAP_MODE13X_SPEED	230400

/* WER = 0x7F
 * Enable module level wakeup in WER reg
 */
#define OMAP2_UART_WER_MOD_WKUP	0X7F
#define OMAP4_UART_WER_MOD_WKUP	0XFF

/* Enable XON/XOFF flow control on output */
#define OMAP_UART_SW_TX		0x8

/* Enable XON/XOFF flow control on input */
#define OMAP_UART_SW_RX		0x2

#define OMAP_UART_SYSC_RESET	0X07
#define OMAP_UART_TCR_TRIG	0X0F
#define OMAP_UART_SW_CLR	0XF0
#define OMAP_UART_FIFO_CLR	0X06

#define OMAP_UART_DMA_CH_FREE	-1

#define RX_TIMEOUT			(3 * HZ) /* RX DMA timeout (jiffies) */

#define DEFAULT_RXDMA_TIMEOUT	(3 * HZ)	/* RX DMA timeout (jiffies) */
#define DEFAULT_RXDMA_POLLRATE	1		/* RX DMA polling rate (us) */
#define DEFAULT_RXDMA_BUFSIZE	4096		/* RX DMA buffer size */
#define DEFAULT_AUTOSUSPEND_DELAY	3000	/* Runtime autosuspend (msecs)*/

/*
 * (Errata i659) - From OMAP4430 ES 2.0 onwards set
 * tx_threshold while using UART in DMA Mode
 * and ensure tx_threshold + tx_trigger <= 63
 */
#define UART_MDR3		0x20
#define UART_TX_DMA_THRESHOLD	0x21
#define SET_DMA_TX_THRESHOLD	BIT(2)
/* Setting TX Threshold Level to 62 */
#define TX_FIFO_THR_LVL		0x3E

#define OMAP_MAX_HSUART_PORTS	4

#define MSR_SAVE_FLAGS		UART_MSR_ANY_DELTA

#define UART_ERRATA_i202_MDR1_ACCESS	BIT(0)
#define OMAP4_UART_ERRATA_i659_TX_THR	BIT(1)

struct omap_uart_port_info {
	int                     dma_rx_buf_size;/* DMA Rx Buffer Size */
	int                     dma_rx_timeout; /* DMA RX timeout */
	unsigned int            idle_timeout;   /* Omap Uart Idle Time out */
	int                     use_dma;        /* DMA Enable / Disable */
	unsigned int		uartclk;	/* UART clock rate */
	upf_t			flags;		/* UPF_* flags */
	unsigned int		errata;
	unsigned int		console_uart;
	u16			wer;		/* Module Wakeup register */
	unsigned int		dma_rx_poll_rate; /* DMA RX poll_rate */
	unsigned int		auto_sus_timeout; /* Auto_suspend timeout */
	unsigned		rts_mux_driver_control:1;

	void (*enable_wakeup)(struct platform_device *, bool);
	bool (*chk_wakeup)(struct platform_device *);
	void (*wake_peer)(struct uart_port *);
	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
};

struct uart_omap_dma {
	u8			uart_dma_tx;
	u8			uart_dma_rx;
	int			rx_dma_channel;
	int			tx_dma_channel;
	dma_addr_t		rx_buf_dma_phys;
	dma_addr_t		tx_buf_dma_phys;
	unsigned int		uart_base;
	/*
	 * Buffer for rx dma.It is not required for tx because the buffer
	 * comes from port structure.
	 */
	unsigned char		*rx_buf;
	unsigned int		prev_rx_dma_pos;
	int			tx_buf_size;
	int			tx_dma_used;
	int			rx_dma_used;
	spinlock_t		tx_lock;
	spinlock_t		rx_lock;
	/* timer to poll activity on rx dma */
	struct timer_list	rx_timer;
	unsigned int		rx_buf_size;
	unsigned int		rx_poll_rate;
	unsigned int		rx_timeout;
};

struct uart_omap_port {
	struct uart_port	port;
	struct uart_omap_dma	uart_dma;
	struct platform_device	*pdev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		fcr;
	unsigned char		efr;
	unsigned char		dll;
	unsigned char		dlh;
	unsigned char		mdr1;
	unsigned char		wer;

	int			use_dma;
	bool			suspended;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
	unsigned char		msr_saved_flags;
	char			name[20];
	unsigned int		console_lock;
	unsigned long		port_activity;
	int			context_loss_cnt;
	/* RTS control via driver */
	unsigned		rts_mux_driver_control:1;
	unsigned		rts_pullup_in_suspend:1;

	unsigned int		errata;
	void (*enable_wakeup)(struct platform_device *, bool);
	bool (*chk_wakeup)(struct platform_device *);
	void (*wake_peer)(struct uart_port *);
};
#endif /* __OMAP_SERIAL_H__ */
