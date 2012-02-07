/*
 * hsi_driver.h
 *
 * Header file for the HSI driver low level interface.
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __HSI_DRIVER_H__
#define __HSI_DRIVER_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>

#include <linux/hsi_driver_if.h>
#include <plat/omap_hsi.h>

/* Channel states */
#define	HSI_CH_OPEN		  0x01
#define HSI_CH_RX_POLL	0x10
#define HSI_CH_ACWAKE		0x02	/* ACWAKE line status */

#define HSI_CH_NUMBER_NONE	0xFF
/*
 * The number of channels handled by the driver in the ports, or the highest
 * port channel number (+1) used. (MAX:8 for SSI; 16 for HSI)
 * Reducing this value optimizes the driver memory footprint.
 */
#define HSI_PORT_MAX_CH		HSI_CHANNELS_MAX

/* Number of DMA channels when nothing is defined for the device */
#define HSI_DMA_CHANNEL_DEFAULT		8

/* Defines bit number for atomic operations */
#define HSI_FLAGS_TASKLET_LOCK		0 /* prevents to disable IRQ and */
					  /* schedule tasklet more than once */


#define LOG_NAME		"OMAP HSI: "

/* SW strategies for HSI FIFO mapping */
enum {
	HSI_FIFO_MAPPING_UNDEF = 0,
	HSI_FIFO_MAPPING_ALL_PORT1,	/* ALL FIFOs mapped on port 1 */
	HSI_FIFO_MAPPING_ALL_PORT2,	/* ALL FIFOs mapped on port 2 */
	HSI_FIFO_MAPPING_SSI,	/* 8 FIFOs per port (SSI compatible mode) */
};

/* Device identifying constants */
enum {
	HSI_DRV_DEVICE_HSI,
	HSI_DRV_DEVICE_SSI
};

/**
 * struct hsi_data - HSI buffer descriptor
 * @addr: pointer to the buffer where to send or receive data
 * @size: size in words (32 bits) of the buffer
 * @lch: associated GDD (DMA) logical channel number, if any
 */
struct hsi_data {
	u32 *addr;
	unsigned int size;
	int lch;
};

/**
 * struct hsi_channel - HSI channel data
 * @read_data: Incoming HSI buffer descriptor
 * @write_data: Outgoing HSI buffer descriptor
 * @hsi_port: Reference to port where the channel belongs to
 * @flags: Tracks if channel has been open
 * @channel_number: HSI channel number
 * @rw_lock: Read/Write lock to serialize access to callback and hsi_device
 * @dev: Reference to the associated hsi_device channel
 * @write_done: Callback to signal TX completed.
 * @read_done: Callback to signal RX completed.
 * @port_event: Callback to signal port events (RX Error, HWBREAK, CAWAKE ...)
 */
struct hsi_channel {
	struct hsi_data read_data;
	struct hsi_data write_data;
	struct hsi_port *hsi_port;
	u8 flags;
	u8 channel_number;
	rwlock_t rw_lock;
	struct hsi_device *dev;
	void (*write_done) (struct hsi_device *dev, unsigned int size);
	void (*read_done) (struct hsi_device *dev, unsigned int size);
	void (*port_event) (struct hsi_device *dev, unsigned int event,
			    void *arg);
};

/**
 * struct hsi_port - hsi port driver data
 * @hsi_channel: Array of channels in the port
 * @hsi_controller: Reference to the HSI controller
 * @flags: atomic flags (for atomic operations)
 * @port_number: port number. Range [1,2]
 * @max_ch: maximum number of channels supported on the port
 * @n_irq: HSI irq line use to handle interrupts (0 or 1)
 * @irq: IRQ number
 * @wake_rx_3_wires_mode: receiver 3 wires mode (1) or 4 wires mode (0)
 * @cawake_gpio: GPIO number for cawake line (-1 if none)
 * @cawake_gpio_irq: IRQ number for cawake gpio events
 * @cawake_status: Tracks CAWAKE line status
 * @cawake_off_event: True if CAWAKE event was detected from OFF mode
 * @acwake_status: Bitmap to track ACWAKE line status per channel
 * @in_int_tasklet: True if interrupt tasklet for this port is currently running
 * @in_cawake_tasklet: True if CAWAKE tasklet for this port is currently running
 * @tasklet_lock: prevents to disable IRQ and schedule tasklet more than once
 * @counters_on: indicates if the HSR counters are in use or not
 * @reg_counters: stores the previous counters values when deactivated
 * @lock: Serialize access to the port registers and internal data
 * @hsi_tasklet: Bottom half for interrupts when clocks are enabled
 * @cawake_tasklet: Bottom half for cawake events
 */
struct hsi_port {
	struct hsi_channel hsi_channel[HSI_PORT_MAX_CH];
	struct hsi_dev *hsi_controller;
	unsigned long flags;
	u8 port_number;
	u8 max_ch;
	u8 n_irq;
	int irq;
	int wake_rx_3_wires_mode;
	int cawake_gpio;
	int cawake_gpio_irq;
	int cawake_status;
	bool cawake_off_event;
	unsigned int acwake_status;	/* HSI_TODO : fine tune init values */
	bool in_int_tasklet;
	bool in_cawake_tasklet;
	int counters_on;
	unsigned long reg_counters;
	spinlock_t lock; /* access to the port registers and internal data */
	struct tasklet_struct hsi_tasklet;
	struct tasklet_struct cawake_tasklet;
};

/**
 * struct hsi_dev - hsi controller driver data
 * This structure is saved into platform_device->dev->p->driver_data
 *
 * @hsi_port: Array of hsi ports enabled in the controller
 * @id: HSI controller platform id number
 * @max_p: Number of ports enabled in the controller
 * @hsi_clk: Reference to the HSI custom clock
 * @base: HSI registers base virtual address
 * @phy_base: HSI registers base physical address
 * @lock: Serializes access to internal data and regs
 * @clock_enabled: Indicates if HSI Clocks are ON
 * @clock_rate: Indicates current HSI Fclock speed
 * @gdd_irq: GDD (DMA) irq number
 * @fifo_mapping_strategy: Selected strategy for fifo to ports/channels mapping
 * @gdd_usecount: Holds the number of ongoning DMA transfers
 * @last_gdd_lch: Last used GDD logical channel
 * @gdd_chan_count: Number of available DMA channels on the device (must be ^2)
 * @in_dma_tasklet: True if DMA tasklet for the controller is currently running
 * @set_min_bus_tput: (PM) callback to set minimun bus throuput
 * @clk_notifier_register: (PM) callabck for DVFS support
 * @clk_notifier_unregister: (PM) callabck for DVFS support
 * @hsi_nb: (PM) Notification block for DVFS notification chain
 * @hsi_gdd_tasklet: Bottom half for DMA Interrupts when clocks are enabled
 * @dir: debugfs base directory
 * @dev: Reference to the HSI platform device
 */
struct hsi_dev { /* HSI_TODO:  should be later renamed into hsi_controller*/
	struct hsi_port hsi_port[HSI_MAX_PORTS];
	int id;
	u8 max_p;
	void __iomem *base;
	unsigned long phy_base;
	spinlock_t lock; /* Serializes access to internal data and regs */
	bool clock_enabled;
	unsigned long clock_rate;
	int gdd_irq;
	unsigned int fifo_mapping_strategy;
	unsigned int gdd_usecount;
	unsigned int last_gdd_lch;
	unsigned int gdd_chan_count;
	bool in_dma_tasklet;
	void (*set_min_bus_tput) (struct device *dev, u8 agent_id,
				  unsigned long r);
	struct notifier_block hsi_nb;
	struct tasklet_struct hsi_gdd_tasklet;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dir;
#endif
	struct device *dev;
};

/**
 * struct hsi_platform_data - Board specific data
*/
struct hsi_platform_data {
	void (*set_min_bus_tput) (struct device *dev, u8 agent_id,
						unsigned long r);
	int (*device_enable) (struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle) (struct platform_device *pdev);
	int (*device_scale) (struct device *req_dev, struct device *target_dev,
			unsigned long rate);
	int (*wakeup_enable) (int hsi_port);
	int (*wakeup_disable) (int hsi_port);
	bool (*wakeup_is_from_hsi) (int *hsi_port);
	int (*board_suspend)(int hsi_port, bool dev_may_wakeup);
	int (*board_resume)(int hsi_port);
	u8 num_ports;
	struct hsi_ctrl_ctx *ctx;
	u8 hsi_gdd_chan_count;
	unsigned long default_hsi_fclk;
	unsigned int fifo_mapping_strategy;
	u32 errata;
};

/* HSI Bus */
extern struct bus_type hsi_bus_type;

int hsi_port_event_handler(struct hsi_port *p, unsigned int event, void *arg);
int hsi_bus_init(void);
void hsi_bus_exit(void);
/* End HSI Bus */

void hsi_reset_ch_read(struct hsi_channel *ch);
void hsi_reset_ch_write(struct hsi_channel *ch);
bool hsi_is_channel_busy(struct hsi_channel *ch);
bool hsi_is_hsi_port_busy(struct hsi_port *pport);
bool hsi_is_hsi_controller_busy(struct hsi_dev *hsi_ctrl);
bool hsi_is_hst_port_busy(struct hsi_port *pport);
bool hsi_is_hst_controller_busy(struct hsi_dev *hsi_ctrl);
void hsi_driver_ack_interrupt(struct hsi_port *pport, u32 flag, bool backup);
bool hsi_driver_is_interrupt_pending(struct hsi_port *pport, u32 flag,
					bool backup);
int hsi_driver_enable_interrupt(struct hsi_port *pport, u32 flag);
int hsi_driver_disable_interrupt(struct hsi_port *pport, u32 flag);
int hsi_driver_enable_read_interrupt(struct hsi_channel *hsi_channel,
					u32 *data);
int hsi_driver_enable_write_interrupt(struct hsi_channel *hsi_channel,
					u32 *data);
bool hsi_is_dma_read_int_pending(struct hsi_dev *hsi_ctrl);
int hsi_driver_read_dma(struct hsi_channel *hsi_channel, u32 * data,
			unsigned int count);
int hsi_driver_write_dma(struct hsi_channel *hsi_channel, u32 * data,
			 unsigned int count);

int hsi_driver_cancel_read_interrupt(struct hsi_channel *ch);
int hsi_driver_cancel_write_interrupt(struct hsi_channel *ch);
void hsi_driver_disable_read_interrupt(struct hsi_channel *ch);
void hsi_driver_disable_write_interrupt(struct hsi_channel *ch);
int hsi_driver_cancel_write_dma(struct hsi_channel *ch);
int hsi_driver_cancel_read_dma(struct hsi_channel *ch);
int hsi_do_cawake_process(struct hsi_port *pport);

int hsi_driver_device_is_hsi(struct platform_device *dev);

int hsi_mpu_init(struct hsi_port *hsi_p, const char *irq_name);
void hsi_mpu_exit(struct hsi_port *hsi_p);

int hsi_gdd_init(struct hsi_dev *hsi_ctrl, const char *irq_name);
void hsi_gdd_exit(struct hsi_dev *hsi_ctrl);

int hsi_cawake_init(struct hsi_port *port, const char *irq_name);
void hsi_cawake_exit(struct hsi_port *port);

int hsi_fifo_get_id(struct hsi_dev *hsi_ctrl, unsigned int channel,
		    unsigned int port);
int hsi_fifo_get_chan(struct hsi_dev *hsi_ctrl, unsigned int fifo,
		      unsigned int *channel, unsigned int *port);
int hsi_fifo_mapping(struct hsi_dev *hsi_ctrl, unsigned int mtype);
long hsi_hst_bufstate_f_reg(struct hsi_dev *hsi_ctrl,
			    unsigned int port, unsigned int channel);
long hsi_hsr_bufstate_f_reg(struct hsi_dev *hsi_ctrl,
			    unsigned int port, unsigned int channel);
long hsi_hst_buffer_reg(struct hsi_dev *hsi_ctrl,
			unsigned int port, unsigned int channel);
long hsi_hsr_buffer_reg(struct hsi_dev *hsi_ctrl,
			unsigned int port, unsigned int channel);
u8 hsi_get_rx_fifo_occupancy(struct hsi_dev *hsi_ctrl, u8 fifo);
void hsi_set_pm_force_hsi_on(struct hsi_dev *hsi_ctrl);
void hsi_set_pm_default(struct hsi_dev *hsi_ctrl);
int hsi_softreset(struct hsi_dev *hsi_ctrl);
void hsi_softreset_driver(struct hsi_dev *hsi_ctrl);

void hsi_clocks_disable_channel(struct device *dev, u8 channel_number,
				const char *s);
int hsi_clocks_enable_channel(struct device *dev, u8 channel_number,
				const char *s);
#ifdef CONFIG_PM_RUNTIME
extern int hsi_runtime_resume(struct device *dev);
extern int hsi_runtime_suspend(struct device *dev);
#else
static inline int hsi_runtime_resume(struct device *dev) { return -ENOSYS; }
static inline int hsi_runtime_suspend(struct device *dev) { return -ENOSYS; }
#endif
void hsi_save_ctx(struct hsi_dev *hsi_ctrl);
void hsi_restore_ctx(struct hsi_dev *hsi_ctrl);


#ifdef CONFIG_DEBUG_FS
int hsi_debug_init(void);
void hsi_debug_exit(void);
int hsi_debug_add_ctrl(struct hsi_dev *hsi_ctrl);
void hsi_debug_remove_ctrl(struct hsi_dev *hsi_ctrl);
#else
#define	hsi_debug_add_ctrl(hsi_ctrl)	0
#define	hsi_debug_remove_ctrl(hsi_ctrl)
#define	hsi_debug_init()		0
#define	hsi_debug_exit()
#endif /* CONFIG_DEBUG_FS */

static inline struct hsi_channel *hsi_ctrl_get_ch(struct hsi_dev *hsi_ctrl,
					      unsigned int port,
					      unsigned int channel)
{
	return &hsi_ctrl->hsi_port[port - 1].hsi_channel[channel];
}

/* HSI IO access */
static inline u32 hsi_inl(void __iomem *base, u32 offset)
{
	return inl((unsigned int)base + offset);
}

static inline void hsi_outl(u32 data, void __iomem *base, u32 offset)
{
	outl(data, (unsigned int)base + offset);
}

static inline void hsi_outl_or(u32 data, void __iomem *base, u32 offset)
{
	u32 tmp = hsi_inl(base, offset);
	hsi_outl((tmp | data), base, offset);
}

static inline void hsi_outl_and(u32 data, void __iomem *base, u32 offset)
{
	u32 tmp = hsi_inl(base, offset);
	hsi_outl((tmp & data), base, offset);
}

static inline u16 hsi_inw(void __iomem *base, u32 offset)
{
	return inw((unsigned int)base + offset);
}

static inline void hsi_outw(u16 data, void __iomem *base, u32 offset)
{
	outw(data, (unsigned int)base + offset);
}

static inline void hsi_outw_or(u16 data, void __iomem *base, u32 offset)
{
	u16 tmp = hsi_inw(base, offset);
	hsi_outw((tmp | data), base, offset);
}

static inline void hsi_outw_and(u16 data, void __iomem *base, u32 offset)
{
	u16 tmp = hsi_inw(base, offset);
	hsi_outw((tmp & data), base, offset);
}

static inline int hsi_get_cawake(struct hsi_port *port)
{
	struct platform_device *pdev =
				to_platform_device(port->hsi_controller->dev);

	if (hsi_driver_device_is_hsi(pdev))
		return (HSI_HSR_MODE_WAKE_STATUS ==
			(hsi_inl(port->hsi_controller->base,
				HSI_HSR_MODE_REG(port->port_number)) &
				HSI_HSR_MODE_WAKE_STATUS));
	else if (port->cawake_gpio >= 0)
		return gpio_get_value(port->cawake_gpio);
	else
		return -ENXIO;
}

static inline void hsi_clocks_disable(struct device *dev, const char *s)
{
	hsi_clocks_disable_channel(dev, HSI_CH_NUMBER_NONE, s);
}

static inline int hsi_clocks_enable(struct device *dev, const char *s)
{
	return hsi_clocks_enable_channel(dev, HSI_CH_NUMBER_NONE, s);
}

static inline int is_hsi_errata(struct hsi_dev *hsi_ctrl, unsigned int id)
{
	struct hsi_platform_data *pdata = dev_get_platdata(hsi_ctrl->dev);

	return IS_HSI_ERRATA(pdata->errata, id);
}

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP4)
extern void omap_pm_clear_dsp_wake_up(void);
#else
#define static inline void omap_pm_clear_dsp_wake_up(void) { }
#endif

#endif /* __HSI_DRIVER_H__ */
