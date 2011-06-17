/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MODEM_PRJ_H__
#define __MODEM_PRJ_H__

#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/skbuff.h>


#define MAX_LINK_DEVTYPE 3
#define MAX_RAW_DEVS 32
#define MAX_NUM_IO_DEV	(MAX_RAW_DEVS + 4)

#define IOCTL_MODEM_ON	_IO('o', 0x19)
#define IOCTL_MODEM_OFF	_IO('o', 0x20)
#define IOCTL_MODEM_RESET	_IO('o', 0x21)
#define IOCTL_MODEM_BOOT_ON	_IO('o', 0x22)
#define IOCTL_MODEM_BOOT_OFF	_IO('o', 0x23)
#define IOCTL_MODEM_START	_IO('o', 0x24)

#define IOCTL_MODEM_SEND	_IO('o', 0x25)
#define IOCTL_MODEM_RECV	_IO('o', 0x26)

#define IOCTL_MODEM_STATUS		_IO('o', 0x27)

/* modem status */
#define MODEM_OFF	0
#define MODEM_CRASHED	1
#define MODEM_RAMDUMP	2
#define MODEM_POWER_ON	3
#define MODEM_BOOTING_NORMAL	4
#define MODEM_BOOTING_RAMDUMP	5
#define MODEM_DUMPING	6
#define MODEM_RUNNING	7

#define IPC_HEADER_MAX_SIZE	6 /* fmt 3, raw 6, rfs 6 */

/* Does modem ctl structure will use state ? or status defined below ?*/
enum modem_state {
	STATE_OFFLINE,
	STATE_CRASH_RESET, /* silent reset */
	STATE_CRASH_EXIT, /* cp ramdump */
	STATE_BOOTING,
	STATE_ONLINE,
};

enum {
	COM_NONE,
	COM_ONLINE,
	COM_HANDSHAKE,
	COM_BOOT,
	COM_CRASH,
};

struct header_data {
	char hdr[IPC_HEADER_MAX_SIZE];
	unsigned len;
	unsigned flag_len;
	char start; /*hdlc start header 0x7F*/
};

struct misc_data_io {
	uint32_t size;
	uint32_t id;
	uint32_t cmd;
	void *data;
};

struct vnet {
	struct io_device *iod;
};

struct io_device {
	struct list_head list;
	char *name;

	wait_queue_head_t wq;

	struct miscdevice miscdev;
	struct net_device *ndev;

	/* ID and Format for channel on the link */
	unsigned id;
	enum dev_format format;
	enum modem_io io_typ;

	struct sk_buff_head sk_rx_q;

	/* for fragmentation data from link device */
	struct sk_buff *skb_recv;
	struct header_data h_data;

	/* called from linkdevice when a packet arrives for this iodevice */
	int (*recv)(struct io_device *iod, const char *data, unsigned int len);

	/* inform the IO device that the modem is now online or offline or
	 * crashing or whatever...
	 */
	void (*modem_state_changed)(struct io_device *iod, enum modem_state);

	struct link_device *link;
	struct modem_ctl *mc;

	void *private_data;
};
#define to_io_device(misc) container_of(misc, struct io_device, miscdev)

struct io_raw_devices {
	struct io_device *raw_devices[MAX_RAW_DEVS];
	int num_of_raw_devs;
};

struct link_device {
	char *name;

	struct sk_buff_head sk_fmt_tx_q;
	struct sk_buff_head sk_raw_tx_q;

	struct workqueue_struct *tx_wq;
	struct work_struct tx_work;
	struct delayed_work tx_delayed_work;

	int irq; /* for dpram int */
	unsigned com_state;

	/* called during init to associate an io device with this link */
	int (*attach)(struct link_device *ld, struct io_device *iod);

	/* init communication - setting link driver */
	int (*init_comm)(struct link_device *ld, struct io_device *iod);

	/* called by an io_device when it has a packet to send over link
	 * - the io device is passed so the link device can look at id and
	 *   format fields to determine how to route/format the packet
	 */
	int (*send)(struct link_device *ld, struct io_device *iod,
				struct sk_buff *skb);
};

struct modemctl_ops {
	int (*modem_on) (struct modem_ctl *);
	int (*modem_off) (struct modem_ctl *);
	int (*modem_reset) (struct modem_ctl *);
	int (*modem_boot_on) (struct modem_ctl *);
	int (*modem_boot_off) (struct modem_ctl *);
};

struct modem_ctl {
	struct device *dev;
	char *name;

	int phone_state;

	unsigned gpio_cp_on;
	unsigned gpio_reset_req_n;
	unsigned gpio_cp_reset;
	unsigned gpio_pda_active;
	unsigned gpio_phone_active;
	unsigned gpio_cp_dump_int;
	unsigned gpio_flm_uart_sel;
	unsigned gpio_cp_warm_reset;

	int irq_phone_active;

	struct work_struct work;

#ifdef CONFIG_LTE_MODEM_CMC221
	const struct attribute_group *group;
	unsigned gpio_cp_off;
	unsigned gpio_slave_wakeup;
	unsigned gpio_host_wakeup;
	unsigned gpio_host_active;
	int irq_host_wakeup;
	struct delayed_work dwork;
	struct work_struct resume_work;
	int wakeup_flag; /*flag for CP boot GPIO sync flag*/
	int cpcrash_flag;
	struct completion *l2_done;
	int irq[3];
#endif /*CONFIG_LTE_MODEM_CMC221*/

	struct modemctl_ops ops;
	struct io_device *iod;
};

int init_io_device(struct io_device *iod);

#endif
