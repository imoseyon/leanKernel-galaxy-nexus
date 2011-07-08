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
#include <linux/wakelock.h>

#ifndef __MODEM_LINK_DEVICE_DPRAM_H__
#define __MODEM_LINK_DEVICE_DPRAM_H__

#define DPRAM_ERR_MSG_LEN	128
#define DPRAM_ERR_DEVICE	"dpramerr"

#define MAX_IDX			2

struct dpram_device {
	/* DPRAM memory addresses */
	u16           *in_head_addr;
	u16           *in_tail_addr;
	u8            *in_buff_addr;
	unsigned long  in_buff_size;

	u16           *out_head_addr;
	u16           *out_tail_addr;
	u8            *out_buff_addr;
	unsigned long  out_buff_size;

	unsigned long  in_head_saved;
	unsigned long  in_tail_saved;
	unsigned long  out_head_saved;
	unsigned long  out_tail_saved;

	u16            mask_req_ack;
	u16            mask_res_ack;
	u16            mask_send;
};

struct memory_region {
	u8 *control;
	u8 *fmt_out;
	u8 *raw_out;
	u8 *fmt_in;
	u8 *raw_in;
	u8 *mbx;
};

struct dpram_link_device {
	struct link_device ld;

	/* maybe -list of io devices for the link device to use
	 * to find where to send incoming packets to */
	struct list_head list_of_io_devices;

	/*only dpram*/
	struct wake_lock dpram_wake_lock;
	atomic_t raw_txq_req_ack_rcvd;
	atomic_t fmt_txq_req_ack_rcvd;
	u8 is_net_stopped ;
	int phone_sync;
	u8 phone_status;
	 struct work_struct xmit_work_struct;
	int dpram_init_cmd_wait_condition;

	struct dpram_device dev_map[MAX_IDX];

	u8 dpram_read_data[131072];
	wait_queue_head_t dpram_init_cmd_wait_q;

	int modem_pif_init_wait_condition;
	wait_queue_head_t modem_pif_init_done_wait_q;

	unsigned int is_dpram_err ;
	char dpram_err_buf[DPRAM_ERR_MSG_LEN];
	struct fasync_struct *dpram_err_async_q;

	void (*clear_interrupt)(struct dpram_link_device *);

	char cpdump_debug_file_name[DPRAM_ERR_MSG_LEN];

	struct memory_region m_region;

	unsigned long  fmt_out_buff_size;
	unsigned long  raw_out_buff_size;
	unsigned long  fmt_in_buff_size;
	unsigned long  raw_in_buff_size;
};
/* converts from struct link_device* to struct xxx_link_device* */
#define to_dpram_link_device(linkdev) \
			container_of(linkdev, struct dpram_link_device, ld)
#endif
