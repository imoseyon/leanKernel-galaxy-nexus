/*
 * hsi-if.h
 *
 * Part of the HSI character driver, private headers.
 *
 * Copyright (C) 2009 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Andras Domokos <andras.domokos@nokia.com>
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

#ifndef _HSI_IF_H
#define _HSI_IF_H

#define HSI_EV_MASK		(0xffff << 0)
#define HSI_EV_TYPE_MASK	(0x0f << 16)
#define HSI_EV_IN		(0x01 << 16)
#define HSI_EV_OUT		(0x02 << 16)
#define HSI_EV_EXCEP		(0x03 << 16)
#define HSI_EV_AVAIL		(0x04 << 16)
#define HSI_EV_TYPE(event)	((event) & HSI_EV_TYPE_MASK)

#define HSI_HWBREAK		1
#define HSI_ERROR		2

#define HSI_MAX_CHANNELS	16
#define CHANNEL_MASK		0xFF
#define HSI_LL_INVALID_CHANNEL	0xFF

struct hsi_event {
	unsigned int event;
	u32 *data;
	unsigned int count;
};

struct if_hsi_channel {
	struct hsi_device *dev;
	unsigned int channel_id;
	u32 *tx_data;
	unsigned int tx_count;
	u32 *rx_data;
	unsigned int rx_count;
	unsigned int opened;
	unsigned int state;
	u32 *tx_buf;
	u32 *rx_buf;
	unsigned int tx_state;
	unsigned int rx_state;
	unsigned int tx_nak_count;
	unsigned int rx_nak_count;
	spinlock_t lock; /* Serializes access to channel data */
};

struct if_hsi_iface {
	struct if_hsi_channel channels[HSI_MAX_CHANNELS];
#if 0
	int bootstrap;
#endif
	unsigned long init_chan_map;
	spinlock_t lock; /* Serializes access to HSI functional interface */
};

struct if_hsi_cmd {
	u32 tx_cmd[50];
	u32 rx_cmd[50];
	struct timespec tx_cmd_time[50];
	struct timespec rx_cmd_time[50];
};

enum {
	HSI_LL_MSG_BREAK           = 0x00,
	HSI_LL_MSG_ECHO            = 0x01,
	HSI_LL_MSG_INFO_REQ        = 0x02,
	HSI_LL_MSG_INFO            = 0x03,
	HSI_LL_MSG_CONFIGURE       = 0x04,
	HSI_LL_MSG_ALLOCATE_CH     = 0x05,
	HSI_LL_MSG_RELEASE_CH      = 0x06,
	HSI_LL_MSG_OPEN_CONN       = 0x07,
	HSI_LL_MSG_CONN_READY      = 0x08,
	HSI_LL_MSG_CONN_CLOSED     = 0x09,
	HSI_LL_MSG_CANCEL_CONN     = 0x0A,
	HSI_LL_MSG_ACK             = 0x0B,
	HSI_LL_MSG_NAK             = 0x0C,
	HSI_LL_MSG_CONF_RATE       = 0x0D,
	HSI_LL_MSG_OPEN_CONN_OCTET = 0x0E,
	HSI_LL_MSG_INVALID         = 0xFF,
};

enum {
	HSI_LL_TX_STATE_UNDEF,
	HSI_LL_TX_STATE_CLOSED,
	HSI_LL_TX_STATE_IDLE,
	HSI_LL_TX_STATE_POWER_DOWN,
	HSI_LL_TX_STATE_ERROR,
	HSI_LL_TX_STATE_SEND_OPEN_CONN,
	HSI_LL_TX_STATE_WAIT_FOR_ACK,
	HSI_LL_TX_STATE_NACK,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_READY,
	HSI_LL_TX_STATE_SEND_CONF_RATE,
	HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK,
	HSI_LL_TX_STATE_TX,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED,
	HSI_LL_TX_STATE_TO_OPEN_CONN,
	HSI_LL_TX_STATE_TO_ACK,
	HSI_LL_TX_STATE_TO_READY,
	HSI_LL_TX_STATE_TO_CONF,
	HSI_LL_TX_STATE_TO_CONF_ACK,
	HSI_LL_TX_STATE_TO_TX,
	HSI_LL_TX_STATE_TO_CLOSE,
	HSI_LL_TX_STATE_SEND_BREAK,
};

enum {
	HSI_LL_RX_STATE_UNDEF,
	HSI_LL_RX_STATE_CLOSED,
	HSI_LL_RX_STATE_IDLE,
	HSI_LL_RX_STATE_POWER_DOWN,
	HSI_LL_RX_STATE_ERROR,
	HSI_LL_RX_STATE_BLOCKED,
	HSI_LL_RX_STATE_SEND_ACK,
	HSI_LL_RX_STATE_SEND_NACK,
	HSI_LL_RX_STATE_SEND_CONN_READY,
	HSI_LL_RX_STATE_RX,
	HSI_LL_RX_STATE_SEND_CONN_CLOSED,
	HSI_LL_RX_STATE_SEND_CONN_CANCEL,
	HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK,
	HSI_LL_RX_STATE_SEND_CONF_ACK,
	HSI_LL_RX_STATE_SEND_CONF_NACK,
	HSI_LL_RX_STATE_TO_RX,
	HSI_LL_RX_STATE_TO_ACK,
	HSI_LL_RX_STATE_TO_NACK,
	HSI_LL_RX_STATE_TO_CONN_READY,
	HSI_LL_RX_STATE_TO_CONN_CLOSED,
	HSI_LL_RX_STATE_TO_CONN_CANCEL,
	HSI_LL_RX_STATE_TO_CONN_CANCEL_ACK,
	HSI_LL_RX_STATE_TO_CONF_ACK,
	HSI_LL_RX_STATE_SEND_BREAK,
};


int if_hsi_init(void);
int if_hsi_exit(void);

int if_hsi_start(int ch);
void if_hsi_stop(int ch);

void if_hsi_send_break(int ch);
void if_hsi_flush_rx(int ch);
void if_hsi_flush_tx(int ch);
void if_hsi_bootstrap(int ch);
void if_hsi_set_wakeline(int ch, unsigned int state);
void if_hsi_get_wakeline(int ch, unsigned int *state);

#if 0
int if_hsi_set_rx(int ch, struct hsi_rx_config *cfg);
void if_hsi_get_rx(int ch, struct hsi_rx_config *cfg);
int if_hsi_set_tx(int ch, struct hsi_tx_config *cfg);
void if_hsi_get_tx(int ch, struct hsi_tx_config *cfg);
#endif

int if_hsi_read(int ch, u32 *data, unsigned int count);
int if_hsi_poll(int ch);
int if_hsi_write(int ch, u32 *data, unsigned int count);

void if_hsi_cancel_read(int ch);
void if_hsi_cancel_write(int ch);

void if_notify(int ch, struct hsi_event *ev);
int hsi_proto_read(int ch, u32 *buffer, int count);
int hsi_proto_write(int ch, u32 *buffer, int length);
int hsi_decode_cmd(u32 *data, u32 *cmd, u32 *ch, u32 *param);
int protocol_create_cmd(int cmd_type, unsigned int channel, void *arg);
int hsi_protocol_send_command(u32 cmd, u32 channel, u32 param);
void rx_stm(u32 cmd, u32 ch, u32 param);
#if 0
int hsi_start_protocol(void);
#endif
#endif /* _HSI_IF_H */
