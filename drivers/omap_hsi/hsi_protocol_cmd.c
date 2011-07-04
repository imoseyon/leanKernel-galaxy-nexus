/*
 * File -  hsi_protocol_if_cmd.c
 *
 * Implements HSI protocol for Infineon Modem.
 *
 * Copyright (C) 2011 Samsung Electronics. All rights reserved.
 *
 * Author: Rupesh Gujare <rupesh.g@samsung.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/hsi_driver_if.h>
#include "hsi-protocol-if.h"

extern struct if_hsi_iface hsi_protocol_iface;
extern wait_queue_head_t ipc_read_wait, ipc_write_wait;
int if_hsi_openchannel(struct if_hsi_channel *channel);
int if_hsi_closechannel(struct if_hsi_channel *channel);

extern struct if_hsi_cmd hsi_cmd_history;
extern int tx_cmd_history_p;
extern int rx_cmd_history_p;

/*Decode command from received PDU on channle 0*/
int hsi_decode_cmd(u32 *cmd_data, u32 *cmd, u32 *ch, u32 *param)
{
	int ret = 0;
	u32 data = *cmd_data;
	u8 lrc_cal, lrc_act;
	u8 val1, val2, val3;

	*cmd = ((data & 0xF0000000) >> 28);

	switch (*cmd) {
	case    HSI_LL_MSG_BREAK:
		pr_err("Command MSG_BREAK Received.\n");
		break;

	case    HSI_LL_MSG_OPEN_CONN:
		*ch = ((data & 0x0F000000) >> 24);
		*param   = ((data & 0x00FFFF00) >> 8);
		/*Check LRC*/
		val1 = ((data & 0xFF000000) >> 24);
		val2 = ((data & 0x00FF0000) >> 16);
		val3 = ((data & 0x0000FF00) >>  8);
		lrc_act = (data & 0x000000FF);
		lrc_cal = val1 ^ val2 ^ val3;
		if (lrc_cal != lrc_act)
			ret = -1;
		break;

	case HSI_LL_MSG_CONN_READY:
	case HSI_LL_MSG_CONN_CLOSED:
	case HSI_LL_MSG_CANCEL_CONN:
	case HSI_LL_MSG_NAK:
		*ch = ((data & 0x0F000000) >> 24);
		break;

	case HSI_LL_MSG_ACK:
		*ch = ((data & 0x0F000000) >> 24);
		*param = (data & 0x00FFFFFF);
		//printk(KERN_INFO "ACK Received ch=%d, param=%d\n",*ch, *param);
		break;

	case HSI_LL_MSG_CONF_RATE:
		*ch = ((data & 0x0F000000) >> 24);
		*param   = ((data & 0x0F000000) >> 24);
		break;

	case HSI_LL_MSG_OPEN_CONN_OCTET:
		*ch = ((data & 0x0F000000) >> 24);
		*param   = (data & 0x00FFFFFF);
		break;

	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
		*cmd = HSI_LL_MSG_INVALID;
		*ch  = HSI_LL_INVALID_CHANNEL;
		ret = -1;
		break;
	}
	return ret;
}

int protocol_create_cmd(int cmd_type, unsigned int channel, void *arg)
{
	unsigned int command = 0;
	int ret = 0;

	switch (cmd_type) {
	case HSI_LL_MSG_BREAK:
		{
			command = 0;
		}
		break;

	case HSI_LL_MSG_OPEN_CONN:
		{
			unsigned int size = *(unsigned int *)arg;
			unsigned int lcr  = 0;

/*       if(size > 4)
	 size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2);
       else
	 size = 1;*/

			command = ((HSI_LL_MSG_OPEN_CONN & 0x0000000F) << 28) |
				  ((channel              & 0x000000FF) << 24) |
				  ((size                 & 0x0000FFFF) << 8);

			lcr = ((command & 0xFF000000) >> 24) ^
			      ((command & 0x00FF0000) >> 16) ^
			      ((command & 0x0000FF00) >>  8);

			command = command | (lcr & 0x000000FF);
		}
		break;

	case HSI_LL_MSG_CONN_READY:
		{
			command = ((HSI_LL_MSG_CONN_READY & 0x0000000F) << 28) |
				  ((channel               & 0x000000FF) << 24);
		}
		break;

	case HSI_LL_MSG_CONN_CLOSED:
		{
			command = ((HSI_LL_MSG_CONN_CLOSED & 0x0000000F) << 28) |
				  ((channel                & 0x000000FF) << 24);
		}
		break;

	case HSI_LL_MSG_CANCEL_CONN:
		{
			unsigned int role = *(unsigned int *)arg;

			command = ((HSI_LL_MSG_CANCEL_CONN & 0x0000000F) << 28) |
				  ((channel                & 0x000000FF) << 24) |
				  ((role                   & 0x000000FF) << 16);
		}
		break;

	case HSI_LL_MSG_ACK:
		{
			unsigned int echo_params = *(unsigned int *)arg;

			command = ((HSI_LL_MSG_ACK & 0x0000000F) << 28) |
				  ((channel        & 0x000000FF) << 24) |
				  ((echo_params    & 0x00FFFFFF));
		}
		break;

	case HSI_LL_MSG_NAK:
		{
			command = ((HSI_LL_MSG_NAK & 0x0000000F) << 28) |
				  ((channel        & 0x000000FF) << 24);
		}
		break;

	case HSI_LL_MSG_CONF_RATE:
		{
			unsigned int baud_rate = *(unsigned int *)arg;

			command = ((HSI_LL_MSG_CONF_RATE & 0x0000000F) << 28) |
				  ((channel              & 0x000000FF) << 24) |
				  ((baud_rate            & 0x00FFFFFF));
		}
		break;

	case HSI_LL_MSG_OPEN_CONN_OCTET:
		{
			unsigned int size = *(unsigned int *)arg;

			command = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F) << 28) |
				  ((channel                    & 0x000000FF) << 24) |
				  ((size                       & 0x00FFFFFF));

		}
		break;

	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
		ret = -1;
		break;
	}
	return command;
}

int set_tx_config(struct if_hsi_channel *ch, u32 mode, u32 max_channels)
{
	struct hst_ctx tx_config;
	int ret;

	hsi_ioctl(ch->dev, HSI_IOCTL_GET_TX, &tx_config);
	tx_config.mode = mode;
	tx_config.channels = max_channels;
	ret = hsi_ioctl(ch->dev, HSI_IOCTL_SET_TX, &tx_config);
	return ret;
}

static int saved_cmd_queue = 0;
static u32 cmd_saved[5];
int hsi_protocol_send_command(u32 cmd, u32 channel, u32 param)
{
	struct if_hsi_channel *channel_zero;
	u32 cmd_array[4] = {0x00000000, 0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC}, ret = -1;

	channel_zero = &hsi_protocol_iface.channels[0];
	cmd_array[0] = protocol_create_cmd(cmd, channel, &param);
	pr_debug("[%s] CMD = %08x\n",__func__, cmd_array[0]);
	while (channel_zero->tx_state != HSI_LL_TX_STATE_IDLE) {
		cmd_saved[saved_cmd_queue] = cmd_array[0];
		saved_cmd_queue++;
		pr_debug("(%s) cmd_saved : %x(%d)\n", __func__, cmd_array[0], saved_cmd_queue);

		return 0;
	}

send_retry:

	channel_zero->tx_state = HSI_LL_TX_STATE_TX;

	// For es 2.1 ver.
	ret = hsi_proto_write(0, cmd_array, 4);
	if (ret < 0) {
		pr_err("(%s) Command Write failed, CMD->%X\n", __func__, cmd_array[0]);
		channel_zero->tx_state = HSI_LL_TX_STATE_IDLE;
		return -1;
	} else {
		channel_zero->tx_state = HSI_LL_TX_STATE_IDLE;

		pr_debug("[%s] CMD = %08x\n", __func__, cmd_array[0]);

		hsi_cmd_history.tx_cmd[tx_cmd_history_p] = cmd_array[0];
		hsi_cmd_history.tx_cmd_time[tx_cmd_history_p] = CURRENT_TIME;
		tx_cmd_history_p++;
		if (tx_cmd_history_p >= 50)
			tx_cmd_history_p = 0;

		if (saved_cmd_queue) {
			saved_cmd_queue--;
			cmd_array[0] = cmd_saved[saved_cmd_queue];

			goto send_retry;
		}

		return 0;
	}
}

void rx_stm(u32 cmd, u32 ch, u32 param)
{
	struct if_hsi_channel *channel;
	u32 size = 0, tmp_cmd = 0, ret, i;
	channel = &hsi_protocol_iface.channels[ch];

	switch (cmd) {
	case HSI_LL_MSG_OPEN_CONN:
		pr_err("ERROR... OPEN_CONN Not supported. Should use OPEN_CONN_OCTECT instead.\n");
		break;

	case HSI_LL_MSG_ECHO:
		pr_err("ERROR... HSI_LL_MSG_ECHO not supported.\n");
		break;

	case HSI_LL_MSG_CONN_CLOSED:
		switch (channel->tx_state) {
		case HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED:
			channel->tx_state = HSI_LL_TX_STATE_IDLE;

			/* ACWAKE ->LOW */
			ret = hsi_ioctl(hsi_protocol_iface.channels[0].dev, HSI_IOCTL_ACWAKE_DOWN, NULL);
			if (ret == 0)
				pr_debug("ACWAKE pulled low in %s()\n", __func__);
			else
				pr_err("ACWAKE pulled low in %s() ERROR : %d\n", __func__, ret);

			pr_debug("[%s] Received CONN_CLOSED. ch-> %d\n", __func__,ch);
			break;

		default:
			pr_err("Wrong STATE for CONN_CLOSED\n");
		}
		break;

	case HSI_LL_MSG_CANCEL_CONN:
		pr_debug("Received CANCEL_CONN\n");
		break;

	case HSI_LL_MSG_ACK:
		switch (channel->tx_state) {
		case HSI_LL_TX_STATE_WAIT_FOR_ACK:
		case HSI_LL_TX_STATE_SEND_OPEN_CONN:
			//printk(KERN_INFO "ACK received %s()\n",__func__);

			channel->tx_state = HSI_LL_TX_STATE_TX;
			size = param;
#if 0
			// TEMP: send/read by 16 byte unit for v.11A(CP)
			if ((size > 16) && (size % 16))
				size += (16 - (size % 16));
			else if (size < 16)
				size = 16;
#endif

			// For es 2.1 ver.
			if (size % 4)
				size += (4 - (size % 4));

			pr_debug("Writing %d bytes data on channel %d, tx_buf = %x,  in %s()\n", size, ch, channel->tx_buf, __func__);
			ret = hsi_proto_write(ch, channel->tx_buf, size);
			channel->tx_state = HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED;
			wake_up_interruptible(&ipc_write_wait);
			channel->tx_nak_count = 0;
			break;

		case HSI_LL_TX_STATE_CLOSED:/* ACK as response to CANCEL_CONN */
			if (channel->rx_state == HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK)
				channel->rx_state = HSI_LL_RX_STATE_IDLE;
			break;

		case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK:	/* ACK as response to CONF_RATE */
			//TODO: SET CONF RATE
			pr_debug("ACK Received for CONF_RATE\n");
			break;

		default:
			pr_err("ACK Received for Unknown state\n");
		}
		break;

	case HSI_LL_MSG_NAK:
		switch (channel->tx_state) {
		case HSI_LL_TX_STATE_WAIT_FOR_ACK:
			printk(KERN_INFO "(%s) NAK received. ch->%d\n", __func__, ch);
			//channel->tx_state = HSI_LL_TX_STATE_NACK;
			if (channel->tx_nak_count < 10) {
				msleep(10);

				tmp_cmd = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F) << 28) |
					  ((ch                         & 0x000000FF) << 24);
				for (i = 49; i >= 0; i--) {
					if ((hsi_cmd_history.tx_cmd[i] & 0xFFF00000) == tmp_cmd)
						break;
				}
				size = (hsi_cmd_history.tx_cmd[i] & 0x000FFFFF);

				pr_debug("(%s) Re Send OPEN CONN ch->%d, size->%d, count->%d\n", __func__, ch, size, channel->tx_nak_count);

				hsi_protocol_send_command(HSI_LL_MSG_OPEN_CONN_OCTET, ch, size);
				channel->tx_nak_count++;
			} else {
				hsi_protocol_send_command(HSI_LL_MSG_BREAK, ch, size);
				pr_debug("(%s) Sending MSG_BREAK. ch->%d\n", __func__, ch);
				//TODO Reset All channels and inform IPC write about failure (Possibly by sending signal)
			}
			break;

		case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK:	/* NAK as response to CONF_RATE */
			channel->tx_state = HSI_LL_TX_STATE_IDLE;
			break;

		default:
			pr_err("ERROR - Received NAK in invalid state. state->%d\n", channel->tx_state);
		}
		break;

	case HSI_LL_MSG_CONF_RATE:
		//TODO: Set Conf Rate
		pr_debug("CONF_RATE Received\n");
		break;

	case HSI_LL_MSG_OPEN_CONN_OCTET:
		switch (channel->rx_state) {
		/* case HSI_LL_RX_STATE_CLOSED: */
		case HSI_LL_RX_STATE_IDLE:
			pr_debug("OPEN_CONN_OCTET in %s(), ch-> %d\n", __func__, ch);
			channel->rx_state = HSI_LL_RX_STATE_TO_ACK;
			hsi_protocol_send_command(HSI_LL_MSG_ACK, ch, param);

			channel->rx_count = param;
			channel->rx_state = HSI_LL_RX_STATE_RX;
			wake_up_interruptible(&ipc_read_wait);
			break;

		case HSI_LL_RX_STATE_BLOCKED:
			/* TODO */
			break;

		default:
			pr_err("OPEN_CONN_OCTET in invalid state, Current State -> %d\n", channel->rx_state);
			pr_info("Sending NAK to channel-> %d\n", ch);
			hsi_protocol_send_command(HSI_LL_MSG_NAK, ch, param);
		}
		break;

	default:
		pr_err("Invalid Command encountered in rx_state()\n");
	}

}
