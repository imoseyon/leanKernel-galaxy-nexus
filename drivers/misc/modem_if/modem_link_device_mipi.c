/* /linux/drivers/new_modem_if/link_dev_mipi.c
 *
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>

#include <linux/hsi_driver_if.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_link_device_mipi.h"


static int mipi_hsi_attach_io_dev(struct link_device *ld,
			struct io_device *iod)
{
	struct mipi_link_device *mipi_ld = to_mipi_link_device(ld);

	iod->link = ld;

	/* list up io devices */
	list_add_tail(&iod->list, &mipi_ld->list_of_io_devices);
	return 0;
}

static int mipi_hsi_init_communication(struct link_device *ld,
			struct io_device *iod)
{
	struct mipi_link_device *mipi_ld = to_mipi_link_device(ld);

	switch (iod->format) {
	case IPC_FMT:
		return hsi_init_handshake(mipi_ld, HSI_INIT_MODE_NORMAL);

	case IPC_BOOT:
		return hsi_init_handshake(mipi_ld,
					HSI_INIT_MODE_FLASHLESS_BOOT);

	case IPC_RFS:
	case IPC_RAW:
	default:
		return 0;
	}
}

static int mipi_hsi_send(struct link_device *ld, struct io_device *iod,
			struct sk_buff *skb)
{
	int ret;
	u32 mipi_boot_len = 0;
	struct mipi_link_device *mipi_ld = to_mipi_link_device(ld);

	struct sk_buff_head *txq;

	switch (iod->format) {
	case IPC_RAW:
		txq = &ld->sk_raw_tx_q;
		break;

	case IPC_BOOT: /* sync operation */
		ret = if_hsi_write(&mipi_ld->hsi_channles[
					HSI_FLASHLESS_CHANNEL],
					(u32 *)skb->data, skb->len);
		if (ret < 0) {
			pr_err("[MIPI-HSI] write fail : %d\n", ret);
			dev_kfree_skb_any(skb);
			return ret;
		} else
			pr_debug("[MIPI-HSI] write Done\n");
		dev_kfree_skb_any(skb);

		ret = if_hsi_read(&mipi_ld->hsi_channles[
					HSI_FLASHLESS_CHANNEL],
					(const char *)&mipi_boot_len, 4);
		if (ret < 0) {
			pr_err("[MIPI-HSI] read len fail : %d\n", ret);
			return ret;
		} else
			pr_debug("[MIPI-HSI] read len Done : %d\n",
						mipi_boot_len);

		ret = if_hsi_read(&mipi_ld->hsi_channles[
					HSI_FLASHLESS_CHANNEL],
					(const char *)mipi_ld->hsi_channles[
					HSI_FLASHLESS_CHANNEL].rx_data,
					mipi_boot_len);
		if (ret < 0) {
			pr_err("[MIPI-HSI] read len fail : %d\n", ret);
			return ret;
		} else
			pr_debug("[MIPI-HSI] read len Done\n");

		ret = iod->recv(iod, (char *)mipi_ld->hsi_channles[
			HSI_FLASHLESS_CHANNEL].rx_data, mipi_ld->hsi_channles[
					HSI_FLASHLESS_CHANNEL].rx_count);
		if (ret < 0) {
			pr_err("[MIPI-HSI] iod recv fail : %d\n", ret);
			return ret;
		} else
			pr_debug("[MIPI-HSI] iod recv Done : %d\n", ret);

		return mipi_boot_len;

	case IPC_FMT:
	case IPC_RFS:
	default:
		txq = &ld->sk_fmt_tx_q;
		break;
	}

	/* save io device into cb area */
	*((struct io_device **)skb->cb) = iod;
	/* en queue skb data */
	skb_queue_tail(txq, skb);

	if (!work_pending(&ld->tx_work))
		queue_work(ld->tx_wq, &ld->tx_work);

	return skb->len;
}

static void mipi_hsi_tx_work(struct work_struct *work)
{
	int ret;
	struct link_device *ld = container_of(work, struct link_device,
				tx_work);
	struct mipi_link_device *mipi_ld = to_mipi_link_device(ld);
	struct io_device *iod;
	struct sk_buff *fmt_skb;
	struct sk_buff *raw_skb;
	int send_channel = 0;

	while (ld->sk_fmt_tx_q.qlen || ld->sk_raw_tx_q.qlen) {
		pr_debug("[MIPI-HSI] fmt qlen : %d, raw qlen:%d\n",
				ld->sk_fmt_tx_q.qlen, ld->sk_raw_tx_q.qlen);

		fmt_skb = skb_dequeue(&ld->sk_fmt_tx_q);
		if (fmt_skb) {
			iod = *((struct io_device **)fmt_skb->cb);

			pr_debug("[MIPI-HSI] dequeue. fmt qlen : %d\n",
						ld->sk_fmt_tx_q.qlen);

			if (ld->com_state != COM_ONLINE) {
				pr_info("[MIPI-HSI] CP not ready\n");
				skb_queue_head(&ld->sk_fmt_tx_q, fmt_skb);
				return;
			}

			switch (iod->format) {
			case IPC_FMT:
				send_channel = HSI_FMT_CHANNEL;
				break;

			case IPC_RFS:
				send_channel = HSI_RFS_CHANNEL;
				break;

			case IPC_BOOT:
				send_channel = HSI_FLASHLESS_CHANNEL;
				break;
			default:
				break;
			}
			ret = if_hsi_protocol_send(mipi_ld, send_channel,
					(u32 *)fmt_skb->data, fmt_skb->len);
			if (ret < 0) {
				/* TODO: Re Enqueue */
				pr_err("[MIPI-HSI] write fail : %d\n", ret);
			}  else
				pr_debug("[MIPI-HSI] write Done\n");

			dev_kfree_skb_any(fmt_skb);
		}

		raw_skb = skb_dequeue(&ld->sk_raw_tx_q);
		if (raw_skb) {
			if (ld->com_state != COM_ONLINE) {
				pr_info("[MIPI-HSI] RAW CP not ready\n");
				skb_queue_head(&ld->sk_raw_tx_q, raw_skb);
				return;
			}

			pr_debug("[MIPI-HSI] dequeue. raw qlen:%d\n",
						ld->sk_raw_tx_q.qlen);

			ret = if_hsi_protocol_send(mipi_ld, HSI_RAW_CHANNEL,
					(u32 *)raw_skb->data, raw_skb->len);
			if (ret < 0) {
				/* TODO: Re Enqueue */
				pr_err("[MIPI-HSI] write fail : %d\n", ret);
			}  else
				pr_debug("[MIPI-HSI] write Done\n");

			dev_kfree_skb_any(raw_skb);
		}
	}
}

static int __devinit if_hsi_probe(struct hsi_device *dev);
static struct hsi_device_driver if_hsi_driver = {
	.ctrl_mask = ANY_HSI_CONTROLLER,
	.probe = if_hsi_probe,
	.driver = {
		.name = "if_hsi_driver"
	},
};

static int if_hsi_set_wakeline(struct if_hsi_channel *channel,
			unsigned int state)
{
	int ret;

	ret = hsi_ioctl(channel->dev, state ?
		HSI_IOCTL_ACWAKE_UP : HSI_IOCTL_ACWAKE_DOWN, NULL);
	if (ret) {
		pr_debug("[MIPI-HSI] ACWAKE(%d) setting fail : %d\n", state,
					ret);
		return ret;
	}

	pr_info("[MIPI-HSI] ACWAKE_%d(%d)\n", channel->channel_id, state);
	return 0;
}

static void if_hsi_acwake_down_func(unsigned long data)
{
	int i;
	struct if_hsi_channel *channel;
	struct mipi_link_device *mipi_ld = (struct mipi_link_device *)data;

	pr_info("[MIPI-HSI] %s\n", __func__);

	for (i = 1; i < HSI_NUM_OF_USE_CHANNELS; i++) {
		channel = &mipi_ld->hsi_channles[i];

		if (channel->send_step == STEP_IDLE) {
			if_hsi_set_wakeline(channel, 0);
		} else {
			mod_timer(&mipi_ld->hsi_acwake_down_timer, jiffies +
					HSI_ACWAKE_DOWN_TIMEOUT);
			pr_info("[MIPI-HSI] mod_timer done(%d)\n",
					HSI_ACWAKE_DOWN_TIMEOUT);
			return;
		}
	}
}

static int if_hsi_open_channel(struct if_hsi_channel *channel)
{
	int ret;

	if (channel->opened) {
		pr_debug("[MIPI-HSI] channel %d is already opened\n",
					channel->channel_id);
		return 0;
	}

	ret = hsi_open(channel->dev);
	if (ret) {
		pr_err("[MIPI-HSI] hsi_open fail : %d\n", ret);
		return ret;
	}
	channel->opened = 1;

	channel->send_step = STEP_IDLE;
	channel->recv_step = STEP_IDLE;

	pr_info("[MIPI-HSI] hsi_open Done : %d\n", channel->channel_id);
	return 0;
}

static int if_hsi_close_channel(struct if_hsi_channel *channel)
{
	unsigned long int flags;

	if (!channel->opened) {
		pr_debug("[MIPI-HSI] channel %d is already closed\n",
					channel->channel_id);
		return 0;
	}

	if_hsi_set_wakeline(channel, 0);
	hsi_write_cancel(channel->dev);
	hsi_read_cancel(channel->dev);

	spin_lock_irqsave(&channel->tx_state_lock, flags);
	channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
	spin_unlock_irqrestore(&channel->tx_state_lock, flags);
	spin_lock_irqsave(&channel->rx_state_lock, flags);
	channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
	spin_unlock_irqrestore(&channel->rx_state_lock, flags);

	hsi_ioctl(channel->dev, HSI_IOCTL_SW_RESET, NULL);

	hsi_close(channel->dev);
	channel->opened = 0;

	channel->send_step = STEP_CLOSED;
	channel->recv_step = STEP_CLOSED;

	pr_info("[MIPI-HSI] hsi_close Done : %d\n", channel->channel_id);
	return 0;
}

static void mipi_hsi_start_work(struct work_struct *work)
{
	int ret;
	u32 start_cmd = 0xC2;
	struct mipi_link_device *mipi_ld =
			container_of(work, struct mipi_link_device,
						start_work.work);

	ret = if_hsi_protocol_send(mipi_ld, HSI_CMD_CHANNEL, &start_cmd, 1);
	if (ret < 0) {
		/* TODO: Re Enqueue */
		pr_err("[MIPI-HSI] First write fail : %d\n", ret);
	}  else {
		pr_info("[MIPI-HSI] First write Done : %d\n", ret);
		mipi_ld->ld.com_state = COM_ONLINE;
	}
}

static int hsi_init_handshake(struct mipi_link_device *mipi_ld, int mode)
{
	int ret;
	int i;
	struct hst_ctx tx_config;
	struct hsr_ctx rx_config;

	switch (mode) {
	case HSI_INIT_MODE_NORMAL:
		if (timer_pending(&mipi_ld->hsi_acwake_down_timer))
			del_timer(&mipi_ld->hsi_acwake_down_timer);

		for (i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++) {
			if_hsi_close_channel(&mipi_ld->hsi_channles[i]);
			ret = if_hsi_open_channel(&mipi_ld->hsi_channles[i]);
			if (ret)
				return ret;

			hsi_ioctl(mipi_ld->hsi_channles[i].dev,
						HSI_IOCTL_GET_TX, &tx_config);
			tx_config.mode = 2;
			tx_config.divisor = 0; /* Speed : 96MHz */
			tx_config.channels = HSI_MAX_CHANNELS;
			hsi_ioctl(mipi_ld->hsi_channles[i].dev,
						HSI_IOCTL_SET_TX, &tx_config);

			hsi_ioctl(mipi_ld->hsi_channles[i].dev,
						HSI_IOCTL_GET_RX, &rx_config);
			rx_config.mode = 2;
			rx_config.divisor = 0; /* Speed : 96MHz */
			rx_config.channels = HSI_MAX_CHANNELS;
			hsi_ioctl(mipi_ld->hsi_channles[i].dev,
						HSI_IOCTL_SET_RX, &rx_config);
			pr_debug("[MIPI-HSI] Set TX/RX MIPI-HSI\n");

			hsi_ioctl(mipi_ld->hsi_channles[i].dev,
				HSI_IOCTL_SET_ACREADY_NORMAL, NULL);
			pr_debug("[MIPI-HSI] ACREADY_NORMAL\n");
		}

		mipi_ld->ld.com_state = COM_HANDSHAKE;

		ret = hsi_read(mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].dev,
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].rx_data,
					1);
		if (ret)
			pr_err("[MIPI-HSI] hsi_read fail : %d\n", ret);

		schedule_delayed_work(&mipi_ld->start_work, 3 * HZ);

		pr_info("[MIPI-HSI] hsi_init_handshake Done : MODE_NORMAL\n");
		return 0;

	case HSI_INIT_MODE_FLASHLESS_BOOT:
	case HSI_INIT_MODE_CP_RAMDUMP:
		mipi_ld->ld.com_state = COM_BOOT;

		for (i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++)
			if_hsi_close_channel(&mipi_ld->hsi_channles[i]);
		ret = if_hsi_open_channel(
			&mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL]);
		if (ret)
			return ret;

		hsi_ioctl(mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL].dev,
					HSI_IOCTL_GET_TX, &tx_config);
		tx_config.mode = 2;
		tx_config.divisor = 0; /* Speed : 96MHz */
		tx_config.channels = 1;
		hsi_ioctl(mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL].dev,
					HSI_IOCTL_SET_TX, &tx_config);

		hsi_ioctl(mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL].dev,
					HSI_IOCTL_GET_RX, &rx_config);
		rx_config.mode = 2;
		rx_config.divisor = 0; /* Speed : 96MHz */
		rx_config.channels = 1;
		hsi_ioctl(mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL].dev,
					HSI_IOCTL_SET_RX, &rx_config);
		pr_debug("[MIPI-HSI] Set TX/RX MIPI-HSI\n");

		if_hsi_set_wakeline(
			&mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL], 1);

		hsi_ioctl(mipi_ld->hsi_channles[HSI_FLASHLESS_CHANNEL].dev,
			HSI_IOCTL_SET_ACREADY_NORMAL, NULL);
		pr_debug("[MIPI-HSI] ACREADY_NORMAL\n");

		pr_info("[MIPI-HSI] hsi_init_handshake Done : FLASHLESS_BOOT\n");
		return 0;

	default:
		return -EINVAL;
	}
}

static u32 if_hsi_create_cmd(u32 cmd_type, int ch, void *arg)
{
	u32 cmd = 0;
	unsigned int size = 0;

	switch (cmd_type) {
	case HSI_LL_MSG_BREAK:
		return 0;

	case HSI_LL_MSG_CONN_CLOSED:
		cmd =  ((HSI_LL_MSG_CONN_CLOSED & 0x0000000F) << 28)
					|((ch & 0x000000FF) << 24);
		return cmd;

	case HSI_LL_MSG_ACK:
		size = *(unsigned int *)arg;

		cmd = ((HSI_LL_MSG_ACK & 0x0000000F) << 28)
			|((ch & 0x000000FF) << 24) | ((size & 0x00FFFFFF));
		return cmd;

	case HSI_LL_MSG_NAK:
		cmd = ((HSI_LL_MSG_NAK & 0x0000000F) << 28)
					|((ch & 0x000000FF) << 24);
		return cmd;

	case HSI_LL_MSG_OPEN_CONN_OCTET:
		size = *(unsigned int *)arg;

		cmd = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F)
				<< 28) | ((ch & 0x000000FF) << 24)
					| ((size & 0x00FFFFFF));
		return cmd;

	case HSI_LL_MSG_OPEN_CONN:
	case HSI_LL_MSG_CONF_RATE:
	case HSI_LL_MSG_CANCEL_CONN:
	case HSI_LL_MSG_CONN_READY:
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		pr_err("[MIPI-HSI] ERROR... CMD Not supported : %08x\n",
					cmd_type);
		return -EINVAL;
	}
}

static void if_hsi_cmd_work(struct work_struct *work)
{
	int ret;
	struct mipi_link_device *mipi_ld =
			container_of(work, struct mipi_link_device, cmd_work);
	struct if_hsi_channel *channel =
			&mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL];
	struct if_hsi_command *hsi_cmd;

	pr_debug("[MIPI-HSI] cmd_work\n");

	if (!list_empty(&mipi_ld->list_of_hsi_cmd))
		if_hsi_set_wakeline(channel, 1);
	else
		return;

	do {
		hsi_cmd = list_entry(mipi_ld->list_of_hsi_cmd.next,
					struct if_hsi_command, list);
		pr_debug("[MIPI-HSI] take command : %08x\n", hsi_cmd->command);

		ret = if_hsi_write(channel, &hsi_cmd->command, 4);
		if (ret < 0) {
			pr_err("[MIPI-HSI] write command fail : %d\n", ret);
			if_hsi_set_wakeline(channel, 0);
			return;
		}
		pr_info("[MIPI-HSI] SEND CMD : %08x\n", hsi_cmd->command);

		list_del(&hsi_cmd->list);
		kfree(hsi_cmd);
	} while (!list_empty(&mipi_ld->list_of_hsi_cmd));

	if_hsi_set_wakeline(channel, 0);
}

static int if_hsi_send_command(struct mipi_link_device *mipi_ld,
			u32 cmd_type, int ch, u32 param)
{
	struct if_hsi_command *hsi_cmd;

	hsi_cmd = kmalloc(sizeof(struct if_hsi_command), GFP_ATOMIC);
	if (!hsi_cmd) {
		pr_err("[MIPI-HSI] hsi_cmd kmalloc fail\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&hsi_cmd->list);

	hsi_cmd->command = if_hsi_create_cmd(cmd_type, ch, &param);
	pr_debug("[MIPI-HSI] made command : %08x\n", hsi_cmd->command);

	list_add_tail(&hsi_cmd->list, &mipi_ld->list_of_hsi_cmd);

	pr_debug("[MIPI-HSI] queue_work : cmd_work\n");
	if (!work_pending(&mipi_ld->cmd_work))
		queue_work(mipi_ld->mipi_wq, &mipi_ld->cmd_work);

	return 0;
}

static int if_hsi_decode_cmd(u32 *cmd_data, u32 *cmd, u32 *ch,
			u32 *param)
{
	u32 data = *cmd_data;
	u8 lrc_cal, lrc_act;
	u8 val1, val2, val3;

	*cmd = ((data & 0xF0000000) >> 28);
	switch (*cmd) {
	case HSI_LL_MSG_BREAK:
		pr_err("[MIPI-HSI] Command MSG_BREAK Received\n");
		return -1;

	case HSI_LL_MSG_OPEN_CONN:
		*ch = ((data & 0x0F000000) >> 24);
		*param = ((data & 0x00FFFF00) >> 8);
		val1 = ((data & 0xFF000000) >> 24);
		val2 = ((data & 0x00FF0000) >> 16);
		val3 = ((data & 0x0000FF00) >>  8);
		lrc_act = (data & 0x000000FF);
		lrc_cal = val1 ^ val2 ^ val3;

		if (lrc_cal != lrc_act) {
			pr_err("[MIPI-HSI] CAL is broken\n");
			return -1;
		}
		return 0;

	case HSI_LL_MSG_CONN_READY:
	case HSI_LL_MSG_CONN_CLOSED:
	case HSI_LL_MSG_CANCEL_CONN:
	case HSI_LL_MSG_NAK:
		*ch = ((data & 0x0F000000) >> 24);
		return 0;

	case HSI_LL_MSG_ACK:
		*ch = ((data & 0x0F000000) >> 24);
		*param = (data & 0x00FFFFFF);
		return 0;

	case HSI_LL_MSG_CONF_RATE:
		*ch = ((data & 0x0F000000) >> 24);
		*param = ((data & 0x0F000000) >> 24);
		return 0;

	case HSI_LL_MSG_OPEN_CONN_OCTET:
		*ch = ((data & 0x0F000000) >> 24);
		*param = (data & 0x00FFFFFF);
		return 0;

	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		pr_err("[MIPI-HSI] Invalid command received : %08x\n", *cmd);
		*cmd = HSI_LL_MSG_INVALID;
		*ch  = HSI_LL_INVALID_CHANNEL;
		return -1;
	}
	return 0;
}

static int if_hsi_rx_cmd_handle(struct mipi_link_device *mipi_ld, u32 cmd,
			u32 ch, u32 param)
{
	int ret;
	struct if_hsi_channel *channel = &mipi_ld->hsi_channles[ch];

	pr_debug("[MIPI-HSI] if_hsi_rx_cmd_handle cmd=0x%x, ch=%d, param=%d\n",
				cmd, ch, param);

	switch (cmd) {
	case HSI_LL_MSG_OPEN_CONN_OCTET:
		switch (channel->recv_step) {
		case STEP_IDLE:
			channel->recv_step = STEP_TO_ACK;
			ret = if_hsi_send_command(mipi_ld, HSI_LL_MSG_ACK, ch,
						param);
			if (ret) {
				pr_err("[MIPI-HSI] if_hsi_send_command fail : %d\n",
							ret);
				return ret;
			}

			channel->packet_size = param;
			channel->recv_step = STEP_RX;
			if (param % 4)
				param += (4 - (param % 4));
			channel->rx_count = param;
			ret = hsi_read(channel->dev, channel->rx_data,
						channel->rx_count / 4);
			if (ret) {
				pr_err("[MIPI-HSI] hsi_read fail : %d\n", ret);
				return ret;
			}
			return 0;

		default:
			ret = if_hsi_send_command(mipi_ld, HSI_LL_MSG_NAK, ch,
						param);
			if (ret) {
				pr_err("[MIPI-HSI] if_hsi_send_command fail : %d\n",
							ret);
				return ret;
			}
			return 0;
		}

	case HSI_LL_MSG_ACK:
	case HSI_LL_MSG_NAK:
		switch (channel->send_step) {
		case STEP_WAIT_FOR_ACK:
		case STEP_SEND_OPEN_CONN:
			if (cmd == HSI_LL_MSG_ACK) {
				channel->send_step = STEP_TX;
				channel->got_nack = 0;
				pr_debug("[MIPI-HSI] got ack\n");
			} else {
				channel->send_step = STEP_WAIT_FOR_ACK;
				channel->got_nack = 1;
				pr_debug("[MIPI-HSI] got nack\n");
			}

			up(&channel->ack_done_sem);
			return 0;

		default:
			pr_err("[MIPI-HSI] wrong state : %08x\n", cmd);
			return -1;
		}

	case HSI_LL_MSG_CONN_CLOSED:
		switch (channel->send_step) {
		case STEP_TX:
		case STEP_WAIT_FOR_CONN_CLOSED:
			pr_debug("[MIPI-HSI] got close\n");

			channel->send_step = STEP_IDLE;
			up(&channel->close_conn_done_sem);
			return 0;

		default:
			pr_err("[MIPI-HSI] wrong state : %08x\n", cmd);
			return -1;
		}

	case HSI_LL_MSG_OPEN_CONN:
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_CANCEL_CONN:
	case HSI_LL_MSG_CONF_RATE:
	default:
		pr_err("[MIPI-HSI] ERROR... CMD Not supported : %08x\n", cmd);
		return -EINVAL;
	}
}

static int if_hsi_protocol_send(struct mipi_link_device *mipi_ld, int ch,
			u32 *data, unsigned int len)
{
	int ret;
	int retry_count = 0;
	struct if_hsi_channel *channel = &mipi_ld->hsi_channles[ch];

	if (channel->send_step != STEP_IDLE) {
		pr_err("[MIPI-HSI] send step is not IDLE : %d\n",
					channel->send_step);
		return -EBUSY;
	}
	channel->send_step = STEP_SEND_OPEN_CONN;

	if_hsi_set_wakeline(channel, 1);
	mod_timer(&mipi_ld->hsi_acwake_down_timer, jiffies +
					HSI_ACWAKE_DOWN_TIMEOUT);
	pr_info("[MIPI-HSI] mod_timer done(%d)\n",
				HSI_ACWAKE_DOWN_TIMEOUT);

retry_send:

	ret = if_hsi_send_command(mipi_ld, HSI_LL_MSG_OPEN_CONN_OCTET, ch,
				len);
	if (ret) {
		pr_err("[MIPI-HSI] if_hsi_send_command fail : %d\n", ret);
		channel->send_step = STEP_IDLE;
		return -1;
	}

	channel->send_step = STEP_WAIT_FOR_ACK;

	if (down_timeout(&channel->ack_done_sem, HSI_ACK_DONE_TIMEOUT) < 0) {
		pr_err("[MIPI-HSI] ch=%d, ack_done timeout\n",
					channel->channel_id);

		channel->send_step = STEP_IDLE;
		return -ETIMEDOUT;
	}
	pr_debug("[MIPI-HSI] ch=%d, got ack_done=%d\n", channel->channel_id,
				channel->got_nack);

	if (channel->got_nack && (retry_count < 10)) {
		pr_info("[MIPI-HSI] ch=%d, got nack=%d retry=%d\n",
				channel->channel_id, channel->got_nack,
				retry_count);
		retry_count++;
		msleep(1);
		goto retry_send;
	}
	retry_count = 0;

	channel->send_step = STEP_TX;

	ret = if_hsi_write(channel, data, len);
	if (ret < 0) {
		pr_err("[MIPI-HSI] if_hsi_write fail : %d\n", ret);
		channel->send_step = STEP_IDLE;
		return ret;
	}
	pr_info("[MIPI-HSI] SEND DATA : %08x(%d)\n", *data, len);

	pr_debug("%08x %08x %08x %08x %08x %08x %08x %08x\n",
		*channel->tx_data, *(channel->tx_data + 1),
		*(channel->tx_data + 2), *(channel->tx_data + 3),
		*(channel->tx_data + 4), *(channel->tx_data + 5),
		*(channel->tx_data + 6), *(channel->tx_data + 7));

	channel->send_step = STEP_WAIT_FOR_CONN_CLOSED;
	if (down_timeout(&channel->close_conn_done_sem,
				HSI_CLOSE_CONN_DONE_TIMEOUT) < 0) {
		pr_err("[MIPI-HSI] ch=%d, close conn timeout\n",
					channel->channel_id);
		channel->send_step = STEP_IDLE;
		return -ETIMEDOUT;
	}
	pr_debug("[MIPI-HSI] ch=%d, got close_conn_done\n",
				channel->channel_id);

	channel->send_step = STEP_IDLE;

	pr_debug("[MIPI-HSI] write protocol Done : %d\n", channel->tx_count);
	return channel->tx_count;
}

static int if_hsi_write(struct if_hsi_channel *channel, u32 *data,
			unsigned int size)
{
	int ret;
	unsigned long int flags;

	spin_lock_irqsave(&channel->tx_state_lock, flags);
	if (channel->tx_state & HSI_CHANNEL_TX_STATE_WRITING) {
		spin_unlock_irqrestore(&channel->tx_state_lock, flags);
		return -EBUSY;
	}
	channel->tx_state |= HSI_CHANNEL_TX_STATE_WRITING;
	spin_unlock_irqrestore(&channel->tx_state_lock, flags);

	channel->tx_data = data;
	if (size % 4)
		size += (4 - (size % 4));
	channel->tx_count = size;

	pr_debug("[MIPI-HSI] submit write data : 0x%x(%d)\n",
				*(u32 *)channel->tx_data, channel->tx_count);
	ret = hsi_write(channel->dev, channel->tx_data, channel->tx_count / 4);
	if (ret) {
		pr_err("[MIPI-HSI] ch=%d, hsi_write fail : %d\n",
					channel->channel_id, ret);

		spin_lock_irqsave(&channel->tx_state_lock, flags);
		channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
		spin_unlock_irqrestore(&channel->tx_state_lock, flags);

		return ret;
	}

	if (down_timeout(&channel->write_done_sem,
				HSI_WRITE_DONE_TIMEOUT) < 0) {
		pr_err("[MIPI-HSI] ch=%d, hsi_write_done timeout : %d\n",
					channel->channel_id, size);

		pr_err("[MIPI-HSI] data : %08x %08x %08x %08x %08x ...\n",
			*channel->tx_data, *(channel->tx_data + 1),
			*(channel->tx_data + 2), *(channel->tx_data + 3),
			*(channel->tx_data + 4));

		hsi_write_cancel(channel->dev);

		spin_lock_irqsave(&channel->tx_state_lock, flags);
		channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
		spin_unlock_irqrestore(&channel->tx_state_lock, flags);

		return -ETIMEDOUT;
	}

	if (channel->tx_count != size)
		pr_err("[MIPI-HSI] ch:%d,write_done fail,write_size:%d,origin_size:%d\n",
				channel->channel_id, channel->tx_count, size);

	pr_debug("[MIPI-HSI] len:%d, id:%d, data : %08x %08x %08x %08x %08x ...\n",
		channel->tx_count, channel->channel_id, *channel->tx_data,
		*(channel->tx_data + 1), *(channel->tx_data + 2),
		*(channel->tx_data + 3), *(channel->tx_data + 4));

	return channel->tx_count;
}

static void if_hsi_write_done(struct hsi_device *dev, unsigned int size)
{
	unsigned long int flags;
	struct mipi_link_device *mipi_ld =
			(struct mipi_link_device *)if_hsi_driver.priv_data;
	struct if_hsi_channel *channel = &mipi_ld->hsi_channles[dev->n_ch];

	pr_debug("[MIPI-HSI] got write data : 0x%x(%d)\n",
				*(u32 *)channel->tx_data, size);

	spin_lock_irqsave(&channel->tx_state_lock, flags);
	channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
	spin_unlock_irqrestore(&channel->tx_state_lock, flags);

	pr_debug("%08x %08x %08x %08x %08x %08x %08x %08x\n",
			*channel->tx_data, *(channel->tx_data + 1),
			*(channel->tx_data + 2), *(channel->tx_data + 3),
			*(channel->tx_data + 4), *(channel->tx_data + 5),
			*(channel->tx_data + 6), *(channel->tx_data + 7));

	channel->tx_count = 4 * size;
	up(&channel->write_done_sem);
}

static int if_hsi_read(struct if_hsi_channel *channel, const char *data,
			unsigned int size)
{
	int ret;
	unsigned long int flags;

	spin_lock_irqsave(&channel->rx_state_lock, flags);
	if (channel->rx_state & HSI_CHANNEL_RX_STATE_READING) {
		spin_unlock_irqrestore(&channel->rx_state_lock, flags);
		return -EBUSY;
	}
	channel->rx_state |= HSI_CHANNEL_RX_STATE_READING;
	spin_unlock_irqrestore(&channel->rx_state_lock, flags);

	if (size % 4)
		size += (4 - (size % 4));
	channel->rx_count = size;

	ret = hsi_read(channel->dev, (u32 *)data, channel->rx_count / 4);
	if (ret) {
		pr_err("[MIPI-HSI] ch=%d, hsi_read fail : %d\n",
					channel->channel_id, ret);

		spin_lock_irqsave(&channel->rx_state_lock, flags);
		channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
		spin_unlock_irqrestore(&channel->rx_state_lock, flags);

		return ret;
	}

	pr_debug("[MIPI-HSI] submit read data : 0x%x(%d)\n",
				*(u32 *)channel->rx_data, channel->rx_count);

	if (down_timeout(&channel->read_done_sem,
				HSI_READ_DONE_TIMEOUT) < 0) {
		pr_err("[MIPI-HSI] ch=%d, hsi_read_done timeout : %d\n",
					channel->channel_id, channel->rx_count);

		hsi_read_cancel(channel->dev);

		spin_lock_irqsave(&channel->rx_state_lock, flags);
		channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
		spin_unlock_irqrestore(&channel->rx_state_lock, flags);

		return -ETIMEDOUT;
	}

	if (channel->rx_count != size)
		pr_err("[MIPI-HSI] ch:%d,read_done fail,read_size:%d,origin_size:%d\n",
				channel->channel_id, channel->rx_count, size);

	return channel->rx_count;
}

static void if_hsi_read_done(struct hsi_device *dev, unsigned int size)
{
	int ret;
	unsigned long int flags;
	u32 cmd = 0, ch = 0, param = 0;
	struct mipi_link_device *mipi_ld =
			(struct mipi_link_device *)if_hsi_driver.priv_data;
	struct if_hsi_channel *channel = &mipi_ld->hsi_channles[dev->n_ch];
	struct io_device *iod;
	enum dev_format format_type = 0;

	pr_debug("[MIPI-HSI] got read data : 0x%x(%d)\n",
				*(u32 *)channel->rx_data, size);

	spin_lock_irqsave(&channel->rx_state_lock, flags);
	channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
	spin_unlock_irqrestore(&channel->rx_state_lock, flags);

	channel->rx_count = 4 * size;

	switch (channel->channel_id) {
	case HSI_CONTROL_CHANNEL:
		switch (mipi_ld->ld.com_state) {
		case COM_HANDSHAKE:
		case COM_ONLINE:
			pr_info("[MIPI-HSI] RECV CMD : %08x\n",
						*channel->rx_data);

			if (channel->rx_count != 4) {
				pr_err("[MIPI-HSI] wrong command len : %d\n",
					channel->rx_count);
				return;
			}

			ret = if_hsi_decode_cmd(channel->rx_data, &cmd, &ch,
						&param);
			if (ret)
				pr_err("[MIPI-HSI] decode_cmd fail=%d, cmd=%x\n",
							ret, cmd);
			else {
				pr_debug("[MIPI-HSI] decode_cmd : %08x\n", cmd);
				ret = if_hsi_rx_cmd_handle(mipi_ld, cmd, ch,
							param);
				if (ret)
					pr_err("[MIPI-HSI] handle cmd cmd=%x\n",
								cmd);
			}

			ret = hsi_read(channel->dev, channel->rx_data, 1);
			if (ret)
				pr_err("[MIPI-HSI] hsi_read fail : %d\n", ret);

			return;

		case COM_BOOT:
			pr_debug("[MIPI-HSI] receive data : 0x%x(%d)\n",
					*channel->rx_data, channel->rx_count);

			up(&channel->read_done_sem);
			return;

		case COM_CRASH:
		case COM_NONE:
		default:
			pr_info("[MIPI-HSI] receive data in wrong state : 0x%x(%d)\n",
					*channel->rx_data, channel->rx_count);
			up(&channel->read_done_sem);
			return;
		}
		break;

	case HSI_FMT_CHANNEL:
		pr_debug("[MIPI-HSI] iodevice format : IPC_FMT\n");
		format_type = IPC_FMT;
		break;
	case HSI_RAW_CHANNEL:
		pr_debug("[MIPI-HSI] iodevice format : IPC_MULTI_RAW\n");
		format_type = IPC_MULTI_RAW;
		break;
	case HSI_RFS_CHANNEL:
		pr_debug("[MIPI-HSI] iodevice format : IPC_RFS\n");
		format_type = IPC_RFS;
		break;

	case HSI_CMD_CHANNEL:
		pr_info("[MIPI-HSI] receive command data : 0x%x\n",
					*channel->rx_data);
		return;

	default:
		up(&channel->read_done_sem);
		return;
	}

	list_for_each_entry(iod, &mipi_ld->list_of_io_devices, list) {
		pr_debug("[MIPI-HSI] iodevice format : %d\n", iod->format);

		if (iod->format == format_type) {
			pr_info("[MIPI-HSI] RECV DATA : %08x(%d)-%d\n",
				*channel->rx_data, channel->packet_size,
					iod->format);

			pr_debug("%08x %08x %08x %08x %08x %08x %08x %08x\n",
			*channel->rx_data, *(channel->rx_data + 1),
			*(channel->rx_data + 2), *(channel->rx_data + 3),
			*(channel->rx_data + 4), *(channel->rx_data + 5),
			*(channel->rx_data + 6), *(channel->rx_data + 7));

			ret = iod->recv(iod, (char *)channel->rx_data,
						channel->packet_size);
			if (ret < 0)
				pr_err("[MIPI-HSI] recv call fail : %d\n", ret);

			if (channel->recv_step == STEP_RX) {
				ch = channel->channel_id;
				param = 0;
				ret = if_hsi_send_command(mipi_ld,
					HSI_LL_MSG_CONN_CLOSED, ch, param);
				if (ret)
					pr_err("[MIPI-HSI] send_cmd fail=%d\n",
								ret);

				channel->recv_step = STEP_IDLE;
			}
			return;
		}
	}
}

static void if_hsi_port_event(struct hsi_device *dev, unsigned int event,
			void *arg)
{
	int acwake_level = 1;
	struct mipi_link_device *mipi_ld =
			(struct mipi_link_device *)if_hsi_driver.priv_data;

	switch (event) {
	case HSI_EVENT_BREAK_DETECTED:
		pr_err("[MIPI-HSI] HSI_EVENT_BREAK_DETECTED\n");
		return;

	case HSI_EVENT_HSR_DATAAVAILABLE:
		pr_err("[MIPI-HSI] HSI_EVENT_HSR_DATAAVAILABLE\n");
		return;

	case HSI_EVENT_CAWAKE_UP:
		if (dev->n_ch == HSI_CONTROL_CHANNEL)
			pr_info("[MIPI-HSI] CAWAKE_%d(1)\n", dev->n_ch);

		if ((dev->n_ch == HSI_CONTROL_CHANNEL) &&
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].opened) {
			hsi_ioctl(
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].dev,
			HSI_IOCTL_SET_ACREADY_NORMAL, NULL);

			pr_debug("[MIPI-HSI] ACREADY_NORMAL. Ch : %d\n",
					dev->n_ch);
		}

		return;

	case HSI_EVENT_CAWAKE_DOWN:
		if (dev->n_ch == HSI_CONTROL_CHANNEL)
			pr_info("[MIPI-HSI] CAWAKE_%d(0)\n", dev->n_ch);

		if ((dev->n_ch == HSI_CONTROL_CHANNEL) &&
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].opened) {
			hsi_ioctl(
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].dev,
			HSI_IOCTL_GET_ACWAKE, &acwake_level);

			pr_debug("[MIPI-HSI] GET_ACWAKE. Ch : %d, level : %d\n",
					dev->n_ch, acwake_level);
		}

		if ((dev->n_ch == HSI_CONTROL_CHANNEL) &&
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].opened &&
			!acwake_level) {
			hsi_ioctl(
			mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].dev,
			HSI_IOCTL_SET_ACREADY_SAFEMODE, NULL);

			pr_debug("[MIPI-HSI] ACREADY_SAFEMODE. Ch : %d\n",
					dev->n_ch);
		}

		return;

	case HSI_EVENT_ERROR:
		pr_err("[MIPI-HSI] HSI_EVENT_ERROR\n");
		return;

	default:
		pr_err("[MIPI-HSI] Unknown Event : %d\n", event);
		return;
	}
}

static int __devinit if_hsi_probe(struct hsi_device *dev)
{
	int port = 0;
	unsigned long *address;

	struct mipi_link_device *mipi_ld =
			(struct mipi_link_device *)if_hsi_driver.priv_data;

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (if_hsi_driver.ch_mask[port])
			break;
	}
	address = (unsigned long *)&if_hsi_driver.ch_mask[port];

	if (test_bit(dev->n_ch, address) && (dev->n_p == port)) {
		/* Register callback func */
		hsi_set_write_cb(dev, if_hsi_write_done);
		hsi_set_read_cb(dev, if_hsi_read_done);
		hsi_set_port_event_cb(dev, if_hsi_port_event);

		/* Init device data */
		mipi_ld->hsi_channles[dev->n_ch].dev = dev;
		mipi_ld->hsi_channles[dev->n_ch].tx_count = 0;
		mipi_ld->hsi_channles[dev->n_ch].rx_count = 0;
		mipi_ld->hsi_channles[dev->n_ch].tx_state = 0;
		mipi_ld->hsi_channles[dev->n_ch].rx_state = 0;
		mipi_ld->hsi_channles[dev->n_ch].packet_size = 0;
		mipi_ld->hsi_channles[dev->n_ch].send_step = STEP_UNDEF;
		mipi_ld->hsi_channles[dev->n_ch].recv_step = STEP_UNDEF;
		spin_lock_init(&mipi_ld->hsi_channles[dev->n_ch].tx_state_lock);
		spin_lock_init(&mipi_ld->hsi_channles[dev->n_ch].rx_state_lock);
		sema_init(&mipi_ld->hsi_channles[dev->n_ch].write_done_sem, 0);
		sema_init(&mipi_ld->hsi_channles[dev->n_ch].read_done_sem, 0);
		sema_init(&mipi_ld->hsi_channles[dev->n_ch].ack_done_sem, 0);
		sema_init(&mipi_ld->hsi_channles[dev->n_ch].close_conn_done_sem,
					0);
	}

	pr_info("[MIPI-HSI] if_hsi_probe() done. ch : %d\n", dev->n_ch);
	return 0;
}

static int if_hsi_init(struct link_device *ld)
{
	int ret;
	int i = 0;
	struct mipi_link_device *mipi_ld = to_mipi_link_device(ld);

	for (i = 0; i < HSI_MAX_PORTS; i++)
		if_hsi_driver.ch_mask[i] = 0;

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		mipi_ld->hsi_channles[i].dev = NULL;
		mipi_ld->hsi_channles[i].opened = 0;
		mipi_ld->hsi_channles[i].channel_id = i;
	}
	if_hsi_driver.ch_mask[0] = CHANNEL_MASK;

	/* TODO - need to get priv data (request to TI) */
	if_hsi_driver.priv_data = (void *)mipi_ld;
	ret = hsi_register_driver(&if_hsi_driver);
	if (ret) {
		pr_err("[MIPI-HSI] hsi_register_driver() fail : %d\n", ret);
		return ret;
	}

	mipi_ld->mipi_wq = create_singlethread_workqueue("mipi_cmd_wq");
	if (!mipi_ld->mipi_wq) {
		pr_err("[MIPI-HSI] fail to create work Q.\n");
		return -ENOMEM;
	}
	INIT_WORK(&mipi_ld->cmd_work, if_hsi_cmd_work);
	INIT_DELAYED_WORK(&mipi_ld->start_work, mipi_hsi_start_work);

	setup_timer(&mipi_ld->hsi_acwake_down_timer, if_hsi_acwake_down_func,
				(unsigned long)mipi_ld);
	pr_info("[MIPI-HSI] setup_timer done\n");

	/* TODO - allocate rx buff */
	mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].rx_data =
				kmalloc(64 * 1024, GFP_DMA | GFP_ATOMIC);
	if (!mipi_ld->hsi_channles[HSI_CONTROL_CHANNEL].rx_data) {
		pr_err("[MIPI-HSI] alloc HSI_CONTROL_CHANNEL rx_data fail\n");
		return -ENOMEM;
	}
	mipi_ld->hsi_channles[HSI_FMT_CHANNEL].rx_data =
				kmalloc(256 * 1024, GFP_DMA | GFP_ATOMIC);
	if (!mipi_ld->hsi_channles[HSI_FMT_CHANNEL].rx_data) {
		pr_err("[MIPI-HSI] alloc HSI_FMT_CHANNEL rx_data fail\n");
		return -ENOMEM;
	}
	mipi_ld->hsi_channles[HSI_RAW_CHANNEL].rx_data =
				kmalloc(256 * 1024, GFP_DMA | GFP_ATOMIC);
	if (!mipi_ld->hsi_channles[HSI_RAW_CHANNEL].rx_data) {
		pr_err("[MIPI-HSI] alloc HSI_RAW_CHANNEL rx_data fail\n");
		return -ENOMEM;
	}
	mipi_ld->hsi_channles[HSI_RFS_CHANNEL].rx_data =
				kmalloc(256 * 1024, GFP_DMA | GFP_ATOMIC);
	if (!mipi_ld->hsi_channles[HSI_RFS_CHANNEL].rx_data) {
		pr_err("[MIPI-HSI] alloc HSI_RFS_CHANNEL rx_data fail\n");
		return -ENOMEM;
	}
	mipi_ld->hsi_channles[HSI_CMD_CHANNEL].rx_data =
				kmalloc(256 * 1024, GFP_DMA | GFP_ATOMIC);
	if (!mipi_ld->hsi_channles[HSI_CMD_CHANNEL].rx_data) {
		pr_err("[MIPI-HSI] alloc HSI_CMD_CHANNEL rx_data fail\n");
		return -ENOMEM;
	}

	pr_info("[MIPI-HSI] if_hsi_init() done : %d\n", ret);
	return 0;
}

struct link_device *mipi_create_link_device(struct platform_device *pdev)
{
	int ret;
	struct mipi_link_device *mipi_ld;
	struct link_device *ld;

	/* for dpram int */
	/* struct modem_data *pdata = pdev->dev.platform_data; */

	mipi_ld = kzalloc(sizeof(struct mipi_link_device), GFP_KERNEL);
	if (!mipi_ld)
		return NULL;

	INIT_LIST_HEAD(&mipi_ld->list_of_io_devices);
	INIT_LIST_HEAD(&mipi_ld->list_of_hsi_cmd);
	skb_queue_head_init(&mipi_ld->ld.sk_fmt_tx_q);
	skb_queue_head_init(&mipi_ld->ld.sk_raw_tx_q);

	ld = &mipi_ld->ld;

	ld->name = "mipi_hsi";
	ld->attach = mipi_hsi_attach_io_dev;
	ld->init_comm = mipi_hsi_init_communication;
	ld->send = mipi_hsi_send;
	ld->com_state = COM_NONE;

	/* for dpram int */
	/* ld->irq = gpio_to_irq(pdata->gpio); s*/

	ld->tx_wq = create_singlethread_workqueue("mipi_tx_wq");
	if (!ld->tx_wq) {
		pr_err("[MIPI-HSI] fail to create work Q.\n");
		return NULL;
	}
	INIT_WORK(&ld->tx_work, mipi_hsi_tx_work);

	ret = if_hsi_init(ld);
	if (ret)
		return NULL;

	pr_info("[MODEM_IF] %s : create_io_device DONE\n", mipi_ld->ld.name);
	return ld;
}

