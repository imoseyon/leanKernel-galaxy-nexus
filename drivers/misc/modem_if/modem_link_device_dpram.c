/*
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/if_arp.h>
#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_link_device_dpram.h"

/* interrupt masks.*/
#define INT_MASK_VALID			0x0080
#define INT_MASK_CMD			0x0040
#define INT_MASK_REQ_ACK_F		0x0020
#define INT_MASK_REQ_ACK_R		0x0010
#define INT_MASK_RES_ACK_F		0x0008
#define INT_MASK_RES_ACK_R		0x0004
#define INT_MASK_SEND_F			0x0002
#define INT_MASK_SEND_R			0x0001
#define INT_VALID(x)			((x) & INT_MASK_VALID)
#define INT_CMD_VALID(x)		((x) & INT_MASK_CMD)
#define INT_NON_CMD(x)			(INT_MASK_VALID | (x))
#define INT_CMD(x)			(INT_MASK_VALID | INT_MASK_CMD | (x))

#define INT_CMD_MASK(x)			((x) & 0xF)
#define INT_CMD_INIT_START		0x1
#define INT_CMD_INIT_END		0x2
#define INT_CMD_REQ_ACTIVE		0x3
#define INT_CMD_RES_ACTIVE		0x4
#define INT_CMD_REQ_TIME_SYNC		0x5
#define INT_CMD_PHONE_START		0x8
#define INT_CMD_ERR_DISPLAY		0x9
#define INT_CMD_PHONE_DEEP_SLEEP	0xA
#define INT_CMD_NV_REBUILDING		0xB
#define INT_CMD_EMER_DOWN		0xC
#define INT_CMD_PIF_INIT_DONE		0xD
#define INT_CMD_SILENT_NV_REBUILDING	0xE
#define INT_CMD_NORMAL_POWER_OFF	0xF

/* special interrupt cmd indicating modem boot failure. */
#define INT_POWERSAFE_FAIL              0xDEAD

#define GOTA_CMD_VALID(x)		(((x) & 0xA000) == 0xA000)
#define GOTA_RESULT_FAIL		0x2
#define GOTA_RESULT_SUCCESS		0x1
#define GOTA_CMD_MASK(x)		(((x) >> 8) & 0xF)
#define GOTA_CMD_RECEIVE_READY		0x1
#define GOTA_CMD_DOWNLOAD_START_REQ	0x2
#define GOTA_CMD_DOWNLOAD_START_RESP	0x3
#define GOTA_CMD_IMAGE_SEND_REQ		0x4
#define GOTA_CMD_IMAGE_SEND_RESP	0x5
#define GOTA_CMD_SEND_DONE_REQ		0x6
#define GOTA_CMD_SEND_DONE_RESP		0x7
#define GOTA_CMD_STATUS_UPDATE		0x8
#define GOTA_CMD_UPDATE_DONE		0x9
#define GOTA_CMD_EFS_CLEAR_RESP		0xB
#define GOTA_CMD_ALARM_BOOT_OK		0xC
#define GOTA_CMD_ALARM_BOOT_FAIL	0xD

#define CMD_DL_START_REQ		0x9200
#define CMD_IMG_SEND_REQ		0x9400
#define CMD_DL_SEND_DONE_REQ		0x9600
#define CMD_UL_RECEIVE_RESP		0x9601
#define CMD_UL_RECEIVE_DONE_RESP	0x9801

#define START_INDEX			0x7F
#define END_INDEX			0x7E

#define DP_MAGIC_CODE			0xAA
#define DP_MAGIC_DMDL			0x4445444C
#define DP_MAGIC_UMDL			0x4445444D
#define DP_DPRAM_SIZE			0x4000
#define DP_DEFAULT_WRITE_LEN		8168
#define DP_DEFAULT_DUMP_LEN		16366
#define DP_DUMP_HEADER_SIZE		7

#define GOTA_TIMEOUT			(50 * HZ)
#define GOTA_SEND_TIMEOUT		(200 * HZ)
#define DUMP_TIMEOUT			(30 * HZ)
#define DUMP_START_TIMEOUT		(100 * HZ)

static int
dpram_download(struct dpram_link_device *dpld, const char *buf, int len);
static int
dpram_upload(struct dpram_link_device *dpld, struct dpram_firmware *uploaddata);

static inline int dpram_readh(void __iomem *p_dest)
{
	unsigned long dest = (unsigned long)p_dest;
	return ioread16(dest);
}

static inline void dpram_writew(u32 value,  void __iomem *p_dest)
{
	unsigned long dest = (unsigned long)p_dest;
	iowrite32(value, dest);
}

static inline void dpram_writeh(u16 value,  void __iomem *p_dest)
{
	unsigned long dest = (unsigned long)p_dest;
	iowrite16(value, dest);
}

static inline void dpram_writeb(u8 value,  void __iomem *p_dest)
{
	unsigned long dest = (unsigned long)p_dest;
	iowrite8(value, dest);
}


static void dpram_write_command(struct dpram_link_device *dpld, u16 cmd)
{
	dpram_writeh(cmd, &dpld->dpram->mbx_ap2cp);
}

static void dpram_clear_interrupt(struct dpram_link_device *dpld)
{
	dpram_writeh(0, &dpld->dpram->mbx_cp2ap);
}

static void dpram_drop_data(struct dpram_device *device, u16 head)
{
	dpram_writeh(head, &device->in->tail);
}

static void dpram_zero_circ(struct dpram_circ *circ)
{
	dpram_writeh(0, &circ->head);
	dpram_writeh(0, &circ->tail);
}

static void dpram_clear(struct dpram_link_device *dpld)
{
	dpram_zero_circ(&dpld->dpram->fmt_out);
	dpram_zero_circ(&dpld->dpram->raw_out);
	dpram_zero_circ(&dpld->dpram->fmt_in);
	dpram_zero_circ(&dpld->dpram->raw_in);
}

static bool dpram_circ_valid(int size, u16 head, u16 tail)
{
	if (head >= size) {
		pr_err("[DPRAM] head(%d) >= size(%d)\n", head, size);
		return false;
	}
	if (tail >= size) {
		pr_err("[DPRAM] tail(%d) >= size(%d)\n", tail, size);
		return false;
	}
	return true;
}

static int dpram_init_and_report(struct dpram_link_device *dpld)
{
	const u16 init_end = INT_CMD(INT_CMD_INIT_END);
	u16 magic;
	u16 enable;

	dpram_writeh(0, &dpld->dpram->enable);
	dpram_clear(dpld);
	dpram_writeh(DP_MAGIC_CODE, &dpld->dpram->magic);
	dpram_writeh(1, &dpld->dpram->enable);

	/* Send init end code to modem */
	dpram_write_command(dpld, init_end);

	magic = dpram_readh(&dpld->dpram->magic);
	if (magic != DP_MAGIC_CODE) {
		pr_err("[DPRAM]: %s: Failed to check magic\n", __func__);
		return -1;
	}

	enable = dpram_readh(&dpld->dpram->enable);
	if (!enable) {
		pr_err("[DPRAM]: %s: DPRAM enable failed\n", __func__);
		return -1;
	}

	return 0;
}

static struct io_device *dpram_find_iod(struct dpram_link_device *dpld, int id)
{
	struct io_device *iod;

	list_for_each_entry(iod, &dpld->list_of_io_devices, list) {
		if ((id == FMT_IDX && iod->format == IPC_FMT) ||
				(id == RAW_IDX && iod->format == IPC_MULTI_RAW))
			return iod;
	}

	return NULL;
}

static void cmd_req_active_handler(struct dpram_link_device *dpld)
{
	dpram_write_command(dpld, INT_CMD(INT_CMD_RES_ACTIVE));
}

static void cmd_error_display_handler(struct dpram_link_device *dpld)
{
	struct io_device *iod = dpram_find_iod(dpld, FMT_IDX);

	pr_info("[DPRAM] Received 0xc9 from modem (CP Crash)\n");
	pr_info("[DPRAM] %s\n", dpld->dpram->fmt_in_buff);

	if (iod && iod->modem_state_changed)
		iod->modem_state_changed(iod, STATE_CRASH_EXIT);
}

static void cmd_phone_start_handler(struct dpram_link_device *dpld)
{
	pr_debug("[DPRAM] Received 0xc8 from modem (Boot OK)\n");
	complete_all(&dpld->dpram_init_cmd);
	dpram_init_and_report(dpld);
}

static void command_handler(struct dpram_link_device *dpld, u16 cmd)
{
	pr_debug("[DPRAM] %s: %x\n", __func__, cmd);

	switch (INT_CMD_MASK(cmd)) {
	case INT_CMD_REQ_ACTIVE:
		cmd_req_active_handler(dpld);
		break;

	case INT_CMD_ERR_DISPLAY:
		cmd_error_display_handler(dpld);
		break;

	case INT_CMD_PHONE_START:
		cmd_phone_start_handler(dpld);
		break;

	case INT_CMD_NV_REBUILDING:
		pr_err("[MODEM_IF] NV_REBUILDING\n");
		break;

	case INT_CMD_PIF_INIT_DONE:
		complete_all(&dpld->modem_pif_init_done);
		break;

	case INT_CMD_SILENT_NV_REBUILDING:
		pr_err("[MODEM_IF] SILENT_NV_REBUILDING\n");
		break;

	case INT_CMD_NORMAL_POWER_OFF:
		/*ToDo:*/
		/*kernel_sec_set_cp_ack()*/;
		break;

	case INT_CMD_REQ_TIME_SYNC:
	case INT_CMD_PHONE_DEEP_SLEEP:
	case INT_CMD_EMER_DOWN:
		break;

	default:
		pr_err("Unknown command.. %x\n", cmd);
	}
}


static int dpram_process_modem_update(struct dpram_link_device *dpld,
					struct dpram_firmware *pfw)
{
	int ret = 0;
	char *buff = vmalloc(pfw->size);

	pr_debug("[GOTA] modem size =[%d]\n", pfw->size);

	if (!buff)
		return -ENOMEM;

	ret = copy_from_user(buff, pfw->firmware, pfw->size);
	if (ret < 0) {
		pr_err("[%s:%d] Copy from user failed\n", __func__, __LINE__);
		goto out;
	}

	ret = dpram_download(dpld, buff, pfw->size);
	if (ret < 0)
		pr_err("firmware write failed\n");

out:
	vfree(buff);
	return ret;
}


static int dpram_modem_update(struct link_device *ld, struct io_device *iod,
							unsigned long _arg)
{
	int ret;
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_firmware fw;

	pr_debug("[GOTA] dpram_modem_update\n");

	ret = copy_from_user(&fw, (void __user *)_arg, sizeof(fw));
	if (ret  < 0) {
		pr_err("copy from user failed!");
		return ret;
	}

	return dpram_process_modem_update(dpld, &fw);
}

static int dpram_dump_update(struct link_device *ld, struct io_device *iod,
							unsigned long _arg)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_firmware *fw = (struct dpram_firmware *)_arg ;

	pr_debug("[DPRAM] dpram_dump_update()\n");

	return dpram_upload(dpld, fw);
}

static int dpram_read(struct dpram_link_device *dpld,
		      struct dpram_device *device, int dev_idx)
{
	struct io_device *iod;
	int size;
	int tmp_size;
	u16 head, tail;
	char *buff;

	head = dpram_readh(&device->in->head);
	tail = dpram_readh(&device->in->tail);
	pr_debug("=====> %s,  head: %d, tail: %d\n", __func__, head, tail);

	if (head == tail) {
		pr_err("[DPRAM] %s: head == tail\n", __func__);
		goto err_dpram_read;
	}

	if (!dpram_circ_valid(device->in_buff_size, head, tail)) {
		pr_err("[DPRAM] %s: invalid circular buffer\n", __func__);
		dpram_zero_circ(device->in);
		goto err_dpram_read;
	}

	iod = dpram_find_iod(dpld, dev_idx);
	if (!iod) {
		pr_err("[DPRAM] iod == NULL\n");
		goto err_dpram_read;
	}

	/* Get data size in DPRAM*/
	size = (head > tail) ? (head - tail) :
		(device->in_buff_size - tail + head);

	/* ----- (tail) 7f 00 00 7e (head) ----- */
	if (head > tail) {
		buff = device->in_buff_addr + tail;
		if (iod->recv(iod, buff, size) < 0) {
			pr_err("[DPRAM] %s: recv error, dropping data\n",
								__func__);
			dpram_drop_data(device, head);
			goto err_dpram_read;
		}
	} else { /* 00 7e (head) ----------- (tail) 7f 00 */
		/* 1. tail -> buffer end.*/
		tmp_size = device->in_buff_size - tail;
		buff = device->in_buff_addr + tail;
		if (iod->recv(iod, buff, tmp_size) < 0) {
			pr_err("[DPRAM] %s: recv error, dropping data\n",
								__func__);
			dpram_drop_data(device, head);
			goto err_dpram_read;
		}

		/* 2. buffer start -> head.*/
		if (size > tmp_size) {
			buff = (char *)device->in_buff_addr;
			if (iod->recv(iod, buff, (size - tmp_size)) < 0) {
				pr_err("[DPRAM] %s: recv error, dropping data\n",
								__func__);
				dpram_drop_data(device, head);
				goto err_dpram_read;
			}
		}
	}

	/* new tail */
	tail = (u16)((tail + size) % device->in_buff_size);
	dpram_writeh(tail, &device->in->tail);

	return size;

err_dpram_read:
	return -EINVAL;
}

static void non_command_handler(struct dpram_link_device *dpld,
				u16 non_cmd)
{
	struct dpram_device *device = NULL;
	u16 head, tail;
	u16 magic, access;
	int ret = 0;

	pr_debug("[DPRAM] Entering non_command_handler(0x%04X)\n", non_cmd);

	magic = dpram_readh(&dpld->dpram->magic);
	access = dpram_readh(&dpld->dpram->enable);

	if (!access || magic != DP_MAGIC_CODE) {
		pr_err("fmr recevie error!!!!  access = 0x%x, magic =0x%x",
				access, magic);
		return;
	}

	/* Check formatted data region */
	device = &dpld->dev_map[FMT_IDX];
	head = dpram_readh(&device->in->head);
	tail = dpram_readh(&device->in->tail);

	if (!dpram_circ_valid(device->in_buff_size, head, tail)) {
		pr_err("[DPRAM] %s: invalid circular buffer\n", __func__);
		dpram_zero_circ(device->in);
		return;
	}

	if (head != tail) {
		if (non_cmd & INT_MASK_REQ_ACK_F)
			atomic_inc(&dpld->fmt_txq_req_ack_rcvd);

		ret = dpram_read(dpld, device, FMT_IDX);
		if (ret < 0)
			pr_err("%s, dpram_read failed\n", __func__);

		if (atomic_read(&dpld->fmt_txq_req_ack_rcvd) > 0) {
			dpram_write_command(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_F));
			atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_F) {
			dpram_write_command(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_F));
			atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);
		}
	}

	/* Check raw data region */
	device = &dpld->dev_map[RAW_IDX];
	head = dpram_readh(&device->in->head);
	tail = dpram_readh(&device->in->tail);

	if (!dpram_circ_valid(device->in_buff_size, head, tail)) {
		pr_err("[DPRAM] %s: invalid circular buffer\n", __func__);
		dpram_zero_circ(device->in);
		return;
	}

	if (head != tail) {
		if (non_cmd & INT_MASK_REQ_ACK_R)
			atomic_inc(&dpld->raw_txq_req_ack_rcvd);

		ret = dpram_read(dpld, device, RAW_IDX);
		if (ret < 0)
			pr_err("%s, dpram_read failed\n", __func__);

		if (atomic_read(&dpld->raw_txq_req_ack_rcvd) > 0) {
			dpram_write_command(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_R));
			atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_R) {
			dpram_write_command(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_R));
			atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
		}
	}
}

static void gota_cmd_handler(struct dpram_link_device *dpld, u16 cmd)
{
	if (cmd & GOTA_RESULT_FAIL) {
		pr_err("[GOTA] Command failed: %04x\n", cmd);
		return;
	}

	switch (GOTA_CMD_MASK(cmd)) {
	case GOTA_CMD_RECEIVE_READY:
		pr_debug("[GOTA] Send CP-->AP RECEIVE_READY\n");
		dpram_write_command(dpld, CMD_DL_START_REQ);
		break;

	case GOTA_CMD_DOWNLOAD_START_RESP:
		pr_debug("[GOTA] Send CP-->AP DOWNLOAD_START_RESP\n");
		complete_all(&dpld->gota_download_start_complete);
		break;

	case GOTA_CMD_SEND_DONE_RESP:
		pr_debug("[GOTA] Send CP-->AP SEND_DONE_RESP\n");
		complete_all(&dpld->gota_send_done);
		break;

	case GOTA_CMD_UPDATE_DONE:
		pr_debug("[GOTA] Send CP-->AP UPDATE_DONE\n");
		complete_all(&dpld->gota_update_done);
		break;

	case GOTA_CMD_IMAGE_SEND_RESP:
		pr_debug("[DPRAM] Send CP-->AP IMAGE_SEND_RESP\n");
		complete_all(&dpld->dump_receive_done);
		break;

	default:
		pr_err("[GOTA] Unknown command.. %x\n", cmd);
	}
}

static irqreturn_t dpram_irq_handler(int irq, void *p_ld)
{
	u16 cp2ap;

	struct link_device *ld = (struct link_device *)p_ld;
	struct dpram_link_device *dpld = to_dpram_link_device(ld);

	cp2ap = dpram_readh(&dpld->dpram->mbx_cp2ap);

	pr_debug("[DPRAM] received CP2AP = 0x%x\n", cp2ap);

	if (cp2ap == INT_POWERSAFE_FAIL) {
		pr_err("[DPRAM] Received POWERSAFE_FAIL\n");
		goto exit_irq;
	}

	if (GOTA_CMD_VALID(cp2ap))
		gota_cmd_handler(dpld, cp2ap);
	else if (INT_CMD_VALID(cp2ap))
		command_handler(dpld, cp2ap);
	else if (INT_VALID(cp2ap))
		non_command_handler(dpld, cp2ap);
	else
		pr_err("[DPRAM] Invalid command %04x\n", cp2ap);

exit_irq:
	return IRQ_HANDLED;
}

static int dpram_attach_io_dev(struct link_device *ld, struct io_device *iod)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);

	iod->link = ld;
	/* list up io devices */
	list_add(&iod->list, &dpld->list_of_io_devices);

	return 0;
}

static int dpram_write(struct dpram_link_device *dpld,
			struct dpram_device *device,
			const unsigned char *buf,
			int len)
{
	u16 head;
	u16 tail;
	u16 irq_mask;
	int free_space;
	int last_size;

	head = dpram_readh(&device->out->head);
	tail = dpram_readh(&device->out->tail);

	if (!dpram_circ_valid(device->out_buff_size, head, tail)) {
		pr_err("[DPRAM] %s: invalid circular buffer\n", __func__);
		dpram_zero_circ(device->out);
		return -EINVAL;
	}

	free_space = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;
	if (len > free_space) {
		pr_debug("WRITE: No space in Q\n"
			 "len[%d] free_space[%d] head[%u] tail[%u] out_buff_size =%d\n",
			 len, free_space, head, tail, device->out_buff_size);
		return -EINVAL;
	}

	pr_debug("WRITE: len[%d] free_space[%d] head[%u] tail[%u] out_buff_size =%d\n",
			len, free_space, head, tail, device->out_buff_size);

	if (head < tail) {
		/* +++++++++ head ---------- tail ++++++++++ */
		memcpy((device->out_buff_addr + head), buf, len);
	} else {
		/* ------ tail +++++++++++ head ------------ */
		last_size = device->out_buff_size - head;
		memcpy((device->out_buff_addr + head), buf,
			len > last_size ? last_size : len);
		if (len > last_size) {
			memcpy(device->out_buff_addr, (buf + last_size),
				(len - last_size));
		}
	}

	/* Update new head */
	head = (u16)((head + len) % device->out_buff_size);
	dpram_writeh(head, &device->out->head);

	irq_mask = INT_MASK_VALID;

	if (len > 0)
		irq_mask |= device->mask_send;

	dpram_write_command(dpld, irq_mask);

	return len;
}

static void dpram_write_work(struct work_struct *work)
{
	struct link_device *ld =
		container_of(work, struct link_device, tx_delayed_work.work);
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_device *device;
	struct sk_buff *skb;
	bool reschedule = false;
	int ret;

	device = &dpld->dev_map[FMT_IDX];
	while ((skb = skb_dequeue(&ld->sk_fmt_tx_q))) {
		ret = dpram_write(dpld, device, skb->data, skb->len);
		if (ret < 0) {
			skb_queue_head(&ld->sk_fmt_tx_q, skb);
			reschedule = true;
			break;
		}
		dev_kfree_skb_any(skb);
	}

	device = &dpld->dev_map[RAW_IDX];
	while ((skb = skb_dequeue(&ld->sk_raw_tx_q))) {
		ret = dpram_write(dpld, device, skb->data, skb->len);
		if (ret < 0) {
			skb_queue_head(&ld->sk_raw_tx_q, skb);
			reschedule = true;
			break;
		}
		dev_kfree_skb_any(skb);
	}

	if (reschedule)
		schedule_delayed_work(&ld->tx_delayed_work,
					msecs_to_jiffies(10));
}

static int dpram_send(struct link_device *ld, struct io_device *iod,
		      struct sk_buff *skb)
{
	int len = skb->len;
	pr_debug("%s: iod->format = %d\n", __func__, iod->format);

	switch (iod->format) {
	case IPC_FMT:
		skb_queue_tail(&ld->sk_fmt_tx_q, skb);
		break;

	case IPC_RAW:
		skb_queue_tail(&ld->sk_raw_tx_q, skb);
		break;

	case IPC_BOOT:
	case IPC_RFS:
	default:
		dev_kfree_skb_any(skb);
		return 0;
	}

	schedule_delayed_work(&ld->tx_delayed_work, 0);
	return len;
}

static void dpram_table_init(struct dpram_link_device *dpld)
{
	struct dpram_device *dev;
	struct dpram_map __iomem *dpram = dpld->dpram;

	dev                 = &dpld->dev_map[FMT_IDX];
	dev->in             = &dpram->fmt_in;
	dev->in_buff_addr   = dpram->fmt_in_buff;
	dev->in_buff_size   = DP_FMT_IN_BUFF_SIZE;
	dev->out            = &dpram->fmt_out;
	dev->out_buff_addr  = dpram->fmt_out_buff;
	dev->out_buff_size  = DP_FMT_OUT_BUFF_SIZE;
	dev->mask_req_ack   = INT_MASK_REQ_ACK_F;
	dev->mask_res_ack   = INT_MASK_RES_ACK_F;
	dev->mask_send      = INT_MASK_SEND_F;

	dev                 = &dpld->dev_map[RAW_IDX];
	dev->in             = &dpram->raw_in;
	dev->in_buff_addr   = dpram->raw_in_buff;
	dev->in_buff_size   = DP_RAW_IN_BUFF_SIZE;
	dev->out            = &dpram->raw_out;
	dev->out_buff_addr  = dpram->raw_out_buff;
	dev->out_buff_size  = DP_RAW_OUT_BUFF_SIZE;
	dev->mask_req_ack   = INT_MASK_REQ_ACK_R;
	dev->mask_res_ack   = INT_MASK_RES_ACK_R;
	dev->mask_send      = INT_MASK_SEND_R;
}

static int dpram_set_dlmagic(struct link_device *ld, struct io_device *iod)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	dpram_writew(DP_MAGIC_DMDL, &dpld->dpram->magic);
	return 0;
}

static int dpram_set_ulmagic(struct link_device *ld, struct io_device *iod)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_map *dpram = (void *)dpld->dpram;
	u8 *dest;
	dest = (u8 *)(&dpram->fmt_out);

	dpram_writew(DP_MAGIC_UMDL, &dpld->dpram->magic);

	dpram_writeb((u8)START_INDEX, dest + 0);
	dpram_writeb((u8)0x1, dest + 1);
	dpram_writeb((u8)0x1, dest + 2);
	dpram_writeb((u8)0x0, dest + 3);
	dpram_writeb((u8)END_INDEX, dest + 4);

	init_completion(&dpld->gota_download_start_complete);
	dpram_write_command(dpld, CMD_DL_START_REQ);

	return 0;
}

static int
dpram_download(struct dpram_link_device *dpld, const char *buf, int len)
{
	struct dpram_map *dpram = (void *)dpld->dpram;
	struct dpram_ota_header header;
	u16 nframes;
	u16 curframe = 1;
	u16 plen;
	u8 *dest;
	int ret;

	nframes = DIV_ROUND_UP(len, DP_DEFAULT_WRITE_LEN);

	pr_debug("[GOTA] download len = %d\n", len);

	header.start_index = START_INDEX;
	header.nframes = nframes;

	while (len > 0) {
		plen = min(len, DP_DEFAULT_WRITE_LEN);
		dest = (u8 *)&dpram->fmt_out;

		pr_debug("[GOTA] Start write frame %d/%d\n", curframe, nframes);

		header.curframe = curframe;
		header.len = plen;

		memcpy(dest, &header, sizeof(header));
		dest += sizeof(header);

		memcpy(dest, buf, plen);
		dest += plen;
		buf += plen;
		len -= plen;

		dpram_writeb(END_INDEX, dest+3);

		init_completion(&dpld->gota_send_done);

		if (curframe == 1) {
			ret = wait_for_completion_interruptible_timeout(
				&dpld->gota_download_start_complete,
				GOTA_TIMEOUT);
			if (!ret) {
				pr_err("[GOTA] CP didn't send DOWNLOAD_START\n");
				return -ENXIO;
			}
		}

		dpram_write_command(dpld, CMD_IMG_SEND_REQ);
		ret = wait_for_completion_interruptible_timeout(
				&dpld->gota_send_done, GOTA_SEND_TIMEOUT);
		if (!ret) {
			pr_err("[GOTA] CP didn't send SEND_DONE_RESP\n");
			return -ENXIO;
		}

		curframe++;
	}

	dpram_write_command(dpld, CMD_DL_SEND_DONE_REQ);
	ret = wait_for_completion_interruptible_timeout(
			&dpld->gota_update_done, GOTA_TIMEOUT);
	if (!ret) {
		pr_err("[GOTA] CP didn't send UPDATE_DONE_NOTIFICATION\n");
		return -ENXIO;
	}

	return 0;
}

static int
dpram_upload(struct dpram_link_device *dpld, struct dpram_firmware *uploaddata)
{
	struct dpram_map *dpram = (void *)dpld->dpram;
	struct ul_header header;
	u8 *dest;
	u8 *buff = vmalloc(DP_DEFAULT_DUMP_LEN);
	u16 plen = 0;
	u32 tlen = 0;
	int ret;
	int region = 0;

	pr_debug("[DPRAM] dpram_upload()\n");

	ret = wait_for_completion_interruptible_timeout(
			&dpld->gota_download_start_complete,
			DUMP_START_TIMEOUT);
	if (!ret) {
		pr_err("[GOTA] CP didn't send DOWNLOAD_START\n");
		goto err_out;
	}

	wake_lock(&dpld->dpram_wake_lock);

	memset(buff, 0, DP_DEFAULT_DUMP_LEN);

	dpram_write_command(dpld, CMD_IMG_SEND_REQ);
	pr_debug("[DPRAM] write CMD_IMG_SEND_REQ(0x9400)\n");

	while (1) {
		init_completion(&dpld->dump_receive_done);
		ret = wait_for_completion_interruptible_timeout(
				&dpld->dump_receive_done, DUMP_TIMEOUT);
		if (!ret) {
			pr_err("[DPRAM] CP didn't send DATA_SEND_DONE_RESP\n");
			goto err_out;
		}

		dest = (u8 *)(&dpram->fmt_out);

		header.bop = *(u8 *)(dest);
		header.total_frame = *(u16 *)(dest + 1);
		header.curr_frame = *(u16 *)(dest + 3);
		header.len = *(u16 *)(dest + 5);

		pr_debug("total frame:%d, current frame:%d, data len:%d\n",
			header.total_frame, header.curr_frame,
			header.len);

		dest += DP_DUMP_HEADER_SIZE;
		plen = min(header.len, (u16)DP_DEFAULT_DUMP_LEN);

		memcpy(buff, dest, plen);
		dest += plen;

		ret = copy_to_user(uploaddata->firmware + tlen,	buff,  plen);
		if (ret < 0) {
			pr_err("[DPRAM] Copy to user failed\n");
			goto err_out;
		}

		tlen += plen;

		if (header.total_frame == header.curr_frame) {
			if (region) {
				uploaddata->is_delta = tlen - uploaddata->size;
				dpram_write_command(dpld, CMD_UL_RECEIVE_RESP);
				break;
			} else {
				uploaddata->size = tlen;
				region = 1;
			}
		}
		dpram_write_command(dpld, CMD_UL_RECEIVE_RESP);
	}

	pr_debug("1st dump region data size=%d\n", uploaddata->size);
	pr_debug("2st dump region data size=%d\n", uploaddata->is_delta);

	init_completion(&dpld->gota_send_done);
	ret = wait_for_completion_interruptible_timeout(
			&dpld->gota_send_done, DUMP_TIMEOUT);
	if (!ret) {
		pr_err("[GOTA] CP didn't send SEND_DONE_RESP\n");
		goto err_out;
	}

	dpram_write_command(dpld, CMD_UL_RECEIVE_DONE_RESP);
	pr_debug("[DPRAM] write CMD_UL_RECEIVE_DONE_RESP(0x9801)\n");

	dpram_writew(0, &dpld->dpram->magic); /*clear magic code */

	wake_unlock(&dpld->dpram_wake_lock);

	vfree(buff);
	return 0;

err_out:
	vfree(buff);
	dpram_writew(0, &dpld->dpram->magic);
	pr_err("CDMA dump error out\n");
	wake_unlock(&dpld->dpram_wake_lock);
	return -EIO;
}

struct link_device *dpram_create_link_device(struct platform_device *pdev)
{
	int ret;
	struct dpram_link_device *dpld;
	struct link_device *ld;
	struct resource *res;

	BUILD_BUG_ON(sizeof(struct dpram_map) != DP_DPRAM_SIZE);

	dpld = kzalloc(sizeof(struct dpram_link_device), GFP_KERNEL);
	if (!dpld)
		return NULL;

	ld = &dpld->ld;

	INIT_LIST_HEAD(&dpld->list_of_io_devices);
	skb_queue_head_init(&ld->sk_fmt_tx_q);
	skb_queue_head_init(&ld->sk_raw_tx_q);
	INIT_DELAYED_WORK(&ld->tx_delayed_work, dpram_write_work);

	wake_lock_init(&dpld->dpram_wake_lock, WAKE_LOCK_SUSPEND, "DPRAM");

	init_completion(&dpld->modem_pif_init_done);
	init_completion(&dpld->dpram_init_cmd);
	init_completion(&dpld->gota_send_done);
	init_completion(&dpld->gota_update_done);
	init_completion(&dpld->gota_download_start_complete);
	init_completion(&dpld->dump_receive_done);

	ld->name = "dpram";
	ld->attach = dpram_attach_io_dev;
	ld->send = dpram_send;
	ld->gota_start = dpram_set_dlmagic;
	ld->modem_update = dpram_modem_update;
	ld->dump_start = dpram_set_ulmagic;
	ld->dump_update = dpram_dump_update;

	dpld->clear_interrupt = dpram_clear_interrupt;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("[DPRAM] Failed to get mem region\n");
		goto err;
	}
	dpld->dpram = ioremap(res->start, resource_size(res));

	dpld->irq = platform_get_irq(pdev, 1);
	if (!dpld->irq) {
		pr_err("[MODEM_IF] %s: Failed to get IRQ\n", __func__);
		goto err;
	}

	dpram_table_init(dpld);

	atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
	atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);

	dpram_clear_interrupt(dpld);
	dpram_writeh(0, &dpld->dpram->magic);

	ret = request_irq(dpld->irq, dpram_irq_handler, IRQ_TYPE_LEVEL_LOW,
							"dpram irq", ld);
	if (ret) {
		pr_err("DPRAM interrupt handler failed\n");
		goto err;
	}
	enable_irq_wake(dpld->irq);

	return ld;

err:
	iounmap(dpld->dpram);
	kfree(dpld);
	return NULL;
}
