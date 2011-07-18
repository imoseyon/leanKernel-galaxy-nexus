/* /linux/drivers/new_modem_if/link_dev_dpram.c
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
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/platform_data/modem.h>
#include <linux/crc-ccitt.h>
#include "modem_prj.h"
#include "modem_link_device_dpram.h"

#include <linux/platform_data/dpram.h>

#include "../../../arch/arm/mach-omap2/mux44xx.h"

#define GOTA_TIMEOUT		(50 * HZ)
#define GOTA_SEND_TIMEOUT	(200 * HZ)

static int dpram_download(struct dpram_link_device *dpld,
			const unsigned char *buf,
			int Len);


static void dpram_send_interrupt_to_phone(struct dpram_link_device *dpld,
					u16 irq_mask)
{
	memcpy(dpld->m_region.mbx + 2,
			&irq_mask, DPRAM_INTR_PORT_SIZE);
}

static void dpram_clear(struct dpram_link_device *dpld)
{
	int i = 0;
	unsigned long flags;
	char *fmt_out_dest, *raw_out_dest, *fmt_in_dest, *raw_in_dest;
	int size = 0;

	pr_debug("[DPRAM] *** entering dpram_clear()\n");

	/* clear DPRAM except interrupt area */
	local_irq_save(flags);

	size = DP_HEAD_SIZE + DP_TAIL_SIZE;
	fmt_out_dest = (char *)dpld->m_region.fmt_out;
	raw_out_dest = (char *)dpld->m_region.raw_out;
	fmt_in_dest = (char *)dpld->m_region.fmt_in;
	raw_in_dest = (char *)dpld->m_region.raw_in;

	for (i = 0; i < size; i += 2) {
		*((u16 *)(fmt_out_dest + i)) = 0;
		*((u16 *)(raw_out_dest + i)) = 0;
		*((u16 *)(fmt_in_dest + i)) = 0;
		*((u16 *)(raw_in_dest + i)) = 0;
	}

	local_irq_restore(flags);

	pr_debug("[DPRAM] *** leaving dpram_clear()\n");
}

static int dpram_init_and_report(struct dpram_link_device *dpld)
{
	const u16 magic_code = DP_MAGIC_CODE;
	u16 ac_code = 0x0000;
	const u16 init_end = INT_CMD(INT_CMD_INIT_END);
	u16 magic, enable;

	/* @LDK@ write DPRAM disable code */
	memcpy(dpld->m_region.control + 2, &ac_code, sizeof(ac_code));

	/* @LDK@ dpram clear */
	dpram_clear(dpld);

	/* @LDK@ write magic code */
	memcpy(dpld->m_region.control, &magic_code, sizeof(magic_code));

	/* @LDK@ write DPRAM enable code */
	ac_code = 0x0001;
	memcpy(dpld->m_region.control + 2, &ac_code, sizeof(ac_code));

	/* @LDK@ send init end code to phone */
	dpram_send_interrupt_to_phone(dpld, init_end);

	memcpy((void *)&magic, dpld->m_region.control, sizeof(magic));
	memcpy((void *)&enable, dpld->m_region.control + 2, sizeof(enable));
	pr_debug("[DPRAM] magic code = %x, access enable = %x\n",
			magic, enable);
	pr_debug("[DPRAM] Send 0x%x to MailboxBA(Dpram init finish)\n",
			init_end);

	dpld->phone_sync = 1;

	return 0;
}

static void cmd_req_active_handler(struct dpram_link_device *dpld)
{
	dpram_send_interrupt_to_phone(dpld, INT_CMD(INT_CMD_RES_ACTIVE));
}

static void cmd_error_display_handler(struct dpram_link_device *dpld)
{
	char buf[DPRAM_ERR_MSG_LEN] = {0,};

	if (dpld->phone_status) {
		memcpy(dpld->cpdump_debug_file_name,
				dpld->m_region.fmt_in + 4,
				sizeof(dpld->cpdump_debug_file_name));
	} else {
		/* --- can't catch the CDMA watchdog reset!!*/
		sprintf((void *)buf, "8 $PHONE-OFF");
	}

	memcpy(dpld->dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	dpld->is_dpram_err = TRUE;
	kill_fasync(&dpld->dpram_err_async_q, SIGIO, POLL_IN);
}

static void cmd_phone_start_handler(struct dpram_link_device *dpld)
{
	pr_debug("[DPRAM] Received 0xc8 from Phone (Phone Boot OK).\n");
	dpld->dpram_init_cmd_wait_condition = 1;
	wake_up_interruptible(&dpld->dpram_init_cmd_wait_q);
	dpram_init_and_report(dpld);
}

static void cmd_req_time_sync_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_phone_deep_sleep_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_nv_rebuilding_handler(struct dpram_link_device *dpld)
{
	sprintf(dpld->dpram_err_buf, "NV_REBUILDING");
	dpld->is_dpram_err = TRUE;

	kill_fasync(&dpld->dpram_err_async_q, SIGIO, POLL_IN);
}

static void cmd_silent_nv_rebuilding_handler(struct dpram_link_device *dpld)
{
	sprintf(dpld->dpram_err_buf, "SILENT_NV_REBUILDING");
	dpld->is_dpram_err = TRUE;

	kill_fasync(&dpld->dpram_err_async_q, SIGIO, POLL_IN);
}

static void cmd_emer_down_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_pif_init_done_handler(struct dpram_link_device *dpld)
{
	pr_debug("[DPRAM] cmd_pif_init_done_handler\n");
	if (&dpld->modem_pif_init_done_wait_q == NULL) {
		pr_err("[DPRAM] Error - modem_pif_init_done_wait_q is NULL\n");
		return;
	}
	dpld->modem_pif_init_wait_condition = 1;
	pr_debug(" modem_pif_init_wait_condition =%d\n",
		dpld->modem_pif_init_wait_condition);
	wake_up_interruptible(&dpld->modem_pif_init_done_wait_q);
}

static void command_handler(struct dpram_link_device *dpld, u16 cmd)
{
	switch (cmd) {
	case INT_CMD_REQ_ACTIVE:
		cmd_req_active_handler(dpld);
		break;

	case INT_CMD_ERR_DISPLAY:
		cmd_error_display_handler(dpld);
		break;

	case INT_CMD_PHONE_START:
		cmd_phone_start_handler(dpld);
		break;

	case INT_CMD_REQ_TIME_SYNC:
		cmd_req_time_sync_handler();
		break;

	case INT_CMD_PHONE_DEEP_SLEEP:
		cmd_phone_deep_sleep_handler();
		break;

	case INT_CMD_NV_REBUILDING:
		cmd_nv_rebuilding_handler(dpld);
		break;

	case INT_CMD_EMER_DOWN:
		cmd_emer_down_handler();
		break;

	case INT_CMD_PIF_INIT_DONE:
		cmd_pif_init_done_handler(dpld);
		break;

	case INT_CMD_SILENT_NV_REBUILDING:
		cmd_silent_nv_rebuilding_handler(dpld);
		break;

	case INT_CMD_NORMAL_POWER_OFF:
		/*ToDo:*/
		/*kernel_sec_set_cp_ack()*/;
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

	if (ret  < 0) {
		pr_err("[%s:%d] Copy from user failed\n",
							__func__, __LINE__);
		return -EINVAL;
	} else if (dpram_download(dpld, buff, pfw->size) < 0) {
		pr_err("firmware write failed\n");
		return -EIO;
		}

	vfree(buff);

	return ret;
}


static int dpram_modem_update(struct link_device *ld,
				struct io_device *iod,
				unsigned long _arg)
{
	int ret = 0;
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_firmware fw;

	pr_debug("[GOTA] dpram_modem_update\n");

	ret = copy_from_user((void *)&fw, (void *)_arg, sizeof(fw));
	if (ret  < 0) {
		pr_err("copy from user failed!");
		ret = -EINVAL;
	} else if (dpram_process_modem_update(dpld, &fw) < 0) {
		pr_err("firmware write failed\n");
		ret = -EIO;
	}
	return ret;
}

static void dpram_drop_data(struct dpram_device *device)
{
	u16 head, tail;

	memcpy(&head, device->in_head_addr, sizeof(head));
	memcpy(&tail, device->in_tail_addr, sizeof(tail));
	pr_debug("[DPRAM] %s, head: %d, tail: %d\n", __func__, head, tail);

	if (head >= device->in_buff_size || tail >= device->in_buff_size) {
		head = tail = 0;
		memcpy(device->in_head_addr, (u16 *)&head, sizeof(head));
	}
	memcpy(device->in_tail_addr, (u16 *)&head, sizeof(head));
}

static int dpram_read(struct dpram_link_device *dpld,
		struct dpram_device *device, int dev_idx)
{
	struct io_device *iod = NULL;
	int   size = 0, tmp_size = 0;
	u16   head = 0, tail = 0;
	u16   up_tail = 0;
	char *buff = NULL;

	memcpy(&head, device->in_head_addr, sizeof(head));
	memcpy(&tail, device->in_tail_addr, sizeof(tail));
	pr_debug("=====> %s,  head: %d, tail: %d\n", __func__, head, tail);

	if (device->in_head_saved == head) {
		pr_err("[DPRAM] device->in_head_saved == head (NO new data)\n");
		goto err_dpram_read;
	}

	if (head == tail) {
		pr_err("[DPRAM] head == tail\n");
		goto err_dpram_read;
	}

	if (tail >= device->in_buff_size || head >= device->in_buff_size) {
		pr_err("[DPRAM] head(%d) or tail(%d) >= buff_size(%lu)\n",
			head, tail, device->in_buff_size);
		goto err_dpram_read;
	}

	list_for_each_entry(iod, &dpld->list_of_io_devices, list) {
		if ((dev_idx == FMT_IDX && iod->format == IPC_FMT) ||
			(dev_idx == RAW_IDX && iod->format == IPC_MULTI_RAW))
			break;
	}
	if (!iod) {
		pr_err("[DPRAM] iod == NULL\n");
		goto err_dpram_read;
	}

	/* Get data size in DPRAM*/
	size = (head > tail) ? (head - tail) :
		(device->in_buff_size - tail + head);

	/* ----- (tail) 7f 00 00 7e (head) ----- */
	if (head > tail) {
		buff = (char *)device->in_buff_addr + tail;
		if (iod->recv(iod, buff, size) < 0)
			dpram_drop_data(device);
		pr_debug("[DPRAM] size : %d\n", size);
	} else { /* 00 7e (head) ----------- (tail) 7f 00 */
		/* 1. tail -> buffer end.*/
		tmp_size = device->in_buff_size - tail;
		buff = (char *)device->in_buff_addr + tail;
		if (iod->recv(iod, buff, tmp_size) < 0)
			dpram_drop_data(device);

		/* 2. buffer start -> head.*/
		if (size > tmp_size) {
			buff = (char *)device->in_buff_addr;
			if (iod->recv(iod, buff, (size - tmp_size)) < 0)
				dpram_drop_data(device);
		}
	}

	/* new tail */
	up_tail = (u16)((tail + size) % device->in_buff_size);
	memcpy(device->in_tail_addr, (u16 *)&up_tail, sizeof(up_tail));
	pr_debug(" head= %d, tail = %d", head, up_tail);

	device->in_head_saved = head;
	device->in_tail_saved = up_tail;

	return size;

err_dpram_read:
	return -EINVAL;
}

static void non_command_handler(struct dpram_link_device *dpld,
				u16 non_cmd)
{
	struct dpram_device *device = NULL;
	u16 head = 0, tail = 0;
	u16 magic = 0, access = 0;
	int ret = 0;

	pr_debug("[DPRAM] Entering non_command_handler(0x%04X)\n", non_cmd);

	memcpy((void *)&magic, dpld->m_region.control, sizeof(magic));
	memcpy((void *)&access, dpld->m_region.control + 2, sizeof(access));

	if (!access || magic != DP_MAGIC_CODE) {
		pr_err("fmr recevie error!!!! phone status =%d, access = 0x%x, magic =0x%x",
				dpld->phone_status, access, magic);
		return;
	}

	/* Check formatted data region */
	device = &dpld->dev_map[FMT_IDX];
	memcpy(&head, device->in_head_addr, sizeof(head));
	memcpy(&tail, device->in_tail_addr, sizeof(tail));

	if (head != tail) {
		if (non_cmd & INT_MASK_REQ_ACK_F)
			atomic_inc(&dpld->fmt_txq_req_ack_rcvd);

		ret = dpram_read(dpld, device, FMT_IDX);
		if (ret < 0) {
			pr_err("%s, dpram_read failed\n", __func__);
			/* TODO: ... wrong.. */
		}

		if (atomic_read(&dpld->fmt_txq_req_ack_rcvd) > 0) {
			dpram_send_interrupt_to_phone(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_F));
			atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_F) {
			dpram_send_interrupt_to_phone(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_F));
			atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);
		}
	}

	/* Check raw data region */
	device = &dpld->dev_map[RAW_IDX];
	memcpy(&head, device->in_head_addr, sizeof(head));
	memcpy(&tail, device->in_tail_addr, sizeof(tail));

	if (head != tail) {
		if (non_cmd & INT_MASK_REQ_ACK_R)
			atomic_inc(&dpld->raw_txq_req_ack_rcvd);

		ret = dpram_read(dpld, device, RAW_IDX);
		if (ret < 0) {
			pr_err("%s, dpram_read failed\n", __func__);
			/* TODO: ... wrong.. */
		}

		if (atomic_read(&dpld->raw_txq_req_ack_rcvd) > 0) {
			dpram_send_interrupt_to_phone(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_R));
			atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_R) {
			dpram_send_interrupt_to_phone(dpld,
				INT_NON_CMD(INT_MASK_RES_ACK_R));
			atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
		}
	}
}

static irqreturn_t dpram_irq_handler(int irq, void *p_ld)
{
	u16 irq_mask = 0;

	struct link_device *ld = (struct link_device *)p_ld;
	struct dpram_link_device *dpld = to_dpram_link_device(ld);

	disable_irq_wake(irq);

	memcpy((u16 *)&irq_mask, (u16 *)dpld->m_region.mbx, sizeof(irq_mask));
	pr_debug("received mailboxAB = 0x%x\n", irq_mask);

	/* valid bit verification.
	* or Say something about the phone being dead...*/
	if (!(irq_mask & (INT_MASK_VALID | INT_GOTA_MASK_VALID)) ||
			irq_mask == INT_POWERSAFE_FAIL)
		goto exit_irq;

	if (irq_mask & INT_GOTA_MASK_VALID) {
		dpld->gota_irq_hander_cmd = irq_mask;
		queue_work(dpld->gota_wq, &dpld->gota_cmd_work);
	} else if (irq_mask & INT_MASK_CMD) {
		irq_mask &= ~(INT_MASK_VALID | INT_MASK_CMD);
		command_handler(dpld, irq_mask);
	} else {
		irq_mask &= ~INT_MASK_VALID;
		non_command_handler(dpld, irq_mask);
	}
	goto exit_irq;

exit_irq:
	enable_irq_wake(irq);
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
	u16   head = 0, tail = 0, up_head = 0;
	u16 irq_mask = 0;
	int free_space = 0;
	int last_size = 0;

	memcpy((u16 *)&head, device->out_head_addr, sizeof(head));
	memcpy((u16 *)&tail, device->out_tail_addr, sizeof(tail));
	free_space = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;
	if (len > free_space) {
		pr_err("WRITE: No space in Q\n");
		pr_err("len[%d] free_space[%d] head[%u] tail[%u] out_buff_size =%lu\n",
			len, free_space, head, tail, device->out_buff_size);
		return -EINVAL;
	}

	pr_debug("WRITE: len[%d] free_space[%d] head[%u] tail[%u] out_buff_size =%lu\n",
			len, free_space, head, tail, device->out_buff_size);

	pr_debug("%s, head: %d, tail: %d\n", __func__, head, tail);
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
	up_head = (u16)((head + len) % device->out_buff_size);
	memcpy(device->out_head_addr, (u16 *)&up_head, sizeof(head));

	device->out_head_saved = up_head;
	device->out_tail_saved = tail;

	irq_mask = INT_MASK_VALID;

	if (len > 0)
		irq_mask |= device->mask_send;

	dpram_send_interrupt_to_phone(dpld, irq_mask);

	return len;
}

static int dpram_send
(
	struct link_device *ld,
	struct io_device *iod,
	struct sk_buff *skb
)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);
	struct dpram_device *device = NULL;
	int ret;

	switch (iod->format) {
	pr_debug(" %s iod->format = %d\n", __func__, iod->format);
	case IPC_FMT:
		device = &dpld->dev_map[FMT_IDX];
		break;

	case IPC_RAW:
		device = &dpld->dev_map[RAW_IDX];
		break;

	case IPC_BOOT:
	case IPC_RFS:
	default:
		device = NULL;
		return 0;
	}

	ret = dpram_write(dpld, device, skb->data, skb->len);

	dev_kfree_skb_any(skb);

	return ret;
}

void ClearPendingInterruptFromModem(struct dpram_link_device *dpld)
{
	u16 in_interrupt = 0;
	memcpy((void *)&in_interrupt,
		dpld->m_region.mbx,
		sizeof(in_interrupt));
}

static int dpram_shared_bank_remap(struct platform_device *pdev,
				struct dpram_link_device *dpld)
{
	struct resource *res;

	pr_debug("dev->num_resources = %d\n", pdev->num_resources);
	pr_debug("&dev->resource[6] = %x\n", pdev->resource[6].start);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 2\n");
		return -EINVAL;
	}
	dpld->m_region.control = (u8 *)ioremap_nocache(res->start,
					res->end - res->start + 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 3\n");
		return -EINVAL;
	}
	dpld->fmt_out_buff_size = res->end - res->start - 3;
	dpld->m_region.fmt_out = (u8 *)ioremap_nocache(res->start ,
					res->end - res->start + 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 4\n");
		return -EINVAL;
	}
	dpld->raw_out_buff_size = res->end - res->start - 3;
	dpld->m_region.raw_out = (u8 *)ioremap_nocache(res->start,
					res->end - res->start + 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 5\n");
		return -EINVAL;
	}
	dpld->fmt_in_buff_size =  res->end - res->start - 3;
	dpld->m_region.fmt_in = (u8 *)ioremap_nocache(res->start,
					res->end - res->start + 1);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 4);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 6\n res = %s\n",
			res->name);
		return -EINVAL;
	}
	dpld->raw_in_buff_size = res->end - res->start - 3;
	dpld->m_region.raw_in = (u8 *)ioremap_nocache(res->start,
					res->end - res->start + 1);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	if (!res) {
		pr_err("[DPRAM]failed to get mem region 7\n");
		return -EINVAL;
	}

	dpld->m_region.mbx = (u8 *)ioremap_nocache(res->start, 4);

	return 0;
}

static void dpram_table_init(struct dpram_link_device *dpld)
{
	dpld->dev_map[FMT_IDX].in_head_addr = (u16 *)dpld->m_region.fmt_in;
	dpld->dev_map[FMT_IDX].in_tail_addr =
					(u16 *)(dpld->m_region.fmt_in + 2);
	dpld->dev_map[FMT_IDX].in_buff_addr =
					(u8 *)(dpld->m_region.fmt_in + 4);
	dpld->dev_map[FMT_IDX].in_buff_size = dpld->fmt_in_buff_size;

	dpld->dev_map[FMT_IDX].in_head_saved = 0;
	dpld->dev_map[FMT_IDX].in_tail_saved = 0;

	dpld->dev_map[FMT_IDX].out_head_addr = (u16 *)dpld->m_region.fmt_out;
	dpld->dev_map[FMT_IDX].out_tail_addr =
					(u16 *)(dpld->m_region.fmt_out + 2);
	dpld->dev_map[FMT_IDX].out_buff_addr =
					(u8 *)(dpld->m_region.fmt_out + 4);
	dpld->dev_map[FMT_IDX].out_buff_size = dpld->fmt_out_buff_size;

	dpld->dev_map[FMT_IDX].out_head_saved = 0;
	dpld->dev_map[FMT_IDX].out_tail_saved = 0;

	dpld->dev_map[FMT_IDX].mask_req_ack = INT_MASK_REQ_ACK_F;
	dpld->dev_map[FMT_IDX].mask_res_ack = INT_MASK_RES_ACK_F;
	dpld->dev_map[FMT_IDX].mask_send    = INT_MASK_SEND_F;

	dpld->dev_map[RAW_IDX].in_head_addr = (u16 *)dpld->m_region.raw_in;
	dpld->dev_map[RAW_IDX].in_tail_addr =
					(u16 *)(dpld->m_region.raw_in + 2);
	dpld->dev_map[RAW_IDX].in_buff_addr =
					(u8 *)(dpld->m_region.raw_in + 4);
	dpld->dev_map[RAW_IDX].in_buff_size = dpld->raw_in_buff_size;

	dpld->dev_map[RAW_IDX].in_head_saved = 0;
	dpld->dev_map[RAW_IDX].in_tail_saved = 0;

	dpld->dev_map[RAW_IDX].out_head_addr = (u16 *)dpld->m_region.raw_out;
	dpld->dev_map[RAW_IDX].out_tail_addr =
					(u16 *)(dpld->m_region.raw_out + 2);
	dpld->dev_map[RAW_IDX].out_buff_addr =
					(u8 *)(dpld->m_region.raw_out + 4);
	dpld->dev_map[RAW_IDX].out_buff_size = dpld->raw_out_buff_size;

	dpld->dev_map[RAW_IDX].out_head_saved = 0;
	dpld->dev_map[RAW_IDX].out_tail_saved = 0;

	dpld->dev_map[RAW_IDX].mask_req_ack = INT_MASK_REQ_ACK_R;
	dpld->dev_map[RAW_IDX].mask_res_ack = INT_MASK_RES_ACK_R;
	dpld->dev_map[RAW_IDX].mask_send    = INT_MASK_SEND_R;
}

static void dpram_write_magic_code(struct dpram_link_device *dpld, u32 cmd)
{
	memcpy(dpld->m_region.control, &cmd, sizeof(cmd));
}

static void dpram_write_command(struct dpram_link_device *dpld, u16 cmd)
{
	memcpy(dpld->m_region.mbx+2, &cmd, sizeof(cmd));
}

u16 calc_total_frame(u32 nDividend, u16 nDivisor)
{
	u16 nCompVal1 = 0;
	u16 nCompVal2 = 0;

	nCompVal1 = (u16)(nDividend / nDivisor);
	nCompVal2 = (u16)(nDividend  - (nCompVal1 * nDivisor));

	if (nCompVal2 > 0)
		nCompVal1++;

	return nCompVal1;
}

static int dpram_set_dlmagic(struct link_device *ld, struct io_device *iod)
{
	struct dpram_link_device *dpld = to_dpram_link_device(ld);

	dpram_write_magic_code(dpld, MAGIC_DMDL);

	return 0;
}

static int dpram_download(struct dpram_link_device *dpld,
			const unsigned char *buf,
			int Len)
{
	u32 dwWriteLen, dwWrittenLen, dwTotWrittenLen;
	u16 nTotalFrame = 0;
	u8 *pDest;
	u8 *pDest_Data;
	u16 pLen = 0;
	u16 nCrc;
	int nrRetry = 0;
	u16 g_TotFrame = 0;
	u16 g_CurFrame = 1;
	u16 currDownFrame = 0;
	u32 control_size = 0;
	u32 fmt_out_size = 0;
	u32 raw_out_size = 0;
	u32 fmt_in_size = 0;
	u32 raw_in_size = 0;
	u32 mailbox_size = 0;
	int buffOffest = 0;
	const u8 *crcdata;
	int download_send_done_RetVal = 0;
	int download_update_done_RetVal = 0;

	pr_debug("[GOTA] Start download\n");

	control_size = 4;
	fmt_out_size = dpld->fmt_out_buff_size + 4;
	raw_out_size = dpld->raw_out_buff_size + 4;
	fmt_in_size =  dpld->fmt_in_buff_size + 4;
	raw_in_size =  dpld->raw_in_buff_size + 4;
	mailbox_size = 4;

	nTotalFrame = calc_total_frame(Len, DPDN_DEFAULT_WRITE_LEN);
	pr_debug("[GOTA] download Len = %d\n", Len);

	g_TotFrame =  nTotalFrame;

	while (dwTotWrittenLen < Len) {
		pr_debug("[GOTA]Start write : currDownFrame = %d,nTotalFrame = %d\n",
			currDownFrame, nTotalFrame);
		dwWriteLen = min((u32)Len - dwTotWrittenLen,
			(u32)DPDN_DEFAULT_WRITE_LEN);
		pDest = (u8 *)(dpld->m_region.fmt_out);
		pLen   = (u16)min(dwWriteLen, (u32)DPDN_DEFAULT_WRITE_LEN);

		pr_debug("[GOTA] pLen=%d, dwWriteLen=%d\n", pLen, dwWriteLen);

		*pDest++ = START_INDEX;

		*pDest++ = (u8)g_TotFrame;
		*pDest++ = (u8)((u16)g_TotFrame >> 8);

		*pDest++ = (u8)g_CurFrame;
		*pDest++ = (u8)((u16)g_CurFrame >> 8);
		g_CurFrame++;

		*pDest++ = (u8)pLen;
		*pDest++ = (u8)((u16)pLen >> 8);

		pDest_Data = pDest;

		if (pLen == DPDN_DEFAULT_WRITE_LEN) {
			memcpy((u8 *)pDest, (u8 *)buf, fmt_out_size - 7);
			buffOffest = fmt_out_size - 7;

			pDest = (u8 *)(dpld->m_region.raw_out);
			memcpy((u8 *)pDest,
				(u8 *)(buf + buffOffest),
				pLen - buffOffest);
			pDest =	(u8 *)(dpld->m_region.raw_out +
						pLen - buffOffest);
		}

		if (pLen%2 != 0)
			*pDest++ = 0xff;

		if (pLen < DPDN_DEFAULT_WRITE_LEN) {
			if (pLen < fmt_out_size - 7) {
				memcpy((u8 *)pDest, (u8 *)buf, pLen);
				pDest += pLen;
				memset((void *)pDest,
						0x0,
						fmt_out_size - 7 - pLen);
				pDest = (u8 *)(dpld->m_region.raw_out);

				memset((void *)pDest,
					0x0,
					DPDN_DEFAULT_WRITE_LEN -
						(fmt_out_size - 7));
				pDest = (u8 *)(dpld->m_region.raw_out +
					DPDN_DEFAULT_WRITE_LEN -
					(fmt_out_size - 7));
			} else {
				memcpy((u8 *)pDest, (u8 *)buf,
						fmt_out_size - 7);
				pDest = (u8 *)(dpld->m_region.raw_out);

				memcpy((u8 *)pDest, (u8 *)buf,
						pLen - (fmt_out_size - 7));
				pDest = (u8 *)(dpld->m_region.raw_out + pLen -
						(fmt_out_size - 7));

				memset((void *)pDest,
					0x0,
					DPDN_DEFAULT_WRITE_LEN -
					(fmt_out_size - 7) - pLen);
				pDest = (u8 *)(dpld->m_region.raw_out +
					DPDN_DEFAULT_WRITE_LEN -
					(fmt_out_size - 7) - pLen);
				}
			}

		crcdata = (u8 *) pDest_Data;
		nCrc = crc_ccitt((u16)CRC_16_L_SEED, crcdata, (size_t)pLen/2);
		*pDest++ = (u8)(nCrc);
		*pDest++ = (u8)((u16)nCrc >> 8) & 0xFF;
		pDest = (u8 *)(dpld->m_region.raw_out +
			(BSP_DPRAM_BASE_SIZE - fmt_out_size - control_size) -
			DPRAM_INDEX_SIZE);

		*pDest++ = END_INDEX;

		if (currDownFrame == 0) {
			dpld->gota_download_start_cmd_wait_condition = 1;
			wake_up_interruptible(
			&dpld->gota_download_start_cmd_wait_q);
		} else {
			dpram_write_command(dpld, CMD_IMG_SEND_REQ);
			pr_debug("[GOTA] Send AP-->CP CMD_IMG_SEND_REQ(0x9400)\n");
		}

		download_send_done_RetVal =
			wait_event_interruptible_timeout(
			dpld->gota_send_done_cmd_wait_q,
				dpld->gota_send_done_cmd_wait_condition,
				GOTA_SEND_TIMEOUT);
		if (!download_send_done_RetVal) {
			pr_warn("[GOTA] CP don't send SEND_DONE_RESPONSE(packet).\n");
			pr_warn("init_cmd_wait_condition is 0 and wait timeout happend\n");
			return -ENXIO;
		}
		dpld->gota_send_done_cmd_wait_condition = 0;

		dwWrittenLen = pLen;

		if (dwWrittenLen > 0) {
			dwTotWrittenLen += dwWrittenLen;
			buf += dwWrittenLen;
		} else
			pr_err("[GOTA] Failed download for dwWrittenLen < 0\n");

		nrRetry = 0;

		currDownFrame++;
	}

	g_CurFrame = 1;

	dpram_write_command(dpld, CMD_DL_SEND_DONE_REQ);
	pr_debug("[GOTA] Send AP-->CP CMD_DL_SEND_DONE_REQ(0x9600)\n");

	download_update_done_RetVal =
		wait_event_interruptible_timeout(
		dpld->gota_update_done_cmd_wait_q,
			dpld->gota_update_done_cmd_wait_condition,
			GOTA_TIMEOUT);
	if (!download_update_done_RetVal) {
		pr_warn("[GOTA] CP don't send UPDATE_DONE_NOTIFICATION.\n");
		pr_warn("init_cmd_wait_condition is 0 and wait timeout happend\n");
		return -ENXIO;
	}

	return TRUE;
}

static void if_gota_cmd_work(struct work_struct *work)
{
	int download_start_RetVal = 0;
	struct dpram_link_device *dpld =
		container_of(work, struct dpram_link_device, gota_cmd_work);

	pr_debug("[GOTA] cmd_work\n");

	switch (dpld->gota_irq_hander_cmd) {
	case MASK_CMD_RECEIVE_READY_NOTIFICATION:
		pr_debug("[GOTA] Send CP-->AP CMD_RECEIVE_READY_NOTIFICATION(0xA100)\n");
		dpram_write_command(dpld, CMD_DL_START_REQ);
		pr_debug("[GOTA] Send AP-->CP CMD_DL_START_REQ(0x9200)\n");
		break;
	case MASK_CMD_DOWNLOAD_START_RESPONSE:
		pr_debug("[GOTA] Send CP-->AP CMD_RECEIVE_READY_NOTIFICATION(0xA301)\n");
		download_start_RetVal =
			wait_event_interruptible_timeout(
			dpld->gota_download_start_cmd_wait_q,
				dpld->gota_download_start_cmd_wait_condition,
				GOTA_TIMEOUT);
		if (!download_start_RetVal) {
			pr_warn("[GOTA] CP don't send DOWNLOAD_START_RESPONSE.\n");
			pr_warn("[GOTA] init_cmd_wait_condition is 0 and wait timeout happend\n");
		}
		dpram_write_command(dpld, CMD_IMG_SEND_REQ);
		pr_debug("[GOTA] Send AP-->CP CMD_DL_START_REQ(0x9400)\n");
		break;

	case MASK_CMD_SEND_DONE_RESPONSE:
		pr_debug("[GOTA] Send CP-->AP CMD_SEND_DONE_RESPONSE(0xA701) about CMD_IMG_SEND_REQ(0x9400)\n");
		dpld->gota_send_done_cmd_wait_condition = 1;
		wake_up_interruptible(&dpld->gota_send_done_cmd_wait_q);
		break;

	case MASK_CMD_UPDATE_DONE_NOTIFICATION:
		pr_debug("[GOTA] Send CP-->AP CMD_UPDATE_DONE_NOTIFICATION(0xA900)\n");
		dpld->gota_update_done_cmd_wait_condition = 1;
		wake_up_interruptible(&dpld->gota_update_done_cmd_wait_q);
		break;
	default:
		pr_err("[GOTA] Unknown command.. %x\n",
					dpld->gota_irq_hander_cmd);
	}
}


static int if_dpram_init(struct platform_device *pdev, struct link_device *ld)
{
	int ret = 0;

	struct dpram_link_device *dpld = to_dpram_link_device(ld);

	dpld->is_dpram_err = FALSE;
	strcpy(&dpld->cpdump_debug_file_name[0], "CDMA Crash");
	wake_lock_init(&dpld->dpram_wake_lock, WAKE_LOCK_SUSPEND, "DPRAM");

	init_waitqueue_head(&dpld->modem_pif_init_done_wait_q);

	dpram_shared_bank_remap(pdev, dpld);
	dpram_table_init(dpld);

	dpld->gota_wq = create_singlethread_workqueue("gota_cmd_wq");
	if (!dpld->gota_wq) {
		pr_err("[GOTA] fail to create work Q.\n");
		return -ENOMEM;
	}
	init_waitqueue_head(&dpld->dpram_init_cmd_wait_q);

	init_waitqueue_head(&dpld->gota_download_start_cmd_wait_q);

	init_waitqueue_head(&dpld->gota_send_done_cmd_wait_q);

	init_waitqueue_head(&dpld->gota_update_done_cmd_wait_q);

	INIT_WORK(&dpld->gota_cmd_work, if_gota_cmd_work);

	ld->irq = IRQ_DPRAM_INT_N;

	atomic_set(&dpld->raw_txq_req_ack_rcvd, 0);
	atomic_set(&dpld->fmt_txq_req_ack_rcvd, 0);

	INIT_WORK(&dpld->xmit_work_struct, NULL);

	ClearPendingInterruptFromModem(dpld);
	ret = request_irq(ld->irq, dpram_irq_handler, IRQ_TYPE_LEVEL_LOW,
			"dpram irq", ld);
	if (ret) {
		pr_err("DPRAM interrupt handler failed.\n");
		return -1;
	}

	enable_irq_wake(ld->irq);

	pr_debug("[DPRAM] if_dpram_init() done : %d\n", ret);
	return ret;
}

struct link_device *dpram_create_link_device(struct platform_device *pdev)
{
	int ret;
	struct dpram_link_device *dpld;
	struct link_device *ld;
	struct modem_data *pdata;

	pdata = pdev->dev.platform_data;

	dpld = kzalloc(sizeof(struct dpram_link_device), GFP_KERNEL);
	if (!dpld)
		return NULL;

	INIT_LIST_HEAD(&dpld->list_of_io_devices);
	skb_queue_head_init(&dpld->ld.sk_fmt_tx_q);
	skb_queue_head_init(&dpld->ld.sk_raw_tx_q);

	ld = &dpld->ld;
	dpld->pdata = pdata;


	ld->name = "dpram";
	ld->attach = dpram_attach_io_dev;
	ld->send = dpram_send;
	ld->gota_start = dpram_set_dlmagic;
	ld->modem_update = dpram_modem_update;

	dpld->clear_interrupt = ClearPendingInterruptFromModem;

	ret = if_dpram_init(pdev, ld);
	if (ret)
		return NULL;

	pr_debug("[MODEM_IF] %s : create_io_device DONE\n", dpld->ld.name);
	return ld;
}
