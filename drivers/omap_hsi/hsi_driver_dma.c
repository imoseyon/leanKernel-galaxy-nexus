/*
 * hsi_driver_dma.c
 *
 * Implements HSI low level interface driver functionality with DMA support.
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

#include <linux/dma-mapping.h>
#include "hsi_driver.h"

#define HSI_SYNC_WRITE	0
#define HSI_SYNC_READ	1
#define HSI_L3_TPUT	13428	/* 13428 KiB/s => ~110 Mbit/s */

static unsigned char hsi_sync_table[2][2][8] = {
	{
	 {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
	 {0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00}
	 }, {
	     {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},
	     {0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f}
	     }
};

/**
 * hsi_is_dma_read_int_pending - Indicates if a DMA read interrupt is pending
 * @hsi_ctrl - HSI controller of the GDD.
 *
 * Needs to be called holding the hsi_controller lock
 *
 * Returns true if DMA read interrupt is pending, else false
 */
bool hsi_is_dma_read_int_pending(struct hsi_dev *hsi_ctrl)
{
	void __iomem *base = hsi_ctrl->base;
	unsigned int gdd_lch = 0;
	u32 status_reg = 0;
	int i, j;
	status_reg = hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG);
	status_reg &= hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	if (!status_reg)
		return false;

	/* Scan all enabled DMA channels */
	for (gdd_lch = 0; gdd_lch < hsi_ctrl->gdd_chan_count; gdd_lch++) {
		if (!(status_reg & HSI_GDD_LCH(gdd_lch)))
			continue;
		for (i = 0; i < hsi_ctrl->max_p; i++)
			for (j = 0; j < hsi_ctrl->hsi_port[i].max_ch; j++)
				if (hsi_ctrl->hsi_port[i].
					hsi_channel[j].read_data.lch == gdd_lch)
					return true;
	}
	return false;
}
/**
 * hsi_get_free_lch - Get a free GDD(DMA) logical channel
 * @hsi_ctrl - HSI controller of the GDD.
 *
 * Needs to be called holding the hsi_controller lock
 *
 * Returns the logical channel number, or -EBUSY if none available
 */
static int hsi_get_free_lch(struct hsi_dev *hsi_ctrl)
{
	unsigned int enable_reg;
	int          i, lch;

	enable_reg = hsi_inl(hsi_ctrl->base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	lch = hsi_ctrl->last_gdd_lch;
	for (i = 0; i < hsi_ctrl->gdd_chan_count; i++) {
		if (++lch >= hsi_ctrl->gdd_chan_count)
			lch = 0;
		if ((enable_reg & HSI_GDD_LCH(lch)) == 0) {
			hsi_ctrl->last_gdd_lch = lch;
			return lch;
		}
	}
	return -EBUSY;
}

/**
 * hsi_driver_write_dma - Program GDD [DMA] to write data from memory to
 * the hsi channel buffer.
 * @hsi_channel - pointer to the hsi_channel to write data to.
 * @data - 32-bit word pointer to the data.
 * @size - Number of 32bit words to be transfered.
 *
 * hsi_controller lock must be held before calling this function.
 *
 * Return 0 on success and < 0 on error.
 */
int hsi_driver_write_dma(struct hsi_channel *hsi_channel, u32 * data,
			 unsigned int size)
{
	struct hsi_dev *hsi_ctrl = hsi_channel->hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = hsi_channel->hsi_port->port_number;
	unsigned int channel = hsi_channel->channel_number;
	unsigned int sync;
	int lch;
	dma_addr_t src_addr;
	dma_addr_t dest_addr;
	u16 tmp;
	int fifo;

	if ((size < 1) || (data == NULL))
		return -EINVAL;

	lch = hsi_get_free_lch(hsi_ctrl);
	if (lch < 0) {
		dev_err(hsi_ctrl->dev, "No free DMA channels.\n");
		return -EBUSY;	/* No free GDD logical channels. */
	} else {
		dev_dbg(hsi_ctrl->dev, "Allocated DMA channel %d for write on"
					" HSI channel %d.\n", lch,
					hsi_channel->channel_number);
	}

	/* NOTE: Getting a free gdd logical channel and
	 * reserve it must be done atomicaly. */
	hsi_channel->write_data.lch = lch;

	/* Sync is required for SSI but not for HSI */
	sync = hsi_sync_table[HSI_SYNC_WRITE][port - 1][channel];

	src_addr = dma_map_single(hsi_ctrl->dev, data, size * 4, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(hsi_ctrl->dev, src_addr))) {
		dev_err(hsi_ctrl->dev, "Failed to create DMA write mapping.\n");
		return -ENOMEM;
	}

	tmp = HSI_SRC_SINGLE_ACCESS0 |
	    HSI_SRC_MEMORY_PORT |
	    HSI_DST_SINGLE_ACCESS0 |
	    HSI_DST_PERIPHERAL_PORT | HSI_DATA_TYPE_S32;
	hsi_outw(tmp, base, HSI_GDD_CSDP_REG(lch));

	tmp = HSI_SRC_AMODE_POSTINC | HSI_DST_AMODE_CONST | sync;
	hsi_outw(tmp, base, HSI_GDD_CCR_REG(lch));

	hsi_outw((HSI_BLOCK_IE | HSI_TOUT_IE), base, HSI_GDD_CCIR_REG(lch));

	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev, "No valid FIFO id for DMA "
				"transfer to FIFO.\n");
			return -EFAULT;
		}
		/* HSI CDSA register takes a FIFO ID when copying to FIFO */
		hsi_outl(fifo, base, HSI_GDD_CDSA_REG(lch));
	} else {
		dest_addr = hsi_ctrl->phy_base + HSI_HST_BUFFER_CH_REG(port,
								channel);
		/* SSI CDSA register always takes a 32-bit address */
		hsi_outl(dest_addr, base, HSI_GDD_CDSA_REG(lch));
	}

	/* HSI CSSA register takes a 32-bit address when copying from memory */
	/* SSI CSSA register always takes a 32-bit address */
	hsi_outl(src_addr, base, HSI_GDD_CSSA_REG(lch));
	hsi_outw(size, base, HSI_GDD_CEN_REG(lch));

	/* TODO : Need to clean interrupt status here to avoid spurious int */

	hsi_outl_or(HSI_GDD_LCH(lch), base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outw_or(HSI_CCR_ENABLE, base, HSI_GDD_CCR_REG(lch));

	return 0;
}

/**
 * hsi_driver_read_dma - Program GDD [DMA] to write data to memory from
 * the hsi channel buffer.
 * @hsi_channel - pointer to the hsi_channel to read data from.
 * @data - 32-bit word pointer where to store the incoming data.
 * @size - Number of 32bit words to be transfered to the buffer.
 *
 * hsi_controller lock must be held before calling this function.
 *
 * Return 0 on success and < 0 on error.
 */
int hsi_driver_read_dma(struct hsi_channel *hsi_channel, u32 * data,
			unsigned int count)
{
	struct hsi_dev *hsi_ctrl = hsi_channel->hsi_port->hsi_controller;
	void __iomem *base = hsi_ctrl->base;
	unsigned int port = hsi_channel->hsi_port->port_number;
	unsigned int channel = hsi_channel->channel_number;
	unsigned int sync;
	int lch;
	dma_addr_t src_addr;
	dma_addr_t dest_addr;
	u16 tmp;
	int fifo;

	lch = hsi_get_free_lch(hsi_ctrl);
	if (lch < 0) {
		dev_err(hsi_ctrl->dev, "No free DMA channels.\n");
		return -EBUSY;	/* No free GDD logical channels. */
	} else {
		dev_dbg(hsi_ctrl->dev, "Allocated DMA channel %d for read on"
					" HSI channel %d.\n", lch,
					hsi_channel->channel_number);
	}

	/* When DMA is used for Rx, disable the Rx Interrupt.
	 * (else DATAAVAILLABLE event would get triggered on first
	 * received data word)
	 * (Rx interrupt might be active for polling feature)
	 */
#if 0
	if (omap_readl(0x4A05A810)) {
		dev_err(hsi_ctrl->dev,
			"READ INTERRUPT IS PENDING DMA() but still disabling %0x\n",
			omap_readl(0x4A05A810));
	}
#endif
	hsi_driver_disable_read_interrupt(hsi_channel);

	/*
	 * NOTE: Gettting a free gdd logical channel and
	 * reserve it must be done atomicaly.
	 */
	hsi_channel->read_data.lch = lch;

	/* Sync is required for SSI but not for HSI */
	sync = hsi_sync_table[HSI_SYNC_READ][port - 1][channel];

	dest_addr = dma_map_single(hsi_ctrl->dev, data, count * 4,
				  DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(hsi_ctrl->dev, dest_addr))) {
		dev_err(hsi_ctrl->dev, "Failed to create DMA read mapping.\n");
		return -ENOMEM;
	}

	tmp = HSI_DST_SINGLE_ACCESS0 |
	    HSI_DST_MEMORY_PORT |
	    HSI_SRC_SINGLE_ACCESS0 |
	    HSI_SRC_PERIPHERAL_PORT | HSI_DATA_TYPE_S32;
	hsi_outw(tmp, base, HSI_GDD_CSDP_REG(lch));

	tmp = HSI_DST_AMODE_POSTINC | HSI_SRC_AMODE_CONST | sync;
	hsi_outw(tmp, base, HSI_GDD_CCR_REG(lch));

	hsi_outw((HSI_BLOCK_IE | HSI_TOUT_IE), base, HSI_GDD_CCIR_REG(lch));

	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev, "No valid FIFO id for DMA "
				"transfer from FIFO.\n");
			return -EFAULT;
		}
		/* HSI CSSA register takes a FIFO ID when copying from FIFO */
		hsi_outl(fifo, base, HSI_GDD_CSSA_REG(lch));
	} else{
		src_addr = hsi_ctrl->phy_base + HSI_HSR_BUFFER_CH_REG(port,
								channel);
		/* SSI CSSA register always takes a 32-bit address */
		hsi_outl(src_addr, base, HSI_GDD_CSSA_REG(lch));
	}

	/* HSI CDSA register takes a 32-bit address when copying to memory */
	/* SSI CDSA register always takes a 32-bit address */
	hsi_outl(dest_addr, base, HSI_GDD_CDSA_REG(lch));
	hsi_outw(count, base, HSI_GDD_CEN_REG(lch));

	/* TODO : Need to clean interrupt status here to avoid spurious int */

	hsi_outl_or(HSI_GDD_LCH(lch), base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outw_or(HSI_CCR_ENABLE, base, HSI_GDD_CCR_REG(lch));

	return 0;
}

/**
 * hsi_driver_cancel_write_dma - Cancel an ongoing GDD [DMA] write for the
 *				specified hsi channel.
 * @hsi_ch - pointer to the hsi_channel to cancel DMA write.
 *
 * hsi_controller lock must be held before calling this function.
 *
 * Return: -ENXIO : No DMA channel found for specified HSI channel
 *	   -ECANCELED : DMA cancel success, data not transfered to TX FIFO
 *	   0 : DMA transfer is already over, data already transfered to TX FIFO
 *
 * Note: whatever returned value, write callback will not be called after
 *       write cancel.
 */
int hsi_driver_cancel_write_dma(struct hsi_channel *hsi_ch)
{
	int lch = hsi_ch->write_data.lch;
	unsigned int port = hsi_ch->hsi_port->port_number;
	unsigned int channel = hsi_ch->channel_number;
	struct hsi_dev *hsi_ctrl = hsi_ch->hsi_port->hsi_controller;
	u16 ccr, gdd_csr;
	long buff_offset;
	u32 status_reg;
	dma_addr_t dma_h;
	size_t size;

	dev_err(&hsi_ch->dev->device, "hsi_driver_cancel_write_dma( "
				"channel %d\n", hsi_ch->channel_number);

	if (lch < 0) {
		dev_err(&hsi_ch->dev->device, "No DMA channel found for HSI "
				"channel %d\n", hsi_ch->channel_number);
		return -ENXIO;
	}
	ccr = hsi_inw(hsi_ctrl->base, HSI_GDD_CCR_REG(lch));
	if (!(ccr & HSI_CCR_ENABLE)) {
		dev_dbg(&hsi_ch->dev->device, "Write cancel on not "
			"enabled logical channel %d CCR REG 0x%04X\n",
			lch, ccr);
	}
	status_reg = hsi_inl(hsi_ctrl->base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG);
	status_reg &= hsi_inl(hsi_ctrl->base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outw_and(~HSI_CCR_ENABLE, hsi_ctrl->base, HSI_GDD_CCR_REG(lch));

	/* Clear CSR register by reading it, as it is cleared automaticaly */
	/* by HW after SW read. */
	gdd_csr = hsi_inw(hsi_ctrl->base, HSI_GDD_CSR_REG(lch));
	hsi_outl_and(~HSI_GDD_LCH(lch), hsi_ctrl->base,
			HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outl(HSI_GDD_LCH(lch), hsi_ctrl->base,
			HSI_SYS_GDD_MPU_IRQ_STATUS_REG);

	/* Unmap DMA region */
	dma_h = hsi_inl(hsi_ctrl->base, HSI_GDD_CSSA_REG(lch));
	size = hsi_inw(hsi_ctrl->base, HSI_GDD_CEN_REG(lch)) * 4;
	dma_unmap_single(hsi_ctrl->dev, dma_h, size, DMA_TO_DEVICE);

	buff_offset = hsi_hst_bufstate_f_reg(hsi_ctrl, port, channel);
	if (buff_offset >= 0)
		hsi_outl_and(~HSI_BUFSTATE_CHANNEL(channel), hsi_ctrl->base,
			     buff_offset);

	hsi_reset_ch_write(hsi_ch);
	return status_reg & HSI_GDD_LCH(lch) ? 0 : -ECANCELED;
}

/**
 * hsi_driver_cancel_read_dma - Cancel an ongoing GDD [DMA] read for the
 *				specified hsi channel.
 * @hsi_ch - pointer to the hsi_channel to cancel DMA read.
 *
 * hsi_controller lock must be held before calling this function.
 *
 * Return: -ENXIO : No DMA channel found for specified HSI channel
 *	   -ECANCELED : DMA cancel success, data not available at expected
 *	                address.
 *	   0 : DMA transfer is already over, data already available at
 *	       expected address.
 *
 * Note: whatever returned value, read callback will not be called after cancel.
 */
int hsi_driver_cancel_read_dma(struct hsi_channel *hsi_ch)
{
	int lch = hsi_ch->read_data.lch;
	struct hsi_dev *hsi_ctrl = hsi_ch->hsi_port->hsi_controller;
	u16 ccr, gdd_csr;
	u32 status_reg;
	dma_addr_t dma_h;
	size_t size;

	dev_err(&hsi_ch->dev->device, "hsi_driver_cancel_read_dma "
				"channel %d\n", hsi_ch->channel_number);

	/* Re-enable interrupts for polling if needed */
	if (hsi_ch->flags & HSI_CH_RX_POLL)
		hsi_driver_enable_read_interrupt(hsi_ch, NULL);

	if (lch < 0) {
		dev_err(&hsi_ch->dev->device, "No DMA channel found for HSI "
			"channel %d\n", hsi_ch->channel_number);
		return -ENXIO;
	}

	ccr = hsi_inw(hsi_ctrl->base, HSI_GDD_CCR_REG(lch));
	if (!(ccr & HSI_CCR_ENABLE)) {
		dev_dbg(&hsi_ch->dev->device, "Read cancel on not "
			"enabled logical channel %d CCR REG 0x%04X\n",
			lch, ccr);
	}

	status_reg = hsi_inl(hsi_ctrl->base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG);
	status_reg &= hsi_inl(hsi_ctrl->base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outw_and(~HSI_CCR_ENABLE, hsi_ctrl->base, HSI_GDD_CCR_REG(lch));

	/* Clear CSR register by reading it, as it is cleared automaticaly */
	/* by HW after SW read */
	gdd_csr = hsi_inw(hsi_ctrl->base, HSI_GDD_CSR_REG(lch));
	hsi_outl_and(~HSI_GDD_LCH(lch), hsi_ctrl->base,
		HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	hsi_outl(HSI_GDD_LCH(lch), hsi_ctrl->base,
		HSI_SYS_GDD_MPU_IRQ_STATUS_REG);

	/* Unmap DMA region - Access to the buffer is now safe */
	dma_h = hsi_inl(hsi_ctrl->base, HSI_GDD_CDSA_REG(lch));
	size = hsi_inw(hsi_ctrl->base, HSI_GDD_CEN_REG(lch)) * 4;
	dma_unmap_single(hsi_ctrl->dev, dma_h, size, DMA_FROM_DEVICE);

	hsi_reset_ch_read(hsi_ch);
	return status_reg & HSI_GDD_LCH(lch) ? 0 : -ECANCELED;
}

/**
 * hsi_get_info_from_gdd_lch - Retrieve channels information from DMA channel
 * @hsi_ctrl - HSI device control structure
 * @lch - DMA logical channel
 * @port - HSI port
 * @channel - HSI channel
 * @is_read_path - channel is used for reading
 *
 * Updates the port, channel and is_read_path parameters depending on the
 * lch DMA channel status.
 *
 * Return 0 on success and < 0 on error.
 */
int hsi_get_info_from_gdd_lch(struct hsi_dev *hsi_ctrl, unsigned int lch,
			      unsigned int *port, unsigned int *channel,
			      unsigned int *is_read_path)
{
	int i, j;
	int err = -1;

	for (i = 0; i < hsi_ctrl->max_p; i++)
		for (j = 0; j < hsi_ctrl->hsi_port[i].max_ch; j++)
			if (hsi_ctrl->hsi_port[i].
			    hsi_channel[j].read_data.lch == lch) {
				*is_read_path = 1;
				*port = i + 1;
				*channel = j;
				err = 0;
				goto get_info_bk;
			} else if (hsi_ctrl->hsi_port[i].
				   hsi_channel[j].write_data.lch == lch) {
				*is_read_path = 0;
				*port = i + 1;
				*channel = j;
				err = 0;
				goto get_info_bk;
			}
get_info_bk:
	return err;
}

static void do_hsi_gdd_lch(struct hsi_dev *hsi_ctrl, unsigned int gdd_lch)
{
	void __iomem *base = hsi_ctrl->base;
	struct platform_device *pdev = to_platform_device(hsi_ctrl->dev);
	struct hsi_channel *ch;
	unsigned int port;
	unsigned int channel;
	unsigned int is_read_path;
	u32 gdd_csr;
	dma_addr_t dma_h;
	size_t size;
	int fifo, fifo_words_avail;

	if (hsi_get_info_from_gdd_lch(hsi_ctrl, gdd_lch, &port, &channel,
				      &is_read_path) < 0) {
		dev_err(hsi_ctrl->dev, "Unable to match the DMA channel %d with"
			" an HSI channel\n", gdd_lch);
		return;
	} else {
		dev_dbg(hsi_ctrl->dev, "DMA event on gdd_lch=%d => port=%d, "
			"channel=%d, read=%d\n", gdd_lch, port, channel,
			is_read_path);
	}

	hsi_outl_and(~HSI_GDD_LCH(gdd_lch), base,
		     HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	/* Warning : CSR register is cleared automaticaly by HW after SW read */
	gdd_csr = hsi_inw(base, HSI_GDD_CSR_REG(gdd_lch));

	if (!(gdd_csr & HSI_CSR_TOUT)) {
		if (is_read_path) {	/* Read path */
			dma_h = hsi_inl(base, HSI_GDD_CDSA_REG(gdd_lch));
			size = hsi_inw(base, HSI_GDD_CEN_REG(gdd_lch)) * 4;
			dma_sync_single_for_cpu(hsi_ctrl->dev, dma_h, size,
						DMA_FROM_DEVICE);
			dma_unmap_single(hsi_ctrl->dev, dma_h, size,
					 DMA_FROM_DEVICE);
			ch = hsi_ctrl_get_ch(hsi_ctrl, port, channel);
			hsi_reset_ch_read(ch);

			dev_dbg(hsi_ctrl->dev, "Calling ch %d read callback "
					"(size %d).\n", channel,  size/4);
			spin_unlock(&hsi_ctrl->lock);
			ch->read_done(ch->dev, size / 4);
			spin_lock(&hsi_ctrl->lock);

			/* Check if FIFO is correctly emptied */
			if (hsi_driver_device_is_hsi(pdev)) {
				fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
				if (unlikely(fifo < 0)) {
					dev_err(hsi_ctrl->dev, "No valid FIFO "
						"id found for channel %d.\n",
						channel);
					return;
				}
				fifo_words_avail =
					hsi_get_rx_fifo_occupancy(hsi_ctrl,
								fifo);
				if (fifo_words_avail)
					dev_dbg(hsi_ctrl->dev,
						"WARNING: FIFO %d not empty "
						"after DMA copy, remaining "
						"%d/%d frames\n",
						fifo, fifo_words_avail,
						HSI_HSR_FIFO_SIZE);
			}
			/* Re-enable interrupts for polling if needed */
			if (ch->flags & HSI_CH_RX_POLL)
				hsi_driver_enable_read_interrupt(ch, NULL);
		} else {	/* Write path */
			dma_h = hsi_inl(base, HSI_GDD_CSSA_REG(gdd_lch));
			size = hsi_inw(base, HSI_GDD_CEN_REG(gdd_lch)) * 4;
			dma_unmap_single(hsi_ctrl->dev, dma_h, size,
					 DMA_TO_DEVICE);
			ch = hsi_ctrl_get_ch(hsi_ctrl, port, channel);
			hsi_reset_ch_write(ch);

			dev_dbg(hsi_ctrl->dev, "Calling ch %d write callback "
					"(size %d).\n", channel, size/4);
			spin_unlock(&hsi_ctrl->lock);
			ch->write_done(ch->dev, size / 4);
			spin_lock(&hsi_ctrl->lock);
		}
	} else {
		dev_err(hsi_ctrl->dev, "Time-out overflow Error on GDD transfer"
			" on gdd channel %d\n", gdd_lch);
		spin_unlock(&hsi_ctrl->lock);
		/* TODO : need to perform a DMA soft reset */
		hsi_port_event_handler(&hsi_ctrl->hsi_port[port - 1],
				       HSI_EVENT_ERROR, NULL);
		spin_lock(&hsi_ctrl->lock);
	}
}

static u32 hsi_process_dma_event(struct hsi_dev *hsi_ctrl)
{
	void __iomem *base = hsi_ctrl->base;
	unsigned int gdd_lch = 0;
	u32 status_reg = 0;
	u32 lch_served = 0;
	unsigned int gdd_max_count = hsi_ctrl->gdd_chan_count;

	status_reg = hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG);
	status_reg &= hsi_inl(base, HSI_SYS_GDD_MPU_IRQ_ENABLE_REG);
	if (!status_reg) {
		dev_dbg(hsi_ctrl->dev, "DMA : no event, exit.\n");
		return 0;
	}

	for (gdd_lch = 0; gdd_lch < gdd_max_count; gdd_lch++) {
		if (status_reg & HSI_GDD_LCH(gdd_lch)) {
			do_hsi_gdd_lch(hsi_ctrl, gdd_lch);
			lch_served |= HSI_GDD_LCH(gdd_lch);
		}
	}

	/* Acknowledge interrupt for DMA channel */
	hsi_outl(lch_served, base, HSI_SYS_GDD_MPU_IRQ_STATUS_REG);


	return status_reg;
}

static void do_hsi_gdd_tasklet(unsigned long device)
{
	struct hsi_dev *hsi_ctrl = (struct hsi_dev *)device;

	dev_dbg(hsi_ctrl->dev, "DMA Tasklet : clock_enabled=%d\n",
		hsi_ctrl->clock_enabled);

	spin_lock(&hsi_ctrl->lock);
	hsi_clocks_enable(hsi_ctrl->dev, __func__);
	hsi_ctrl->in_dma_tasklet = true;

	hsi_process_dma_event(hsi_ctrl);

	hsi_ctrl->in_dma_tasklet = false;
	hsi_clocks_disable(hsi_ctrl->dev, __func__);
	spin_unlock(&hsi_ctrl->lock);

	enable_irq(hsi_ctrl->gdd_irq);
}

static irqreturn_t hsi_gdd_mpu_handler(int irq, void *p)
{
	struct hsi_dev *hsi_ctrl = p;

	tasklet_hi_schedule(&hsi_ctrl->hsi_gdd_tasklet);

	/* Disable interrupt until Bottom Half has cleared the IRQ status */
	/* register */
	disable_irq_nosync(hsi_ctrl->gdd_irq);

	return IRQ_HANDLED;
}

int __init hsi_gdd_init(struct hsi_dev *hsi_ctrl, const char *irq_name)
{
	tasklet_init(&hsi_ctrl->hsi_gdd_tasklet, do_hsi_gdd_tasklet,
		     (unsigned long)hsi_ctrl);

	dev_info(hsi_ctrl->dev, "Registering IRQ %s (%d)\n",
					irq_name, hsi_ctrl->gdd_irq);

	if (request_irq(hsi_ctrl->gdd_irq, hsi_gdd_mpu_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_HIGH,
			irq_name, hsi_ctrl) < 0) {
		dev_err(hsi_ctrl->dev, "FAILED to request GDD IRQ %d\n",
			hsi_ctrl->gdd_irq);
		return -EBUSY;
	}

	return 0;
}

void hsi_gdd_exit(struct hsi_dev *hsi_ctrl)
{
	tasklet_kill(&hsi_ctrl->hsi_gdd_tasklet);
	free_irq(hsi_ctrl->gdd_irq, hsi_ctrl);
}
