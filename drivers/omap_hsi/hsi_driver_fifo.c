/*
 * hsi_driver_fifo.c
 *
 * Implements HSI module fifo management.
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include "hsi_driver.h"

/**
 * hsi_fifo_get_id - Get fifo index corresponding to (port, channel)
 * @hsi_ctrl - HSI controler data
 * @channel - channel used
 * @port - HSI port used. Range [1, 2]
 *
 * Returns the fifo index associated to the provided (port, channel).
 * Notes: 1) The fifo <=> (port, channel) correspondance depends on the selected
 * SW strategy for channels mapping (fifo management).
 * 2) the mapping is identical for Read and Write path.
 * This exclusively applies to HSI devices.
 */
int hsi_fifo_get_id(struct hsi_dev *hsi_ctrl, unsigned int channel,
		    unsigned int port)
{
	int fifo_index = 0;
	int err = 0;
	int fifo_port; /* Range [1, 2] */

	if (unlikely((channel >= HSI_CHANNELS_MAX) || (port < 1) ||
						      (port > 2))) {
		err = -EINVAL;
		goto fifo_id_bk;
	}

	if ((hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_ALL_PORT1) ||
	    (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_ALL_PORT2)) {
		fifo_port = (hsi_ctrl->fifo_mapping_strategy ==
					HSI_FIFO_MAPPING_ALL_PORT1) ? 1 : 2;
		if (unlikely(port != fifo_port)) {
			err = -EINVAL;
			goto fifo_id_bk;
		} else {
			fifo_index = channel;
		}
	} else if (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_SSI) {
		if (unlikely(channel >= 8)) {
			err = -EINVAL;
			goto fifo_id_bk;
		} else {
			fifo_index = channel + 8 * (port - 1);
		}
	} else {
		err = -EPERM;
		goto fifo_id_bk;
	}

fifo_id_bk:
	if (unlikely(err < 0)) {
		fifo_index = err;
		dev_err(hsi_ctrl->dev, "Cannot map a FIFO to the requested "
			"params: channel:%d, port:%d; ERR=%d\n", channel, port,
			err);
	}

	return fifo_index;
}

/**
 * hsi_fifo_get_chan - Get (port, channel) from a fifo index
 * @hsi_ctrl - HSI controler data
 * @fifo - HSI fifo used (0..HSI_HST_FIFO_COUNT)
 * @channel - related channel if any (0..)
 * @port - related port if any (1..2)
 *
 * Returns 0 in case of success, and errocode (< 0) else
 * Notes: 1) The fifo <=> (port, channel) correspondance depends on the selected
 * SW strategy for channels mapping (fifo management).
 * 2) the mapping is identical for Read and Write path.
 * This exclusively applies to HSI devices.
 */
int hsi_fifo_get_chan(struct hsi_dev *hsi_ctrl, unsigned int fifo,
		      unsigned int *channel, unsigned int *port)
{
	int err = 0;

	if (unlikely(fifo >= HSI_HST_FIFO_COUNT)) {
		err = -EINVAL;
		goto fifo_id_bk;
	}

	if (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_ALL_PORT1) {
		*channel = fifo;
		*port = 1;
	} else if (hsi_ctrl->fifo_mapping_strategy ==
						HSI_FIFO_MAPPING_ALL_PORT2) {
		*channel = fifo;
		*port = 2;
	} else if (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_SSI) {
		if (fifo < 8) {
			*channel = fifo;
			*port = 1;
		} else {
			*channel = fifo - 8;
			*port = 2;
		}
	} else {
		err = -EPERM;
		goto fifo_id_bk;
	}

fifo_id_bk:
	if (unlikely(err < 0))
		dev_err(hsi_ctrl->dev, "Cannot map a channel / port to the "
			"requested params: fifo:%d; ERR=%d\n", fifo, err);

	return err;
}

/**
 * hsi_fifo_mapping - Configures the HSI FIFO mapping registers.
 * @hsi_ctrl - HSI controler data
 * @mtype - mapping strategy
 *
 * Returns 0 in case of success, and error code (< 0) else
 * Configures the HSI FIFO mapping registers. Several mapping strategies are
 * proposed.
 * Note: The mapping is identical for Read and Write path.
 * This exclusively applies to HSI devices.
 */
int hsi_fifo_mapping(struct hsi_dev *hsi_ctrl, unsigned int mtype)
{
	int err = 0;
	void __iomem *base = hsi_ctrl->base;
	int i;
	unsigned int channel, port;

	if ((mtype == HSI_FIFO_MAPPING_ALL_PORT1) ||
	    (mtype == HSI_FIFO_MAPPING_ALL_PORT2)) {
		port = (mtype == HSI_FIFO_MAPPING_ALL_PORT1) ? 0 : 1;
		channel = 0;
		for (i = 0; i < HSI_HST_FIFO_COUNT; i++) {
			hsi_outl(HSI_MAPPING_ENABLE |
				 (channel << HSI_MAPPING_CH_NUMBER_OFFSET) |
				 (port << HSI_MAPPING_PORT_NUMBER_OFFSET) |
				 HSI_HST_MAPPING_THRESH_VALUE,
				 base, HSI_HST_MAPPING_FIFO_REG(i));
			hsi_outl(HSI_MAPPING_ENABLE |
				 (channel << HSI_MAPPING_CH_NUMBER_OFFSET) |
				 (port << HSI_MAPPING_PORT_NUMBER_OFFSET),
				 base, HSI_HSR_MAPPING_FIFO_REG(i));
			channel++;
		}

		if (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_UNDEF)
			dev_dbg(hsi_ctrl->dev, "Fifo mapping : All FIFOs for "
						"Port %d\n", port + 1);
	} else if (mtype == HSI_FIFO_MAPPING_SSI) {
		channel = 0;
		port = 0;
		for (i = 0; i < HSI_HST_FIFO_COUNT; i++) {
			hsi_outl(HSI_MAPPING_ENABLE |
				 (channel << HSI_MAPPING_CH_NUMBER_OFFSET) |
				 (port << HSI_MAPPING_PORT_NUMBER_OFFSET) |
				 HSI_HST_MAPPING_THRESH_VALUE,
				 base, HSI_HST_MAPPING_FIFO_REG(i));
			hsi_outl(HSI_MAPPING_ENABLE |
				 (channel << HSI_MAPPING_CH_NUMBER_OFFSET) |
				 (port << HSI_MAPPING_PORT_NUMBER_OFFSET),
				 base, HSI_HSR_MAPPING_FIFO_REG(i));
			channel++;
			if (channel == 8) {
				channel = 0;
				port = 1;
			}
		}

		if (hsi_ctrl->fifo_mapping_strategy == HSI_FIFO_MAPPING_UNDEF)
			dev_dbg(hsi_ctrl->dev, "Fifo mapping : 8 FIFOs per Port"
						" (SSI compatible mode)\n");
	} else {
		dev_err(hsi_ctrl->dev, "Bad Fifo strategy request : %d\n",
			mtype);
		err = -EINVAL;
	}

	hsi_ctrl->fifo_mapping_strategy = mtype;

	return err;
}

/**
 * hsi_hst_bufstate_f_reg - Return the proper HSI_HST_BUFSTATE register offset
 * @hsi_ctrl - HSI controler data
 * @port - HSI port used
 * @channel - channel used
 *
 * Returns the HSI_HST_BUFSTATE register offset
 * Note: indexing of BUFSTATE registers is different on SSI and HSI:
 * On SSI: it is linked to the ports
 * On HSI: it is linked to the FIFOs (and depend on the SW strategy)
 */
long hsi_hst_bufstate_f_reg(struct hsi_dev *hsi_ctrl,
			    unsigned int port, unsigned int channel)
{
	int fifo;
	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev,
				"hsi_hst_bufstate_f_reg  ERROR : %d\n", fifo);
			return fifo;
		} else
			return HSI_HST_BUFSTATE_FIFO_REG(fifo);
	} else {
		return HSI_HST_BUFSTATE_REG(port);
	}
}

/**
 * hsi_hsr_bufstate_f_reg - Return the proper HSI_HSR_BUFSTATE register offset
 * @hsi_ctrl - HSI controler data
 * @port - HSI port used
 * @channel - channel used
 *
 * Returns the HSI_HSR_BUFSTATE register offset
 * Note: indexing of BUFSTATE registers is different on SSI and HSI:
 * On SSI: it is linked to the ports
 * On HSI: it is linked to the FIFOs (and depend on the SW strategy)
 */
long hsi_hsr_bufstate_f_reg(struct hsi_dev *hsi_ctrl,
			    unsigned int port, unsigned int channel)
{
	int fifo;
	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev,
				"hsi_hsr_bufstate_f_reg  ERROR : %d\n", fifo);
			return fifo;
		} else
			return HSI_HSR_BUFSTATE_FIFO_REG(fifo);
	} else {
		return HSI_HSR_BUFSTATE_REG(port);
	}
}

/**
 * hsi_hst_buffer_f_reg - Return the proper HSI_HST_BUFFER register offset
 * @hsi_ctrl - HSI controler data
 * @port - HSI port used
 * @channel - channel used
 *
 * Returns the HSI_HST_BUFFER register offset
 * Note: indexing of BUFFER registers is different on SSI and HSI:
 * On SSI: it is linked to the ports
 * On HSI: it is linked to the FIFOs (and depend on the SW strategy)
 */
long hsi_hst_buffer_reg(struct hsi_dev *hsi_ctrl,
			unsigned int port, unsigned int channel)
{
	int fifo;
	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev,
				"hsi_hst_bufstate_f_reg   ERROR : %d\n", fifo);
			return fifo;
		} else
			return HSI_HST_BUFFER_FIFO_REG(fifo);
	} else {
		return HSI_HST_BUFFER_CH_REG(port, channel);
	}
}

/**
 * hsi_hsr_buffer_f_reg - Return the proper HSI_HSR_BUFFER register offset
 * @hsi_ctrl - HSI controler data
 * @port - HSI port used
 * @channel - channel used
 *
 * Returns the HSI_HSR_BUFFER register offset
 * Note: indexing of BUFFER registers is different on SSI and HSI:
 * On SSI: it is linked to the ports
 * On HSI: it is linked to the FIFOs (and depend on the SW strategy)
 */
long hsi_hsr_buffer_reg(struct hsi_dev *hsi_ctrl,
			unsigned int port, unsigned int channel)
{
	int fifo;
	if (hsi_driver_device_is_hsi(to_platform_device(hsi_ctrl->dev))) {
		fifo = hsi_fifo_get_id(hsi_ctrl, channel, port);
		if (unlikely(fifo < 0)) {
			dev_err(hsi_ctrl->dev,
				"hsi_hsr_bufstate_f_reg  ERROR : %d\n", fifo);
			return fifo;
		} else
			return HSI_HSR_BUFFER_FIFO_REG(fifo);
	} else {
		return HSI_HSR_BUFFER_CH_REG(port, channel);
	}
}

/**
 * hsi_get_rx_fifo_occupancy - Return the size of data remaining
 *				in the given FIFO
 * @hsi_ctrl - HSI controler data
 * @fifo - FIFO to look at
 *
 * Returns the number of frames (32bits) remaining in the FIFO
 */
u8 hsi_get_rx_fifo_occupancy(struct hsi_dev *hsi_ctrl, u8 fifo)
{
	void __iomem *base = hsi_ctrl->base;
	int hsr_mapping, mapping_words;

	if (unlikely(fifo < 0)) {
		dev_err(hsi_ctrl->dev, "Invalid FIFO id %d.\n", fifo);
		return 0;
	}

	hsr_mapping = hsi_inl(base, HSI_HSR_MAPPING_FIFO_REG(fifo));
	mapping_words = (hsr_mapping >> HSI_HST_MAPPING_THRESH_OFFSET) & 0xF;
	return mapping_words;
}

