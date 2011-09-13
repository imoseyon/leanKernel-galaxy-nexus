/*
 * hsi_char.h
 *
 * HSI character driver public declaration header file.
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

#ifndef HSI_CHAR_H
#define HSI_CHAR_H

#define HSI_CHAR_BASE		'S'
#define CS_IOW(num, dtype)	_IOW(HSI_CHAR_BASE, num, dtype)
#define CS_IOR(num, dtype)	_IOR(HSI_CHAR_BASE, num, dtype)
#define CS_IOWR(num, dtype)	_IOWR(HSI_CHAR_BASE, num, dtype)
#define CS_IO(num)		_IO(HSI_CHAR_BASE, num)

#define CS_SEND_BREAK		CS_IO(1)
#define CS_FLUSH_RX		CS_IO(2)
#define CS_FLUSH_TX		CS_IO(3)
#define CS_BOOTSTRAP		CS_IO(4)
#define CS_SET_ACWAKELINE	CS_IOW(5, unsigned int)
#define CS_GET_ACWAKELINE	CS_IOR(6, unsigned int)
#define CS_SET_RX		CS_IOW(7, struct hsi_rx_config)
#define CS_GET_RX		CS_IOW(8, struct hsi_rx_config)
#define CS_SET_TX		CS_IOW(9, struct hsi_tx_config)
#define CS_GET_TX		CS_IOW(10, struct hsi_tx_config)
#define CS_SW_RESET		CS_IO(11)
#define CS_GET_FIFO_OCCUPANCY	CS_IOR(12, size_t)
#define CS_GET_CAWAKELINE	CS_IOR(13, unsigned int)
#define CS_SET_WAKE_RX_3WIRES_MODE	CS_IOR(14, unsigned int)

#define HSI_MODE_SLEEP		0
#define HSI_MODE_STREAM		1
#define HSI_MODE_FRAME		2

#define HSI_ARBMODE_RR		0
#define HSI_ARBMODE_PRIO	1

#define WAKE_UP			1
#define WAKE_DOWN		0

/**
 * struct hsi_tx_config - HSI TX configuration data
 * @mode: Bit transmission mode
 * @flow: Data flow type
 * @frame_size: frame payload size
 * @channels: Number of active channels
 * @divisor: Transmission bit rate divisor
 * @arb_mode: Arbitration type for the transmit FIFOs
 */
struct hsi_tx_config {
	__u32 mode;       /* Stream:1, Frame:2 */
	__u32 flow;       /* Synchronized:0, Pipelined:1. No Realtime support */
	__u32 frame_size; /* HSI: 31,  SSI: <= 31 */
	__u32 channels;   /* 1, 2, 4, 8, 16 (HSI only) */
	__u32 divisor;    /* For HSI: <= 0xFF, for SSI: <= 0x7F */
	__u32 arb_mode;   /* Round Robin: 0, Priority: 1 */
};

/**
 * struct hsi_rx_config - HSI RX configuration data
 * @mode: Bit transmission mode
 * @flow: Data flow type
 * @frame_size: frame payload size
 * @channels: Number of active channels
 * @divisor: Transmission bit rate divisor
 *           Auto mode:ON:0x1000, OFF(SSI):0x1001)
 *	     Normal range : HSI <= 255, SSI <= 127
 * @counters: Counters settings for error generation.
 *		Use HSI_HSR_COMBINE_COUNTERS for formatting the register value
 */
struct hsi_rx_config {
	__u32 mode;       /* Stream:1, Frame:2 */
	__u32 flow;       /* Synchronized:0, Pipelined:1. No Realtime support */
	__u32 frame_size; /* HSI: 31,  SSI: <= 31 */
	__u32 channels;   /* 1, 2, 4, 8, 16(HSI only) */
	__u32 divisor;    /* Normal range : HSI <= 255, SSI <= 127 */
	__u32 counters;   /* HSI: FB[31..24], TB[23..20], FT[19..0] */
			  /* SSI: FT[8..0] */
};


#endif /* HSI_CHAR_H */
