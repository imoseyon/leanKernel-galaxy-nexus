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

struct hsi_event {
	unsigned int event;
	u32 *data;
	unsigned int count;
};

int if_hsi_init(unsigned int port, unsigned int *channels_map);
int if_hsi_exit(void);

int if_hsi_start(int ch);
void if_hsi_stop(int ch);

void if_hsi_send_break(int ch);
void if_hsi_flush_rx(int ch);
void if_hsi_flush_tx(int ch);
void if_hsi_bootstrap(int ch);
void if_hsi_set_wakeline(int ch, unsigned int state);
void if_hsi_get_wakeline(int ch, unsigned int *state);
int if_hsi_set_rx(int ch, struct hsi_rx_config *cfg);
void if_hsi_get_rx(int ch, struct hsi_rx_config *cfg);
int if_hsi_set_tx(int ch, struct hsi_tx_config *cfg);
void if_hsi_get_tx(int ch, struct hsi_tx_config *cfg);
void if_hsi_sw_reset(int ch);
void if_hsi_get_fifo_occupancy(int ch, size_t *occ);

int if_hsi_read(int ch, u32 *data, unsigned int count);
int if_hsi_poll(int ch);
int if_hsi_write(int ch, u32 *data, unsigned int count);

void if_hsi_cancel_read(int ch);
void if_hsi_cancel_write(int ch);

#endif /* _HSI_IF_H */
