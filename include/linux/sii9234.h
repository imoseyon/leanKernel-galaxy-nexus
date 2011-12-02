/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Author: Adam Hampson <ahampson@sta.samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _SII9234_H_
#define _SII9234_H_

#include <linux/i2c.h>

#define MHL_DEVCAP_DEVSTATE		0x0
#define MHL_DEVCAP_MHL_VERSION		0x1
#define MHL_DEVCAP_DEV_CAT		0x2
#define MHL_DEVCAP_ADOPTER_ID_H		0x3
#define MHL_DEVCAP_ADOPTER_ID_L		0x4
#define MHL_DEVCAP_VID_LINK_MODE	0x5
#define MHL_DEVCAP_AUD_LINK_MODE	0x6
#define MHL_DEVCAP_VIDEO_TYPE		0x7
#define MHL_DEVCAP_LOG_DEV_MAP		0x8
#define MHL_DEVCAP_BANDWIDTH		0x9
#define MHL_DEVCAP_FEATURE_FLAG		0xa
#define MHL_DEVCAP_DEVICE_ID_H		0xb
#define MHL_DEVCAP_DEVICE_ID_L		0xc
#define MHL_DEVCAP_SCRATHPAD_SIZE	0xd
#define MHL_DEVCAP_INT_STAT_SIZE	0xe
#define MHL_DEVCAP_RESERVED		0xf

struct sii9234_platform_data {
	int prio;
	void (*enable)(bool enable);
	void (*power)(int on);
	void (*enable_vbus)(bool enable);
	void (*connect)(bool connected, u8 *devcap);
	struct i2c_client *mhl_tx_client;
	struct i2c_client *tpi_client;
	struct i2c_client *hdmi_rx_client;
	struct i2c_client *cbus_client;
};

#endif /* _SII9234_H_ */
