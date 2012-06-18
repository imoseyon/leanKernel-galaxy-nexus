/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Authors: Adam Hampson <ahampson@sta.samsung.com>
 *          Erik Gilling <konkers@android.com>
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sii9234.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/usb/otg_id.h>

#define T_SRC_VBUS_CBUS_TO_STABLE	200
#define T_SRC_WAKE_PULSE_WIDTH_1	19
#define T_SRC_WAKE_PULSE_WIDTH_2	60
#define T_SRC_WAKE_TO_DISCOVER		500
#define T_SRC_VBUS_CBUS_T0_STABLE	500
#define T_SRC_CBUS_FLOAT		50
#define T_WAIT_TIMEOUT_RSEN_INT		200
#define T_SRC_RXSENSE_DEGLITCH		110

/* MHL feature flags */
#define MHL_FEATURE_FLAG_RCP_SUPPORT	(1 << 0)

/* MHL TX Addr 0x72 Registers */
#define MHL_TX_IDL_REG			0x02
#define MHL_TX_IDH_REG			0x03
#define MHL_TX_REV_REG			0x04
#define MHL_TX_SRST			0x05
#define MHL_TX_INTR1_REG		0x71
#define MHL_TX_INTR2_REG		0x72	/* Not Documented */
#define MHL_TX_INTR3_REG		0x73	/* Not Documented */
#define MHL_TX_INTR4_REG		0x74
#define MHL_TX_INTR1_ENABLE_REG		0x75
#define MHL_TX_INTR2_ENABLE_REG		0x76	/* Not Documented */
#define MHL_TX_INTR3_ENABLE_REG		0x77	/* Not Documented */
#define MHL_TX_INTR4_ENABLE_REG		0x78

#define MHL_TX_INT_CTRL_REG		0x79
#define   INTR_POLARITY		(1 << 1)
#define   INTR_OPEN_DRAIN	(1 << 2)
#define   HPD_OUT_OVR_EN	(1 << 4)
#define   HPD_OUT_OVR_VAL	(1 << 5)
#define   HPD_OUT_OPEN_DRAIN	(1 << 6)

#define MHL_TX_TMDS_CCTRL		0x80

#define MHL_TX_DISC_CTRL1_REG		0x90
#define MHL_TX_DISC_CTRL2_REG		0x91
#define MHL_TX_DISC_CTRL3_REG		0x92
#define MHL_TX_DISC_CTRL4_REG		0x93	/* Not Documented */

/* There doesn't seem to be any documentation for CTRL5 but it looks like
 * it is some sort of pull up control register
 */
#define MHL_TX_DISC_CTRL5_REG		0x94
#define MHL_TX_DISC_CTRL6_REG		0x95
#define MHL_TX_DISC_CTRL7_REG		0x96
#define MHL_TX_DISC_CTRL8_REG		0x97	/* Not Documented */
#define MHL_TX_STAT1_REG		0x98	/* Not Documented */
#define MHL_TX_STAT2_REG		0x99

#define MHL_TX_MHLTX_CTL1_REG		0xA0
#define MHL_TX_MHLTX_CTL2_REG		0xA1
#define MHL_TX_MHLTX_CTL4_REG		0xA3
#define MHL_TX_MHLTX_CTL6_REG		0xA5
#define MHL_TX_MHLTX_CTL7_REG		0xA6

/* MHL TX SYS STAT Registers
 * Not Documented, mentioned only in reference of RSEN
 */
#define MHL_TX_SYSSTAT_REG		0x09

/* MHL TX SYS STAT Register Bits */
#define RSEN_STATUS			(1<<2)

/* MHL TX INTR4 Register Bits */
#define RGND_READY_INT			(1<<6)
#define VBUS_LOW_INT			(1<<5)
#define CBUS_LKOUT_INT			(1<<4)
#define MHL_DISC_FAIL_INT		(1<<3)
#define MHL_EST_INT			(1<<2)

/* MHL TX INTR4_ENABLE 0x78 Register Bits */
#define RGND_READY_MASK			(1<<6)
#define CBUS_LKOUT_MASK			(1<<4)
#define MHL_DISC_FAIL_MASK		(1<<3)
#define MHL_EST_MASK			(1<<2)

/* MHL TX INTR1 Register Bits*/
#define HPD_CHANGE_INT			(1<<6)
#define RSEN_CHANGE_INT			(1<<5)

/* MHL TX INTR1_ENABLE 0x75 Register Bits*/
#define HPD_CHANGE_INT_MASK		(1<<6)
#define RSEN_CHANGE_INT_MASK		(1<<5)

#define CBUS_CONFIG_REG			0x07

#define CBUS_INT_STATUS_1_REG		0x08
#define CBUS_INT_1_MASK			0x09

#define CBUS_MSC_COMMAND_START		0x12
#define	  START_MSC_RESERVED	(1 << 0)
#define   START_MSC_MSG		(1 << 1)
#define   START_READ_DEVCAP	(1 << 2)
#define   START_WRITE_STAT_INT	(1 << 3)
#define   START_WRITE_BURST	(1 << 4)

#define CBUS_MSC_RAP_POLL		0x00
#define CBUS_MSC_RAP_CONTENT_ON		0x10
#define CBUS_MSC_RAP_CONTENT_OFF	0x11
#define CBUS_MSC_OFFSET_REG		0x13
#define CBUS_MSC_FIRST_DATA_OUT		0x14
#define CBUS_MSC_SECOND_DATA_OUT	0x15
#define CBUS_MSC_FIRST_DATA_IN		0x16
#define CBUS_MSC_MSG_CMD_IN		0x18
#define CBUS_MSC_MSG_DATA_IN		0x19
#define CBUS_INT_STATUS_2_REG		0x1E
#define CBUS_INT_2_MASK			0x1F
#define CBUS_LINK_CONTROL_2_REG		0x31

#define CBUS_INT_STATUS_2_REG		0x1E

/* MHL Interrupt Registers */
#define CBUS_MHL_INTR_REG_0		0xA0
#define MHL_INT_DCAP_CHG	(1<<0)
#define MHL_INT_DSCR_CHG	(1<<1)
#define MHL_INT_REQ_WRT		(1<<2)
#define MHL_INT_GRT_WRT		(1<<3)

#define CBUS_MHL_INTR_REG_1		0xA1
#define   MHL_INT_EDID_CHG	(1<<1)

#define CBUS_MHL_INTR_REG_2		0xA2
#define CBUS_MHL_INTR_REG_3		0xA3

/* MHL Status Registers */
#define CBUS_MHL_STATUS_REG_0		0xB0
#define   MHL_STATUS_DCAP_READY	(1<<0)

#define CBUS_MHL_STATUS_REG_1		0xB1
#define MHL_STATUS_CLK_NORMAL		((1<<0) | (1<<1))
#define MHL_STATUS_CLK_PACKEDPIXEL	(1<<1)
#define MHL_STATUS_PATH_ENABLED		(1<<3)
#define MHL_STATUS_MUTED		(1<<4)

#define CBUS_MHL_STATUS_REG_2		0xB2
#define CBUS_MHL_STATUS_REG_3		0xB3

/* Device interrupt register offset of connected device */
#define CBUS_MHL_INTR_OFFSET_0		0x20 /* RCHANGE_INT */
#define CBUS_MHL_INTR_OFFSET_1		0x21 /* DCHANGE_INT */
#define CBUS_MHL_INTR_OFFSET_2		0x22
#define CBUS_MHL_INTR_OFFSET_3		0x23

/* Device status register offset of connected device */
#define CBUS_MHL_STATUS_OFFSET_0	0x30 /* CONNECTED_RDY */
#define CBUS_MHL_STATUS_OFFSET_1	0x31 /* LINK_MODE */
#define CBUS_MHL_STATUS_OFFSET_2	0x32
#define CBUS_MHL_STATUS_OFFSET_3	0x33

/* CBUS INTR1 STATUS Register bits */
#define MSC_RESP_ABORT			(1<<6)
#define MSC_REQ_ABORT			(1<<5)
#define MSC_REQ_DONE			(1<<4)
#define MSC_MSG_RECD			(1<<3)
#define CBUS_DDC_ABORT			(1<<2)

/* CBUS INTR1 STATUS 0x09 Enable Mask*/
#define MSC_RESP_ABORT_MASK		(1<<6)
#define MSC_REQ_ABORT_MASK		(1<<5)
#define MSC_REQ_DONE_MASK		(1<<4)
#define MSC_MSG_RECD_MASK		(1<<3)
#define CBUS_DDC_ABORT_MASK		(1<<2)

/* CBUS INTR2 STATUS Register bits */
#define WRT_STAT_RECD			(1<<3)
#define SET_INT_RECD			(1<<2)
#define WRT_BURST_RECD			(1<<0)

/* CBUS INTR2 STATUS 0x1F Enable Mask*/
#define WRT_STAT_RECD_MASK		(1<<3)
#define SET_INT_RECD_MASK		(1<<2)
#define WRT_BURST_RECD_MASK		(1<<0)

/* CBUS Control Registers*/
/* Retry count for all MSC commands*/
#define MSC_RETRY_FAIL_LIM_REG		0x1D

/* reason for MSC_REQ_ABORT interrupt on CBUS */
#define MSC_REQ_ABORT_REASON_REG	0x0D

#define MSC_RESP_ABORT_REASON_REG	0x0E

/* MSC Requestor Abort Reason Register bits*/
#define ABORT_BY_PEER			(1<<7)
#define UNDEF_CMD			(1<<3)
#define TIMEOUT				(1<<2)
#define PROTO_ERROR			(1<<1)
#define MAX_FAIL			(1<<0)

/* MSC Responder Abort Reason Register bits*/
#define ABORT_BY_PEER			(1<<7)
#define UNDEF_CMD			(1<<3)
#define TIMEOUT				(1<<2)

/* Set HPD came from Downstream, not documented */
#define SET_HPD_DOWNSTREAM		(1<<6)

/* MHL TX DISC1 Register Bits */
#define DISC_EN				(1<<0)

/* MHL TX DISC2 Register Bits */
#define SKIP_GND			(1<<6)
#define ATT_THRESH_SHIFT		0x04
#define ATT_THRESH_MASK			(0x03 << ATT_THRESH_SHIFT)
#define USB_D_OEN			(1<<3)
#define DEGLITCH_TIME_MASK		0x07
#define DEGLITCH_TIME_2MS		0
#define DEGLITCH_TIME_4MS		1
#define DEGLITCH_TIME_8MS		2
#define DEGLITCH_TIME_16MS		3
#define DEGLITCH_TIME_40MS		4
#define DEGLITCH_TIME_50MS		5
#define DEGLITCH_TIME_60MS		6
#define DEGLITCH_TIME_128MS		7

#define DISC_CTRL3_COMM_IMME		(1<<7)
#define DISC_CTRL3_FORCE_MHL		(1<<6)
#define DISC_CTRL3_FORCE_USB		(1<<4)
#define DISC_CTRL3_USB_EN		(1<<3)

/* MHL TX DISC4 0x93 Register Bits: undocumented */
#define CBUS_DISC_PUP_SEL_SHIFT		6
#define CBUS_DISC_PUP_SEL_MASK		(3<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_DISC_PUP_SEL_10K		(2<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_DISC_PUP_SEL_OPEN		(0<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_IDLE_PUP_SEL_SHIFT		4
#define CBUS_IDLE_PUP_SEL_MASK		(3<<CBUS_IDLE_PUP_SEL_SHIFT)
#define CBUS_IDLE_PUP_SEL_OPEN		(0<<CBUS_IDLE_PUP_SEL_SHIFT)

/* MHL TX DISC5 0x94 Register Bits */
#define CBUS_MHL_PUP_SEL_MASK		0x03	/* Not Documented */
#define CBUS_MHL_PUP_SEL_5K		0x01	/* Not Documented */
#define CBUS_MHL_PUP_SEL_OPEN		0x00

/* MHL TX DISC6 0x95 Register Bits */
#define USB_D_OVR			(1<<7)
#define USB_ID_OVR			(1<<6)
#define DVRFLT_SEL			(1<<5)
#define BLOCK_RGND_INT			(1<<4)
#define SKIP_DEG			(1<<3)
#define CI2CA_POL			(1<<2)
#define CI2CA_WKUP			(1<<1)
#define SINGLE_ATT			(1<<0)

/* MHL TX DISC7 0x96 Register Bits
 *
 * Bits 7 and 6 are labeled as reserved but seem to be related to toggling
 * the CBUS signal when generating the wake pulse sequence.
 */
#define USB_D_ODN			(1<<5)
#define VBUS_CHECK			(1<<2)
#define RGND_INTP_MASK			0x03
#define RGND_INTP_OPEN			0
#define RGND_INTP_2K			1
#define RGND_INTP_1K			2
#define RGND_INTP_SHORT			3

/* TPI Addr 0x7A Registers */
#define TPI_DPD_REG			0x3D

#define TPI_PD_TMDS			(1<<5)
#define TPI_PD_OSC_EN			(1<<4)
#define TPI_TCLK_PHASE			(1<<3)
#define TPI_PD_IDCK			(1<<2)
#define TPI_PD_OSC			(1<<1)
#define TPI_PD				(1<<0)



/* HDMI RX Registers */
#define HDMI_RX_TMDS0_CCTRL1_REG	0x10
#define HDMI_RX_TMDS_CLK_EN_REG		0x11
#define HDMI_RX_TMDS_CH_EN_REG		0x12
#define HDMI_RX_PLL_CALREFSEL_REG	0x17
#define HDMI_RX_PLL_VCOCAL_REG		0x1A
#define HDMI_RX_EQ_DATA0_REG		0x22
#define HDMI_RX_EQ_DATA1_REG		0x23
#define HDMI_RX_EQ_DATA2_REG		0x24
#define HDMI_RX_EQ_DATA3_REG		0x25
#define HDMI_RX_EQ_DATA4_REG		0x26
#define HDMI_RX_TMDS_ZONE_CTRL_REG	0x4C
#define HDMI_RX_TMDS_MODE_CTRL_REG	0x4D

enum rgnd_state {
	RGND_UNKNOWN = 0,
	RGND_OPEN,
	RGND_1K,
	RGND_2K,
	RGND_SHORT
};

enum mhl_state {
	STATE_DISCONNECTED = 0,
	STATE_DISCOVERY_FAILED,
	STATE_CBUS_LOCKOUT,
	STATE_ESTABLISHED,
	STATE_DISCONNECTING,
};

enum cbus_command {
	IDLE =			0x00,
	ACK =			0x33,
	NACK =			0x34,
	ABORT =			0x35,
	WRITE_STAT =		0x60 | 0x80,
	SET_INT =		0x60,
	READ_DEVCAP =		0x61,
	GET_STATE =		0x62,
	GET_VENDOR_ID =		0x63,
	SET_HPD =		0x64,
	CLR_HPD =		0x65,
	SET_CAP_ID =		0x66,
	GET_CAP_ID =		0x67,
	MSC_MSG =		0x68,
	GET_SC1_ERR_CODE =	0x69,
	GET_DDC_ERR_CODE =	0x6A,
	GET_MSC_ERR_CODE =	0x6B,
	WRITE_BURST =		0x6C,
	GET_SC3_ERR_CODE =	0x6D,
};

enum msc_subcommand {
	/* MSC_MSG Sub-Command codes */
	MSG_RCP =	0x10,
	MSG_RCPK =	0x11,
	MSG_RCPE =	0x12,
	MSG_RAP =	0x20,
	MSG_RAPK =	0x21,
};

static inline bool mhl_state_is_error(enum mhl_state state)
{
	return state == STATE_DISCOVERY_FAILED ||
		state == STATE_CBUS_LOCKOUT;
}

struct msc_data {

	enum cbus_command cmd;		/* cbus command type */
	u8 offset;			/* for MSC_MSG,stores msc_subcommand */
	u8 data;
	struct completion *cvar;	/* optional completion signaled when
					   event is handled */
	int *ret;			/* optional return value */
	struct list_head list;
};

static const u16 sii9234_rcp_def_keymap[] = {
	KEY_SELECT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UNKNOWN,	/* right-up */
	KEY_UNKNOWN,	/* right-down */
	KEY_UNKNOWN,	/* left-up */
	KEY_UNKNOWN,	/* left-down */
	KEY_MENU,
	KEY_UNKNOWN,	/* setup */
	KEY_UNKNOWN,	/* contents */
	KEY_UNKNOWN,	/* favorite */
	KEY_EXIT,
	KEY_RESERVED,	/* 0x0e */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x1F */
	KEY_NUMERIC_0,
	KEY_NUMERIC_1,
	KEY_NUMERIC_2,
	KEY_NUMERIC_3,
	KEY_NUMERIC_4,
	KEY_NUMERIC_5,
	KEY_NUMERIC_6,
	KEY_NUMERIC_7,
	KEY_NUMERIC_8,
	KEY_NUMERIC_9,
	KEY_DOT,
	KEY_ENTER,
	KEY_CLEAR,
	KEY_RESERVED,	/* 0x2D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x2F */
	KEY_UNKNOWN,	/* channel up */
	KEY_UNKNOWN,	/* channel down */
	KEY_UNKNOWN,	/* previous channel */
	KEY_UNKNOWN,	/* sound select */
	KEY_UNKNOWN,	/* input select */
	KEY_UNKNOWN,	/* show information */
	KEY_UNKNOWN,	/* help */
	KEY_UNKNOWN,	/* page up */
	KEY_UNKNOWN,	/* page down */
	KEY_RESERVED,	/* 0x39 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x3F */
	KEY_RESERVED,	/* 0x40 */
	KEY_UNKNOWN,	/* volume up */
	KEY_UNKNOWN,	/* volume down */
	KEY_UNKNOWN,	/* mute */
	KEY_PLAY,
	KEY_STOP,
	KEY_PLAYPAUSE,
	KEY_UNKNOWN,	/* record */
	KEY_REWIND,
	KEY_FASTFORWARD,
	KEY_UNKNOWN,	/* eject */
	KEY_NEXTSONG,
	KEY_PREVIOUSSONG,
	KEY_RESERVED,	/* 0x4D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x4F */
	KEY_UNKNOWN,	/* angle */
	KEY_UNKNOWN,	/* subtitle */
	KEY_RESERVED,	/* 0x52 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x5F */
	KEY_PLAY,
	KEY_PAUSE,
	KEY_UNKNOWN,	/* record_function */
	KEY_UNKNOWN,	/* pause_record_function */
	KEY_STOP,
	KEY_UNKNOWN,	/* mute_function */
	KEY_UNKNOWN,	/* restore_volume_function */
	KEY_UNKNOWN,	/* tune_function */
	KEY_UNKNOWN,	/* select_media_function */
	KEY_RESERVED,	/* 0x69 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x70 */
	KEY_UNKNOWN,	/* F1 */
	KEY_UNKNOWN,	/* F2 */
	KEY_UNKNOWN,	/* F3 */
	KEY_UNKNOWN,	/* F4 */
	KEY_UNKNOWN,	/* F5 */
	KEY_RESERVED,	/* 0x76 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x7D */
	KEY_VENDOR,
	KEY_RESERVED,	/* 0x7F */
};
#define SII9234_RCP_NUM_KEYS ARRAY_SIZE(sii9234_rcp_def_keymap)

struct sii9234_data {
	struct sii9234_platform_data	*pdata;
	struct otg_id_notifier_block	otg_id_nb;
	wait_queue_head_t		wq;

	bool				claimed;
	enum mhl_state			state;
	enum rgnd_state			rgnd;
	int				irq;
	bool				rsen;

	struct mutex			lock;

	bool				msc_ready;
	struct mutex			msc_lock;
	struct completion		msc_complete;

	u8				devcap[16];
	u8				link_mode;

	struct work_struct		msc_work;
	struct list_head		msc_data_list;

	struct input_dev		*input_dev;
	struct mutex			input_lock;
	u16				keycode[SII9234_RCP_NUM_KEYS];

	struct work_struct		redetect_work;
	struct workqueue_struct		*redetect_wq;
};

static irqreturn_t sii9234_irq_thread(int irq, void *data);

static int mhl_tx_write_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9234->pdata->mhl_tx_client, offset,
			value);
}

static int mhl_tx_read_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9234->pdata->mhl_tx_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9234->pdata->mhl_tx_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int mhl_tx_set_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = mhl_tx_read_reg(sii9234, offset, &value);
	if (ret < 0)
		return ret;

	value |= mask;

	return mhl_tx_write_reg(sii9234, offset, value);
}

static int mhl_tx_clear_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = mhl_tx_read_reg(sii9234, offset, &value);
	if (ret < 0)
		return ret;

	value &= ~mask;

	return mhl_tx_write_reg(sii9234, offset, value);
}

static int tpi_write_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9234->pdata->tpi_client, offset,
			value);
}

static int tpi_read_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9234->pdata->tpi_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9234->pdata->tpi_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int hdmi_rx_write_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9234->pdata->hdmi_rx_client, offset,
			value);
}

static int cbus_write_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9234->pdata->cbus_client, offset,
			value);
}

static int cbus_read_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9234->pdata->cbus_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9234->pdata->cbus_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int cbus_set_reg(struct sii9234_data *sii9234, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = cbus_read_reg(sii9234, offset, &value);
	if (ret < 0)
		return ret;

	value |= mask;

	return cbus_write_reg(sii9234, offset, value);
}

static int mhl_wake_toggle(struct sii9234_data *sii9234,
		unsigned long high_period,
		unsigned long low_period)
{
	int ret;

	/* These bits are not documented. */
	ret = mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL7_REG, (1<<7) | (1<<6));
	if (ret < 0)
		return ret;

	usleep_range(high_period * USEC_PER_MSEC, high_period * USEC_PER_MSEC);

	ret = mhl_tx_clear_reg(sii9234, MHL_TX_DISC_CTRL7_REG, (1<<7) | (1<<6));
	if (ret < 0)
		return ret;

	usleep_range(low_period * USEC_PER_MSEC, low_period * USEC_PER_MSEC);

	return 0;
}

static int mhl_send_wake_pulses(struct sii9234_data *sii9234)
{
	int ret;

	ret = mhl_wake_toggle(sii9234, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_1);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9234, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_2);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9234, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_1);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9234, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_TO_DISCOVER);
	if (ret < 0)
		return ret;

	return 0;
}
static int sii9234_cbus_reset(struct sii9234_data *sii9234)
{
	int ret;
	/* Reset CBUS */
	ret = mhl_tx_set_reg(sii9234, MHL_TX_SRST, 0x03);
	if (ret < 0)
		return ret;

	usleep_range(2000, 3000);

	ret = mhl_tx_clear_reg(sii9234, MHL_TX_SRST, 0x03);
	if (ret < 0)
		return ret;

       /* Adjust interrupt mask everytime reset is performed.*/
	ret = cbus_write_reg(sii9234,
			     CBUS_INT_1_MASK,
			     MSC_RESP_ABORT_MASK |
			     MSC_REQ_ABORT_MASK |
			     MSC_REQ_DONE_MASK |
			     MSC_MSG_RECD_MASK |
			     CBUS_DDC_ABORT_MASK);
	if (ret < 0)
		return ret;

	ret = cbus_write_reg(sii9234,
			     CBUS_INT_2_MASK,
			     WRT_STAT_RECD_MASK |
			     SET_INT_RECD_MASK);
	if (ret < 0)
		return ret;

	return 0;
}

static int sii9234_cbus_init(struct sii9234_data *sii9234)
{
	u8 value;

	cbus_write_reg(sii9234, 0x07, 0x32);
	cbus_write_reg(sii9234, 0x40, 0x03);
	cbus_write_reg(sii9234, 0x42, 0x06);
	cbus_write_reg(sii9234, 0x36, 0x0C);

	cbus_write_reg(sii9234, 0x3D, 0xFD);
	cbus_write_reg(sii9234, 0x1C, 0x00);

	cbus_write_reg(sii9234, 0x44, 0x02);

	/* Setup our devcap*/
	cbus_write_reg(sii9234, 0x80, 0x04);
	cbus_write_reg(sii9234, 0x81, 0x10);
	cbus_write_reg(sii9234, 0x82, 0x02);
	cbus_write_reg(sii9234, 0x83, 0);
	cbus_write_reg(sii9234, 0x84, 0);
	cbus_write_reg(sii9234, 0x85, 0x01 | 0x02);
	cbus_write_reg(sii9234, 0x86, 0x01);
	cbus_write_reg(sii9234, 0x87, 0);
	cbus_write_reg(sii9234, 0x88, (1<<2) | (1<<1) | (1<<3) | (1<<7));
	cbus_write_reg(sii9234, 0x89, 0x0F);
	cbus_write_reg(sii9234, 0x8A, (1<<0) | (1<<1) | (1<<2));
	cbus_write_reg(sii9234, 0x8B, 0);
	cbus_write_reg(sii9234, 0x8C, 0);
	cbus_write_reg(sii9234, 0x8D, 16);
	cbus_write_reg(sii9234, 0x8E, 0x44);
	cbus_write_reg(sii9234, 0x8F, 0);

	cbus_read_reg(sii9234, 0x31, &value);
	value |= 0x0C;
	cbus_write_reg(sii9234, 0x31, value);

	cbus_read_reg(sii9234, 0x22, &value);
	value &= 0x0F;
	cbus_write_reg(sii9234, 0x22, value);

	cbus_write_reg(sii9234, 0x30, 0x01);

	return 0;
}

static int sii9234_power_init(struct sii9234_data *sii9234)
{
	int ret;

	/* Force the SiI9234 into the D0 state. */
	ret = tpi_write_reg(sii9234, TPI_DPD_REG, 0x3F);
	if (ret < 0)
		return ret;

	/* Enable TxPLL Clock */
	ret = hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS_CLK_EN_REG, 0x01);
	if (ret < 0)
		return ret;

	/* Enable Tx Clock Path & Equalizer*/
	ret = hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS_CH_EN_REG, 0x15);
	if (ret < 0)
		return ret;

	/* Power Up TMDS*/
	ret = mhl_tx_write_reg(sii9234, 0x08, 0x35);
	if (ret < 0)
		return ret;

	return 0;
}

static void sii9234_hdmi_init(struct sii9234_data *sii9234)
{
	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
	hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);

	/* PLL Calrefsel */
	hdmi_rx_write_reg(sii9234, HDMI_RX_PLL_CALREFSEL_REG, 0x03);

	/* VCO Cal */
	hdmi_rx_write_reg(sii9234, HDMI_RX_PLL_VCOCAL_REG, 0x20);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9234, HDMI_RX_EQ_DATA0_REG, 0x8A);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9234, HDMI_RX_EQ_DATA1_REG, 0x6A);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9234, HDMI_RX_EQ_DATA2_REG, 0xAA);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9234, HDMI_RX_EQ_DATA3_REG, 0xCA);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9234, HDMI_RX_EQ_DATA4_REG, 0xEA);

	/* Manual zone */
	hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS_ZONE_CTRL_REG, 0xA0);

	/* PLL Mode Value */
	hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS_MODE_CTRL_REG, 0x00);

	mhl_tx_write_reg(sii9234, MHL_TX_TMDS_CCTRL, 0x34);

	hdmi_rx_write_reg(sii9234, 0x45, 0x44);

	/* Rx PLL BW ~ 4MHz */
	hdmi_rx_write_reg(sii9234, 0x31, 0x0A);

	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
	hdmi_rx_write_reg(sii9234, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);
}

static int sii9234_register_input_device(struct sii9234_data *sii9234)
{
	struct input_dev *input;
	int ret;
	u8 i;

	input = input_allocate_device();
	if (!input) {
		pr_err("sii9234: failed to allocate input device\n");
		return -ENOMEM;
	}

	/* indicate that we generate key events */
	set_bit(EV_KEY, input->evbit);
	memcpy(sii9234->keycode, sii9234_rcp_def_keymap,
			SII9234_RCP_NUM_KEYS *
				sizeof(sii9234_rcp_def_keymap[0]));
	input->keycode = sii9234->keycode;
	input->keycodemax = SII9234_RCP_NUM_KEYS;
	input->keycodesize = sizeof(sii9234->keycode[0]);
	for (i = 0; i < SII9234_RCP_NUM_KEYS; i++) {
		u16 keycode = sii9234->keycode[i];
		if (keycode != KEY_UNKNOWN && keycode != KEY_RESERVED)
			set_bit(keycode, input->keybit);
	}

	input_set_drvdata(input, sii9234);
	input->name = "sii9234_rcp";
	input->id.bustype = BUS_I2C;

	pr_debug("sii9234: registering input device\n");
	ret = input_register_device(input);
	if (ret < 0) {
		pr_err("sii9234: failed to register input device\n");
		input_free_device(input);
		return ret;
	}

	mutex_lock(&sii9234->input_lock);
	sii9234->input_dev = input;
	mutex_unlock(&sii9234->input_lock);

	return 0;
}

static void sii9234_unregister_input_device(struct sii9234_data *sii9234)
{
	mutex_lock(&sii9234->input_lock);
	if (sii9234->input_dev) {
		pr_debug("sii9234: unregistering input device\n");
		input_unregister_device(sii9234->input_dev);
		sii9234->input_dev = NULL;
	}
	mutex_unlock(&sii9234->input_lock);
}

static void sii9234_mhl_tx_ctl_int(struct sii9234_data *sii9234)
{
	mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL1_REG, 0xD0);
	mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL2_REG, 0xFC);
	mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL4_REG, 0xEB);
	mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL7_REG, 0x0C);
}

static void sii9234_power_down(struct sii9234_data *sii9234)
{
	if (sii9234->claimed)
		sii9234->pdata->connect(false, NULL);

	sii9234_unregister_input_device(sii9234);
	sii9234->state = STATE_DISCONNECTED;
	sii9234->claimed = false;

	tpi_write_reg(sii9234, TPI_DPD_REG, 0);

	sii9234->pdata->power(0);
	sii9234->pdata->enable(0);
}

/* toggle hpd line low for 100ms */
static void sii9234_toggle_hpd(struct sii9234_data *sii9234)
{
	mhl_tx_set_reg(sii9234, MHL_TX_INT_CTRL_REG, HPD_OUT_OVR_EN);
	mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG, HPD_OUT_OVR_VAL);
	msleep(100);
	mhl_tx_set_reg(sii9234, MHL_TX_INT_CTRL_REG, HPD_OUT_OVR_VAL);
	mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG, HPD_OUT_OVR_EN);
}

/* Must call with sii9234->lock held */
static int sii9234_msc_req_locked(struct sii9234_data *sii9234, u8 req_type,
				  u8 offset, u8 first_data, u8 second_data)
{
	int ret;
	bool write_offset = req_type &
		(START_READ_DEVCAP | START_WRITE_STAT_INT | START_WRITE_BURST);
	bool write_first_data = req_type &
		(START_WRITE_STAT_INT | START_MSC_MSG);
	bool write_second_data = req_type & START_MSC_MSG;

	if (sii9234->state != STATE_ESTABLISHED)
		return -ENOENT;

	mutex_unlock(&sii9234->lock);
	ret = wait_event_timeout(sii9234->wq, sii9234->msc_ready,
				 msecs_to_jiffies(2000));
	mutex_lock(&sii9234->lock);
	if (!sii9234->msc_ready)
		return -EIO;

	init_completion(&sii9234->msc_complete);

	if (write_offset)
		cbus_write_reg(sii9234, CBUS_MSC_OFFSET_REG, offset);
	if (write_first_data)
		cbus_write_reg(sii9234, CBUS_MSC_FIRST_DATA_OUT, first_data);
	if (write_second_data)
		cbus_write_reg(sii9234, CBUS_MSC_SECOND_DATA_OUT, second_data);
	cbus_write_reg(sii9234, CBUS_MSC_COMMAND_START, req_type);

	mutex_unlock(&sii9234->lock);
	ret = wait_for_completion_timeout(&sii9234->msc_complete,
					  msecs_to_jiffies(500));
	mutex_lock(&sii9234->lock);

	return ret ? 0 : -EIO;
}

/* Must call with sii9234->lock held */
static int sii9234_devcap_read_locked(struct sii9234_data *sii9234, u8 offset)
{
	int ret;
	u8 val;

	if (offset > 0xf)
		return -EINVAL;

	ret = sii9234_msc_req_locked(sii9234, START_READ_DEVCAP, offset, 0, 0);
	if (ret < 0)
		return ret;

	ret = cbus_read_reg(sii9234, CBUS_MSC_FIRST_DATA_IN, &val);
	if (ret < 0)
		return ret;

	return val;
}

static int sii9234_queue_devcap_read_locked(struct sii9234_data *sii9234,
		u8 offset)
{
	struct completion cvar;
	struct msc_data *data;
	int ret;

	data = kzalloc(sizeof(struct msc_data), GFP_KERNEL);
	if (!data) {
		dev_err(&sii9234->pdata->mhl_tx_client->dev,
			"failed to allocate msc data");
		return -ENOMEM;
	}
	init_completion(&cvar);
	data->cmd = READ_DEVCAP;
	data->offset = offset;
	data->cvar = &cvar;
	data->ret = &ret;
	list_add_tail(&data->list, &sii9234->msc_data_list);

	mutex_unlock(&sii9234->lock);
	schedule_work(&sii9234->msc_work);
	wait_for_completion(&cvar);
	mutex_lock(&sii9234->lock);

	return ret;
}


static int sii9234_detection_callback(struct otg_id_notifier_block *nb)
{
	struct sii9234_data *sii9234 = container_of(nb, struct sii9234_data,
						otg_id_nb);
	int ret;
	int i;
	u8 value;
	int handled = OTG_ID_UNHANDLED;

	pr_debug("si9234: detection started\n");

	mutex_lock(&sii9234->lock);
	sii9234->link_mode = MHL_STATUS_CLK_NORMAL;
	sii9234->rgnd = RGND_UNKNOWN;
	sii9234->rsen = false;
	sii9234->msc_ready = false;
	if (sii9234->state == STATE_DISCONNECTING) {
		pr_debug("sii9234: disconnecting, bypassing detection\n");
		sii9234->state = STATE_DISCONNECTED;

		mutex_unlock(&sii9234->lock);
		return OTG_ID_UNHANDLED;
	}
	sii9234->state = STATE_DISCONNECTED;

	/* Set the board configuration so the  SiI9234 has access to the
	 * external connector.
	 */
	sii9234->pdata->enable(1);
	sii9234->pdata->power(1);

	ret = sii9234_power_init(sii9234);
	if (ret < 0)
		goto unhandled;

	sii9234_hdmi_init(sii9234);

	sii9234_mhl_tx_ctl_int(sii9234);

	/* Enable HDCP Compliance safety*/
	ret = mhl_tx_write_reg(sii9234, 0x2B, 0x01);
	if (ret < 0)
		goto unhandled;

	/* CBUS discovery cycle time for each drive and float = 150us*/
	ret = mhl_tx_read_reg(sii9234, MHL_TX_DISC_CTRL1_REG, &value);
	if (ret < 0)
		goto unhandled;

	value &= ~(1<<2);
	value |= (1<<3);

	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL1_REG, value);
	if (ret < 0)
		goto unhandled;

	/* Clear bit 6 (reg_skip_rgnd) */
	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL2_REG,
			(1<<7) /* Reserved Bit */ |
			2 << ATT_THRESH_SHIFT |
			DEGLITCH_TIME_128MS);
	if (ret < 0)
		goto unhandled;

	/* Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel */
	/* 1.8V CBUS VTH & GND threshold */
	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL5_REG, 0x75);
	if (ret < 0)
		goto unhandled;

	/* set bit 2 and 3, which is Initiator Timeout */
	ret = cbus_read_reg(sii9234, CBUS_LINK_CONTROL_2_REG, &value);
	if (ret < 0)
		goto unhandled;

	value |= 0x0C;

	ret = cbus_write_reg(sii9234, CBUS_LINK_CONTROL_2_REG, value);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL6_REG, 0xA0);
	if (ret < 0)
		goto unhandled;

	/* RGND & single discovery attempt (RGND blocking) */
	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL6_REG, BLOCK_RGND_INT |
			DVRFLT_SEL | SINGLE_ATT);
	if (ret < 0)
		goto unhandled;

	/* Use VBUS path of discovery state machine*/
	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL8_REG, 0);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	/* To allow RGND engine to operate correctly.
	 * When moving the chip from D2 to D0 (power up, init regs)
	 * the values should be
	 * 94[1:0] = 01  reg_cbusmhl_pup_sel[1:0] should be set for 5k
	 * 93[7:6] = 10  reg_cbusdisc_pup_sel[1:0] should be
	 * set for 10k (default)
	 * 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
	 */
	ret = mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL3_REG, 0x86);
	if (ret < 0)
		goto unhandled;

	/* change from CC to 8C to match 5K*/
	ret = mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL4_REG, 0x8C);
	if (ret < 0)
		goto unhandled;

	/* Configure the interrupt as active high */
	ret = mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG, (1<<2) | (1<<1));
	if (ret < 0)
		goto unhandled;

	msleep(25);

	ret = mhl_tx_clear_reg(sii9234, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL1_REG, 0x27);
	if (ret < 0)
		goto unhandled;

	/* Reset CBUS */
	ret = sii9234_cbus_reset(sii9234);
	if (ret < 0)
		goto unhandled;

	sii9234_cbus_init(sii9234);

	/* Enable Auto soft reset on SCDT = 0*/
	ret = mhl_tx_write_reg(sii9234, 0x05, 0x04);

	if (ret < 0)
		goto unhandled;

	/* HDMI Transcode mode enable*/
	ret = mhl_tx_write_reg(sii9234, 0x0D, 0x1C);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9234, MHL_TX_INTR4_ENABLE_REG,
			RGND_READY_MASK | CBUS_LKOUT_MASK |
			MHL_DISC_FAIL_MASK | MHL_EST_MASK);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9234, MHL_TX_INTR1_ENABLE_REG,
			       (1<<5) | (1<<6));
	if (ret < 0)
		goto unhandled;


	pr_debug("sii9234: waiting for RGND measurement\n");
	enable_irq(sii9234->irq);

	/* SiI9244 Programmer's Reference Section 2.4.3
	 * State : RGND Ready
	 */
	mutex_unlock(&sii9234->lock);
	ret = wait_event_timeout(sii9234->wq,
				 ((sii9234->rgnd != RGND_UNKNOWN) ||
				  mhl_state_is_error(sii9234->state)),
				 msecs_to_jiffies(2000));

	mutex_lock(&sii9234->lock);
	if (sii9234->rgnd == RGND_UNKNOWN || mhl_state_is_error(sii9234->state))
		goto unhandled;

	if (sii9234->rgnd != RGND_1K)
		goto unhandled;

	mutex_unlock(&sii9234->lock);

	pr_debug("sii9234: waiting for detection\n");
	ret = wait_event_timeout(sii9234->wq,
				 sii9234->state != STATE_DISCONNECTED,
				 msecs_to_jiffies(500));
	mutex_lock(&sii9234->lock);
	if (sii9234->state == STATE_DISCONNECTED)
		goto unhandled;

	if (sii9234->state == STATE_DISCOVERY_FAILED) {
		handled = OTG_ID_PROXY_WAIT;
		goto unhandled;
	}

	if (mhl_state_is_error(sii9234->state))
		goto unhandled;

	mutex_unlock(&sii9234->lock);
	wait_event_timeout(sii9234->wq, sii9234->rsen,
				msecs_to_jiffies(T_WAIT_TIMEOUT_RSEN_INT));
	mutex_lock(&sii9234->lock);
	if (!sii9234->rsen) {
		ret = mhl_tx_read_reg(sii9234, MHL_TX_SYSSTAT_REG, &value);
		pr_debug("sii9234: Recheck RSEN value\n");
		if (!(ret && (value & RSEN_STATUS))) {
			usleep_range(T_SRC_RXSENSE_DEGLITCH * USEC_PER_MSEC,
					T_SRC_RXSENSE_DEGLITCH * USEC_PER_MSEC);
			pr_debug("sii9234: RSEN is low -> retry once\n");
			ret = mhl_tx_read_reg(sii9234, MHL_TX_SYSSTAT_REG,
								&value);
			if (!(ret && (value & RSEN_STATUS))) {
				pr_debug("sii9234: RSEN is still low\n");
				goto unhandled;
			}
		}
		sii9234->rsen = value & RSEN_STATUS;
	}

	memset(sii9234->devcap, 0x0, sizeof(sii9234->devcap));
	for (i = 0; i < 16; i++) {
		ret = sii9234_queue_devcap_read_locked(sii9234, i);
		if (ret < 0)
			goto unhandled;
	}

#ifdef DEBUG
	if (ret >= 0)
		print_hex_dump(KERN_DEBUG, "sii9234: devcap = ", DUMP_PREFIX_NONE,
			       16, 1, sii9234->devcap, 16, false);
#endif

	/* It's possible for devcap reading to fail but the adapter still
	 * be connected.  Therefore we must keep ownership of the port
	 * as long as it's still connected.
	 */
	if (sii9234->state != STATE_ESTABLISHED)
		goto unhandled;

	pr_info("si9234: connection established\n");

	sii9234->claimed = true;
	sii9234->pdata->connect(true, ret >= 0 ? sii9234->devcap : NULL);
	if (sii9234->devcap[MHL_DEVCAP_FEATURE_FLAG] &
			MHL_FEATURE_FLAG_RCP_SUPPORT)
		sii9234_register_input_device(sii9234);
	mutex_unlock(&sii9234->lock);

	return OTG_ID_HANDLED;

unhandled:
	pr_info("sii9234: Detection failed");
	if (sii9234->state == STATE_DISCONNECTED)
		pr_cont(" (timeout)");
	else if (sii9234->state == STATE_DISCOVERY_FAILED)
		pr_cont(" (discovery failed)");
	else if (sii9234->state == STATE_CBUS_LOCKOUT)
		pr_cont(" (cbus_lockout)");
	pr_cont("\n");

	disable_irq_nosync(sii9234->irq);
	/* MHL Specs:"A source should reattempt discovery multiple times
	 * for as long as the Source requirement of discovery persists".
	 */
	if (sii9234->state == STATE_DISCOVERY_FAILED &&
				sii9234->rgnd == RGND_1K) {
		sii9234->pdata->power(0);
		queue_work(sii9234->redetect_wq, &sii9234->redetect_work);
		handled = OTG_ID_HANDLED;
	} else {
		sii9234_power_down(sii9234);
	}

	mutex_unlock(&sii9234->lock);
	return handled;
}

static void sii9234_cancel_callback(struct otg_id_notifier_block *nb)
{
	struct sii9234_data *sii9234 = container_of(nb, struct sii9234_data,
						otg_id_nb);

	mutex_lock(&sii9234->lock);
	sii9234_power_down(sii9234);
	mutex_unlock(&sii9234->lock);
}

static void sii9234_retry_detection(struct work_struct *work)
{
	struct sii9234_data *sii9234 = container_of(work, struct sii9234_data,
						redetect_work);

	pr_info("sii9234: detection restarted\n");
	/* if redetection fails, notify otg to take control */
	if (sii9234_detection_callback(&sii9234->otg_id_nb) == OTG_ID_UNHANDLED)
		otg_id_notify();
}

static void rcp_key_report(struct sii9234_data *sii9234, u16 key)
{
	pr_debug("sii9234: report rcp key: %d\n", key);
	mutex_lock(&sii9234->input_lock);
	if (sii9234->input_dev) {
		input_report_key(sii9234->input_dev, key, 1);
		input_report_key(sii9234->input_dev, key, 0);
		input_sync(sii9234->input_dev);
	}
	mutex_unlock(&sii9234->input_lock);
}

static void cbus_process_rcp_key(struct sii9234_data *sii9234, u8 key)
{
	if (key < SII9234_RCP_NUM_KEYS &&
			sii9234->keycode[key] != KEY_UNKNOWN &&
			sii9234->keycode[key] != KEY_RESERVED) {
		/* Report the key */
		rcp_key_report(sii9234, sii9234->keycode[key]);
	} else {
		/*
		* Send a RCPE(RCP Error Message) to Peer followed by
		* RCPK with old key-code so that initiator(TV) can
		* recognize failed key code.error code = 0x01 means
		* Ineffective key code was received.
		* See Table 21.(PRM)for details.
		*/
		sii9234_msc_req_locked(sii9234, START_MSC_MSG,
				0, MSG_RCPE, 0x01);
	}

	/* Send the RCP ack */
	sii9234_msc_req_locked(sii9234, START_MSC_MSG, 0, MSG_RCPK, key);
}

static u8 sii9234_tmds_control(struct sii9234_data *sii9234, bool enable)
{
	u8 ret = -1;

	if (enable) {
		ret = mhl_tx_set_reg(sii9234, MHL_TX_TMDS_CCTRL, (1<<4));
		if (ret < 0)
			return ret;
		pr_debug("sii9234: MHL HPD High, enabled TMDS\n");
		ret = mhl_tx_set_reg(sii9234, MHL_TX_INT_CTRL_REG,
							(1<<4) | (1<<5));
	} else {
		ret = mhl_tx_clear_reg(sii9234, MHL_TX_TMDS_CCTRL, (1<<4));
		if (ret < 0)
			return ret;
		pr_debug("sii9234 MHL HPD low, disabled TMDS\n");
		ret = mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG,
							(1<<4) | (1<<5));
	}

	return ret;
}

static void cbus_process_rap_key(struct sii9234_data *sii9234, u8 key)
{
	u8 err = 0x00; /* no error */

	switch (key) {
	case CBUS_MSC_RAP_POLL:
		/* no action, just sent to elicit an ACK */
		break;
	case CBUS_MSC_RAP_CONTENT_ON:
		sii9234_tmds_control(sii9234, true);
		break;
	case CBUS_MSC_RAP_CONTENT_OFF:
		sii9234_tmds_control(sii9234, false);
		break;
	default:
		pr_debug("sii9234: unrecognized RAP code %u\n", key);
		err = 0x01; /* unrecognized action code */
	}

	sii9234_msc_req_locked(sii9234, START_MSC_MSG, 0, MSG_RAPK, err);
}

static int cbus_handle_set_interrupt(struct sii9234_data *sii9234,
							u8 offset, u8 data)
{
	u8 ret = -1;
	ret = sii9234_msc_req_locked(sii9234, START_WRITE_STAT_INT, offset,
					 data, 0);

	if (ret < 0)
		return ret;
	if (offset == CBUS_MHL_INTR_OFFSET_0 && data == MHL_INT_DCAP_CHG) {

		/* notify the peer by updating the status register too */
		sii9234_msc_req_locked(sii9234, START_WRITE_STAT_INT,
					CBUS_MHL_STATUS_OFFSET_0,
					MHL_STATUS_DCAP_READY, 0);
	}

	return ret;
}

static void sii9234_msc_event(struct work_struct *work)
{
	int ret = -1;
	struct msc_data *data, *next;
	struct sii9234_data *sii9234 = container_of(work, struct sii9234_data,
			msc_work);

	mutex_lock(&sii9234->msc_lock);
	mutex_lock(&sii9234->lock);

	list_for_each_entry_safe(data, next, &sii9234->msc_data_list, list) {
		switch (data->cmd) {
		case MSC_MSG:
			switch (data->offset) {
			case MSG_RCP:
				pr_debug("sii9234: RCP Arrived. KEY CODE:%d\n",
					data->data);
				cbus_process_rcp_key(sii9234, data->data);
				break;
			case MSG_RAP:
				pr_debug("sii9234: RAP Arrived\n");
				cbus_process_rap_key(sii9234, data->data);
				break;
			case MSG_RCPK:
				pr_debug("sii9234: RCPK Arrived\n");
				break;
			case MSG_RCPE:
				pr_debug("sii9234: RCPE Arrived\n");
				break;
			case MSG_RAPK:
				pr_debug("sii9234: RAPK Arrived\n");
				break;
			default:
				pr_debug("sii9234: MAC error\n");
				break;
			}
			break;

		case READ_DEVCAP:
			ret = sii9234_devcap_read_locked(sii9234, data->offset);
			if (ret < 0) {
				pr_err("sii9234: error reading device capability"
						 "register:%d", data->offset);
				break;
			}
			sii9234->devcap[data->offset] = ret;
			ret = 0;
			break;

		case SET_INT:
			ret = cbus_handle_set_interrupt(sii9234, data->offset,
								data->data);
			if (ret < 0)
				pr_err("sii9234: error requesting set_int\n");
			break;
		case WRITE_STAT:
			ret = sii9234_msc_req_locked(sii9234,
					START_WRITE_STAT_INT, data->offset,
					data->data, 0);
			if (ret < 0)
				pr_err("sii9234: error requesting write_stat\n");
			break;

		case WRITE_BURST:
			/* TODO: */
			break;

		case GET_STATE:
		case GET_VENDOR_ID:
		case SET_HPD:
		case CLR_HPD:
		case GET_MSC_ERR_CODE:
		case GET_SC3_ERR_CODE:
		case GET_SC1_ERR_CODE:
		case GET_DDC_ERR_CODE:
			ret = sii9234_msc_req_locked(sii9234,
					START_MSC_RESERVED, data->offset,
					data->data, 0);
			if (ret < 0)
				pr_err("sii9234: error requesting offset:%d"
					 "data:%d", data->offset, data->data);
			break;

		default:
			pr_info("sii9234: invalid msc command\n");
			break;
		}

		if (data->cvar) {
			*data->ret = ret;
			complete(data->cvar);
		}

		list_del(&data->list);
		kfree(data);
	}

	mutex_unlock(&sii9234->lock);
	mutex_unlock(&sii9234->msc_lock);
}

static void cbus_resp_abort_error(struct sii9234_data *sii9234)
{
	u8 abort_reason = 0;
	pr_debug("sii9234: MSC Response Aborted:");
	cbus_read_reg(sii9234, MSC_RESP_ABORT_REASON_REG, &abort_reason);
	cbus_write_reg(sii9234, MSC_RESP_ABORT_REASON_REG, 0xFF);

	if (abort_reason) {
		if (abort_reason & ABORT_BY_PEER)
			pr_cont(" Peer Sent an ABORT");
		if (abort_reason & UNDEF_CMD)
			pr_cont(" Undefined Opcode");
		if (abort_reason & TIMEOUT)
			pr_cont(" Requestor Translation layer Timeout");
	}
	pr_cont("\n");
}

static void cbus_req_abort_error(struct sii9234_data *sii9234)
{
	u8 abort_reason = 0;
	pr_debug("sii9234: MSC Request Aborted:");
	cbus_read_reg(sii9234, MSC_REQ_ABORT_REASON_REG, &abort_reason);
	cbus_write_reg(sii9234, MSC_REQ_ABORT_REASON_REG, 0xFF);

	if (abort_reason) {
		if (abort_reason & ABORT_BY_PEER)
			pr_cont(" Peer Sent an ABORT");
		if (abort_reason & UNDEF_CMD)
			pr_cont(" Undefined Opcode");
		if (abort_reason & TIMEOUT)
			pr_cont(" Requestor Translation layer Timeout");
		if (abort_reason & MAX_FAIL) {
			u8 msc_retry_thr_val = 0;
			pr_cont(" Retry Threshold exceeded");
			cbus_read_reg(sii9234,
					MSC_RETRY_FAIL_LIM_REG,
					&msc_retry_thr_val);
			pr_cont("Retry Threshold value is:%d",
					msc_retry_thr_val);
		}
	}
	pr_cont("\n");
}

static void force_usb_id_switch_open(struct sii9234_data *sii9234)
{
	pr_debug("sii9234: open usb_id\n");
	/*Disable CBUS discovery*/
	mhl_tx_clear_reg(sii9234, MHL_TX_DISC_CTRL1_REG, (1<<0));

	/*Force USB ID switch to open*/
	mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);

	mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL3_REG, 0xA6);

	/*Force upstream HPD to 0 when not in MHL mode.*/
	mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG, (1<<5));
	mhl_tx_set_reg(sii9234, MHL_TX_INT_CTRL_REG, (1<<4));
}

static void release_usb_id_switch_open(struct sii9234_data *sii9234)
{
	usleep_range(T_SRC_CBUS_FLOAT * USEC_PER_MSEC,
			T_SRC_CBUS_FLOAT * USEC_PER_MSEC);
	pr_debug("sii9234: release usb_id\n");
	/* clear USB ID switch to open*/
	mhl_tx_clear_reg(sii9234, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);

	/* Enable CBUS discovery*/
	mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL1_REG, (1<<0));
}


static bool cbus_ddc_abort_error(struct sii9234_data *sii9234)
{
	u8 val1, val2;
	/* clear the ddc abort counter */
	cbus_write_reg(sii9234, 0x29, 0xFF);
	cbus_read_reg(sii9234, 0x29, &val1);
	usleep_range(3000, 4000);
	cbus_read_reg(sii9234, 0x29, &val2);
	if (val2 > val1 + 50) {
		pr_debug("Applying DDC Abort Safety(SWA 18958)\n)");
		mhl_tx_set_reg(sii9234, MHL_TX_SRST, (1<<3));
		mhl_tx_clear_reg(sii9234, MHL_TX_SRST, (1<<3));
		force_usb_id_switch_open(sii9234);
		release_usb_id_switch_open(sii9234);
		mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL1_REG, 0xD0);
		sii9234_tmds_control(sii9234, false);
		/* Disconnect and notify to OTG */
		return true;
	}
	pr_debug("sii9234: DDC abort interrupt\n");

	return false;
}

static int sii9234_cbus_irq(struct sii9234_data *sii9234)
{
	u8 cbus_intr1, cbus_intr2;
	u8 mhl_intr0, mhl_intr1;
	u8 mhl_status0, mhl_status1, mhl_status2, mhl_status3;

	int ret = 0;

	cbus_read_reg(sii9234, CBUS_INT_STATUS_1_REG, &cbus_intr1);
	cbus_read_reg(sii9234, CBUS_INT_STATUS_2_REG, &cbus_intr2);
	cbus_read_reg(sii9234, CBUS_MHL_INTR_REG_0, &mhl_intr0);
	cbus_read_reg(sii9234, CBUS_MHL_INTR_REG_1, &mhl_intr1);
	cbus_read_reg(sii9234, CBUS_MHL_STATUS_REG_0, &mhl_status0);
	cbus_read_reg(sii9234, CBUS_MHL_STATUS_REG_1, &mhl_status1);
	cbus_read_reg(sii9234, CBUS_MHL_STATUS_REG_2, &mhl_status2);
	cbus_read_reg(sii9234, CBUS_MHL_STATUS_REG_3, &mhl_status3);

	pr_debug("sii9234: cbus_intr %02x %02x\n", cbus_intr1, cbus_intr2);

	if (cbus_intr1 & MSC_RESP_ABORT)
		cbus_resp_abort_error(sii9234);

	if (cbus_intr1 & MSC_REQ_ABORT)
		cbus_req_abort_error(sii9234);

	if (cbus_intr1 & CBUS_DDC_ABORT) {
		pr_warn("sii9234: ddc abort\n");
		if (cbus_ddc_abort_error(sii9234)) {
			/* error on ddc line,should it be -EIO? */
			ret = -EINVAL;
			goto err_exit;
		}
	}

	if (cbus_intr1 & MSC_REQ_DONE) {
		pr_debug("sii9234: msc request done\n");
		complete(&sii9234->msc_complete);
	}

	if (cbus_intr1 & MSC_MSG_RECD) {
		struct msc_data *data;

		pr_debug("sii9234: msc msg received\n");

		data = kzalloc(sizeof(struct msc_data), GFP_KERNEL);
		if (!data) {
			dev_err(&sii9234->pdata->mhl_tx_client->dev,
				"failed to allocate msc data");
			ret = -ENOMEM;
			goto err_exit;
		}
		data->cmd = MSC_MSG;
		cbus_read_reg(sii9234, CBUS_MSC_MSG_CMD_IN, &data->offset);
		cbus_read_reg(sii9234, CBUS_MSC_MSG_DATA_IN, &data->data);
		list_add_tail(&data->list, &sii9234->msc_data_list);

		schedule_work(&sii9234->msc_work);
	}


	if (cbus_intr2 & WRT_STAT_RECD) {
		struct msc_data *data;
		bool path_en_changed = false;
		pr_debug("sii9234: write status received\n");
		sii9234->msc_ready = mhl_status0 & MHL_STATUS_DCAP_READY;

		if (!(sii9234->link_mode & MHL_STATUS_PATH_ENABLED) &&
			(MHL_STATUS_PATH_ENABLED & mhl_status1)) {

			/* PATH_EN{SOURCE} = 0 and PATH_EN{SINK}= 1 */
			sii9234->link_mode |= MHL_STATUS_PATH_ENABLED;
			path_en_changed = true;

		} else if ((sii9234->link_mode & MHL_STATUS_PATH_ENABLED) &&
				!(MHL_STATUS_PATH_ENABLED & mhl_status1)) {

			/* PATH_EN{SOURCE} = 1 and PATH_EN{SINK}= 0 */
			sii9234->link_mode &= ~MHL_STATUS_PATH_ENABLED;
			path_en_changed = true;
		}

		if (path_en_changed) {
			data = kzalloc(sizeof(struct msc_data), GFP_KERNEL);
			if (!data) {
				dev_err(&sii9234->pdata->mhl_tx_client->dev,
					"failed to allocate msc data");
				ret = -ENOMEM;
				goto err_exit;
			}
			data->cmd = WRITE_STAT;
			data->offset = CBUS_MHL_STATUS_OFFSET_1;
			data->data = sii9234->link_mode;
			list_add_tail(&data->list, &sii9234->msc_data_list);
			schedule_work(&sii9234->msc_work);
		}
	}

	if (cbus_intr2 & SET_INT_RECD) {

		if (mhl_intr0 & MHL_INT_DCAP_CHG) {
			struct msc_data *data;
			/*
			 * devcap[] had already been populated while detection
			 * callback;now sink(or dongle) is again notiftying some
			 * capability change.
			 * TODO: should we read the complete devcap[] again?
			 */
			pr_debug("sii9234: device capability changed\n");
			data = kzalloc(sizeof(struct msc_data), GFP_KERNEL);
			if (!data) {
				dev_err(&sii9234->pdata->mhl_tx_client->dev,
					"failed to allocate msc data");
				ret = -ENOMEM;
				goto err_exit;
			}
			data->cmd = READ_DEVCAP;
			data->offset = MHL_DEVCAP_DEV_CAT;
			list_add_tail(&data->list, &sii9234->msc_data_list);
			schedule_work(&sii9234->msc_work);
		}

		if (mhl_intr0 & MHL_INT_DSCR_CHG) {
			/*
			 * TODO: Peer is done updating the scratchpad
			 * registers;Source should read the register values from
			 * local register space
			 */
			pr_debug("sii9234: scratchpad register change done\n");
		}

		if (mhl_intr0 & MHL_INT_REQ_WRT) {
			struct msc_data *data;
			pr_debug("sii9234: request-to-write received\n");
			data = kzalloc(sizeof(struct msc_data), GFP_KERNEL);
			if (!data) {
				dev_err(&sii9234->pdata->mhl_tx_client->dev,
					"failed to allocate msc data");
				ret = -ENOMEM;
				goto err_exit;
			}
			data->cmd = SET_INT;
			data->offset = CBUS_MHL_INTR_OFFSET_0;
			/* signal grant-to-write to the peer */
			data->data = MHL_INT_GRT_WRT;
			list_add_tail(&data->list, &sii9234->msc_data_list);
			schedule_work(&sii9234->msc_work);
		}

		if (mhl_intr0 & MHL_INT_GRT_WRT) {
			/* TODO: received a grant-to-write from peer;Source
			 * should initiate a WRITE_BURST
			 */
			pr_debug("sii9234: grant-to-write received\n");
		}

		if (mhl_intr1 & MHL_INT_EDID_CHG)
			sii9234_toggle_hpd(sii9234);
	}

	if (cbus_intr2 & WRT_BURST_RECD)
		pr_debug("sii9234: write burst received\n");

err_exit:
	cbus_write_reg(sii9234, CBUS_MHL_INTR_REG_0, mhl_intr0);
	cbus_write_reg(sii9234, CBUS_MHL_INTR_REG_1, mhl_intr1);
	cbus_write_reg(sii9234, CBUS_INT_STATUS_1_REG, cbus_intr1);
	cbus_write_reg(sii9234, CBUS_INT_STATUS_2_REG, cbus_intr2);

	return ret;
}

static irqreturn_t sii9234_irq_thread(int irq, void *data)
{
	struct sii9234_data *sii9234 = data;
	int ret;
	u8 intr1, intr4, value;
	u8 intr1_en, intr4_en;
	bool release_otg = false;

	mutex_lock(&sii9234->lock);
	mhl_tx_read_reg(sii9234, MHL_TX_INTR1_REG, &intr1);
	mhl_tx_read_reg(sii9234, MHL_TX_INTR4_REG, &intr4);

	mhl_tx_read_reg(sii9234, MHL_TX_INTR1_ENABLE_REG, &intr1_en);
	mhl_tx_read_reg(sii9234, MHL_TX_INTR4_ENABLE_REG, &intr4_en);
	pr_debug("sii9234: irq %02x/%02x %02x/%02x\n", intr1, intr1_en,
		 intr4, intr4_en);

	if (intr4 & RGND_READY_INT) {
		ret = mhl_tx_read_reg(sii9234, MHL_TX_STAT2_REG, &value);
		if (ret < 0) {
			dev_err(&sii9234->pdata->mhl_tx_client->dev,
					"STAT2 reg, err %d\n", ret);
			goto err_exit;
		}

		switch (value & RGND_INTP_MASK) {
		case RGND_INTP_OPEN:
			pr_debug("RGND Open\n");
			sii9234->rgnd = RGND_OPEN;
			break;
		case RGND_INTP_1K:
			pr_debug("RGND 1K\n");
			ret = mhl_tx_write_reg(sii9234, MHL_TX_DISC_CTRL1_REG,
					0x25);

			ret = mhl_send_wake_pulses(sii9234);
			sii9234->rgnd = RGND_1K;
			break;
		case RGND_INTP_2K:
			pr_debug("RGND 2K\n");
			ret = mhl_send_wake_pulses(sii9234);
			sii9234->rgnd = RGND_2K;
			break;
		case RGND_INTP_SHORT:
			pr_debug("RGND Short\n");
			sii9234->rgnd = RGND_SHORT;
			break;
		};
	}

	if (intr4 & CBUS_LKOUT_INT) {
		pr_debug("sii9234: CBUS Lockout Interrupt\n");
		sii9234->state = STATE_CBUS_LOCKOUT;
	}

	if (intr4 & MHL_DISC_FAIL_INT)
		sii9234->state = STATE_DISCOVERY_FAILED;

	if (intr4 & MHL_EST_INT) {
		/* discovery override */
		ret = mhl_tx_write_reg(sii9234, MHL_TX_MHLTX_CTL1_REG, 0x10);

		/* increase DDC translation layer timer (byte mode) */
		cbus_write_reg(sii9234, 0x07, 0x32);
		cbus_set_reg(sii9234, 0x44, 1<<1);

		/* Keep the discovery enabled. Need RGND interrupt */
		ret = mhl_tx_set_reg(sii9234, MHL_TX_DISC_CTRL1_REG, (1<<0));

		sii9234->state = STATE_ESTABLISHED;
	}

	if (intr1 & HPD_CHANGE_INT) {
		ret = cbus_read_reg(sii9234, MSC_REQ_ABORT_REASON_REG, &value);

		if (value & SET_HPD_DOWNSTREAM) {
			/* Downstream HPD Highi */

			/* Do we need to send HPD upstream using
			 * Register 0x79(page0)? Is HPD need to be overriden??
			 *      TODO: See if we need code for overriding HPD OUT
			 *      as per Page 0,0x79 Register
			 */

			/* Enable TMDS */
			sii9234_tmds_control(sii9234, true);
		} else {
			/*Downstream HPD Low*/

			/* Similar to above comments.
			 * TODO:Do we need to override HPD OUT value
			 * and do we need to disable TMDS here?
			 */

			/* Disable TMDS */
			sii9234_tmds_control(sii9234, false);
		}
	}

	if (intr1 & RSEN_CHANGE_INT) {
		ret = mhl_tx_read_reg(sii9234, MHL_TX_SYSSTAT_REG, &value);

		sii9234->rsen = value & RSEN_STATUS;

		if (value & RSEN_STATUS) {
			pr_info("sii9234: MHL cable connected.. RESN High\n");
		} else {
			pr_info("sii9234: RSEN lost\n");
			/* Once RSEN loss is confirmed,we need to check
			 * based on cable status and chip power status,whether
			 * it is SINK Loss(HDMI cable not connected, TV Off)
			 * or MHL cable disconnection
			 */

			/* sleep for handling glitch on RSEN */
			usleep_range(T_SRC_RXSENSE_DEGLITCH * USEC_PER_MSEC,
					T_SRC_RXSENSE_DEGLITCH * USEC_PER_MSEC);
			ret = mhl_tx_read_reg(sii9234, MHL_TX_SYSSTAT_REG,
								&value);
			pr_cont(" sys_stat:%x\n", value);
			if ((value & RSEN_STATUS) == 0) {
				/* Notify Disconnection to OTG */
				if (sii9234->claimed == true) {
					disable_irq_nosync(sii9234->irq);
					release_otg = true;
				}

				sii9234_tmds_control(sii9234, false);
				sii9234_power_down(sii9234);
			}

		}
	}

	if (sii9234->state == STATE_ESTABLISHED) {
		ret = sii9234_cbus_irq(sii9234);
		if (ret < 0) {
			if (sii9234->claimed == true) {
				disable_irq_nosync(sii9234->irq);
				release_otg = true;
			}
			sii9234_power_down(sii9234);
		}
	}

err_exit:
	mhl_tx_write_reg(sii9234, MHL_TX_INTR1_REG, intr1);
	mhl_tx_write_reg(sii9234, MHL_TX_INTR4_REG, intr4);

	mutex_unlock(&sii9234->lock);

	pr_debug("si9234: wake_up\n");
	wake_up(&sii9234->wq);

	if (release_otg) {
		sii9234->state = STATE_DISCONNECTING;
		otg_id_notify();
	}

	return IRQ_HANDLED;
}

static int __devinit sii9234_mhl_tx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sii9234_data *sii9234;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	sii9234 = kzalloc(sizeof(struct sii9234_data), GFP_KERNEL);
	if (!sii9234) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	sii9234->pdata = client->dev.platform_data;
	sii9234->pdata->mhl_tx_client = client;
	if (!sii9234->pdata) {
		ret = -EINVAL;
		goto err_exit;
	}

	i2c_set_clientdata(client, sii9234);

	sii9234->irq = client->irq;

	init_waitqueue_head(&sii9234->wq);
	mutex_init(&sii9234->lock);
	mutex_init(&sii9234->msc_lock);
	mutex_init(&sii9234->input_lock);

	INIT_WORK(&sii9234->msc_work, sii9234_msc_event);
	INIT_LIST_HEAD(&sii9234->msc_data_list);

	ret = request_threaded_irq(client->irq, NULL, sii9234_irq_thread,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "sii9234", sii9234);
	if (ret < 0)
		goto err_exit;

	disable_irq(client->irq);

	sii9234->otg_id_nb.detect = sii9234_detection_callback;
	sii9234->otg_id_nb.cancel = sii9234_cancel_callback;
	sii9234->otg_id_nb.priority = sii9234->pdata->prio;

	plist_node_init(&sii9234->otg_id_nb.p, sii9234->pdata->prio);

	ret = otg_id_register_notifier(&sii9234->otg_id_nb);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register notifier\n");
		goto err_exit;
	}

	sii9234->redetect_wq = create_singlethread_workqueue("sii9234");
	if (!sii9234->redetect_wq) {
		dev_err(&client->dev, "unable to create workqueue\n");
		goto err_exit;
	}
	INIT_WORK(&sii9234->redetect_work, sii9234_retry_detection);
	return 0;

err_exit:
	kfree(sii9234);
	return ret;
}

static int __devinit sii9234_tpi_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9234_platform_data *pdata = client->dev.platform_data;
	pdata->tpi_client = client;
	return 0;
}

static int __devinit sii9234_hdmi_rx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9234_platform_data *pdata = client->dev.platform_data;
	pdata->hdmi_rx_client = client;
	return 0;
}

static int __devinit sii9234_cbus_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9234_platform_data *pdata = client->dev.platform_data;
	pdata->cbus_client = client;
	return 0;
}

static int __devexit sii9234_mhl_tx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9234_tpi_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9234_hdmi_rx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9234_cbus_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sii9234_mhl_tx_id[] = {
	{"sii9234_mhl_tx", 0},
	{}
};

static const struct i2c_device_id sii9234_tpi_id[] = {
	{"sii9234_tpi", 0},
	{}
};

static const struct i2c_device_id sii9234_hdmi_rx_id[] = {
	{"sii9234_hdmi_rx", 0},
	{}
};

static const struct i2c_device_id sii9234_cbus_id[] = {
	{"sii9234_cbus", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sii9234_mhl_tx_id);
MODULE_DEVICE_TABLE(i2c, sii9234_tpi_id);
MODULE_DEVICE_TABLE(i2c, sii9234_hdmi_rx_id);
MODULE_DEVICE_TABLE(i2c, sii9234_cbus_id);

static struct i2c_driver sii9234_mhl_tx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9234_mhl_tx",
	},
	.id_table = sii9234_mhl_tx_id,
	.probe = sii9234_mhl_tx_i2c_probe,
	.remove = __devexit_p(sii9234_mhl_tx_remove),
	.command = NULL,
};

static struct i2c_driver sii9234_tpi_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9234_tpi",
	},
	.id_table = sii9234_tpi_id,
	.probe = sii9234_tpi_i2c_probe,
	.remove = __devexit_p(sii9234_tpi_remove),
};

static struct i2c_driver sii9234_hdmi_rx_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9234_hdmi_rx",
	},
	.id_table	= sii9234_hdmi_rx_id,
	.probe	= sii9234_hdmi_rx_i2c_probe,
	.remove	= __devexit_p(sii9234_hdmi_rx_remove),
};

static struct i2c_driver sii9234_cbus_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9234_cbus",
	},
	.id_table = sii9234_cbus_id,
	.probe = sii9234_cbus_i2c_probe,
	.remove = __devexit_p(sii9234_cbus_remove),
};

static int __init sii9234_init(void)
{
	int ret;

	ret = i2c_add_driver(&sii9234_mhl_tx_i2c_driver);
	if (ret < 0)
		return ret;

	ret = i2c_add_driver(&sii9234_tpi_i2c_driver);
	if (ret < 0)
		goto err_exit1;

	ret = i2c_add_driver(&sii9234_hdmi_rx_i2c_driver);
	if (ret < 0)
		goto err_exit2;

	ret = i2c_add_driver(&sii9234_cbus_i2c_driver);
	if (ret < 0)
		goto err_exit3;

	return 0;

err_exit3:
	i2c_del_driver(&sii9234_hdmi_rx_i2c_driver);
err_exit2:
	i2c_del_driver(&sii9234_tpi_i2c_driver);
err_exit1:
	i2c_del_driver(&sii9234_mhl_tx_i2c_driver);
	return ret;
}

static void __exit sii9234_exit(void)
{
	i2c_del_driver(&sii9234_cbus_i2c_driver);
	i2c_del_driver(&sii9234_hdmi_rx_i2c_driver);
	i2c_del_driver(&sii9234_tpi_i2c_driver);
	i2c_del_driver(&sii9234_mhl_tx_i2c_driver);
}

module_init(sii9234_init);
module_exit(sii9234_exit);
