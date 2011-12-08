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
#define   START_MSC_MSG		(1 << 1)
#define   START_READ_DEVCAP	(1 << 2)
#define   START_WRITE_STAT_INT	(1 << 3)
#define   START_WRITE_BURST	(1 << 4)

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

#define CBUS_MHL_INTR_REG_1		0xA1
#define   MHL_INT_EDID_CHG	(1<<1)

#define CBUS_MHL_INTR_REG_2		0xA2
#define CBUS_MHL_INTR_REG_3		0xA3

/* MHL Status Registers */
#define CBUS_MHL_STATUS_REG_0		0xB0
#define   MHL_STATUS_DCAP_READY	(1<<0)

#define CBUS_MHL_STATUS_REG_1		0xB1
#define CBUS_MHL_STATUS_REG_2		0xB2
#define CBUS_MHL_STATUS_REG_3		0xB3

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
};

static inline bool mhl_state_is_error(enum mhl_state state)
{
	return state == STATE_DISCOVERY_FAILED ||
		state == STATE_CBUS_LOCKOUT;
}

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
				  u8 offset)
{
	int ret;

	if (sii9234->state != STATE_ESTABLISHED)
		return -ENOENT;

	mutex_unlock(&sii9234->lock);
	ret = wait_event_timeout(sii9234->wq, sii9234->msc_ready,
				 msecs_to_jiffies(2000));
	mutex_lock(&sii9234->lock);
	if (!sii9234->msc_ready)
		return -EIO;

	mutex_lock(&sii9234->msc_lock);

	init_completion(&sii9234->msc_complete);

	cbus_write_reg(sii9234, CBUS_MSC_OFFSET_REG, offset);
	cbus_write_reg(sii9234, CBUS_MSC_COMMAND_START, req_type);

	mutex_unlock(&sii9234->lock);
	ret = wait_for_completion_timeout(&sii9234->msc_complete,
					  msecs_to_jiffies(500));
	mutex_lock(&sii9234->lock);

	mutex_unlock(&sii9234->msc_lock);

	return ret ? 0 : -EIO;
}

/* Must call with sii9234->lock held */
static int sii9234_devcap_read_locked(struct sii9234_data *sii9234, u8 offset)
{
	int ret;
	u8 val;

	if (offset > 0xf)
		return -EINVAL;

	ret = sii9234_msc_req_locked(sii9234, START_READ_DEVCAP, offset);
	if (ret < 0)
		return ret;

	ret = cbus_read_reg(sii9234, CBUS_MSC_FIRST_DATA_IN, &val);
	if (ret < 0)
		return ret;

	return val;
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
	sii9234->rgnd = RGND_UNKNOWN;
	sii9234->state = STATE_DISCONNECTED;
	sii9234->rsen = false;
	sii9234->msc_ready = false;

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
	wait_event_timeout(sii9234->wq, sii9234->rsen, msecs_to_jiffies(400));
	mutex_lock(&sii9234->lock);
	if (!sii9234->rsen)
		goto unhandled;

	memset(sii9234->devcap, 0x0, sizeof(sii9234->devcap));
	for (i = 0; i < 16; i++) {
		ret = sii9234_devcap_read_locked(sii9234, i);
		if (ret < 0)
			break;
		sii9234->devcap[i] = ret;
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

	sii9234_power_down(sii9234);

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
		pr_warn("sii9234: msc resp abort\n");

	if (cbus_intr1 & MSC_REQ_ABORT)
		pr_warn("sii9234: msc req abort\n");

	if (cbus_intr1 & CBUS_DDC_ABORT)
		pr_warn("sii9234: ddc abort\n");

	if (cbus_intr1 & MSC_REQ_DONE) {
		pr_debug("sii9234: msc request done\n");
		complete(&sii9234->msc_complete);
	}

	if (cbus_intr1 & MSC_MSG_RECD)
		pr_debug("sii9234: msc msg received\n");


	if (cbus_intr2 & WRT_STAT_RECD) {
		pr_debug("sii9234: write stat received\n");
		sii9234->msc_ready = mhl_status0 & MHL_STATUS_DCAP_READY;
	}

	if (cbus_intr2 & SET_INT_RECD) {
		if (mhl_intr1 & MHL_INT_EDID_CHG)
			sii9234_toggle_hpd(sii9234);
	}

	if (cbus_intr2 & WRT_BURST_RECD)
		pr_debug("sii9234: write burst received\n");

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
			ret = mhl_tx_set_reg(sii9234, MHL_TX_TMDS_CCTRL,
					     (1<<4));
			pr_debug("sii9234: MHL HPD High, enabled TMDS\n");

			ret = mhl_tx_set_reg(sii9234, MHL_TX_INT_CTRL_REG,
					     (1<<4) | (1<<5));
		} else {
			/*Downstream HPD Low*/

			/* Similar to above comments.
			 * TODO:Do we need to override HPD OUT value
			 * and do we need to disable TMDS here?
			 */

			/* Disable TMDS */
			ret = mhl_tx_clear_reg(sii9234, MHL_TX_TMDS_CCTRL,
					       (1<<4));
			pr_debug("sii9234 MHL HPD low, disabled TMDS\n");
			ret = mhl_tx_clear_reg(sii9234, MHL_TX_INT_CTRL_REG,
					       (1<<4) | (1<<5));
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
			 * TODO: Define the below mhl_disconnection()
			 */
			/* mhl_disconnection(); */
			/* Notify Disconnection to OTG */
			if (sii9234->claimed == true) {
				disable_irq_nosync(sii9234->irq);
				release_otg = true;
			}
			sii9234_power_down(sii9234);
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

	if (release_otg)
		otg_id_notify();

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
		goto err_exit1;
	}

	i2c_set_clientdata(client, sii9234);

	sii9234->irq = client->irq;

	init_waitqueue_head(&sii9234->wq);
	mutex_init(&sii9234->lock);
	mutex_init(&sii9234->msc_lock);

	ret = request_threaded_irq(client->irq, NULL, sii9234_irq_thread,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "sii9234", sii9234);
	if (ret < 0)
		goto err_exit2;

	disable_irq(client->irq);

	sii9234->otg_id_nb.detect = sii9234_detection_callback;
	sii9234->otg_id_nb.cancel = sii9234_cancel_callback;
	sii9234->otg_id_nb.priority = sii9234->pdata->prio;

	plist_node_init(&sii9234->otg_id_nb.p, sii9234->pdata->prio);

	ret = otg_id_register_notifier(&sii9234->otg_id_nb);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register notifier\n");
		goto err_exit2;
	}

	return 0;

err_exit2:
err_exit1:
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
