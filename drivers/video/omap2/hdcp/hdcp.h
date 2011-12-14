/*
 * hdcp.h
 *
 * HDCP interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Fabrice Olivero
 *	Fabrice Olivero <f-olivero@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _HDCP_H_
#define _HDCP_H_


/********************************/
/* Structures related to ioctl  */
/********************************/

/* HDCP key size in 32-bit words */
#define DESHDCP_KEY_SIZE 160

/* HDCP ioctl */
#include <linux/ioctl.h>
#include <linux/types.h>

struct hdcp_encrypt_control {
	uint32_t in_key[DESHDCP_KEY_SIZE];
	uint32_t *out_key;
};

struct hdcp_enable_control {
	uint32_t key[DESHDCP_KEY_SIZE];
	int nb_retry;
};

#define MAX_SHA_DATA_SIZE	645
#define MAX_SHA_VPRIME_SIZE	20

struct hdcp_sha_in {
	uint8_t data[MAX_SHA_DATA_SIZE];
	uint32_t byte_counter;
	uint8_t vprime[MAX_SHA_VPRIME_SIZE];
};

struct hdcp_wait_control {
	uint32_t event;
	struct hdcp_sha_in *data;
};

/* HDCP ioctl */
#define HDCP_IOCTL_MAGIC 'h'
#define HDCP_ENABLE	  _IOW(HDCP_IOCTL_MAGIC, 0, \
				struct hdcp_enable_control)
#define HDCP_DISABLE	  _IO(HDCP_IOCTL_MAGIC, 1)
#define HDCP_ENCRYPT_KEY  _IOWR(HDCP_IOCTL_MAGIC, 2, \
				struct hdcp_encrypt_control)
#define HDCP_QUERY_STATUS _IOWR(HDCP_IOCTL_MAGIC, 3, uint32_t)
#define HDCP_WAIT_EVENT _IOWR(HDCP_IOCTL_MAGIC, 4, \
				struct hdcp_wait_control)
#define HDCP_DONE	_IOW(HDCP_IOCTL_MAGIC, 5, uint32_t)

/* HDCP state */
#define HDCP_STATE_DISABLED		0
#define HDCP_STATE_INIT			1
#define HDCP_STATE_AUTH_1ST_STEP	2
#define HDCP_STATE_AUTH_2ND_STEP	3
#define HDCP_STATE_AUTH_3RD_STEP	4
#define HDCP_STATE_AUTH_FAIL_RESTARTING	5
#define HDCP_STATE_AUTH_FAILURE		6

/* HDCP events */
#define HDCP_EVENT_STEP1	(1 << 0x0)
#define HDCP_EVENT_STEP2	(1 << 0x1)
#define HDCP_EVENT_EXIT		(1 << 0x2)

/* HDCP user space status */
#define HDCP_US_NO_ERR		(0 << 8)
#define HDCP_US_FAILURE		(1 << 8)

#ifdef __KERNEL__

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/fs.h>

#define _9032_AUTO_RI_	/* Auto Ri mode */
#define _9032_BCAP_	/* BCAP polling */
#undef _9032_AN_STOP_FIX_

#ifdef DEBUG
#define DDC_DBG			/* Log DDC data */
#undef POWER_TRANSITION_DBG	/* Add wait loops to allow testing DSS power
				 * transition during HDCP */
#endif

/***************************/
/* HW specific definitions */
/***************************/

/* DESHDCP base address */
/*----------------------*/

#define DSS_SS_FROM_L3__DESHDCP 0x58007000

/* DESHDCP registers */
#define DESHDCP__DHDCP_CTRL   0x020
#define DESHDCP__DHDCP_DATA_L 0x024
#define DESHDCP__DHDCP_DATA_H 0x028

/* DESHDCP CTRL bits */
#define DESHDCP__DHDCP_CTRL__DIRECTION_POS_F 2
#define DESHDCP__DHDCP_CTRL__DIRECTION_POS_L 2

#define DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F 0
#define DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L 0

/* HDMI WP base address */
/*----------------------*/
#define HDMI_WP			0x58006000

/* HDMI CORE SYSTEM base address */
/*-------------------------------*/

#define HDMI_IP_CORE_SYSTEM 0x400

/* HDMI CORE registers */
#define HDMI_IP_CORE_SYSTEM__DCTL	0x034

#define HDMI_IP_CORE_SYSTEM__HDCP_CTRL	0x03C

#define HDMI_IP_CORE_SYSTEM__BKSV0	0x040

#define HDMI_IP_CORE_SYSTEM__AN0	0x054

#define HDMI_IP_CORE_SYSTEM__AKSV0	0x074

#define HDMI_IP_CORE_SYSTEM__R1		0x088
#define HDMI_IP_CORE_SYSTEM__R2		0x08C

#define HDMI_IP_CORE_SYSTEM__RI_CMD	0x09C
#define HDMI_IP_CORE_SYSTEM__RI_STAT	0x098

#define HDMI_IP_CORE_SYSTEM__INTR2	0x1C8
#define HDMI_IP_CORE_SYSTEM__INTR3	0x1CC

#define HDMI_IP_CORE_SYSTEM__INT_UNMASK2	0x1D8
#define HDMI_IP_CORE_SYSTEM__INT_UNMASK3	0x1DC

#define HDMI_IP_CORE_SYSTEM__SHA_CTRL	0x330

#define HDMI_IP_CORE_SYSTEM__INTR2__BCAP	0x80
#define HDMI_IP_CORE_SYSTEM__INTR3__RI_ERR	0xF0

enum hdcp_repeater {
	HDCP_RECEIVER = 0,
	HDCP_REPEATER = 1
};

enum encryption_state {
	HDCP_ENC_OFF = 0x0,
	HDCP_ENC_ON  = 0x1
};

/* HDMI CORE AV base address */
/*---------------------------*/

#define HDMI_CORE_AV_BASE	0x900
#ifndef HDMI_CORE_AV_HDMI_CTRL
#define HDMI_CORE_AV_HDMI_CTRL  0x0BC
#define HDMI_CORE_AV_PB_CTRL2   0x0FC
#define	HDMI_CORE_AV_CP_BYTE1	0x37C
#endif

#define HDMI_CORE_AV_HDMI_CTRL__HDMI_MODE	0x01

enum av_mute {
	AV_MUTE_SET = 0x01,
	AV_MUTE_CLEAR = 0x10
};
/***********************/
/* HDCP DDC addresses  */
/***********************/

#define DDC_BKSV_ADDR		0x00
#define DDC_Ri_ADDR		0x08
#define DDC_AKSV_ADDR		0x10
#define DDC_AN_ADDR		0x18
#define DDC_V_ADDR		0x20
#define DDC_BCAPS_ADDR		0x40
#define DDC_BSTATUS_ADDR	0x41
#define DDC_KSV_FIFO_ADDR	0x43

#define DDC_BKSV_LEN		5
#define DDC_Ri_LEN		2
#define DDC_AKSV_LEN		5
#define DDC_AN_LEN		8
#define DDC_V_LEN		20
#define DDC_BCAPS_LEN		1
#define DDC_BSTATUS_LEN		2

#define DDC_BIT_REPEATER	6

#define DDC_BSTATUS0_MAX_DEVS	0x80
#define DDC_BSTATUS0_DEV_COUNT	0x7F
#define DDC_BSTATUS1_MAX_CASC	0x08

/***************************/
/* Definitions             */
/***************************/

/* Status / error codes */
#define HDCP_OK			0
#define HDCP_DDC_ERROR		1
#define HDCP_AUTH_FAILURE	2
#define HDCP_AKSV_ERROR		3
#define HDCP_3DES_ERROR		4
#define HDCP_SHA1_ERROR		5
#define HDCP_DRIVER_ERROR	6
#define HDCP_CANCELLED_AUTH	7

#define HDCP_INFINITE_REAUTH	0x100
#define HDCP_MAX_DDC_ERR	5

/* FIXME: should be 300ms delay between HDMI start frame event and HDCP enable
 * (to respect 7 VSYNC delay in 24 Hz)
 */
#define HDCP_ENABLE_DELAY	300
#define HDCP_R0_DELAY		110
#define HDCP_KSV_TIMEOUT_DELAY  5000
#define HDCP_REAUTH_DELAY	100

/* DDC access timeout in ms */
#define HDCP_DDC_TIMEOUT	500
#define HDCP_STOP_FRAME_BLOCKING_TIMEOUT (2*HDCP_DDC_TIMEOUT)

/* Event source */
#define HDCP_SRC_SHIFT		8
#define HDCP_IOCTL_SRC		(0x1 << HDCP_SRC_SHIFT)
#define HDCP_HDMI_SRC		(0x2 << HDCP_SRC_SHIFT)
#define HDCP_IRQ_SRC		(0x4 << HDCP_SRC_SHIFT)
#define HDCP_WORKQUEUE_SRC	(0x8 << HDCP_SRC_SHIFT)

/* Workqueue events */
/* Note: HDCP_ENABLE_CTL, HDCP_R0_EXP_EVENT, HDCP_KSV_TIMEOUT_EVENT and
 * HDCP_AUTH_REATT_EVENT can be cancelled by HDCP disabling
 */
#define HDCP_ENABLE_CTL		(HDCP_IOCTL_SRC		| 0)
#define HDCP_DISABLE_CTL	(HDCP_IOCTL_SRC		| 1)
#define HDCP_START_FRAME_EVENT	(HDCP_HDMI_SRC		| 2)
#define HDCP_STOP_FRAME_EVENT	(HDCP_HDMI_SRC		| 3)
#define HDCP_HPD_LOW_EVENT	(HDCP_IRQ_SRC		| 4)
#define HDCP_RI_FAIL_EVENT	(HDCP_IRQ_SRC		| 5)
#define HDCP_KSV_LIST_RDY_EVENT	(HDCP_IRQ_SRC		| 6)
#define HDCP_R0_EXP_EVENT	(HDCP_WORKQUEUE_SRC	| 7)
#define HDCP_KSV_TIMEOUT_EVENT	(HDCP_WORKQUEUE_SRC	| 8)
#define HDCP_AUTH_REATT_EVENT	(HDCP_WORKQUEUE_SRC	| 9)

/* IRQ status */
#define HDCP_IRQ_RI_FAIL 0x01
#define HDCP_IRQ_KSV_RDY 0x02

enum hdcp_states {
	HDCP_DISABLED,
	HDCP_ENABLE_PENDING,
	HDCP_AUTHENTICATION_START,
	HDCP_WAIT_R0_DELAY,
	HDCP_WAIT_KSV_LIST,
	HDCP_LINK_INTEGRITY_CHECK,
	HDCP_KEY_ENCRYPTION_ONGOING
};

enum hdmi_states {
	HDMI_STOPPED,
	HDMI_STARTED
};

struct hdcp_delayed_work {
	struct delayed_work work;
	int event;
};

struct hdcp {
	void __iomem *hdmi_wp_base_addr;
	void __iomem *deshdcp_base_addr;
	struct mutex lock;
	struct hdcp_enable_control *en_ctrl;
	dev_t dev_id;
	struct class *hdcp_class;
	enum hdmi_states hdmi_state;
	enum hdcp_states hdcp_state;
	int auth_state;
	struct delayed_work *pending_start;
	/* Following variable should store works submitted from workqueue
	 * context
	 * WARNING: only ONE work at a time can be stored (no conflict
	 * should happen). It is used to allow cancelled pending works when
	 * disabling HDCP
	 */
	struct delayed_work *pending_wq_event;
	int retry_cnt;
	int dss_state;
	int pending_disable;
	int hdmi_restart;
	int hpd_low;
	spinlock_t spinlock;
	struct workqueue_struct *workqueue;
	int hdcp_up_event;
	int hdcp_down_event;
	bool hdcp_keys_loaded;
};

extern struct hdcp hdcp;
extern struct hdcp_sha_in sha_input;


/***************************/
/* Macros for accessing HW */
/***************************/

#define WR_REG_32(base, offset, val)	__raw_writel(val, base + offset)
#define RD_REG_32(base, offset)		__raw_readl(base + offset)


#undef FLD_MASK
#define FLD_MASK(start, end)	(((1 << (start - end + 1)) - 1) << (end))
#undef FLD_VAL
#define FLD_VAL(val, start, end) (((val) << end) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define WR_FIELD_32(base, offset, start, end, val) \
	WR_REG_32(base, offset, FLD_MOD(RD_REG_32(base, offset), val, \
		  start, end))

#define RD_FIELD_32(base, offset, start, end) \
	((RD_REG_32(base, offset) & FLD_MASK(start, end)) >> (end))


#undef DBG

#ifdef HDCP_DEBUG
#define DBG(format, ...) \
		printk(KERN_DEBUG "HDCP: " format "\n", ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

/***************************/
/* Function prototypes     */
/***************************/

int hdcp_user_space_task(int flags);

/* 3DES */
int hdcp_3des_load_key(uint32_t *deshdcp_encrypted_key);
void hdcp_3des_encrypt_key(struct hdcp_encrypt_control *enc_ctrl,
			   uint32_t out_key[DESHDCP_KEY_SIZE]);

/* IP control */
int hdcp_lib_disable(void);
int hdcp_lib_step1_start(void);
int hdcp_lib_step1_r0_check(void);
int hdcp_lib_step2(void);
int hdcp_lib_irq(void);
void hdcp_lib_auto_ri_check(bool state);
void hdcp_lib_auto_bcaps_rdy_check(bool state);
void hdcp_lib_set_av_mute(enum av_mute av_mute_state);
void hdcp_lib_set_encryption(enum encryption_state enc_state);
u8 hdcp_lib_check_repeater_bit_in_tx(void);

/* DDC */
int hdcp_ddc_read(u16 no_bytes, u8 addr, u8 *pdata);
int hdcp_ddc_write(u16 no_bytes, u8 addr, u8 *pdata);
void hdcp_ddc_abort(void);

#endif /* __KERNEL__ */

#endif /* _HDCP_H_ */
