/*
 * Copyright (C) 2011 Samsung, Inc.
 * Copyright (C) 2011 Google Inc.
 *
 * Author: Adam Hampson <ahampson@sta.samsung.com>
 *         Dima Zavin <dima@android.com>
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_data/fsa9480.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>
#include <linux/sii9234.h>
#include <linux/i2c/twl.h>
#include <linux/mutex.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

#include <linux/fastchg.h>
#include <plat/usb.h>

#include "mux.h"
#include "board-tuna.h"

#define GPIO_JACK_INT_N		4
#define GPIO_CP_USB_ON		22
#define GPIO_USB_OTG_ID		24
#define GPIO_MHL_SEL		96
#define GPIO_AP_SEL		97
#define GPIO_MUX3_SEL0		139
#define GPIO_MUX3_SEL1		140
#define GPIO_USB_ID_SEL		191
#define GPIO_IF_UART_SEL	101

#define GPIO_MHL_RST		161
#define GPIO_MHL_WAKEUP		64
#define GPIO_MHL_INT		175
#define GPIO_HDMI_EN		100

#define MUX3_SEL0_AP		1
#define MUX3_SEL1_AP		1
#define MUX3_SEL0_MHL		1
#define MUX3_SEL1_MHL		0
#define MUX3_SEL0_FSA		0
#define MUX3_SEL1_FSA		1

#define FSA3200_AP_SEL_AP	0
#define FSA3200_MHL_SEL_AP	0
#define FSA3200_AP_SEL_FSA	1
#define FSA3200_MHL_SEL_FSA	0
#define FSA3200_AP_SEL_MHL	1
#define FSA3200_MHL_SEL_MHL	1

#define USB_ID_SEL_FSA		0
#define USB_ID_SEL_MHL		1

#define IF_UART_SEL_DEFAULT	1
#define IF_UART_SEL_AP		1
#define IF_UART_SEL_CP		0

#define TUNA_MANUAL_USB_NONE	0
#define TUNA_MANUAL_USB_MODEM	1
#define TUNA_MANUAL_USB_AP	2

#define TUNA_MANUAL_UART_NONE	0
#define TUNA_MANUAL_UART_MODEM	1
#define TUNA_MANUAL_UART_LTE	2
#define TUNA_MANUAL_UART_AP	3

#define CHARGERUSB_CTRL1	0x8
#define CHARGERUSB_CTRL3	0xA
#define CHARGERUSB_CINLIMIT	0xE

#define TWL6030_VBUS_IRQ	(TWL6030_IRQ_BASE + USB_PRES_INTR_OFFSET)
#define TWL6030_VBUS_FLAGS	(IRQF_TRIGGER_FALLING | IRQF_ONESHOT)

#define TWL_REG_CONTROLLER_INT_MASK	0x00
#define TWL_CONTROLLER_MVBUS_DET	BIT(1)
#define TWL_CONTROLLER_RSVD		BIT(5)

#define TWL_REG_CONTROLLER_STAT1	0x03
#define TWL_STAT1_VBUS_DET		BIT(2)

struct tuna_otg {
	struct otg_transceiver		otg;
	struct device			dev;

	struct regulator		*vusb;
	struct work_struct		set_vbus_work;
	struct mutex			lock;

	bool				reg_on;
	bool				need_vbus_drive;
	int				usb_manual_mode;
	int				uart_manual_mode;
	int				current_device;

	struct switch_dev		dock_switch;
};
static struct tuna_otg tuna_otg_xceiv;

enum {
	TUNA_USB_MUX_FSA = 0,
	TUNA_USB_MUX_MHL,
	TUNA_USB_MUX_AP,
	NUM_TUNA_USB_MUX,

	TUNA_USB_MUX_DEFAULT = TUNA_USB_MUX_FSA,
};

static struct {
	int mux3_sel0;
	int mux3_sel1;
} tuna_usb_mux_states[] = {
	[TUNA_USB_MUX_FSA] = { MUX3_SEL0_FSA, MUX3_SEL1_FSA },
	[TUNA_USB_MUX_MHL] = { MUX3_SEL0_MHL, MUX3_SEL1_MHL },
	[TUNA_USB_MUX_AP] = { MUX3_SEL0_AP, MUX3_SEL1_AP },
};

static struct {
	int ap_sel;
	int mhl_sel;
} tuna_fsa3200_mux_pair_states[] = {
	[TUNA_USB_MUX_FSA] = { FSA3200_AP_SEL_FSA, FSA3200_MHL_SEL_FSA },
	[TUNA_USB_MUX_MHL] = { FSA3200_AP_SEL_MHL, FSA3200_MHL_SEL_MHL },
	[TUNA_USB_MUX_AP] = { FSA3200_AP_SEL_AP, FSA3200_MHL_SEL_AP },
};

static int tuna_usb_id_mux_states[] = {
	[TUNA_USB_MUX_FSA] = USB_ID_SEL_FSA,
	[TUNA_USB_MUX_MHL] = USB_ID_SEL_MHL,
	[TUNA_USB_MUX_AP] = USB_ID_SEL_FSA,
};

static ssize_t tuna_otg_usb_sel_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf);
static ssize_t tuna_otg_usb_sel_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size);
static ssize_t tuna_otg_uart_switch_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
static ssize_t tuna_otg_uart_switch_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size);

static DEVICE_ATTR(usb_sel, S_IRUSR | S_IWUSR,
			tuna_otg_usb_sel_show, tuna_otg_usb_sel_store);
static DEVICE_ATTR(uart_sel, S_IRUSR | S_IWUSR,
			tuna_otg_uart_switch_show, tuna_otg_uart_switch_store);

static struct attribute *manual_mode_attributes[] = {
	&dev_attr_usb_sel.attr,
	&dev_attr_uart_sel.attr,
	NULL,
};

static const struct attribute_group manual_mode_group = {
	.attrs = manual_mode_attributes,
};

static bool tuna_twl_chgctrl_init;

static void tuna_mux_usb(int state)
{
	BUG_ON(state >= NUM_TUNA_USB_MUX);

	pr_debug("mux to %d\n", state);
	gpio_direction_output(GPIO_MUX3_SEL0,
			      tuna_usb_mux_states[state].mux3_sel0);
	gpio_direction_output(GPIO_MUX3_SEL1,
			      tuna_usb_mux_states[state].mux3_sel1);
}

static void tuna_mux_usb_id(int state)
{
	BUG_ON(state >= NUM_TUNA_USB_MUX);

	pr_debug("mux to %d\n", state);
	gpio_direction_output(GPIO_USB_ID_SEL, tuna_usb_id_mux_states[state]);
}

static void tuna_fsa3200_mux_pair(int state)
{
	BUG_ON(state >= NUM_TUNA_USB_MUX);

	pr_debug("mux to %d\n", state);
	gpio_direction_output(GPIO_AP_SEL,
			      tuna_fsa3200_mux_pair_states[state].ap_sel);
	gpio_direction_output(GPIO_MHL_SEL,
			      tuna_fsa3200_mux_pair_states[state].mhl_sel);
}

static void tuna_mux_usb_to_fsa(bool enable)
{
	if (omap4_tuna_get_revision() >= 3) {
		tuna_fsa3200_mux_pair(enable ? TUNA_USB_MUX_FSA :
				TUNA_USB_MUX_DEFAULT);
	} else {
		tuna_mux_usb(enable ? TUNA_USB_MUX_FSA : TUNA_USB_MUX_DEFAULT);

		/* When switching ID away from FSA, we want to ensure we switch
		 * it off FSA, and force it to MHL. Ideally, we'd just say mux
		 * to default, but FSA is likely the default mux position and
		 * there's no way to force the ID pin to float to the FSA.
		 */
		tuna_mux_usb_id(enable ? TUNA_USB_MUX_FSA : TUNA_USB_MUX_MHL);
	}
}

static void tuna_mux_usb_to_mhl(bool enable)
{
	if (omap4_tuna_get_revision() >= 3) {
		tuna_fsa3200_mux_pair(enable ? TUNA_USB_MUX_MHL :
				TUNA_USB_MUX_DEFAULT);
	} else {
		tuna_mux_usb(enable ? TUNA_USB_MUX_MHL : TUNA_USB_MUX_DEFAULT);
		tuna_mux_usb_id(enable ? TUNA_USB_MUX_MHL : TUNA_USB_MUX_DEFAULT);
	}
}

static void tuna_vusb_enable(struct tuna_otg *tuna_otg, bool enable)
{
	/* delay getting the regulator until later */
	if (IS_ERR_OR_NULL(tuna_otg->vusb)) {
		tuna_otg->vusb = regulator_get(&tuna_otg->dev, "vusb");
		if (IS_ERR(tuna_otg->vusb)) {
			dev_err(&tuna_otg->dev, "cannot get vusb regulator\n");
			return;
		}
	}

	if (enable) {
		regulator_enable(tuna_otg->vusb);
		tuna_otg->reg_on = true;
	} else if (tuna_otg->reg_on) {
		regulator_disable(tuna_otg->vusb);
		tuna_otg->reg_on = false;
	}
}

static void tuna_set_vbus_drive(bool enable)
{
	if (enable) {
		/* Set the VBUS current limit to 500mA */
		twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, 0x09,
				CHARGERUSB_CINLIMIT);

		/* The TWL6030 has a feature to automatically turn on
		 * boost mode (VBUS Drive) when the ID signal is not
		 * grounded.  This feature needs to be disabled on Tuna
		 * as the ID signal is not hooked up to the TWL6030.
		 */
		twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, 0x21,
				CHARGERUSB_CTRL3);
		twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, 0x40,
				CHARGERUSB_CTRL1);
	} else {
		twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, 0x01,
				CHARGERUSB_CTRL3);
		twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, 0, CHARGERUSB_CTRL1);
	}
}

static void tuna_ap_usb_attach(struct tuna_otg *tuna_otg)
{
	tuna_vusb_enable(tuna_otg, true);

	if (omap4_tuna_get_revision() >= 3) {
		tuna_fsa3200_mux_pair(TUNA_USB_MUX_AP);
	} else {
		tuna_mux_usb(TUNA_USB_MUX_AP);
		tuna_mux_usb_id(TUNA_USB_MUX_FSA);
	}

	tuna_otg->otg.state = OTG_STATE_B_IDLE;
	tuna_otg->otg.default_a = false;
	tuna_otg->otg.last_event = USB_EVENT_VBUS;
	atomic_notifier_call_chain(&tuna_otg->otg.notifier,
				   USB_EVENT_VBUS,
				   tuna_otg->otg.gadget);
}

static void tuna_ap_usb_detach(struct tuna_otg *tuna_otg)
{
	tuna_vusb_enable(tuna_otg, false);

	tuna_otg->otg.state = OTG_STATE_B_IDLE;
	tuna_otg->otg.default_a = false;
	tuna_otg->otg.last_event = USB_EVENT_NONE;
	atomic_notifier_call_chain(&tuna_otg->otg.notifier,
				   USB_EVENT_NONE,
				   tuna_otg->otg.gadget);
}

static void tuna_cp_usb_attach(struct tuna_otg *tuna_otg)
{
	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO)
		gpio_set_value(GPIO_CP_USB_ON, 1);

	tuna_mux_usb_to_fsa(true);
}

static void tuna_cp_usb_detach(struct tuna_otg *tuna_otg)
{
	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO)
		gpio_set_value(GPIO_CP_USB_ON, 0);
}

static void tuna_usb_host_detach(struct tuna_otg *tuna_otg)
{
	/* Make sure the VBUS drive is turned off */
	tuna_set_vbus_drive(false);

	tuna_vusb_enable(tuna_otg, false);

	tuna_otg->otg.state = OTG_STATE_B_IDLE;
	tuna_otg->otg.default_a = false;
	tuna_otg->otg.last_event = USB_EVENT_NONE;
	atomic_notifier_call_chain(&tuna_otg->otg.notifier,
				   USB_EVENT_NONE,
				   tuna_otg->otg.gadget);
}

static void tuna_ap_uart_actions(struct tuna_otg *tuna_otg)
{
	tuna_mux_usb_to_fsa(true);
	gpio_set_value(GPIO_IF_UART_SEL, IF_UART_SEL_AP);
}

static void tuna_cp_uart_actions(struct tuna_otg *tuna_otg)
{
	tuna_mux_usb_to_fsa(true);
	gpio_set_value(GPIO_IF_UART_SEL, IF_UART_SEL_CP);
}

static void tuna_lte_uart_actions(struct tuna_otg *tuna_otg)
{
	tuna_mux_usb_to_fsa(true);

	/* The LTE modem's UART lines are connected to the V_AUDIO_L and
	 * V_AUDIO_R pins on the FSA9480.  The RIL will configure the FSA9480
	 * separately to set manual routing.
	 */
}

static void tuna_otg_mask_vbus_irq(void)
{
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);
}

static void tuna_otg_unmask_vbus_irq(void)
{
	if (!tuna_twl_chgctrl_init) {
		int r;

		r = twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE,
				     (u8) ~(TWL_CONTROLLER_RSVD |
					    TWL_CONTROLLER_MVBUS_DET),
				     TWL_REG_CONTROLLER_INT_MASK);

		if (r)
			pr_err_once("%s: Error writing twl charge ctrl int mask\n",
				    __func__);
		else
			tuna_twl_chgctrl_init = true;
	}

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);
}

static bool tuna_otg_vbus_present(void)
{
	u8 vbus_state;

	twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &vbus_state,
				TWL_REG_CONTROLLER_STAT1);

	return !!(vbus_state & TWL_STAT1_VBUS_DET);
}

static void tuna_fsa_usb_detected(int device)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;
	int old_device;

	mutex_lock(&tuna_otg->lock);

	old_device = tuna_otg->current_device;
	tuna_otg->current_device = device;

	pr_debug("detected %x\n", device);
	switch (device) {
	case FSA9480_DETECT_AV_365K_CHARGER:
		tuna_otg_set_dock_switch(1);
		/* intentional fall-through */
	case FSA9480_DETECT_USB:
		if (tuna_otg->usb_manual_mode == TUNA_MANUAL_USB_MODEM)
			tuna_cp_usb_attach(tuna_otg);
		else
			tuna_ap_usb_attach(tuna_otg);
		break;
	case FSA9480_DETECT_AV_POWERED:
		tuna_ap_usb_attach(tuna_otg);
		break;
	case FSA9480_DETECT_CHARGER:
		tuna_mux_usb_to_fsa(true);

		tuna_otg->otg.state = OTG_STATE_B_IDLE;
		tuna_otg->otg.default_a = false;
		tuna_otg->otg.last_event = USB_EVENT_CHARGER;
		atomic_notifier_call_chain(&tuna_otg->otg.notifier,
					   USB_EVENT_CHARGER,
					   tuna_otg->otg.gadget);
		break;
	case FSA9480_DETECT_USB_HOST:
		tuna_vusb_enable(tuna_otg, true);

		if (omap4_tuna_get_revision() >= 3) {
			tuna_fsa3200_mux_pair(TUNA_USB_MUX_AP);
		} else {
			tuna_mux_usb(TUNA_USB_MUX_AP);
			tuna_mux_usb_id(TUNA_USB_MUX_FSA);
		}

		tuna_otg->otg.state = OTG_STATE_A_IDLE;
		tuna_otg->otg.default_a = true;
		tuna_otg->otg.last_event = USB_EVENT_ID;
		atomic_notifier_call_chain(&tuna_otg->otg.notifier,
					   USB_EVENT_ID,
					   tuna_otg->otg.gadget);
		break;
	case FSA9480_DETECT_NONE:
		tuna_mux_usb_to_fsa(true);

		switch (old_device) {
		case FSA9480_DETECT_JIG:
			if (tuna_otg->uart_manual_mode == TUNA_MANUAL_UART_NONE)
				tuna_ap_uart_actions(tuna_otg);
			break;
		case FSA9480_DETECT_AV_365K_CHARGER:
			tuna_otg_set_dock_switch(0);
			/* intentional fall-through */
		case FSA9480_DETECT_USB:
			if (tuna_otg->usb_manual_mode == TUNA_MANUAL_USB_MODEM)
				tuna_cp_usb_detach(tuna_otg);
			else
				tuna_ap_usb_detach(tuna_otg);
			break;
		case FSA9480_DETECT_AV_POWERED:
			tuna_ap_usb_detach(tuna_otg);
			break;
		case FSA9480_DETECT_USB_HOST:
			tuna_usb_host_detach(tuna_otg);
			break;
		case FSA9480_DETECT_CHARGER:
			tuna_ap_usb_detach(tuna_otg);
			break;
		case FSA9480_DETECT_AV_365K:
			tuna_otg_set_dock_switch(0);
			break;
		case FSA9480_DETECT_UART:
		default:
			break;
		};
		break;
	case FSA9480_DETECT_JIG:
		switch (tuna_otg->uart_manual_mode) {
		case TUNA_MANUAL_UART_AP:
			tuna_ap_uart_actions(tuna_otg);
			break;
		case TUNA_MANUAL_UART_LTE:
			tuna_lte_uart_actions(tuna_otg);
			break;
		case TUNA_MANUAL_UART_MODEM:
		default:
			tuna_cp_uart_actions(tuna_otg);
			break;
		};
		break;
	case FSA9480_DETECT_AV_365K:
		tuna_otg_set_dock_switch(1);
		break;
	case FSA9480_DETECT_UART:
	default:
		break;
	}

	mutex_unlock(&tuna_otg->lock);
}

static struct fsa9480_detect_set fsa_detect_sets[] = {
	{
		.prio = TUNA_OTG_ID_FSA9480_PRIO,
		.mask = FSA9480_DETECT_ALL & ~FSA9480_DETECT_AV_POWERED,
	},
	{
		.prio = TUNA_OTG_ID_SII9234_FAILED_PRIO,
		.mask = FSA9480_DETECT_AV_POWERED,
	},
	{
		.prio = TUNA_OTG_ID_FSA9480_LAST_PRIO,
		.mask = 0,
		.fallback = true,
	},
};

static struct fsa9480_platform_data tuna_fsa9480_pdata = {
	.detect_time		= 500,
	.detect_sets		= fsa_detect_sets,
	.num_sets		= ARRAY_SIZE(fsa_detect_sets),

	.enable			= tuna_mux_usb_to_fsa,
	.detected		= tuna_fsa_usb_detected,
	.external_id		= GPIO_USB_OTG_ID,

	.external_vbus_irq	= TWL6030_VBUS_IRQ,
	.external_vbus_flags	= TWL6030_VBUS_FLAGS,
	.mask_vbus_irq		= tuna_otg_mask_vbus_irq,
	.unmask_vbus_irq	= tuna_otg_unmask_vbus_irq,
	.vbus_present		= tuna_otg_vbus_present,
};

static struct i2c_board_info __initdata tuna_connector_i2c4_boardinfo[] = {
	{
		I2C_BOARD_INFO("fsa9480", 0x4A >> 1),
		.irq = OMAP_GPIO_IRQ(GPIO_JACK_INT_N),
		.platform_data = &tuna_fsa9480_pdata,
	},
};

static int tuna_otg_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	otg->host = host;
	if (!host)
		otg->state = OTG_STATE_UNDEFINED;
	return 0;
}

static int tuna_otg_set_peripheral(struct otg_transceiver *otg,
				   struct usb_gadget *gadget)
{
	otg->gadget = gadget;
	if (!gadget)
		otg->state = OTG_STATE_UNDEFINED;
	return 0;
}

static void tuna_otg_work(struct work_struct *data)
{
	struct tuna_otg *tuna_otg = container_of(data, struct tuna_otg,
						set_vbus_work);

	mutex_lock(&tuna_otg->lock);

	/* Only allow VBUS drive when in host mode. */
	if (tuna_otg->current_device != FSA9480_DETECT_USB_HOST) {
		mutex_unlock(&tuna_otg->lock);
		return;
	}

	tuna_set_vbus_drive(tuna_otg->need_vbus_drive);

	mutex_unlock(&tuna_otg->lock);
}

static int tuna_otg_set_vbus(struct otg_transceiver *otg, bool enabled)
{
	struct tuna_otg *tuna_otg = container_of(otg, struct tuna_otg, otg);

	dev_dbg(otg->dev, "vbus %s\n", enabled ? "on" : "off");

	tuna_otg->need_vbus_drive = enabled;
	schedule_work(&tuna_otg->set_vbus_work);

	return 0;
}

static int tuna_otg_phy_init(struct otg_transceiver *otg)
{
	if (otg->last_event == USB_EVENT_ID)
		omap4430_phy_power(otg->dev, 1, 1);
	else
		omap4430_phy_power(otg->dev, 0, 1);
	return 0;
}

static void tuna_otg_phy_shutdown(struct otg_transceiver *otg)
{
	omap4430_phy_power(otg->dev, 0, 0);
}

static int tuna_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	return omap4430_phy_suspend(otg->dev, suspend);
}

static ssize_t tuna_otg_usb_sel_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct tuna_otg *tuna_otg = dev_get_drvdata(dev);
	const char* mode;

	switch (tuna_otg->usb_manual_mode) {
	case TUNA_MANUAL_USB_AP:
		mode = "PDA";
		break;
	case TUNA_MANUAL_USB_MODEM:
		mode = "MODEM";
		break;
	default:
		mode = "NONE";
	};

	return sprintf(buf, "%s\n", mode);
}

static ssize_t tuna_otg_usb_sel_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tuna_otg *tuna_otg = dev_get_drvdata(dev);
	int old_mode;

	mutex_lock(&tuna_otg->lock);

	old_mode = tuna_otg->usb_manual_mode;

	if (!strncasecmp(buf, "PDA", 3)) {
		tuna_otg->usb_manual_mode = TUNA_MANUAL_USB_AP;

		/* If we are transitioning from CP USB to AP USB then notify the
		 * USB stack that is now attached.
		 */
		if (tuna_otg->current_device == FSA9480_DETECT_USB &&
				old_mode == TUNA_MANUAL_USB_MODEM) {
			tuna_cp_usb_detach(tuna_otg);
			tuna_ap_usb_attach(tuna_otg);
		}
	} else if (!strncasecmp(buf, "MODEM", 5)) {
		tuna_otg->usb_manual_mode = TUNA_MANUAL_USB_MODEM;

		/* If we are transitioning from AP USB to CP USB then notify the
		 * USB stack that is has been detached.
		 */
		if (tuna_otg->current_device == FSA9480_DETECT_USB &&
				(old_mode == TUNA_MANUAL_USB_AP ||
				old_mode == TUNA_MANUAL_USB_NONE)) {
			tuna_ap_usb_detach(tuna_otg);
			tuna_cp_usb_attach(tuna_otg);
		}
	} else if (!strncasecmp(buf, "NONE", 4)) {
		tuna_otg->usb_manual_mode = TUNA_MANUAL_USB_NONE;

		/* If we are transitioning from CP USB to AP USB then notify the
		 * USB stack that it is now attached.
		 */
		if (tuna_otg->current_device == FSA9480_DETECT_USB &&
				old_mode == TUNA_MANUAL_USB_MODEM) {
			tuna_cp_usb_detach(tuna_otg);
			tuna_ap_usb_attach(tuna_otg);
		}
	}

	mutex_unlock(&tuna_otg->lock);

	return size;
}

static ssize_t tuna_otg_uart_switch_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct tuna_otg *tuna_otg = dev_get_drvdata(dev);
	const char* mode;

	switch (tuna_otg->uart_manual_mode) {
	case TUNA_MANUAL_UART_AP:
		mode = "PDA";
		break;
	case TUNA_MANUAL_UART_MODEM:
		mode = "MODEM";
		break;
	case TUNA_MANUAL_UART_LTE:
		mode = "LTEMODEM";
		break;
	default:
		mode = "NONE";
	};

	return sprintf(buf, "%s\n", mode);
}

static ssize_t tuna_otg_uart_switch_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct tuna_otg *tuna_otg = dev_get_drvdata(dev);

	mutex_lock(&tuna_otg->lock);

	if (!strncasecmp(buf, "PDA", 3)) {
		tuna_otg->uart_manual_mode = TUNA_MANUAL_UART_AP;

		if (tuna_otg->current_device == FSA9480_DETECT_JIG)
			tuna_ap_uart_actions(tuna_otg);
	} else if (!strncasecmp(buf, "MODEM", 5)) {
		tuna_otg->uart_manual_mode = TUNA_MANUAL_UART_MODEM;

		if (tuna_otg->current_device == FSA9480_DETECT_JIG)
			tuna_cp_uart_actions(tuna_otg);
	} else if (!strncasecmp(buf, "LTEMODEM", 8) &&
			omap4_tuna_get_type() == TUNA_TYPE_TORO) {
		tuna_otg->uart_manual_mode = TUNA_MANUAL_UART_LTE;

		if (tuna_otg->current_device == FSA9480_DETECT_JIG)
			tuna_lte_uart_actions(tuna_otg);
	} else if (!strncasecmp(buf, "NONE", 4)) {
		tuna_otg->uart_manual_mode = TUNA_MANUAL_UART_NONE;

		if (tuna_otg->current_device == FSA9480_DETECT_JIG)
			tuna_ap_uart_actions(tuna_otg);
	}

	mutex_unlock(&tuna_otg->lock);

	return size;
}

static struct wake_lock sii9234_wake_lock;

#define OMAP_HDMI_HPD_ADDR	0x4A100098
#define OMAP_HDMI_PULLTYPE_MASK	0x00000010
static void sii9234_power(int on)
{
	struct omap_mux_partition *p = omap_mux_get("core");

	u16 mux;

	mux = omap_mux_read(p, OMAP4_CTRL_MODULE_PAD_HDMI_HPD_OFFSET);

	if (on) {
		gpio_set_value(GPIO_HDMI_EN, 1);
		msleep(20);
		gpio_set_value(GPIO_MHL_RST, 1);

		omap_mux_write(p, mux | OMAP_PULL_UP,
			       OMAP4_CTRL_MODULE_PAD_HDMI_HPD_OFFSET);
	} else {
		omap_mux_write(p, mux & ~OMAP_PULL_UP,
			       OMAP4_CTRL_MODULE_PAD_HDMI_HPD_OFFSET);

		gpio_set_value(GPIO_HDMI_EN, 0);
		gpio_set_value(GPIO_MHL_RST, 0);

	}
}

static void sii9234_enable_vbus(bool enable)
{

}

static void sii9234_connect(bool on, u8 *devcap)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;
	unsigned long val;
	int dock = 0;

	if (on) {
#ifdef CONFIG_FORCE_FAST_CHARGE
		val = (force_fast_charge !=0) ? USB_EVENT_CHARGER : USB_EVENT_VBUS;
#else
		val = USB_EVENT_VBUS;
#endif
		if (devcap) {
			u16 adopter_id =
				(devcap[MHL_DEVCAP_ADOPTER_ID_H] << 8) |
				devcap[MHL_DEVCAP_ADOPTER_ID_L];
			u16 device_id =
				(devcap[MHL_DEVCAP_DEVICE_ID_H] << 8) |
				devcap[MHL_DEVCAP_DEVICE_ID_L];

			if (adopter_id == 0x3333 || adopter_id == 321) {
				if (devcap[MHL_DEVCAP_RESERVED] == 2)
					val = USB_EVENT_CHARGER;

				if (device_id == 0x1234)
					dock = 1;
			}
		}

		wake_lock(&sii9234_wake_lock);
	} else {
		wake_unlock(&sii9234_wake_lock);
		val = USB_EVENT_NONE;
	}

	tuna_otg->otg.state = OTG_STATE_B_IDLE;
	tuna_otg->otg.default_a = false;
	tuna_otg->otg.last_event = val;

	atomic_notifier_call_chain(&tuna_otg->otg.notifier,
				   val, tuna_otg->otg.gadget);
	tuna_otg_set_dock_switch(dock);

}

void tuna_otg_pogo_charger(enum pogo_power_state pogo_state)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;
	unsigned long power_state;

	switch (pogo_state) {
		case POGO_POWER_CHARGER:
			power_state = USB_EVENT_CHARGER;
			break;
		case POGO_POWER_HOST:
			power_state = USB_EVENT_VBUS;
			break;
		case POGO_POWER_DISCONNECTED:
		default:
			power_state = USB_EVENT_NONE;
			break;
	}

	tuna_otg->otg.state = OTG_STATE_B_IDLE;
	tuna_otg->otg.default_a = false;
	tuna_otg->otg.last_event = power_state;
	atomic_notifier_call_chain(&tuna_otg->otg.notifier, power_state,
				tuna_otg->otg.gadget);
}

void tuna_otg_set_dock_switch(int enable)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;

	switch_set_state(&tuna_otg->dock_switch, enable);
}

static struct sii9234_platform_data sii9234_pdata = {
	.prio = TUNA_OTG_ID_SII9234_PRIO,
	.enable = tuna_mux_usb_to_mhl,
	.power = sii9234_power,
	.enable_vbus = sii9234_enable_vbus,
	.connect = sii9234_connect,
};

static struct i2c_board_info __initdata tuna_i2c5_boardinfo[] = {
	{
		I2C_BOARD_INFO("sii9234_mhl_tx", 0x72>>1),
		.irq = OMAP_GPIO_IRQ(GPIO_MHL_INT),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_tpi", 0x7A>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_hdmi_rx", 0x92>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_cbus", 0xC8>>1),
		.platform_data = &sii9234_pdata,
	},
};

int __init omap4_tuna_connector_init(void)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;
	int ret;

	if (omap4_tuna_get_revision() >= 3) {
		gpio_request(GPIO_MHL_SEL, "fsa3200_mhl_sel");
		gpio_request(GPIO_AP_SEL, "fsa3200_ap_sel");

		tuna_fsa3200_mux_pair(TUNA_USB_MUX_DEFAULT);

		omap_mux_init_gpio(GPIO_MHL_SEL, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(GPIO_AP_SEL, OMAP_PIN_OUTPUT);
	} else {
		gpio_request(GPIO_MUX3_SEL0, "usb_mux3_sel0");
		gpio_request(GPIO_MUX3_SEL1, "usb_mux3_sel1");
		gpio_request(GPIO_USB_ID_SEL, "usb_id_sel");

		tuna_mux_usb(TUNA_USB_MUX_DEFAULT);
		tuna_mux_usb_id(TUNA_USB_MUX_DEFAULT);

		omap_mux_init_gpio(GPIO_MUX3_SEL0, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(GPIO_MUX3_SEL1, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(GPIO_USB_ID_SEL, OMAP_PIN_OUTPUT);
	}

	if (omap4_tuna_get_type() == TUNA_TYPE_MAGURO) {
		gpio_request(GPIO_CP_USB_ON, "cp_usb_on");
		omap_mux_init_gpio(GPIO_CP_USB_ON, OMAP_PIN_OUTPUT);
		gpio_direction_output(GPIO_CP_USB_ON, 0);
	}

	omap_mux_init_gpio(GPIO_IF_UART_SEL, OMAP_PIN_OUTPUT);
	gpio_request(GPIO_IF_UART_SEL, "uart_sel");
	gpio_direction_output(GPIO_IF_UART_SEL, IF_UART_SEL_DEFAULT);

	omap_mux_init_gpio(GPIO_USB_OTG_ID, OMAP_PIN_INPUT |
			   OMAP_WAKEUP_EN);

	omap_mux_init_gpio(GPIO_JACK_INT_N,
			   OMAP_PIN_INPUT_PULLUP |
			   OMAP_PIN_OFF_INPUT_PULLUP);

	wake_lock_init(&sii9234_wake_lock, WAKE_LOCK_SUSPEND, "sii9234(mhl)");

	mutex_init(&tuna_otg->lock);

	INIT_WORK(&tuna_otg->set_vbus_work, tuna_otg_work);

	device_initialize(&tuna_otg->dev);
	dev_set_name(&tuna_otg->dev, "%s", "tuna_otg");
	ret = device_add(&tuna_otg->dev);
	if (ret) {
		pr_err("%s: cannot reg device '%s' (%d)\n", __func__,
		       dev_name(&tuna_otg->dev), ret);
		return ret;
	}

	dev_set_drvdata(&tuna_otg->dev, tuna_otg);

	tuna_otg->otg.dev		= &tuna_otg->dev;
	tuna_otg->otg.label		= "tuna_otg_xceiv";
	tuna_otg->otg.set_host		= tuna_otg_set_host;
	tuna_otg->otg.set_peripheral	= tuna_otg_set_peripheral;
	tuna_otg->otg.set_suspend	= tuna_otg_set_suspend;
	tuna_otg->otg.set_vbus		= tuna_otg_set_vbus;
	tuna_otg->otg.init		= tuna_otg_phy_init;
	tuna_otg->otg.shutdown		= tuna_otg_phy_shutdown;

	ATOMIC_INIT_NOTIFIER_HEAD(&tuna_otg->otg.notifier);

	ret = otg_set_transceiver(&tuna_otg->otg);
	if (ret)
		pr_err("tuna_otg: cannot set transceiver (%d)\n", ret);

	omap4430_phy_init(&tuna_otg->dev);
	tuna_otg_set_suspend(&tuna_otg->otg, 0);

	i2c_register_board_info(4, tuna_connector_i2c4_boardinfo,
				ARRAY_SIZE(tuna_connector_i2c4_boardinfo));

	ret = sysfs_create_group(&tuna_otg->dev.kobj, &manual_mode_group);
	if (ret)
		pr_err("tuna_otg: Unable to create manual mode sysfs group"
			"(%d)\n", ret);

	gpio_request(GPIO_HDMI_EN, NULL);
	omap_mux_init_gpio(GPIO_HDMI_EN, OMAP_PIN_OUTPUT);
	gpio_direction_output(GPIO_HDMI_EN, 0);

	gpio_request(GPIO_MHL_RST, NULL);
	omap_mux_init_gpio(GPIO_MHL_RST, OMAP_PIN_OUTPUT);
	gpio_direction_output(GPIO_MHL_RST, 0);

	gpio_request(GPIO_MHL_INT, NULL);
	omap_mux_init_gpio(GPIO_MHL_INT, OMAP_PIN_INPUT);
	gpio_direction_input(GPIO_MHL_INT);

	gpio_request(TUNA_GPIO_HDMI_HPD, NULL);
	omap_mux_init_gpio(TUNA_GPIO_HDMI_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(TUNA_GPIO_HDMI_HPD);

	i2c_register_board_info(5, tuna_i2c5_boardinfo,
			ARRAY_SIZE(tuna_i2c5_boardinfo));

	tuna_otg->dock_switch.name = "dock";
	switch_dev_register(&tuna_otg->dock_switch);

	return 0;
}
