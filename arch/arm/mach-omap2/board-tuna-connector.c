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
#include <linux/kernel.h>
#include <linux/platform_data/fsa9480.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/otg.h>

#include <plat/usb.h>

#include "mux.h"
#include "board-tuna.h"

#define GPIO_JACK_INT_N		4
#define GPIO_MHL_SEL		96
#define GPIO_AP_SEL		97
#define GPIO_MUX3_SEL0		139
#define GPIO_MUX3_SEL1		140
#define GPIO_USB_ID_SEL		191
#define GPIO_IF_UART_SEL	101

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

#define TUNA_OTG_ID_FSA9480_PRIO		INT_MIN
#define TUNA_OTG_ID_FSA9480_LAST_PRIO		INT_MAX

struct tuna_otg {
	struct otg_transceiver		otg;
	struct device			dev;

	struct regulator		*vusb;
	bool				reg_on;
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

static void tuna_fsa_usb_detected(int device)
{
	struct tuna_otg *tuna_otg = &tuna_otg_xceiv;

	pr_debug("detected %x\n", device);
	switch (device) {
	case FSA9480_DETECT_USB:
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
		break;
	case FSA9480_DETECT_CHARGER:
		if (omap4_tuna_get_revision() >= 3) {
			tuna_fsa3200_mux_pair(TUNA_USB_MUX_FSA);
		} else {
			tuna_mux_usb(TUNA_USB_MUX_FSA);
			tuna_mux_usb_id(TUNA_USB_MUX_FSA);
		}

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

		if (omap4_tuna_get_revision() >= 3) {
			tuna_fsa3200_mux_pair(TUNA_USB_MUX_FSA);
		} else {
			tuna_mux_usb(TUNA_USB_MUX_FSA);
			tuna_mux_usb_id(TUNA_USB_MUX_FSA);
		}

		tuna_vusb_enable(tuna_otg, false);

		tuna_otg->otg.state = OTG_STATE_B_IDLE;
		tuna_otg->otg.default_a = false;
		tuna_otg->otg.last_event = USB_EVENT_NONE;
		atomic_notifier_call_chain(&tuna_otg->otg.notifier,
					   USB_EVENT_NONE,
					   tuna_otg->otg.gadget);
		break;
	}
}

static struct fsa9480_detect_set fsa_detect_sets[] = {
	{
		.prio = TUNA_OTG_ID_FSA9480_PRIO,
		.mask = FSA9480_DETECT_ALL & ~FSA9480_DETECT_USB_HOST,
	},
	{
		.prio = TUNA_OTG_ID_FSA9480_LAST_PRIO,
		.mask = FSA9480_DETECT_USB_HOST,
		.fallback = true,
	},
};

static struct fsa9480_platform_data tuna_fsa9480_pdata = {
	.detect_time	= 500,
	.detect_sets	= fsa_detect_sets,
	.num_sets	= ARRAY_SIZE(fsa_detect_sets),

	.enable		= tuna_mux_usb_to_fsa,
	.detected	= tuna_fsa_usb_detected,
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

static int tuna_otg_set_vbus(struct otg_transceiver *otg, bool enabled)
{
	dev_dbg(otg->dev, "vbus %s\n", enabled ? "on" : "off");
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

	omap_mux_init_gpio(GPIO_JACK_INT_N,
			   OMAP_PIN_INPUT_PULLUP |
			   OMAP_PIN_OFF_INPUT_PULLUP);

	device_initialize(&tuna_otg->dev);
	dev_set_name(&tuna_otg->dev, "%s", "tuna_otg");
	ret = device_add(&tuna_otg->dev);
	if (ret) {
		pr_err("%s: cannot reg device '%s' (%d)\n", __func__,
		       dev_name(&tuna_otg->dev), ret);
		return ret;
	}

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

	return 0;
}
