/*
 * Copyright (C) 2011 Samsung, Inc.
 *
 * Author: Adam Hampson <ahampson@sta.samsung.com>
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/usb/otg.h>

#include <asm/div64.h>

#include "mux.h"
#include "control.h"
#include "board-tuna.h"

#define GPIO_POGO_DATA		121
#define GPIO_POGO_DET		169

/* The below constants are in milliseconds */
#define POGO_WAKE_PERIOD		100
#define POGO_ID_PERIOD_TIMEOUT		750
#define POGO_ID_CARDOCK			100
#define POGO_ID_DESKDOCK		200
#define POGO_ENTER_SPDIF_PERIOD		200
#define POGO_ENTER_SPDIF_WAIT_PERIOD	50
#define POGO_ID_PERIOD_TOLERANCE	20

#define POGO_DOCK_ID_MAX_RETRY		10

#define POGO_AUDIO_DISCONNECTED		0
#define POGO_AUDIO_CONNECTED		2

#define POGO_DOCK_UNDOCKED		0
#define POGO_DOCK_DESK			1
#define POGO_DOCK_CAR			2


struct tuna_pogo {
	struct switch_dev		dock_switch;
	struct switch_dev		audio_switch;
	struct wake_lock		wake_lock;
	struct completion		completion;
	struct timespec			rise_time;
	struct timespec			fall_time;
	int				det_irq;
	int				data_irq;
	int				dock_type;
	bool				fall_detected;
};
static struct tuna_pogo tuna_pogo;

static void pogo_send_pulse(unsigned int duration_in_ms)
{
	gpio_direction_output(GPIO_POGO_DATA, 1);
	msleep(duration_in_ms);
	gpio_direction_output(GPIO_POGO_DATA, 0);
}

static int pogo_read_id_period(struct tuna_pogo *pogo,
		unsigned int timeout_in_ms)
{
	struct timespec temp;
	int ret;

	pogo->rise_time.tv_sec = 0;
	pogo->rise_time.tv_nsec = 0;
	pogo->fall_time.tv_sec = 0;
	pogo->fall_time.tv_nsec = 0;
	pogo->fall_detected = false;

	gpio_direction_input(GPIO_POGO_DATA);

	enable_irq(pogo->data_irq);

	ret = wait_for_completion_timeout(&pogo->completion,
			msecs_to_jiffies(timeout_in_ms));
	if (ret <= 0) {
		if (!pogo->fall_detected)
			pr_debug("No response to wake within timeout\n");
		else
			pr_debug("ID period did not conclude within timeout\n");
		disable_irq(pogo->data_irq);
		return -1;
	}

	temp = timespec_sub(pogo->fall_time, pogo->rise_time);
	return temp.tv_nsec / NSEC_PER_MSEC;
}

static irqreturn_t pogo_det_irq_thread(int irq, void *data)
{
	struct tuna_pogo *pogo = data;
	int id_period;
	unsigned int retry = 0;

	if (gpio_get_value(GPIO_POGO_DET)) {
		wake_lock(&pogo->wake_lock);

		while (gpio_get_value(GPIO_POGO_DET) &&
				retry++ <= POGO_DOCK_ID_MAX_RETRY) {

			/* Start the detection process by sending a wake pulse
			 * to the dock.
			 */
			pogo_send_pulse(POGO_WAKE_PERIOD);

			id_period = pogo_read_id_period(pogo,
					POGO_ID_PERIOD_TIMEOUT);
			if (id_period == -1)
				continue;

			/* The length of the ID period will indicate the type of
			 * dock that is attached.
			 */
			if (abs(id_period - POGO_ID_CARDOCK) <=
					POGO_ID_PERIOD_TOLERANCE) {
				pr_info("POGO Car Dock Detected, ID period"
						" %dms\n",
						id_period);

				tuna_otg_pogo_charger(true);

				pogo->dock_type = POGO_DOCK_CAR;
				switch_set_state(&pogo->dock_switch,
						POGO_DOCK_CAR);
				switch_set_state(&pogo->audio_switch,
						POGO_AUDIO_CONNECTED);
				break;
			} else if (abs(id_period - POGO_ID_DESKDOCK) <=
					POGO_ID_PERIOD_TOLERANCE) {
				pr_info("POGO Desk Dock Detected, ID period"
						" %dms\n",
						id_period);

				tuna_otg_pogo_charger(true);

				pogo->dock_type = POGO_DOCK_DESK;
				switch_set_state(&pogo->dock_switch,
						POGO_DOCK_DESK);
				switch_set_state(&pogo->audio_switch,
					POGO_AUDIO_CONNECTED);
				break;
			} else {
				pr_err("Unknown POGO dock detected, ID period"
						" %ums\n",
						id_period);
			}
		}

		if (pogo->dock_type == POGO_DOCK_UNDOCKED) {
			wake_unlock(&pogo->wake_lock);
			pr_err("Unable to identify pogo dock, giving up\n");
			return IRQ_HANDLED;
		}

		/* Instruct the dock to enter SPDIF mode */
		pogo_send_pulse(POGO_ENTER_SPDIF_PERIOD);

		msleep(POGO_ENTER_SPDIF_WAIT_PERIOD);

		omap_mux_set_gpio(OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT,
				GPIO_POGO_DATA);

		wake_unlock(&pogo->wake_lock);
	} else {
		if (pogo->dock_type != POGO_DOCK_UNDOCKED) {
			pogo->dock_type = POGO_DOCK_UNDOCKED;
			pr_info("POGO Dock Detached\n");

			tuna_otg_pogo_charger(false);

			switch_set_state(&pogo->dock_switch,
					POGO_DOCK_UNDOCKED);
			switch_set_state(&pogo->audio_switch,
					POGO_AUDIO_DISCONNECTED);
		}

		omap_mux_set_gpio(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN,
				GPIO_POGO_DATA);
	}

	return IRQ_HANDLED;
}

static irqreturn_t pogo_data_irq(int irq, void *data)
{
	struct tuna_pogo *pogo = data;

	if (gpio_get_value(GPIO_POGO_DATA)) {
		ktime_get_ts(&pogo->rise_time);
		irq_set_irq_type(pogo->data_irq,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT);
	} else {
		ktime_get_ts(&pogo->fall_time);
		pogo->fall_detected = true;
		irq_set_irq_type(pogo->data_irq,
				IRQF_TRIGGER_HIGH | IRQF_ONESHOT);
		complete(&pogo->completion);
		disable_irq_nosync(pogo->data_irq);
	}
	return IRQ_HANDLED;
}

void __init omap4_tuna_pogo_init(void)
{
	struct tuna_pogo *pogo = &tuna_pogo;
	unsigned int r;
	int ret;

	omap_mux_init_signal("usbb2_hsic_data.gpio_169",
			OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE3);

	/* The pullup/pulldown controls in the mux register are not the controls
	 * that you are looking for.  The usbb2_hsic_data signal has a separate
	 * special control in the CONTROL_USBB_HSIC register.
	 */
	r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);
	r &= ~OMAP4_USBB2_HSIC_DATA_WD_MASK;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USBB_HSIC);

	ret = gpio_request(GPIO_POGO_DET, "pogo_det");
	if (ret < 0)
		pr_err("request for pogo_det gpio failed, err %d\n", ret);

	gpio_direction_input(GPIO_POGO_DET);

	omap_mux_init_signal("abe_dmic_din2.gpio_121",
		OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE3);
	ret = gpio_request(GPIO_POGO_DATA, "pogo_data");
	if (ret < 0)
		pr_err("request for pogo_data gpio failed, err %d\n", ret);

	gpio_direction_output(GPIO_POGO_DATA, 0);

	pogo->dock_switch.name = "dock";

	/* The POGO dock does not involve USB but we are reusing the existing
	 * usb audio switch report the availabilty of SPDIF audio through the
	 * POGO dock.
	 */
	pogo->audio_switch.name = "usb_audio";

	switch_dev_register(&pogo->dock_switch);

	switch_dev_register(&pogo->audio_switch);

	wake_lock_init(&pogo->wake_lock, WAKE_LOCK_IDLE, "pogo");

	init_completion(&pogo->completion);

	pogo->det_irq = gpio_to_irq(GPIO_POGO_DET);

	ret = request_threaded_irq(pogo->det_irq, NULL,
				pogo_det_irq_thread,
				IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				"pogo_det", pogo);
	if (ret < 0)
		pr_err("Unable to register pogo_det interrupt (%d)\n", ret);

	pogo->data_irq = gpio_to_irq(GPIO_POGO_DATA);

	ret = request_irq(pogo->data_irq, pogo_data_irq,
				IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				"pogo_data", pogo);
	if (ret < 0)
		pr_err("Unable to register pogo_data interrupt (%d)\n", ret);

	disable_irq(pogo->data_irq);
}
