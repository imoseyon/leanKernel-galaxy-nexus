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
#include <linux/usb/otg_id.h>

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
#define POGO_DESKDOCK_PREWAIT_PERIOD	100
#define POGO_CARDOCK_PREWAIT_PERIOD	200
#define POGO_ENTER_SPDIF_PERIOD		100
#define POGO_ENTER_SPDIF_WAIT_PERIOD	100
#define POGO_ID_PERIOD_TOLERANCE	20
#define POGO_DET_DEBOUNCE		80

#define POGO_DOCK_ID_MAX_RETRY		10

#define POGO_AUDIO_DISCONNECTED		0
#define POGO_AUDIO_CONNECTED		2

#define POGO_DOCK_UNDOCKED		0
#define POGO_DOCK_DESK			1
#define POGO_DOCK_CAR			2

enum debounce_state {
	POGO_DET_DOCKED,	/* interrupt enabled,  timer stopped */
	POGO_DET_UNSTABLE,	/* interrupt disabled, timer running */
	POGO_DET_WAIT_STABLE,	/* interrupt enabled,  timer running */
	POGO_DET_UNDOCKED,	/* interrupt disabled, timer stopped */
};

struct tuna_pogo {
	struct otg_id_notifier_block	otg_id_nb;
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
	struct mutex			mutex;
	struct timer_list		det_timer;
	struct work_struct		det_work;
	spinlock_t			det_irq_lock;
	enum debounce_state		debounce_state;
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

static void pogo_dock_change(struct tuna_pogo *pogo)
{
	switch_set_state(&pogo->dock_switch, pogo->dock_type);
	switch_set_state(&pogo->audio_switch,
			pogo->dock_type != POGO_DOCK_UNDOCKED ?
			POGO_AUDIO_CONNECTED : POGO_AUDIO_DISCONNECTED);
	tuna_otg_pogo_charger(pogo->dock_type != POGO_DOCK_UNDOCKED);

	switch (pogo->dock_type) {
	case POGO_DOCK_DESK:
		pr_info("Desk Dock\n");
		break;
	case POGO_DOCK_CAR:
		pr_info("Car Dock\n");
		break;
	default:
		pr_info("Undocked\n");
	};
}

static int pogo_detect_callback(struct otg_id_notifier_block *nb)
{
	struct tuna_pogo *pogo = container_of(nb, struct tuna_pogo, otg_id_nb);
	int id_period;
	unsigned int retry = 0;
	unsigned long irqflags;

	if (gpio_get_value(GPIO_POGO_DET)) {
		wake_lock(&pogo->wake_lock);

		while (pogo->dock_type == POGO_DOCK_UNDOCKED) {

			if (!gpio_get_value(GPIO_POGO_DET)) {
				wake_unlock(&pogo->wake_lock);
				return OTG_ID_UNHANDLED;
			}

			if (retry++ > POGO_DOCK_ID_MAX_RETRY) {
				wake_unlock(&pogo->wake_lock);
				pr_err("Unable to identify pogo dock\n");
				return OTG_ID_UNHANDLED;
			}

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
					POGO_ID_PERIOD_TOLERANCE)
				pogo->dock_type = POGO_DOCK_CAR;
			else if (abs(id_period - POGO_ID_DESKDOCK) <=
					POGO_ID_PERIOD_TOLERANCE)
				pogo->dock_type = POGO_DOCK_DESK;
		}

		if (pogo->dock_type == POGO_DOCK_CAR)
			msleep(POGO_CARDOCK_PREWAIT_PERIOD);
		else
			msleep(POGO_DESKDOCK_PREWAIT_PERIOD);

		/* Instruct the dock to enter SPDIF mode */
		pogo_send_pulse(POGO_ENTER_SPDIF_PERIOD);

		msleep(POGO_ENTER_SPDIF_WAIT_PERIOD);

		mutex_lock(&pogo->mutex);
		omap_mux_set_gpio(OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT,
				GPIO_POGO_DATA);

		pogo_dock_change(pogo);
		wake_lock_timeout(&pogo->wake_lock, msecs_to_jiffies(1000));

		spin_lock_irqsave(&pogo->det_irq_lock, irqflags);
		pogo->debounce_state = POGO_DET_DOCKED;
		enable_irq(pogo->det_irq);
		enable_irq_wake(pogo->det_irq);
		spin_unlock_irqrestore(&pogo->det_irq_lock, irqflags);
		mutex_unlock(&pogo->mutex);

		return OTG_ID_HANDLED;
	}

	return OTG_ID_UNHANDLED;
}

static void pogo_dock_undock(struct tuna_pogo *pogo)
{
	mutex_lock(&pogo->mutex);
	if (pogo->dock_type != POGO_DOCK_UNDOCKED &&
	    pogo->debounce_state == POGO_DET_UNDOCKED) {
		pogo->dock_type = POGO_DOCK_UNDOCKED;
		pogo_dock_change(pogo);

		omap_mux_set_gpio(OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN,
				GPIO_POGO_DATA);
	}
	mutex_unlock(&pogo->mutex);
}

/* This callback is used to cancel any ownership of the chain */
static void pogo_cancel_callback(struct otg_id_notifier_block *nb)
{
	struct tuna_pogo *pogo = container_of(nb, struct tuna_pogo, otg_id_nb);
	unsigned long irqflags;

	/* Disable the POGO_DET IRQ and cancel any pending timer if needed */
	spin_lock_irqsave(&pogo->det_irq_lock, irqflags);
	switch (pogo->debounce_state) {
	case POGO_DET_UNDOCKED:
		break;
	case POGO_DET_UNSTABLE:
		del_timer(&pogo->det_timer);
		break;
	case POGO_DET_WAIT_STABLE:
		del_timer(&pogo->det_timer);
		/* fall through */
	case POGO_DET_DOCKED:
		disable_irq_wake(pogo->det_irq);
		disable_irq_nosync(pogo->det_irq);
		break;
	}
	pogo->debounce_state = POGO_DET_UNDOCKED;
	spin_unlock_irqrestore(&pogo->det_irq_lock, irqflags);

	/* Change the state to undocked */
	pogo_dock_undock(pogo);
}

static void det_work_func(struct work_struct *work)
{
	struct tuna_pogo *pogo = container_of(work, struct tuna_pogo, det_work);

	pogo_dock_undock(pogo);

	/* Notify the otg_id chain that a change has occurred */
	otg_id_notify();
}

static void pogo_det_timer_func(unsigned long arg)
{
	struct tuna_pogo *pogo = (struct tuna_pogo *)arg;
	unsigned long irqflags;

	spin_lock_irqsave(&pogo->det_irq_lock, irqflags);
	switch (pogo->debounce_state) {
	case POGO_DET_DOCKED:
		break;
	case POGO_DET_UNSTABLE:
		/*
		 * The detect gpio changed in one the previous two timeslots,
		 * so enable the irq, reset the timer, and wait again. If the
		 * detect gpio changed after we last disabled the interrupt we
		 * will get anther interrupt right away and the state will go
		 * back to POGO_DET_UNSTABLE.
		 */
		pogo->debounce_state = POGO_DET_WAIT_STABLE;
		enable_irq(pogo->det_irq);
		enable_irq_wake(pogo->det_irq);
		mod_timer(&pogo->det_timer,
			jiffies + msecs_to_jiffies(POGO_DET_DEBOUNCE));
		break;
	case POGO_DET_WAIT_STABLE:
		if (gpio_get_value(GPIO_POGO_DET) == 0) {
			pogo->debounce_state = POGO_DET_UNDOCKED;
			disable_irq_wake(pogo->det_irq);
			disable_irq_nosync(pogo->det_irq);
			wake_lock_timeout(&pogo->wake_lock,
					  msecs_to_jiffies(1000));
			schedule_work(&pogo->det_work);
		} else {
			/* The device appears to be back in the dock */
			pogo->debounce_state = POGO_DET_DOCKED;
			wake_unlock(&pogo->wake_lock);
		}
		break;
	case POGO_DET_UNDOCKED:
		break;
	}
	spin_unlock_irqrestore(&pogo->det_irq_lock, irqflags);
}

static irqreturn_t pogo_det_irq(int irq, void *data)
{
	struct tuna_pogo *pogo = data;
	unsigned long irqflags;

	spin_lock_irqsave(&pogo->det_irq_lock, irqflags);
	switch (pogo->debounce_state) {
	case POGO_DET_DOCKED:
		wake_lock(&pogo->wake_lock);
		mod_timer(&pogo->det_timer,
			jiffies + msecs_to_jiffies(POGO_DET_DEBOUNCE));
		/* fall through */
	case POGO_DET_WAIT_STABLE:
		/*
		 * Disable IRQ line in case there is noise. It will be
		 * re-enabled when the timer expires
		 */
		pogo->debounce_state = POGO_DET_UNSTABLE;
		disable_irq_wake(pogo->det_irq);
		disable_irq_nosync(pogo->det_irq);
		break;
	case POGO_DET_UNSTABLE:
	case POGO_DET_UNDOCKED:
		break;
	}
	spin_unlock_irqrestore(&pogo->det_irq_lock, irqflags);

	return IRQ_HANDLED;
}

static irqreturn_t pogo_data_irq(int irq, void *data)
{
	struct tuna_pogo *pogo = data;

	if (gpio_get_value(GPIO_POGO_DATA)) {
		ktime_get_ts(&pogo->rise_time);
		irq_set_irq_type(pogo->data_irq,
				IRQF_TRIGGER_LOW);
	} else {
		ktime_get_ts(&pogo->fall_time);
		pogo->fall_detected = true;
		irq_set_irq_type(pogo->data_irq,
				IRQF_TRIGGER_HIGH);
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
			OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE3 |
			OMAP_WAKEUP_EN);

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

	wake_lock_init(&pogo->wake_lock, WAKE_LOCK_SUSPEND, "pogo");

	init_completion(&pogo->completion);

	pogo->otg_id_nb.detect = pogo_detect_callback;
	pogo->otg_id_nb.proxy_wait = NULL;
	pogo->otg_id_nb.cancel = pogo_cancel_callback;
	pogo->otg_id_nb.priority = TUNA_OTG_ID_POGO_PRIO;

	ret = otg_id_register_notifier(&pogo->otg_id_nb);
	if (ret < 0)
		pr_err("Unable to register notifier\n");

	setup_timer(&pogo->det_timer, pogo_det_timer_func, (unsigned long)pogo);
	spin_lock_init(&pogo->det_irq_lock);
	mutex_init(&pogo->mutex);
	INIT_WORK(&pogo->det_work, det_work_func);
	pogo->debounce_state = POGO_DET_UNDOCKED;

	pogo->det_irq = gpio_to_irq(GPIO_POGO_DET);

	ret = request_irq(pogo->det_irq, pogo_det_irq,
				IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING,
				"pogo_det", pogo);
	if (ret < 0)
		pr_err("Unable to register pogo_det interrupt (%d)\n", ret);

	disable_irq(pogo->det_irq);

	pogo->data_irq = gpio_to_irq(GPIO_POGO_DATA);

	ret = request_irq(pogo->data_irq, pogo_data_irq,
				IRQF_TRIGGER_HIGH,
				"pogo_data", pogo);
	if (ret < 0)
		pr_err("Unable to register pogo_data interrupt (%d)\n", ret);

	disable_irq(pogo->data_irq);
}
