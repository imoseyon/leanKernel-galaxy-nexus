/* /linux/drivers/misc/modem_if/modem_modemctl_device_cmc221.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#include <linux/init.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_link_device_usb.h"

static int cmc221_on(struct modem_ctl *mc)
{
	pr_info("[MODEM_IF] %s()\n", __func__);

	if (!mc->gpio_cp_off || !mc->gpio_cp_on || !mc->gpio_cp_reset) {
		pr_err("[MODEM_IF] no gpio data\n");
		return -ENXIO;
	}

	gpio_set_value(mc->gpio_cp_on, 1);
	msleep(300);

	gpio_set_value(mc->gpio_cp_reset, 1);
	gpio_set_value(mc->gpio_cp_off, 0);
	msleep(300);
	mc->phone_state = STATE_BOOTING;
	return 0;
}

static int cmc221_off(struct modem_ctl *mc)
{
	pr_info("[MODEM_IF] %s()\n", __func__);

	if (!mc->gpio_cp_off || !mc->gpio_cp_on || !mc->gpio_cp_reset) {
		pr_err("[MODEM_IF] no gpio data\n");
		return -ENXIO;
	}

	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(300);

	gpio_set_value(mc->gpio_cp_off, 1);
	msleep(300);

	mc->phone_state = STATE_OFFLINE;

	return 0;
}


static int cmc221_reset(struct modem_ctl *mc)
{
	pr_info("[MODEM_IF] %s()\n", __func__);

	if (!mc->gpio_cp_reset)
		return -ENXIO;

	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(100);

	gpio_set_value(mc->gpio_cp_reset, 1);
	msleep(100);
	return 0;
}

static int cmc221_boot_on(struct modem_ctl *mc)
{
	pr_info("[MODEM_IF] %s()\n", __func__);
	return 0;
}

static int cmc221_boot_off(struct modem_ctl *mc)
{
	pr_info("[MODEM_IF] %s()\n", __func__);
	return 0;
}

static int cmc221_get_active(struct modem_ctl *mc)
{
	pr_info("%s\n", __func__);
	if (!mc->gpio_phone_active || !mc->gpio_cp_reset)
		return -ENXIO;

	pr_info("cp %d phone %d\n",
			gpio_get_value(mc->gpio_cp_reset),
			gpio_get_value(mc->gpio_phone_active));

	if (gpio_get_value(mc->gpio_cp_reset))
		return gpio_get_value(mc->gpio_phone_active);

	return 0;
}


static void mc_work(struct work_struct *work_arg)
{

	struct modem_ctl *mc = container_of(work_arg, struct modem_ctl,
		dwork.work);

	int phone_active;
	char *envs[2] = { NULL, NULL };


	phone_active = cmc221_get_active(mc);
	if (phone_active < 0) {
		pr_err("[MODEM_CTRL] gpio not initialized\n");
		return;
	}
	if (phone_active && (mc->phone_state == STATE_BOOTING))
		mc->phone_state = STATE_ONLINE;
	else if (!phone_active && (mc->phone_state == STATE_ONLINE)) {
		mc->phone_state = STATE_CRASH_EXIT;/* DUMP START */
		envs[0] = "MAILBOX=dump_start";
		pr_err("[MODEM_CTRL][%s]%d, lte crash!dump start !!!\n",
				__func__, __LINE__);
		mc->cpcrash_flag = 1;
	} else if (phone_active && (mc->phone_state == STATE_CRASH_EXIT)) {
		mc->phone_state = STATE_CRASH_RESET;/* DUMP END */
		envs[0] = "MAILBOX=dump_end";
		pr_err("[MODEM_CTRL][%s]%d, lte crash!dump end !!!\n",
				__func__, __LINE__);
	} else {
		mc->phone_state = STATE_OFFLINE;
		pr_err("[MODEM_CTRL][%s]%d, phone_status changed to invalid!!!\n",
				__func__, __LINE__);
	}
	msleep(300);


	/*kobject_uevent_env(&mc->dev->kobj, KOBJ_OFFLINE, envs);*/
	kobject_uevent(&mc->dev->kobj, KOBJ_OFFLINE);

}


static irqreturn_t phone_active_irq_handler(int irq, void *_mc)
{
	struct modem_ctl *mc = (struct modem_ctl *)_mc;

	if (!work_pending(&mc->work))
		schedule_delayed_work(&mc->dwork, 20); /*1s*/

	disable_irq_nosync(mc->irq_phone_active);

	return IRQ_HANDLED;
}

static void cmc221_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on = cmc221_on;
	mc->ops.modem_off = cmc221_off;
	mc->ops.modem_reset = cmc221_reset;
	mc->ops.modem_boot_on = cmc221_boot_on;
	mc->ops.modem_boot_off = cmc221_boot_off;
	pr_info("[MODEM_IF] cmc221_get_ops() done\n");
}

int cmc221_init_modemctl_device(void *data)
{
	int ret = 0;
	struct modem_ctl *mc = (struct modem_ctl *)data;

	cmc221_get_ops(mc);

	dev_set_drvdata(mc->dev, mc);

	INIT_DELAYED_WORK(&mc->dwork, mc_work);

	ret = request_irq(mc->irq_phone_active, phone_active_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"phone_active", mc);
	if (ret) {
		pr_err("[MODEM_IF] Failed to allocate an interrupt(%d)\n",
							mc->irq_phone_active);
		goto irq_fail;
	}
	mc->irq[0] = mc->irq_phone_active;
	enable_irq_wake(mc->irq_phone_active);
	/*disable_irq(mc->irq_phone_active);*/

	pr_info("[MODEM_IF] init_modemctl_device() done : %d\n", ret);
	return ret;

irq_fail:
	kfree(mc);
	return ret;

}


