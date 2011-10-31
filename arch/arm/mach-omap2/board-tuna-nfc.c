/* Control power to pn544
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <plat/serial.h>

#include "mux.h"

#define GPIO_NFC_EN	173
#define GPIO_NFC_FW	172
#define GPIO_NFC_IRQ	17

#define PWR_OFF		0
#define PWR_ON		1
#define PWR_ON_FW	2

#define NFC_UART_NUM	4  /* omap_uart_wake() counts from 1 */

static unsigned int nfc_power;
static struct wake_lock nfc_wake_lock;

static void nfc_power_apply(void) {
	switch (nfc_power) {
	case PWR_OFF:
		pr_info("%s OFF\n", __func__);
		gpio_set_value(GPIO_NFC_FW, 0);
		gpio_set_value(GPIO_NFC_EN, 0);
		msleep(60);
		break;
	case PWR_ON:
		pr_info("%s ON\n", __func__);
		gpio_set_value(GPIO_NFC_FW, 0);
		gpio_set_value(GPIO_NFC_EN, 1);
		msleep(20);
		break;
	case PWR_ON_FW:
		pr_info("%s ON (firmware download)\n", __func__);
		gpio_set_value(GPIO_NFC_FW, 1);
		gpio_set_value(GPIO_NFC_EN, 1);
		msleep(20);
		gpio_set_value(GPIO_NFC_EN, 0);  /* fw mode requires reset */
		msleep(60);
		gpio_set_value(GPIO_NFC_EN, 1);
		msleep(20);
		break;
	}
}

static ssize_t nfc_power_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", nfc_power);
}

static ssize_t nfc_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned int val;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val > PWR_ON_FW)
		return -EINVAL;
	nfc_power = val;
	nfc_power_apply();
	return count;
}

static DEVICE_ATTR(nfc_power, S_IWUSR | S_IRUGO, nfc_power_show,
		nfc_power_store);

static irqreturn_t nfc_irq_isr(int irq, void *dev)
{
	omap_uart_wake(NFC_UART_NUM);

	/*
	 * take a 500ms wakelock, to give time for higher layers
	 * to either take their own wakelock or finish processing
	 */
	wake_lock_timeout(&nfc_wake_lock, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

void __init omap4_tuna_nfc_init(void)
{
	struct platform_device *pdev;
	int irq;

	gpio_request(GPIO_NFC_FW, "nfc_fw");
	gpio_direction_output(GPIO_NFC_FW, 0);
	omap_mux_init_gpio(GPIO_NFC_FW, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_EN, "nfc_en");
	gpio_direction_output(GPIO_NFC_EN, 0);
	omap_mux_init_gpio(GPIO_NFC_EN, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_IRQ, "nfc_irq");
	gpio_direction_input(GPIO_NFC_IRQ);
	omap_mux_init_gpio(GPIO_NFC_IRQ, OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_WAKEUPENABLE);

	wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc");

	irq = gpio_to_irq(GPIO_NFC_IRQ);
	if (request_irq(irq, nfc_irq_isr, IRQF_TRIGGER_RISING, "nfc_irq",
			NULL)) {
		pr_err("%s: request_irq() failed\n", __func__);
		return;
	}

	if (enable_irq_wake(irq)) {
		pr_err("%s: irq_set_irq_wake() failed\n", __func__);
		return;
	}

	nfc_power = PWR_OFF;

	pdev = platform_device_register_simple("nfc-power", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("%s: platform_device_register_simple() failed\n", __func__);
		return;
	}
	if (device_create_file(&pdev->dev, &dev_attr_nfc_power))
		pr_err("%s: device_create_file() failed\n", __func__);
}
