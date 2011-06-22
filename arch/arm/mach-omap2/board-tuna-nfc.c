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

#include "mux.h"

#define GPIO_NFC_EN	173
#define GPIO_NFC_FW	172
#define GPIO_NFC_IRQ	17

#define PWR_OFF		0
#define PWR_ON		1
#define PWR_ON_FW	2

static unsigned int nfc_power;

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

void __init omap4_tuna_nfc_init(void)
{
	struct platform_device *pdev;

	gpio_request(GPIO_NFC_FW, "nfc_fw");
	gpio_direction_output(GPIO_NFC_FW, 0);
	omap_mux_init_gpio(GPIO_NFC_FW, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_EN, "nfc_en");
	gpio_direction_output(GPIO_NFC_EN, 0);
	omap_mux_init_gpio(GPIO_NFC_EN, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_IRQ, "nfc_irq");
	gpio_direction_input(GPIO_NFC_IRQ);
	omap_mux_init_gpio(GPIO_NFC_IRQ, OMAP_PIN_INPUT_PULLUP);

	nfc_power = PWR_OFF;

	pdev = platform_device_register_simple("nfc-power", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("%s: platform_device_register_simple() failed\n", __func__);
		return;
	}
	if (device_create_file(&pdev->dev, &dev_attr_nfc_power))
		pr_err("%s: device_create_file() failed\n", __func__);
}
