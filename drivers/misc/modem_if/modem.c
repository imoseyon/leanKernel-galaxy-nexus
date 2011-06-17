/* linux/drivers/modem/modem.c
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_variation.h"


static struct modem_ctl *create_modemctl_device(struct platform_device *pdev)
{
	int ret = 0;
	struct modem_data *pdata;
	struct modem_ctl *modemctl;
	struct device *dev = &pdev->dev;

	/* create modem control device */
	modemctl = kzalloc(sizeof(struct modem_ctl), GFP_KERNEL);
	if (!modemctl)
		return NULL;

	modemctl->dev = dev;
	modemctl->phone_state = STATE_OFFLINE;

	/* get platform data */
	pdata = pdev->dev.platform_data;

	modemctl->name = pdata->name;

	modemctl->gpio_cp_on = pdata->gpio_cp_on;
	modemctl->gpio_reset_req_n = pdata->gpio_reset_req_n;
	modemctl->gpio_cp_reset = pdata->gpio_cp_reset;
	modemctl->gpio_pda_active = pdata->gpio_pda_active;
	modemctl->gpio_phone_active = pdata->gpio_phone_active;
	modemctl->gpio_cp_dump_int = pdata->gpio_cp_dump_int;
	modemctl->gpio_flm_uart_sel = pdata->gpio_flm_uart_sel;
	modemctl->gpio_cp_warm_reset = pdata->gpio_cp_warm_reset;

	modemctl->irq_phone_active = platform_get_irq(pdev, 0);

#ifdef CONFIG_LTE_MODEM_CMC221
	modemctl->gpio_cp_off = pdata->gpio_cp_off;
	modemctl->gpio_slave_wakeup = pdata->gpio_slave_wakeup;
	modemctl->gpio_host_active = pdata->gpio_host_active;
	modemctl->gpio_host_wakeup = pdata->gpio_host_wakeup;
	modemctl->irq_host_wakeup = platform_get_irq(pdev, 1);
#endif
	/* init modemctl device for getting modemctl operations */
	ret = call_modem_init_func(modemctl, pdata);
	if (ret) {
		kfree(modemctl);
		return NULL;
	}

	pr_info("[MODEM_IF] %s:create_modemctl_device DONE\n", modemctl->name);
	return modemctl;
}

static struct io_device *create_io_device(struct modem_io_t *io_t,
			struct modem_ctl *modemctl)
{
	int ret = 0;
	struct io_device *iod = NULL;

	iod = kzalloc(sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		pr_err("[MODEM_IF] io device memory alloc fail\n");
		return NULL;
	}

	iod->name = io_t->name;
	iod->id = io_t->id;
	iod->format = io_t->format;
	iod->io_typ = io_t->io_type;

	/* link between io device and modem control */
	iod->mc = modemctl;
	modemctl->iod = iod;

	/* register misc device or net device */
	ret = init_io_device(iod);
	if (ret) {
		kfree(iod);
		return NULL;
	}

	pr_info("[MODEM_IF] %s : create_io_device DONE\n", io_t->name);
	return iod;
}

static int __devinit modem_probe(struct platform_device *pdev)
{
	int i;
	struct modem_data *pdata;
	struct modem_ctl *modemctl;
	struct io_device *iod[MAX_NUM_IO_DEV];
	struct link_device *ld;
	struct io_raw_devices *io_raw_devs = NULL;

	pdata = pdev->dev.platform_data;

	modemctl = create_modemctl_device(pdev);
	if (!modemctl)
		return -ENOMEM;

	/* create link device */
	ld = call_link_init_func(pdev, pdata->link_type);
		if (!ld)
			goto err_free_modemctl;

	io_raw_devs = kzalloc(sizeof(struct io_raw_devices), GFP_KERNEL);
	if (!io_raw_devs)
		return -ENOMEM;

	/* create io deivces and connect to modemctl device */
	for (i = 0; i < pdata->num_iodevs; i++) {
		iod[i] = create_io_device(&pdata->iodevs[i], modemctl);
		if (!iod[i])
			goto err_free_modemctl;

		if (iod[i]->format == IPC_RAW) {
			int ch = iod[i]->id & 0x1F;
			io_raw_devs->raw_devices[ch] = iod[i];
			io_raw_devs->num_of_raw_devs++;
			iod[i]->link = ld;
		} else {
			/* connect io devices to one link device */
			ld->attach(ld, iod[i]);
		}

		if (iod[i]->format == IPC_MULTI_RAW)
			iod[i]->private_data = (void *)io_raw_devs;
	}

	platform_set_drvdata(pdev, modemctl);

	pr_info("[MODEM_IF] modem_probe DONE\n");
	return 0;

err_free_modemctl:
	for (i = 0; i < pdata->num_iodevs; i++)
		if (iod[i] != NULL)
			kfree(iod[i]);

	if (io_raw_devs != NULL)
		kfree(io_raw_devs);

	if (modemctl != NULL)
		kfree(modemctl);

	return -ENOMEM;
}

static int modem_suspend(struct device *pdev)
{
	struct modem_ctl *mc = dev_get_drvdata(pdev);
	gpio_set_value(mc->gpio_pda_active, 0);
	return 0;
}

static int modem_resume(struct device *pdev)
{
	struct modem_ctl *mc = dev_get_drvdata(pdev);
	gpio_set_value(mc->gpio_pda_active, 1);
	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend    = modem_suspend,
	.resume     = modem_resume,
};

static struct platform_driver modem_driver = {
	.probe = modem_probe,
	.driver = {
		.name = "modem_if",
		.pm   = &modem_pm_ops,
	},
};

static int __init modem_init(void)
{
	return platform_driver_register(&modem_driver);
}

module_init(modem_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung Modem Interface Driver");
