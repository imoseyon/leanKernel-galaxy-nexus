/*
 * hsi_driver_gpio.c
 *
 * Implements HSI GPIO related functionality. (i.e: wake lines management)
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include "hsi_driver.h"

static void do_hsi_cawake_tasklet(unsigned long hsi_p)
{
	struct hsi_port *port = (struct hsi_port *)hsi_p;
	struct hsi_dev *hsi_ctrl = port->hsi_controller;

	spin_lock(&hsi_ctrl->lock);
	hsi_clocks_enable(hsi_ctrl->dev, __func__);
	port->in_cawake_tasklet = true;

	port->cawake_status = hsi_get_cawake(port);
	hsi_do_cawake_process(port);

	port->in_cawake_tasklet = false;
	hsi_clocks_disable(hsi_ctrl->dev, __func__);
	spin_unlock(&hsi_ctrl->lock);
}

static irqreturn_t hsi_cawake_isr(int irq, void *hsi_p)
{
	struct hsi_port *port = hsi_p;

	tasklet_hi_schedule(&port->cawake_tasklet);

	return IRQ_HANDLED;
}

int __init hsi_cawake_init(struct hsi_port *port, const char *irq_name)
{
	tasklet_init(&port->cawake_tasklet, do_hsi_cawake_tasklet,
		     (unsigned long)port);

	if (request_irq(port->cawake_gpio_irq, hsi_cawake_isr,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING, irq_name, port) < 0) {
		dev_err(port->hsi_controller->dev,
			"FAILED to request %s GPIO IRQ %d on port %d\n",
			irq_name, port->cawake_gpio_irq, port->port_number);
		return -EBUSY;
	}

	return 0;
}

void hsi_cawake_exit(struct hsi_port *port)
{
	if (port->cawake_gpio < 0)
		return;	/* Nothing to do (case SSI with GPIO or */
			/* HSI with IO ring wakeup */

	tasklet_kill(&port->cawake_tasklet);
	free_irq(port->cawake_gpio_irq, port);
}
