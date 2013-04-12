/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/mach-types.h>
#include <plat/serial.h>
#include <plat/board-tuna-bluetooth.h>
#include <linux/regulator/driver.h>

#define BT_REG_GPIO 103
#define BT_RESET_GPIO 42

#define BT_WAKE_GPIO 27
#define BT_HOST_WAKE_GPIO 177

static struct rfkill *bt_rfkill;
static struct regulator *clk32kaudio_reg;
static bool bt_enabled;
static bool host_wake_uart_enabled;
static bool wake_uart_enabled;
static struct dentry *btdebugdent;

struct bcm_bt_lpm {
	int wake;
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock wake_lock;
	char wake_lock_name[100];
} bt_lpm;

static int bcm4330_bt_rfkill_set_power(void *data, bool blocked)
{
	// rfkill_ops callback. Turn transmitter on when blocked is false
	if (!blocked) {
		if (clk32kaudio_reg && !bt_enabled)
			regulator_enable(clk32kaudio_reg);

		gpio_set_value(BT_REG_GPIO, 1);
		gpio_set_value(BT_RESET_GPIO, 1);

	} else {
		gpio_set_value(BT_RESET_GPIO, 0);
		gpio_set_value(BT_REG_GPIO, 0);
		if (clk32kaudio_reg && bt_enabled)
			regulator_disable(clk32kaudio_reg);
	}

	bt_enabled = !blocked;

	return 0;
}

static const struct rfkill_ops bcm4330_bt_rfkill_ops = {
	.set_block = bcm4330_bt_rfkill_set_power,
};

static void set_wake_locked(int wake)
{
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock);

	if (!wake_uart_enabled && wake)
		omap_uart_enable(2);

	gpio_set_value(BT_WAKE_GPIO, wake);

	if (wake_uart_enabled && !wake)
		omap_uart_disable(2);

	wake_uart_enabled = wake;
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer) {
	unsigned long flags;
	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	set_wake_locked(0);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport) {
	bt_lpm.uport = uport;

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		wake_lock(&bt_lpm.wake_lock);
		if (!host_wake_uart_enabled)
			omap_uart_enable(2);
	} else  {
		if (host_wake_uart_enabled)
			omap_uart_disable(2);
		// Take a timed wakelock, so that upper layers can take it.
		// The chipset deasserts the hostwake lock, when there is no
		// more data to send.
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
	}

	host_wake_uart_enabled = host_wake;

}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;

	host_wake = gpio_get_value(BT_HOST_WAKE_GPIO);
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	update_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;
	int rc;

	rc = gpio_request(BT_WAKE_GPIO, "bcm4330_wake_gpio");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(BT_HOST_WAKE_GPIO, "bcm4330_host_wake_gpio");
	if (unlikely(rc)) {
		gpio_free(BT_WAKE_GPIO);
		return rc;
	}

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;

	irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
		"bt host_wake", NULL);
	if (ret) {
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return ret;
	}

	gpio_direction_output(BT_WAKE_GPIO, 0);
	gpio_direction_input(BT_HOST_WAKE_GPIO);

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BTLowPower");
	wake_lock_init(&bt_lpm.wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);
	return 0;
}

static int btdebug_dump(struct seq_file *sf, void *private)
{
	seq_printf(sf, "en=%d bt_wake=%d lpm.w=%d w_uart_en=%d\n",
		   bt_enabled, gpio_get_value(BT_WAKE_GPIO),
		   bt_lpm.wake, wake_uart_enabled);
	seq_printf(sf, "bt_host_wake=%d lpm.hw=%d hw_uart_en=%d\n",
		   gpio_get_value(BT_HOST_WAKE_GPIO), bt_lpm.host_wake,
		   host_wake_uart_enabled);
	return 0;
}

static int btdebug_open(struct inode *inode, struct file *file)
{
	return single_open(file, btdebug_dump, NULL);
}

static const struct file_operations btdebug_fops = {
	.open = btdebug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bcm4330_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	int ret = 0;

	rc = gpio_request(BT_RESET_GPIO, "bcm4330_nreset_gpip");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(BT_REG_GPIO, "bcm4330_nshutdown_gpio");
	if (unlikely(rc)) {
		gpio_free(BT_RESET_GPIO);
		return rc;
	}

	clk32kaudio_reg = regulator_get(0, "clk32kaudio");
	if (IS_ERR(clk32kaudio_reg)) {
		pr_err("clk32kaudio reg not found!\n");
		clk32kaudio_reg = NULL;
	}

	gpio_direction_output(BT_REG_GPIO, 1);
	gpio_direction_output(BT_RESET_GPIO, 1);

	bt_rfkill = rfkill_alloc("bcm4330 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4330_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill)) {
		gpio_free(BT_RESET_GPIO);
		gpio_free(BT_REG_GPIO);
		return -ENOMEM;
	}

	rfkill_set_states(bt_rfkill, true, false);
	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(bt_rfkill);
		gpio_free(BT_RESET_GPIO);
		gpio_free(BT_REG_GPIO);
		return -1;
	}

	ret = bcm_bt_lpm_init(pdev);
	if (ret) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);

		gpio_free(BT_RESET_GPIO);
		gpio_free(BT_REG_GPIO);
	}

	btdebugdent = debugfs_create_file("bt", S_IRUGO, NULL, NULL,
					  &btdebug_fops);
	if (IS_ERR_OR_NULL(btdebugdent))
		pr_err("%s: failed to create debugfs file\n", __func__);

	return ret;
}

static int bcm4330_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	if (!IS_ERR_OR_NULL(btdebugdent))
		debugfs_remove(btdebugdent);

	gpio_free(BT_REG_GPIO);
	gpio_free(BT_RESET_GPIO);
	gpio_free(BT_WAKE_GPIO);
	gpio_free(BT_HOST_WAKE_GPIO);
	regulator_put(clk32kaudio_reg);

	wake_lock_destroy(&bt_lpm.wake_lock);
	return 0;
}

int bcm4430_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	int irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	int host_wake;

	disable_irq(irq);
	host_wake = gpio_get_value(BT_HOST_WAKE_GPIO);

	if (host_wake) {
		enable_irq(irq);
		return -EBUSY;
	}

	return 0;
}

int bcm4430_bluetooth_resume(struct platform_device *pdev)
{
	int irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	enable_irq(irq);
	return 0;
}

static struct platform_driver bcm4330_bluetooth_platform_driver = {
	.probe = bcm4330_bluetooth_probe,
	.remove = bcm4330_bluetooth_remove,
	.suspend = bcm4430_bluetooth_suspend,
	.resume = bcm4430_bluetooth_resume,
	.driver = {
		   .name = "bcm4330_bluetooth",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm4330_bluetooth_init(void)
{
	bt_enabled = false;
	return platform_driver_register(&bcm4330_bluetooth_platform_driver);
}

static void __exit bcm4330_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm4330_bluetooth_platform_driver);
}


module_init(bcm4330_bluetooth_init);
module_exit(bcm4330_bluetooth_exit);

MODULE_ALIAS("platform:bcm4330");
MODULE_DESCRIPTION("bcm4330_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL");
