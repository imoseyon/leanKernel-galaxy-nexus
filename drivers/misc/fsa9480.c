/*
 * driver/misc/fsa9480.c - FSA9480 micro USB switch device driver
 *
 * Copyright (C) 2010 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/fsa9480.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/otg_id.h>

/* FSA9480 I2C registers */
#define FSA9480_REG_DEVID		0x01
#define FSA9480_REG_CTRL		0x02
#define FSA9480_REG_INT1		0x03
#define FSA9480_REG_INT2		0x04
#define FSA9480_REG_INT1_MASK		0x05
#define FSA9480_REG_INT2_MASK		0x06
#define FSA9480_REG_ADC			0x07
#define FSA9480_REG_TIMING1		0x08
#define FSA9480_REG_TIMING2		0x09
#define FSA9480_REG_DEV_T1		0x0a
#define FSA9480_REG_DEV_T2		0x0b
#define FSA9480_REG_BTN1		0x0c
#define FSA9480_REG_BTN2		0x0d
#define FSA9480_REG_CK			0x0e
#define FSA9480_REG_CK_INT1		0x0f
#define FSA9480_REG_CK_INT2		0x10
#define FSA9480_REG_CK_INTMASK1		0x11
#define FSA9480_REG_CK_INTMASK2		0x12
#define FSA9480_REG_MANSW1		0x13
#define FSA9480_REG_MANSW2		0x14
#define FSA9480_REG_MANOVERRIDE1	0x1B

/* Control */
#define CON_SWITCH_OPEN		(1 << 4)
#define CON_RAW_DATA		(1 << 3)
#define CON_MANUAL_SW		(1 << 2)
#define CON_WAIT		(1 << 1)
#define CON_INT_MASK		(1 << 0)
#define CON_MASK		(CON_SWITCH_OPEN | CON_RAW_DATA | \
				CON_MANUAL_SW | CON_WAIT)

/* we always read these as a word */
/* Device Type 2 */
#define DEV_AV			(1 << 14)
#define DEV_TTY			(1 << 13)
#define DEV_PPD			(1 << 12)
#define DEV_JIG_UART_OFF	(1 << 11)
#define DEV_JIG_UART_ON		(1 << 10)
#define DEV_JIG_USB_OFF		(1 << 9)
#define DEV_JIG_USB_ON		(1 << 8)
/* Device Type 1 */
#define DEV_USB_OTG		(1 << 7)
#define DEV_DEDICATED_CHG	(1 << 6)
#define DEV_USB_CHG		(1 << 5)
#define DEV_CAR_KIT		(1 << 4)
#define DEV_UART		(1 << 3)
#define DEV_USB			(1 << 2)
#define DEV_AUDIO_2		(1 << 1)
#define DEV_AUDIO_1		(1 << 0)

#define DEV_USB_MASK		(DEV_USB_OTG | DEV_USB | DEV_JIG_USB_OFF | \
				 DEV_JIG_USB_ON)
#define DEV_UART_MASK		(DEV_UART | DEV_JIG_UART_OFF)
#define DEV_JIG_MASK		(DEV_JIG_USB_OFF | DEV_JIG_USB_ON | \
				 DEV_JIG_UART_OFF | DEV_JIG_UART_ON)
#define DEV_CHARGER_MASK	(DEV_DEDICATED_CHG | DEV_USB_CHG | DEV_CAR_KIT)

/*
 * Manual Switch
 * D- [7:5] / D+ [4:2]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART / 100: V_AUDIO
 */
#define SW_VAUDIO		((4 << 5) | (4 << 2))
#define SW_UART			((3 << 5) | (3 << 2))
#define SW_AUDIO		((2 << 5) | (2 << 2))
#define SW_DHOST		((1 << 5) | (1 << 2))
#define SW_AUTO			((0 << 5) | (0 << 2))

/* Interrupt 1 */
#define INT_DETACH		(1 << 1)
#define INT_ATTACH		(1 << 0)

static const unsigned int adc_timing[] = {
	50, /* ms */
	100,
	150,
	200,
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000
};

static const char *device_names[] = {
	[FSA9480_DETECT_NONE]		= "unknown/none",
	[FSA9480_DETECT_USB]		= "usb-peripheral",
	[FSA9480_DETECT_USB_HOST]	= "usb-host",
	[FSA9480_DETECT_CHARGER]	= "charger",
	[FSA9480_DETECT_JIG]		= "jig",
	[FSA9480_DETECT_UART]		= "uart",
};

struct usbsw_nb_info {
	struct otg_id_notifier_block	otg_id_nb;
	struct fsa9480_detect_set	*detect_set;

	struct fsa9480_usbsw		*usbsw;
};

struct fsa9480_usbsw {
	struct i2c_client		*client;
	struct fsa9480_platform_data	*pdata;
	int				mansw;
	u32				curr_dev;
	struct mutex			lock;

	int				num_notifiers;
	struct usbsw_nb_info		notifiers[0];
};
#define xceiv_to_fsa(x)		container_of((x), struct fsa9480_usbsw, otg)

static ssize_t fsa9480_show_control(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	s32 value;

	value = i2c_smbus_read_byte_data(client, FSA9480_REG_CTRL);
	if (value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, value);
		return (ssize_t)value;
	}

	return sprintf(buf, "%02x\n", value);
}

static ssize_t fsa9480_show_device_type(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	s32 value;

	value = i2c_smbus_read_byte_data(client, FSA9480_REG_DEV_T1);
	if (value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, value);
		return (ssize_t)value;
	}

	return sprintf(buf, "%02x\n", value);
}

static ssize_t fsa9480_show_manualsw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	s32 value;

	value = i2c_smbus_read_byte_data(client, FSA9480_REG_MANSW1);
	if (value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, value);
		return (ssize_t)value;
	}

	if (value == SW_VAUDIO)
		return sprintf(buf, "VAUDIO\n");
	else if (value == SW_UART)
		return sprintf(buf, "UART\n");
	else if (value == SW_AUDIO)
		return sprintf(buf, "AUDIO\n");
	else if (value == SW_DHOST)
		return sprintf(buf, "DHOST\n");
	else if (value == SW_AUTO)
		return sprintf(buf, "AUTO\n");
	else
		return sprintf(buf, "%x", value);
}

static ssize_t fsa9480_set_manualsw(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
	struct i2c_client *client = usbsw->client;
	s32 value;
	unsigned int path = 0;
	int ret;

	value = i2c_smbus_read_byte_data(client, FSA9480_REG_CTRL);
	if (value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, value);
		return (ssize_t)value;
	}

	if ((value & ~CON_MANUAL_SW) !=
			(CON_SWITCH_OPEN | CON_RAW_DATA | CON_WAIT))
		return -EINVAL;

	if (!strncmp(buf, "VAUDIO", 6)) {
		path = SW_VAUDIO;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "UART", 4)) {
		path = SW_UART;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "AUDIO", 5)) {
		path = SW_AUDIO;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "DHOST", 5)) {
		path = SW_DHOST;
		value &= ~CON_MANUAL_SW;
	} else if (!strncmp(buf, "AUTO", 4)) {
		path = SW_AUTO;
		value |= CON_MANUAL_SW;
	} else {
		dev_err(dev, "Wrong command\n");
		return -EINVAL;
	}

	usbsw->mansw = path;

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_MANSW1, path);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return (ssize_t)value;
	}

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_CTRL, value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return (ssize_t)value;
	}

	return count;
}

static DEVICE_ATTR(control, S_IRUGO, fsa9480_show_control, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, fsa9480_show_device_type, NULL);
static DEVICE_ATTR(switch, S_IRUGO | S_IWUSR,
		fsa9480_show_manualsw, fsa9480_set_manualsw);

static struct attribute *fsa9480_attributes[] = {
	&dev_attr_control.attr,
	&dev_attr_device_type.attr,
	&dev_attr_switch.attr,
	NULL
};

static const struct attribute_group fsa9480_group = {
	.attrs = fsa9480_attributes,
};

static int fsa9480_reg_init(struct fsa9480_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	unsigned int ctrl = CON_MASK;
	s32 ret;
	u8 i;

	/* soft reset to re-initialize the fsa, and re-do detection */
	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_MANOVERRIDE1, 1);
	if (ret < 0) {
		dev_err(&client->dev, "cannot soft reset,  err %d\n", ret);
		return ret;
	}

	/* mask interrupts (unmask attach/detach only) */
	ret = i2c_smbus_write_word_data(client, FSA9480_REG_INT1_MASK, 0x1ffc);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	/* mask all car kit interrupts */
	ret = i2c_smbus_write_word_data(client, FSA9480_REG_CK_INTMASK1,
			0x07ff);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	/* Reconcile the requested ADC detect time with the available settings
	 * on the FSA9480.
	 */
	for (i = 0; i < ARRAY_SIZE(adc_timing); i++) {
		if (usbsw->pdata->detect_time <= adc_timing[i])
			break;
	}

	if (i == ARRAY_SIZE(adc_timing))
		return -ERANGE;

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_TIMING1, i);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(client, FSA9480_REG_MANSW1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	usbsw->mansw = ret;

	if (usbsw->mansw)
		ctrl &= ~CON_MANUAL_SW;	/* Manual Switching Mode */

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_CTRL, ctrl);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void _detected(struct fsa9480_usbsw *usbsw, int device)
{
	dev_info(&usbsw->client->dev,
		 "cable detect change, from '%s' to '%s'\n",
		 device_names[usbsw->curr_dev], device_names[device]);
	usbsw->curr_dev = device;
	usbsw->pdata->detected(usbsw->curr_dev);
}

static int fsa9480_detect_callback(struct otg_id_notifier_block *nb)
{
	struct usbsw_nb_info *nb_info =
			container_of(nb, struct usbsw_nb_info, otg_id_nb);
	struct fsa9480_usbsw *usbsw = nb_info->usbsw;
	struct i2c_client *client = usbsw->client;
	u16 dev_type;
	u8 adc_val;
	u32 prev_dev;

	usbsw->pdata->enable(true);

	dev_type = i2c_smbus_read_word_data(client, FSA9480_REG_DEV_T1);
	adc_val = i2c_smbus_read_byte_data(client, FSA9480_REG_ADC);
	if (dev_type < 0 || adc_val < 0) {
		dev_err(&client->dev, "error reading adc/dev_type regs\n");
		goto err;
	}

	dev_dbg(&client->dev, "trying detect (prio=%d): type=%x adc=%x\n",
		nb_info->detect_set->prio, dev_type, adc_val);

	mutex_lock(&usbsw->lock);
	prev_dev = usbsw->curr_dev;

	if (dev_type & DEV_USB_MASK) {
		/* usb peripheral mode */
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_USB))
			goto unhandled;
		_detected(usbsw, FSA9480_DETECT_USB);
		goto handled;
	} else if (dev_type & DEV_UART_MASK) {
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_UART))
			goto unhandled;
		_detected(usbsw, FSA9480_DETECT_UART);
		goto handled;
	} else if (dev_type & DEV_CHARGER_MASK) {
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_CHARGER))
			goto unhandled;
		_detected(usbsw, FSA9480_DETECT_CHARGER);
		goto handled;
	} else if (dev_type & DEV_JIG_MASK) {
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_JIG))
			goto unhandled;
		_detected(usbsw, FSA9480_DETECT_JIG);
		goto handled;
	}

unhandled:
	usbsw->curr_dev = 0;
	if (nb_info->detect_set->fallback) {
		/* In this case, we are the last resort and we are supposed to
		 * keep ownership of ID/D+/D- to monitor them for changes.
		 * This can happen when no one else
		 * detected a valid device and it is not one of the above.
		 *
		 * If this is the case, then one of the possibilities is that
		 * we are going into host mode and there is a usb peripheral
		 * plugged into the usb port with a dongle that pulls ID pin
		 * low.
		 */

		/* the host-port dongle is detected as A/V cable, and ADC
		 * reading is 0x0 */
		if (((dev_type & DEV_AV) && (adc_val == 0x0)) &&
		    (nb_info->detect_set->mask & FSA9480_DETECT_USB_HOST)) {
			dev_dbg(&client->dev, "host mode detected\n");
			_detected(usbsw, FSA9480_DETECT_USB_HOST);
			goto handled;
		}

		dev_info(&usbsw->client->dev,
			 "nothing known attached, keeping ownership of port\n");
		goto handled;
	}

	mutex_unlock(&usbsw->lock);

err:
	usbsw->pdata->enable(false);
	return OTG_ID_UNHANDLED;

handled:
	BUG_ON((usbsw->curr_dev == FSA9480_DETECT_NONE) &&
	       (prev_dev != FSA9480_DETECT_NONE));
	mutex_unlock(&usbsw->lock);
	enable_irq(usbsw->client->irq);

	return OTG_ID_HANDLED;
}

static void fsa9480_cancel_callback(struct otg_id_notifier_block *nb)
{
	struct usbsw_nb_info *nb_info =
			container_of(nb, struct usbsw_nb_info, otg_id_nb);
	struct fsa9480_usbsw *usbsw = nb_info->usbsw;
	struct i2c_client *client = usbsw->client;

	dev_info(&client->dev, "cancelling");
}

static irqreturn_t fsa9480_irq_thread(int irq, void *data)
{
	struct fsa9480_usbsw *usbsw = data;
	struct i2c_client *client = usbsw->client;
	s32 intr;

	/* read and clear interrupt status bits */
	intr = i2c_smbus_read_word_data(client, FSA9480_REG_INT1);
	if (intr < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, intr);
	} else if (intr == 0) {
		dev_warn(&client->dev, "irq fired, but nothing happened\n");
	} else {
		dev_dbg(&client->dev, "got irq 0x%x\n", intr);
	}

	disable_irq_nosync(client->irq);

	mutex_lock(&usbsw->lock);
	if (usbsw->curr_dev != FSA9480_DETECT_NONE) {
		_detected(usbsw, FSA9480_DETECT_NONE);

		/* undo whatever else we did */
	}
	mutex_unlock(&usbsw->lock);

	otg_id_notify();

	return IRQ_HANDLED;
}

static int __devinit fsa9480_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct fsa9480_platform_data *pdata = client->dev.platform_data;
	struct fsa9480_usbsw *usbsw;
	int ret = 0;
	int i;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	if (!pdata || !pdata->detected || !pdata->enable) {
		dev_err(&client->dev, "missing/invalid platform data\n");
		return -EINVAL;
	}

	usbsw = kzalloc(sizeof(struct fsa9480_usbsw) +
			pdata->num_sets * sizeof(struct usbsw_nb_info),
			GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	usbsw->client = client;
	usbsw->pdata = pdata;

	i2c_set_clientdata(client, usbsw);
	mutex_init(&usbsw->lock);

	/* mask all irqs to prevent event processing between
	 * request_irq and disable_irq
	 */
	i2c_smbus_write_word_data(client, FSA9480_REG_INT1_MASK, 0x1fff);

	ret = request_threaded_irq(client->irq, NULL, fsa9480_irq_thread,
				   IRQF_TRIGGER_FALLING, "fsa9480", usbsw);
	if (ret) {
		dev_err(&client->dev, "failed to request IRQ\n");
		goto err_req_irq;
	}
	disable_irq(client->irq);

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to enable wakeup src %d\n", ret);
		goto err_en_wake;
	}

	ret = fsa9480_reg_init(usbsw);
	if (ret)
		goto err_reg_init;

	ret = sysfs_create_group(&client->dev.kobj, &fsa9480_group);
	if (ret) {
		dev_err(&client->dev,
				"failed to create fsa9480 attribute group\n");
		goto err_sys_create;
	}

	usbsw->num_notifiers = pdata->num_sets;
	for (i = 0; i < usbsw->num_notifiers; i++) {
		struct usbsw_nb_info *info = &usbsw->notifiers[i];

		info->detect_set = &pdata->detect_sets[i];
		info->usbsw = usbsw;
		info->otg_id_nb.detect = fsa9480_detect_callback;
		info->otg_id_nb.cancel = fsa9480_cancel_callback;
		info->otg_id_nb.priority = pdata->detect_sets[i].prio;

		ret = otg_id_register_notifier(&info->otg_id_nb);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to register notifier\n");
			goto err_reg_notifiers;
		}
	}

	return 0;

err_reg_notifiers:
	for (i--; i >= 0; i--)
		otg_id_unregister_notifier(&usbsw->notifiers[i].otg_id_nb);
	sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
err_sys_create:
err_reg_init:
	if (client->irq)
		disable_irq_wake(client->irq);
err_en_wake:
	if (client->irq)
		free_irq(client->irq, usbsw);
err_req_irq:
	mutex_destroy(&usbsw->lock);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int __devexit fsa9480_remove(struct i2c_client *client)
{
	struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < usbsw->num_notifiers; i++)
		otg_id_unregister_notifier(&usbsw->notifiers[i].otg_id_nb);

	if (usbsw->curr_dev != FSA9480_DETECT_NONE)
		_detected(usbsw, FSA9480_DETECT_NONE);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, usbsw);
	}

	i2c_set_clientdata(client, NULL);

	sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
	mutex_destroy(&usbsw->lock);

	kfree(usbsw);

	return 0;
}

static const struct i2c_device_id fsa9480_id[] = {
	{"fsa9480", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_i2c_driver = {
	.driver = {
		.name = "fsa9480",
	},
	.probe = fsa9480_probe,
	.remove = __devexit_p(fsa9480_remove),
	.id_table = fsa9480_id,
};

static int __init fsa9480_init(void)
{
	return i2c_add_driver(&fsa9480_i2c_driver);
}
module_init(fsa9480_init);

static void __exit fsa9480_exit(void)
{
	i2c_del_driver(&fsa9480_i2c_driver);
}
module_exit(fsa9480_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("FSA9480 USB Switch driver");
MODULE_LICENSE("GPL");
