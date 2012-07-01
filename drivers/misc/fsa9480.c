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
#include <linux/gpio.h>
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
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/fastchg.h>

#define DEBUG_DUMP_REGISTERS

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
#define FSA9480_REG_ANALOG_TEST		0x15
#define FSA9480_REG_SCAN_TEST		0x16
#define FSA9480_REG_DAC_OVERRIDE_1	0x17
#define FSA9480_REG_DAC_OVERRIDE_2	0x18
#define FSA9480_REG_VIDEO_DETECT	0x19
#define FSA9480_REG_CK_PULSE_WIDTH	0x1A
#define FSA9480_REG_MANOVERRIDE1	0x1B
#define FSA9480_REG_STATUS1		0x1C
#define FSA9480_REG_STATUS2		0x1D
#define FSA9480_REG_FUSE1		0x1E

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

#define DEV_USB_MASK		(DEV_USB | DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
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

/* Interrupt Mask */
#define INT_STUCK_KEY_RCV	(1 << 12)
#define INT_STUCK_KEY		(1 << 11)
#define INT_ADC_CHANGE		(1 << 10)
#define INT_RESERVE_ATTACH	(1 << 9)
#define INT_AV_CHARGING		(1 << 8)
#define INT_OVP_OCP_DIS		(1 << 7)
#define INT_OCP_EN		(1 << 6)
#define INT_OVP_EN		(1 << 5)
#define INT_LKR			(1 << 4)
#define INT_LKP			(1 << 3)
#define INT_KP			(1 << 2)
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
	[FSA9480_DETECT_NONE]			= "unknown/none",
	[FSA9480_DETECT_USB]			= "usb-peripheral",
	[FSA9480_DETECT_USB_HOST]		= "usb-host",
	[FSA9480_DETECT_CHARGER]		= "charger",
	[FSA9480_DETECT_JIG]			= "jig",
	[FSA9480_DETECT_UART]			= "uart",
	[FSA9480_DETECT_AV_365K]		= "av-365k",
	[FSA9480_DETECT_AV_365K_CHARGER]	= "av-365k-charger",
	[FSA9480_DETECT_AV_POWERED]		= "av-powered",
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
	u16				intr_mask;
	u8				timing;
	int				external_id_irq;
	bool				wake_enabled;
#if defined(CONFIG_DEBUG_FS) && defined(DEBUG_DUMP_REGISTERS)
	struct dentry			*debug_dir;
#endif

	int				num_notifiers;
	struct usbsw_nb_info		notifiers[0];
};
#define xceiv_to_fsa(x)		container_of((x), struct fsa9480_usbsw, otg)

#if defined(CONFIG_DEBUG_FS) && defined(DEBUG_DUMP_REGISTERS)

#define DUMP_FSA9480_REG(client, m, x) ({				\
	int __val;							\
	__val = i2c_smbus_read_byte_data((client), FSA9480_REG_##x);	\
	seq_printf((m), "%s = 0x%02x\n", #x, __val);			\
	__val;								\
})

static int fsa9480_show_registers(struct seq_file *m, void *p)
{
	struct fsa9480_usbsw *usbsw = m->private;

	DUMP_FSA9480_REG(usbsw->client, m, DEVID);
	DUMP_FSA9480_REG(usbsw->client, m, CTRL);
	DUMP_FSA9480_REG(usbsw->client, m, INT1);
	DUMP_FSA9480_REG(usbsw->client, m, INT2);
	DUMP_FSA9480_REG(usbsw->client, m, INT1_MASK);
	DUMP_FSA9480_REG(usbsw->client, m, INT2_MASK);
	DUMP_FSA9480_REG(usbsw->client, m, ADC);
	DUMP_FSA9480_REG(usbsw->client, m, TIMING1);
	DUMP_FSA9480_REG(usbsw->client, m, TIMING2);
	DUMP_FSA9480_REG(usbsw->client, m, DEV_T1);
	DUMP_FSA9480_REG(usbsw->client, m, DEV_T2);
	DUMP_FSA9480_REG(usbsw->client, m, BTN1);
	DUMP_FSA9480_REG(usbsw->client, m, BTN2);
	DUMP_FSA9480_REG(usbsw->client, m, CK);
	DUMP_FSA9480_REG(usbsw->client, m, CK_INT1);
	DUMP_FSA9480_REG(usbsw->client, m, CK_INT2);
	DUMP_FSA9480_REG(usbsw->client, m, CK_INTMASK1);
	DUMP_FSA9480_REG(usbsw->client, m, CK_INTMASK2);
	DUMP_FSA9480_REG(usbsw->client, m, MANSW1);
	DUMP_FSA9480_REG(usbsw->client, m, MANSW2);
	DUMP_FSA9480_REG(usbsw->client, m, BTN1);
	DUMP_FSA9480_REG(usbsw->client, m, BTN2);
	DUMP_FSA9480_REG(usbsw->client, m, ANALOG_TEST);
	DUMP_FSA9480_REG(usbsw->client, m, SCAN_TEST);
	DUMP_FSA9480_REG(usbsw->client, m, DAC_OVERRIDE_1);
	DUMP_FSA9480_REG(usbsw->client, m, DAC_OVERRIDE_2);
	DUMP_FSA9480_REG(usbsw->client, m, VIDEO_DETECT);
	DUMP_FSA9480_REG(usbsw->client, m, CK_PULSE_WIDTH);
	DUMP_FSA9480_REG(usbsw->client, m, MANOVERRIDE1);
	DUMP_FSA9480_REG(usbsw->client, m, STATUS1);
	DUMP_FSA9480_REG(usbsw->client, m, STATUS2);
	DUMP_FSA9480_REG(usbsw->client, m, FUSE1);

	return 0;
}

static int fsa9480_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, fsa9480_show_registers, inode->i_private);
}

static const struct file_operations fsa9480_regs_fops = {
	.open = fsa9480_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

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

	value = i2c_smbus_read_word_data(client, FSA9480_REG_DEV_T1);
	if (value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, value);
		return (ssize_t)value;
	}

	return sprintf(buf, "%04x\n", value);
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

	ret = i2c_smbus_write_word_data(client, FSA9480_REG_INT1_MASK,
			usbsw->intr_mask);
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

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_TIMING1,
			usbsw->timing);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_MANSW1,
			usbsw->mansw);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	if (usbsw->mansw)
		ctrl &= ~CON_MANUAL_SW;	/* Manual Switching Mode */

	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_CTRL, ctrl);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int fsa9480_reset(struct fsa9480_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	s32 ret;

	/* soft reset to re-initialize the fsa, and re-do detection */
	ret = i2c_smbus_write_byte_data(client, FSA9480_REG_MANOVERRIDE1, 1);
	if (ret < 0) {
		dev_err(&client->dev, "cannot soft reset,  err %d\n", ret);
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
	int max_events = 100;

	mutex_lock(&usbsw->lock);

	usbsw->pdata->enable(true);

	/* the fsa could have queued up a few events if we haven't processed
	 * them promptly
	 */
	while (max_events-- > 0) {
		s32 ret = i2c_smbus_read_word_data(client, FSA9480_REG_INT1);
		if (!ret)
			break;
	}
	if (!max_events)
		dev_warn(&client->dev, "too many events. fsa hosed?\n");

	/* fsa may take some time to update the dev_type reg after reading
	 * the int reg.
	 */
	usleep_range(200, 300);

	dev_type = i2c_smbus_read_word_data(client, FSA9480_REG_DEV_T1);
	adc_val = i2c_smbus_read_byte_data(client, FSA9480_REG_ADC);
	if (dev_type < 0 || adc_val < 0) {
		dev_err(&client->dev, "error reading adc/dev_type regs\n");
		goto err;
	}

	dev_dbg(&client->dev, "trying detect (prio=%d): type=%x adc=%x\n",
		nb_info->detect_set->prio, dev_type, adc_val);

	prev_dev = usbsw->curr_dev;

	if (dev_type & DEV_USB_MASK) {
		/* If there is an external id signal then verify that the ID
		 * signal is floating.  If the ID signal is pulled low then this
		 * may be a cable misidentification.  This can occur if the
		 * board allows for the ID signal to be redirected away from the
		 * FSA9480.  If the ID signal is not visible to the FSA9480 and
		 * VBUS is present then the cable will be identified as a USB
		 * peripheral cable.
		 *
		 * In the event of a cable misidentification the FSA9480 chip
		 * will be reset to force a new detection cycle.
		 */
		if (usbsw->pdata->external_id >= 0 &&
				!gpio_get_value(usbsw->pdata->external_id)) {
			dev_info(&usbsw->client->dev, "Cable misidentified as "
					"a USB-peripheral cable, resetting the "
					"FSA9480\n");
			fsa9480_reset(usbsw);
			goto handled;
		}

		/* usb peripheral mode */
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_USB))
			goto unhandled;
#ifdef CONFIG_FORCE_FAST_CHARGE
		if (force_fast_charge != 0) {
		_detected(usbsw, FSA9480_DETECT_CHARGER);
		} else {
		_detected(usbsw, FSA9480_DETECT_USB);
		}
#else
		_detected(usbsw, FSA9480_DETECT_USB);
#endif
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
	} else if (dev_type & DEV_USB_OTG) {
		if (!(nb_info->detect_set->mask & FSA9480_DETECT_USB_HOST))
			goto unhandled;
		_detected(usbsw, FSA9480_DETECT_USB_HOST);

		mutex_unlock(&usbsw->lock);

		/* Enable the external ID interrupt to detect the detach of the
		 * USB host cable since the FSA9480 is unable to detect it.
		 * The FSA9480 takes a while pulling that line down, so a sleep
		 * is needed.
		 */
		usleep_range(10000, 11000);
		enable_irq(usbsw->external_id_irq);
		return OTG_ID_HANDLED;
	} else if (dev_type & DEV_AV) {
		/* There are two ID resistances, 1K and 365K that the FSA9480
		 * will resolve to the A/V Cable device type.  The ADC value can
		 * be used to tell the difference between the two.
		 */
		if (adc_val == 0x1a) {
			/* Delay to allow VBUS to be seen, if present. There's
			 * a possibility that we won't charge if it takes
			 * longer than this for VBUS to be present. */
			msleep(10);
			if ((nb_info->detect_set->mask &
					FSA9480_DETECT_AV_365K_CHARGER) &&
					usbsw->pdata->vbus_present()) {
				_detected(usbsw,
					FSA9480_DETECT_AV_365K_CHARGER);
				/* The FSA9480 will not interrupt when a USB or
				 * charger cable is disconnected from the dock
				 * so we must detect loss of VBUS via an
				 * external interrupt. */
				enable_irq(usbsw->pdata->external_vbus_irq);
				mutex_unlock(&usbsw->lock);
				return OTG_ID_HANDLED;
			} else if ((nb_info->detect_set->mask &
					FSA9480_DETECT_AV_365K) &&
					!usbsw->pdata->vbus_present()) {
				_detected(usbsw, FSA9480_DETECT_AV_365K);
			} else {
				goto unhandled;
			}
			goto handled;
		} else if ((nb_info->detect_set->mask &
				FSA9480_DETECT_AV_POWERED) &&
				usbsw->pdata->vbus_present()) {
			_detected(usbsw, FSA9480_DETECT_AV_POWERED);
			enable_irq(usbsw->pdata->external_vbus_irq);
			mutex_unlock(&usbsw->lock);
			return OTG_ID_HANDLED;
		}
	} else if (dev_type == 0) {
		usbsw->curr_dev = 0;
		dev_info(&usbsw->client->dev,
			 "nothing attached, keeping ownership of port\n");
		goto handled;
	}

unhandled:
	usbsw->curr_dev = 0;
	if (nb_info->detect_set->fallback) {
		/* In this case, we are the last resort and we are supposed to
		 * keep ownership of ID/D+/D- to monitor them for changes.
		 * This can happen when no one else
		 * detected a valid device and it is not one of the above.
		 */

		dev_info(&usbsw->client->dev,
			 "nothing known attached, keeping ownership of port\n");
		goto handled;
	}


err:
	usbsw->pdata->enable(false);
	mutex_unlock(&usbsw->lock);
	return OTG_ID_UNHANDLED;

handled:
	BUG_ON((usbsw->curr_dev == FSA9480_DETECT_NONE) &&
	       (prev_dev != FSA9480_DETECT_NONE));

	mutex_unlock(&usbsw->lock);
	enable_irq_wake(usbsw->client->irq);
	enable_irq(usbsw->client->irq);

	return OTG_ID_HANDLED;
}

static int fsa9480_proxy_wait_callback(struct otg_id_notifier_block *nb)
{
	struct usbsw_nb_info *nb_info =
			container_of(nb, struct usbsw_nb_info, otg_id_nb);
	struct fsa9480_usbsw *usbsw = nb_info->usbsw;

	dev_info(&usbsw->client->dev, "taking proxy ownership of port\n");

	usbsw->pdata->enable(true);
	enable_irq_wake(usbsw->client->irq);
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
		intr = 0;
	} else if (intr == 0) {
		/* When the FSA9480 triggers an interrupt with no status bits
		 * set the FSA9480 may have reset and the registers need to be
		 * reinitialized.
		 */
		fsa9480_reg_init(usbsw);
		dev_warn(&client->dev, "irq fired, but nothing happened\n");
	} else {
		dev_dbg(&client->dev, "got irq 0x%x\n", intr);
	}

	if (intr & INT_OCP_EN)
		dev_err(&client->dev, "entering over-current protection\n");

	if (intr & INT_OVP_EN)
		dev_err(&client->dev, "entering over-voltage protection\n");

	if (intr & INT_OVP_OCP_DIS)
		dev_err(&client->dev, "exiting protection mode\n");

	disable_irq_nosync(client->irq);
	disable_irq_wake(client->irq);

	mutex_lock(&usbsw->lock);
	if (usbsw->curr_dev != FSA9480_DETECT_NONE) {
		_detected(usbsw, FSA9480_DETECT_NONE);

		/* undo whatever else we did */
	}
	mutex_unlock(&usbsw->lock);

	otg_id_notify();

	return IRQ_HANDLED;
}

static irqreturn_t usb_id_irq_thread(int irq, void *data)
{
	struct fsa9480_usbsw *usbsw = data;
	struct i2c_client *client = usbsw->client;

	mutex_lock(&usbsw->lock);

	/* The external ID interrupt is only used when a USB host cable is
	 * attached.
	 */
	if (usbsw->curr_dev != FSA9480_DETECT_USB_HOST) {
		disable_irq_nosync(usbsw->external_id_irq);
		mutex_unlock(&usbsw->lock);
		return IRQ_HANDLED;
	}

	/* The FSA9480 has a bug that prevents it from detecting a change in the
	 * ID signal when the device type is USB OTG.  As a workaround the
	 * driver uses an external mechanism to determine if the USB OTG cable
	 * has been detached.
	 */
	if (gpio_get_value(usbsw->pdata->external_id)) {
		disable_irq_nosync(usbsw->external_id_irq);

		usbsw->pdata->enable(true);

		/* If the client has been informed of the USB host attach then
		 * report the disconnect before reseting the FSA9480.  VBUS
		 * drive needs to be turned off before the reset otherwise the
		 * FSA9480 will misidentify the unattached state as a USB
		 * peripheral cable.
		 */
		_detected(usbsw, FSA9480_DETECT_NONE);

		dev_dbg(&client->dev, "usb host detach workaround, resetting"
				" FSA9480 chip\n");

		/* The FSA9480 will not be able to detect a new cable until it
		 * has been reset.
		 */
		fsa9480_reset(usbsw);

		enable_irq_wake(client->irq);
		enable_irq(client->irq);
	}

	mutex_unlock(&usbsw->lock);

	return IRQ_HANDLED;
}

static irqreturn_t vbus_irq_thread(int irq, void *data)
{
	struct fsa9480_usbsw *usbsw = data;

	disable_irq_nosync(usbsw->pdata->external_vbus_irq);

	mutex_lock(&usbsw->lock);
	if (usbsw->curr_dev != FSA9480_DETECT_AV_365K_CHARGER &&
			usbsw->curr_dev != FSA9480_DETECT_AV_POWERED) {
		mutex_unlock(&usbsw->lock);
		return IRQ_HANDLED;
	}

	/* VBUS has gone away when docked, so reset the state to
	 * FSA_DETECT_NONE and reset the FSA9480, because it cannot
	 * detect ID pin changes correctly after dock detach. */
	_detected(usbsw, FSA9480_DETECT_NONE);
	fsa9480_reset(usbsw);
	enable_irq_wake(usbsw->client->irq);
	enable_irq(usbsw->client->irq);
	mutex_unlock(&usbsw->lock);

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
	if (!pdata || !pdata->detected || !pdata->enable ||
			!pdata->mask_vbus_irq || !pdata->unmask_vbus_irq ||
			!pdata->vbus_present ||
			(pdata->external_vbus_irq < 0)) {
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

	if (usbsw->pdata->external_id >= 0) {
		gpio_request(usbsw->pdata->external_id, "fsa9840_external_id");

		usbsw->external_id_irq = gpio_to_irq(usbsw->pdata->external_id);

		ret = request_threaded_irq(usbsw->external_id_irq, NULL,
				usb_id_irq_thread,
				IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				"fsa9480_external_id", usbsw);
		if (ret) {
			dev_err(&client->dev,
					"failed to request ID IRQ err %d\n",
					ret);
			goto err_req_id_irq;
		}
	}

	pdata->mask_vbus_irq();
	ret = request_threaded_irq(pdata->external_vbus_irq, NULL,
			vbus_irq_thread, pdata->external_vbus_flags,
			"external_vbus", usbsw);
	if (ret) {
		dev_err(&client->dev,
				"failed to request vbus IRQ err %d\n",
				ret);
		goto err_req_vbus_irq;
	}
	disable_irq(pdata->external_vbus_irq);
	pdata->unmask_vbus_irq();

	/* mask all irqs to prevent event processing between
	 * request_irq and disable_irq
	 */
	usbsw->intr_mask = 0x1fff;
	i2c_smbus_write_word_data(client, FSA9480_REG_INT1_MASK,
			usbsw->intr_mask);

	ret = request_threaded_irq(client->irq, NULL, fsa9480_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fsa9480",
				   usbsw);
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
	disable_irq_wake(client->irq);

	/* Reconcile the requested ADC detect time with the available settings
	 * on the FSA9480.
	 */
	for (i = 0; i < ARRAY_SIZE(adc_timing); i++) {
		if (usbsw->pdata->detect_time <= adc_timing[i]) {
			usbsw->timing = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(adc_timing)) {
		ret = -ERANGE;
		goto err_timing;
	}

	/* mask interrupts (unmask attach/detach only) */
	usbsw->intr_mask = ~(INT_ATTACH | INT_DETACH | INT_OCP_EN | INT_OVP_EN |
			INT_OVP_OCP_DIS | INT_AV_CHARGING);
	ret = fsa9480_reset(usbsw);
	if (ret < 0)
		goto err_reset;

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
		info->otg_id_nb.proxy_wait = fsa9480_proxy_wait_callback;
		info->otg_id_nb.cancel = fsa9480_cancel_callback;
		info->otg_id_nb.priority = pdata->detect_sets[i].prio;

		ret = otg_id_register_notifier(&info->otg_id_nb);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to register notifier\n");
			goto err_reg_notifiers;
		}
	}
#if defined(CONFIG_DEBUG_FS) && defined(DEBUG_DUMP_REGISTERS)
	usbsw->debug_dir = debugfs_create_dir("fsa9480", NULL);

	if (usbsw->debug_dir)
		debugfs_create_file("regs", S_IRUSR, usbsw->debug_dir, usbsw,
				&fsa9480_regs_fops);
#endif

	return 0;

err_reg_notifiers:
	for (i--; i >= 0; i--)
		otg_id_unregister_notifier(&usbsw->notifiers[i].otg_id_nb);
	sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
err_sys_create:
err_reset:
err_timing:
err_reg_init:
err_en_wake:
	if (client->irq)
		free_irq(client->irq, usbsw);
err_req_irq:
	free_irq(usbsw->pdata->external_vbus_irq, usbsw);
err_req_vbus_irq:
	if (usbsw->pdata->external_id >= 0)
		free_irq(usbsw->external_id_irq, usbsw);
err_req_id_irq:
	if (usbsw->pdata->external_id >= 0)
		gpio_free(usbsw->pdata->external_id);
	mutex_destroy(&usbsw->lock);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int __devexit fsa9480_remove(struct i2c_client *client)
{
	struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
	int i;

#if defined(CONFIG_DEBUG_FS) && defined(DEBUG_DUMP_REGISTERS)
	if (usbsw->debug_dir)
		debugfs_remove_recursive(usbsw->debug_dir);
#endif

	for (i = 0; i < usbsw->num_notifiers; i++)
		otg_id_unregister_notifier(&usbsw->notifiers[i].otg_id_nb);

	if (usbsw->curr_dev != FSA9480_DETECT_NONE)
		_detected(usbsw, FSA9480_DETECT_NONE);

	if (client->irq) {
		disable_irq_wake(client->irq);
		free_irq(client->irq, usbsw);
	}

	if (usbsw->pdata->external_id >= 0) {
		if (usbsw->wake_enabled)
			disable_irq_wake(usbsw->external_id_irq);
		free_irq(usbsw->external_id_irq, usbsw);
		gpio_free(usbsw->pdata->external_id);
	}

	free_irq(usbsw->pdata->external_vbus_irq, usbsw);

	i2c_set_clientdata(client, NULL);

	sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
	mutex_destroy(&usbsw->lock);

	kfree(usbsw);

	return 0;
}

#if defined(CONFIG_PM)
static int fsa9480_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);

	if (usbsw->wake_enabled) {
		disable_irq_wake(usbsw->external_id_irq);
		usbsw->wake_enabled = false;
	}

	otg_id_resume();
	enable_irq(usbsw->external_id_irq);
	enable_irq(client->irq);

	return 0;
}

static int fsa9480_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
	int ret;

	disable_irq(client->irq);
	disable_irq(usbsw->external_id_irq);

	mutex_lock(&usbsw->lock);
	if (usbsw->curr_dev == FSA9480_DETECT_USB_HOST) {
		enable_irq_wake(usbsw->external_id_irq);
		usbsw->wake_enabled = true;
	}
	mutex_unlock(&usbsw->lock);

	ret = otg_id_suspend();
	if (ret)
		goto err;

	return 0;

err:
	if (usbsw->wake_enabled) {
		disable_irq_wake(usbsw->external_id_irq);
		usbsw->wake_enabled = false;
	}
	enable_irq(usbsw->external_id_irq);
	enable_irq(client->irq);
	return ret;
}

static const struct dev_pm_ops fsa9480_pm_ops = {
	.suspend	= fsa9480_suspend,
	.resume		= fsa9480_resume,
};
#endif

static const struct i2c_device_id fsa9480_id[] = {
	{"fsa9480", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_i2c_driver = {
	.driver = {
		.name = "fsa9480",
#if defined(CONFIG_PM)
		.pm	= &fsa9480_pm_ops,
#endif
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
