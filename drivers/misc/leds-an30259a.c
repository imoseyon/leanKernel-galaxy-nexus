/*
 * leds_an30259a.c - driver for panasonic AN30259A led control chip
 *
 * Copyright (C) 2011, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 * Contact: Yufi Li <tai-yun.li@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds-an30259a.h>

/* AN30259A register map */
#define AN30259A_REG_SRESET		0x00
#define AN30259A_REG_LEDON		0x01
#define AN30259A_REG_SEL		0x02

#define AN30259A_REG_LED1CC		0x03
#define AN30259A_REG_LED2CC		0x04
#define AN30259A_REG_LED3CC		0x05

#define AN30259A_REG_LED1SLP		0x06
#define AN30259A_REG_LED2SLP		0x07
#define AN30259A_REG_LED3SLP		0x08

#define AN30259A_REG_LED1CNT1		0x09
#define AN30259A_REG_LED1CNT2		0x0a
#define AN30259A_REG_LED1CNT3		0x0b
#define AN30259A_REG_LED1CNT4		0x0c

#define AN30259A_REG_LED2CNT1		0x0d
#define AN30259A_REG_LED2CNT2		0x0e
#define AN30259A_REG_LED2CNT3		0x0f
#define AN30259A_REG_LED2CNT4		0x10

#define AN30259A_REG_LED3CNT1		0x11
#define AN30259A_REG_LED3CNT2		0x12
#define AN30259A_REG_LED3CNT3		0x13
#define AN30259A_REG_LED3CNT4		0x14
#define AN30259A_REG_MAX		0x15
/* MASK */
#define AN30259A_MASK_IMAX		0xc0
#define AN30259A_MASK_DELAY		0xf0
#define AN30259A_SRESET			0x01
#define LED_SLOPE_MODE			0x10
#define LED_ON				0x01

#define DUTYMAX_MAX_VALUE		0x7f
#define DUTYMIN_MIN_VALUE		0x00
#define SLPTT_MAX_VALUE			0x0f

#define DETENTION_MAX_VALUE		60
#define DELAY_MAX_VALUE			7500
#define AN30259A_TIME_UNIT		500
#define AN30259A_DT_TIME_UNIT		4

#define LED_R_MASK			0x00ff0000
#define LED_G_MASK			0x0000ff00
#define LED_B_MASK			0x000000ff
#define LED_R_SHIFT			16
#define LED_G_SHIFT			8
#define LED_IMAX_SHIFT			6
#define AN30259A_CTN_RW_FLG		0x80

enum an30259a_led {
	LED_R,
	LED_G,
	LED_B,
};

struct an30259a_data {
	struct i2c_client	*client;
	struct miscdevice	dev;
	struct mutex		mutex;
	u8			shadow_reg[AN30259A_REG_MAX];
};

static int leds_i2c_write_all(struct i2c_client *client)
{
	struct an30259a_data *data = i2c_get_clientdata(client);
	int ret;

	/*we need to set all the configs setting first, then LEDON later*/
	ret = i2c_smbus_write_i2c_block_data(client,
			AN30259A_REG_SEL | AN30259A_CTN_RW_FLG,
			AN30259A_REG_MAX - AN30259A_REG_SEL,
			&data->shadow_reg[AN30259A_REG_SEL]);
	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c block write\n",
			__func__);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, AN30259A_REG_LEDON,
					data->shadow_reg[AN30259A_REG_LEDON]);

	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c byte write\n",
			__func__);
		return ret;
	}

	return 0;
}

/*
 * leds_set_slope_mode() sets correct values to corresponding shadow registers.
 * led: stands for LED_R or LED_G or LED_B.
 * delay: represents for starting delay time in multiple of .5 second.
 * dutymax: led at slope lighting maximum PWM Duty setting.
 * dutymid: led at slope lighting middle PWM Duty setting.
 * dutymin: led at slope lighting minimum PWM Duty Setting.
 * slptt1: total time of slope operation 1 and 2, in multiple of .5 second.
 * slptt2: total time of slope operation 3 and 4, in multiple of .5 second.
 * dt1: detention time at each step in slope operation 1, in multiple of 4ms.
 * dt2: detention time at each step in slope operation 2, in multiple of 4ms.
 * dt3: detention time at each step in slope operation 3, in multiple of 4ms.
 * dt4: detention time at each step in slope operation 4, in multiple of 4ms.
 */
static void leds_set_slope_mode(struct i2c_client *client,
				enum an30259a_led led, u8 delay,
				u8 dutymax, u8 dutymid, u8 dutymin,
				u8 slptt1, u8 slptt2,
				u8 dt1, u8 dt2, u8 dt3, u8 dt4)
{
	struct an30259a_data *data = i2c_get_clientdata(client);

	data->shadow_reg[AN30259A_REG_LED1CNT1 + led * 4] =
							dutymax << 4 | dutymid;
	data->shadow_reg[AN30259A_REG_LED1CNT2 + led * 4] =
							delay << 4 | dutymin;
	data->shadow_reg[AN30259A_REG_LED1CNT3 + led * 4] = dt2 << 4 | dt1;
	data->shadow_reg[AN30259A_REG_LED1CNT4 + led * 4] = dt4 << 4 | dt3;
	data->shadow_reg[AN30259A_REG_LED1SLP + led] = slptt2 << 4 | slptt1;
}

static void leds_on(struct i2c_client *client, enum an30259a_led led,
			bool on, bool slopemode,
			u8 ledcc)
{
	struct an30259a_data *data = i2c_get_clientdata(client);

	if (on)
		data->shadow_reg[AN30259A_REG_LEDON] |= LED_ON << led;
	else {
		data->shadow_reg[AN30259A_REG_LEDON] &= ~(LED_ON << led);
		data->shadow_reg[AN30259A_REG_LED1CNT2 + led * 4] &=
							~AN30259A_MASK_DELAY;
	}
	if (slopemode)
		data->shadow_reg[AN30259A_REG_LEDON] |= LED_SLOPE_MODE << led;
	else
		data->shadow_reg[AN30259A_REG_LEDON] &=
						~(LED_SLOPE_MODE << led);

	data->shadow_reg[AN30259A_REG_LED1CC + led] = ledcc;
}

/* calculate the detention time for each step, return ms */
static u8 calculate_dt(u8 min, u8 max, u16 time)
{
	u16 step_time;
	u16 detention_time;

	if (min >= max)
		return 0;

	step_time = time / (u16)(max - min);
	detention_time = (step_time + 0x03) & 0xfffc;
	/* the detention time at each step can be set as 4ms, 8ms, ...60ms */
	detention_time = (detention_time > DETENTION_MAX_VALUE) ?
				DETENTION_MAX_VALUE : detention_time;
	return detention_time;
}

/* calculate the constant current output */
static u8 calculate_cc(u32 brightness, enum an30259a_led led)
{
	u8 value = 0;
	switch (led) {
	case LED_R:
		value = (brightness & LED_R_MASK) >> LED_R_SHIFT;
		break;
	case LED_G:
		value = (brightness & LED_G_MASK) >> LED_G_SHIFT;
		break;
	case LED_B:
		value = brightness & LED_B_MASK;
		break;
	default:
		break;
	}
	return value;
}

static u8 calculate_slope_tt(u8 mid, u8 dt1, u8 dt2, u16 time)
{
	u8 slptt;
	u16 tt = (dt1 * (mid - DUTYMIN_MIN_VALUE) +
		dt2 * (DUTYMAX_MAX_VALUE - mid)) * AN30259A_DT_TIME_UNIT + time;
	/* round up to the nearest .5 seconds */
	slptt = (tt + AN30259A_TIME_UNIT - 1) / AN30259A_TIME_UNIT;
	slptt = slptt > SLPTT_MAX_VALUE ? SLPTT_MAX_VALUE : slptt;
	return slptt;
}

static int leds_handle_cmds(struct i2c_client *client,
				enum an30259a_led color,
				struct an30259a_pr_control *leds)
{
	u8 cc, delay, dutymid, dt1, dt2, dt3, dt4, tt1, tt2;

	cc = calculate_cc(leds->color, color);
	switch (leds->state) {
	case LED_LIGHT_OFF:
		leds_on(client, color, false, false, 0);
		break;
	case LED_LIGHT_ON:
		leds_on(client, color, true, false, cc);
		break;
	case LED_LIGHT_PULSE:
		/*
		 * PULSE is a special case of slope with delay=0,
		 * dutymid = dutymax ,dt1,dt2,dt3,dt4 are all zero
		 */
		leds_on(client, color, true, true, cc);
		leds_set_slope_mode(client, color, 0,
				DUTYMAX_MAX_VALUE >> 3, DUTYMAX_MAX_VALUE >> 3,
				DUTYMIN_MIN_VALUE >> 3,
				(leds->time_on + AN30259A_TIME_UNIT - 1) /
				AN30259A_TIME_UNIT,
				(leds->time_off + AN30259A_TIME_UNIT - 1) /
				AN30259A_TIME_UNIT,
				0, 0, 0, 0);

		break;
	case LED_LIGHT_SLOPE:
		if (leds->mid_brightness > DUTYMAX_MAX_VALUE)
			return -EINVAL;

		delay = ((leds->start_delay > DELAY_MAX_VALUE) ?
			DELAY_MAX_VALUE : leds->start_delay) /
			AN30259A_TIME_UNIT;

		dutymid = leds->mid_brightness >> 3;
		dt1 = calculate_dt(DUTYMIN_MIN_VALUE, leds->mid_brightness,
				leds->time_slope_up_1) / AN30259A_DT_TIME_UNIT;
		dt2 = calculate_dt(leds->mid_brightness, DUTYMAX_MAX_VALUE,
				leds->time_slope_up_2) / AN30259A_DT_TIME_UNIT;
		dt3 = calculate_dt(leds->mid_brightness, DUTYMAX_MAX_VALUE,
				leds->time_slope_down_1) /
				AN30259A_DT_TIME_UNIT;
		dt4 = calculate_dt(DUTYMIN_MIN_VALUE, leds->mid_brightness,
				leds->time_slope_down_2) /
				AN30259A_DT_TIME_UNIT;
		tt1 = calculate_slope_tt(leds->mid_brightness,
				dt1, dt2, leds->time_on);
		tt2 = calculate_slope_tt(leds->mid_brightness,
				dt4, dt3, leds->time_off);

		leds_on(client, color, true, true, cc);
		leds_set_slope_mode(client, color, delay,
			DUTYMAX_MAX_VALUE >> 3, dutymid, DUTYMIN_MIN_VALUE >> 3,
			tt1, tt2, dt1, dt2, dt3, dt4);
		break;
	}

	return 0;
}

static int leds_set_imax(struct i2c_client *client, u8 imax)
{
	int ret;
	struct an30259a_data *data = i2c_get_clientdata(client);

	data->shadow_reg[AN30259A_REG_SEL] &= ~AN30259A_MASK_IMAX;
	data->shadow_reg[AN30259A_REG_SEL] |= imax << LED_IMAX_SHIFT;

	ret = i2c_smbus_write_byte_data(client, AN30259A_REG_SEL,
			data->shadow_reg[AN30259A_REG_SEL]);
	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c write\n",
			__func__);
	}
	return 0;
}

static long an30250a_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct an30259a_data *leds_data = container_of(file->private_data,
						struct an30259a_data, dev);
	struct i2c_client *client = leds_data->client;
	int retval, i;
	u8 imax;
	struct an30259a_pr_control leds[3];

	mutex_lock(&leds_data->mutex);

	switch (cmd) {
	case AN30259A_PR_SET_LED:
		retval = copy_from_user(leds, (unsigned char __user *)arg,
					sizeof(struct an30259a_pr_control));
		if (retval)
			break;

		for (i = LED_R; i <= LED_B; i++) {
			retval = leds_handle_cmds(client, i, leds);
			if (retval < 0)
				goto an30259a_ioctl_failed;
		}
		retval = leds_i2c_write_all(client);
		break;
	case AN30259A_PR_SET_LEDS:
		retval = copy_from_user(leds, (unsigned char __user *)arg,
					3 * sizeof(struct an30259a_pr_control));

		if (retval)
			break;

		for (i = LED_R; i <= LED_B; i++) {
			retval = leds_handle_cmds(client, i, &leds[i]);
			if (retval < 0)
				goto an30259a_ioctl_failed;
		}
		retval = leds_i2c_write_all(client);
		break;

	case AN30259A_PR_SET_IMAX:
		retval = copy_from_user(&imax, (unsigned char __user *)arg,
					sizeof(u8));
		if (retval)
			break;

		retval = leds_set_imax(client, imax);
		break;

	default:
		dev_err(&client->adapter->dev,
			"%s: Unknown cmd %x, arg %lu\n",
			__func__, cmd, arg);
		retval = -EINVAL;
		break;
	}

an30259a_ioctl_failed:
	mutex_unlock(&leds_data->mutex);
	return retval;
}

static int an30250a_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations an30259a_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = an30250a_ioctl,
	.open = an30250a_open,
};

static int __devinit an30259a_initialize(struct i2c_client *client)
{
	struct an30259a_data *data = i2c_get_clientdata(client);
	int ret;

	/* reset an30259a*/
	ret = i2c_smbus_write_byte_data(client, AN30259A_REG_SRESET,
					AN30259A_SRESET);
	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c write (reg = 0x%2x)\n",
			__func__, AN30259A_REG_SRESET);
		return ret;
	}
	ret = i2c_smbus_read_i2c_block_data(client,
			AN30259A_REG_SRESET | AN30259A_CTN_RW_FLG,
			AN30259A_REG_MAX, data->shadow_reg);
	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c read block(ledxcc)\n",
			__func__);
		return ret;
	}

	return 0;
}

static int __devinit an30259a_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct an30259a_data *data;
	int ret;

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C.\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->adapter->dev,
			"failed to allocate driver data.\n");
		return -ENOMEM;
	}

	data->client = client;

	i2c_set_clientdata(client, data);

	mutex_init(&data->mutex);

	/* initialize LED */
	ret = an30259a_initialize(client);
	if (ret < 0) {
		dev_err(&client->adapter->dev, "failure on initialization\n");
		goto exit;
	}

	data->dev.minor = MISC_DYNAMIC_MINOR;
	data->dev.name = "an30259a_leds";
	data->dev.fops = &an30259a_fops;
	ret = misc_register(&data->dev);
	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: ERROR: misc_register returned %d\n",
			__func__, ret);
		goto exit;
	}

	return 0;
exit:
	mutex_destroy(&data->mutex);
	kfree(data);
	return ret;
}

static int __devexit an30259a_remove(struct i2c_client *client)
{
	struct an30259a_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	misc_deregister(&data->dev);
	mutex_destroy(&data->mutex);
	kfree(data);
	return 0;
}

static struct i2c_device_id an30259a_id[] = {
	{"an30259a", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, an30259a_id);

static struct i2c_driver an30259a_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "an30259a",
	},
	.id_table = an30259a_id,
	.probe = an30259a_probe,
	.remove = __devexit_p(an30259a_remove),
};

static int __init an30259a_init(void)
{
	return i2c_add_driver(&an30259a_i2c_driver);
}

static void __exit an30259a_exit(void)
{
	i2c_del_driver(&an30259a_i2c_driver);
}

module_init(an30259a_init);
module_exit(an30259a_exit);

MODULE_DESCRIPTION("AN30259A LED driver");
MODULE_AUTHOR("Yufi Li <tai-yun.li@samsung.com");
MODULE_LICENSE("GPL v2");
