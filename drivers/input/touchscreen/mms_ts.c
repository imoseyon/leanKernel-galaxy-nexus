/*
 * mms_ts.c - Touchscreen driver for Melfas MMS-series touch controllers
 *
 * Copyright (C) 2011 Google Inc.
 * Author: Dima Zavin <dima@android.com>
 *         Simon Wilson <simonwilson@google.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

//#define DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <linux/platform_data/mms_ts.h>

#define MAX_FINGERS		10

/* Registers */
#define MMS_HEADER		0x10
#define MMS_HEADER_FINGER_INFO	0x11
#define MMS_X_POS_LO		0x12
#define MMS_X_POS_HI		0x13
#define MMS_Y_POS_LO		0x14
#define MMS_Y_POS_HI		0x15

struct mms_ts_info {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	char			phys[32];

	int			max_x;
	int			max_y;

	bool			invert_x;
	bool			invert_y;

	unsigned long		pressed;
};

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGERS * 4 + 2];
	unsigned long was_pressed = info->pressed;
	int ret;
	int i;
	int num_fingers;
	unsigned long pressed = 0;
	int x = 0;
	int y = 0;

	/* XXX: firmware bug, should be able to read the first finger info
	 * always */
	ret = i2c_smbus_read_i2c_block_data(client, MMS_HEADER, 2, buf);

#if defined(VERBOSE_DEBUG)
	print_hex_dump_bytes("mms_ts raw: ",  DUMP_PREFIX_NONE,
			     buf, sizeof(buf));
#endif

	num_fingers = buf[1] & 0xf;
	if (num_fingers < 0) {
		dev_err(&client->dev, "%s fingers=%d\n", __func__, num_fingers);
		goto out;
	}
	dev_dbg(&client->dev, "fingers: %d\n", num_fingers);

	WARN_ON(num_fingers > MAX_FINGERS);

	/* XXX: firmware bug, we should start from finger 2 if necessary */
	ret = i2c_smbus_read_i2c_block_data(client, MMS_HEADER,
					    num_fingers * 4 + 2, buf);
	for (i = 0; i < num_fingers; i++) {
		u8 *tmp = &buf[i*4 + 2];
		int id = (tmp[1] >> 3) & 0x1f;
		unsigned long finger_mask = 1 << id;

		x = tmp[0] | ((tmp[1] & 0x7) << 8);
		y = tmp[2] | ((tmp[3] & 0x7) << 8);

		if (info->invert_x)
			x = info->max_x - x;
		if (info->invert_y)
			y = info->max_y - y;
		if (x < 0)
			x = 0;
		if (y < 0)
			y = 0;

		pressed |= finger_mask;

		input_mt_slot(info->input_dev, id);
		input_mt_report_slot_state(info->input_dev,
					   MT_TOOL_FINGER, true);
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, 0xff);
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);

		dev_dbg(&client->dev,
			"finger %d (%d): x=%d y=%d\n", id, i, x, y);
	}

	was_pressed &= ~pressed;

	if (was_pressed)
		dev_vdbg(&client->dev, "fingers %lx lifted\n", was_pressed);

	for_each_set_bit(i, &was_pressed, MAX_FINGERS) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev,
					   MT_TOOL_FINGER, false);
	}

	input_sync(info->input_dev);
	info->pressed = pressed;

out:
	return IRQ_HANDLED;
}

static int __devinit mms_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	ret = i2c_smbus_read_byte_data(client, MMS_HEADER);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read header: %d", ret);
		return -ENODEV;
	} else if (ret != 3) {
		dev_err(&client->dev, "unrecognized header: %d\n", ret);
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!info || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		goto err_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;

	if (client->dev.platform_data) {
		struct mms_ts_platform_data *pdata = client->dev.platform_data;
		info->max_x = pdata->max_x;
		info->max_y = pdata->max_y;
		info->invert_x = pdata->invert_x;
		info->invert_y = pdata->invert_y;
	} else {
		info->max_x = 480;
		info->max_y = 854;
	}

	input_mt_init_slots(input_dev, MAX_FINGERS);

	snprintf(info->phys, sizeof(info->phys),
		 "%s/input0", dev_name(&client->dev));
	input_dev->name = "Melfas MMSxxx Touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, info->max_y, 0, 0);

	input_set_drvdata(input_dev, info);

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "mms_ts", info);

	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_req_irq;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev (%d)\n",
			ret);
		goto err_reg_input_dev;
	}

	i2c_set_clientdata(client, info);

	dev_info(&client->dev,
		 "Melfas MMS-series touch controller initialized\n");
	return 0;

err_reg_input_dev:
	free_irq(client->irq, info);
err_req_irq:
err_alloc:
	input_free_device(input_dev);
	kfree(info);
	return ret;
}

static int __devexit mms_ts_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	free_irq(client->irq, info);
	input_unregister_device(info->input_dev);
	kfree(info);

	return 0;
}

static const struct i2c_device_id mms_ts_id[] = {
	{ "mms_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mms_ts_id);

static struct i2c_driver mms_ts_driver = {
	.probe		= mms_ts_probe,
	.remove		= __devexit_p(mms_ts_remove),
	.driver = {
		.name = "mms_ts",
	},
	.id_table	= mms_ts_id,
};

static int __init mms_ts_init(void)
{
	return i2c_add_driver(&mms_ts_driver);
}

static void __exit mms_ts_exit(void)
{
	i2c_del_driver(&mms_ts_driver);
}

module_init(mms_ts_init);
module_exit(mms_ts_exit);

/* Module information */
MODULE_DESCRIPTION("Touchscreen driver for Melfas MMS-series controllers");
MODULE_LICENSE("GPL");
