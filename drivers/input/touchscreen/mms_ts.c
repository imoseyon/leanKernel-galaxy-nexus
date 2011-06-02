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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/platform_data/mms_ts.h>

#define MAX_FINGERS		10

/* Registers */
#define MMS_MODE_CONTROL	0x01
#define MMS_XYRES_HI		0x02
#define MMS_XRES_LO		0x03
#define MMS_YRES_LO		0x04

#define MMS_INPUT_EVENT_PKT_SZ	0x0F
#define MMS_INPUT_EVENT0	0x10
#define 	FINGER_EVENT_SZ	6

#define MMS_VERSION		0xF0
#define 	MMS_VERSION_SZ	4
#define MMS_TSP_REVISION	0x0
#define MMS_HW_REVISION		0x1
#define MMS_COMPAT_GROUP	0x2
#define MMS_FW_VERSION		0x3


#define REQUIRED_FW_VERSION	0x08

struct mms_ts_info {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	char				phys[32];

	int				max_x;
	int				max_y;

	bool				invert_x;
	bool				invert_y;

	struct mms_ts_platform_data	*pdata;
};

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGERS*FINGER_EVENT_SZ] = { 0 };
	int ret;
	int i;
	int num_fingers;
	int x = 0;
	int y = 0;
	int sz;

	num_fingers = i2c_smbus_read_byte_data(client, MMS_INPUT_EVENT_PKT_SZ);
	if (num_fingers < 0) {
		dev_err(&client->dev, "%s fingers=%d\n", __func__, num_fingers);
		goto out;
	}
	dev_dbg(&client->dev, "fingers: %d\n", num_fingers);
	WARN_ON(num_fingers > MAX_FINGERS);
	if (num_fingers == 0)
		goto out;

	sz = num_fingers * FINGER_EVENT_SZ;
	ret = i2c_smbus_read_i2c_block_data(client, MMS_INPUT_EVENT0, sz, buf);

#if defined(VERBOSE_DEBUG)
	print_hex_dump(KERN_DEBUG, "mms_ts raw: ",
		       DUMP_PREFIX_OFFSET, 32, 1, buf, sz, false);
#endif
	for (i = 0; i < num_fingers; i++) {
		u8 *tmp = &buf[i*6];
		int id = tmp[0] & 0xf;

		x = tmp[2] | ((tmp[1] & 0xf) << 8);
		y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);

		if (info->invert_x) {
			x = info->max_x - x;
			if (x < 0)
				x = 0;
		}
		if (info->invert_y) {
			y = info->max_y - y;
			if (y < 0)
				y = 0;
		}

		if ((tmp[0] & 0x80) == 0) {
			dev_dbg(&client->dev, "finger %d up\n", id);
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, false);
			continue;
		}

		input_mt_slot(info->input_dev, id);
		input_mt_report_slot_state(info->input_dev,
					   MT_TOOL_FINGER, true);
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, tmp[4]);
		input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR, tmp[4]);
		input_report_abs(info->input_dev, ABS_MT_PRESSURE, tmp[5]);
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);

		dev_dbg(&client->dev,
			"finger %d: x=%d y=%d p=%d w=%d\n", id, x, y, tmp[5],
			tmp[4]);
	}

	input_sync(info->input_dev);

out:
	return IRQ_HANDLED;
}

static int get_fw_version(struct mms_ts_info *info)
{
	u8 buf[MMS_VERSION_SZ];
	int ret;

	memset(buf, 0, sizeof(buf));
	ret = i2c_smbus_read_i2c_block_data(info->client, MMS_VERSION,
					    MMS_VERSION_SZ, buf);
	return buf[MMS_FW_VERSION];
}

static int do_flash_fw(struct mms_ts_info *info)
{
	return 0;
}

static int do_update_fw(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);
	int ret = 0;

	if (!info->pdata->mux_fw_flash) {
		dev_err(&info->client->dev,
			"need to update fw, but can't mux pins to gpios\n");
		return -EINVAL;
	}

	i2c_lock_adapter(adapter);
	info->pdata->mux_fw_flash(true);

	ret = do_flash_fw(info);

	info->pdata->mux_fw_flash(false);
	i2c_unlock_adapter(adapter);

	return ret;
}

static int mms_ts_config(struct mms_ts_info *info)
{
	int ret;
	int ver;

	ver = get_fw_version(info);
	if (ver != REQUIRED_FW_VERSION) {
		dev_info(&info->client->dev,
			 "need fw update (0x%02x != 0x%02x)\n",
			 ver, REQUIRED_FW_VERSION);

		ret = do_update_fw(info);
		if (ret < 0) {
			dev_err(&info->client->dev,
				"error updating firmware to version 0x%02x\n",
				ver);
			goto out;
		}

		ver = get_fw_version(info);
		dev_info(&info->client->dev,
			 "fw update done. new ver = 0x%02x\n", ver);
	} else {
		dev_info(&info->client->dev,
			 "fw version 0x%02x already present\n", ver);
	}

out:
	return ret;
}

static int __devinit mms_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!info || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		goto err_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->pdata = client->dev.platform_data;

	ret = mms_ts_config(info);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure the device\n");
		goto err_config;
	}

	if (info->pdata) {
		info->max_x = info->pdata->max_x;
		info->max_y = info->pdata->max_y;
		info->invert_x = info->pdata->invert_x;
		info->invert_y = info->pdata->invert_y;
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
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xff, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, info->max_y, 0, 0);

	input_set_drvdata(input_dev, info);

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "mms_ts", info);
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
err_config:
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
