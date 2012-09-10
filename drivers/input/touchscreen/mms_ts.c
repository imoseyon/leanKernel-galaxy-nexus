/*
 * mms_ts.c - Touchscreen driver for Melfas MMS-series touch controllers
 *
 * Copyright (C) 2011 Google Inc.
 * Author: Dima Zavin <dima@android.com>
 *         Simon Wilson <simonwilson@google.com>
 *
 * ISP reflashing code based on original code from Melfas.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

//#define DEBUG
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
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

#include <asm/unaligned.h>

#define MAX_FINGERS		10
#define MAX_WIDTH		30
#define MAX_PRESSURE		255

/* Registers */
#define MMS_MODE_CONTROL	0x01
#define MMS_XYRES_HI		0x02
#define MMS_XRES_LO		0x03
#define MMS_YRES_LO		0x04

#define MMS_INPUT_EVENT_PKT_SZ	0x0F
#define MMS_INPUT_EVENT0	0x10
#define 	FINGER_EVENT_SZ	6

#define MMS_TSP_REVISION	0xF0
#define MMS_HW_REVISION		0xF1
#define MMS_COMPAT_GROUP	0xF2
#define MMS_FW_VERSION		0xF3

enum {
	ISP_MODE_FLASH_ERASE	= 0x59F3,
	ISP_MODE_FLASH_WRITE	= 0x62CD,
	ISP_MODE_FLASH_READ	= 0x6AC9,
};

/* each address addresses 4-byte words */
#define ISP_MAX_FW_SIZE		(0x1F00 * 4)
#define ISP_IC_INFO_ADDR	0x1F00

static bool mms_force_reflash = false;
module_param_named(force_reflash, mms_force_reflash, bool, S_IWUSR | S_IRUGO);

static bool mms_flash_from_probe;
module_param_named(flash_from_probe, mms_flash_from_probe, bool,
		   S_IWUSR | S_IRUGO);

static bool mms_die_on_flash_fail = true;
module_param_named(die_on_flash_fail, mms_die_on_flash_fail, bool,
		   S_IWUSR | S_IRUGO);

struct mms_ts_info {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	char				phys[32];

	int				max_x;
	int				max_y;

	bool				invert_x;
	bool				invert_y;

	int				irq;

	struct mms_ts_platform_data	*pdata;

	char				*fw_name;
	struct completion		init_done;
	struct early_suspend		early_suspend;

	/* protects the enabled flag */
	struct mutex			lock;
	bool				enabled;
};

struct mms_fw_image {
    __le32 hdr_len;
    __le32 data_len;
    __le32 fw_ver;
    __le32 hdr_ver;
    u8 data[0];
} __attribute__ ((packed));

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_ts_early_suspend(struct early_suspend *h);
static void mms_ts_late_resume(struct early_suspend *h);
#endif

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGERS*FINGER_EVENT_SZ] = { 0 };
	int ret;
	int i;
	int sz;
	u8 reg = MMS_INPUT_EVENT0;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.buf    = &reg,
			.len    = 1,
		}, {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.buf    = buf,
		},
	};

	sz = i2c_smbus_read_byte_data(client, MMS_INPUT_EVENT_PKT_SZ);
	if (sz < 0) {
		dev_err(&client->dev, "%s bytes=%d\n", __func__, sz);
		goto out;
	}
	dev_dbg(&client->dev, "bytes available: %d\n", sz);
	BUG_ON(sz > MAX_FINGERS*FINGER_EVENT_SZ);
	if (sz == 0)
		goto out;

	msg[1].len = sz;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev,
			"failed to read %d bytes of touch data (%d)\n",
			sz, ret);
		goto out;
	}

#if defined(VERBOSE_DEBUG)
	print_hex_dump(KERN_DEBUG, "mms_ts raw: ",
		       DUMP_PREFIX_OFFSET, 32, 1, buf, sz, false);
#endif
	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		u8 *tmp = &buf[i];
		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);

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

static void hw_reboot(struct mms_ts_info *info, bool bootloader)
{
	gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	gpio_direction_output(info->pdata->gpio_sda, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata->gpio_scl, bootloader ? 0 : 1);
	gpio_direction_output(info->pdata->gpio_resetb, 0);
	msleep(30);
	gpio_set_value(info->pdata->gpio_vdd_en, 1);
	msleep(30);

	if (bootloader) {
		gpio_set_value(info->pdata->gpio_scl, 0);
		gpio_set_value(info->pdata->gpio_sda, 1);
	} else {
		gpio_set_value(info->pdata->gpio_resetb, 1);
		gpio_direction_input(info->pdata->gpio_resetb);
		gpio_direction_input(info->pdata->gpio_scl);
		gpio_direction_input(info->pdata->gpio_sda);
	}
	msleep(40);
}

static inline void hw_reboot_bootloader(struct mms_ts_info *info)
{
	hw_reboot(info, true);
}

static inline void hw_reboot_normal(struct mms_ts_info *info)
{
	hw_reboot(info, false);
}

static inline void mms_pwr_on_reset(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	if (!info->pdata->mux_fw_flash) {
		dev_info(&info->client->dev,
			 "missing platform data, can't do power-on-reset\n");
		return;
	}

	i2c_lock_adapter(adapter);
	info->pdata->mux_fw_flash(true);

	gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	gpio_direction_output(info->pdata->gpio_sda, 1);
	gpio_direction_output(info->pdata->gpio_scl, 1);
	gpio_direction_output(info->pdata->gpio_resetb, 1);
	msleep(50);
	gpio_direction_output(info->pdata->gpio_vdd_en, 1);
	msleep(50);

	info->pdata->mux_fw_flash(false);
	i2c_unlock_adapter(adapter);

	/* TODO: Seems long enough for the firmware to boot.
	 * Find the right value */
	msleep(250);
}

static void isp_toggle_clk(struct mms_ts_info *info, int start_lvl, int end_lvl,
			   int hold_us)
{
	gpio_set_value(info->pdata->gpio_scl, start_lvl);
	udelay(hold_us);
	gpio_set_value(info->pdata->gpio_scl, end_lvl);
	udelay(hold_us);
}

/* 1 <= cnt <= 32 bits to write */
static void isp_send_bits(struct mms_ts_info *info, u32 data, int cnt)
{
	gpio_direction_output(info->pdata->gpio_resetb, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 0);

	/* clock out the bits, msb first */
	while (cnt--) {
		gpio_set_value(info->pdata->gpio_sda, (data >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}
}

/* 1 <= cnt <= 32 bits to read */
static u32 isp_recv_bits(struct mms_ts_info *info, int cnt)
{
	u32 data = 0;

	gpio_direction_output(info->pdata->gpio_resetb, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_set_value(info->pdata->gpio_sda, 0);
	gpio_direction_input(info->pdata->gpio_sda);

	/* clock in the bits, msb first */
	while (cnt--) {
		isp_toggle_clk(info, 0, 1, 1);
		data = (data << 1) | (!!gpio_get_value(info->pdata->gpio_sda));
	}

	gpio_direction_output(info->pdata->gpio_sda, 0);
	return data;
}

static void isp_enter_mode(struct mms_ts_info *info, u32 mode)
{
	int cnt;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata->gpio_resetb, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 1);

	mode &= 0xffff;
	for (cnt = 15; cnt >= 0; cnt--) {
		gpio_set_value(info->pdata->gpio_resetb, (mode >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}

	gpio_set_value(info->pdata->gpio_resetb, 0);
	local_irq_restore(flags);
}

static void isp_exit_mode(struct mms_ts_info *info)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	gpio_direction_output(info->pdata->gpio_resetb, 0);
	udelay(3);

	for (i = 0; i < 10; i++)
		isp_toggle_clk(info, 1, 0, 3);
	local_irq_restore(flags);
}

static void flash_set_address(struct mms_ts_info *info, u16 addr)
{
	/* Only 13 bits of addr are valid.
	 * The addr is in bits 13:1 of cmd */
	isp_send_bits(info, (u32)(addr & 0x1fff) << 1, 18);
}

static void flash_erase(struct mms_ts_info *info)
{
	isp_enter_mode(info, ISP_MODE_FLASH_ERASE);

	gpio_direction_output(info->pdata->gpio_resetb, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 1);

	/* 4 clock cycles with different timings for the erase to
	 * get processed, clk is already 0 from above */
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(25000, 35000);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(150, 200);
	isp_toggle_clk(info, 1, 0, 3);

	gpio_set_value(info->pdata->gpio_sda, 0);

	isp_exit_mode(info);
}

static u32 flash_readl(struct mms_ts_info *info, u16 addr)
{
	int i;
	u32 val;
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_READ);
	flash_set_address(info, addr);

	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_sda, 0);
	udelay(40);

	/* data load cycle */
	for (i = 0; i < 6; i++)
		isp_toggle_clk(info, 1, 0, 10);

	val = isp_recv_bits(info, 32);
	isp_exit_mode(info);
	local_irq_restore(flags);

	return val;
}

static void flash_writel(struct mms_ts_info *info, u16 addr, u32 val)
{
	unsigned long flags;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_WRITE);
	flash_set_address(info, addr);
	isp_send_bits(info, val, 32);

	gpio_direction_output(info->pdata->gpio_sda, 1);
	/* 6 clock cycles with different timings for the data to get written
	 * into flash */
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 6);
	isp_toggle_clk(info, 0, 1, 12);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);

	isp_toggle_clk(info, 1, 0, 1);

	gpio_direction_output(info->pdata->gpio_sda, 0);
	isp_exit_mode(info);
	local_irq_restore(flags);
	usleep_range(300, 400);
}

static bool flash_is_erased(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 val;
	u16 addr;

	for (addr = 0; addr < (ISP_MAX_FW_SIZE / 4); addr++) {
		udelay(40);
		val = flash_readl(info, addr);

		if (val != 0xffffffff) {
			dev_dbg(&client->dev,
				"addr 0x%x not erased: 0x%08x != 0xffffffff\n",
				addr, val);
			return false;
		}
	}
	return true;
}

static int fw_write_image(struct mms_ts_info *info, const u8 *data, size_t len)
{
	struct i2c_client *client = info->client;
	u16 addr = 0;

	for (addr = 0; addr < (len / 4); addr++, data += 4) {
		u32 val = get_unaligned_le32(data);
		u32 verify_val;
		int retries = 3;

		while (retries--) {
			flash_writel(info, addr, val);
			verify_val = flash_readl(info, addr);
			if (val == verify_val)
				break;
			dev_err(&client->dev,
				"mismatch @ addr 0x%x: 0x%x != 0x%x\n",
				addr, verify_val, val);
			hw_reboot_bootloader(info);
			continue;
		}
		if (retries < 0)
			return -ENXIO;
	}

	return 0;
}

static int fw_download(struct mms_ts_info *info, const u8 *data, size_t len)
{
	struct i2c_client *client = info->client;
	u32 val;
	int ret = 0;

	if (len % 4) {
		dev_err(&client->dev,
			"fw image size (%d) must be a multiple of 4 bytes\n",
			len);
		return -EINVAL;
	} else if (len > ISP_MAX_FW_SIZE) {
		dev_err(&client->dev,
			"fw image is too big, %d > %d\n", len, ISP_MAX_FW_SIZE);
		return -EINVAL;
	}

	dev_info(&client->dev, "fw download start\n");

	gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	gpio_direction_output(info->pdata->gpio_sda, 0);
	gpio_direction_output(info->pdata->gpio_scl, 0);
	gpio_direction_output(info->pdata->gpio_resetb, 0);

	hw_reboot_bootloader(info);

	val = flash_readl(info, ISP_IC_INFO_ADDR);
	dev_info(&client->dev, "IC info: 0x%02x (%x)\n", val & 0xff, val);

	dev_info(&client->dev, "fw erase...\n");
	flash_erase(info);
	if (!flash_is_erased(info)) {
		ret = -ENXIO;
		goto err;
	}

	dev_info(&client->dev, "fw write...\n");
	/* XXX: what does this do?! */
	flash_writel(info, ISP_IC_INFO_ADDR, 0xffffff00 | (val & 0xff));
	usleep_range(1000, 1500);
	ret = fw_write_image(info, data, len);
	if (ret)
		goto err;
	usleep_range(1000, 1500);

	hw_reboot_normal(info);
	usleep_range(1000, 1500);
	dev_info(&client->dev, "fw download done...\n");
	return 0;

err:
	dev_err(&client->dev, "fw download failed...\n");
	hw_reboot_normal(info);
	return ret;
}

static int get_fw_version(struct mms_ts_info *info)
{
	int ret;
	int retries = 3;

	/* this seems to fail sometimes after a reset.. retry a few times */
	do {
		ret = i2c_smbus_read_byte_data(info->client, MMS_FW_VERSION);
	} while (ret < 0 && retries-- > 0);

	return ret;
}

static int mms_ts_enable(struct mms_ts_info *info)
{
	mutex_lock(&info->lock);
	if (info->enabled)
		goto out;
	/* wake up the touch controller. */
	i2c_smbus_write_byte_data(info->client, 0, 0);
	usleep_range(3000, 5000);
	info->enabled = true;
	enable_irq(info->irq);
out:
	mutex_unlock(&info->lock);
	return 0;
}

static int mms_ts_disable(struct mms_ts_info *info)
{
	mutex_lock(&info->lock);
	if (!info->enabled)
		goto out;
	disable_irq(info->irq);
	i2c_smbus_write_byte_data(info->client, MMS_MODE_CONTROL, 0);
	usleep_range(10000, 12000);
	info->enabled = false;
out:
	mutex_unlock(&info->lock);
	return 0;
}

static int mms_ts_input_open(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);
	int ret;

	ret = wait_for_completion_interruptible_timeout(&info->init_done,
			msecs_to_jiffies(90 * MSEC_PER_SEC));

	if (ret > 0) {
		if (info->irq != -1)
			ret = mms_ts_enable(info);
		else
			ret = -ENXIO;
	} else if (ret < 0) {
		dev_err(&dev->dev,
			"error while waiting for device to init (%d)\n", ret);
		ret = -ENXIO;
	} else if (ret == 0) {
		dev_err(&dev->dev,
			"timedout while waiting for device to init\n");
		ret = -ENXIO;
	}

	return ret;
}

static void mms_ts_input_close(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);
	mms_ts_disable(info);
}

static int mms_ts_finish_config(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret;

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "mms_ts", info);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_req_irq;
	}
	disable_irq(client->irq);

	info->irq = client->irq;
	barrier();

	dev_info(&client->dev,
		 "Melfas MMS-series touch controller initialized\n");

	complete_all(&info->init_done);
	return 0;

err_req_irq:
	return ret;
}

static void mms_ts_fw_load(const struct firmware *fw, void *context)
{
	struct mms_ts_info *info = context;
	struct i2c_client *client = info->client;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	int ver;
	int retries = 3;
	struct mms_fw_image *fw_img;

	ver = get_fw_version(info);
	if (ver < 0) {
		ver = 0;
		dev_err(&client->dev,
			"can't read version, controller dead? forcing reflash");
	}

	if (!fw) {
		dev_info(&client->dev, "could not find firmware file '%s'\n",
			info->fw_name);
		goto done;
	}

	fw_img = (struct mms_fw_image *)fw->data;
	if (fw_img->hdr_len != sizeof(struct mms_fw_image) ||
		   fw_img->data_len + fw_img->hdr_len != fw->size ||
		   fw_img->hdr_ver != 0x1) {
		dev_err(&client->dev,
			"firmware image '%s' invalid, may continue\n",
			info->fw_name);
		goto err;
	}

	if (ver == fw_img->fw_ver && !mms_force_reflash) {
		dev_info(&client->dev,
			 "fw version 0x%02x already present\n", ver);
		goto done;
	}

	dev_info(&client->dev, "need fw update (0x%02x != 0x%02x)\n",
		 ver, fw_img->fw_ver);

	if (!info->pdata || !info->pdata->mux_fw_flash) {
		dev_err(&client->dev,
			"fw cannot be updated, missing platform data\n");
		goto err;
	}

	while (retries--) {
		i2c_lock_adapter(adapter);
		info->pdata->mux_fw_flash(true);

		ret = fw_download(info, fw_img->data, fw_img->data_len);

		info->pdata->mux_fw_flash(false);
		i2c_unlock_adapter(adapter);

		if (ret < 0) {
			dev_err(&client->dev,
				"error updating firmware to version 0x%02x\n",
				fw_img->fw_ver);
			if (retries)
				dev_err(&client->dev, "retrying flashing\n");
			continue;
		}

		ver = get_fw_version(info);
		if (ver == fw_img->fw_ver) {
			dev_info(&client->dev,
				 "fw update done. ver = 0x%02x\n", ver);
			goto done;
		} else {
			dev_err(&client->dev,
				"ERROR: fw update succeeded, but fw version is still wrong (0x%x != 0x%x)\n",
				ver, fw_img->fw_ver);
		}
		if (retries)
			dev_err(&client->dev, "retrying flashing\n");
	}

	dev_err(&client->dev, "could not flash firmware, ran out of retries\n");
	BUG_ON(mms_die_on_flash_fail);

err:
	/* complete anyway, so open() doesn't get blocked */
	complete_all(&info->init_done);
	goto out;

done:
	mms_ts_finish_config(info);
out:
	release_firmware(fw);
}

static int __devinit mms_ts_config(struct mms_ts_info *info, bool nowait)
{
	struct i2c_client *client = info->client;
	int ret = 0;
	const char *filename = info->pdata->fw_name ?: "mms144_ts.fw";

	mms_pwr_on_reset(info);

	if (nowait) {
		const struct firmware *fw;
		info->fw_name = kasprintf(GFP_KERNEL, "melfas/%s", filename);
		ret = request_firmware(&fw, info->fw_name, &client->dev);
		if (ret) {
			dev_err(&client->dev,
				"error requesting built-in firmware\n");
			goto out;
		}
		mms_ts_fw_load(fw, info);
	} else {
		info->fw_name = kstrdup(filename, GFP_KERNEL);
		ret = request_firmware_nowait(THIS_MODULE, true, info->fw_name,
					      &client->dev, GFP_KERNEL,
					      info, mms_ts_fw_load);
		if (ret)
			dev_err(&client->dev,
				"cannot schedule firmware update (%d)\n", ret);
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
	init_completion(&info->init_done);
	info->irq = -1;
	mutex_init(&info->lock);

	if (info->pdata) {
		info->max_x = info->pdata->max_x;
		info->max_y = info->pdata->max_y;
		info->invert_x = info->pdata->invert_x;
		info->invert_y = info->pdata->invert_y;
	} else {
		info->max_x = 720;
		info->max_y = 1280;
	}

	input_mt_init_slots(input_dev, MAX_FINGERS);

	snprintf(info->phys, sizeof(info->phys),
		 "%s/input0", dev_name(&client->dev));
	input_dev->name = "Melfas MMSxxx Touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mms_ts_input_open;
	input_dev->close = mms_ts_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, info->max_x - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, info->max_y - 1, 0, 0);

	input_set_drvdata(input_dev, info);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev (%d)\n",
			ret);
		goto err_reg_input_dev;
	}

	i2c_set_clientdata(client, info);

	ret = mms_ts_config(info, mms_flash_from_probe);
	if (ret) {
		dev_err(&client->dev, "failed to initialize (%d)\n", ret);
		goto err_config;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = mms_ts_early_suspend;
	info->early_suspend.resume = mms_ts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

	return 0;

err_config:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_reg_input_dev:
err_alloc:
	input_free_device(input_dev);
	kfree(info->fw_name);
	kfree(info);
	return ret;
}

static int __devexit mms_ts_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);
	input_unregister_device(info->input_dev);
	kfree(info->fw_name);
	kfree(info);

	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int mms_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int i;

	/* TODO: turn off the power (set vdd_en to 0) to the touchscreen
	 * on suspend
	 */

	mutex_lock(&info->input_dev->mutex);
	if (!info->input_dev->users)
		goto out;

	mms_ts_disable(info);
	for (i = 0; i < MAX_FINGERS; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER,
					   false);
	}
	input_sync(info->input_dev);

out:
	mutex_unlock(&info->input_dev->mutex);
	return 0;
}

static int mms_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&info->input_dev->mutex);
	if (info->input_dev->users)
		ret = mms_ts_enable(info);
	mutex_unlock(&info->input_dev->mutex);

	return ret;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_ts_early_suspend(struct early_suspend *h)
{
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	mms_ts_suspend(&info->client->dev);
}

static void mms_ts_late_resume(struct early_suspend *h)
{
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	mms_ts_resume(&info->client->dev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops mms_ts_pm_ops = {
	.suspend	= mms_ts_suspend,
	.resume		= mms_ts_resume,
};
#endif

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
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &mms_ts_pm_ops,
#endif
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
