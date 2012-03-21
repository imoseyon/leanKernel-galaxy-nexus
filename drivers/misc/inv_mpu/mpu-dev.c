/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/poll.h>

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "mpuirq.h"
#include "slaveirq.h"
#include "mlsl.h"
#include "mldl_cfg.h"
#include <linux/mpu.h>


/* Platform data for the MPU */
struct mpu_private_data {
	struct miscdevice dev;
	struct i2c_client *client;
	struct mldl_cfg mldl_cfg;

	struct mutex mutex;
	wait_queue_head_t mpu_event_wait;
	struct completion completion;
	struct timer_list timeout;
	struct notifier_block nb;
	struct mpuirq_data mpu_pm_event;
	int response_timeout;	/* In seconds */
	unsigned long event;
	int pid;
	struct module *slave_modules[EXT_SLAVE_NUM_TYPES];
};

struct mpu_private_data *mpu_private_data;

static void mpu_pm_timeout(u_long data)
{
	struct mpu_private_data *mpu = (struct mpu_private_data *)data;
	struct i2c_client *client = mpu->client;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	complete(&mpu->completion);
}

static int mpu_pm_notifier_callback(struct notifier_block *nb,
				    unsigned long event, void *unused)
{
	struct mpu_private_data *mpu =
	    container_of(nb, struct mpu_private_data, nb);
	struct i2c_client *client = mpu->client;
	dev_dbg(&client->adapter->dev, "%s: %ld\n", __func__, event);

	/* Prevent the file handle from being closed before we initialize
	   the completion event */
	mutex_lock(&mpu->mutex);
	if (!(mpu->pid) ||
	    (event != PM_SUSPEND_PREPARE && event != PM_POST_SUSPEND)) {
		mutex_unlock(&mpu->mutex);
		return NOTIFY_OK;
	}

	if (event == PM_SUSPEND_PREPARE)
		mpu->event = MPU_PM_EVENT_SUSPEND_PREPARE;
	if (event == PM_POST_SUSPEND)
		mpu->event = MPU_PM_EVENT_POST_SUSPEND;

	mpu->mpu_pm_event.irqtime = ktime_to_ns(ktime_get());
	mpu->mpu_pm_event.interruptcount++;
	mpu->mpu_pm_event.data_type = MPUIRQ_DATA_TYPE_PM_EVENT;
	mpu->mpu_pm_event.data = mpu->event;

	if (mpu->response_timeout > 0) {
		mpu->timeout.expires = jiffies + mpu->response_timeout * HZ;
		add_timer(&mpu->timeout);
	}
	INIT_COMPLETION(mpu->completion);
	mutex_unlock(&mpu->mutex);

	wake_up_interruptible(&mpu->mpu_event_wait);
	wait_for_completion(&mpu->completion);
	del_timer_sync(&mpu->timeout);
	dev_dbg(&client->adapter->dev, "%s: %ld DONE\n", __func__, event);
	return NOTIFY_OK;
}

static int mpu_dev_open(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int result;
	int ii;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	dev_dbg(&client->adapter->dev, "current->pid %d\n", current->pid);

	result = mutex_lock_interruptible(&mpu->mutex);
	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}
	mpu->pid = current->pid;

	/* Reset the sensors to the default */
	if (result) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, result);
		return result;
	}

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		__module_get(mpu->slave_modules[ii]);

	mldl_cfg->requested_sensors = INV_THREE_AXIS_GYRO;
	if (mldl_cfg->accel && mldl_cfg->accel->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_ACCEL;

	if (mldl_cfg->compass && mldl_cfg->compass->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_COMPASS;

	if (mldl_cfg->pressure && mldl_cfg->pressure->resume)
		mldl_cfg->requested_sensors |= INV_THREE_AXIS_PRESSURE;
	mutex_unlock(&mpu->mutex);
	return 0;
}

/* close function - called when the "file" /dev/mpu is closed in userspace   */
static int mpu_release(struct inode *inode, struct file *file)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;
	int result = 0;
	int ii;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	mldl_cfg->requested_sensors = 0;
	result = inv_mpu_suspend(mldl_cfg, client->adapter,
				 accel_adapter, compass_adapter,
				 pressure_adapter, INV_ALL_SENSORS);
	mpu->pid = 0;
	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		module_put(mpu->slave_modules[ii]);

	mutex_unlock(&mpu->mutex);
	complete(&mpu->completion);
	dev_dbg(&client->adapter->dev, "mpu_release\n");

	return result;
}

/* read function called when from /dev/mpu is read.  Read from the FIFO */
static ssize_t mpu_read(struct file *file,
			char __user *buf, size_t count, loff_t *offset)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	size_t len = sizeof(mpu->mpu_pm_event) + sizeof(unsigned long);
	int err;

	if (!mpu->event && (!(file->f_flags & O_NONBLOCK)))
		wait_event_interruptible(mpu->mpu_event_wait, mpu->event);

	if (!mpu->event || !buf
	    || count < sizeof(mpu->mpu_pm_event))
		return 0;

	err = copy_to_user(buf, &mpu->mpu_pm_event, sizeof(mpu->mpu_pm_event));
	if (err) {
		dev_err(&client->adapter->dev,
			"Copy to user returned %d\n", err);
		return -EFAULT;
	}
	mpu->event = 0;
	return len;
}

static unsigned int mpu_poll(struct file *file, struct poll_table_struct *poll)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	int mask = 0;

	poll_wait(file, &mpu->mpu_event_wait, poll);
	if (mpu->event)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int
mpu_dev_ioctl_set_mpu_pdata(struct i2c_client *client, unsigned long arg)
{
	int ii;
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mpu_platform_data *pdata = mpu->mldl_cfg.pdata;
	struct mpu_platform_data local_pdata;

	if (copy_from_user(&local_pdata, (unsigned char __user *)arg,
			   sizeof(local_pdata)))
		return -EFAULT;

	pdata->int_config = local_pdata.int_config;
	for (ii = 0; ii < ARRAY_SIZE(pdata->orientation); ii++)
		pdata->orientation[ii] = local_pdata.orientation[ii];
	pdata->level_shifter = local_pdata.level_shifter;

	pdata->accel.address = local_pdata.accel.address;
	for (ii = 0; ii < ARRAY_SIZE(pdata->accel.orientation); ii++)
		pdata->accel.orientation[ii] =
		    local_pdata.accel.orientation[ii];

	pdata->compass.address = local_pdata.compass.address;
	for (ii = 0; ii < ARRAY_SIZE(pdata->compass.orientation); ii++)
		pdata->compass.orientation[ii] =
		    local_pdata.compass.orientation[ii];

	pdata->pressure.address = local_pdata.pressure.address;
	for (ii = 0; ii < ARRAY_SIZE(pdata->pressure.orientation); ii++)
		pdata->pressure.orientation[ii] =
		    local_pdata.pressure.orientation[ii];

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	return 0;
}

static int
mpu_dev_ioctl_set_mpu_config(struct i2c_client *client, unsigned long arg)
{
	int ii;
	int result = 0;
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mldl_cfg *temp_mldl_cfg;

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	temp_mldl_cfg = kmalloc(offsetof(struct mldl_cfg, silicon_revision),
				GFP_KERNEL);
	if (!temp_mldl_cfg)
		return -ENOMEM;

	/*
	 * User space is not allowed to modify accel compass pressure or
	 * pdata structs, as well as silicon_revision product_id or trim
	 */
	if (copy_from_user(temp_mldl_cfg, (struct mldl_cfg __user *)arg,
			   offsetof(struct mldl_cfg, silicon_revision))) {
		result = -EFAULT;
		goto out;
	}

	if (mldl_cfg->gyro_is_suspended) {
		if (mldl_cfg->addr != temp_mldl_cfg->addr)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->int_config != temp_mldl_cfg->int_config)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->ext_sync != temp_mldl_cfg->ext_sync)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->full_scale != temp_mldl_cfg->full_scale)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->lpf != temp_mldl_cfg->lpf)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->clk_src != temp_mldl_cfg->clk_src)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->divider != temp_mldl_cfg->divider)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_enable != temp_mldl_cfg->dmp_enable)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->fifo_enable != temp_mldl_cfg->fifo_enable)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_cfg1 != temp_mldl_cfg->dmp_cfg1)
			mldl_cfg->gyro_needs_reset = TRUE;

		if (mldl_cfg->dmp_cfg2 != temp_mldl_cfg->dmp_cfg2)
			mldl_cfg->gyro_needs_reset = TRUE;

		for (ii = 0; ii < GYRO_NUM_AXES; ii++)
			if (mldl_cfg->offset_tc[ii] !=
			    temp_mldl_cfg->offset_tc[ii])
				mldl_cfg->gyro_needs_reset = TRUE;

		for (ii = 0; ii < GYRO_NUM_AXES; ii++)
			if (mldl_cfg->offset[ii] != temp_mldl_cfg->offset[ii])
				mldl_cfg->gyro_needs_reset = TRUE;

		if (memcmp(mldl_cfg->ram, temp_mldl_cfg->ram,
			   MPU_MEM_NUM_RAM_BANKS * MPU_MEM_BANK_SIZE *
			   sizeof(unsigned char)))
			mldl_cfg->gyro_needs_reset = TRUE;
	}

	memcpy(mldl_cfg, temp_mldl_cfg,
	       offsetof(struct mldl_cfg, silicon_revision));

 out:
	kfree(temp_mldl_cfg);
	return result;
}

static int
mpu_dev_ioctl_get_mpu_config(struct i2c_client *client,
			     struct mldl_cfg __user *arg)
{
	/* Have to be careful as there are 3 pointers in the mldl_cfg
	 * structure */
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mldl_cfg *local_mldl_cfg;
	int retval = 0;

	local_mldl_cfg = kmalloc(sizeof(struct mldl_cfg), GFP_KERNEL);
	if (!local_mldl_cfg)
		return -ENOMEM;

	retval =
	    copy_from_user(local_mldl_cfg, arg, sizeof(*arg));
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s|%s:%d: EFAULT on arg\n",
			__FILE__, __func__, __LINE__);
		retval = -EFAULT;
		goto out;
	}

	/* Fill in the accel, compass, pressure and pdata pointers */
	if (mldl_cfg->accel) {
		retval = copy_to_user((void __user *)local_mldl_cfg->accel,
				      mldl_cfg->accel,
				      sizeof(*mldl_cfg->accel));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on accel\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->compass) {
		retval = copy_to_user((void __user *)local_mldl_cfg->compass,
				      mldl_cfg->compass,
				      sizeof(*mldl_cfg->compass));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on compass\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->pressure) {
		retval = copy_to_user((void __user *)local_mldl_cfg->pressure,
				      mldl_cfg->pressure,
				      sizeof(*mldl_cfg->pressure));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on pressure\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	if (mldl_cfg->pdata) {
		retval = copy_to_user((void __user *)local_mldl_cfg->pdata,
				      mldl_cfg->pdata,
				      sizeof(*mldl_cfg->pdata));
		if (retval) {
			dev_err(&client->adapter->dev,
				"%s|%s:%d: EFAULT on pdata\n",
				__FILE__, __func__, __LINE__);
			retval = -EFAULT;
			goto out;
		}
	}

	/* Do not modify the accel, compass, pressure and pdata pointers */
	retval = copy_to_user(arg, mldl_cfg, offsetof(struct mldl_cfg, accel));

	if (retval)
		retval = -EFAULT;
 out:
	kfree(local_mldl_cfg);
	return retval;
}

/**
 * slave_config() - Pass a requested slave configuration to the slave sensor
 *
 * @adapter the adaptor to use to communicate with the slave
 * @mldl_cfg the mldl configuration structuer
 * @slave pointer to the slave descriptor
 * @usr_config The configuration to pass to the slave sensor
 *
 * returns 0 or non-zero error code
 */
static int slave_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			void *slave_adapter,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	if ((!slave) || (!slave->config))
		return -ENODEV;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_slave_config(mldl_cfg, gyro_adapter, slave_adapter,
				      &config, slave, pdata);
	kfree(config.data);
	return retval;
}

/**
 * slave_get_config() - Get requested slave configuration from the slave sensor
 *
 * @adapter the adaptor to use to communicate with the slave
 * @mldl_cfg the mldl configuration structuer
 * @slave pointer to the slave descriptor
 * @usr_config The configuration for the slave to fill out
 *
 * returns 0 or non-zero error code
 */
static int slave_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    void *slave_adapter,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata,
			    struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	void *user_data;
	if (!(slave) || !(slave->get_config))
		return -ENODEV;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_get_slave_config(mldl_cfg, gyro_adapter,
					  slave_adapter, &config, slave, pdata);
	if (retval) {
		kfree(config.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      config.data, config.len);
	kfree(config.data);
	return retval;
}

static int inv_slave_read(struct mldl_cfg *mldl_cfg,
			  void *gyro_adapter,
			  void *slave_adapter,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata,
			  void __user *usr_data)
{
	int retval;
	unsigned char *data;
	data = kzalloc(slave->read_len, GFP_KERNEL);
	if (!data)
		return -EFAULT;

	retval = inv_mpu_slave_read(mldl_cfg, gyro_adapter, slave_adapter,
				    slave, pdata, data);

	if ((!retval) &&
	    (copy_to_user((unsigned char __user *)usr_data,
			  data, slave->read_len)))
		retval = -EFAULT;

	kfree(data);
	return retval;
}

static int mpu_handle_mlsl(void *sl_handle,
			   unsigned char addr,
			   unsigned int cmd,
			   struct mpu_read_write __user *usr_msg)
{
	int retval = 0;
	struct mpu_read_write msg;
	unsigned char *user_data;
	retval = copy_from_user(&msg, usr_msg, sizeof(msg));
	if (retval)
		return -EFAULT;

	user_data = msg.data;
	if (msg.length && msg.data) {
		unsigned char *data;
		data = kmalloc(msg.length, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)msg.data, msg.length);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		msg.data = data;
	} else {
		return -EPERM;
	}

	switch (cmd) {
	case MPU_READ:
		retval = inv_serial_read(sl_handle, addr,
					 msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE:
		retval = inv_serial_write(sl_handle, addr,
					  msg.length, msg.data);
		break;
	case MPU_READ_MEM:
		retval = inv_serial_read_mem(sl_handle, addr,
					     msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE_MEM:
		retval = inv_serial_write_mem(sl_handle, addr,
					      msg.address, msg.length,
					      msg.data);
		break;
	case MPU_READ_FIFO:
		retval = inv_serial_read_fifo(sl_handle, addr,
					      msg.length, msg.data);
		break;
	case MPU_WRITE_FIFO:
		retval = inv_serial_write_fifo(sl_handle, addr,
					       msg.length, msg.data);
		break;

	};
	if (retval) {
		dev_err(&((struct i2c_adapter *)sl_handle)->dev,
			"%s: i2c %d error %d\n",
			__func__, cmd, retval);
		kfree(msg.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      msg.data, msg.length);
	kfree(msg.data);
	return retval;
}

/* ioctl - I/O control */
static long mpu_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct mpu_private_data *mpu =
	    container_of(file->private_data, struct mpu_private_data, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int retval = 0;
	struct i2c_adapter *gyro_adapter;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	gyro_adapter = client->adapter;
	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	retval = mutex_lock_interruptible(&mpu->mutex);
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, retval);
		return retval;
	}

	switch (cmd) {
	case MPU_SET_MPU_CONFIG:
		retval = mpu_dev_ioctl_set_mpu_config(client, arg);
		break;
	case MPU_SET_PLATFORM_DATA:
		retval = mpu_dev_ioctl_set_mpu_pdata(client, arg);
		break;
	case MPU_GET_MPU_CONFIG:
		retval = mpu_dev_ioctl_get_mpu_config(client,
						(struct mldl_cfg __user *)arg);
		break;
	case MPU_READ:
	case MPU_WRITE:
	case MPU_READ_MEM:
	case MPU_WRITE_MEM:
	case MPU_READ_FIFO:
	case MPU_WRITE_FIFO:
		retval = mpu_handle_mlsl(gyro_adapter, mldl_cfg->addr, cmd,
					 (struct mpu_read_write __user *)arg);
		break;
	case MPU_CONFIG_ACCEL:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      accel_adapter,
				      mldl_cfg->accel,
				      &mldl_cfg->pdata->accel,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_COMPASS:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      compass_adapter,
				      mldl_cfg->compass,
				      &mldl_cfg->pdata->compass,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_PRESSURE:
		retval = slave_config(mldl_cfg,
				      gyro_adapter,
				      pressure_adapter,
				      mldl_cfg->pressure,
				      &mldl_cfg->pdata->pressure,
				      (struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_ACCEL:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  accel_adapter,
					  mldl_cfg->accel,
					  &mldl_cfg->pdata->accel,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_GET_CONFIG_COMPASS:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  compass_adapter,
					  mldl_cfg->compass,
					  &mldl_cfg->pdata->compass,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_GET_CONFIG_PRESSURE:
		retval = slave_get_config(mldl_cfg,
					  gyro_adapter,
					  pressure_adapter,
					  mldl_cfg->pressure,
					  &mldl_cfg->pdata->pressure,
					  (struct ext_slave_config __user *)
					  arg);
		break;
	case MPU_SUSPEND:
		retval = inv_mpu_suspend(mldl_cfg,
					 gyro_adapter,
					 accel_adapter,
					 compass_adapter,
					 pressure_adapter,
					 (~(mldl_cfg->requested_sensors))
					  & INV_ALL_SENSORS);
		break;
	case MPU_RESUME:
		retval = inv_mpu_resume(mldl_cfg,
					gyro_adapter,
					accel_adapter,
					compass_adapter,
					pressure_adapter,
					mldl_cfg->requested_sensors);
		break;
	case MPU_PM_EVENT_HANDLED:
		dev_dbg(&client->adapter->dev, "%s: %d\n", __func__, cmd);
		complete(&mpu->completion);
		break;
	case MPU_READ_ACCEL:
		retval = inv_slave_read(mldl_cfg,
					gyro_adapter,
					accel_adapter,
					mldl_cfg->accel,
					&mldl_cfg->pdata->accel,
					(unsigned char __user *)arg);
		break;
	case MPU_READ_COMPASS:
		retval = inv_slave_read(mldl_cfg,
					gyro_adapter,
					compass_adapter,
					mldl_cfg->compass,
					&mldl_cfg->pdata->compass,
					(unsigned char __user *)arg);
		break;
	case MPU_READ_PRESSURE:
		retval = inv_slave_read(mldl_cfg,
					gyro_adapter,
					pressure_adapter,
					mldl_cfg->pressure,
					&mldl_cfg->pdata->pressure,
					(unsigned char __user *)arg);
		break;
	default:
		dev_err(&client->adapter->dev,
			"%s: Unknown cmd %x, arg %lu: MIN %x MAX %x\n",
			__func__, cmd, arg,
			MPU_SET_MPU_CONFIG, MPU_SET_MPU_CONFIG);
		retval = -EINVAL;
	};

	mutex_unlock(&mpu->mutex);
	return retval;
}

void mpu_shutdown(struct i2c_client *client)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	(void)inv_mpu_suspend(mldl_cfg, client->adapter,
			      accel_adapter, compass_adapter, pressure_adapter,
			      INV_ALL_SENSORS);
	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
}

int mpu_dev_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	if (!mldl_cfg->ignore_system_suspend) {
		dev_dbg(&client->adapter->dev,
			"%s: suspending on event %d\n", __func__, mesg.event);
		(void)inv_mpu_suspend(mldl_cfg, client->adapter,
				      accel_adapter, compass_adapter,
				      pressure_adapter, INV_ALL_SENSORS);
	} else {
		dev_dbg(&client->adapter->dev,
			"%s: Already suspended %d\n", __func__, mesg.event);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

int mpu_dev_resume(struct i2c_client *client)
{
	struct mpu_private_data *mpu =
	    (struct mpu_private_data *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	mutex_lock(&mpu->mutex);
	if (mpu->pid && !mldl_cfg->ignore_system_suspend) {
		(void)inv_mpu_resume(mldl_cfg, client->adapter,
				     accel_adapter,
				     compass_adapter,
				     pressure_adapter,
				     mldl_cfg->requested_sensors);
		dev_dbg(&client->adapter->dev,
			"%s for pid %d\n", __func__, mpu->pid);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

/* define which file operations are supported */
static const struct file_operations mpu_fops = {
	.owner = THIS_MODULE,
	.read = mpu_read,
	.poll = mpu_poll,
	.unlocked_ioctl = mpu_dev_ioctl,
	.open = mpu_dev_open,
	.release = mpu_release,
};

int inv_mpu_register_slave(struct module *slave_module,
			struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data *mpu = mpu_private_data;
	struct mldl_cfg *mldl_cfg;
	struct mpu_platform_data *pdata;
	struct ext_slave_descr *slave_descr;
	int result = 0;

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return -EINVAL;

	if (!mpu) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null mpu_private_data\n", __func__);
		return -EINVAL;
	}
	mldl_cfg = &mpu->mldl_cfg;
	pdata = mldl_cfg->pdata;

	slave_descr = get_slave_descr();
	if (!slave_descr) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null ext_slave_descr\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mpu->mutex);
	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}

	mpu->slave_modules[slave_descr->type] = slave_module;

	switch (slave_descr->type) {
	case EXT_SLAVE_TYPE_ACCELEROMETER:
		if (pdata->accel.get_slave_descr) {
			result = -EBUSY;
			break;
		}

		pdata->accel.address   = slave_client->addr;
		pdata->accel.irq       = slave_client->irq;
		pdata->accel.adapt_num = i2c_adapter_id(slave_client->adapter);

		if (pdata->accel.irq > 0) {
			dev_info(&slave_client->adapter->dev,
				"Installing Accel irq using %d\n",
				pdata->accel.irq);
			result = slaveirq_init(slave_client->adapter,
					&pdata->accel, "accelirq");
			if (result)
				break;
		} else {
			dev_WARN(&slave_client->adapter->dev,
				"Accel irq not assigned\n");
		}

		if (slave_descr->init) {
			result = slave_descr->init(slave_client->adapter,
						slave_descr,
						&pdata->accel);
			if (result) {
				dev_err(&slave_client->adapter->dev,
					"Accel init failed %d\n", result);
				if (pdata->accel.irq > 0)
					slaveirq_exit(&pdata->accel);
				break;
			}
		}

		pdata->accel.get_slave_descr = get_slave_descr;
		mldl_cfg->accel = slave_descr;
		dev_info(&slave_client->adapter->dev,
			"%s: +%s\n", MPU_NAME, mldl_cfg->accel->name);
		break;
	case EXT_SLAVE_TYPE_COMPASS:
		if (pdata->compass.get_slave_descr) {
			result = -EBUSY;
			break;
		}

		pdata->compass.address   = slave_client->addr;
		pdata->compass.irq       = slave_client->irq;
		pdata->compass.adapt_num =
			i2c_adapter_id(slave_client->adapter);
		if (pdata->compass.irq > 0) {
			dev_info(&slave_client->adapter->dev,
				"Installing Compass irq using %d\n",
				pdata->compass.irq);
			result = slaveirq_init(slave_client->adapter,
					&pdata->compass,
					"compassirq");
			if (result)
				break;
		} else {
			dev_warn(&slave_client->adapter->dev,
				"Compass irq not assigned\n");
		}

		if (slave_descr->init) {
			result = slave_descr->init(slave_client->adapter,
						slave_descr,
						&pdata->compass);
			if (result) {
				dev_err(&slave_client->adapter->dev,
					"Compass init failed %d\n", result);
				if (pdata->compass.irq > 0)
					slaveirq_exit(&pdata->compass);
				break;
			}
		}

		pdata->compass.get_slave_descr = get_slave_descr;
		mldl_cfg->compass = pdata->compass.get_slave_descr();
		dev_info(&slave_client->adapter->dev,
			"%s: +%s\n", MPU_NAME,
			mldl_cfg->compass->name);
		break;
	case EXT_SLAVE_TYPE_PRESSURE:
		if (pdata->pressure.get_slave_descr) {
			result = -EBUSY;
			break;
		}

		pdata->pressure.address   = slave_client->addr;
		pdata->pressure.irq       = slave_client->irq;
		pdata->pressure.adapt_num =
			i2c_adapter_id(slave_client->adapter);
		if (pdata->pressure.irq > 0) {
			dev_info(&slave_client->adapter->dev,
				"Installing Pressure irq using %d\n",
				pdata->pressure.irq);
			result = slaveirq_init(slave_client->adapter,
					&pdata->pressure,
					"pressureirq");
			if (result)
				break;
		} else {
			dev_warn(&slave_client->adapter->dev,
				"Pressure irq not assigned\n");
		}

		if (slave_descr->init) {
			result = slave_descr->init(slave_client->adapter,
						slave_descr,
						&pdata->pressure);
			if (result) {
				dev_err(&slave_client->adapter->dev,
					"Pressure init failed %d\n", result);
				if (pdata->pressure.irq > 0)
					slaveirq_exit(&pdata->pressure);
				break;
			}
		}

		pdata->pressure.get_slave_descr = get_slave_descr;
		mldl_cfg->pressure = pdata->pressure.get_slave_descr();
		dev_info(&slave_client->adapter->dev,
			"%s: +%s\n", MPU_NAME,
			mldl_cfg->pressure->name);
		break;
	default:
		dev_err(&slave_client->adapter->dev,
			"Invalid slave type %d\n", slave_descr->type);
		result = -EINVAL;
		break;
	};

	mutex_unlock(&mpu->mutex);
	return result;
}
EXPORT_SYMBOL(inv_mpu_register_slave);

void inv_mpu_unregister_slave(struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data *mpu = mpu_private_data;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mpu_platform_data *pdata;
	struct ext_slave_descr *slave_descr;
	int result;

	dev_info(&slave_client->adapter->dev, "%s\n", __func__);

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return;

	slave_descr = get_slave_descr();
	if (!slave_descr)
		return;

	pdata = mldl_cfg->pdata;
	if (!pdata)
		return;

	mutex_lock(&mpu->mutex);

	if (slave_descr->exit) {
		result = slave_descr->exit(slave_client->adapter,
					slave_descr,
					slave_pdata);
		if (INV_SUCCESS != result)
			MPL_LOGE("Accel exit failed %d\n", result);
	}

	if (slave_pdata->irq)
		slaveirq_exit(slave_pdata);

	switch (slave_descr->type) {
	case EXT_SLAVE_TYPE_ACCELEROMETER:
		mldl_cfg->accel = NULL;
		pdata->accel.get_slave_descr = NULL;
		break;
	case EXT_SLAVE_TYPE_COMPASS:
		mldl_cfg->compass = NULL;
		pdata->compass.get_slave_descr = NULL;
		break;
	case EXT_SLAVE_TYPE_PRESSURE:
		mldl_cfg->pressure = NULL;
		pdata->pressure.get_slave_descr = NULL;
		break;
	default:
		break;
	};
	mpu->slave_modules[slave_descr->type] = NULL;
	mutex_unlock(&mpu->mutex);
}
EXPORT_SYMBOL(inv_mpu_unregister_slave);

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static const struct i2c_device_id mpu_id[] = {
	{"mpu3050", 0},
	{"mpu6050", 0},
	{"mpu6050_no_accel", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpu_id);

int mpu_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct mpu_platform_data *pdata;
	struct mpu_private_data *mpu;
	struct mldl_cfg *mldl_cfg;
	int res = 0;
	struct i2c_adapter *accel_adapter = NULL;
	struct i2c_adapter *compass_adapter = NULL;
	struct i2c_adapter *pressure_adapter = NULL;
	int ii = 0;

	dev_info(&client->adapter->dev, "%s: %d\n", __func__, ii++);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		res = -ENODEV;
		goto out_check_functionality_failed;
	}

	mpu = kzalloc(sizeof(struct mpu_private_data), GFP_KERNEL);
	if (!mpu) {
		res = -ENOMEM;
		goto out_alloc_data_failed;
	}
	mpu_private_data = mpu;
	i2c_set_clientdata(client, mpu);
	mpu->client = client;
	mldl_cfg = &mpu->mldl_cfg;

	init_waitqueue_head(&mpu->mpu_event_wait);

	mutex_init(&mpu->mutex);
	init_completion(&mpu->completion);

	mpu->response_timeout = 60;	/* Seconds */
	mpu->timeout.function = mpu_pm_timeout;
	mpu->timeout.data = (u_long) mpu;
	init_timer(&mpu->timeout);

	mpu->nb.notifier_call = mpu_pm_notifier_callback;
	mpu->nb.priority = 0;
	register_pm_notifier(&mpu->nb);

	pdata = (struct mpu_platform_data *)client->dev.platform_data;
	if (!pdata) {
		dev_WARN(&client->adapter->dev,
			 "Missing platform data for mpu\n");
	}
	mldl_cfg->pdata = pdata;

	mldl_cfg->addr = client->addr;
	res = inv_mpu_open(&mpu->mldl_cfg, client->adapter,
			   accel_adapter, compass_adapter, pressure_adapter);

	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to open %s %d\n", MPU_NAME, res);
		res = -ENODEV;
		goto out_whoami_failed;
	}

	mpu->dev.minor = MISC_DYNAMIC_MINOR;
	mpu->dev.name = "mpu";		/* Same for both 3050 and 6000 */
	mpu->dev.fops = &mpu_fops;
	res = misc_register(&mpu->dev);
	if (res < 0) {
		dev_err(&client->adapter->dev,
			"ERROR: misc_register returned %d\n", res);
		goto out_misc_register_failed;
	}

	if (client->irq) {
		dev_info(&client->adapter->dev,
			 "Installing irq using %d\n", client->irq);
		res = mpuirq_init(client, mldl_cfg);
		if (res)
			goto out_mpuirq_failed;
	} else {
		dev_WARN(&client->adapter->dev,
			 "Missing %s IRQ\n", MPU_NAME);
	}

	return res;

 out_mpuirq_failed:
	misc_deregister(&mpu->dev);
 out_misc_register_failed:
	inv_mpu_close(&mpu->mldl_cfg, client->adapter,
		      accel_adapter, compass_adapter, pressure_adapter);
 out_whoami_failed:
	unregister_pm_notifier(&mpu->nb);
	kfree(mpu);
	mpu_private_data = NULL;
 out_alloc_data_failed:
 out_check_functionality_failed:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, res);
	return res;

}

static int mpu_remove(struct i2c_client *client)
{
	struct mpu_private_data *mpu = i2c_get_clientdata(client);
	struct i2c_adapter *accel_adapter;
	struct i2c_adapter *compass_adapter;
	struct i2c_adapter *pressure_adapter;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct mpu_platform_data *pdata = mldl_cfg->pdata;

	accel_adapter = i2c_get_adapter(mldl_cfg->pdata->accel.adapt_num);
	compass_adapter = i2c_get_adapter(mldl_cfg->pdata->compass.adapt_num);
	pressure_adapter = i2c_get_adapter(mldl_cfg->pdata->pressure.adapt_num);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_close(mldl_cfg, client->adapter,
		      accel_adapter, compass_adapter, pressure_adapter);

	if (client->irq)
		mpuirq_exit();

	if (pdata && pdata->pressure.get_slave_descr && pdata->pressure.irq) {
		slaveirq_exit(&pdata->pressure);
		pdata->pressure.get_slave_descr = NULL;
	}

	if (pdata && pdata->compass.get_slave_descr && pdata->compass.irq) {
		slaveirq_exit(&pdata->compass);
		pdata->compass.get_slave_descr = NULL;
	}

	if (pdata && pdata->accel.get_slave_descr && pdata->accel.irq) {
		slaveirq_exit(&pdata->accel);
		pdata->accel.get_slave_descr = NULL;
	}

	unregister_pm_notifier(&mpu->nb);
	misc_deregister(&mpu->dev);
	kfree(mpu);

	return 0;
}

static struct i2c_driver mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mpu_probe,
	.remove = mpu_remove,
	.id_table = mpu_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MPU_NAME,
		   },
	.address_list = normal_i2c,
	.shutdown = mpu_shutdown,	/* optional */
	.suspend = mpu_dev_suspend,	/* optional */
	.resume = mpu_dev_resume,	/* optional */

};

static int __init mpu_init(void)
{
	int res = i2c_add_driver(&mpu_driver);
	pr_info("%s: Probe name %s\n", __func__, MPU_NAME);
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit mpu_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mpu_driver);
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("User space character device interface for MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS(MPU_NAME);
