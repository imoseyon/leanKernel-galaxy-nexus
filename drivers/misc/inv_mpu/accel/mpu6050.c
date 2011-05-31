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

/**
 *  @addtogroup ACCELDL
 *  @brief      Provides the interface to setup and handle an accelerometer.
 *
 *  @{
 *      @file   mpu6050.c
 *      @brief  Accelerometer setup and handling methods for Invensense MPU6050
 */

/* -------------------------------------------------------------------------- */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "mpu-dev.h"

#include <log.h>
#include <linux/mpu.h>
#include "mlsl.h"
#include "mldl_cfg.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

/* -------------------------------------------------------------------------- */

struct mpu6050_config {
	unsigned int odr;	/**< output data rate 1/1000 Hz */
	unsigned int fsr;	/**< full scale range mg */
	unsigned int ths;	/**< mot/no-mot thseshold mg */
	unsigned int dur;	/**< mot/no-mot duration ms */
};

struct mpu6050_private_data {
	struct mpu6050_config suspend;
	struct mpu6050_config resume;
};

/* -------------------------------------------------------------------------- */

/**
 * Record the odr for use in computing duration values.
 *
 * @param config Config to set, suspend or resume structure
 * @param odr output data rate in 1/1000 hz
 */
static int mpu6050_set_odr(void *mlsl_handle,
			  struct ext_slave_platform_data *slave,
			  struct mpu6050_config *config, long apply, long odr)
{
	config->odr = odr;
	return INV_SUCCESS;
}

static int mpu6050_set_ths(void *mlsl_handle,
			  struct ext_slave_platform_data *slave,
			  struct mpu6050_config *config, long apply, long ths)
{
	if (ths < 0)
		ths = 0;

	config->ths = ths;
	MPL_LOGV("THS: %d\n", config->ths);
	return INV_SUCCESS;
}

static int mpu6050_set_dur(void *mlsl_handle,
			  struct ext_slave_platform_data *slave,
			  struct mpu6050_config *config, long apply, long dur)
{
	if (dur < 0)
		dur = 0;

	config->dur = dur;
	MPL_LOGV("DUR: %d\n", config->dur);
	return INV_SUCCESS;
}

static int mpu6050_set_fsr(void *mlsl_handle,
			  struct ext_slave_platform_data *slave,
			  struct mpu6050_config *config, long apply, long fsr)
{
	if (fsr <= 2000)
		config->fsr = 2000;
	else if (fsr <= 4000)
		config->fsr = 4000;
	else
		config->fsr = 8000;

	MPL_LOGV("FSR: %d\n", config->fsr);
	return INV_SUCCESS;
}

static int mpu6050_init(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata)
{
	struct mpu6050_private_data *private_data;

	(void *)private_data;
	return INV_ERROR_INVALID_MODULE;

	private_data = (struct mpu6050_private_data *)
	    kzalloc(sizeof(struct mpu6050_private_data), GFP_KERNEL);

	if (!private_data)
		return INV_ERROR_MEMORY_EXAUSTED;

	pdata->private_data = private_data;

	mpu6050_set_odr(mlsl_handle, pdata, &private_data->suspend, FALSE, 0);
	mpu6050_set_odr(mlsl_handle, pdata,
		       &private_data->resume, FALSE, 200000);
	mpu6050_set_fsr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 2000);
	mpu6050_set_fsr(mlsl_handle, pdata, &private_data->resume, FALSE, 2000);
	mpu6050_set_ths(mlsl_handle, pdata, &private_data->suspend, FALSE, 80);
	mpu6050_set_ths(mlsl_handle, pdata, &private_data->resume, FALSE, 40);
	mpu6050_set_dur(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 1000);
	mpu6050_set_dur(mlsl_handle, pdata, &private_data->resume, FALSE, 2540);
	return INV_SUCCESS;
}

static int mpu6050_exit(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata)
{
	kfree(pdata->private_data);
	return INV_SUCCESS;
}

static int mpu6050_suspend(void *mlsl_handle,
			   struct ext_slave_descr *slave,
			   struct ext_slave_platform_data *pdata)
{
	unsigned char reg;
	int result;

	result = inv_serial_read(mlsl_handle, pdata->address,
				 MPUREG_PWR_MGMT_2, 1, &reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	reg |= (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_PWR_MGMT_2, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	return INV_SUCCESS;
}

static int mpu6050_resume(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	int result = INV_SUCCESS;
	unsigned char reg;
	struct mpu6050_private_data *private_data;

	private_data = (struct mpu6050_private_data *)pdata->private_data;

	result = inv_serial_read(mlsl_handle, pdata->address,
				 MPUREG_PWR_MGMT_1, 1, &reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	if ((reg & BITS_PWRSEL) != BITS_PWRSEL) {
		result = inv_serial_single_write(mlsl_handle, pdata->address,
					MPUREG_PWR_MGMT_1, reg | BITS_PWRSEL);
		if (result) {
			LOG_RESULT_LOCATION(result);
			return result;
		}
	}
	msleep(2);

	result = inv_serial_read(mlsl_handle, pdata->address,
			MPUREG_PWR_MGMT_2, 1, &reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	reg &= ~(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA);
	result = inv_serial_single_write(mlsl_handle, pdata->address,
				       MPUREG_PWR_MGMT_2, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	if (slave->range.mantissa == 2)
		reg = 0;
	else if (slave->range.mantissa == 4)
		reg = 1 << 3;
	else if (slave->range.mantissa == 8)
		reg = 2 << 3;
	else if (slave->range.mantissa == 16)
		reg = 3 << 3;
	else
		return INV_ERROR;

	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_ACCEL_CONFIG, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	reg = (unsigned char)private_data->suspend.ths / ACCEL_MOT_THR_LSB;
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_ACCEL_MOT_THR, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	reg = (unsigned char)
	    ACCEL_ZRMOT_THR_LSB_CONVERSION(private_data->resume.ths);
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_ACCEL_ZRMOT_THR, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	reg = (unsigned char)private_data->suspend.ths / ACCEL_MOT_DUR_LSB;
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_ACCEL_MOT_DUR, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}

	reg = (unsigned char)private_data->resume.ths / ACCEL_ZRMOT_DUR_LSB;
	result = inv_serial_single_write(mlsl_handle, pdata->address,
					 MPUREG_ACCEL_ZRMOT_DUR, reg);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	return result;
}

static int mpu6050_read(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			unsigned char *data)
{
	int result;
	result = inv_serial_read(mlsl_handle, pdata->address,
				 slave->read_reg, slave->read_len, data);
	return result;
}

static int mpu6050_config(void *mlsl_handle,
			 struct ext_slave_descr *slave,
			 struct ext_slave_platform_data *pdata,
			 struct ext_slave_config *data)
{
	struct mpu6050_private_data *private_data = pdata->private_data;
	if (!data->data)
		return INV_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		return mpu6050_set_odr(mlsl_handle, pdata,
				      &private_data->suspend,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		return mpu6050_set_odr(mlsl_handle, pdata,
				      &private_data->resume,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		return mpu6050_set_fsr(mlsl_handle, pdata,
				      &private_data->suspend,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		return mpu6050_set_fsr(mlsl_handle, pdata,
				      &private_data->resume,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_THS:
		return mpu6050_set_ths(mlsl_handle, pdata,
				      &private_data->suspend,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_THS:
		return mpu6050_set_ths(mlsl_handle, pdata,
				      &private_data->resume,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_MOT_DUR:
		return mpu6050_set_dur(mlsl_handle, pdata,
				      &private_data->suspend,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		return mpu6050_set_dur(mlsl_handle, pdata,
				      &private_data->resume,
				      data->apply, *((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
#if 0
		return mpu6050_set_irq(mlsl_handle, pdata,
				      &private_data->suspend,
				      data->apply, *((long *)data->data));
#endif
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
#if 0
		return mpu6050_set_irq(mlsl_handle, pdata,
				      &private_data->resume,
				      data->apply, *((long *)data->data));
#endif
		break;
	default:
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return INV_SUCCESS;
}

static int mpu6050_get_config(void *mlsl_handle,
			     struct ext_slave_descr *slave,
			     struct ext_slave_platform_data *pdata,
			     struct ext_slave_config *data)
{
	struct mpu6050_private_data *private_data = pdata->private_data;
	if (!data->data)
		return INV_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.odr;
		break;
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.odr;
		break;
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.fsr;
		break;
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.fsr;
		break;
	case MPU_SLAVE_CONFIG_MOT_THS:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.ths;
		break;
	case MPU_SLAVE_CONFIG_NMOT_THS:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.ths;
		break;
	case MPU_SLAVE_CONFIG_MOT_DUR:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.dur;
		break;
	case MPU_SLAVE_CONFIG_NMOT_DUR:
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.dur;
		break;
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
#if 0
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->suspend.irq_type;
#endif
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
#if 0
		(*(unsigned long *)data->data) =
		    (unsigned long)private_data->resume.irq_type;
#endif
		break;
	default:
		return INV_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return INV_SUCCESS;
}

static struct ext_slave_descr mpu6050_descr = {
	.init             = mpu6050_init,
	.exit             = mpu6050_exit,
	.suspend          = mpu6050_suspend,
	.resume           = mpu6050_resume,
	.read             = mpu6050_read,
	.config           = mpu6050_config,
	.get_config       = mpu6050_get_config,
	.name             = "mpu6050",
	.type             = EXT_SLAVE_TYPE_ACCELEROMETER,
	.id               = ACCEL_ID_MPU6050,
	.read_reg         = 0x3B,
	.read_len         = 6,
	.endian           = EXT_SLAVE_BIG_ENDIAN,
	.range            = {2, 0},
	.trigger          = NULL,
};

struct ext_slave_descr *mpu6050_get_slave_descr(void)
{
	return &mpu6050_descr;
}

/**
 *  @}
 */
