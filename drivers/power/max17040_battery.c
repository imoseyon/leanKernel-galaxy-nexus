/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <linux/android_alarm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_BATTERY_FULL	100

#define HAS_ALERT_INTERRUPT(ver)	(ver >= 3)

#define FAST_POLL		(1 * 60)
#define SLOW_POLL		(10 * 60)

#define STATUS_CHARGABLE	0x0
#define STATUS_CHARGE_FULL	0x1
#define STATUS_ABNORMAL_TEMP	0x2
#define STATUS_CHARGE_TIMEOVER	0x3

struct max17040_chip {
	struct i2c_client		*client;
	struct work_struct		work;
	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* Health of Battery */
	int bat_health;
	/* Temperature of Battery */
	int bat_temp;

	struct notifier_block	pm_notifier;
	struct wake_lock	work_wake_lock;

	struct alarm	alarm;
	ktime_t last_poll;
	int slow_poll;
	int shutdown;
	/* chip version */
	u16 ver;

	int charger_status;
	unsigned long chg_limit_time;

	bool is_timer_flag;
};

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->bat_health;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!chip->pdata->get_bat_temp)
			return -ENODATA;
		val->intval = chip->bat_temp;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17040_write_reg(struct i2c_client *client, int reg, u16 val)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, cpu_to_be16(val));

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg, u16 *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		*val = 0;
		return ret;
	}

	*val = be16_to_cpu(ret);
	return 0;
}

static void max17040_reset(struct i2c_client *client)
{
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x5400);

	msleep(125);

	max17040_write_reg(client, MAX17040_MODE_MSB, 0x4000);
}

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u16 val;

	if (!max17040_read_reg(client, MAX17040_VCELL_MSB, &val))
		chip->vcell = (val >> 4) * 1250;
	else
		dev_warn(&client->dev, "i2c error, not updating vcell\n");
}

#define TO_FIXED(a,b) (((a) << 8) + (b))
#define FIXED_TO_INT(x) ((int)((x) >> 8))
#define FIXED_MULT(x,y) ((((u32)(x) * (u32)(y)) + (1 << 7)) >> 8)
#define FIXED_DIV(x,y) ((((u32)(x) << 8) + ((u32)(y) >> 1)) / (u32)(y))

static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u32 val;
	u32 fmin_cap = TO_FIXED(chip->pdata->min_capacity, 0);
	u16 regval;

	if (max17040_read_reg(client, MAX17040_SOC_MSB, &regval)) {
		dev_warn(&client->dev, "i2c error, not updating soc\n");
		return;
	}

	/* convert msb.lsb to Q8.8 */
	val = TO_FIXED(regval >> 8, regval & 0xff);
	if (val <= fmin_cap) {
		chip->soc = 0;
		return;
	}

	val = FIXED_MULT(TO_FIXED(100, 0), val - fmin_cap);
	val = FIXED_DIV(val, TO_FIXED(100, 0) - fmin_cap);
	chip->soc = clamp(FIXED_TO_INT(val), 0, 100);
}

static void max17040_get_version(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u16 val;

	if (!max17040_read_reg(client, MAX17040_VER_MSB, &val)) {
		chip->ver = val;
		dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d\n", val);
	} else {
		dev_err(&client->dev,
			"Error reading version, some features disabled\n");
	}
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (!chip->pdata->charger_online || !chip->pdata->charger_enable) {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) {
		if (chip->pdata->charger_enable()) {
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			chip->status =
				chip->charger_status == STATUS_CHARGE_FULL ?
				POWER_SUPPLY_STATUS_FULL :
				POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->chg_limit_time = 0;
		chip->charger_status = STATUS_CHARGABLE;
	}
}

static void max17040_get_temp_status(struct max17040_chip *chip)
{
	int r;
	int t;

	if (!chip->pdata->get_bat_temp)
		return;

	r = chip->pdata->get_bat_temp(&t);

	if (r < 0) {
		dev_err(&chip->client->dev,
			"error %d reading battery temperature\n", r);
		chip->bat_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		return;
	}

	chip->bat_temp = t;

	if (chip->bat_temp >= chip->pdata->high_block_temp) {
		chip->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (chip->bat_temp <= chip->pdata->high_recover_temp &&
		chip->bat_temp >= chip->pdata->low_recover_temp) {
		chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->bat_temp <= chip->pdata->low_block_temp) {
		chip->bat_health = POWER_SUPPLY_HEALTH_COLD;
	}
}

static void max17040_charger_update(struct max17040_chip *chip)
{
	ktime_t ktime;
	struct timespec cur_time;

	if (!chip->pdata->is_full_charge || !chip->pdata->allow_charging)
		return;

	ktime = alarm_get_elapsed_realtime();
	cur_time = ktime_to_timespec(ktime);

	switch (chip->charger_status) {
	case STATUS_CHARGABLE:
		if (chip->pdata->is_full_charge() &&
			chip->soc >= MAX17040_BATTERY_FULL &&
				chip->vcell > chip->pdata->fully_charged_vol) {
			chip->charger_status = STATUS_CHARGE_FULL;
			chip->is_timer_flag = true;
			chip->chg_limit_time = 0;
			chip->pdata->allow_charging(0);
		} else if (chip->chg_limit_time &&
				 cur_time.tv_sec > chip->chg_limit_time) {
			chip->charger_status = STATUS_CHARGE_TIMEOVER;
			chip->is_timer_flag = true;
			chip->chg_limit_time = 0;
			chip->pdata->allow_charging(0);
		} else if (chip->bat_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
			   chip->bat_health == POWER_SUPPLY_HEALTH_COLD) {
			chip->charger_status = STATUS_ABNORMAL_TEMP;
			chip->chg_limit_time = 0;
			chip->pdata->allow_charging(0);
		}
		break;

	case STATUS_CHARGE_FULL:
		if (chip->vcell <= chip->pdata->recharge_vol) {
			chip->charger_status = STATUS_CHARGABLE;
			chip->pdata->allow_charging(1);
		}
		break;

	case STATUS_ABNORMAL_TEMP:
		if (chip->bat_temp <= chip->pdata->high_recover_temp &&
			chip->bat_temp >=
				chip->pdata->low_recover_temp) {
			chip->charger_status = STATUS_CHARGABLE;
			chip->pdata->allow_charging(1);
		}
		break;

	case STATUS_CHARGE_TIMEOVER:
		if (chip->vcell <= chip->pdata->fully_charged_vol) {
			chip->charger_status = STATUS_CHARGABLE;
			chip->pdata->allow_charging(1);
		}
		break;

	default:
		dev_err(&chip->client->dev, "%s : invalid status [%d]\n",
			__func__, chip->charger_status);
	}

	if (!chip->chg_limit_time &&
			chip->charger_status == STATUS_CHARGABLE) {
		chip->chg_limit_time =
			chip->is_timer_flag ?
			cur_time.tv_sec + chip->pdata->limit_recharging_time :
			cur_time.tv_sec + chip->pdata->limit_charging_time;
	}

	dev_dbg(&chip->client->dev, "%s, Charger Status : %d, Limit Time : %ld\n",
			__func__, chip->charger_status, chip->chg_limit_time);
}

static void max17040_update(struct max17040_chip *chip)
{
	int prev_status = chip->status;
	int prev_soc = chip->soc;

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_temp_status(chip);
	if (chip->pdata->charger_online())
		max17040_charger_update(chip);
	else
		chip->is_timer_flag = false;
	max17040_get_status(chip->client);
	if ((chip->soc != prev_soc) || (chip->status != prev_status))
		power_supply_changed(&chip->battery);

	dev_info(&chip->client->dev, "online = %d vcell = %d soc = %d "
		"status = %d health = %d temp = %d "
		"charger status = %d\n", chip->online, chip->vcell,
		chip->soc, chip->status, chip->bat_health, chip->bat_temp,
		chip->charger_status);
}

static void max17040_program_alarm(struct max17040_chip *chip, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(chip->last_poll, low_interval);
	alarm_start_range(&chip->alarm, next, ktime_add(next, slack));
}

static void max17040_work(struct work_struct *work)
{
	unsigned long flags;
	struct timespec ts;
	struct max17040_chip *chip;

	chip = container_of(work, struct max17040_chip, work);

	max17040_update(chip);

	chip->last_poll = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(chip->last_poll);

	local_irq_save(flags);
	wake_unlock(&chip->work_wake_lock);
	if (!chip->shutdown)
		max17040_program_alarm(chip, FAST_POLL);
	local_irq_restore(flags);
}

static void max17040_battery_alarm(struct alarm *alarm)
{
	struct max17040_chip *chip =
		container_of(alarm, struct max17040_chip, alarm);

	wake_lock(&chip->work_wake_lock);
	schedule_work(&chip->work);

}

static void max17040_ext_power_changed(struct power_supply *psy)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	wake_lock(&chip->work_wake_lock);
	schedule_work(&chip->work);
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	/* must be last */
	POWER_SUPPLY_PROP_TEMP,
};

static int max17040_pm_notifier(struct notifier_block *notifier,
		unsigned long pm_event,
		void *unused)
{
	struct max17040_chip *chip =
		container_of(notifier, struct max17040_chip, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		if (!chip->pdata->charger_enable()) {
			cancel_work_sync(&chip->work);
			max17040_program_alarm(chip, SLOW_POLL);
			chip->slow_poll = 1;
		}
		break;

	case PM_POST_SUSPEND:
		/* We might be on a slow sample cycle.  If we're
		 * resuming we should resample the battery state
		 * if it's been over a minute since we last did
		 * so, and move back to sampling every minute until
		 * we suspend again.
		 */
		if (chip->slow_poll) {
			max17040_program_alarm(chip, FAST_POLL);
			chip->slow_poll = 0;
		}
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block max17040_pm_notifier_block = {
	.notifier_call = max17040_pm_notifier,
};

static irqreturn_t max17040_alert(int irq, void *data)
{
	struct max17040_chip *chip = data;
	struct i2c_client *client = chip->client;

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);

	dev_info(&client->dev, "Low battery alert fired: soc=%d vcell=%d\n",
		 chip->soc, chip->vcell);

	if (chip->soc != 0) {
		dev_err(&client->dev, "false low battery alert, ignoring\n");
		goto out;
	}

	dev_info(&client->dev, "shutting down due to low battery...\n");
	kernel_power_off();

out:
	return IRQ_HANDLED;
}

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;
	u16 val;
	u16 athd;
	int num_props = ARRAY_SIZE(max17040_battery_props);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	if (!chip->pdata->get_bat_temp)
		num_props--;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= num_props;
	chip->battery.external_power_changed	= max17040_ext_power_changed;

	chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
	chip->charger_status = STATUS_CHARGABLE;
	chip->is_timer_flag = false;
	chip->chg_limit_time = 0;

	if (!chip->pdata->high_block_temp)
		chip->pdata->high_block_temp = 500;
	if (!chip->pdata->high_recover_temp)
		chip->pdata->high_recover_temp = 420;
	if (!chip->pdata->low_block_temp)
		chip->pdata->low_block_temp = -50;
	if (!chip->pdata->fully_charged_vol)
		chip->pdata->fully_charged_vol = 4150000;
	if (!chip->pdata->recharge_vol)
		chip->pdata->recharge_vol = 4140000;
	if (!chip->pdata->limit_charging_time)
		chip->pdata->limit_charging_time = 21600;
	if (!chip->pdata->limit_recharging_time)
		chip->pdata->limit_recharging_time = 5400;

	chip->last_poll = alarm_get_elapsed_realtime();
	alarm_init(&chip->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			max17040_battery_alarm);

	wake_lock_init(&chip->work_wake_lock, WAKE_LOCK_SUSPEND,
			"max17040-battery");

	if (!chip->pdata->skip_reset)
		max17040_reset(client);

	max17040_get_version(client);
	INIT_WORK(&chip->work, max17040_work);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto err_battery_supply_register;
	}

	/* i2c-core does not support dev_pm_ops.prepare and .complete
	 * So, used pm_notifier for use android_alarm.
	 */
	chip->pm_notifier = max17040_pm_notifier_block;
	ret = register_pm_notifier(&chip->pm_notifier);
	if (ret) {
		dev_err(&client->dev, "failed: register pm notifier\n");
		goto err_pm_notifier;
	}
	schedule_work(&chip->work);

	if (HAS_ALERT_INTERRUPT(chip->ver) && chip->pdata->use_fuel_alert) {
		/* setting the low SOC alert threshold */
		if (!max17040_read_reg(client, MAX17040_RCOMP_MSB, &val)) {
			athd = chip->pdata->min_capacity > 1 ?
				chip->pdata->min_capacity - 1 : 0;
			max17040_write_reg(client, MAX17040_RCOMP_MSB,
					   (val & ~0x1f) | (-athd & 0x1f));
		} else {
			dev_err(&client->dev,
				"Error setting battery alert threshold\n");
		}

		/* add alert irq handler */
		ret = request_threaded_irq(client->irq, NULL, max17040_alert,
				IRQF_TRIGGER_FALLING, "fuel gauge alert", chip);
		if (ret < 0) {
			dev_err(&client->dev,
				"request_threaded_irq() failed: %d", ret);
			goto err_pm_notifier;
		}
	}

	return 0;

err_pm_notifier:
	power_supply_unregister(&chip->battery);
err_battery_supply_register:
	wake_lock_destroy(&chip->work_wake_lock);
	alarm_cancel(&chip->alarm);
	kfree(chip);

	return ret;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	chip->shutdown = 1;
	unregister_pm_notifier(&chip->pm_notifier);
	power_supply_unregister(&chip->battery);
	alarm_cancel(&chip->alarm);
	cancel_work_sync(&chip->work);
	wake_lock_destroy(&chip->work_wake_lock);
	if (HAS_ALERT_INTERRUPT(chip->ver) && chip->pdata->use_fuel_alert)
		free_irq(client->irq, chip);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
