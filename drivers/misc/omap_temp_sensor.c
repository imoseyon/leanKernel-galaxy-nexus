/*
 * OMAP4 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <plat/common.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/temperature_sensor.h>
#include <plat/omap-pm.h>

/* TO DO: This needs to be fixed */
#include "../../../../arch/arm/mach-omap2/control.h"
/* #include <plat/control.h> */

#include <mach/ctrl_module_core_44xx.h>

extern void omap_thermal_throttle(void);
extern void omap_thermal_unthrottle(void);

static void throttle_delayed_work_fn(struct work_struct *work);

#define THROTTLE_DELAY_MS	1000

#define TSHUT_THRESHOLD_TSHUT_HOT	110000	/* 110 deg C */
#define TSHUT_THRESHOLD_TSHUT_COLD	100000	/* 100 deg C */
#define BGAP_THRESHOLD_T_HOT		64000	/* 64 deg C */
#define BGAP_THRESHOLD_T_COLD		61000	/* 61 deg C */
#define OMAP_ADC_START_VALUE	530
#define OMAP_ADC_END_VALUE	923

/*
 * omap_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 * @clock - Clock pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @irq - MPU Irq number for thermal alertemp_sensor
 * @tshut_irq -  Thermal shutdown IRQ
 * @phy_base - Physical base of the temp I/O
 * @is_efuse_valid - Flag to determine if eFuse is valid or not
 * @clk_on - Manages the current clock state
 * @clk_rate - Holds current clock rate
 */
struct omap_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
	struct clk *clock;
	struct spinlock lock;
	unsigned int irq;
	unsigned int tshut_irq;
	unsigned long phy_base;
	int is_efuse_valid;
	u8 clk_on;
	unsigned long clk_rate;
	u32 current_temp;
	struct delayed_work throttle_work;
};

#ifdef CONFIG_PM
struct omap_temp_sensor_regs {
	u32 temp_sensor_ctrl;
	u32 bg_ctrl;
	u32 bg_counter;
	u32 bg_threshold;
	u32 temp_sensor_tshut_threshold;
};

static struct omap_temp_sensor_regs temp_sensor_context;
static struct omap_temp_sensor *temp_sensor_pm;
#endif

/*
 * Temperature values in milli degrees celsius ADC code values from 530 to 923
 */
static int adc_to_temp[] = {
	-40000, -40000, -40000, -40000, -39800, -39400, -39000, -38600, -38200,
	-37800, -37300, -36800, -36400, -36000, -35600, -35200, -34800,
	-34300, -33800, -33400, -33000, -32600, -32200, -31800, -31300,
	-30800, -30400, -30000, -29600, -29200, -28700, -28200, -27800,
	-27400, -27000, -26600, -26200, -25700, -25200, -24800, -24400,
	-24000, -23600, -23200, -22700, -22200, -21800, -21400, -21000,
	-20600, -20200, -19700, -19200, -18800, -18400, -18000, -17600,
	-17200, -16700, -16200, -15800, -15400, -15000, -14600, -14200,
	-13700, -13200, -12800, -12400, -12000, -11600, -11200, -10700,
	-10200, -9800, -9400, -9000, -8600, -8200, -7700, -7200, -6800,
	-6400, -6000, -5600, -5200, -4800, -4300, -3800, -3400, -3000,
	-2600, -2200, -1800, -1300, -800, -400, 0, 400, 800, 1200, 1600,
	2100, 2600, 3000, 3400, 3800, 4200, 4600, 5100, 5600, 6000, 6400,
	6800, 7200, 7600, 8000, 8500, 9000, 9400, 9800, 10200, 10600, 11000,
	11400, 11900, 12400, 12800, 13200, 13600, 14000, 14400, 14800,
	15300, 15800, 16200, 16600, 17000, 17400, 17800, 18200, 18700,
	19200, 19600, 20000, 20400, 20800, 21200, 21600, 22100, 22600,
	23000, 23400, 23800, 24200, 24600, 25000, 25400, 25900, 26400,
	26800, 27200, 27600, 28000, 28400, 28800, 29300, 29800, 30200,
	30600, 31000, 31400, 31800, 32200, 32600, 33100, 33600, 34000,
	34400, 34800, 35200, 35600, 36000, 36400, 36800, 37300, 37800,
	38200, 38600, 39000, 39400, 39800, 40200, 40600, 41100, 41600,
	42000, 42400, 42800, 43200, 43600, 44000, 44400, 44800, 45300,
	45800, 46200, 46600, 47000, 47400, 47800, 48200, 48600, 49000,
	49500, 50000, 50400, 50800, 51200, 51600, 52000, 52400, 52800,
	53200, 53700, 54200, 54600, 55000, 55400, 55800, 56200, 56600,
	57000, 57400, 57800, 58200, 58700, 59200, 59600, 60000, 60400,
	60800, 61200, 61600, 62000, 62400, 62800, 63300, 63800, 64200,
	64600, 65000, 65400, 65800, 66200, 66600, 67000, 67400, 67800,
	68200, 68700, 69200, 69600, 70000, 70400, 70800, 71200, 71600,
	72000, 72400, 72800, 73200, 73600, 74100, 74600, 75000, 75400,
	75800, 76200, 76600, 77000, 77400, 77800, 78200, 78600, 79000,
	79400, 79800, 80300, 80800, 81200, 81600, 82000, 82400, 82800,
	83200, 83600, 84000, 84400, 84800, 85200, 85600, 86000, 86400,
	86800, 87300, 87800, 88200, 88600, 89000, 89400, 89800, 90200,
	90600, 91000, 91400, 91800, 92200, 92600, 93000, 93400, 93800,
	94200, 94600, 95000, 95500, 96000, 96400, 96800, 97200, 97600,
	98000, 98400, 98800, 99200, 99600, 100000, 100400, 100800, 101200,
	101600, 102000, 102400, 102800, 103200, 103600, 104000, 104400,
	104800, 105200, 105600, 106100, 106600, 107000, 107400, 107800,
	108200, 108600, 109000, 109400, 109800, 110200, 110600, 111000,
	111400, 111800, 112200, 112600, 113000, 113400, 113800, 114200,
	114600, 115000, 115400, 115800, 116200, 116600, 117000, 117400,
	117800, 118200, 118600, 119000, 119400, 119800, 120200, 120600,
	121000, 121400, 121800, 122200, 122600, 123000
};

static unsigned long omap_temp_sensor_readl(struct omap_temp_sensor
					    *temp_sensor, u32 reg)
{
	return omap_ctrl_readl(temp_sensor->phy_base + reg);
}

static void omap_temp_sensor_writel(struct omap_temp_sensor *temp_sensor,
				    u32 val, u32 reg)
{
	omap_ctrl_writel(val, (temp_sensor->phy_base + reg));
}

static int adc_to_temp_conversion(int adc_val)
{
	if (adc_val < OMAP_ADC_START_VALUE || adc_val > OMAP_ADC_END_VALUE) {
		pr_err("%s:Temp read is invalid %i\n", __func__, adc_val);
		return -EINVAL;
	}

	return adc_to_temp[adc_val - OMAP_ADC_START_VALUE];
}

static int temp_to_adc_conversion(long temp)
{
	int i;

	for (i = 0; i <= OMAP_ADC_END_VALUE - OMAP_ADC_START_VALUE; i++)
		if (temp < adc_to_temp[i])
			return OMAP_ADC_START_VALUE + i - 1;
	return -EINVAL;
}

static int omap_read_current_temp(struct omap_temp_sensor *temp_sensor)
{
	int adc;

	adc = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	adc &= (OMAP4_BGAP_TEMP_SENSOR_DTEMP_MASK);

	if (!temp_sensor->is_efuse_valid)
		pr_err_once("%s: Invalid EFUSE, Non-trimmed BGAP,"
			    "Temp not accurate\n", __func__ );

	if (adc < OMAP_ADC_START_VALUE || adc > OMAP_ADC_END_VALUE) {
		pr_err("%s:Invalid adc code reported by the sensor %d",
			__func__, adc);
		return -EINVAL;
	}

	return adc_to_temp_conversion(adc);
}

static void omap_configure_temp_sensor_thresholds(struct omap_temp_sensor
						  *temp_sensor)
{
	u32 temp = 0, t_hot, t_cold, tshut_hot, tshut_cold;

	t_hot = temp_to_adc_conversion(BGAP_THRESHOLD_T_HOT);
	t_cold = temp_to_adc_conversion(BGAP_THRESHOLD_T_COLD);

	if ((t_hot == -EINVAL) || (t_cold == -EINVAL)) {
		pr_err("%s:Temp thresholds out of bounds\n", __func__);
		return;
	}
	temp |= ((t_hot << OMAP4_T_HOT_SHIFT) | (t_cold << OMAP4_T_COLD_SHIFT));
	omap_temp_sensor_writel(temp_sensor, temp, BGAP_THRESHOLD_OFFSET);

	tshut_hot = temp_to_adc_conversion(TSHUT_THRESHOLD_TSHUT_HOT);
	tshut_cold = temp_to_adc_conversion(TSHUT_THRESHOLD_TSHUT_COLD);
	if ((tshut_hot == -EINVAL) || (tshut_cold == -EINVAL)) {
		pr_err("%s:Temp shutdown thresholds out of bounds\n", __func__);
		return;
	}
	temp |= ((tshut_hot << OMAP4_TSHUT_HOT_SHIFT)
			| (tshut_cold << OMAP4_TSHUT_COLD_SHIFT));
	omap_temp_sensor_writel(temp_sensor, temp, BGAP_TSHUT_OFFSET);
}

static void omap_configure_temp_sensor_counter(struct omap_temp_sensor
					       *temp_sensor, u32 counter)
{
	u32 val;

	val = omap_temp_sensor_readl(temp_sensor, BGAP_COUNTER_OFFSET);
	val = val & ~(OMAP4_COUNTER_MASK);
	val = val | (counter << OMAP4_COUNTER_SHIFT);
	omap_temp_sensor_writel(temp_sensor, val, BGAP_COUNTER_OFFSET);
}

static void omap_enable_continuous_mode(struct omap_temp_sensor *temp_sensor)
{
	u32 val;

	val = omap_temp_sensor_readl(temp_sensor, BGAP_CTRL_OFFSET);

	val = val | (1 << OMAP4_SINGLE_MODE_SHIFT);

	omap_temp_sensor_writel(temp_sensor, val, BGAP_CTRL_OFFSET);
}

/*
 * sysfs hook functions
 */
static ssize_t omap_temp_show_current(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", omap_read_current_temp(temp_sensor));
}

static ssize_t omap_throttle_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	if (count && buf[0] == '1')
		omap_thermal_throttle();
	else
		omap_thermal_unthrottle();

	return count;
}

static DEVICE_ATTR(temperature, S_IRUGO, omap_temp_show_current, NULL);
static DEVICE_ATTR(throttle, S_IWUSR, NULL, omap_throttle_store);
static struct attribute *omap_temp_sensor_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_throttle.attr,
	NULL
};

static const struct attribute_group omap_temp_sensor_group = {
	.attrs = omap_temp_sensor_attributes,
};

static int omap_temp_sensor_enable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	unsigned long clk_rate;

	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (temp_sensor->clk_on) {
		pr_debug("%s: clock already on\n", __func__);
		goto out;
	}

	ret = pm_runtime_get_sync(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:get sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	clk_set_rate(temp_sensor->clock, 1000000);
	clk_rate = clk_get_rate(temp_sensor->clock);
	temp_sensor->clk_rate = clk_rate;

	temp = omap_temp_sensor_readl(temp_sensor,
					TEMP_SENSOR_CTRL_OFFSET);
	temp &= ~(OMAP4_BGAP_TEMPSOFF_MASK);

	/* write BGAP_TEMPSOFF should be reset to 0 */
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);
	temp_sensor->clk_on = 1;

out:
spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}


static int omap_temp_sensor_disable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	u32 counter = 1000;
	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (!temp_sensor->clk_on) {
		pr_debug("%s: clock already off\n", __func__);
		goto out;
	}
	temp = omap_temp_sensor_readl(temp_sensor,
				TEMP_SENSOR_CTRL_OFFSET);
	temp |= OMAP4_BGAP_TEMPSOFF_MASK;

	/* write BGAP_TEMPSOFF should be set to 1 before gating clock */
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);
	temp = omap_temp_sensor_readl(temp_sensor, BGAP_STATUS_OFFSET);

	/* wait till the clean stop bit is set */
	while ((temp & OMAP4_CLEAN_STOP_MASK) && --counter)
		temp = omap_temp_sensor_readl(temp_sensor,
						BGAP_STATUS_OFFSET);
	/* Gate the clock */
	ret = pm_runtime_put_sync_suspend(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:put sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	temp_sensor->clk_on = 0;

out:
	spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}

/*
 * Check if the die sensor is cooling down. If it's higher than
 * t_hot since the last throttle then throttle it again.
 * OMAP junction temperature could stay for a long time in an
 * unacceptable temperature range. The idea here is to check after
 * t_hot->throttle the system really came below t_hot else re-throttle
 * and keep doing till it's under t_hot temp range.
 */
static void throttle_delayed_work_fn(struct work_struct *work)
{
	int curr;
	struct omap_temp_sensor *temp_sensor =
				container_of(work, struct omap_temp_sensor,
					     throttle_work.work);
	curr = omap_read_current_temp(temp_sensor);

	if (curr >= BGAP_THRESHOLD_T_HOT || curr < 0) {
		pr_warn("%s: OMAP temp read %d exceeds the threshold\n",
			__func__, curr);
		omap_thermal_throttle();
		schedule_delayed_work(&temp_sensor->throttle_work,
			msecs_to_jiffies(THROTTLE_DELAY_MS));
	} else {
		schedule_delayed_work(&temp_sensor->throttle_work,
			msecs_to_jiffies(THROTTLE_DELAY_MS));
	}
}

static irqreturn_t omap_tshut_irq_handler(int irq, void *data)
{
	struct omap_temp_sensor *temp_sensor = (struct omap_temp_sensor *)data;

	/* Need to handle thermal mgmt in bootloader
	 * to avoid restart again at kernel level
	 */
	if (temp_sensor->is_efuse_valid) {
		pr_emerg("%s: Thermal shutdown reached rebooting device\n",
			__func__);
		kernel_restart(NULL);
	} else {
		pr_err("%s:Invalid EFUSE, Non-trimmed BGAP\n", __func__);
	}

	return IRQ_HANDLED;
}

static irqreturn_t omap_talert_irq_handler(int irq, void *data)
{
	struct omap_temp_sensor *temp_sensor = (struct omap_temp_sensor *)data;
	int t_hot, t_cold, temp_offset;

	t_hot = omap_temp_sensor_readl(temp_sensor, BGAP_STATUS_OFFSET)
	    & OMAP4_HOT_FLAG_MASK;
	t_cold = omap_temp_sensor_readl(temp_sensor, BGAP_STATUS_OFFSET)
	    & OMAP4_COLD_FLAG_MASK;
	temp_offset = omap_temp_sensor_readl(temp_sensor, BGAP_CTRL_OFFSET);
	if (t_hot) {
		omap_thermal_throttle();
		schedule_delayed_work(&temp_sensor->throttle_work,
			msecs_to_jiffies(THROTTLE_DELAY_MS));
		temp_offset &= ~(OMAP4_MASK_HOT_MASK);
		temp_offset |= OMAP4_MASK_COLD_MASK;
	} else if (t_cold) {
		cancel_delayed_work_sync(&temp_sensor->throttle_work);
		omap_thermal_unthrottle();
		temp_offset &= ~(OMAP4_MASK_COLD_MASK);
		temp_offset |= OMAP4_MASK_HOT_MASK;
	}

	omap_temp_sensor_writel(temp_sensor, temp_offset, BGAP_CTRL_OFFSET);

	return IRQ_HANDLED;
}

static int __devinit omap_temp_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_temp_sensor_pdata *pdata = pdev->dev.platform_data;
	struct omap_temp_sensor *temp_sensor;
	struct resource *mem;
	int ret = 0, val;

	if (!pdata) {
		dev_err(dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct omap_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	spin_lock_init(&temp_sensor->lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "%s:no mem resource\n", __func__);
		ret = -EINVAL;
		goto plat_res_err;
	}

	temp_sensor->irq = platform_get_irq_byname(pdev, "thermal_alert");
	if (temp_sensor->irq < 0) {
		dev_err(dev, "%s:Cannot get thermal alert irq\n",
			__func__);
		ret = -EINVAL;
		goto get_irq_err;
	}

	ret = gpio_request_one(OMAP_TSHUT_GPIO, GPIOF_DIR_IN,
		"thermal_shutdown");
	if (ret) {
		dev_err(dev, "%s: Could not get tshut_gpio\n",
			__func__);
		goto tshut_gpio_req_err;
	}

	temp_sensor->tshut_irq = gpio_to_irq(OMAP_TSHUT_GPIO);
	if (temp_sensor->tshut_irq < 0) {
		dev_err(dev, "%s:Cannot get thermal shutdown irq\n",
			__func__);
		ret = -EINVAL;
		goto get_tshut_irq_err;
	}

	temp_sensor->phy_base = pdata->offset;
	temp_sensor->pdev = pdev;
	temp_sensor->dev = dev;

	pm_runtime_enable(dev);
	pm_runtime_irq_safe(dev);

	/*
	 * check if the efuse has a non-zero value if not
	 * it is an untrimmed sample and the temperatures
	 * may not be accurate */
	if (omap_readl(OMAP4_CTRL_MODULE_CORE +
			OMAP4_CTRL_MODULE_CORE_STD_FUSE_OPP_BGAP))
		temp_sensor->is_efuse_valid = 1;

	temp_sensor->clock = clk_get(&temp_sensor->pdev->dev, "fck");
	if (IS_ERR(temp_sensor->clock)) {
		ret = PTR_ERR(temp_sensor->clock);
		pr_err("%s:Unable to get fclk: %d\n", __func__, ret);
		ret = -EINVAL;
		goto clk_get_err;
	}

	/* Init delayed work for throttle decision */
	INIT_DELAYED_WORK(&temp_sensor->throttle_work,
			  throttle_delayed_work_fn);

	platform_set_drvdata(pdev, temp_sensor);

	ret = omap_temp_sensor_enable(temp_sensor);
	if (ret) {
		dev_err(dev, "%s:Cannot enable temp sensor\n", __func__);
		goto sensor_enable_err;
	}

	omap_enable_continuous_mode(temp_sensor);
	omap_configure_temp_sensor_thresholds(temp_sensor);
	/* 1 ms */
	omap_configure_temp_sensor_counter(temp_sensor, 1);

	/* Wait till the first conversion is done wait for at least 1ms */
	mdelay(2);

	/* Read the temperature once due to hw issue*/
	omap_read_current_temp(temp_sensor);

	/* Set 2 seconds time as default counter */
	omap_configure_temp_sensor_counter(temp_sensor,
						temp_sensor->clk_rate * 2);
	ret = request_threaded_irq(temp_sensor->irq, NULL,
			omap_talert_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"temp_sensor", (void *)temp_sensor);
	if (ret) {
		dev_err(dev, "Request threaded irq failed.\n");
		goto req_irq_err;
	}

	ret = request_threaded_irq(temp_sensor->tshut_irq, NULL,
			omap_tshut_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"tshut", (void *)temp_sensor);
	if (ret) {
		dev_err(dev, "Request threaded irq failed for TSHUT.\n");
		goto tshut_irq_req_err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	/* unmask the T_COLD and unmask T_HOT at init */
	val = omap_temp_sensor_readl(temp_sensor, BGAP_CTRL_OFFSET);
	val |= OMAP4_MASK_COLD_MASK;
	val |= OMAP4_MASK_HOT_MASK;
	omap_temp_sensor_writel(temp_sensor, val, BGAP_CTRL_OFFSET);

	dev_info(dev, "%s probed", pdata->name);

	temp_sensor_pm = temp_sensor;

	return 0;

sysfs_create_err:
	free_irq(temp_sensor->tshut_irq, temp_sensor);
	cancel_delayed_work_sync(&temp_sensor->throttle_work);
tshut_irq_req_err:
	free_irq(temp_sensor->irq, temp_sensor);
req_irq_err:
	platform_set_drvdata(pdev, NULL);
	omap_temp_sensor_disable(temp_sensor);
sensor_enable_err:
	clk_put(temp_sensor->clock);
clk_get_err:
	pm_runtime_disable(dev);
get_tshut_irq_err:
	gpio_free(OMAP_TSHUT_GPIO);
tshut_gpio_req_err:
get_irq_err:
plat_res_err:
	kfree(temp_sensor);
	return ret;
}

static int __devexit omap_temp_sensor_remove(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	cancel_delayed_work_sync(&temp_sensor->throttle_work);
	omap_temp_sensor_disable(temp_sensor);
	clk_put(temp_sensor->clock);
	platform_set_drvdata(pdev, NULL);
	if (temp_sensor->irq)
		free_irq(temp_sensor->irq, temp_sensor);
	if (temp_sensor->tshut_irq)
		free_irq(temp_sensor->tshut_irq, temp_sensor);
	kfree(temp_sensor);

	return 0;
}

#ifdef CONFIG_PM
static void omap_temp_sensor_save_ctxt(struct omap_temp_sensor *temp_sensor)
{
	temp_sensor_context.temp_sensor_ctrl =
	    omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp_sensor_context.bg_ctrl =
	    omap_temp_sensor_readl(temp_sensor, BGAP_CTRL_OFFSET);
	temp_sensor_context.bg_counter =
	    omap_temp_sensor_readl(temp_sensor, BGAP_COUNTER_OFFSET);
	temp_sensor_context.bg_threshold =
	    omap_temp_sensor_readl(temp_sensor, BGAP_THRESHOLD_OFFSET);
	temp_sensor_context.temp_sensor_tshut_threshold =
	    omap_temp_sensor_readl(temp_sensor, BGAP_TSHUT_OFFSET);
}

static void omap_temp_sensor_restore_ctxt(struct omap_temp_sensor *temp_sensor)
{
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.temp_sensor_ctrl,
				TEMP_SENSOR_CTRL_OFFSET);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.bg_ctrl,
				BGAP_CTRL_OFFSET);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.bg_counter,
				BGAP_COUNTER_OFFSET);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.bg_threshold,
				BGAP_THRESHOLD_OFFSET);
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.temp_sensor_tshut_threshold,
				BGAP_TSHUT_OFFSET);
}

static int omap_temp_sensor_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_disable(temp_sensor);

	return 0;
}

static int omap_temp_sensor_resume(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_enable(temp_sensor);

	return 0;
}

void omap_temp_sensor_idle(int idle_state)
{
	if (!temp_sensor_pm)
		return;

	if (idle_state)
		omap_temp_sensor_disable(temp_sensor_pm);
	else
		omap_temp_sensor_enable(temp_sensor_pm);
}

#else
omap_temp_sensor_suspend NULL
omap_temp_sensor_resume NULL

#endif /* CONFIG_PM */
static int omap_temp_sensor_runtime_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));

	omap_temp_sensor_save_ctxt(temp_sensor);
	return 0;
}

static int omap_temp_sensor_runtime_resume(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));
	if (omap_pm_was_context_lost(dev)) {
		omap_temp_sensor_restore_ctxt(temp_sensor);
	}
	return 0;
}

static const struct dev_pm_ops omap_temp_sensor_dev_pm_ops = {
	.runtime_suspend = omap_temp_sensor_runtime_suspend,
	.runtime_resume = omap_temp_sensor_runtime_resume,
};

static struct platform_driver omap_temp_sensor_driver = {
	.probe = omap_temp_sensor_probe,
	.remove = omap_temp_sensor_remove,
	.suspend = omap_temp_sensor_suspend,
	.resume = omap_temp_sensor_resume,
	.driver = {
		.name = "omap_temp_sensor",
		.pm = &omap_temp_sensor_dev_pm_ops,
	},
};

int __init omap_temp_sensor_init(void)
{
	if (!cpu_is_omap446x())
		return 0;

	return platform_driver_register(&omap_temp_sensor_driver);
}

static void __exit omap_temp_sensor_exit(void)
{
	platform_driver_unregister(&omap_temp_sensor_driver);
}

module_init(omap_temp_sensor_init);
module_exit(omap_temp_sensor_exit);

MODULE_DESCRIPTION("OMAP446X Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
