/*
 * omap-pm-interface.c - OMAP power management interface
 *
 * This code implements the OMAP power management interface to
 * drivers, CPUIdle, CPUFreq, and DSP Bridge.
 *
 * Copyright (C) 2008-2011 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 * Paul Walmsley
 *
 * Interface developed by (in alphabetical order):
 * Karthik Dasu, Tony Lindgren, Rajendra Nayak, Sakari Poussa, Veeramanikandan
 * Raju, Anand Sawant, Igor Stoppa, Paul Walmsley, Richard Woodruff
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/platform_device.h>

/* Interface documentation is in mach/omap-pm.h */
#include <plat/omap-pm.h>
#include <plat/omap_device.h>

static bool off_mode_enabled;

/*
 * Device-driver-originated constraints (via board-*.c files)
 * WARNING: Device drivers need to now use pm_qos directly.
 */
int omap_pm_set_max_mpu_wakeup_lat(struct pm_qos_request_list **pmqos_req,
			long t)
{
	WARN(1, "Deprecated %s: Driver should use pm_qos to add request\n",
		__func__);

	return -EINVAL;
}

int omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, long r)
{
	if (!dev || (agent_id != OCP_INITIATOR_AGENT &&
	    agent_id != OCP_TARGET_AGENT)) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	};

	if (r == 0)
		pr_debug("OMAP PM: remove min bus tput constraint: "
			 "dev %s for agent_id %d\n", dev_name(dev), agent_id);
	else
		pr_debug("OMAP PM: add min bus tput constraint: "
			 "dev %s for agent_id %d: rate %ld KiB\n",
			 dev_name(dev), agent_id, r);

	/*
	 * This code should model the interconnect and compute the
	 * required clock frequency, convert that to a VDD2 OPP ID, then
	 * set the VDD2 OPP appropriately.
	 *
	 * TI CDP code can call constraint_set here on the VDD2 OPP.
	 */

	return 0;
}

int omap_pm_set_max_dev_wakeup_lat(struct device *req_dev, struct device *dev,
				   long t)
{
	if (!req_dev || !dev || t < -1) {
		WARN(1, "OMAP PM: %s: invalid parameter(s)", __func__);
		return -EINVAL;
	};

	if (t == -1)
		pr_debug("OMAP PM: remove max device latency constraint: "
			 "dev %s\n", dev_name(dev));
	else
		pr_debug("OMAP PM: add max device latency constraint: "
			 "dev %s, t = %ld usec\n", dev_name(dev), t);

	/*
	 * For current Linux, this needs to map the device to a
	 * powerdomain, then go through the list of current max lat
	 * constraints on that powerdomain and find the smallest.  If
	 * the latency constraint has changed, the code should
	 * recompute the state to enter for the next powerdomain
	 * state.  Conceivably, this code should also determine
	 * whether to actually disable the device clocks or not,
	 * depending on how long it takes to re-enable the clocks.
	 *
	 * TI CDP code can call constraint_set here.
	 */

	return 0;
}

/* WARNING: Device drivers need to now use pm_qos directly. */
int omap_pm_set_max_sdma_lat(struct pm_qos_request_list **qos_request, long t)
{
	WARN(1, "Deprecated %s: Driver should use pm_qos to add request\n",
		__func__);

	return -EINVAL;
}

int omap_pm_set_min_clk_rate(struct device *dev, struct clk *c, long r)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return -EINVAL;
}

/*
 * DSP Bridge-specific constraints
 * WARNING: Device drivers need to now use opp layer/omap_device_scale directly.
 */
const struct omap_opp *omap_pm_dsp_get_opp_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_dsp_set_min_opp(u8 opp_id)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return;
}


u8 omap_pm_dsp_get_opp(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return 0;
}

/*
 * CPUFreq-originated constraint
 *
 * In the future, this should be handled by custom OPP clocktype
 * functions.
 */

struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return ERR_PTR(-EINVAL);
}

void omap_pm_cpu_set_freq(unsigned long f)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return;
}

unsigned long omap_pm_cpu_get_freq(void)
{
	WARN(1, "Deprecated %s: Driver should use omap_device_scale/opp\n",
		__func__);

	return 0;
}

/**
 * omap_pm_enable_off_mode - notify OMAP PM that off-mode is enabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been enabled.
 */
void omap_pm_enable_off_mode(void)
{
	off_mode_enabled = true;
}

/**
 * omap_pm_disable_off_mode - notify OMAP PM that off-mode is disabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been disabled.
 */
void omap_pm_disable_off_mode(void)
{
	off_mode_enabled = false;
}

/*
 * Device context loss tracking
 * WARNING: at this point we dont have a reliable context loss reporting
 * mechanism. Instead, we ensure that we report context loss always.
 */
int omap_pm_get_dev_context_loss_count(struct device *dev)
{
	static u32 count = 1;

	if (!dev) {
		WARN_ON(1);
		return -EINVAL;
	};

	count++;

	/*
	 * Context loss count has to be a non-negative value.
	 * Clear the sign bit to get a value range from 0 to
	 * INT_MAX. Roll over to 1
	 */
	count = (count & ~INT_MAX) ? 1 : count;

	pr_debug("OMAP PM: returning context loss count for dev %s count %ul\n",
		 dev_name(dev), count);
	return count;
}

/* Should be called before clk framework init */
int __init omap_pm_if_early_init(void)
{
	return 0;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init(void)
{
	return 0;
}

void omap_pm_if_exit(void)
{
	/* Deallocate CPUFreq frequency table here */
}

