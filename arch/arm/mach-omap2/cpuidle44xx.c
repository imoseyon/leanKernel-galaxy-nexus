/*
 * OMAP4 CPU idle Routines
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>

#include <asm/proc-fns.h>

#include <mach/omap4-common.h>

#include "pm.h"

#ifdef CONFIG_CPU_IDLE

#define OMAP4_MAX_STATES	1
/* C1 - CPU0 ON + + CPU1 ON + MPU ON + CORE ON */
#define OMAP4_STATE_C1		0

struct omap4_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 cpu0_state;
	u32 mpu_state;
	u32 core_state;
	u32 threshold;
	u32 flags;
};

struct omap4_processor_cx omap4_power_states[OMAP4_MAX_STATES];

static struct cpuidle_params cpuidle_params_table[] = {
	/* C1 */
	{1, 2, 2, 5},
};

/**
 * omap4_enter_idle - Programs OMAP4 to enter the specified state
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework to program the device to the
 * specified low power state selected by the governor.
 * Returns the amount of time spent in the low power state.
 */
static int omap4_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct timespec ts_preidle, ts_postidle, ts_idle;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();

	cpu_do_idle();

	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	local_irq_enable();
	local_fiq_enable();

	return ts_idle.tv_nsec / NSEC_PER_USEC + ts_idle.tv_sec * USEC_PER_SEC;
}

DEFINE_PER_CPU(struct cpuidle_device, omap4_idle_dev);

/**
 * omap4_init_power_states - Initialises the OMAP4 specific C states.
 *
 * Below is the desciption of each C state.
 * C1 : CPUx wfi + MPU inative + Core inactive
 */
void omap4_init_power_states(void)
{
	/*
	 * C1 - CPU0 ON + + CPU1 ON + MPU ON + CORE ON
	 */
	omap4_power_states[OMAP4_STATE_C1].valid =
			cpuidle_params_table[OMAP4_STATE_C1].valid;
	omap4_power_states[OMAP4_STATE_C1].type = OMAP4_STATE_C1;
	omap4_power_states[OMAP4_STATE_C1].sleep_latency =
			cpuidle_params_table[OMAP4_STATE_C1].sleep_latency;
	omap4_power_states[OMAP4_STATE_C1].wakeup_latency =
			cpuidle_params_table[OMAP4_STATE_C1].wake_latency;
	omap4_power_states[OMAP4_STATE_C1].threshold =
			cpuidle_params_table[OMAP4_STATE_C1].threshold;
	omap4_power_states[OMAP4_STATE_C1].mpu_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C1].core_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C1].flags = CPUIDLE_FLAG_TIME_VALID;

}

struct cpuidle_driver omap4_idle_driver = {
	.name =		"omap4_idle",
	.owner =	THIS_MODULE,
};

/**
 * omap4_idle_init - Init routine for OMAP4 idle
 *
 * Registers the OMAP4 specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init omap4_idle_init(void)
{
	int cpu_id = 0, i, count = 0;
	struct omap4_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	omap4_init_power_states();
	cpuidle_register_driver(&omap4_idle_driver);

	dev = &per_cpu(omap4_idle_dev, cpu_id);
	dev->cpu = cpu_id;
	count = 0;
	for (i = OMAP4_STATE_C1; i < OMAP4_MAX_STATES; i++) {
		cx = &omap4_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency +
						cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = omap4_enter_idle;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;

	if (cpuidle_register_device(dev)) {
		pr_err("%s: CPUidle register device failed\n", __func__);
			return -EIO;
		}

	return 0;
}
#else
int __init omap4_idle_init(void)
{
	return 0;
}
#endif /* CONFIG_CPU_IDLE */
