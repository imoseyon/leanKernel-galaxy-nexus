/*
 * OMAP4 CPU idle Routines
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/clockchips.h>
#include <linux/notifier.h>
#include <linux/cpu.h>

#include <asm/proc-fns.h>

#include <mach/omap4-common.h>

#include "pm.h"
#include "prm.h"

#ifdef CONFIG_CPU_IDLE

#define OMAP4_MAX_STATES	4

/* C1 - CPU0 ON + + CPU1 ON + MPU ON + CORE ON */
#define OMAP4_STATE_C1		0
/* C2 - CPU0 ON + CPU1 OFF + MPU ON + CORE ON */
#define OMAP4_STATE_C2		1
/* C3 - CPU0 OFF + CPU1 OFF + MPU CSWR + CORE ON */
#define OMAP4_STATE_C3		2
/* C4 - CPU0 OFF + CPU1 OFF + MPU OFF + CORE ON */
#define OMAP4_STATE_C4		3

struct omap4_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 cpu0_state;
	u32 mpu_state;
	u32 mpu_logic_state;
	u32 core_state;
	u32 core_logic_state;
	u32 threshold;
	u32 flags;
	const char *desc;
};

struct omap4_processor_cx omap4_power_states[OMAP4_MAX_STATES];
static struct powerdomain *mpu_pd, *cpu1_pd, *core_pd;
static int needs_state_data_update;
static unsigned int state_flags = CPUIDLE_FLAG_IGNORE;

/*
 * FIXME: Full latency numbers needs to be updated as part of
 * cpuidle CORE retention support.
 * Currently only MPUSS latency numbers are added based on
 * measurements done internally. The numbers for MPUSS are
 * not board dependent and hence set directly here instead of
 * passing it from board files.
 */
static struct cpuidle_params cpuidle_params_table[] = {
	/* C1 - CPU0 ON + CPU1 ON + MPU ON  + CORE ON */
	{1,	2,	2,	5},
	/* C2 - CPU0 ON + CPU1 OFF + MPU ON  + CORE ON */
	{1,	140,	160,	300},
	/* C3 - CPU0 OFF + CPU1 OFF + MPU CSWR + CORE ON */
	{1,	200,	300,	700},
	/* C4 - CPU0 OFF + CPU1 OFF + MPU OFF + CORE ON */
	{1,	1400,	600,	5000},
};

/**
 * omap4_prepare_idle - Update C-state parameters dynamically
 * @dev: cpuidle device
 *
 * Called from the CPUidle framework to prepare the device
 * for idle before before calling the governor's select function.
 */
static int omap4_prepare_idle(struct cpuidle_device *dev)
{
	int i, ret = 0;

	if (!needs_state_data_update)
		return ret;

	/*
	 * Update the C-state flags based on CPU1 online
	 * or offline state. On OMAP4, the low power C-states
	 * are made available when only CPU1 is offline.
	 */
	for (i = OMAP4_STATE_C2; i < OMAP4_MAX_STATES; i++)
		dev->states[i].flags = state_flags;

	return ret;
}

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
	struct omap4_processor_cx *cx = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	u32 cpu1_state;
	int cpu_id = smp_processor_id();

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();

	/*
	 * Continue to do only WFI on CPU0 till CPU1 hits OFF state.
	 * This is necessary to honour hardware recommondation
	 * of triggeing all the possible low power modes once CPU1 is
	 * out of coherency and in OFF mode.
	 * Update dev->last_state so that governor stats reflects right
	 * data.
	 */
	cpu1_state = pwrdm_read_pwrst(cpu1_pd);
	if ((cpu1_state != PWRDM_POWER_OFF) || (!cx->valid)) {
		dev->last_state = dev->safe_state;
		cx = cpuidle_get_statedata(dev->safe_state);
	}

	pwrdm_set_logic_retst(mpu_pd, cx->mpu_logic_state);
	omap_set_pwrdm_state(mpu_pd, cx->mpu_state);
	pwrdm_set_logic_retst(core_pd, cx->core_logic_state);
	omap_set_pwrdm_state(core_pd, cx->core_state);

	if (cx->type > OMAP4_STATE_C1)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu_id);

	omap4_enter_lowpower(dev->cpu, cx->cpu0_state);

	if (cx->type > OMAP4_STATE_C1)
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu_id);

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
	omap4_power_states[OMAP4_STATE_C1].cpu0_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C1].mpu_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C1].mpu_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C1].core_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C1].core_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C1].flags = CPUIDLE_FLAG_TIME_VALID;
	omap4_power_states[OMAP4_STATE_C1].desc = "MPU ON + CORE ON";

	/*
	 * C2 - CPU0 ON + CPU1 OFF + MPU ON + CORE ON
	 */
	omap4_power_states[OMAP4_STATE_C2].valid =
			cpuidle_params_table[OMAP4_STATE_C2].valid;
	omap4_power_states[OMAP4_STATE_C2].type = OMAP4_STATE_C2;
	omap4_power_states[OMAP4_STATE_C2].sleep_latency =
			cpuidle_params_table[OMAP4_STATE_C2].sleep_latency;
	omap4_power_states[OMAP4_STATE_C2].wakeup_latency =
			cpuidle_params_table[OMAP4_STATE_C2].wake_latency;
	omap4_power_states[OMAP4_STATE_C2].threshold =
			cpuidle_params_table[OMAP4_STATE_C2].threshold;
	omap4_power_states[OMAP4_STATE_C2].cpu0_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C2].mpu_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C2].mpu_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C2].core_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C2].core_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C2].flags = CPUIDLE_FLAG_IGNORE;
	omap4_power_states[OMAP4_STATE_C2].desc = "MPU ON + CORE ON";

	/*
	 * C3 - CPU0 OFF + CPU1 OFF + MPU CSWR + CORE ON
	 */
	omap4_power_states[OMAP4_STATE_C3].valid =
			cpuidle_params_table[OMAP4_STATE_C3].valid;
	omap4_power_states[OMAP4_STATE_C3].type = OMAP4_STATE_C3;
	omap4_power_states[OMAP4_STATE_C3].sleep_latency =
			cpuidle_params_table[OMAP4_STATE_C3].sleep_latency;
	omap4_power_states[OMAP4_STATE_C3].wakeup_latency =
			cpuidle_params_table[OMAP4_STATE_C3].wake_latency;
	omap4_power_states[OMAP4_STATE_C3].threshold =
			cpuidle_params_table[OMAP4_STATE_C3].threshold;
	omap4_power_states[OMAP4_STATE_C3].cpu0_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C3].mpu_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].mpu_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].core_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C3].core_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].flags = CPUIDLE_FLAG_IGNORE;
	omap4_power_states[OMAP4_STATE_C3].desc = "MPU CSWR + CORE ON";

	/*
	 * C4 - CPU0 OFF + CPU1 OFF + MPU OFF + CORE ON
	 */
	omap4_power_states[OMAP4_STATE_C4].valid =
			cpuidle_params_table[OMAP4_STATE_C4].valid;
	omap4_power_states[OMAP4_STATE_C4].type = OMAP4_STATE_C4;
	omap4_power_states[OMAP4_STATE_C4].sleep_latency =
			cpuidle_params_table[OMAP4_STATE_C4].sleep_latency;
	omap4_power_states[OMAP4_STATE_C4].wakeup_latency =
			cpuidle_params_table[OMAP4_STATE_C4].wake_latency;
	omap4_power_states[OMAP4_STATE_C4].threshold =
			cpuidle_params_table[OMAP4_STATE_C4].threshold;
	omap4_power_states[OMAP4_STATE_C4].cpu0_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C4].mpu_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C4].mpu_logic_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C4].core_state = PWRDM_POWER_ON;
	omap4_power_states[OMAP4_STATE_C4].core_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C4].flags = CPUIDLE_FLAG_IGNORE;
	omap4_power_states[OMAP4_STATE_C4].desc = "MPU OFF + CORE ON";

}

struct cpuidle_driver omap4_idle_driver = {
	.name =		"omap4_idle",
	.owner =	THIS_MODULE,
};

/*
 * CPU hotplug notifier to update the C-states when
 * CPU1 is offline or onine. While updating C-state flag,
 * keep the cpuidle disabled.
 */
static int __cpuinit omap_cpu_hotplug_notify(struct notifier_block *self,
					 unsigned long action, void *unused)
{
	switch (action) {
	case CPU_ONLINE:
		disable_hlt();
		needs_state_data_update = 1;
		state_flags = CPUIDLE_FLAG_IGNORE;
		enable_hlt();
		break;
	case CPU_DEAD:
		disable_hlt();
		needs_state_data_update = 1;
		state_flags = CPUIDLE_FLAG_TIME_VALID;
		enable_hlt();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata omap_ilde_hotplug_notifier = {
	.notifier_call = omap_cpu_hotplug_notify,
};

/**
 * omap4_idle_init - Init routine for OMAP4 idle
 *
 * Registers the OMAP4 specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int __init omap4_idle_init(void)
{
	int cpu_id = 0, i, count = 0, ret;
	struct omap4_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	cpu1_pd = pwrdm_lookup("cpu1_pwrdm");
	core_pd = pwrdm_lookup("core_pwrdm");

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
		if (cx->type == OMAP4_STATE_C1)
			dev->safe_state = state;
		state->enter = omap4_enter_idle;
		sprintf(state->name, "C%d", count+1);
		strncpy(state->desc, cx->desc, CPUIDLE_DESC_LEN);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;
	dev->prepare = omap4_prepare_idle;

	if (cpuidle_register_device(dev)) {
		pr_err("%s: CPUidle register device failed\n", __func__);
			return -EIO;
		}

	ret = register_hotcpu_notifier(&omap_ilde_hotplug_notifier);
	if (ret)
		return ret;

	return 0;
}
#else
int __init omap4_idle_init(void)
{
	return 0;
}
#endif /* CONFIG_CPU_IDLE */
