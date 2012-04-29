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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/clockchips.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>

#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/hardware/gic.h>

#include <mach/omap4-common.h>
#include <mach/omap-wakeupgen.h>

#include <plat/gpio.h>

#include "clockdomain.h"
#include "pm.h"
#include "prm.h"

#ifdef CONFIG_CPU_IDLE

#ifdef CONFIG_OMAP_ALLOW_OSWR
#define CPU_IDLE_ALLOW_OSWR	1
#else
#define CPU_IDLE_ALLOW_OSWR	0
#endif

/* C1 is a single-cpu C-state, it can be entered by each cpu independently */
/* C1 - CPUx WFI + MPU ON + CORE ON */
#define OMAP4_STATE_C1		0
/* C2 through C4 are shared C-states, both CPUs must agree to enter */
/* C2 - CPUx OFF + MPU INA + CORE INA */
#define OMAP4_STATE_C2		1
/* C3 - CPUx OFF + MPU CSWR + CORE OSWR */
#define OMAP4_STATE_C3		2
/* C4 - CPUx OFF + MPU OSWR + CORE OSWR */
#define OMAP4_STATE_C4		3

#define OMAP4_MAX_STATES	4

static bool disallow_smp_idle;
module_param(disallow_smp_idle, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disallow_smp_idle,
	"Don't enter idle if multiple cpus are active");

static bool skip_off;
module_param(skip_off, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(skip_off,
	"Do everything except actually enter the low power state (debugging)");

static bool keep_core_on;
module_param(keep_core_on, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(keep_core_on,
	"Prevent core powerdomain from entering any low power states (debugging)");

static bool keep_mpu_on;
module_param(keep_mpu_on, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(keep_mpu_on,
	"Prevent mpu powerdomain from entering any low power states (debugging)");

static int max_state;
module_param(max_state, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_state,
	"Select deepest power state allowed (0=any, 1=WFI, 2=INA, 3=CSWR, 4=OSWR)");

static int only_state;
module_param(only_state, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(only_state,
	"Select only power state allowed (0=any, 1=WFI, 2=INA, 3=CSWR, 4=OSWR)");

static const int omap4_poke_interrupt[2] = {
	OMAP44XX_IRQ_CPUIDLE_POKE0,
	OMAP44XX_IRQ_CPUIDLE_POKE1
};

struct omap4_processor_cx {
	u8 valid;
	u8 type;
	u32 exit_latency;
	u32 target_residency;
	u32 mpu_state;
	u32 mpu_logic_state;
	u32 core_state;
	u32 core_logic_state;
	const char *desc;
};

struct omap4_processor_cx omap4_power_states[OMAP4_MAX_STATES];
static struct powerdomain *mpu_pd, *cpu1_pd, *core_pd;
static struct omap4_processor_cx *omap4_idle_requested_cx[NR_CPUS];
static int omap4_idle_ready_count;
static DEFINE_SPINLOCK(omap4_idle_lock);
static struct clockdomain *cpu1_cd;

/*
 * Raw measured exit latency numbers (us):
 * state	average		max
 * C2		383		1068
 * C3		641		1190
 * C4		769		1323
 */

static __initdata struct cpuidle_params omap443x_cpuidle_params_table[] = {
	/* C1 - CPUx WFI + MPU ON  + CORE ON */
	{
		.exit_latency = 4,
		.target_residency = 4,
		.valid = 1,
	},
	/* C2 - CPUx OFF + MPU INA  + CORE INA */
	{
		.exit_latency = 300,
		.target_residency = 300,
		.valid = 1,
	},
	/* C3 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 5000,
		.target_residency = 10000,
		.valid = 1,
	},
	/* C4 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 5200,
		.target_residency = 35000,
		.valid = CPU_IDLE_ALLOW_OSWR,
	},
};

static __initdata struct cpuidle_params omap446x_cpuidle_params_table[] = {
	/* C1 - CPUx WFI + MPU ON  + CORE ON */
	{
		.exit_latency = 4,
		.target_residency = 4,
		.valid = 1,
	},
	/* C2 - CPUx OFF + MPU INA  + CORE INA */
	{
		.exit_latency = 300,
		.target_residency = 1800,
		.valid = 1,
	},
	/* C3 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 4000,
		.target_residency = 4000,
		.valid = 1,
	},
	/* C4 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 4200,
		.target_residency = 4200,
		.valid = CPU_IDLE_ALLOW_OSWR,
	},
};

static __initdata struct cpuidle_params omap447x_cpuidle_params_table[] = {
	/* C1 - CPUx WFI + MPU ON  + CORE ON */
	{
		.exit_latency = 4,
		.target_residency = 4,
		.valid = 1,
	},
	/* C2 - CPUx OFF + MPU INA  + CORE INA */
	{
		.exit_latency = 500,
		.target_residency = 1200,
		.valid = 1,
	},
	/* C3 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 5300,
		.target_residency = 5300,
		.valid = 1,
	},
	/* C4 - CPUx OFF + MPU CSWR + CORE OSWR */
	{
		.exit_latency = 5500,
		.target_residency = 15000,
		.valid = CPU_IDLE_ALLOW_OSWR,
	},
};

static void omap4_update_actual_state(struct cpuidle_device *dev,
	struct omap4_processor_cx *cx)
{
	int i;

	for (i = 0; i < dev->state_count; i++) {
		if (dev->states[i].driver_data == cx) {
			dev->last_state = &dev->states[i];
			return;
		}
	}
}

static bool omap4_gic_interrupt_pending(void)
{
	void __iomem *gic_cpu = omap4_get_gic_cpu_base();

	return (__raw_readl(gic_cpu + GIC_CPU_HIGHPRI) != 0x3FF);
}

/**
 * omap4_wfi_until_interrupt
 *
 * wfi can sometimes return with no interrupts pending, for example on a
 * broadcast cache flush or tlb op.  This function will call wfi repeatedly
 * until an interrupt is actually pending.  Returning without looping would
 * cause very short idle times to be reported to the idle governor, messing
 * with repeating interrupt detection, and causing deep idle states to be
 * avoided.
 */
static void omap4_wfi_until_interrupt(void)
{
retry:
	omap_do_wfi();

	if (!omap4_gic_interrupt_pending())
		goto retry;
}

/**
 * omap4_idle_wait
 *
 * similar to WFE, but can be woken by an interrupt even though interrupts
 * are masked.  An "event" is emulated by per-cpu unused interrupt in the GIC.
 * Returns false if wake caused by an interrupt, true if by an "event".
 */
static bool omap4_idle_wait(void)
{
	int cpu = hard_smp_processor_id();
	void __iomem *gic_dist = omap4_get_gic_dist_base();
	u32 bit = BIT(omap4_poke_interrupt[cpu] % 32);
	u32 reg = (omap4_poke_interrupt[cpu] / 32) * 4;
	bool poked;

	/* Unmask the "event" interrupt */
	__raw_writel(bit, gic_dist + GIC_DIST_ENABLE_SET + reg);

	omap4_wfi_until_interrupt();

	/* Read the "event" interrupt pending bit */
	poked = __raw_readl(gic_dist + GIC_DIST_PENDING_SET + reg) & bit;

	/* Mask the "event" */
	__raw_writel(bit, gic_dist + GIC_DIST_ENABLE_CLEAR + reg);

	/* Clear the event */
	if (poked)
		__raw_writel(bit, gic_dist + GIC_DIST_PENDING_CLEAR + reg);

	return poked;
}

/**
 * omap4_poke_cpu
 * @cpu: cpu to wake
 *
 * trigger an "event" to wake a cpu from omap4_idle_wait.
 */
static void omap4_poke_cpu(int cpu)
{
	void __iomem *gic_dist = omap4_get_gic_dist_base();
	u32 bit = BIT(omap4_poke_interrupt[cpu] % 32);
	u32 reg = (omap4_poke_interrupt[cpu] / 32) * 4;

	__raw_writel(bit, gic_dist + GIC_DIST_PENDING_SET + reg);
}

/**
 * omap4_enter_idle
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Idle function for C1 state, WFI on a single CPU.
 * Called with irqs off, returns with irqs on.
 * Returns the amount of time spent in the low power state.
 */
static int omap4_enter_idle_wfi(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	ktime_t preidle, postidle;

	local_fiq_disable();

	preidle = ktime_get();

	omap4_wfi_until_interrupt();

	postidle = ktime_get();

	local_fiq_enable();
	local_irq_enable();

	omap4_update_actual_state(dev, &omap4_power_states[OMAP4_STATE_C1]);

	return ktime_to_us(ktime_sub(postidle, preidle));
}

static inline bool omap4_all_cpus_idle(void)
{
	int i;

	assert_spin_locked(&omap4_idle_lock);

	for_each_online_cpu(i)
		if (omap4_idle_requested_cx[i] == NULL)
			return false;

	return true;
}

static inline struct omap4_processor_cx *omap4_get_idle_state(void)
{
	struct omap4_processor_cx *cx = NULL;
	int i;

	assert_spin_locked(&omap4_idle_lock);

	for_each_online_cpu(i)
		if (!cx || omap4_idle_requested_cx[i]->type < cx->type)
			cx = omap4_idle_requested_cx[i];

	return cx;
}

static void omap4_cpu_poke_others(int cpu)
{
	int i;

	for_each_online_cpu(i)
		if (i != cpu)
			omap4_poke_cpu(i);
}

static void omap4_cpu_update_state(int cpu, struct omap4_processor_cx *cx)
{
	assert_spin_locked(&omap4_idle_lock);

	omap4_idle_requested_cx[cpu] = cx;
	omap4_cpu_poke_others(cpu);
}

/**
 * omap4_enter_idle_primary
 * @cx: target idle state
 *
 * Waits for cpu1 to be off, then starts the transition to the target power
 * state for cpu0, mpu and core power domains.
 */
static void omap4_enter_idle_primary(struct omap4_processor_cx *cx)
{
	int cpu = 0;
	int ret;
	int count = 1000000;

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	cpu_pm_enter();

	if (skip_off)
		goto out;

	/* spin until cpu1 is really off */
	while ((pwrdm_read_pwrst(cpu1_pd) != PWRDM_POWER_OFF) && count--)
		cpu_relax();

	if (pwrdm_read_pwrst(cpu1_pd) != PWRDM_POWER_OFF)
		goto wake_cpu1;

	ret = pwrdm_wait_transition(cpu1_pd);
	if (ret)
		goto wake_cpu1;

	if (!keep_mpu_on) {
		pwrdm_set_logic_retst(mpu_pd, cx->mpu_logic_state);
		omap_set_pwrdm_state(mpu_pd, cx->mpu_state);
	}

	if (!keep_core_on) {
		pwrdm_set_logic_retst(core_pd, cx->core_logic_state);
		omap_set_pwrdm_state(core_pd, cx->core_state);
	}

	pr_debug("%s: cpu0 down\n", __func__);

	omap4_enter_sleep(0, PWRDM_POWER_OFF, false);

	pr_debug("%s: cpu0 up\n", __func__);

	/* restore the MPU and CORE states to ON */
	omap_set_pwrdm_state(mpu_pd, PWRDM_POWER_ON);
	omap_set_pwrdm_state(core_pd, PWRDM_POWER_ON);

wake_cpu1:
	if (!cpu_is_offline(1)) {
		/*
		 * Work around a ROM bug that causes CPU1 to corrupt the
		 * gic distributor enable register on 4460 by disabling
		 * the gic distributor before waking CPU1, and then waiting
		 * for CPU1 to re-enable the gic distributor before continuing.
		 */
		if (!cpu_is_omap443x())
			gic_dist_disable();

		clkdm_wakeup(cpu1_cd);

		if (!cpu_is_omap443x())
			while (gic_dist_disabled())
				cpu_relax();

		/*
		 * cpu1 mucks with page tables while it is starting,
		 * prevent cpu0 executing any processes until cpu1 is up
		 */
		while (omap4_idle_requested_cx[1] && omap4_idle_ready_count)
			cpu_relax();
	}

out:
	cpu_pm_exit();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);
}

/**
 * omap4_enter_idle_secondary
 * @cpu: target cpu number
 *
 * Puts target cpu powerdomain into OFF.
 */
static void omap4_enter_idle_secondary(int cpu)
{
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu);

	cpu_pm_enter();

	pr_debug("%s: cpu1 down\n", __func__);
	flush_cache_all();
	dsb();

	/* TODO: merge CPU1 wakeup masks into CPU0 */
	omap_wakeupgen_irqmask_all(cpu, 1);
	gic_cpu_disable();

	if (!skip_off)
		omap4_enter_lowpower(cpu, PWRDM_POWER_OFF);

	omap_wakeupgen_irqmask_all(cpu, 0);
	gic_cpu_enable();

	pr_debug("%s: cpu1 up\n", __func__);

	cpu_pm_exit();

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu);
}

/**
 * omap4_enter_idle - Programs OMAP4 to enter the specified state
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework to program the device to the
 * specified low power state selected by the governor.
 * Called with irqs off, returns with irqs on.
 * Returns the amount of time spent in the low power state.
 */
static int omap4_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap4_processor_cx *cx = cpuidle_get_statedata(state);
	struct omap4_processor_cx *actual_cx;
	ktime_t preidle, postidle;
	bool idle = true;
	int cpu = dev->cpu;

	/*
	 * If disallow_smp_idle is set, revert to the old hotplug governor
	 * behavior
	 */
	if (dev->cpu != 0 && disallow_smp_idle)
		return omap4_enter_idle_wfi(dev, state);

	/* Clamp the power state at max_state */
	if (max_state > 0 && (cx->type > max_state - 1))
		cx = &omap4_power_states[max_state - 1];

	/*
	 * If only_state is set, use wfi if asking for a shallower idle state,
	 * or the specified state if asking for a deeper idle state
	 */
	if (only_state > 0) {
		if (cx->type < only_state - 1)
			cx = &omap4_power_states[OMAP4_STATE_C1];
		else
			cx = &omap4_power_states[only_state - 1];
	}

	if (cx->type == OMAP4_STATE_C1)
		return omap4_enter_idle_wfi(dev, state);

	preidle = ktime_get();

	local_fiq_disable();

	actual_cx = &omap4_power_states[OMAP4_STATE_C1];

	spin_lock(&omap4_idle_lock);
	omap4_cpu_update_state(cpu, cx);

	/* Wait for both cpus to be idle, exiting if an interrupt occurs */
	while (idle && !omap4_all_cpus_idle()) {
		spin_unlock(&omap4_idle_lock);
		idle = omap4_idle_wait();
		spin_lock(&omap4_idle_lock);
	}

	/*
	 * If we waited for longer than a millisecond, pop out to the governor
	 * to let it recalculate the desired state.
	 */
	if (ktime_to_us(ktime_sub(preidle, ktime_get())) > 1000)
		idle = false;

	if (!idle) {
		omap4_cpu_update_state(cpu, NULL);
		spin_unlock(&omap4_idle_lock);
		goto out;
	}

	/*
	 * If we go to sleep with an IPI pending, we will lose it.  Once we
	 * reach this point, the other cpu is either already idle or will
	 * shortly abort idle.  If it is already idle it can't send us an IPI,
	 * so it is safe to check for pending IPIs here.  If it aborts idle
	 * we will abort as well, and any future IPIs will be processed.
	 */
	if (omap4_gic_interrupt_pending()) {
		omap4_cpu_update_state(cpu, NULL);
		spin_unlock(&omap4_idle_lock);
		goto out;
	}

	/*
	 * Both cpus are probably idle.  There is a small chance the other cpu
	 * just became active.  cpu 0 will set omap4_idle_ready_count to 1,
	 * then each other cpu will increment it.  Once a cpu has incremented
	 * the count, it cannot abort idle and must spin until either the count
	 * has hit num_online_cpus(), or is reset to 0 by an aborting cpu.
	 */
	if (cpu == 0) {
		BUG_ON(omap4_idle_ready_count != 0);
		/* cpu0 requests shared-OFF */
		omap4_idle_ready_count = 1;
		/* cpu0 can no longer abort shared-OFF, but cpu1 can */

		/* wait for cpu1 to ack shared-OFF, or leave idle */
		while (omap4_idle_ready_count != num_online_cpus() &&
		    omap4_idle_ready_count != 0 && omap4_all_cpus_idle()) {
			spin_unlock(&omap4_idle_lock);
			cpu_relax();
			spin_lock(&omap4_idle_lock);
		}

		if (omap4_idle_ready_count != num_online_cpus() ||
		    !omap4_all_cpus_idle()) {
			pr_debug("%s: cpu1 aborted: %d %p\n", __func__,
				omap4_idle_ready_count,
				omap4_idle_requested_cx[1]);
			omap4_idle_ready_count = 0;
			omap4_cpu_update_state(cpu, NULL);
			spin_unlock(&omap4_idle_lock);
			goto out;
		}

		actual_cx = omap4_get_idle_state();
		spin_unlock(&omap4_idle_lock);

		/* cpu1 is turning itself off, continue with turning cpu0 off */

		omap4_enter_idle_primary(actual_cx);

		spin_lock(&omap4_idle_lock);
		omap4_idle_ready_count = 0;
		omap4_cpu_update_state(cpu, NULL);
		spin_unlock(&omap4_idle_lock);
	} else {
		/* wait for cpu0 to request the shared-OFF, or leave idle */
		while ((omap4_idle_ready_count == 0) && omap4_all_cpus_idle()) {
			spin_unlock(&omap4_idle_lock);
			cpu_relax();
			spin_lock(&omap4_idle_lock);
		}

		if (!omap4_all_cpus_idle()) {
			pr_debug("%s: cpu0 aborted: %d %p\n", __func__,
				omap4_idle_ready_count,
				omap4_idle_requested_cx[0]);
			omap4_cpu_update_state(cpu, NULL);
			spin_unlock(&omap4_idle_lock);
			goto out;
		}

		pr_debug("%s: cpu1 acks\n", __func__);
		/* ack shared-OFF */
		if (omap4_idle_ready_count > 0)
			omap4_idle_ready_count++;
		BUG_ON(omap4_idle_ready_count > num_online_cpus());

		while (omap4_idle_ready_count != num_online_cpus() &&
		    omap4_idle_ready_count != 0) {
			spin_unlock(&omap4_idle_lock);
			cpu_relax();
			spin_lock(&omap4_idle_lock);
		}

		if (omap4_idle_ready_count == 0) {
			pr_debug("%s: cpu0 aborted: %d %p\n", __func__,
				omap4_idle_ready_count,
				omap4_idle_requested_cx[0]);
			omap4_cpu_update_state(cpu, NULL);
			spin_unlock(&omap4_idle_lock);
			goto out;
		}

		/* cpu1 can no longer abort shared-OFF */

		actual_cx = omap4_get_idle_state();
		spin_unlock(&omap4_idle_lock);

		omap4_enter_idle_secondary(cpu);

		spin_lock(&omap4_idle_lock);
		omap4_idle_ready_count = 0;
		omap4_cpu_update_state(cpu, NULL);
		spin_unlock(&omap4_idle_lock);

		clkdm_allow_idle(cpu1_cd);

	}

out:
	postidle = ktime_get();

	omap4_update_actual_state(dev, actual_cx);

	local_irq_enable();
	local_fiq_enable();

	return ktime_to_us(ktime_sub(postidle, preidle));
}

DEFINE_PER_CPU(struct cpuidle_device, omap4_idle_dev);

/**
 * omap4_init_power_states - Initialises the OMAP4 specific C states.
 *
 * Below is the desciption of each C state.
 * C1 : CPUx wfi + MPU inative + Core inactive
 */
static void omap4_init_power_states(
	const struct cpuidle_params *cpuidle_params_table)
{
	/*
	 * C1 - CPUx WFI + MPU ON + CORE ON
	 */
	omap4_power_states[OMAP4_STATE_C1].valid =
			cpuidle_params_table[OMAP4_STATE_C1].valid;
	omap4_power_states[OMAP4_STATE_C1].type = OMAP4_STATE_C1;
	omap4_power_states[OMAP4_STATE_C1].exit_latency=
			cpuidle_params_table[OMAP4_STATE_C1].exit_latency;
	omap4_power_states[OMAP4_STATE_C1].target_residency =
			cpuidle_params_table[OMAP4_STATE_C1].target_residency;
	omap4_power_states[OMAP4_STATE_C1].desc = "CPU WFI";

	/*
	 * C2 - CPUx OFF + MPU INA + CORE INA
	 */
	omap4_power_states[OMAP4_STATE_C2].valid =
			cpuidle_params_table[OMAP4_STATE_C2].valid;
	omap4_power_states[OMAP4_STATE_C2].type = OMAP4_STATE_C2;
	omap4_power_states[OMAP4_STATE_C2].exit_latency =
			cpuidle_params_table[OMAP4_STATE_C2].exit_latency;
	omap4_power_states[OMAP4_STATE_C2].target_residency =
			cpuidle_params_table[OMAP4_STATE_C2].target_residency;
	omap4_power_states[OMAP4_STATE_C2].mpu_state = PWRDM_POWER_INACTIVE;
	omap4_power_states[OMAP4_STATE_C2].mpu_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C2].core_state = PWRDM_POWER_INACTIVE;
	omap4_power_states[OMAP4_STATE_C2].core_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C2].desc =
					"CPUs OFF, MPU INA + CORE INA";

	/*
	 * C3 - CPUx OFF + MPU CSWR + CORE OSWR
	 */
	omap4_power_states[OMAP4_STATE_C3].valid =
			cpuidle_params_table[OMAP4_STATE_C3].valid;
	omap4_power_states[OMAP4_STATE_C3].type = OMAP4_STATE_C3;
	omap4_power_states[OMAP4_STATE_C3].exit_latency =
			cpuidle_params_table[OMAP4_STATE_C3].exit_latency;
	omap4_power_states[OMAP4_STATE_C3].target_residency =
			cpuidle_params_table[OMAP4_STATE_C3].target_residency;
	omap4_power_states[OMAP4_STATE_C3].mpu_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].mpu_logic_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].core_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C3].core_logic_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C3].desc =
					"CPUs OFF, MPU CSWR + CORE OSWR";

	/*
	 * C4 - CPUx OFF + MPU OSWR + CORE OSWR
	 */
	omap4_power_states[OMAP4_STATE_C4].valid =
			cpuidle_params_table[OMAP4_STATE_C4].valid;
	omap4_power_states[OMAP4_STATE_C4].type = OMAP4_STATE_C4;
	omap4_power_states[OMAP4_STATE_C4].exit_latency =
			cpuidle_params_table[OMAP4_STATE_C4].exit_latency;
	omap4_power_states[OMAP4_STATE_C4].target_residency =
			cpuidle_params_table[OMAP4_STATE_C4].target_residency;
	omap4_power_states[OMAP4_STATE_C4].mpu_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C4].mpu_logic_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C4].core_state = PWRDM_POWER_RET;
	omap4_power_states[OMAP4_STATE_C4].core_logic_state = PWRDM_POWER_OFF;
	omap4_power_states[OMAP4_STATE_C4].desc =
					"CPUs OFF, MPU OSWR + CORE OSWR";

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
	const struct cpuidle_params *idle_params;

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	BUG_ON(!mpu_pd);
	cpu1_pd = pwrdm_lookup("cpu1_pwrdm");
	BUG_ON(!cpu1_pd);
	cpu1_cd = clkdm_lookup("mpu1_clkdm");
	BUG_ON(!cpu1_cd);
	core_pd = pwrdm_lookup("core_pwrdm");
	BUG_ON(!core_pd);

	if (cpu_is_omap443x())
		idle_params = omap443x_cpuidle_params_table;
	else if (cpu_is_omap446x())
		idle_params = omap446x_cpuidle_params_table;
	else
		idle_params = omap447x_cpuidle_params_table;

	omap4_init_power_states(idle_params);

	cpuidle_register_driver(&omap4_idle_driver);

	for_each_possible_cpu(cpu_id) {
		dev = &per_cpu(omap4_idle_dev, cpu_id);
		dev->cpu = cpu_id;
		count = 0;
		for (i = OMAP4_STATE_C1; i < OMAP4_MAX_STATES; i++) {
			cx = &omap4_power_states[i];
			state = &dev->states[count];

			if (!cx->valid)
				continue;
			cpuidle_set_statedata(state, cx);
			state->exit_latency = cx->exit_latency;
			state->target_residency = cx->target_residency;
			state->flags = CPUIDLE_FLAG_TIME_VALID;
			if (cx->type == OMAP4_STATE_C1) {
				dev->safe_state = state;
				state->enter = omap4_enter_idle_wfi;
			} else {
				state->enter = omap4_enter_idle;
			}

			sprintf(state->name, "C%d", count+1);
			strncpy(state->desc, cx->desc, CPUIDLE_DESC_LEN);
			count++;
		}

		if (!count)
			return -EINVAL;
		dev->state_count = count;

		if (cpuidle_register_device(dev)) {
			pr_err("%s: CPUidle register device failed\n", __func__);
			return -EIO;
		}

		__raw_writeb(BIT(cpu_id), omap4_get_gic_dist_base() +
			GIC_DIST_TARGET + omap4_poke_interrupt[cpu_id]);
	}

	return 0;
}
#else
int __init omap4_idle_init(void)
{
	return 0;
}
#endif /* CONFIG_CPU_IDLE */
