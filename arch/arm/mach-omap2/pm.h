/*
 * OMAP2/3 Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H

#include <linux/err.h>

#include "powerdomain.h"

extern void *omap3_secure_ram_storage;
extern void omap3_pm_off_mode_enable(int);
extern void omap_sram_idle(bool suspend);
extern int omap3_can_sleep(void);
extern int omap_set_pwrdm_state(struct powerdomain *pwrdm, u32 state);
extern int omap3_idle_init(void);
extern int omap4_idle_init(void);
extern void omap4_enter_sleep(unsigned int cpu, unsigned int power_state,
				bool suspend);
extern void omap4_trigger_ioctrl(void);
extern u32 omap4_device_off_counter;

#ifdef CONFIG_PM
extern void omap4_device_set_state_off(u8 enable);
extern bool omap4_device_prev_state_off(void);
extern bool omap4_device_next_state_off(void);
extern void omap4_device_clear_prev_off_state(void);
#else
static inline void omap4_device_set_state_off(u8 enable)
{
}
static inline bool omap4_device_prev_state_off(void)
{
	return false;
}
static inline bool omap4_device_next_state_off(void)
{
	return false;
}
#endif

#if defined(CONFIG_PM_OPP)
extern int omap3_opp_init(void);
extern int omap4_opp_init(void);
#else
static inline int omap3_opp_init(void)
{
	return -EINVAL;
}
static inline int omap4_opp_init(void)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_PM
int omap4_pm_cold_reset(char *reason);
#else
int omap4_pm_cold_reset(char *reason)
{
	return -EINVAL;
}
#endif

/*
 * cpuidle mach specific parameters
 *
 * The board code can override the default C-states definition using
 * omap3_pm_init_cpuidle
 */
struct cpuidle_params {
	u32 exit_latency;	/* exit_latency = sleep + wake-up latencies */
	u32 target_residency;
	u8 valid;		/* validates the C-state */
};

#if defined(CONFIG_PM) && defined(CONFIG_CPU_IDLE)
extern void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params);
#else
static
inline void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params)
{
}
#endif

extern int omap3_pm_get_suspend_state(struct powerdomain *pwrdm);
extern int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state);

extern u32 wakeup_timer_seconds;
extern u32 wakeup_timer_milliseconds;
extern struct omap_dm_timer *gptimer_wakeup;

#ifdef CONFIG_PM_DEBUG
extern void omap2_pm_dump(int mode, int resume, unsigned int us);
extern void omap2_pm_wakeup_on_timer(u32 seconds, u32 milliseconds);
extern int omap2_pm_debug;
extern u32 sleep_while_idle;
#else
#define omap2_pm_dump(mode, resume, us)		do {} while (0);
#define omap2_pm_wakeup_on_timer(seconds, milliseconds)	do {} while (0);
#define omap2_pm_debug				0
#define sleep_while_idle 0
#endif
#ifdef CONFIG_PM_ADVANCED_DEBUG
extern void omap4_pm_suspend_save_regs(void);
#else
static inline void omap4_pm_suspend_save_regs(void) { }
#endif

#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
extern int pm_dbg_regset_save(int reg_set);
extern int pm_dbg_regset_init(int reg_set);
#else
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#define pm_dbg_regset_save(reg_set) do {} while (0);
#define pm_dbg_regset_init(reg_set) do {} while (0);
#endif /* CONFIG_PM_DEBUG */

extern void omap24xx_idle_loop_suspend(void);

extern void omap24xx_cpu_suspend(u32 dll_ctrl, void __iomem *sdrc_dlla_ctrl,
					void __iomem *sdrc_power);
extern void omap34xx_cpu_suspend(u32 *addr, int save_state);
extern int save_secure_ram_context(u32 *addr);
extern void omap3_save_scratchpad_contents(void);

extern unsigned int omap24xx_idle_loop_suspend_sz;
extern unsigned int save_secure_ram_context_sz;
extern unsigned int omap24xx_cpu_suspend_sz;
extern unsigned int omap34xx_cpu_suspend_sz;

#define PM_RTA_ERRATUM_i608		(1 << 0)
#define PM_SDRC_WAKEUP_ERRATUM_i583	(1 << 1)

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3)
extern u16 pm34xx_errata;
#define IS_PM34XX_ERRATUM(id)		(pm34xx_errata & (id))
extern void enable_omap3630_toggle_l2_on_restore(void);
#else
#define IS_PM34XX_ERRATUM(id)		0
static inline void enable_omap3630_toggle_l2_on_restore(void) { }
#endif		/* defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3) */

#ifdef CONFIG_OMAP_SMARTREFLEX
extern int omap_devinit_smartreflex(void);
extern void omap_enable_smartreflex_on_init(void);
#else
static inline int omap_devinit_smartreflex(void)
{
	return -EINVAL;
}

static inline void omap_enable_smartreflex_on_init(void) {}
#endif

/**
 * struct omap_pmic_map - Describe the OMAP PMIC data for OMAP
 * @name:	name of the voltage domain
 * @pmic_data:	pmic data associated with it
 * @omap_chip:	initialize with OMAP_CHIP_INIT the OMAP chips this data maps to
 * @special_action: callback for any specific action to take for that map
 *
 * Since we support multiple PMICs each potentially functioning on multiple
 * OMAP devices, we describe the parameters in a map allowing us to reuse the
 * data as necessary.
 */
struct omap_pmic_map {
	char			*name;
	struct omap_voltdm_pmic	*pmic_data;
	struct omap_chip_id	omap_chip;
	int			(*special_action)(struct voltagedomain *);
};

/**
 * struct omap_pmic_description - Describe low power behavior of the PMIC
 * @pmic_lp_tshut:		Time rounded up to uSec for the PMIC to
 *				go to low power after the LDOs are pulled to
 *				appropriate state. Note: this is not the same as
 *				voltage rampdown time, instead, consider the
 *				PMIC to have switched it's LDOs down, this is
 *				time taken to reach it's lowest power state(say
 *				sleep/OFF).
 * @pmic_lp_tstart:		Time rounded up to uSec for the PMIC to
 *				provide be ready for operation from low power
 *				state. Note: this is not the same as voltage
 *				rampup time, instead, consider the PMIC to be
 *				in lowest power state(say OFF), this is the time
 *				required for it to become ready for it's DCDCs
 *				or LDOs to start operation.
 */
struct omap_pmic_description {
	u32 pmic_lp_tshut;
	u32 pmic_lp_tstart;
};

#ifdef CONFIG_PM
extern int omap_pmic_register_data(struct omap_pmic_map *map,
				   struct omap_pmic_description *desc);
#else
static inline int omap_pmic_register_data(struct omap_pmic_map *map,
				   struct omap_pmic_description *desc)
{
	return -EINVAL;
}
#endif
extern void omap_pmic_data_init(void);

extern int omap_pmic_update(struct omap_pmic_map *tmp_map, char *name,
		u32 old_chip_id, u32 new_chip_id);

#ifdef CONFIG_TWL4030_CORE
extern int omap_twl_init(void);
extern int omap3_twl_set_sr_bit(bool enable);
extern int omap_twl_pmic_update(char *name, u32 old_chip_id, u32 new_chip_id);
#else
static inline int omap_twl_init(void)
{
	return -EINVAL;
}
static inline int omap_twl_pmic_update(char *name, u32 old_chip_id,
	u32 new_chip_id)
{
	return -EINVAL;
}
#endif

#ifdef CONFIG_OMAP_TPS6236X
extern int omap_tps6236x_board_setup(bool use_62361, int gpio_vsel0,
	                int gpio_vsel1, int pull0, int pull1);
extern int omap_tps6236x_init(void);

extern int omap_tps6236x_update(char *name, u32 old_chip_id, u32 new_chip_id);
#else
static inline int omap_tps6236x_board_setup(bool use_62361, int gpio_vsel0,
	                int gpio_vsel1, int pull0, int pull1)
{
	return -EINVAL;
}
static inline int omap_tps6236x_init(void)
{
	return -EINVAL;
}
static inline int omap_tps6236x_update(char *name, u32 old_chip_id,
	u32 new_chip_id)
{
	return -EINVAL;
}
#endif

extern int omap4_ldo_trim_configure(void);

#ifdef CONFIG_PM
extern bool omap_pm_is_ready_status;
/**
 * omap_pm_is_ready() - tells if OMAP pm framework is done it's initialization
 *
 * In few cases, to sequence operations properly, we'd like to know if OMAP's PM
 * framework has completed all it's expected initializations.
 */
static inline bool omap_pm_is_ready(void)
{
	return omap_pm_is_ready_status;
}
extern int omap_pm_get_osc_lp_time(u32 *tstart, u32 *tshut);
extern int omap_pm_get_pmic_lp_time(u32 *tstart, u32 *tshut);
extern void omap_pm_set_osc_lp_time(u32 tstart, u32 tshut);
extern void omap_pm_set_pmic_lp_time(u32 tstart, u32 tshut);
#else
static inline bool omap_pm_is_ready(void)
{
	return false;
}
static inline int omap_pm_get_osc_lp_time(u32 *tstart, u32 *tshut)
{
	return -EINVAL;
}
static inline int omap_pm_get_pmic_lp_time(u32 *tstart, u32 *tshut)
{
	return -EINVAL;
}
static inline void omap_pm_set_osc_lp_time(u32 tstart, u32 tshut) { }
static inline void omap_pm_set_pmic_lp_time(u32 tstart, u32 tshut) { }
#endif
#endif
