/*
 * OMAP44xx EMIF header
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * Aneesh V <aneesh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _EMIF_H
#define _EMIF_H

#include <mach/emif-44xx.h>
#include <mach/lpddr2-jedec.h>

#define EMIF_NUM_INSTANCES 2
#define EMIF1	0
#define EMIF2	1

/* The maximum frequency at which the LPDDR2 interface can operate in Hz*/
#define MAX_LPDDR2_FREQ	400000000	/* 400 MHz */

/* 19.2 MHz to be used for finding initialization values */
#define EMIF_FREQ_19_2_MHZ 19200000	/* 19.2 MHz */
/*
 * The period of DDR clk is represented as numerator and denominator for
 * better accuracy in integer based calculations. However, if the numerator
 * and denominator are very huge there may be chances of overflow in
 * calculations. So, as a trade-off keep denominator(and consequently
 * numerator) within a limit sacrificing some accuracy - but not much
 * If denominator and numerator are already small (such as at 400 MHz)
 * no adjustment is needed
 */
#define EMIF_PERIOD_DEN_LIMIT	1000
/*
 * Maximum number of different frequencies supported by EMIF driver
 * Determines the number of entries in the pointer array for register
 * cache
 */
#define EMIF_MAX_NUM_FREQUENCIES	6
/*
 * Indices into the Addressing Table array.
 * One entry each for all the different types of devices with different
 * addressing schemes
 */
#define ADDR_TABLE_INDEX64M	0
#define ADDR_TABLE_INDEX128M	1
#define ADDR_TABLE_INDEX256M	2
#define ADDR_TABLE_INDEX512M	3
#define ADDR_TABLE_INDEX1GS4	4
#define ADDR_TABLE_INDEX2GS4	5
#define ADDR_TABLE_INDEX4G	6
#define ADDR_TABLE_INDEX8G	7
#define ADDR_TABLE_INDEX1GS2	8
#define ADDR_TABLE_INDEX2GS2	9
#define ADDR_TABLE_INDEXMAX	10

/* Number of Row bits */
#define ROW_9  0
#define ROW_10 1
#define ROW_11 2
#define ROW_12 3
#define ROW_13 4
#define ROW_14 5
#define ROW_15 6
#define ROW_16 7

/* Number of Column bits */
#define COL_8   0
#define COL_9   1
#define COL_10  2
#define COL_11  3
#define COL_7   4 /*Not supported by OMAP included for completeness */

/* Number of Banks*/
#define BANKS1 0
#define BANKS2 1
#define BANKS4 2
#define BANKS8 3

/* Refresh rate in micro seconds x 10 */
#define T_REFI_15_6	156
#define T_REFI_7_8	78
#define T_REFI_3_9	39

#define EBANK_CS1_DIS	0
#define EBANK_CS1_EN	1

/* Read Latency at the base frequency - 19.2 MHz on bootup */
#define RL_19_2_MHZ	3
/* Interleaving policies at EMIF level- between banks and Chip Selects */
#define EMIF_INTERLEAVING_POLICY_MAX_INTERLEAVING	0
#define EMIF_INTERLEAVING_POLICY_NO_BANK_INTERLEAVING	3

/*
 * Interleaving policy to be used
 * Currently set to MAX interleaving for better performance
 */
#define EMIF_INTERLEAVING_POLICY EMIF_INTERLEAVING_POLICY_MAX_INTERLEAVING

/* State of the core voltage:
 * This is important for some parameters such as read idle control and
 * ZQ calibration timings. Timings are much stricter when voltage ramp
 * is happening compared to when the voltage is stable.
 * We need to calculate two sets of values for these parameters and use
 * them accordingly
 */
#define LPDDR2_VOLTAGE_STABLE	0
#define LPDDR2_VOLTAGE_RAMPING	1

/* Length of the forced read idle period in terms of cycles */
#define EMIF_REG_READ_IDLE_LEN_VAL	5

/* Interval between forced 'read idles' */
/* To be used when voltage is changed for DPS/DVFS - 1us */
#define READ_IDLE_INTERVAL_DVFS		(1*1000)
/*
 * To be used when voltage is not scaled except by Smart Reflex
 * 50us - or maximum value will do
 */
#define READ_IDLE_INTERVAL_NORMAL	(50*1000)


/*
 * Unless voltage is changing due to DVFS one ZQCS command every 50ms should
 * be enough. This shoule be enough also in the case when voltage is changing
 * due to smart-reflex.
 */
#define EMIF_ZQCS_INTERVAL_NORMAL_IN_US	(50*1000)
/*
 * If voltage is changing due to DVFS ZQCS should be performed more
 * often(every 50us)
 */
#define EMIF_ZQCS_INTERVAL_DVFS_IN_US	50

/* The interval between ZQCL commands as a multiple of ZQCS interval */
#define REG_ZQ_ZQCL_MULT		4
/* The interval between ZQINIT commands as a multiple of ZQCL interval */
#define REG_ZQ_ZQINIT_MULT		3
/* Enable ZQ Calibration on exiting Self-refresh */
#define REG_ZQ_SFEXITEN_ENABLE		1
/*
 * ZQ Calibration simultaneously on both chip-selects:
 * Needs one calibration resistor per CS
 * None of the boards that we know of have this capability
 * So disabled by default
 */
#define REG_ZQ_DUALCALEN_DISABLE	0
/*
 * Enable ZQ Calibration by default on CS0. If we are asked to program
 * the EMIF there will be something connected to CS0 for sure
 */
#define REG_ZQ_CS0EN_ENABLE		1

/* EMIF_PWR_MGMT_CTRL register */
/* Low power modes */
#define LP_MODE_DISABLE		0
#define LP_MODE_CLOCK_STOP	1
#define LP_MODE_SELF_REFRESH	2
#define LP_MODE_PWR_DN		3

/* REG_DPD_EN */
#define DPD_DISABLE	0
#define DPD_ENABLE	1

/*
 * Value of bits 12:31 of DDR_PHY_CTRL_1 register:
 * All these fields have magic values dependent on frequency and
 * determined by PHY and DLL integration with EMIF. Setting the magic
 * values suggested by hw team.
 */
#define EMIF_DDR_PHY_CTRL_1_BASE_VAL			0x049FF
#define EMIF_DLL_SLAVE_DLY_CTRL_400_MHZ			0x41
#define EMIF_DLL_SLAVE_DLY_CTRL_200_MHZ			0x80
#define EMIF_DLL_SLAVE_DLY_CTRL_100_MHZ_AND_LESS	0xFF

/*
* MR1 value:
* Burst length	: 8
* Burst type	: sequential
* Wrap		: enabled
* nWR		: 3(default). EMIF does not do pre-charge.
*		: So nWR is don't care
*/
#define MR1_VAL	0x23

/* MR10: ZQ calibration codes */
#define MR10_ZQ_ZQCS		0x56
#define MR10_ZQ_ZQCL		0xAB
#define MR10_ZQ_ZQINIT		0xFF
#define MR10_ZQ_ZQRESET		0xC3

/* TEMP_ALERT_CONFIG */
#define TEMP_ALERT_POLL_INTERVAL_MS	360 /* for temp gradient - 5 C/s */
#define TEMP_ALERT_CONFIG_DEVCT_1	0
#define TEMP_ALERT_CONFIG_DEVWDT_32	2

/* MR16 value: refresh full array(no partial array self refresh) */
#define MR16_VAL	0

#if defined(DEBUG)
#define emif_assert(c)	BUG_ON(!(c))
#else
#define emif_assert(c)	({ if (0) BUG_ON(!(c)); 0; })
#endif

/* Details of the devices connected to each chip-select of an EMIF instance */
struct emif_device_details {
	const struct lpddr2_device_info *cs0_device;
	const struct lpddr2_device_info *cs1_device;
};

/*
 * LPDDR2 interface clock frequency:
 * Period (represented as numerator and denominator for better accuracy in
 * calculations) should be <= the real value. Period is used for calculating
 * all timings except refresh rate.
 * freq_mhz_floor - freq in mhz truncated to the lower integer is used for
 * calculating refresh rate
 * freq_mhz_ceil - frequency in mhz rounded up is used for identifying the
 * right speed bin and the corresponding timings table for the LPDDR2 device
 */
struct freq_info {
	u16 period_num;
	u16 period_den;
	u16 freq_mhz_floor;
	u16 freq_mhz_ceil;
};

/*
 * Structure containing shadow of important registers in EMIF
 * The calculation function fills in this structure to be later used for
 * initialization and DVFS
 */
struct emif_regs {
	u32 freq;
	u8 RL_final;
	u32 sdram_config_init;
	u32 sdram_config_final;
	u32 ref_ctrl;
	u32 ref_ctrl_derated;
	u32 sdram_tim1;
	u32 sdram_tim1_derated;
	u32 sdram_tim2;
	u32 sdram_tim3;
	u32 read_idle_ctrl_normal;
	u32 read_idle_ctrl_volt_ramp;
	u32 zq_config_normal;
	u32 zq_config_volt_ramp;
	u32 temp_alert_config;
	u32 temp_alert_config_derated;
	u32 emif_ddr_phy_ctlr_1_init;
	u32 emif_ddr_phy_ctlr_1_final;
};

int omap_emif_setup_registers(u32 freq,
			      u32 volt_state);
int omap_emif_setup_device_details(
			const struct emif_device_details *emif1_devices,
			const struct emif_device_details *emif2_devices);

void emif_clear_irq(int emif_id);
#endif
