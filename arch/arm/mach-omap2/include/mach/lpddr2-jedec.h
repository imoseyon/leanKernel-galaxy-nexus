/*
 * LPDDR2 header based on JESD209-2
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Aneesh V <aneesh@ti.com>
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LPDDR2_JDEC_H
#define _LPDDR2_JDEC_H

#include <linux/types.h>

/*
 * Maximum number of entries we keep in our array of timing tables
 * We need not keep all the speed bins supported by the device
 * We need to keep timing tables for only the speed bins that we
 * are interested in
 */
#define MAX_NUM_SPEEDBINS	4

/* LPDDR2 Densities */
#define LPDDR2_DENSITY_64Mb	0
#define LPDDR2_DENSITY_128Mb	1
#define LPDDR2_DENSITY_256Mb	2
#define LPDDR2_DENSITY_512Mb	3
#define LPDDR2_DENSITY_1Gb	4
#define LPDDR2_DENSITY_2Gb	5
#define LPDDR2_DENSITY_4Gb	6
#define LPDDR2_DENSITY_8Gb	7
#define LPDDR2_DENSITY_16Gb	8
#define LPDDR2_DENSITY_32Gb	9

/* LPDDR2 type */
#define	LPDDR2_TYPE_S4	0
#define	LPDDR2_TYPE_S2	1
#define	LPDDR2_TYPE_NVM	2

/* LPDDR2 IO width */
#define	LPDDR2_IO_WIDTH_32	0
#define	LPDDR2_IO_WIDTH_16	1
#define	LPDDR2_IO_WIDTH_8	2

/* Mode register numbers */
#define LPDDR2_MR0	0
#define LPDDR2_MR1	1
#define LPDDR2_MR2	2
#define LPDDR2_MR3	3
#define LPDDR2_MR4	4
#define LPDDR2_MR5	5
#define LPDDR2_MR6	6
#define LPDDR2_MR7	7
#define LPDDR2_MR8	8
#define LPDDR2_MR9	9
#define LPDDR2_MR10	10
#define LPDDR2_MR11	11
#define LPDDR2_MR16	16
#define LPDDR2_MR17	17
#define LPDDR2_MR18	18

/* MR4 register fields */
#define MR4_SDRAM_REF_RATE_SHIFT	0
#define MR4_SDRAM_REF_RATE_MASK		7
#define MR4_TUF_SHIFT			7
#define MR4_TUF_MASK			(1 << 7)

/* MR4 SDRAM Refresh Rate field values */
#define SDRAM_TEMP_NOMINAL				0x3
#define SDRAM_TEMP_RESERVED_4				0x4
#define SDRAM_TEMP_HIGH_DERATE_REFRESH			0x5
#define SDRAM_TEMP_HIGH_DERATE_REFRESH_AND_TIMINGS	0x6
#define SDRAM_TEMP_VERY_HIGH_SHUTDOWN			0x7

struct lpddr2_addressing {
	u8	num_banks;
	u8	t_REFI_us_x10;
	u8	row_sz[2]; /* One entry each for x32 and x16 */
	u8	col_sz[2]; /* One entry each for x32 and x16 */
};

/* Structure for timings from the DDR datasheet */
struct lpddr2_timings {
	u32 max_freq;
	u8 RL;
	u8 tRPab;
	u8 tRCD;
	u8 tWR;
	u8 tRASmin;
	u8 tRRD;
	u8 tWTRx2;
	u8 tXSR;
	u8 tXPx2;
	u8 tRFCab;
	u8 tRTPx2;
	u8 tCKE;
	u8 tCKESR;
	u8 tZQCS;
	u32 tZQCL;
	u32 tZQINIT;
	u8 tDQSCKMAXx2;
	u8 tRASmax;
	u8 tFAW;
};

/*
 * Min tCK values for some of the parameters:
 * If the calculated clock cycles for the respective parameter is
 * less than the corresponding min tCK value, we need to set the min
 * tCK value. This may happen at lower frequencies.
 */
struct lpddr2_min_tck {
	u32 tRL;
	u32 tRP_AB;
	u32 tRCD;
	u32 tWR;
	u32 tRAS_MIN;
	u32 tRRD;
	u32 tWTR;
	u32 tXP;
	u32 tRTP;
	u8  tCKE;
	u32 tCKESR;
	u32 tFAW;
};

struct lpddr2_device_info {
	const struct lpddr2_timings *device_timings[MAX_NUM_SPEEDBINS];
	const struct lpddr2_min_tck *min_tck;
	u8	type;
	u8	density;
	u8	io_width;

	/* Idle time in cycles to wait before putting the memory in self refresh */
	s32 emif_ddr_selfrefresh_cycles;
};

/* The following are exported for devices which use JDEC specifications */
extern const struct lpddr2_addressing lpddr2_jedec_addressing_table[];
extern const struct lpddr2_timings lpddr2_jedec_timings_400_mhz;
extern const struct lpddr2_timings lpddr2_jedec_timings_333_mhz;
extern const struct lpddr2_timings lpddr2_jedec_timings_200_mhz;
extern const struct lpddr2_min_tck lpddr2_jedec_min_tck;

#endif
