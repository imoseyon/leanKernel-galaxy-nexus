/*
 * LPDDR2 data as per JESD209-2
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

#include <mach/lpddr2-jedec.h>
#include <mach/emif.h>

/*
 * Organization and refresh requirements for LPDDR2 devices of different
 * types and densities. Derived from JESD209-2 section 2.4
 */
const struct lpddr2_addressing lpddr2_jedec_addressing_table[] = {
	/* Banks tREFIx10     rowx32,rowx16	 colx32,colx16	    density */
	{BANKS4, T_REFI_15_6, {ROW_12, ROW_12}, {COL_7, COL_8} },   /*64M*/
	{BANKS4, T_REFI_15_6, {ROW_12, ROW_12}, {COL_8, COL_9} },   /*128M*/
	{BANKS4, T_REFI_7_8,  {ROW_13, ROW_13}, {COL_8, COL_9} },   /*256M*/
	{BANKS4, T_REFI_7_8,  {ROW_13, ROW_13}, {COL_9, COL_10} },  /*512M*/
	{BANKS8, T_REFI_7_8,  {ROW_13, ROW_13}, {COL_9, COL_10} },  /*1GS4*/
	{BANKS8, T_REFI_3_9,  {ROW_14, ROW_14}, {COL_9, COL_10} },  /*2GS4*/
	{BANKS8, T_REFI_3_9,  {ROW_14, ROW_14}, {COL_10, COL_11} }, /*4G*/
	{BANKS8, T_REFI_3_9,  {ROW_15, ROW_15}, {COL_10, COL_11} }, /*8G*/
	{BANKS4, T_REFI_7_8,  {ROW_14, ROW_14}, {COL_9, COL_10} },  /*1GS2*/
	{BANKS4, T_REFI_3_9,  {ROW_15, ROW_15}, {COL_9, COL_10} },  /*2GS2*/
};

/*
 * Base AC Timing values specified by JESD209-2 for 400MHz operation
 * All devices will honour these timings at this frequency.
 * Some devices may have better timings. Using these timings is safe when the
 * timings are not available from the device data sheet.
 */
const struct lpddr2_timings lpddr2_jedec_timings_400_mhz = {
	.max_freq	= 400000000,
	.RL		= 6,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

/* Base AC Timing values specified by JESD209-2 for 333 MHz operation */
const struct lpddr2_timings lpddr2_jedec_timings_333_mhz = {
	.max_freq	= 333000000,
	.RL		= 5,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

/* Base AC Timing values specified by JESD209-2 for 200 MHz operation */
const struct lpddr2_timings lpddr2_jedec_timings_200_mhz = {
	.max_freq	= 200000000,
	.RL		= 3,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 20,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

/*
 * Min tCK values specified by JESD209-2
 * Min tCK specifies the minimum duration of some AC timing parameters in terms
 * of the number of cycles. If the calculated number of cycles based on the
 * absolute time value is less than the min tCK value, min tCK value should
 * be used instead. This typically happens at low frequencies.
 */
const struct lpddr2_min_tck lpddr2_jedec_min_tck = {
	.tRL		= 3,
	.tRP_AB		= 3,
	.tRCD		= 3,
	.tWR		= 3,
	.tRAS_MIN	= 3,
	.tRRD		= 2,
	.tWTR		= 2,
	.tXP		= 2,
	.tRTP		= 2,
	.tCKE		= 3,
	.tCKESR		= 3,
	.tFAW		= 8
};
