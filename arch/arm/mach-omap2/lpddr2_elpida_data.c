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

#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>

const struct lpddr2_timings lpddr2_elpida_timings_400_mhz = {
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

const struct lpddr2_timings lpddr2_elpida_timings_333_mhz = {
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

const struct lpddr2_timings lpddr2_elpida_timings_200_mhz = {
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

const struct lpddr2_min_tck lpddr2_elpida_min_tck = {
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

struct lpddr2_device_info lpddr2_elpida_2G_S4_dev = {
	.device_timings = {
		&lpddr2_elpida_timings_200_mhz,
		&lpddr2_elpida_timings_333_mhz,
		&lpddr2_elpida_timings_400_mhz
	},
	.min_tck	= &lpddr2_elpida_min_tck,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_2Gb,
	.io_width	= LPDDR2_IO_WIDTH_32
};
