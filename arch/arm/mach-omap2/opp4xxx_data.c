/*
 * OMAP4 OPP table definitions.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *	Kevin Hilman
 *	Thara Gopinath
 * Copyright (C) 2010-2011 Nokia Corporation.
 *      Eduardo Valentin
 *      Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/opp.h>

#include <plat/cpu.h>
#include <plat/common.h>

#include "control.h"
#include "omap_opp_data.h"
#include "pm.h"

/*
 * Structures containing OMAP4430 voltage supported and various
 * voltage dependent data for each VDD.
 */

#define OMAP4430_VDD_MPU_OPP50_UV		1025000
#define OMAP4430_VDD_MPU_OPP100_UV		1200000
#define OMAP4430_VDD_MPU_OPPTURBO_UV		1325000
#define OMAP4430_VDD_MPU_OPPNITRO_UV		1388000

struct omap_volt_data omap443x_vdd_mpu_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_MPU_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

#define OMAP4430_VDD_IVA_OPP50_UV		 950000
#define OMAP4430_VDD_IVA_OPP100_UV		1114000
#define OMAP4430_VDD_IVA_OPPTURBO_UV		1291000

struct omap_volt_data omap443x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4430_VDD_IVA_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

#define OMAP4430_VDD_CORE_OPP50_UV		 962000
#define OMAP4430_VDD_CORE_OPP100_UV		1127000

struct omap_volt_data omap443x_vdd_core_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP50_UV, 0, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(OMAP4430_VDD_CORE_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

/* Dependency of domains are as follows for OMAP4430 (OPP based):
 *
 *	MPU	IVA	CORE
 *	50	50	50+
 *	50	100+	100
 *	100+	50	100
 *	100+	100+	100
 */

/* OMAP 4430 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_mpu_core_dep_data[] = {
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap443x_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_mpu_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_mpu_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4430 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap443x_vdd_iva_core_dep_data[] = {
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4430_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4430_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap443x_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap443x_vdd_iva_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap443x_vdd_iva_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap443x_opp_def_list[] = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 300000000, OMAP4430_VDD_MPU_OPP50_UV),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 600000000, OMAP4430_VDD_MPU_OPP100_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 800000000, OMAP4430_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP4 - OPP-SB */
	OPP_INITIALIZER("mpu", "dpll_mpu_ck", "mpu", true, 1008000000, OMAP4430_VDD_MPU_OPPNITRO_UV),
	/* L3 OPP1 - OPP50 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100, OPP-Turbo, OPP-SB */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* IVA OPP1 - OPP50 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4430_VDD_IVA_OPP50_UV),
	/* IVA OPP2 - OPP100 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4430_VDD_IVA_OPP100_UV),
	/* IVA OPP3 - OPP-Turbo */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 332000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4430_VDD_CORE_OPP50_UV),
	/* SGX OPP2 - OPP100 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4430_VDD_CORE_OPP100_UV),
	/* FDIF OPP1 - OPP25 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP2 - OPP50 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* FDIF OPP3 - OPP100 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* DSP OPP1 - OPP50 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4430_VDD_IVA_OPP50_UV),
	/* DSP OPP2 - OPP100 */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4430_VDD_IVA_OPP100_UV),
	/* DSP OPP3 - OPPTB */
	OPP_INITIALIZER("dsp", "dpll_iva_m4x2_ck", "iva", false, 496000000, OMAP4430_VDD_IVA_OPPTURBO_UV),
	/* HSI OPP1 - OPP50 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4430_VDD_CORE_OPP50_UV),
	/* HSI OPP2 - OPP100 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4430_VDD_CORE_OPP100_UV),
	/* ABE OPP1 - OPP50 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4430_VDD_IVA_OPP50_UV),
	/* ABE OPP2 - OPP100 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4430_VDD_IVA_OPP100_UV),
};

#define OMAP4460_VDD_MPU_OPP50_UV		975000
#define OMAP4460_VDD_MPU_OPP100_UV		1150000
#define OMAP4460_VDD_MPU_OPPTURBO_UV		1250000
#define OMAP4460_VDD_MPU_OPPNITRO_UV		1275000
#define OMAP4460_VDD_MPU_OPPNITRO_UV2		1300000
#define OMAP4460_VDD_MPU_OPPOC_UV		1350000
#define OMAP4460_VDD_MPU_OPPOC_UV2		1450000

struct omap_volt_data omap446x_vdd_mpu_volt_data[] = {
        VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP50_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
        VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
        VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_MPU_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
        VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITRO_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPNITRO_UV2, 20000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITRO, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
 	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPOC_UV, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
 	VOLT_DATA_DEFINE(OMAP4460_VDD_MPU_OPPOC_UV2, 10000, OMAP44XX_CONTROL_FUSE_MPU_OPPNITROSB2, 0xfa, 0x27, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

#define OMAP4460_VDD_IVA_OPP50_UV		 950000
#define OMAP4460_VDD_IVA_OPP100_UV		1140000
#define OMAP4460_VDD_IVA_OPPTURBO_UV		1291000
#define OMAP4460_VDD_IVA_OPPNITRO_UV		1375000

struct omap_volt_data omap446x_vdd_iva_volt_data[] = {
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP50_UV, 13000, OMAP44XX_CONTROL_FUSE_IVA_OPP50, 0xf4, 0x0c, OMAP_ABB_NOMINAL_OPP),
	VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPP100_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPP100, 0xf9, 0x16, OMAP_ABB_NOMINAL_OPP),
        VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPTURBO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPTURBO, 0xfa, 0x23, OMAP_ABB_NOMINAL_OPP),
        VOLT_DATA_DEFINE(OMAP4460_VDD_IVA_OPPNITRO_UV, 0, OMAP44XX_CONTROL_FUSE_IVA_OPPNITRO, 0xfa, 0x23, OMAP_ABB_FAST_OPP),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

#define OMAP4460_VDD_CORE_OPP50_UV		 962000
#define OMAP4460_VDD_CORE_OPP100_UV		1127000
#define OMAP4460_VDD_CORE_OPP100_OV_UV		1127000

struct omap_volt_data omap446x_vdd_core_volt_data[] = {
        VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP50_UV, 38000, OMAP44XX_CONTROL_FUSE_CORE_OPP50, 0xf4, 0x0c, OMAP_ABB_NONE),
        VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_UV, 13000, OMAP44XX_CONTROL_FUSE_CORE_OPP100, 0xf9, 0x16, OMAP_ABB_NONE),
        VOLT_DATA_DEFINE(OMAP4460_VDD_CORE_OPP100_OV_UV, 13000, OMAP44XX_CONTROL_FUSE_CORE_OPP100OV, 0xf9, 0x16, OMAP_ABB_NONE),
	VOLT_DATA_DEFINE(0, 0, 0, 0, 0, 0),
};

/* OMAP 4460 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_mpu_core_dep_data[] = {
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITRO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
 	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPNITRO_UV2, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPOC_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
 	{.main_vdd_volt = OMAP4460_VDD_MPU_OPPOC_UV2, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV}
};

struct omap_vdd_dep_info omap446x_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap446x_vdd_mpu_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap446x_vdd_mpu_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

/* OMAP 4460 MPU IVA VDD dependency table */
static struct omap_vdd_dep_volt omap446x_vdd_iva_core_dep_data[] = {
	{.main_vdd_volt = OMAP4460_VDD_IVA_OPP50_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP50_UV},
	{.main_vdd_volt = OMAP4460_VDD_IVA_OPP100_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
        {.main_vdd_volt = OMAP4460_VDD_IVA_OPPTURBO_UV, .dep_vdd_volt = OMAP4460_VDD_CORE_OPP100_UV},
};

struct omap_vdd_dep_info omap446x_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap446x_vdd_iva_core_dep_data,
		.nr_dep_entries = ARRAY_SIZE(omap446x_vdd_iva_core_dep_data),
	},
	{.name = NULL, .dep_table = NULL, .nr_dep_entries = 0},
};

static struct omap_opp_def __initdata omap446x_opp_def_list[] = {
	/* MPU OPP1 - OPP50 */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 350000000, OMAP4460_VDD_MPU_OPP50_UV),
	/* MPU OPP2 - OPP100 */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 700000000, OMAP4460_VDD_MPU_OPP100_UV),
	/* MPU OPP3 - OPP-Turbo */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", true, 920000000, OMAP4460_VDD_MPU_OPPTURBO_UV),
	/* MPU OPP4 - OPP-Nitro */
	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1200000000, OMAP4460_VDD_MPU_OPPNITRO_UV),
	/* MPU OPP4 - OPP-Nitro SpeedBin */
 	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1350000000, OMAP4460_VDD_MPU_OPPNITRO_UV2),
 	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1520000000, OMAP4460_VDD_MPU_OPPOC_UV),
 	OPP_INITIALIZER("mpu", "virt_dpll_mpu_ck", "mpu", false, 1650000000, OMAP4460_VDD_MPU_OPPOC_UV2),
	/* L3 OPP1 - OPP50 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 100000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* L3 OPP2 - OPP100 */
	OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4460_VDD_CORE_OPP100_UV),
        OPP_INITIALIZER("l3_main_1", "virt_l3_ck", "core", true, 200000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
	/* IVA OPP1 - OPP50 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 133000000, OMAP4460_VDD_IVA_OPP50_UV),
	/* IVA OPP2 - OPP100 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", true, 266100000, OMAP4460_VDD_IVA_OPP100_UV),
	/*
	 * IVA OPP3 - OPP-Turbo + Disabled as the reference schematics
	 * recommends Phoenix VCORE2 which can supply only 600mA - so the ones
	 * above this OPP frequency, even though OMAP is capable, should be
	 * enabled by board file which is sure of the chip power capability
	 */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 332000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
	/* IVA OPP4 - OPP-Nitro */
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 430000000, OMAP4460_VDD_IVA_OPPNITRO_UV),
	/* IVA OPP5 - OPP-Nitro SpeedBin*/
	OPP_INITIALIZER("iva", "dpll_iva_m5x2_ck", "iva", false, 500000000, OMAP4460_VDD_IVA_OPPNITRO_UV),

	/* SGX OPP1 - OPP50 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 153600000, OMAP4460_VDD_CORE_OPP50_UV),
	/* SGX OPP2 - OPP100 */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", true, 307200000, OMAP4460_VDD_CORE_OPP100_UV),
	/* SGX OPP3 - OPPOV */
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", false, 384000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
	OPP_INITIALIZER("gpu", "dpll_per_m7x2_ck", "core", false, 512000000, OMAP4460_VDD_CORE_OPP100_OV_UV),
	/* FDIF OPP1 - OPP25 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 32000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* FDIF OPP2 - OPP50 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 64000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* FDIF OPP3 - OPP100 */
	OPP_INITIALIZER("fdif", "fdif_fck", "core", true, 128000000, OMAP4460_VDD_CORE_OPP100_UV),
	/* DSP OPP1 - OPP50 */
	OPP_INITIALIZER("dsp_c0", "dpll_iva_m4x2_ck", "iva", true, 232750000, OMAP4460_VDD_IVA_OPP50_UV),
	/* DSP OPP2 - OPP100 */
	OPP_INITIALIZER("dsp_c0", "dpll_iva_m4x2_ck", "iva", true, 465500000, OMAP4460_VDD_IVA_OPP100_UV),
        OPP_INITIALIZER("dsp_c0", "dpll_iva_m4x2_ck", "iva", false, 496000000, OMAP4460_VDD_IVA_OPPTURBO_UV),
	/* HSI OPP1 - OPP50 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4460_VDD_CORE_OPP50_UV),
	/* HSI OPP2 - OPP100 */
	OPP_INITIALIZER("hsi", "hsi_fck", "core", true, 96000000, OMAP4460_VDD_CORE_OPP100_UV),
	/* ABE OPP1 - OPP50 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 98304000, OMAP4460_VDD_IVA_OPP50_UV),
	/* ABE OPP2 - OPP100 */
	OPP_INITIALIZER("aess", "abe_clk", "iva", true, 196608000, OMAP4460_VDD_IVA_OPP100_UV),
};

/**
 * omap4_opp_enable() - helper to enable the OPP
 * @oh_name: name of the hwmod device
 * @freq:	frequency to enable
 */
static void __init omap4_opp_enable(const char *oh_name, unsigned long freq)
{
	struct device *dev;
	int r;

	dev = omap_hwmod_name_get_dev(oh_name);
	if (IS_ERR(dev)) {
		pr_err("%s: no %s device, did not enable f=%ld\n", __func__,
			oh_name, freq);
		return;
	}

	r = opp_enable(dev, freq);
	if (r < 0)
		dev_err(dev, "%s: opp_enable failed(%d) f=%ld\n", __func__,
			r, freq);
}

/**
 * omap4_opp_init() - initialize omap4 opp table
 */
int __init omap4_opp_init(void)
{
	int r = -ENODEV;

	if (!cpu_is_omap44xx())
		return r;
	if (cpu_is_omap443x())
		r = omap_init_opp_table(omap443x_opp_def_list,
			ARRAY_SIZE(omap443x_opp_def_list));
	else if (cpu_is_omap446x())
		r = omap_init_opp_table(omap446x_opp_def_list,
			ARRAY_SIZE(omap446x_opp_def_list));

	if (!r) {
			omap4_opp_enable("mpu", 1200000000);
			omap4_opp_enable("mpu", 1350000000);
 			omap4_opp_enable("mpu", 1520000000);
 			omap4_opp_enable("mpu", 1650000000);
	}

	return r;
}
device_initcall(omap4_opp_init);
