/*
 * OMAP4 PRCM Debugging
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "prcm-debug.h"
#include "prm-regbits-44xx.h"
#include "prcm44xx.h"
#include "prm44xx.h"
#include "prminst44xx.h"
#include "cm44xx.h"
#include "cm1_44xx.h"
#include "cm2_44xx.h"
#include "cm-regbits-44xx.h"
#include "cminst44xx.h"
#include "prcm_mpu44xx.h"
#include "powerdomain.h"

/* DPLLs */

struct d_dpll_info {
	char *name;
	void *idlestreg;
	struct d_dpll_derived *derived[];
};

struct d_dpll_derived {
	char *name;
	void *gatereg;
	u32 gatemask;
};

static struct d_dpll_derived derived_dpll_per_m2 = {
	.name = "DPLL_PER_M2",
	.gatereg = OMAP4430_CM_DIV_M2_DPLL_PER,
	.gatemask = 0xa00,
};

static struct d_dpll_derived derived_dpll_per_m3 = {
	.name = "DPLL_PER_M3",
	.gatereg = OMAP4430_CM_DIV_M3_DPLL_PER,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_per_m4 = {
	.name = "DPLL_PER_M4",
	.gatereg = OMAP4430_CM_DIV_M4_DPLL_PER,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_per_m5 = {
	.name = "DPLL_PER_M5",
	.gatereg = OMAP4430_CM_DIV_M5_DPLL_PER,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_per_m6 = {
	.name = "DPLL_PER_M6",
	.gatereg = OMAP4430_CM_DIV_M6_DPLL_PER,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_per_m7 = {
	.name = "DPLL_PER_M7",
	.gatereg = OMAP4430_CM_DIV_M7_DPLL_PER,
	.gatemask = 0x200,
};


static struct d_dpll_info dpll_per = {
	.name = "DPLL_PER",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_PER,
	.derived = {&derived_dpll_per_m2, &derived_dpll_per_m3,
		    &derived_dpll_per_m4, &derived_dpll_per_m5,
		    &derived_dpll_per_m6, &derived_dpll_per_m7,
		    NULL},
};

static struct d_dpll_derived derived_dpll_core_m2 = {
	.name = "DPLL_CORE_M2",
	.gatereg = OMAP4430_CM_DIV_M2_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_core_m3 = {
	.name = "DPLL_CORE_M3",
	.gatereg = OMAP4430_CM_DIV_M3_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_core_m4 = {
	.name = "DPLL_CORE_M4",
	.gatereg = OMAP4430_CM_DIV_M4_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_core_m5 = {
	.name = "DPLL_CORE_M5",
	.gatereg = OMAP4430_CM_DIV_M5_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_core_m6 = {
	.name = "DPLL_CORE_M6",
	.gatereg = OMAP4430_CM_DIV_M6_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_derived derived_dpll_core_m7 = {
	.name = "DPLL_CORE_M7",
	.gatereg = OMAP4430_CM_DIV_M7_DPLL_CORE,
	.gatemask = 0x200,
};

static struct d_dpll_info dpll_core = {
	.name = "DPLL_CORE",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_CORE,
	.derived = {&derived_dpll_core_m2, &derived_dpll_core_m3,
		    &derived_dpll_core_m4, &derived_dpll_core_m5,
		    &derived_dpll_core_m6, &derived_dpll_core_m7,
		    NULL},
};

static struct d_dpll_info dpll_abe = {
	.name = "DPLL_ABE",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_ABE,
	.derived = {/* &derived_dpll_abe_m2, &derived_dpll_abe_m3,
		    &derived_dpll_abe_m4, &derived_dpll_abe_m5,
		    &derived_dpll_abe_m6, &derived_dpll_abe_m7,
		    */ NULL},
};

static struct d_dpll_info dpll_mpu = {
	.name = "DPLL_MPU",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_MPU,
	.derived = {/* &derived_dpll_mpu_m2, */ NULL},
};

static struct d_dpll_info dpll_iva = {
	.name = "DPLL_IVA",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_IVA,
	.derived = {/* &derived_dpll_iva_m4, &derived_dpll_iva_m5, */ NULL},
};

static struct d_dpll_info dpll_usb = {
	.name = "DPLL_USB",
	.idlestreg = OMAP4430_CM_IDLEST_DPLL_USB,
	.derived = {/* &derived_dpll_usb_m2, */ NULL},
};


/* Other internal generators */

struct d_intgen_info {
	char *name;
	void *gatereg;
	u32 gatemask;
};

static struct d_intgen_info intgen_cm1_abe = {
	.name = "CM1_ABE",
	.gatereg = OMAP4430_CM_CLKSEL_ABE,
	.gatemask = 0x500,
};



/* Modules */

#define MOD_MASTER (1 << 0)
#define MOD_SLAVE (1 << 1)
#define MOD_MODE (1 << 2)

struct d_mod_info {
	char *name;
	void *clkctrl;
	int flags;
	int optclk;
};

static struct d_mod_info mod_debug = {
	.name = "DEBUG",
	.clkctrl = OMAP4430_CM_EMU_DEBUGSS_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_bandgap = {
	.name = "BANDGAP",
	.clkctrl = OMAP4430_CM_WKUP_BANDGAP_CLKCTRL,
	.flags = 0,
	.optclk = 0x100,
};

static struct d_mod_info mod_gpio1 = {
	.name = "GPIO1",
	.clkctrl = OMAP4430_CM_WKUP_GPIO1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};

static struct d_mod_info mod_keyboard = {
	.name = "KEYBOARD",
	.clkctrl = OMAP4430_CM_WKUP_KEYBOARD_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sar_ram = {
	.name = "SAR_RAM",
	.clkctrl = OMAP4430_CM_WKUP_SARRAM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_32ktimer = {
	.name = "32KTIMER",
	.clkctrl = OMAP4430_CM_WKUP_SYNCTIMER_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_gptimer1 = {
	.name = "GPTIMER1",
	.clkctrl = OMAP4430_CM_WKUP_TIMER1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_wdtimer2 = {
	.name = "WDTIMER2",
	.clkctrl = OMAP4430_CM_WKUP_WDT2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l4_wkup = {
	.name = "L4_WKUP",
	.clkctrl = OMAP4430_CM_WKUP_L4WKUP_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sr_core = {
	.name = "SR_CORE",
	.clkctrl = OMAP4430_CM_ALWON_SR_CORE_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sr_iva = {
	.name = "SR_IVA",
	.clkctrl = OMAP4430_CM_ALWON_SR_IVA_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sr_mpu = {
	.name = "SR_MPU",
	.clkctrl = OMAP4430_CM_ALWON_SR_MPU_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_fdif = {
	.name = "FACE DETECT",
	.clkctrl = OMAP4430_CM_CAM_FDIF_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER| MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_iss = {
	.name = "ISS",
	.clkctrl = OMAP4430_CM_CAM_ISS_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER| MOD_SLAVE,
	.optclk = 0x100,
};

static struct d_mod_info mod_spinlock = {
	.name = "SPINLOCK",
	.clkctrl = OMAP4430_CM_L4CFG_HW_SEM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l4_cfg_interconnect = {
	.name = "L4_CFG interconnect",
	.clkctrl = OMAP4430_CM_L4CFG_L4_CFG_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_mailbox = {
	.name = "MAILBOX",
	.clkctrl = OMAP4430_CM_L4CFG_MAILBOX_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sar_rom = {
	.name = "SAR_ROM",
	.clkctrl = OMAP4430_CM_L4CFG_SAR_ROM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_dmm = {
	.name = "DMM",
	.clkctrl = OMAP4430_CM_MEMIF_DMM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_emif_1 = {
	.name = "EMIF_1",
	.clkctrl = OMAP4430_CM_MEMIF_EMIF_1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_emif_2 = {
	.name = "EMIF_2",
	.clkctrl = OMAP4430_CM_MEMIF_EMIF_2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_emif_fw = {
	.name = "EMIF_FW",
	.clkctrl = OMAP4430_CM_MEMIF_EMIF_FW_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_dll = {
	.name = "DLL",
	.clkctrl = OMAP4430_CM_MEMIF_DLL_CLKCTRL,
	.flags = 0,
	.optclk = 0x100,
};

static struct d_mod_info mod_cortexm3 = {
	.name = "CORTEXM3",
	.clkctrl = OMAP4430_CM_DUCATI_DUCATI_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_gpmc = {
	.name = "GPMC",
	.clkctrl = OMAP4430_CM_L3_2_GPMC_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l3_2_interconnect = {
	.name = "L3_2 interconnect",
	.clkctrl = OMAP4430_CM_L3_2_L3_2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_ocmc_ram = {
	.name = "OCMC_RAM",
	.clkctrl = OMAP4430_CM_L3_2_OCMC_RAM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l3_3_interconnect = {
	.name = "L3_3 interconnect",
	.clkctrl = OMAP4430_CM_L3INSTR_L3_3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l3_instr_interconnect = {
	.name = "L3_INSTR interconnect",
	.clkctrl = OMAP4430_CM_L3INSTR_L3_INSTR_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_wp1 = {
	.name = "WP1",
	.clkctrl = OMAP4430_CM_L3INSTR_OCP_WP1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_l3_1_interconnect = {
	.name = "L3_1 interconnect",
	.clkctrl = OMAP4430_CM_L3_1_L3_1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_c2c = {
	.name = "C2C",
	.clkctrl = OMAP4430_CM_D2D_SAD2D_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_c2c_fw = {
	.name = "C2C_FW",
	.clkctrl = OMAP4430_CM_D2D_SAD2D_FW_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};


static struct d_mod_info mod_sdma = {
	.name = "sDMA",
	.clkctrl = OMAP4430_CM_SDMA_SDMA_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_dss = {
	.name = "DSS",
	.clkctrl = OMAP4430_CM_DSS_DSS_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0xf00,
};

static struct d_mod_info mod_sgx = {
	.name = "SGX",
	.clkctrl = OMAP4430_CM_GFX_GFX_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_hsi = {
	.name = "HSI",
	.clkctrl = OMAP4430_CM_L3INIT_HSI_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_hsmmc1 = {
	.name = "HSMMC1",
	.clkctrl = OMAP4430_CM_L3INIT_MMC1_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_hsmmc2 = {
	.name = "HSMMC2",
	.clkctrl = OMAP4430_CM_L3INIT_MMC2_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_usbphy = {
	.name = "USBPHY",
	.clkctrl = OMAP4430_CM_L3INIT_USBPHYOCP2SCP_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};

static struct d_mod_info mod_fsusb = {
	.name = "FSUSB",
	.clkctrl = OMAP4430_CM_L3INIT_USB_HOST_FS_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_hsusbhost = {
	.name = "HSUSBHOST",
	.clkctrl = OMAP4430_CM_L3INIT_USB_HOST_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0xff00,
};

static struct d_mod_info mod_hsusbotg = {
	.name = "HSUSBOTG",
	.clkctrl = OMAP4430_CM_L3INIT_USB_OTG_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x100,
};

static struct d_mod_info mod_usbtll = {
	.name = "USBTLL",
	.clkctrl = OMAP4430_CM_L3INIT_USB_TLL_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x300,
};

static struct d_mod_info mod_gptimer10 = {
	.name = "GPTIMER10",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER10_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_gptimer11 = {
	.name = "GPTIMER11",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER11_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer2 = {
	.name = "GPTIMER2",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer3 = {
	.name = "GPTIMER3",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer4 = {
	.name = "GPTIMER4",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer9 = {
	.name = "GPTIMER9",
	.clkctrl = OMAP4430_CM_L4PER_DMTIMER9_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_elm = {
	.name = "ELM",
	.clkctrl = OMAP4430_CM_L4PER_ELM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gpio2 = {
	.name = "GPIO2",
	.clkctrl = OMAP4430_CM_L4PER_GPIO2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};
static struct d_mod_info mod_gpio3 = {
	.name = "GPIO3",
	.clkctrl = OMAP4430_CM_L4PER_GPIO3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};
static struct d_mod_info mod_gpio4 = {
	.name = "GPIO4",
	.clkctrl = OMAP4430_CM_L4PER_GPIO4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};
static struct d_mod_info mod_gpio5 = {
	.name = "GPIO5",
	.clkctrl = OMAP4430_CM_L4PER_GPIO5_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};
static struct d_mod_info mod_gpio6 = {
	.name = "GPIO6",
	.clkctrl = OMAP4430_CM_L4PER_GPIO6_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x100,
};
static struct d_mod_info mod_hdq = {
	.name = "HDQ",
	.clkctrl = OMAP4430_CM_L4PER_HDQ1W_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_i2c1 = {
	.name = "I2C1",
	.clkctrl = OMAP4430_CM_L4PER_I2C1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_i2c2 = {
	.name = "I2C2",
	.clkctrl = OMAP4430_CM_L4PER_I2C2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_i2c3 = {
	.name = "I2C3",
	.clkctrl = OMAP4430_CM_L4PER_I2C3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_i2c4 = {
	.name = "I2C4",
	.clkctrl = OMAP4430_CM_L4PER_I2C4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_l4_per_interconnect = {
	.name = "L4_PER interconnect",
	.clkctrl = OMAP4430_CM_L4PER_L4PER_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcbsp4 = {
	.name = "MCBSP4",
	.clkctrl = OMAP4430_CM_L4PER_MCBSP4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcspi1 = {
	.name = "MCSPI1",
	.clkctrl = OMAP4430_CM_L4PER_MCSPI1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcspi2 = {
	.name = "MCSPI2",
	.clkctrl = OMAP4430_CM_L4PER_MCSPI2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcspi3 = {
	.name = "MCSPI3",
	.clkctrl = OMAP4430_CM_L4PER_MCSPI3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcspi4 = {
	.name = "MCSPI4",
	.clkctrl = OMAP4430_CM_L4PER_MCSPI4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_hsmmc3 = {
	.name = "HSMMC3",
	.clkctrl = OMAP4430_CM_L4PER_MMCSD3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_hsmmc4 = {
	.name = "HSMMC4",
	.clkctrl = OMAP4430_CM_L4PER_MMCSD4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_hsmmc5 = {
	.name = "HSMMC5",
	.clkctrl = OMAP4430_CM_L4PER_MMCSD5_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_slimbus2 = {
	.name = "SLIMBUS2",
	.clkctrl = OMAP4430_CM_L4PER_SLIMBUS2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x700,
};
static struct d_mod_info mod_uart1 = {
	.name = "UART1",
	.clkctrl = OMAP4430_CM_L4PER_UART1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_uart2 = {
	.name = "UART2",
	.clkctrl = OMAP4430_CM_L4PER_UART2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_uart3 = {
	.name = "UART3",
	.clkctrl = OMAP4430_CM_L4PER_UART3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_uart4 = {
	.name = "UART4",
	.clkctrl = OMAP4430_CM_L4PER_UART4_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_audio_engine = {
	.name = "AUDIO ENGINE",
	.clkctrl = OMAP4430_CM1_ABE_AESS_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_dmic = {
	.name = "DMIC",
	.clkctrl = OMAP4430_CM1_ABE_DMIC_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_l4_abe_interconnect = {
	.name = "L4_ABE interconnect",
	.clkctrl = OMAP4430_CM1_ABE_L4ABE_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcasp1 = {
	.name = "MCASP1",
	.clkctrl = OMAP4430_CM1_ABE_MCASP_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcbsp1 = {
	.name = "MCBSP1",
	.clkctrl = OMAP4430_CM1_ABE_MCBSP1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcbsp2 = {
	.name = "MCBSP2",
	.clkctrl = OMAP4430_CM1_ABE_MCBSP2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcbsp3 = {
	.name = "MCBSP3",
	.clkctrl = OMAP4430_CM1_ABE_MCBSP3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_mcpdm = {
	.name = "MCPDM",
	.clkctrl = OMAP4430_CM1_ABE_PDM_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_slimbus1 = {
	.name = "SLIMBUS1",
	.clkctrl = OMAP4430_CM1_ABE_SLIMBUS_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0xf00,
};
static struct d_mod_info mod_gptimer5 = {
	.name = "GPTIMER5",
	.clkctrl = OMAP4430_CM1_ABE_TIMER5_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer6 = {
	.name = "GPTIMER6",
	.clkctrl = OMAP4430_CM1_ABE_TIMER6_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer7 = {
	.name = "GPTIMER7",
	.clkctrl = OMAP4430_CM1_ABE_TIMER7_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_gptimer8 = {
	.name = "GPTIMER8",
	.clkctrl = OMAP4430_CM1_ABE_TIMER8_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_wdtimer3 = {
	.name = "WDTIMER3",
	.clkctrl = OMAP4430_CM1_ABE_WDT3_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_ivahd = {
	.name = "IVAHD",
	.clkctrl = OMAP4430_CM_IVAHD_IVAHD_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};
static struct d_mod_info mod_sl2 = {
	.name = "SL2",
	.clkctrl = OMAP4430_CM_IVAHD_SL2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_dsp = {
	.name = "DSP",
	.clkctrl = OMAP4430_CM_TESLA_TESLA_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_cortexa9 = {
	.name = "CORTEXA9",
	.clkctrl = OMAP4430_CM_MPU_MPU_CLKCTRL,
	.flags = MOD_MODE | MOD_MASTER | MOD_SLAVE,
	.optclk = 0x0,
};

/* L4SEC modules not in TRM, below based on Linux code */

static struct d_mod_info mod_aes1 = {
	.name = "AES1",
	.clkctrl = OMAP4430_CM_L4SEC_AES1_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_aes2 = {
	.name = "AES2",
	.clkctrl = OMAP4430_CM_L4SEC_AES2_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_des3des = {
	.name = "DES3DES",
	.clkctrl = OMAP4430_CM_L4SEC_DES3DES_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_pkaeip29 = {
	.name = "PKAEIP29",
	.clkctrl = OMAP4430_CM_L4SEC_PKAEIP29_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_rng = {
	.name = "RNG",
	.clkctrl = OMAP4430_CM_L4SEC_RNG_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_sha2md51 = {
	.name = "SHA2MD51",
	.clkctrl = OMAP4430_CM_L4SEC_SHA2MD51_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

static struct d_mod_info mod_cryptodma = {
	.name = "CRYPTODMA",
	.clkctrl = OMAP4430_CM_L4SEC_CRYPTODMA_CLKCTRL,
	.flags = MOD_MODE | MOD_SLAVE,
	.optclk = 0x0,
};

/* Clock domains */

struct d_clkd_info {
	char *name;
	const u8 prcm_partition;
	const s16 cm_inst;
	const u16 clkdm_offs;
	int activity;
	struct d_dpll_info *dplls[20];
	struct d_intgen_info *intgens[20];
	struct d_mod_info *mods[];
};

static struct d_clkd_info cd_emu = {
	.name = "CD_EMU",
	.prcm_partition	  = OMAP4430_PRM_PARTITION,
	.cm_inst	  = OMAP4430_PRM_EMU_CM_INST,
	.clkdm_offs	  = OMAP4430_PRM_EMU_CM_EMU_CDOFFS,
	.activity	  = 0x300,
	.mods = {&mod_debug, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: CM1_EMU
};

static struct d_clkd_info cd_wkup = {
	.name = "CD_WKUP",
	.prcm_partition	  = OMAP4430_PRM_PARTITION,
	.cm_inst	  = OMAP4430_PRM_WKUP_CM_INST,
	.clkdm_offs	  = OMAP4430_PRM_WKUP_CM_WKUP_CDOFFS,
	.activity	  = 0x1b00,
	.mods = {&mod_bandgap, &mod_gpio1, &mod_keyboard, &mod_sar_ram,
		 &mod_32ktimer, &mod_gptimer1, &mod_wdtimer2,
		 &mod_l4_wkup, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: SYSCTRL_PADCONF_WKUP, SYSCTRL_GENERAL_WKUP, PRM,
	// SCRM
};

static struct d_clkd_info cd_l4_alwon_core = {
	.name = "CD_L4_ALWON_CORE",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_ALWAYS_ON_INST,
	.clkdm_offs	  = OMAP4430_CM2_ALWAYS_ON_ALWON_CDOFFS,
	.activity	  = 0xf00,
	.mods = {&mod_sr_core, &mod_sr_iva, &mod_sr_mpu, NULL},
	.dplls = {&dpll_per, &dpll_core, &dpll_abe, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: CM1, CORTEXM3_WKUPGEN, SDMA_WKUPGEN, SPINNER
};

static struct d_clkd_info cd_cam = {
	.name = "CD_CAM",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CAM_INST,
	.clkdm_offs	  = OMAP4430_CM2_CAM_CAM_CDOFFS,
	.activity	  = 0x700,
	.mods = {&mod_fdif, &mod_iss, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_l4_cfg = {
	.name = "CD_L4_CFG",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_L4CFG_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_spinlock, &mod_l4_cfg_interconnect, &mod_mailbox,
		 &mod_sar_rom, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: SYSCTRL_PADCONF_CORE, SYSCTRL_GENERAL_CORE
};

static struct d_clkd_info cd_emif = {
	.name = "CD_EMIF",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_MEMIF_CDOFFS,
	.activity	  = 0x700,
	.mods = {&mod_dmm, &mod_emif_1, &mod_emif_2, &mod_emif_fw,
		 &mod_dll, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: DDRPHY
};

static struct d_clkd_info cd_cortexm3 = {
	.name = "CD_CORTEXM3",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_DUCATI_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_cortexm3, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_l3_2 = {
	.name = "CD_L3_2",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_L3_2_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_gpmc, &mod_l3_2_interconnect, &mod_ocmc_ram, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_l3_instr = {
	.name = "CD_L3_INSTR",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_L3INSTR_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_l3_3_interconnect, &mod_l3_instr_interconnect,
		 &mod_wp1, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_l3_1 = {
	.name = "CD_L3_1",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_L3_1_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_l3_1_interconnect, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_c2c = {
	.name = "CD_C2C",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_D2D_CDOFFS,
	.activity	  = 0x700,
	.mods = {&mod_c2c, &mod_c2c_fw, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_dma = {
	.name = "CD_DMA",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_CORE_INST,
	.clkdm_offs	  = OMAP4430_CM2_CORE_SDMA_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_sdma, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_dss = {
	.name = "CD_DSS",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_DSS_INST,
	.clkdm_offs	  = OMAP4430_CM2_DSS_DSS_CDOFFS,
	.activity	  = 0xf00,
	.mods = {&mod_dss, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_sgx = {
	.name = "CD_SGX",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_GFX_INST,
	.clkdm_offs	  = OMAP4430_CM2_GFX_GFX_CDOFFS,
	.activity	  = 0x300,
	.mods = {&mod_sgx, NULL},
};

static struct d_clkd_info cd_l3_init = {
	.name = "CD_L3_INIT",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_L3INIT_INST,
	.clkdm_offs	  = OMAP4430_CM2_L3INIT_L3INIT_CDOFFS,
	.activity	  = 0x3ef7f300,
	.mods = {&mod_hsi, &mod_hsmmc1, &mod_hsmmc2, &mod_usbphy,
		 &mod_fsusb, &mod_hsusbhost, &mod_hsusbotg, &mod_usbtll,
		 NULL},
	.dplls = {&dpll_usb, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: CM1_USB
};

static struct d_clkd_info cd_l4_per = {
	.name = "CD_L4_PER",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_L4PER_INST,
	.clkdm_offs	  = OMAP4430_CM2_L4PER_L4PER_CDOFFS,
	.activity	  = 0x24fff00,
	.mods = {&mod_gptimer10, &mod_gptimer11, &mod_gptimer2,
		 &mod_gptimer3, &mod_gptimer4, &mod_gptimer9, &mod_elm,
		 &mod_gpio2, &mod_gpio3, &mod_gpio4, &mod_gpio5, &mod_gpio6,
		 &mod_hdq, &mod_i2c1, &mod_i2c2, &mod_i2c3, &mod_i2c4,
		 &mod_l4_per_interconnect, &mod_mcbsp4, &mod_mcspi1,
		 &mod_mcspi2, &mod_mcspi3, &mod_mcspi4, &mod_hsmmc3,
		 &mod_hsmmc4, &mod_hsmmc5, &mod_slimbus2, &mod_uart1,
		 &mod_uart2, &mod_uart3, &mod_uart4, NULL},
	// TBD: Linux refs: I2C5
	.intgens = {NULL},
};

static struct d_clkd_info cd_abe = {
	.name = "CD_ABE",
	.prcm_partition	  = OMAP4430_CM1_PARTITION,
	.cm_inst	  = OMAP4430_CM1_ABE_INST,
	.clkdm_offs	  = OMAP4430_CM1_ABE_ABE_CDOFFS,
	.activity	  = 0x3400,
	.mods = {&mod_audio_engine, &mod_dmic, &mod_l4_abe_interconnect,
		 &mod_mcasp1, &mod_mcbsp1, &mod_mcbsp2, &mod_mcbsp3,
		 &mod_mcpdm, &mod_slimbus1, &mod_gptimer5, &mod_gptimer6,
		 &mod_gptimer7, &mod_gptimer8, &mod_wdtimer3, NULL},
	.intgens = {&intgen_cm1_abe, NULL},
};

static struct d_clkd_info cd_ivahd = {
	.name = "CD_IVAHD",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_IVAHD_INST,
	.clkdm_offs	  = OMAP4430_CM2_IVAHD_IVAHD_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_ivahd, &mod_sl2, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_dsp = {
	.name = "CD_DSP",
	.prcm_partition	  = OMAP4430_CM1_PARTITION,
	.cm_inst	  = OMAP4430_CM1_TESLA_INST,
	.clkdm_offs	  = OMAP4430_CM1_TESLA_TESLA_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_dsp, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_pd_alwon_mpu_fake = {
	.name = "N/A (clock generator)",
	.prcm_partition	  = -1,
	.cm_inst	  = -1,
	.clkdm_offs	  = -1,
	.activity	  = 0x0,
	.mods = {NULL},
	.dplls = {&dpll_mpu, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: CORTEXA9_MPU_INTC
};

static struct d_clkd_info cd_pd_alwon_dsp_fake = {
	.name = "N/A (clock generator)",
	.prcm_partition	  = -1,
	.cm_inst	  = -1,
	.clkdm_offs	  = -1,
	.activity	  = 0x0,
	.mods = {NULL},
	.dplls = {&dpll_iva, NULL},
	.intgens = {NULL},
	// TBD: TRM mentions: DSP_WKUPGEN
};

static struct d_clkd_info cd_cortexa9 = {
	.name = "CD_CORTEXA9",
	.prcm_partition	  = OMAP4430_CM1_PARTITION,
	.cm_inst	  = OMAP4430_CM1_MPU_INST,
	.clkdm_offs	  = OMAP4430_CM1_MPU_MPU_CDOFFS,
	.activity	  = 0x100,
	.mods = {&mod_cortexa9, NULL},
	.intgens = {NULL},
};

static struct d_clkd_info cd_l4sec = {
	.name = "CD_L4SEC",
	.prcm_partition	  = OMAP4430_CM2_PARTITION,
	.cm_inst	  = OMAP4430_CM2_L4PER_INST,
	.clkdm_offs	  = OMAP4430_CM2_L4PER_L4SEC_CDOFFS,
	.activity	  = 0x300,
	.mods = {&mod_aes1, &mod_aes2, &mod_des3des, &mod_pkaeip29, &mod_rng,
		 &mod_sha2md51, &mod_cryptodma, NULL},
	.intgens = {NULL},
};

#if 0 /* Don't appear to be valid */
/* CD_MPU0 and MPU1 not in TRM, below based on Linux code. */

static struct d_clkd_info cd_mpu0 = {
	.name = "CD_MPU0",
	.prcm_partition	  = OMAP4430_PRCM_MPU_PARTITION,
	.cm_inst	  = OMAP4430_PRCM_MPU_CPU0_INST,
	.clkdm_offs	  = OMAP4430_PRCM_MPU_CPU0_CPU0_CDOFFS,
	.activity	  = 0x0,
	.mods = {NULL},
	.intgens = {NULL}, // TBD: No docs
};

static struct d_clkd_info cd_mpu1 = {
	.name = "CD_MPU1",
	.prcm_partition	  = OMAP4430_PRCM_MPU_PARTITION,
	.cm_inst	  = OMAP4430_PRCM_MPU_CPU1_INST,
	.clkdm_offs	  = OMAP4430_PRCM_MPU_CPU1_CPU1_CDOFFS,
	.activity	  = 0x0,
	.mods = {NULL},
	.intgens = {NULL}, // TBD: No docs
};
#endif

/* Power domains */

struct d_pwrd_info {
	char *name;
	long prminst;
	int pwrst;
	struct d_clkd_info *cds[];
};

static struct d_pwrd_info pd_emu = {
	.name = "PD_EMU",
	.prminst = OMAP4430_PRM_EMU_INST,
	.pwrst = OMAP4_PM_EMU_PWRSTST_OFFSET,
	.cds = {&cd_emu, NULL},
};

static struct d_pwrd_info pd_wkup = {
	.name = "PD_WKUP",
	.prminst = OMAP4430_PRM_WKUP_INST,
	.pwrst = -1,
	.cds = {&cd_wkup, NULL},
};

static struct d_pwrd_info pd_alwon_core = {
	.name = "PD_ALWON_CORE",
	.prminst = OMAP4430_PRM_ALWAYS_ON_INST,
	.pwrst = -1,
	.cds = {&cd_l4_alwon_core, NULL},
};

static struct d_pwrd_info pd_cam = {
	.name = "PD_CAM",
	.prminst = OMAP4430_PRM_CAM_INST,
	.pwrst = OMAP4_PM_CAM_PWRSTST_OFFSET,
	.cds = {&cd_cam, NULL},
};

static struct d_pwrd_info pd_core = {
	.name = "PD_CORE",
	.prminst = OMAP4430_PRM_CORE_INST,
	.pwrst = OMAP4_PM_CORE_PWRSTST_OFFSET,
	.cds = {&cd_l4_cfg, &cd_emif, &cd_cortexm3, &cd_l3_2, &cd_l3_instr,
		&cd_l3_1, &cd_c2c, &cd_dma, NULL},
	// TBD: TRM mentions: CM2
};

static struct d_pwrd_info pd_dss = {
	.name = "PD_DSS",
	.prminst = OMAP4430_PRM_DSS_INST,
	.pwrst = OMAP4_PM_DSS_PWRSTST_OFFSET,
	.cds = {&cd_dss, NULL},
};

static struct d_pwrd_info pd_sgx = {
	.name = "PD_SGX",
	.prminst = OMAP4430_PRM_GFX_INST,
	.pwrst = OMAP4_PM_GFX_PWRSTST_OFFSET,
	.cds = {&cd_sgx, NULL},
};

static struct d_pwrd_info pd_l3_init = {
	.name = "PD_L3_INIT",
	.prminst = OMAP4430_PRM_L3INIT_INST,
	.pwrst = OMAP4_PM_L3INIT_PWRSTST_OFFSET,
	.cds = {&cd_l3_init, NULL},
};

static struct d_pwrd_info pd_l4_per = {
	.name = "PD_L4_PER",
	.prminst = OMAP4430_PRM_L4PER_INST,
	.pwrst = OMAP4_PM_L4PER_PWRSTST_OFFSET,
	.cds = {&cd_l4_per, &cd_l4sec, NULL},
};

static struct d_pwrd_info pd_std_efuse = {
	.name = "PD_STD_EFUSE",
	.prminst = -1,
	.pwrst = -1,
	.cds = {NULL},
};

static struct d_pwrd_info pd_alwon_dsp = {
	.name = "PD_ALWON_DSP",
	.prminst = -1,
	.pwrst = -1,
	.cds = {&cd_pd_alwon_dsp_fake, NULL},
};

static struct d_pwrd_info pd_audio = {
	.name = "PD_AUDIO",
	.prminst = OMAP4430_PRM_ABE_INST,
	.pwrst = OMAP4_PM_ABE_PWRSTST_OFFSET,
	.cds = {&cd_abe, NULL},
};

static struct d_pwrd_info pd_ivahd = {
	.name = "PD_IVAHD",
	.prminst = OMAP4430_PRM_IVAHD_INST,
	.pwrst = OMAP4_PM_IVAHD_PWRSTST_OFFSET,
	.cds = {&cd_ivahd, NULL},
};

static struct d_pwrd_info pd_dsp = {
	.name = "PD_DSP",
	.prminst = OMAP4430_PRM_TESLA_INST,
	.pwrst = OMAP4_PM_TESLA_PWRSTST_OFFSET,
	.cds = {&cd_dsp, NULL},
};

static struct d_pwrd_info pd_alwon_mpu = {
	.name = "PD_ALWON_MPU",
	.prminst = -1,
	.pwrst = -1,
	.cds = {&cd_pd_alwon_mpu_fake, NULL},
};

static struct d_pwrd_info pd_mpu = {
	.name = "PD_MPU",
	.prminst = OMAP4430_PRM_MPU_INST,
	.pwrst = OMAP4_PM_MPU_PWRSTST_OFFSET,
	.cds = {&cd_cortexa9, NULL},
};

#if 0 /* Do not seem to be valid */
/* CPU0 and CPU1 power domains not in TRM, below based on Linux code */

static struct d_pwrd_info pd_cpu0 = {
	.name = "PD_CPU0",
	.prminst = OMAP4430_PRCM_MPU_CPU0_INST,
	.pwrst = OMAP4_PM_CPU0_PWRSTST_OFFSET,
	.cds = {&cd_mpu0, NULL},
};

static struct d_pwrd_info pd_cpu1 = {
	.name = "PD_CPU1",
	.prminst = OMAP4430_PRCM_MPU_CPU1_INST,
	.pwrst = OMAP4_PM_CPU1_PWRSTST_OFFSET,
	.cds = {&cd_mpu1, NULL},
};
#endif

/* Voltage domains to power domains */

static struct d_pwrd_info *ldo_wakeup_pds[] =
{&pd_emu, &pd_wkup, NULL};

static struct d_pwrd_info *vdd_core_pds[] =
{&pd_alwon_core, &pd_cam, &pd_core, &pd_dss, &pd_sgx, &pd_l3_init, &pd_l4_per,
 &pd_std_efuse, NULL};

static struct d_pwrd_info *vdd_iva_pds[] =
{&pd_alwon_dsp, &pd_audio, &pd_ivahd, &pd_dsp, NULL};

static struct d_pwrd_info *vdd_mpu_pds[] =
{&pd_alwon_mpu, &pd_mpu, /* &pd_cpu0, &pd_cpu1, */ NULL};

/* Voltage domains */

#define N_VDDS 4

struct d_vdd_info {
	char *name;
	int auto_ctrl_shift;
	int auto_ctrl_mask;
	struct d_pwrd_info **pds;
};

static struct d_vdd_info d_vdd[N_VDDS] =  {
	{
		.name = "LDO_WAKEUP",
		.auto_ctrl_shift = -1,
		.auto_ctrl_mask = -1,
		.pds = ldo_wakeup_pds,
	},
	{
		.name = "VDD_CORE_L",
		.auto_ctrl_shift = OMAP4430_AUTO_CTRL_VDD_CORE_L_SHIFT,
		.auto_ctrl_mask = OMAP4430_AUTO_CTRL_VDD_CORE_L_MASK,
		.pds = vdd_core_pds,
	},
	{
		.name = "VDD_IVA_L",
		.auto_ctrl_shift = OMAP4430_AUTO_CTRL_VDD_IVA_L_SHIFT,
		.auto_ctrl_mask = OMAP4430_AUTO_CTRL_VDD_IVA_L_MASK,
		.pds = vdd_iva_pds,
	},
	{
		.name = "VDD_MPU_L",
		.auto_ctrl_shift = OMAP4430_AUTO_CTRL_VDD_MPU_L_SHIFT,
		.auto_ctrl_mask = OMAP4430_AUTO_CTRL_VDD_MPU_L_MASK,
		.pds = vdd_mpu_pds,
	},
};


/* Display strings */

static char *vddauto_s[] = {"disabled", "SLEEP", "RET", "reserved"};

static char *pwrstate_s[] = {"OFF", "RET", "INACTIVE", "ON"};

static char *logic_s[] = {"OFF", "ON"};

static char *cmtrctrl_s[] = {"NOSLEEP", "SW_SLEEP", "SW_WKUP", "HW_AUTO"};

static char *modmode_s[] = {"DISABLED", "AUTO", "ENABLED", "3"};

static char *modstbyst_s[] = {"ON", "STBY"};

static char *modidlest_s[] = {"ON", "TRANSITION", "IDLE", "DISABLED"};

#if 0
#define DEP_S_MAX 19

static char *dep_s[DEP_S_MAX] = {"MPU_M3", "DSP", "IVAHD", "ABE", "MEMIF",
				 "L3_1", "L3_2", "L3INIT", "DSS", "ISS",
				 "GFX", "SDMA", "L4CFG", "L4PER", "L4SEC",
				 "L4WKUP", "ALWON_CORE", "STD_EFUSE",
				 "D2D"};
#endif

#define d_pr(sf, fmt, args...)				\
	{						\
		if (sf)					\
			seq_printf(sf, fmt , ## args);	\
		else					\
			pr_info(fmt , ## args);		\
	}

#define d_pr_ctd(sf, fmt, args...)			\
	{						\
		if (sf)					\
			seq_printf(sf, fmt , ## args);	\
		else					\
			pr_cont(fmt , ## args);		\
	}

static void prcmdebug_dump_dpll(struct seq_file *sf,
				struct d_dpll_info *dpll,
				int flags)
{
	u32 idlest = __raw_readl(dpll->idlestreg);
	u32 st_bypass = idlest & OMAP4430_ST_MN_BYPASS_MASK;
	u32 st_dpll_clk = idlest & OMAP4430_ST_DPLL_CLK_MASK;
	struct d_dpll_derived **derived;

	if (flags & (PRCMDEBUG_LASTSLEEP | PRCMDEBUG_ON) && !st_dpll_clk)
		return;

	d_pr(sf, "         %s status=%s\n", dpll->name,
	     st_dpll_clk ? "locked" : st_bypass ? "bypass" : "stopped");

	derived = dpll->derived;

	while (*derived) {
		u32 enabled = __raw_readl((*derived)->gatereg) &
			(*derived)->gatemask;

		if (!(flags & (PRCMDEBUG_LASTSLEEP | PRCMDEBUG_ON)) ||
		    enabled)
			d_pr(sf, "            %s enabled=0x%x\n",
			     (*derived)->name, enabled);
		derived++;
	}
}


static void prcmdebug_dump_intgen(struct seq_file *sf,
				  struct d_intgen_info *intgen,
				  int flags)
{
	u32 enabled = __raw_readl(intgen->gatereg) & intgen->gatemask;

	if (flags & (PRCMDEBUG_LASTSLEEP | PRCMDEBUG_ON) && !enabled)
		return;

	d_pr(sf, "         %s enabled=0x%x\n", intgen->name, enabled);
}

static void prcmdebug_dump_mod(struct seq_file *sf, struct d_mod_info *mod,
	int flags)
{
	u32 clkctrl = __raw_readl(mod->clkctrl);
	u32 stbyst = (clkctrl & OMAP4430_STBYST_MASK) >> OMAP4430_STBYST_SHIFT;
	u32 idlest = (clkctrl & OMAP4430_IDLEST_MASK) >> OMAP4430_IDLEST_SHIFT;
	u32 optclk = clkctrl & mod->optclk;

	if (flags & (PRCMDEBUG_LASTSLEEP | PRCMDEBUG_ON) &&
	    (!(mod->flags & MOD_MASTER) || stbyst == 1) &&
	    (!(mod->flags & MOD_SLAVE) || idlest == 2 || idlest == 3) &&
	    !optclk)
		return;

	if (flags & PRCMDEBUG_LASTSLEEP &&
	    (mod->flags & MOD_MODE &&
	     ((clkctrl & OMAP4430_MODULEMODE_MASK) >>
	      OMAP4430_MODULEMODE_SHIFT) == 1 /* AUTO */) &&
	    (!(mod->flags & MOD_SLAVE) || idlest == 0) /* ON */ &&
	    !optclk)
		return;

	d_pr(sf, "         %s", mod->name);

	if (mod->flags & MOD_MODE)
		d_pr_ctd(sf, " mode=%s",
			 modmode_s[(clkctrl & OMAP4430_MODULEMODE_MASK) >>
				   OMAP4430_MODULEMODE_SHIFT]);

	if (mod->flags & MOD_MASTER)
		d_pr_ctd(sf, " stbyst=%s",
			 modstbyst_s[stbyst]);

	if (mod->flags & MOD_SLAVE)
		d_pr_ctd(sf, " idlest=%s",
			 modidlest_s[idlest]);

	if (optclk)
		d_pr_ctd(sf, " optclk=0x%x", optclk);

	d_pr_ctd(sf, "\n");
}

static void prcmdebug_dump_real_cd(struct seq_file *sf, struct d_clkd_info *cd,
	int flags)
{
	u32 clktrctrl =
		omap4_cminst_read_inst_reg(cd->prcm_partition, cd->cm_inst,
					   cd->clkdm_offs + OMAP4_CM_CLKSTCTRL);
	u32 mode = (clktrctrl & OMAP4430_CLKTRCTRL_MASK) >>
		OMAP4430_CLKTRCTRL_SHIFT;
	u32 activity = clktrctrl & cd->activity;
#if 0
	u32 staticdep =
		omap4_cminst_read_inst_reg(cd->prcm_partition, cd->cm_inst,
					   cd->clkdm_offs + OMAP4_CM_STATICDEP);
	u32 dynamicdep =
		omap4_cminst_read_inst_reg(cd->prcm_partition, cd->cm_inst,
					   cd->clkdm_offs +
					   OMAP4_CM_STATICDEP + 4) & 0xffffff;
	int i;
#endif

	if (flags & PRCMDEBUG_LASTSLEEP && mode == 3 /* HW_AUTO */)
		return;

	d_pr(sf, "      %s mode=%s", cd->name, cmtrctrl_s[mode]);

	d_pr_ctd(sf, " activity=0x%x", activity);

#if 0
	if (staticdep) {
		d_pr_ctd(sf, " static:");

		for (i = 0; i < DEP_S_MAX; i++)
			if (staticdep & (1 << i))
				d_pr_ctd(sf, " %s", dep_s[i]);
	}
#endif

#if 0
	if (dynamicdep)
		d_pr_ctd(sf, " dynamicdep=0x%x", dynamicdep);
#endif

	d_pr_ctd(sf, "\n");
}

static void prcmdebug_dump_cd(struct seq_file *sf, struct d_clkd_info *cd,
	int flags)
{
	struct d_mod_info **mod;
	struct d_intgen_info **intgen;
	struct d_dpll_info **dpll;

	if (cd->cm_inst != -1) {
		prcmdebug_dump_real_cd(sf, cd, flags);
	} else if (!(flags & PRCMDEBUG_LASTSLEEP)) {
		d_pr(sf, "      %s\n", cd->name);
	}

	mod = cd->mods;

	while (*mod) {
		prcmdebug_dump_mod(sf, *mod, flags);
		mod++;
	}

	dpll = cd->dplls;

	while (*dpll) {
		prcmdebug_dump_dpll(sf, *dpll, flags);
		dpll++;
	}

	intgen = cd->intgens;

	while (*intgen) {
		prcmdebug_dump_intgen(sf, *intgen, flags);
		intgen++;
	}
}

static void prcmdebug_dump_pd(struct seq_file *sf, struct d_pwrd_info *pd,
	int flags)
{
	u32 pwrstst, currst, prevst;
	struct d_clkd_info **cd;

	if (pd->pwrst != -1 && pd->prminst != -1) {
		pwrstst = omap4_prm_read_inst_reg(pd->prminst, pd->pwrst);
		currst = (pwrstst & OMAP4430_POWERSTATEST_MASK) >>
			OMAP4430_POWERSTATEST_SHIFT;
		prevst = (pwrstst & OMAP4430_LASTPOWERSTATEENTERED_MASK) >>
			OMAP4430_LASTPOWERSTATEENTERED_SHIFT;

		if (flags & PRCMDEBUG_LASTSLEEP &&
		    (prevst == PWRDM_POWER_OFF || prevst == PWRDM_POWER_RET))
			return;

		if (flags & PRCMDEBUG_ON &&
		    (currst == PWRDM_POWER_OFF || currst == PWRDM_POWER_RET))
			return;

		d_pr(sf, "   %s curr=%s prev=%s logic=%s\n", pd->name,
		     pwrstate_s[currst],
		     pwrstate_s[prevst],
		     logic_s[(pwrstst & OMAP4430_LOGICSTATEST_MASK) >>
			     OMAP4430_LOGICSTATEST_SHIFT]);
	} else {
		if (flags & PRCMDEBUG_LASTSLEEP)
			return;

		d_pr(sf, "   %s\n", pd->name);
	}

	cd = pd->cds;

	while (*cd) {
		prcmdebug_dump_cd(sf, *cd, flags);
		cd++;
	}
}

static int _prcmdebug_dump(struct seq_file *sf, int flags)
{
	int i;
	u32 prm_voltctrl =
		omap4_prm_read_inst_reg(OMAP4430_PRM_DEVICE_INST,
					OMAP4_PRM_VOLTCTRL_OFFSET);
	struct d_pwrd_info **pd;

	for (i = 0; i < N_VDDS; i++) {
		if (!(flags & PRCMDEBUG_LASTSLEEP)) {
			d_pr(sf, "%s",
			     d_vdd[i].name);

			if (d_vdd[i].auto_ctrl_shift != -1) {
				int auto_ctrl =
					(prm_voltctrl &
					 d_vdd[i].auto_ctrl_mask) >>
					d_vdd[i].auto_ctrl_shift;
				d_pr_ctd(sf, " auto=%s\n",
					 vddauto_s[auto_ctrl]);
			} else {
				d_pr_ctd(sf, " (no auto)\n");
			}

		}

		pd = d_vdd[i].pds;

		while (*pd) {
			prcmdebug_dump_pd(sf, *pd, flags);
			pd++;
		}
	}

	return 0;
}

void prcmdebug_dump(int flags)
{
	_prcmdebug_dump(NULL, flags);
}

static int prcmdebug_all_dump(struct seq_file *sf, void *private)
{
	_prcmdebug_dump(sf, 0);
	return 0;
}

static int prcmdebug_all_open(struct inode *inode, struct file *file)
{
        return single_open(file, prcmdebug_all_dump, NULL);
}


static struct file_operations prcmdebug_all_fops = {
        .open = prcmdebug_all_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

static int prcmdebug_on_dump(struct seq_file *sf, void *private)
{
	_prcmdebug_dump(sf, PRCMDEBUG_ON);
	return 0;
}

static int prcmdebug_on_open(struct inode *inode, struct file *file)
{
        return single_open(file, prcmdebug_on_dump, NULL);
}

static struct file_operations prcmdebug_on_fops = {
        .open = prcmdebug_on_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

static int __init prcmdebug_init(void)
{
	if (IS_ERR_OR_NULL(debugfs_create_file("prcm", S_IRUGO, NULL, NULL,
					       &prcmdebug_all_fops)))
		pr_err("%s: failed to create prcm file\n", __func__);

	if (IS_ERR_OR_NULL(debugfs_create_file("prcm-on", S_IRUGO, NULL, NULL,
					       &prcmdebug_on_fops)))
		pr_err("%s: failed to create prcm-on file\n", __func__);

	return 0;
}

arch_initcall(prcmdebug_init);
