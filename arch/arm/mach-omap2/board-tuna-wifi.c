/*
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <plat/mmc.h>

#include <linux/random.h>
#include <linux/jiffies.h>

#include "hsmmc.h"
#include "control.h"
#include "mux.h"

#define GPIO_WLAN_PMENA		104
#define GPIO_WLAN_IRQ		2

#define ATAG_TUNA_MAC	0x57464d41
/* #define ATAG_TUNA_MAC_DEBUG */

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *tuna_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init tuna_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static struct resource tuna_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.end		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

#if 0
/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
  */
static struct embedded_sdio_data tuna_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
};
#endif

static int tuna_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int tuna_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int tuna_wifi_status(struct device *dev)
{
	return tuna_wifi_cd;
}

struct mmc_platform_data tuna_wifi_data = {
	.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_20_21,
	.built_in		= 1,
	.status			= tuna_wifi_status,
	.card_present		= 0,
	.register_status_notify	= tuna_wifi_status_register,
};

static int tuna_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	tuna_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int tuna_wifi_power_state;

struct fixed_voltage_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;
	int microvolts;
	int gpio;
	unsigned startup_delay;
	bool enable_high;
	bool is_enabled;
};

static struct regulator_consumer_supply tuna_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data tuna_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &tuna_vmmc5_supply,
};

static struct fixed_voltage_config tuna_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 2000000, /* 2.0V */
	.gpio = GPIO_WLAN_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &tuna_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &tuna_vwlan,
	},
};

static int tuna_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);
	mdelay(100);
	gpio_set_value(GPIO_WLAN_PMENA, on);
	mdelay(200);

	tuna_wifi_power_state = on;
	return 0;
}

static int tuna_wifi_reset_state;

static int tuna_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	tuna_wifi_reset_state = on;
	return 0;
}

#if 0
static unsigned char tuna_mac_addr[IFHWADDRLEN] = { 0,0x90,0x4c,0,0,0 };

static int __init parse_tag_wlan_mac(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;
#ifdef ATAG_TUNA_MAC_DEBUG
	unsigned i;
#endif

	size = min((tag->hdr.size - 2) * sizeof(__u32), (unsigned)IFHWADDRLEN);
#ifdef ATAG_TUNA_MAC_DEBUG
	printk("WiFi MAC Addr [%d] = 0x%x\n", tag->hdr.size, tag->hdr.tag);
	for(i=0;(i < size);i++) {
		printk(" %02x", dptr[i]);
	}
	printk("\n");
#endif
	memcpy(tuna_mac_addr, dptr, size);
	return 0;
}

__tagtable(ATAG_TUNA_MAC, parse_tag_wlan_mac);

static int tuna_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;

	if (!buf)
		return -EINVAL;

	if ((tuna_mac_addr[4] == 0) && (tuna_mac_addr[5] == 0)) {
		srandom32((uint)jiffies);
		rand_mac = random32();
		tuna_mac_addr[3] = (unsigned char)rand_mac;
		tuna_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		tuna_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, tuna_mac_addr, IFHWADDRLEN);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t tuna_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU",  5}, /* input ISO "EU" to : EU regrev 05 */
	{"FR", "EU",  5},
	{"DE", "EU",  5},
	{"GB", "EU",  5}, /* input ISO "UK" to : EU regrev 05 */
	{"KR", "XY",  3},
	{"AU", "XY",  3},
	{"CN", "XY",  3}, /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY",  3},
	{"AR", "XY",  3},
};

static void *tuna_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(tuna_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode, tuna_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &tuna_wifi_translate_custom_table[i];
	return NULL;
}
#endif

static struct wifi_platform_data tuna_wifi_control = {
	.set_power      = tuna_wifi_power,
	.set_reset      = tuna_wifi_reset,
	.set_carddetect = tuna_wifi_set_carddetect,
	.mem_prealloc	= tuna_wifi_mem_prealloc,
	.get_mac_addr	= NULL, /* tuna_wifi_get_mac_addr, */
	.get_country_code = NULL, /* tuna_wifi_get_country_code, */
};

static struct platform_device tuna_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(tuna_wifi_resources),
        .resource       = tuna_wifi_resources,
        .dev            = {
                .platform_data = &tuna_wifi_control,
        },
};

static void __init tuna_wlan_gpio(void)
{
	pr_debug("%s: start\n", __func__);

	/* WLAN SDIO: MMC5 CMD */
	omap_mux_init_signal("sdmmc5_cmd", OMAP_PIN_INPUT_PULLUP);
	/* WLAN SDIO: MMC5 CLK */
	omap_mux_init_signal("sdmmc5_clk", OMAP_PIN_INPUT_PULLUP);
	/* WLAN SDIO: MMC5 DAT[0-3] */
	omap_mux_init_signal("sdmmc5_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3", OMAP_PIN_INPUT_PULLUP);
	/* WLAN OOB - BCM4330 - GPIO 16 or GPIO 2 */
	omap_mux_init_signal("sim_reset.gpio_wk2", OMAP_PIN_INPUT);
	omap_mux_init_signal("kpd_row1.safe_mode", 0);
	/* WLAN PMENA - GPIO 104 */
	omap_mux_init_signal("gpmc_ncs7.gpio_104", OMAP_PIN_OUTPUT);
	/* Enable power to gpio_wk0-gpio_wk2 */
	omap4_ctrl_wk_pad_writel(0xb0000000,
		OMAP4_CTRL_MODULE_PAD_WKUP_CONTROL_USIMIO);

	/* gpio_enable(GPIO_WLAN_IRQ); */
	gpio_request(GPIO_WLAN_IRQ, "wlan_irq");
	gpio_direction_input(GPIO_WLAN_IRQ);
}

int __init tuna_wlan_init(void)
{
	pr_debug("%s: start\n", __func__);
	tuna_wlan_gpio();
	tuna_init_wifi_mem();
	platform_device_register(&omap_vwlan_device);
	return platform_device_register(&tuna_wifi_device);
}
