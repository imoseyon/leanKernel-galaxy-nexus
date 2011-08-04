/*
 * OMAP446x Temperature sensor header file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __ARCH_ARM_PLAT_OMAP_INCLUDE_PLAT_TEMPERATURE_SENSOR_H
#define __ARCH_ARM_PLAT_OMAP_INCLUDE_PLAT_TEMPERATURE_SENSOR_H

/*
 * Offsets from the base of temperature sensor registers
 */
#define TEMP_SENSOR_CTRL_OFFSET	0x00
#define BGAP_CTRL_OFFSET	0x4c
#define BGAP_COUNTER_OFFSET	0x50
#define BGAP_THRESHOLD_OFFSET	0x54
#define BGAP_TSHUT_OFFSET	0x58
#define BGAP_STATUS_OFFSET	0x5c

#define OMAP_TSHUT_GPIO		86


/*
 * omap_temp_sensor platform data
 * @name - name
 * @irq - Irq number for thermal alertemp_sensor
 * @offset - offset of the temp sensor ctrl register
 */
struct omap_temp_sensor_pdata {
	char *name;
	u32 offset;
	int irq;
};

#ifdef CONFIG_OMAP_TEMP_SENSOR
void omap_temp_sensor_resume_idle(void);
void omap_temp_sensor_prepare_idle(void);
#else
static inline void omap_temp_sensor_resume_idle(void) { }
static inline void omap_temp_sensor_prepare_idle(void) { }
#endif

#ifdef CONFIG_OMAP_DIE_TEMP_SENSOR
void omap_temp_sensor_idle(int idle_state);
#else
static inline void omap_temp_sensor_idle(int idle_state) { }
#endif

#endif
