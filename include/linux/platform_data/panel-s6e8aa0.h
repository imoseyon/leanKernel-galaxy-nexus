/*
 * Samsung s6e8aa0 panel support
 *
 * Copyright 2011 Google, Inc.
 * Author: Erik Gilling <konkers@google.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_PLATFORM_DATA_PANEL_S6E8AA0_H
#define __LINUX_PLATFORM_DATA_PANEL_S6E8AA0_H

struct s6e8aa0_sequence_entry {
	const u8 *cmd;
	int cmd_len;
	unsigned int msleep;
};

enum {
	BV_0	=        0x0,
	BV_1	=   0x13E456,
	BV_15	=  0x1390FFB,
	BV_35	=  0x44D7CF9,
	BV_59	=  0xB323808,
	BV_87	= 0x186611F4,
	BV_171	= 0x6840E4FF,
	BV_255	= 0xFFFFFFFF,
};

struct s6e8aa0_gamma_entry {
	u32 brightness;
	u32 v[3];
};

struct s6e8aa0_gamma_adj_points {
	const u32 v1;
	const u32 v15;
	const u32 v35;
	const u32 v59;
	const u32 v87;
	const u32 v171;
};

struct s6e8aa0_color_adj {
	u32 mult[3];
	int rshift;
};

/**
 * struct s6e8aa0_factory_calibration_info - factory calibration parameters
 * @regs:       Register values used during factory calibration. The first array
 *              index is 0 for dark gamma offsets and 1 for bright gamma
 *              offsets. The second array index is color (0: red, 1: green,
 *              2: blue). The last array index is the adjusment point (0: V1,
 *              1: V15, 2: V35, 3: V59, 4: V87, 5: V171, 6: V255).
 * @brightness: Brightness target used for each adjustment point scaled linearly
 *              so the maximum brightness is BV_255 (0xffffffff).The first array
 *              index is 0 for dark gamma offsets and 1 for bright gamma
 *              offsets. The last array index is the adjusment point (0: V1,
 *              1: V15, 2: V35, 3: V59, 4: V87, 5: V171, 6: V255). If the value
 *              is 0 adjustment point is ignored.
 * @color_adj:  Per color brightness multiplier.
 */

struct s6e8aa0_factory_calibration_info {
	u16 regs[2][3][7];
	u32 brightness[2][7];
	struct s6e8aa0_color_adj color_adj;
};

struct s6e8aa0_acl_parameters {
	unsigned int cd;
	unsigned int acl_val;
	u8 regs[29];
};

/**
 * struct s6e8aa0_elvs_parameters -- elvss parameters
 * @cd:         look up value. if the argument cd is <= this value,
 *              then the elvss_val is added to byte 2 of the panel
 *              id to generate the elvss value passed to the controller.
 * @elvss_val:  Value to add to the elvss for this cd
 */
struct s6e8aa0_elvss_parameters {
	unsigned int cd;
	u8 elvss_val;
};

struct panel_s6e8aa0_data {
	int	reset_gpio;
	void	(* set_power)(bool enable);

	const struct s6e8aa0_sequence_entry *seq_display_set;
	int seq_display_set_size;
	const struct s6e8aa0_sequence_entry *seq_etc_set;
	int seq_etc_set_size;

	struct s6e8aa0_factory_calibration_info *factory_info;

	const struct s6e8aa0_gamma_adj_points *gamma_adj_points;
	const struct s6e8aa0_gamma_entry *gamma_table;
	int gamma_table_size;

	const struct s6e8aa0_acl_parameters *acl_table;
	unsigned int acl_table_size;
	unsigned int acl_average;

	const struct s6e8aa0_elvss_parameters *elvss_table;
	int elvss_table_size;
};

#endif
