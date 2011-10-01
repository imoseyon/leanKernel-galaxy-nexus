/**
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __TF_ZEBRA_H__
#define __TF_ZEBRA_H__

#include "tf_defs.h"

int tf_ctrl_device_register(void);

int tf_start(struct tf_comm *comm,
	u32 workspace_addr, u32 workspace_size,
	u8 *pa_buffer, u32 pa_size,
	u8 *properties_buffer, u32 properties_length);

/* Assembler entry points to/from secure */
u32 schedule_secure_world(u32 app_id, u32 proc_id, u32 flags, u32 args);
u32 rpc_handler(u32 p1, u32 p2, u32 p3, u32 p4);
u32 read_mpidr(void);

/* L4 SEC clockdomain enabling/disabling */
void tf_l4sec_clkdm_wakeup(bool wakelock);
void tf_l4sec_clkdm_allow_idle(bool wakeunlock);

/* Delayed secure resume */
int tf_delayed_secure_resume(void);

#endif /* __TF_ZEBRA_H__ */
