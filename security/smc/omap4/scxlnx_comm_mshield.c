/*
 * Copyright (c) 2010 Trusted Logic S.A.
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

#include <asm/div64.h>
#include <asm/system.h>
#include <asm/cputype.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>
#include <linux/cpu.h>

#include <asm/cacheflush.h>

#include <clockdomain.h>

#include "scxlnx_defs.h"

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock g_tf_wake_lock;
static atomic_t tf_wake_lock_count = ATOMIC_INIT(0);
#endif

static struct clockdomain *smc_l4_sec_clkdm;
static atomic_t smc_l4_sec_clkdm_use_count = ATOMIC_INIT(0);

static int __init tf_early_init(void)
{
	smc_l4_sec_clkdm = clkdm_lookup("l4_secure_clkdm");
	if (smc_l4_sec_clkdm == NULL)
		return -EFAULT;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&g_tf_wake_lock, WAKE_LOCK_SUSPEND,
		SCXLNX_DEVICE_BASE_NAME);
#endif

	return 0;
}
early_initcall(tf_early_init);

/*--------------------------------------------------------------------------
 * L4 SEC Clock domain handling
 *-------------------------------------------------------------------------- */

void tf_l4sec_clkdm_wakeup(bool use_spin_lock, bool wakelock)
{
	if (use_spin_lock)
		spin_lock(&SCXLNXGetDevice()->sm.lock);
#ifdef CONFIG_HAS_WAKELOCK
	if (wakelock) {
		atomic_inc(&tf_wake_lock_count);
		wake_lock(&g_tf_wake_lock);
	}
#endif
	atomic_inc(&smc_l4_sec_clkdm_use_count);
	clkdm_wakeup(smc_l4_sec_clkdm);
	if (use_spin_lock)
		spin_unlock(&SCXLNXGetDevice()->sm.lock);
}

void tf_l4sec_clkdm_allow_idle(bool use_spin_lock, bool wakeunlock)
{
	if (use_spin_lock)
		spin_lock(&SCXLNXGetDevice()->sm.lock);
	if (atomic_dec_return(&smc_l4_sec_clkdm_use_count) == 0)
		clkdm_allow_idle(smc_l4_sec_clkdm);
#ifdef CONFIG_HAS_WAKELOCK
	if (wakeunlock)
		if (atomic_dec_return(&tf_wake_lock_count) == 0)
			wake_unlock(&g_tf_wake_lock);
#endif
	if (use_spin_lock)
		spin_unlock(&SCXLNXGetDevice()->sm.lock);
}

