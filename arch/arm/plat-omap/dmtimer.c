/*
 * linux/arch/arm/plat-omap/dmtimer.c
 *
 * OMAP Dual-Mode Timers
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * dmtimer adaptation to platform_driver.
 *
 * Copyright (C) 2005 Nokia Corporation
 * OMAP2 support by Juha Yrjola
 * API improvements and OMAP2 clock framework support by Timo Teras
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <plat/dmtimer.h>
#include <plat/common.h>
#include <plat/omap-pm.h>

/* register offsets */
#define _OMAP_TIMER_ID_OFFSET		0x00
#define _OMAP_TIMER_OCP_CFG_OFFSET	0x10
#define _OMAP_TIMER_SYS_STAT_OFFSET	0x14
#define _OMAP_TIMER_STAT_OFFSET		0x18
#define _OMAP_TIMER_INT_EN_OFFSET	0x1c
#define _OMAP_TIMER_WAKEUP_EN_OFFSET	0x20
#define _OMAP_TIMER_CTRL_OFFSET		0x24
#define		OMAP_TIMER_CTRL_GPOCFG		(1 << 14)
#define		OMAP_TIMER_CTRL_CAPTMODE	(1 << 13)
#define		OMAP_TIMER_CTRL_PT		(1 << 12)
#define		OMAP_TIMER_CTRL_TCM_LOWTOHIGH	(0x1 << 8)
#define		OMAP_TIMER_CTRL_TCM_HIGHTOLOW	(0x2 << 8)
#define		OMAP_TIMER_CTRL_TCM_BOTHEDGES	(0x3 << 8)
#define		OMAP_TIMER_CTRL_SCPWM		(1 << 7)
#define		OMAP_TIMER_CTRL_CE		(1 << 6) /* compare enable */
#define		OMAP_TIMER_CTRL_PRE		(1 << 5) /* prescaler enable */
#define		OMAP_TIMER_CTRL_PTV_SHIFT	2 /* prescaler value shift */
#define		OMAP_TIMER_CTRL_POSTED		(1 << 2)
#define		OMAP_TIMER_CTRL_AR		(1 << 1) /* auto-reload enable */
#define		OMAP_TIMER_CTRL_ST		(1 << 0) /* start timer */
#define _OMAP_TIMER_COUNTER_OFFSET	0x28
#define _OMAP_TIMER_LOAD_OFFSET		0x2c
#define _OMAP_TIMER_TRIGGER_OFFSET	0x30
#define _OMAP_TIMER_WRITE_PEND_OFFSET	0x34
#define		WP_NONE			0	/* no write pending bit */
#define		WP_TCLR			(1 << 0)
#define		WP_TCRR			(1 << 1)
#define		WP_TLDR			(1 << 2)
#define		WP_TTGR			(1 << 3)
#define		WP_TMAR			(1 << 4)
#define		WP_TPIR			(1 << 5)
#define		WP_TNIR			(1 << 6)
#define		WP_TCVR			(1 << 7)
#define		WP_TOCR			(1 << 8)
#define		WP_TOWR			(1 << 9)
#define _OMAP_TIMER_MATCH_OFFSET	0x38
#define _OMAP_TIMER_CAPTURE_OFFSET	0x3c
#define _OMAP_TIMER_IF_CTRL_OFFSET	0x40
#define _OMAP_TIMER_CAPTURE2_OFFSET		0x44	/* TCAR2, 34xx only */
#define _OMAP_TIMER_TICK_POS_OFFSET		0x48	/* TPIR, 34xx only */
#define _OMAP_TIMER_TICK_NEG_OFFSET		0x4c	/* TNIR, 34xx only */
#define _OMAP_TIMER_TICK_COUNT_OFFSET		0x50	/* TCVR, 34xx only */
#define _OMAP_TIMER_TICK_INT_MASK_SET_OFFSET	0x54	/* TOCR, 34xx only */
#define _OMAP_TIMER_TICK_INT_MASK_COUNT_OFFSET	0x58	/* TOWR, 34xx only */

/* register offsets with the write pending bit encoded */
#define	WPSHIFT					16

#define OMAP_TIMER_ID_REG			(_OMAP_TIMER_ID_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_OCP_CFG_REG			(_OMAP_TIMER_OCP_CFG_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_SYS_STAT_REG			(_OMAP_TIMER_SYS_STAT_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_STAT_REG			(_OMAP_TIMER_STAT_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_INT_EN_REG			(_OMAP_TIMER_INT_EN_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_WAKEUP_EN_REG		(_OMAP_TIMER_WAKEUP_EN_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_CTRL_REG			(_OMAP_TIMER_CTRL_OFFSET \
							| (WP_TCLR << WPSHIFT))

#define OMAP_TIMER_COUNTER_REG			(_OMAP_TIMER_COUNTER_OFFSET \
							| (WP_TCRR << WPSHIFT))

#define OMAP_TIMER_LOAD_REG			(_OMAP_TIMER_LOAD_OFFSET \
							| (WP_TLDR << WPSHIFT))

#define OMAP_TIMER_TRIGGER_REG			(_OMAP_TIMER_TRIGGER_OFFSET \
							| (WP_TTGR << WPSHIFT))

#define OMAP_TIMER_WRITE_PEND_REG		(_OMAP_TIMER_WRITE_PEND_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_MATCH_REG			(_OMAP_TIMER_MATCH_OFFSET \
							| (WP_TMAR << WPSHIFT))

#define OMAP_TIMER_CAPTURE_REG			(_OMAP_TIMER_CAPTURE_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_IF_CTRL_REG			(_OMAP_TIMER_IF_CTRL_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_CAPTURE2_REG			(_OMAP_TIMER_CAPTURE2_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_TICK_POS_REG			(_OMAP_TIMER_TICK_POS_OFFSET \
							| (WP_TPIR << WPSHIFT))

#define OMAP_TIMER_TICK_NEG_REG			(_OMAP_TIMER_TICK_NEG_OFFSET \
							| (WP_TNIR << WPSHIFT))

#define OMAP_TIMER_TICK_COUNT_REG		(_OMAP_TIMER_TICK_COUNT_OFFSET \
							| (WP_TCVR << WPSHIFT))

#define OMAP_TIMER_TICK_INT_MASK_SET_REG				\
		(_OMAP_TIMER_TICK_INT_MASK_SET_OFFSET | (WP_TOCR << WPSHIFT))

#define OMAP_TIMER_TICK_INT_MASK_COUNT_REG				\
		(_OMAP_TIMER_TICK_INT_MASK_COUNT_OFFSET | (WP_TOWR << WPSHIFT))

/*
 * OMAP4 IP revision has different register offsets
 * for interrupt registers and functional registers.
 */
#define VERSION2_TIMER_WAKEUP_EN_REG_OFFSET     0x14
#define VERSION2_TIMER_STAT_REG_OFFSET          0x10

#define MAX_WRITE_PEND_WAIT		10000 /* 10ms timeout delay */

static LIST_HEAD(omap_timer_list);
static DEFINE_MUTEX(dm_timer_mutex);

/**
 * omap_dm_timer_read_reg - read timer registers in posted and non-posted mode
 * @timer:      timer pointer over which read operation to perform
 * @reg:        lowest byte holds the register offset
 *
 * The posted mode bit is encoded in reg. Note that in posted mode write
 * pending bit must be checked. Otherwise a read of a non completed write
 * will produce an error.
 */
static inline u32 omap_dm_timer_read_reg(struct omap_dm_timer *timer, u32 reg)
{
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += timer->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += timer->intr_offset;

	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			timer->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);

		if (WARN_ON_ONCE(i == MAX_WRITE_PEND_WAIT))
			dev_err(&timer->pdev->dev, "read timeout.\n");
	}

	return readl(timer->io_base + (reg & 0xff));
}

/**
 * omap_dm_timer_write_reg - write timer registers in posted and non-posted mode
 * @timer:      timer pointer over which write operation is to perform
 * @reg:        lowest byte holds the register offset
 * @value:      data to write into the register
 *
 * The posted mode bit is encoded in reg. Note that in posted mode the write
 * pending bit must be checked. Otherwise a write on a register which has a
 * pending write will be lost.
 */
static void omap_dm_timer_write_reg(struct omap_dm_timer *timer, u32 reg,
						u32 value)
{
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += timer->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += timer->intr_offset;

	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			timer->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);

		if (WARN_ON(i == MAX_WRITE_PEND_WAIT))
			dev_err(&timer->pdev->dev, "write timeout.\n");
	}

	writel(value, timer->io_base + (reg & 0xff));
}

static void omap_timer_save_context(struct omap_dm_timer *timer)
{
	timer->context.tiocp_cfg =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_OCP_CFG_REG);
	timer->context.tistat =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_SYS_STAT_REG);
	timer->context.tisr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_STAT_REG);
	timer->context.tier =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_INT_EN_REG);
	timer->context.twer =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_WAKEUP_EN_REG);
	timer->context.tclr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	timer->context.tcrr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_COUNTER_REG);
	timer->context.tldr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_LOAD_REG);
	timer->context.tmar =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_MATCH_REG);
	timer->context.tsicr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_IF_CTRL_REG);
}

static void omap_timer_restore_context(struct omap_dm_timer *timer)
{
	omap_dm_timer_write_reg(timer, OMAP_TIMER_OCP_CFG_REG,
				timer->context.tiocp_cfg);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_SYS_STAT_REG,
				timer->context.tistat);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG,
				timer->context.tisr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_EN_REG,
				timer->context.tier);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_WAKEUP_EN_REG,
				timer->context.twer);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG,
				timer->context.tclr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG,
				timer->context.tcrr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG,
				timer->context.tldr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_MATCH_REG,
				timer->context.tmar);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG,
				timer->context.tsicr);
}

static void __timer_enable(struct omap_dm_timer *timer)
{
	if (!timer->enabled) {
		pm_runtime_get_sync(&timer->pdev->dev);
		timer->enabled = 1;
	}
}

static void __timer_disable(struct omap_dm_timer *timer)
{
	if (timer->enabled) {
		pm_runtime_put_sync_suspend(&timer->pdev->dev);
		timer->enabled = 0;
	}
}

#ifdef CONFIG_OMAP_DM_TIMER_DEBUG
#define omap_dm_timer_dump_reg(timer, reg)				\
	pr_info(#reg ": %#08x\n", omap_dm_timer_read_reg(timer, reg))

void omap_dm_timer_dump_regs(struct omap_dm_timer *timer)
{
	bool enabled = timer->enabled;
	struct resource *mem = platform_get_resource(timer->pdev,
						     IORESOURCE_MEM, 0);
	resource_size_t memstart = mem ? mem->start : 0;

	if (!enabled)
		__timer_enable(timer);
	pr_info("dmtimer id %d at %p...\n", timer->pdev->id,
		(void *) memstart);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_OCP_CFG_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_SYS_STAT_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_STAT_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_INT_EN_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_WAKEUP_EN_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_CTRL_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_COUNTER_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_LOAD_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_MATCH_REG);
	omap_dm_timer_dump_reg(timer, OMAP_TIMER_IF_CTRL_REG);
	if (!enabled)
		__timer_disable(timer);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_dump_regs);
#endif

static void omap_dm_timer_wait_for_reset(struct omap_dm_timer *timer)
{
	int c;

	c = 0;
	while (!(omap_dm_timer_read_reg(timer, OMAP_TIMER_SYS_STAT_REG) & 1)) {
		c++;
		if (c > 100000) {
			printk(KERN_ERR "Timer failed to reset\n");
			return;
		}
	}
}

static void omap_dm_timer_reset(struct omap_dm_timer *timer)
{
	u32 l;

	if (!timer->is_early_init)
		__timer_enable(timer);

	if (timer->pdev->id != 1) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG, 0x06);
		omap_dm_timer_wait_for_reset(timer);
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_OCP_CFG_REG);
	l |= 0x02 << 3;  /* Set to smart-idle mode */
	l |= 0x2 << 8;   /* Set clock activity to perserve f-clock on idle */
	omap_dm_timer_write_reg(timer, OMAP_TIMER_OCP_CFG_REG, l);

	if (!timer->is_early_init)
		__timer_disable(timer);
}

static int omap_dm_timer_prepare(struct omap_dm_timer *timer)
{
	int ret;

	timer->fclk = clk_get(&timer->pdev->dev, "fck");
	if (WARN_ON_ONCE(IS_ERR_OR_NULL(timer->fclk))) {
		timer->fclk = NULL;
		dev_err(&timer->pdev->dev, ": No fclk handle.\n");
		return -EINVAL;
	}

	if (unlikely(timer->is_early_init)) {
		ret = clk_enable(timer->fclk);
		if (ret) {
			clk_put(timer->fclk);
			return -EINVAL;
		}
		goto end;
	}

	if (timer->needs_manual_reset)
		omap_dm_timer_reset(timer);

	omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_32_KHZ);

end:
	if (!timer->is_early_init)
		__timer_enable(timer);

	/* Match hardware reset default of posted mode */
	omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG,
			OMAP_TIMER_CTRL_POSTED);

	if (!timer->is_early_init)
		__timer_disable(timer);

	timer->posted = 1;
	return 0;
}

struct omap_dm_timer *omap_dm_timer_request(void)
{
	struct omap_dm_timer *timer = NULL, *t;
	int ret;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->reserved)
			continue;

		timer = t;
		timer->reserved = 1;
		timer->enabled = 0;
		break;
	}
	mutex_unlock(&dm_timer_mutex);

	if (!timer) {
		pr_debug("%s: free timer not available.\n", __func__);
		return NULL;
	}
	ret = omap_dm_timer_prepare(timer);
	if (ret) {
		timer->reserved = 0;
		return NULL;
	}

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request);

struct omap_dm_timer *omap_dm_timer_request_specific(int id)
{
	struct omap_dm_timer *timer = NULL, *t;
	int ret;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->pdev->id == id && !t->reserved) {
			timer = t;
			timer->reserved = 1;
			timer->enabled = 0;
			break;
		}
	}
	mutex_unlock(&dm_timer_mutex);

	if (!timer) {
		pr_debug("%s: timer%d not available.\n", __func__, id);
		return NULL;
	}
	ret = omap_dm_timer_prepare(timer);
	if (ret) {
		timer->reserved = 0;
		return NULL;
	}

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request_specific);

int omap_dm_timer_free(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (!timer->reserved) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return -EINVAL;
	}

	__timer_disable(timer);
	clk_put(timer->fclk);

	timer->reserved = 0;
	timer->context_saved = false;

	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_free);

int omap_dm_timer_enable(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_enable);

int omap_dm_timer_disable(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_disable);

int omap_dm_timer_get_irq(struct omap_dm_timer *timer)
{
	if (timer)
		return timer->irq;
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_irq);

#if defined(CONFIG_ARCH_OMAP1)

/**
 * omap_dm_timer_modify_idlect_mask - Check if any running timers use ARMXOR
 * @inputmask: current value of idlect mask
 */
__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	int i = 0;
	struct omap_dm_timer *timer = NULL;

	/* If ARMXOR cannot be idled this function call is unnecessary */
	if (!(inputmask & (1 << 1)))
		return inputmask;

	/* If any active timer is using ARMXOR return modified mask */
	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(timer, &omap_timer_list, node) {

		u32 l;

		l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
		if (l & OMAP_TIMER_CTRL_ST) {
			if (((omap_readl(MOD_CONF_CTRL_1) >> (i * 2)) & 0x03) == 0)
				inputmask &= ~(1 << 1);
			else
				inputmask &= ~(1 << 2);
		}
		i++;
	}
	mutex_unlock(&dm_timer_mutex);

	return inputmask;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#else

struct clk *omap_dm_timer_get_fclk(struct omap_dm_timer *timer)
{
	if (timer)
		return timer->fclk;
	return NULL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_fclk);

__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	BUG();

	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#endif

int omap_dm_timer_trigger(struct omap_dm_timer *timer)
{
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->enabled) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
		spin_unlock_irqrestore(&timer->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_trigger);

int omap_dm_timer_start(struct omap_dm_timer *timer)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->loses_context) {
		__timer_enable(timer);
		if (omap_pm_was_context_lost(&timer->pdev->dev) &&
			timer->context_saved) {
			omap_timer_restore_context(timer);
			timer->context_saved = false;
		}
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (!(l & OMAP_TIMER_CTRL_ST)) {
		l |= OMAP_TIMER_CTRL_ST;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_start);

int omap_dm_timer_stop(struct omap_dm_timer *timer)
{
	u32 l;
	struct dmtimer_platform_data *pdata;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (!timer->enabled) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return -EINVAL;
	}

	pdata = timer->pdev->dev.platform_data;
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (l & OMAP_TIMER_CTRL_ST) {
		l &= ~0x1;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);

		if (!pdata->needs_manual_reset) {
			/* Readback to make sure write has completed */
			omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
			/*
			* Wait for functional clock period x 3.5 to make
			* sure that timer is stopped
			*/
			udelay(3500000 / clk_get_rate(timer->fclk) + 1);
		}
	}
	/* Ack possibly pending interrupt */
	omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG,
			OMAP_TIMER_INT_OVERFLOW);

	if (timer->loses_context) {
		omap_timer_save_context(timer);
		timer->context_saved = true;
		__timer_disable(timer);
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_stop);

int omap_dm_timer_set_source(struct omap_dm_timer *timer, int source)
{
	int ret;
	struct dmtimer_platform_data *pdata;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	if (source < 0 || source >= 3)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	pdata = timer->pdev->dev.platform_data;
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);

	/* change the timer clock source */
	ret = pdata->set_timer_src(timer->pdev, source);

	/*
	 * When the functional clock disappears, too quick writes seem
	 * to cause an abort. XXX Is this still necessary?
	 */
	__delay(300000);

	return ret;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_source);

int omap_dm_timer_set_load(struct omap_dm_timer *timer, int autoreload,
			    unsigned int load)
{
	u32 l;

	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload)
		l |= OMAP_TIMER_CTRL_AR;
	else
		l &= ~OMAP_TIMER_CTRL_AR;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);

	omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load);

int omap_dm_timer_set_load_start(struct omap_dm_timer *timer, int autoreload,
                            unsigned int load)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->loses_context) {
		__timer_enable(timer);
		if (omap_pm_was_context_lost(&timer->pdev->dev) &&
			timer->context_saved) {
			omap_timer_restore_context(timer);
			timer->context_saved = false;
		}
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload) {
		l |= OMAP_TIMER_CTRL_AR;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);
	} else {
		l &= ~OMAP_TIMER_CTRL_AR;
	}
	l |= OMAP_TIMER_CTRL_ST;

	omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG, load);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load_start);

int omap_dm_timer_set_match(struct omap_dm_timer *timer, int enable,
			     unsigned int match)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (enable)
		l |= OMAP_TIMER_CTRL_CE;
	else
		l &= ~OMAP_TIMER_CTRL_CE;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_MATCH_REG, match);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_match);

int omap_dm_timer_set_pwm(struct omap_dm_timer *timer, int def_on,
			   int toggle, int trigger)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_SCPWM |
	       OMAP_TIMER_CTRL_PT | (0x03 << 10));
	if (def_on)
		l |= OMAP_TIMER_CTRL_SCPWM;
	if (toggle)
		l |= OMAP_TIMER_CTRL_PT;
	l |= trigger << 10;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_pwm);

int omap_dm_timer_set_prescaler(struct omap_dm_timer *timer, int prescaler)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));
	if (prescaler >= 0x00 && prescaler <= 0x07) {
		l |= OMAP_TIMER_CTRL_PRE;
		l |= prescaler << 2;
	}
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_prescaler);

int omap_dm_timer_set_int_enable(struct omap_dm_timer *timer,
				  unsigned int value)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (!timer->is_early_init)
		__timer_enable(timer);

	omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_EN_REG, value);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_WAKEUP_EN_REG, value);

	if (!timer->is_early_init)
		__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_int_enable);

unsigned int omap_dm_timer_read_status(struct omap_dm_timer *timer)
{
	unsigned long flags;
	unsigned int ret;

	if (WARN_ON(!timer))
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		ret = omap_dm_timer_read_reg(timer, OMAP_TIMER_STAT_REG);
		spin_unlock_irqrestore(&timer->lock, flags);
		return ret;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	WARN_ON(!timer->enabled);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_status);

int omap_dm_timer_write_status(struct omap_dm_timer *timer, unsigned int value)
{
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG, value);
		spin_unlock_irqrestore(&timer->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_status);

unsigned int omap_dm_timer_read_counter(struct omap_dm_timer *timer)
{
	unsigned long flags;
	unsigned int ret;

	if (WARN_ON(!timer))
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		ret = omap_dm_timer_read_reg(timer, OMAP_TIMER_COUNTER_REG);
		spin_unlock_irqrestore(&timer->lock, flags);
		return ret;
	}

	spin_unlock_irqrestore(&timer->lock, flags);
	WARN_ON(!timer->enabled);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_counter);

int omap_dm_timer_write_counter(struct omap_dm_timer *timer, unsigned int value)
{
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG, value);
		spin_unlock_irqrestore(&timer->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_counter);

int omap_dm_timers_active(void)
{
	struct omap_dm_timer *timer;

	list_for_each_entry(timer, &omap_timer_list, node) {
		if (!timer->enabled)
			continue;

		if (omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG) &
		    OMAP_TIMER_CTRL_ST) {
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timers_active);

/**
 * omap_dm_timer_probe - probe function called for every registered device
 * @pdev:	pointer to current timer platform device
 *
 * Called by driver framework at the end of device registration for all
 * timer devices.
 */
static int __devinit omap_dm_timer_probe(struct platform_device *pdev)
{
	int ret;
	struct omap_dm_timer *timer;
	struct resource *mem, *irq, *ioarea;
	struct dmtimer_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: no platform data.\n", __func__);
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(!irq)) {
		dev_err(&pdev->dev, "%s: no IRQ resource.\n", __func__);
		return -ENODEV;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!mem)) {
		dev_err(&pdev->dev, "%s: no memory resource.\n", __func__);
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "%s: region already claimed.\n", __func__);
		return -EBUSY;
	}

	timer = kzalloc(sizeof(struct omap_dm_timer), GFP_KERNEL);
	if (!timer) {
		dev_err(&pdev->dev, "%s: no memory for omap_dm_timer.\n",
			__func__);
		ret = -ENOMEM;
		goto err_release_ioregion;
	}

	timer->io_base = ioremap(mem->start, resource_size(mem));
	if (!timer->io_base) {
		dev_err(&pdev->dev, "%s: ioremap failed.\n", __func__);
		ret = -ENOMEM;
		goto err_free_mem;
	}

	if (pdata->timer_ip_type == OMAP_TIMER_IP_VERSION_2) {
		timer->func_offset = VERSION2_TIMER_WAKEUP_EN_REG_OFFSET;
		timer->intr_offset = VERSION2_TIMER_STAT_REG_OFFSET;
	}

	timer->irq = irq->start;
	timer->pdev = pdev;
	timer->is_early_init = pdata->is_early_init;
	timer->needs_manual_reset = pdata->needs_manual_reset;
	timer->loses_context = pdata->loses_context;

	spin_lock_init(&timer->lock);
	 /* Skip pm_runtime_enable during early boot and for OMAP1 */
	if (!pdata->is_early_init && !pdata->needs_manual_reset) {
		pm_runtime_enable(&pdev->dev);
		pm_runtime_irq_safe(&pdev->dev);
	}

	/* add the timer element to the list */
	mutex_lock(&dm_timer_mutex);
	list_add_tail(&timer->node, &omap_timer_list);
	mutex_unlock(&dm_timer_mutex);

	dev_dbg(&pdev->dev, "Device Probed.\n");

	return 0;

err_free_mem:
	kfree(timer);

err_release_ioregion:
	release_mem_region(mem->start, resource_size(mem));

	return ret;
}

/**
 * omap_dm_timer_remove - cleanup a registered timer device
 * @pdev:	pointer to current timer platform device
 *
 * Called by driver framework whenever a timer device is unregistered.
 * In addition to freeing platform resources it also deletes the timer
 * entry from the local list.
 */
static int __devexit omap_dm_timer_remove(struct platform_device *pdev)
{
	struct omap_dm_timer *timer;
	int ret = -EINVAL;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(timer, &omap_timer_list, node) {
		if (timer->pdev->id == pdev->id) {
			list_del(&timer->node);
			kfree(timer);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&dm_timer_mutex);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static struct platform_driver omap_dm_timer_driver = {
	.probe  = omap_dm_timer_probe,
	.remove = omap_dm_timer_remove,
	.driver = {
		.name   = "omap_timer",
	},
};

static int __init omap_dm_timer_driver_init(void)
{
	return platform_driver_register(&omap_dm_timer_driver);
}

static void __exit omap_dm_timer_driver_exit(void)
{
	platform_driver_unregister(&omap_dm_timer_driver);
}

early_platform_init("earlytimer", &omap_dm_timer_driver);
module_init(omap_dm_timer_driver_init);
module_exit(omap_dm_timer_driver_exit);

MODULE_DESCRIPTION("OMAP Dual-Mode Timer Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
