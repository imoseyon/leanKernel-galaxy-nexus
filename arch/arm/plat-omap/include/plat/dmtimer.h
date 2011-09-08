/*
 * arch/arm/plat-omap/include/mach/dmtimer.h
 *
 * OMAP Dual-Mode Timers
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * Platform device conversion and hwmod support.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Lauri Leukkunen <lauri.leukkunen@nokia.com>
 * PWM and clock framwork support by Timo Teras.
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

#ifndef __ASM_ARCH_DMTIMER_H
#define __ASM_ARCH_DMTIMER_H

#include <linux/spinlock_types.h>
/* clock sources */
#define OMAP_TIMER_SRC_SYS_CLK			0x00
#define OMAP_TIMER_SRC_32_KHZ			0x01
#define OMAP_TIMER_SRC_EXT_CLK			0x02

/* timer interrupt enable bits */
#define OMAP_TIMER_INT_CAPTURE			(1 << 2)
#define OMAP_TIMER_INT_OVERFLOW			(1 << 1)
#define OMAP_TIMER_INT_MATCH			(1 << 0)

/* trigger types */
#define OMAP_TIMER_TRIGGER_NONE			0x00
#define OMAP_TIMER_TRIGGER_OVERFLOW		0x01
#define OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE	0x02

/*
 * IP revision identifier so that Highlander IP
 * in OMAP4 can be distinguished.
 */
#define OMAP_TIMER_IP_VERSION_1                        0x1
#define OMAP_TIMER_IP_VERSION_2			0x2

struct omap_secure_timer_dev_attr {
	bool is_secure_timer;
};

struct timer_regs {
	u32 tidr;
	u32 tiocp_cfg;
	u32 tistat;
	u32 tisr;
	u32 tier;
	u32 twer;
	u32 tclr;
	u32 tcrr;
	u32 tldr;
	u32 ttrg;
	u32 twps;
	u32 tmar;
	u32 tcar1;
	u32 tsicr;
	u32 tcar2;
	u32 tpir;
	u32 tnir;
	u32 tcvr;
	u32 tocr;
	u32 towr;
};

struct omap_dm_timer {
	int irq;
	struct clk *fclk;
	void __iomem *io_base;
	unsigned reserved:1;
	unsigned enabled:1;
	unsigned posted:1;
	unsigned is_early_init:1;
	unsigned needs_manual_reset:1;
	spinlock_t lock;
	u8 func_offset;
	u8 intr_offset;
	bool loses_context;
	bool context_saved;
	u32 ctx_loss_count;
	struct timer_regs context;
	struct platform_device *pdev;
	struct list_head node;

};

extern struct omap_dm_timer *gptimer_wakeup;
extern struct sys_timer omap_timer;
struct clk;

struct dmtimer_platform_data {
	int (*set_timer_src)(struct platform_device *pdev, int source);
	int timer_ip_type;
	u32 is_early_init:1;
	u32 needs_manual_reset:1;
	bool loses_context;

};

struct omap_dm_timer *omap_dm_timer_request(void);
struct omap_dm_timer *omap_dm_timer_request_specific(int timer_id);
int omap_dm_timer_free(struct omap_dm_timer *timer);
int omap_dm_timer_enable(struct omap_dm_timer *timer);
int omap_dm_timer_disable(struct omap_dm_timer *timer);

int omap_dm_timer_get_irq(struct omap_dm_timer *timer);

u32 omap_dm_timer_modify_idlect_mask(u32 inputmask);
struct clk *omap_dm_timer_get_fclk(struct omap_dm_timer *timer);

int omap_dm_timer_trigger(struct omap_dm_timer *timer);
int omap_dm_timer_start(struct omap_dm_timer *timer);
int omap_dm_timer_stop(struct omap_dm_timer *timer);

int omap_dm_timer_set_source(struct omap_dm_timer *timer, int source);
int omap_dm_timer_set_load(struct omap_dm_timer *timer, int autoreload,
	unsigned int value);
int omap_dm_timer_set_load_start(struct omap_dm_timer *timer,
	int autoreload, unsigned int value);
int omap_dm_timer_set_match(struct omap_dm_timer *timer, int enable,
	unsigned int match);
int omap_dm_timer_set_pwm(struct omap_dm_timer *timer, int def_on,
	int toggle, int trigger);
int omap_dm_timer_set_prescaler(struct omap_dm_timer *timer, int prescaler);

int omap_dm_timer_set_int_enable(struct omap_dm_timer *timer,
	unsigned int value);

unsigned int omap_dm_timer_read_status(struct omap_dm_timer *timer);
int omap_dm_timer_write_status(struct omap_dm_timer *timer, unsigned int value);
unsigned int omap_dm_timer_read_counter(struct omap_dm_timer *timer);
int omap_dm_timer_write_counter(struct omap_dm_timer *timer,
	unsigned int value);

int omap_dm_timers_active(void);


#endif /* __ASM_ARCH_DMTIMER_H */
