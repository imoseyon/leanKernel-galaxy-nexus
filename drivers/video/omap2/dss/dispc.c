/*
 * linux/drivers/video/omap2/dss/dispc.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#define DSS_SUBSYS_NAME "DISPC"

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>

#include <plat/sram.h>
#include <plat/clock.h>
#include <mach/tiler.h>
#include <plat/omap-pm.h>
#include <video/omapdss.h>

#include "../clockdomain.h"
#include "dss.h"
#include "gammatable.h"
#include "dss_features.h"
#include "dispc.h"

/* DISPC */
#define DISPC_SZ_REGS			SZ_4K

#define DISPC_IRQ_MASK_ERROR            (DISPC_IRQ_GFX_FIFO_UNDERFLOW | \
					 DISPC_IRQ_OCP_ERR | \
					 DISPC_IRQ_VID1_FIFO_UNDERFLOW | \
					 DISPC_IRQ_VID2_FIFO_UNDERFLOW | \
					 DISPC_IRQ_SYNC_LOST | \
					 DISPC_IRQ_SYNC_LOST_DIGIT)

#define DISPC_MAX_NR_ISRS		8

static struct clockdomain *l3_1_clkdm, *l3_2_clkdm;

struct omap_dispc_isr_data {
	omap_dispc_isr_t	isr;
	void			*arg;
	u32			mask;
};

struct dispc_hv_coef {
	s8 hc0_vc00;
	s8 hc1_vc0;
	u8 hc2_vc1;
	s8 hc3_vc2;
	s8 hc4_vc22;
};

#define REG_GET(idx, start, end) \
	FLD_GET(dispc_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end)				\
	dispc_write_reg(idx, FLD_MOD(dispc_read_reg(idx), val, start, end))

struct dispc_irq_stats {
	unsigned long last_reset;
	unsigned irq_count;
	unsigned irqs[32];
};

static struct {
	struct platform_device *pdev;
	void __iomem    *base;

	int		ctx_loss_cnt;
	struct mutex	runtime_lock;
	int		runtime_count;

	int irq;
	struct clk *dss_clk;

	u32	fifo_size[MAX_DSS_OVERLAYS];

	u32	channel_irq[3]; /* Max channels hardcoded to 3*/

	spinlock_t irq_lock;
	u32 irq_error_mask;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];
	u32 error_irqs;
	struct work_struct error_work;

	bool		ctx_valid;
	u32		ctx[DISPC_SZ_REGS / sizeof(u32)];

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spinlock_t irq_stats_lock;
	struct dispc_irq_stats irq_stats;
#endif
} dispc;

enum omap_color_component {
	/* used for all color formats for OMAP3 and earlier
	 * and for RGB and Y color component on OMAP4
	 */
	DISPC_COLOR_COMPONENT_RGB_Y		= 1 << 0,
	/* used for UV component for
	 * OMAP_DSS_COLOR_YUV2, OMAP_DSS_COLOR_UYVY, OMAP_DSS_COLOR_NV12
	 * color formats on OMAP4
	 */
	DISPC_COLOR_COMPONENT_UV		= 1 << 1,
};

static void _omap_dispc_set_irqs(void);

static inline void dispc_write_reg(const u16 idx, u32 val)
{
	__raw_writel(val, dispc.base + idx);
}

static inline u32 dispc_read_reg(const u16 idx)
{
	return __raw_readl(dispc.base + idx);
}

static int dispc_get_ctx_loss_count(void)
{
	struct device *dev = &dispc.pdev->dev;
	struct omap_display_platform_data *pdata = dev->platform_data;
	struct omap_dss_board_info *board_data = pdata->board_data;
	int cnt;

	if (!board_data->get_context_loss_count)
		return -ENOENT;

	cnt = board_data->get_context_loss_count(dev);

	WARN_ONCE(cnt < 0, "get_context_loss_count failed: %d\n", cnt);

	return cnt;
}

#define SR(reg) \
	dispc.ctx[DISPC_##reg / sizeof(u32)] = dispc_read_reg(DISPC_##reg)
#define RR(reg) \
	dispc_write_reg(DISPC_##reg, dispc.ctx[DISPC_##reg / sizeof(u32)])

static void dispc_save_context(void)
{
	int i, o;

	DSSDBG("dispc_save_context\n");

	SR(IRQENABLE);
	SR(CONTROL);
	SR(CONFIG);
	SR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD));
	SR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	SR(TRANS_COLOR(OMAP_DSS_CHANNEL_LCD));
	SR(TRANS_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	SR(LINE_NUMBER);
	SR(TIMING_H(OMAP_DSS_CHANNEL_LCD));
	SR(TIMING_V(OMAP_DSS_CHANNEL_LCD));
	SR(POL_FREQ(OMAP_DSS_CHANNEL_LCD));
	SR(DIVISORo(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_GLOBAL_ALPHA))
		SR(GLOBAL_ALPHA);
	SR(SIZE_MGR(OMAP_DSS_CHANNEL_DIGIT));
	SR(SIZE_MGR(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		SR(CONTROL2);
		SR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD2));
		SR(TRANS_COLOR(OMAP_DSS_CHANNEL_LCD2));
		SR(SIZE_MGR(OMAP_DSS_CHANNEL_LCD2));
		SR(TIMING_H(OMAP_DSS_CHANNEL_LCD2));
		SR(TIMING_V(OMAP_DSS_CHANNEL_LCD2));
		SR(POL_FREQ(OMAP_DSS_CHANNEL_LCD2));
		SR(DIVISORo(OMAP_DSS_CHANNEL_LCD2));
		SR(CONFIG2);
	}

	SR(OVL_BA0(OMAP_DSS_GFX));
	SR(OVL_BA1(OMAP_DSS_GFX));
	SR(OVL_POSITION(OMAP_DSS_GFX));
	SR(OVL_SIZE(OMAP_DSS_GFX));
	SR(OVL_ATTRIBUTES(OMAP_DSS_GFX));
	SR(OVL_FIFO_THRESHOLD(OMAP_DSS_GFX));
	SR(OVL_ROW_INC(OMAP_DSS_GFX));
	SR(OVL_PIXEL_INC(OMAP_DSS_GFX));
	SR(OVL_WINDOW_SKIP(OMAP_DSS_GFX));
	SR(OVL_TABLE_BA(OMAP_DSS_GFX));

	SR(DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD));
	SR(DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD));
	SR(DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD));

	if (dss_has_feature(FEAT_CPR)) {
		SR(CPR_COEF_R(OMAP_DSS_CHANNEL_LCD));
		SR(CPR_COEF_G(OMAP_DSS_CHANNEL_LCD));
		SR(CPR_COEF_B(OMAP_DSS_CHANNEL_LCD));
	}
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		if (dss_has_feature(FEAT_CPR)) {
			SR(CPR_COEF_B(OMAP_DSS_CHANNEL_LCD2));
			SR(CPR_COEF_G(OMAP_DSS_CHANNEL_LCD2));
			SR(CPR_COEF_R(OMAP_DSS_CHANNEL_LCD2));
		}

		SR(DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD2));
		SR(DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD2));
		SR(DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD2));
	}

	if (dss_has_feature(FEAT_PRELOAD))
		SR(OVL_PRELOAD(OMAP_DSS_GFX));

	/* VID1-3 */
	for (o = OMAP_DSS_VIDEO1; o <= OMAP_DSS_VIDEO3; o++) {
		if (o == OMAP_DSS_VIDEO3 && !dss_has_feature(FEAT_OVL_VID3))
			continue;

		SR(OVL_BA0(o));
		SR(OVL_BA1(o));
		SR(OVL_POSITION(o));
		SR(OVL_SIZE(o));
		SR(OVL_ATTRIBUTES(o));
		SR(OVL_FIFO_THRESHOLD(o));
		SR(OVL_ROW_INC(o));
		SR(OVL_PIXEL_INC(o));
		SR(OVL_FIR(o));
		SR(OVL_PICTURE_SIZE(o));
		SR(OVL_ACCU0(o));
		SR(OVL_ACCU1(o));

		for (i = 0; i < 8; i++)
			SR(OVL_FIR_COEF_H(o, i));

		for (i = 0; i < 8; i++)
			SR(OVL_FIR_COEF_HV(o, i));

		for (i = 0; i < 5; i++)
			SR(OVL_CONV_COEF(o, i));

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (i = 0; i < 8; i++)
				SR(OVL_FIR_COEF_V(o, i));
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			SR(OVL_BA0_UV(o));
			SR(OVL_BA1_UV(o));
			SR(OVL_FIR2(o));
			SR(OVL_ACCU2_0(o));
			SR(OVL_ACCU2_1(o));

			for (i = 0; i < 8; i++)
				SR(OVL_FIR_COEF_H2(o, i));

			for (i = 0; i < 8; i++)
				SR(OVL_FIR_COEF_HV2(o, i));

			for (i = 0; i < 8; i++)
				SR(OVL_FIR_COEF_V2(o, i));
		}
		if (dss_has_feature(FEAT_ATTR2))
			SR(OVL_ATTRIBUTES2(o));

		if (dss_has_feature(FEAT_PRELOAD))
			SR(OVL_PRELOAD(o));
	}

	if (dss_has_feature(FEAT_CORE_CLK_DIV))
		SR(DIVISOR);

	dispc.ctx_loss_cnt = dispc_get_ctx_loss_count();
	dispc.ctx_valid = true;

	DSSDBG("context saved, ctx_loss_count %d\n", dispc.ctx_loss_cnt);
}

static void dispc_restore_context(void)
{
	struct device *dev = &dispc.pdev->dev;
	int i, o, ctx;

	DSSDBG("dispc_restore_context\n");

	if (!dispc.ctx_valid)
		return;

	ctx = dispc_get_ctx_loss_count();

	if (!omap_pm_was_context_lost(dev))
		return;

	DSSDBG("ctx_loss_count: saved %d, current %d\n",
			dispc.ctx_loss_cnt, ctx);

	/*RR(IRQENABLE);*/
	/*RR(CONTROL);*/
	RR(CONFIG);
	RR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD));
	RR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	RR(TRANS_COLOR(OMAP_DSS_CHANNEL_LCD));
	RR(TRANS_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	RR(LINE_NUMBER);
	RR(TIMING_H(OMAP_DSS_CHANNEL_LCD));
	RR(TIMING_V(OMAP_DSS_CHANNEL_LCD));
	RR(POL_FREQ(OMAP_DSS_CHANNEL_LCD));
	RR(DIVISORo(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_GLOBAL_ALPHA))
		RR(GLOBAL_ALPHA);
	RR(SIZE_MGR(OMAP_DSS_CHANNEL_DIGIT));
	RR(SIZE_MGR(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		RR(DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD2));
		RR(TRANS_COLOR(OMAP_DSS_CHANNEL_LCD2));
		RR(SIZE_MGR(OMAP_DSS_CHANNEL_LCD2));
		RR(TIMING_H(OMAP_DSS_CHANNEL_LCD2));
		RR(TIMING_V(OMAP_DSS_CHANNEL_LCD2));
		RR(POL_FREQ(OMAP_DSS_CHANNEL_LCD2));
		RR(DIVISORo(OMAP_DSS_CHANNEL_LCD2));
		RR(CONFIG2);
	}

	RR(OVL_BA0(OMAP_DSS_GFX));
	RR(OVL_BA1(OMAP_DSS_GFX));
	RR(OVL_POSITION(OMAP_DSS_GFX));
	RR(OVL_SIZE(OMAP_DSS_GFX));
	RR(OVL_ATTRIBUTES(OMAP_DSS_GFX));
	RR(OVL_FIFO_THRESHOLD(OMAP_DSS_GFX));
	RR(OVL_ROW_INC(OMAP_DSS_GFX));
	RR(OVL_PIXEL_INC(OMAP_DSS_GFX));
	RR(OVL_WINDOW_SKIP(OMAP_DSS_GFX));
	RR(OVL_TABLE_BA(OMAP_DSS_GFX));


	RR(DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD));
	RR(DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD));
	RR(DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD));

	if (dss_has_feature(FEAT_CPR)) {
		RR(CPR_COEF_R(OMAP_DSS_CHANNEL_LCD));
		RR(CPR_COEF_G(OMAP_DSS_CHANNEL_LCD));
		RR(CPR_COEF_B(OMAP_DSS_CHANNEL_LCD));
	}
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		RR(DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD2));
		RR(DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD2));
		RR(DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD2));

		if (dss_has_feature(FEAT_CPR)) {
			RR(CPR_COEF_B(OMAP_DSS_CHANNEL_LCD2));
			RR(CPR_COEF_G(OMAP_DSS_CHANNEL_LCD2));
			RR(CPR_COEF_R(OMAP_DSS_CHANNEL_LCD2));
		}
	}

	if (dss_has_feature(FEAT_PRELOAD))
		RR(OVL_PRELOAD(OMAP_DSS_GFX));


	/* VID1-3 */
	for (o = OMAP_DSS_VIDEO1; o <= OMAP_DSS_VIDEO3; o++) {
		if (o == OMAP_DSS_VIDEO3 && !dss_has_feature(FEAT_OVL_VID3))
			continue;

		RR(OVL_BA0(o));
		RR(OVL_BA1(o));
		RR(OVL_POSITION(o));
		RR(OVL_SIZE(o));
		RR(OVL_ATTRIBUTES(o));
		RR(OVL_FIFO_THRESHOLD(o));
		RR(OVL_ROW_INC(o));
		RR(OVL_PIXEL_INC(o));
		RR(OVL_FIR(o));
		RR(OVL_PICTURE_SIZE(o));
		RR(OVL_ACCU0(o));
		RR(OVL_ACCU1(o));

		for (i = 0; i < 8; i++)
			RR(OVL_FIR_COEF_H(o, i));

		for (i = 0; i < 8; i++)
			RR(OVL_FIR_COEF_HV(o, i));

		for (i = 0; i < 5; i++)
			RR(OVL_CONV_COEF(o, i));

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (i = 0; i < 8; i++)
				RR(OVL_FIR_COEF_V(o, i));
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			RR(OVL_BA0_UV(o));
			RR(OVL_BA1_UV(o));
			RR(OVL_FIR2(o));
			RR(OVL_ACCU2_0(o));
			RR(OVL_ACCU2_1(o));

			for (i = 0; i < 8; i++)
				RR(OVL_FIR_COEF_H2(o, i));

			for (i = 0; i < 8; i++)
				RR(OVL_FIR_COEF_HV2(o, i));

			for (i = 0; i < 8; i++)
				RR(OVL_FIR_COEF_V2(o, i));
		}
		if (dss_has_feature(FEAT_ATTR2))
			RR(OVL_ATTRIBUTES2(o));

		if (dss_has_feature(FEAT_PRELOAD))
			RR(OVL_PRELOAD(o));
	}

	if (dss_has_feature(FEAT_CORE_CLK_DIV))
		RR(DIVISOR);

	/* enable last, because LCD & DIGIT enable are here */
	RR(CONTROL);
	if (dss_has_feature(FEAT_MGR_LCD2))
		RR(CONTROL2);
	/* clear spurious SYNC_LOST_DIGIT interrupts */
	dispc_write_reg(DISPC_IRQSTATUS, DISPC_IRQ_SYNC_LOST_DIGIT);

	/*
	 * enable last so IRQs won't trigger before
	 * the context is fully restored
	 */
	RR(IRQENABLE);

	DSSDBG("context restored\n");
}

#undef SR
#undef RR

static u32 dispc_calculate_threshold(enum omap_plane plane, u32 paddr,
				u32 puv_addr, u16 width, u16 height,
				s32 row_inc, s32 pix_inc)
{
	int shift;
	u32 channel_no = plane;
	u32 val, burstsize, doublestride;
	u32 rotation, bursttype, color_mode;
	struct dispc_config dispc_reg_config;

	if (width >= 1920)
		return 1500;

	/* Get the burst size */
	shift = (plane == OMAP_DSS_GFX) ? 6 : 14;
	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	burstsize = FLD_GET(val, shift + 1, shift);
	doublestride = FLD_GET(val, 22, 22);
	rotation = FLD_GET(val, 13, 12);
	bursttype = FLD_GET(val, 29, 29);
	color_mode = FLD_GET(val, 4, 1);

	/* base address for frame (Luma frame in case of YUV420) */
	dispc_reg_config.ba = paddr;
	/* base address for Chroma frame in case of YUV420 */
	dispc_reg_config.bacbcr = puv_addr;
	/* OrgSizeX for frame */
	dispc_reg_config.sizex = width - 1;
	/* OrgSizeY for frame */
	dispc_reg_config.sizey = height - 1;
	/* burst size */
	dispc_reg_config.burstsize = burstsize;
	/* pixel increment */
	dispc_reg_config.pixelinc = pix_inc;
	/* row increment */
	dispc_reg_config.rowinc  = row_inc;
	/* burst type: 1D/2D */
	dispc_reg_config.bursttype = bursttype;
	/* chroma DoubleStride when in YUV420 format */
	dispc_reg_config.doublestride = doublestride;
	/* Pixcel format of the frame.*/
	dispc_reg_config.format = color_mode;
	/* Rotation of frame */
	dispc_reg_config.rotation = rotation;

	/* DMA buffer allications - assuming reset values */
	dispc_reg_config.gfx_top_buffer = 0;
	dispc_reg_config.gfx_bottom_buffer = 0;
	dispc_reg_config.vid1_top_buffer = 1;
	dispc_reg_config.vid1_bottom_buffer = 1;
	dispc_reg_config.vid2_top_buffer = 2;
	dispc_reg_config.vid2_bottom_buffer = 2;
	dispc_reg_config.vid3_top_buffer = 3;
	dispc_reg_config.vid3_bottom_buffer = 3;
	dispc_reg_config.wb_top_buffer = 4;
	dispc_reg_config.wb_bottom_buffer = 4;

	/* antiFlicker is off */
	dispc_reg_config.antiflicker = 0;

	return sa_calc_wrap(&dispc_reg_config, channel_no);
}

int dispc_runtime_get(void)
{
	int r;

	mutex_lock(&dispc.runtime_lock);

	if (dispc.runtime_count++ == 0) {
		DSSDBG("dispc_runtime_get\n");

		/*
		 * OMAP4 ERRATUM xxxx: Mstandby and disconnect protocol issue
		 * Impacts: all OMAP4 devices
		 * Simplfied Description:
		 * issue #1: The handshake between IP modules on L3_1 and L3_2
		 * peripherals with PRCM has a limitation in a certain time
		 * window of L4 clock cycle. Due to the fact that a wrong
		 * variant of stall signal was used in circuit of PRCM, the
		 * intitator-interconnect protocol is broken when the time
		 * window is hit where the PRCM requires the interconnect to go
		 * to idle while intitator asks to wakeup.
		 * Issue #2: DISPC asserts a sub-mstandby signal for a short
		 * period. In this time interval, IP block requests
		 * disconnection of Master port, and results in Mstandby and
		 * wait request to PRCM. In parallel, if mstandby is de-asserted
		 * by DISPC simultaneously, interconnect requests for a
		 * reconnect for one cycle alone resulting in a disconnect
		 * protocol violation and a deadlock of the system.
		 *
		 * Workaround:
		 * L3_1 clock domain must not be programmed in HW_AUTO if
		 * Static dependency with DSS is enabled and DSS clock domain
		 * is ON. Same for L3_2.
		 */
		if (cpu_is_omap44xx()) {
			clkdm_deny_idle(l3_1_clkdm);
			clkdm_deny_idle(l3_2_clkdm);
		}

		r = dss_runtime_get();
		if (r)
			goto err_dss_get;

		/* XXX dispc fclk can also come from DSI PLL */
		clk_enable(dispc.dss_clk);

		r = pm_runtime_get_sync(&dispc.pdev->dev);
		WARN_ON(r);
		if (r < 0)
			goto err_runtime_get;

		dispc_restore_context();
	}

	mutex_unlock(&dispc.runtime_lock);

	return 0;

err_runtime_get:
	clk_disable(dispc.dss_clk);
	dss_runtime_put();
err_dss_get:
	mutex_unlock(&dispc.runtime_lock);

	return r;
}

void dispc_runtime_put(void)
{
	mutex_lock(&dispc.runtime_lock);

	if (--dispc.runtime_count == 0) {
		int r;

		DSSDBG("dispc_runtime_put\n");

		dispc_save_context();

		r = pm_runtime_put_sync(&dispc.pdev->dev);
		WARN_ON(r);

		clk_disable(dispc.dss_clk);

		dss_runtime_put();

		/*
		 * OMAP4 ERRATUM xxxx: Mstandby and disconnect protocol issue
		 * Workaround:
		 * Restore L3_1 amd L3_2 CD to HW_AUTO, when DSS module idles.
		 */
		if (cpu_is_omap44xx()) {
			clkdm_allow_idle(l3_1_clkdm);
			clkdm_allow_idle(l3_2_clkdm);
		}

	}

	mutex_unlock(&dispc.runtime_lock);
}


bool dispc_go_busy(enum omap_channel channel)
{
	int bit;

	if (channel == OMAP_DSS_CHANNEL_LCD ||
			channel == OMAP_DSS_CHANNEL_LCD2)
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		return REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		return REG_GET(DISPC_CONTROL, bit, bit) == 1;
}

void dispc_go(enum omap_channel channel)
{
	int bit;
	bool enable_bit, go_bit;

	if (channel == OMAP_DSS_CHANNEL_LCD ||
			channel == OMAP_DSS_CHANNEL_LCD2)
		bit = 0; /* LCDENABLE */
	else
		bit = 1; /* DIGITALENABLE */

	/* if the channel is not enabled, we don't need GO */
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		enable_bit = REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		enable_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;

	if (!enable_bit)
		return;

	if (channel == OMAP_DSS_CHANNEL_LCD ||
			channel == OMAP_DSS_CHANNEL_LCD2)
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		go_bit = REG_GET(DISPC_CONTROL2, bit, bit) == 1;
	else
		go_bit = REG_GET(DISPC_CONTROL, bit, bit) == 1;

	if (go_bit) {
		DSSERR("GO bit not down for channel %d\n", channel);
		return;
	}

	DSSDBG("GO %s\n", channel == OMAP_DSS_CHANNEL_LCD ? "LCD" :
		(channel == OMAP_DSS_CHANNEL_LCD2 ? "LCD2" : "DIGIT"));

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, 1, bit, bit);
	else
		REG_FLD_MOD(DISPC_CONTROL, 1, bit, bit);
}

static void _dispc_write_firh_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_H(plane, reg), value);
}

static void _dispc_write_firhv_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_HV(plane, reg), value);
}

static void _dispc_write_firv_reg(enum omap_plane plane, int reg, u32 value)
{
	dispc_write_reg(DISPC_OVL_FIR_COEF_V(plane, reg), value);
}

static void _dispc_write_firh2_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_H2(plane, reg), value);
}

static void _dispc_write_firhv2_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_HV2(plane, reg), value);
}

static void _dispc_write_firv2_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_OVL_FIR_COEF_V2(plane, reg), value);
}

static const struct dispc_hv_coef *
dispc_get_scaling_coef(u32 inc, bool five_taps)
{
	static const struct dispc_hv_coef coef3_M8[8] = {
		{    0,    0,  128,    0,    0 },
		{    0,    2,  123,    3,    0 },
		{    0,    5,  111,   12,    0 },
		{    0,    7,   89,   32,    0 },
		{    0,   64,   64,    0,    0 },
		{    0,   32,   89,    7,    0 },
		{    0,   12,  111,    5,    0 },
		{    0,    3,  123,    2,    0 },
	};

	static const struct dispc_hv_coef coef3_M16[8] = {
		{    0,   36,   56,   36,    0 },
		{    0,   31,   57,   40,    0 },
		{    0,   27,   56,   45,    0 },
		{    0,   23,   55,   50,    0 },
		{    0,   55,   55,   18,    0 },
		{    0,   50,   55,   23,    0 },
		{    0,   45,   56,   27,    0 },
		{    0,   40,   57,   31,    0 },
	};

	static const struct dispc_hv_coef coef_M8[8] = {
		{    0,    0,  128,    0,    0 },
		{    0,   -8,  124,   13,   -1 },
		{   -1,  -11,  112,   30,   -2 },
		{   -2,  -11,   95,   51,   -5 },
		{   -9,   73,   73,   -9,    0 },
		{   -5,   51,   95,  -11,   -2 },
		{   -2,   30,  112,  -11,   -1 },
		{   -1,   13,  124,   -8,    0 },
	};

	static const struct dispc_hv_coef coef_M9[8] = {
		{    8,   -8,  128,   -8,    8 },
		{   14,  -21,  126,    8,    1 },
		{   17,  -27,  117,   30,   -9 },
		{   17,  -30,  103,   56,  -18 },
		{  -26,   83,   83,  -26,   14 },
		{  -18,   56,  103,  -30,   17 },
		{   -9,   30,  117,  -27,   17 },
		{    1,    8,  126,  -21,   14 },
	};

	static const struct dispc_hv_coef coef_M10[8] = {
		{   -2,    2,  128,    2,   -2 },
		{    5,  -12,  125,   20,  -10 },
		{   11,  -22,  116,   41,  -18 },
		{   15,  -27,  102,   62,  -24 },
		{  -28,   83,   83,  -28,   18 },
		{  -24,   62,  102,  -27,   15 },
		{  -18,   41,  116,  -22,   11 },
		{  -10,   20,  125,  -12,    5 },
	};

	static const struct dispc_hv_coef coef_M11[8] = {
		{  -12,   12,  128,   12,  -12 },
		{   -4,   -3,  124,   30,  -19 },
		{    3,  -15,  115,   49,  -24 },
		{    9,  -22,  101,   67,  -27 },
		{  -26,   83,   83,  -26,   14 },
		{  -27,   67,  101,  -22,    9 },
		{  -24,   49,  115,  -15,    3 },
		{  -19,   30,  124,   -3,   -4 },
	};

	static const struct dispc_hv_coef coef_M12[8] = {
		{  -19,   21,  124,   21,  -19 },
		{  -12,    6,  120,   38,  -24 },
		{   -6,   -7,  112,   55,  -26 },
		{    1,  -16,   98,   70,  -25 },
		{  -21,   82,   82,  -21,    6 },
		{  -25,   70,   98,  -16,    1 },
		{  -26,   55,  112,   -7,   -6 },
		{  -24,   38,  120,    6,  -12 },
	};

	static const struct dispc_hv_coef coef_M13[8] = {
		{  -22,   27,  118,   27,  -22 },
		{  -18,   13,  115,   43,  -25 },
		{  -12,    0,  107,   58,  -25 },
		{   -6,  -10,   95,   71,  -22 },
		{  -17,   81,   81,  -17,    0 },
		{  -22,   71,   95,  -10,   -6 },
		{  -25,   58,  107,    0,  -12 },
		{  -25,   43,  115,   13,  -18 },
	};

	static const struct dispc_hv_coef coef_M14[8] = {
		{  -23,   32,  110,   32,  -23 },
		{  -20,   18,  108,   46,  -24 },
		{  -16,    6,  101,   59,  -22 },
		{  -11,   -4,   91,   70,  -18 },
		{  -11,   78,   78,  -11,   -6 },
		{  -18,   70,   91,   -4,  -11 },
		{  -22,   59,  101,    6,  -16 },
		{  -24,   46,  108,   18,  -20 },
	};

	static const struct dispc_hv_coef coef_M16[8] = {
		{  -20,   37,   94,   37,  -20 },
		{  -21,   26,   93,   48,  -18 },
		{  -19,   15,   88,   58,  -14 },
		{  -17,    6,   82,   66,   -9 },
		{   -2,   73,   73,   -2,  -14 },
		{   -9,   66,   82,    6,  -17 },
		{  -14,   58,   88,   15,  -19 },
		{  -18,   48,   93,   26,  -21 },
	};

	static const struct dispc_hv_coef coef_M19[8] = {
		{  -12,   38,   76,   38,  -12 },
		{  -14,   31,   72,   47,   -8 },
		{  -16,   22,   73,   53,   -4 },
		{  -16,   15,   69,   59,    1 },
		{    8,   64,   64,    8,  -16 },
		{    1,   59,   69,   15,  -16 },
		{   -4,   53,   73,   22,  -16 },
		{   -9,   47,   72,   31,  -13 },
	};

	static const struct dispc_hv_coef coef_M22[8] = {
		{   -6,   37,   66,   37,   -6 },
		{   -8,   32,   61,   44,   -1 },
		{  -11,   25,   63,   48,    3 },
		{  -13,   19,   61,   53,    8 },
		{   13,   58,   58,   13,  -14 },
		{    8,   53,   61,   19,  -13 },
		{    3,   48,   63,   25,  -11 },
		{   -2,   44,   61,   32,   -7 },
	};

	static const struct dispc_hv_coef coef_M26[8] = {
		{    1,   36,   54,   36,    1 },
		{   -2,   31,   55,   40,    4 },
		{   -5,   27,   54,   44,    8 },
		{   -8,   22,   53,   48,   13 },
		{   18,   51,   51,   18,  -10 },
		{   13,   48,   53,   22,   -8 },
		{    8,   44,   54,   27,   -5 },
		{    4,   40,   55,   31,   -2 },
	};

	static const struct dispc_hv_coef coef_M32[8] = {
		{    7,   34,   46,   34,    7 },
		{    4,   31,   46,   37,   10 },
		{    1,   27,   46,   39,   14 },
		{   -1,   24,   46,   42,   17 },
		{   21,   45,   45,   21,   -4 },
		{   17,   42,   46,   24,   -1 },
		{   14,   39,   46,   28,    1 },
		{   10,   37,   46,   31,    4 },
	};

	inc >>= 7;	/* /= 128 */
	if (five_taps) {
		if (inc > 26)
			return coef_M32;
		if (inc > 22)
			return coef_M26;
		if (inc > 19)
			return coef_M22;
		if (inc > 16)
			return coef_M19;
		if (inc > 14)
			return coef_M16;
		if (inc > 13)
			return coef_M14;
		if (inc > 12)
			return coef_M13;
		if (inc > 11)
			return coef_M12;
		if (inc > 10)
			return coef_M11;
		if (inc > 9)
			return coef_M10;
		if (inc > 8)
			return coef_M9;
		/* reduce blockiness when upscaling much */
		if (inc > 3)
			return coef_M8;
		if (inc > 2)
			return coef_M11;
		if (inc > 1)
			return coef_M16;
		return coef_M19;
	} else {
		if (inc > 14)
			return coef3_M16;
		/* reduce blockiness when upscaling much */
		if (inc > 3)
			return coef3_M8;
		return coef3_M16;
	}
}

static void _dispc_set_scale_coef(enum omap_plane plane, int hinc,
				  int vinc, bool five_taps,
				  enum omap_color_component color_comp)
{
	const struct dispc_hv_coef *h_coef;
	const struct dispc_hv_coef *v_coef;
	int i;

	h_coef = dispc_get_scaling_coef(hinc, true);
	v_coef = dispc_get_scaling_coef(vinc, five_taps);

	for (i = 0; i < 8; i++) {
		u32 h, hv;

		h = FLD_VAL(h_coef[i].hc0_vc00, 7, 0)
			| FLD_VAL(h_coef[i].hc1_vc0, 15, 8)
			| FLD_VAL(h_coef[i].hc2_vc1, 23, 16)
			| FLD_VAL(h_coef[i].hc3_vc2, 31, 24);
		hv = FLD_VAL(h_coef[i].hc4_vc22, 7, 0)
			| FLD_VAL(v_coef[i].hc1_vc0, 15, 8)
			| FLD_VAL(v_coef[i].hc2_vc1, 23, 16)
			| FLD_VAL(v_coef[i].hc3_vc2, 31, 24);

		if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y) {
			_dispc_write_firh_reg(plane, i, h);
			_dispc_write_firhv_reg(plane, i, hv);
		} else {
			_dispc_write_firh2_reg(plane, i, h);
			_dispc_write_firhv2_reg(plane, i, hv);
		}
	}

	if (five_taps) {
		for (i = 0; i < 8; i++) {
			u32 v;
			v = FLD_VAL(v_coef[i].hc0_vc00, 7, 0)
				| FLD_VAL(v_coef[i].hc4_vc22, 15, 8);
			if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y)
				_dispc_write_firv_reg(plane, i, v);
			else
				_dispc_write_firv2_reg(plane, i, v);
		}
	}
}

void _dispc_setup_color_conv_coef(enum omap_plane plane,
	const struct omap_dss_cconv_coefs *ct)
{
	BUG_ON(plane < OMAP_DSS_VIDEO1 || plane > OMAP_DSS_VIDEO3);

#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 1), CVAL(ct->gy,  ct->rcb));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_OVL_CONV_COEF(plane, 4), CVAL(0, ct->bcb));

#undef CVAL

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), ct->full_range, 11, 11);
}


static void _dispc_set_plane_ba0(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA0(plane), paddr);
}

static void _dispc_set_plane_ba1(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA1(plane), paddr);
}

static void _dispc_set_plane_ba0_uv(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA0_UV(plane), paddr);
}

static void _dispc_set_plane_ba1_uv(enum omap_plane plane, u32 paddr)
{
	dispc_write_reg(DISPC_OVL_BA1_UV(plane), paddr);
}

static void _dispc_set_plane_pos(enum omap_plane plane, int x, int y)
{
	u32 val = FLD_VAL(y, 26, 16) | FLD_VAL(x, 10, 0);

	dispc_write_reg(DISPC_OVL_POSITION(plane), val);
}

static void _dispc_set_pic_size(enum omap_plane plane, int width, int height)
{
	u32 val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);

	if (plane == OMAP_DSS_GFX)
		dispc_write_reg(DISPC_OVL_SIZE(plane), val);
	else
		dispc_write_reg(DISPC_OVL_PICTURE_SIZE(plane), val);
}

static void _dispc_set_vid_size(enum omap_plane plane, int width, int height)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);

	dispc_write_reg(DISPC_OVL_SIZE(plane), val);
}

static void _dispc_set_pre_mult_alpha(enum omap_plane plane, bool enable)
{
	if (!dss_has_feature(FEAT_PRE_MULT_ALPHA))
		return;

	if (!dss_has_feature(FEAT_GLOBAL_ALPHA_VID1) &&
		plane == OMAP_DSS_VIDEO1)
		return;

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable ? 1 : 0, 28, 28);
}

static void _dispc_setup_global_alpha(enum omap_plane plane, u8 global_alpha)
{
	if (!dss_has_feature(FEAT_GLOBAL_ALPHA))
		return;

	if (!dss_has_feature(FEAT_GLOBAL_ALPHA_VID1) &&
		plane == OMAP_DSS_VIDEO1)
		return;

	if (plane == OMAP_DSS_GFX)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 7, 0);
	else if (plane == OMAP_DSS_VIDEO1)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 15, 8);
	else if (plane == OMAP_DSS_VIDEO2)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 23, 16);
	else if (plane == OMAP_DSS_VIDEO3)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 31, 24);
}

static void _dispc_set_pix_inc(enum omap_plane plane, s32 inc)
{
	dispc_write_reg(DISPC_OVL_PIXEL_INC(plane), inc);
}

static void _dispc_set_row_inc(enum omap_plane plane, s32 inc)
{
	dispc_write_reg(DISPC_OVL_ROW_INC(plane), inc);
}

static void _dispc_set_color_mode(enum omap_plane plane,
		enum omap_color_mode color_mode)
{
	u32 m = 0;
	if (plane != OMAP_DSS_GFX) {
		switch (color_mode) {
		case OMAP_DSS_COLOR_NV12:
			m = 0x0; break;
		case OMAP_DSS_COLOR_RGBX16:
			m = 0x1; break;
		case OMAP_DSS_COLOR_RGBA16:
			m = 0x2; break;
		case OMAP_DSS_COLOR_RGB12U:
			m = 0x4; break;
		case OMAP_DSS_COLOR_ARGB16:
			m = 0x5; break;
		case OMAP_DSS_COLOR_RGB16:
			m = 0x6; break;
		case OMAP_DSS_COLOR_ARGB16_1555:
			m = 0x7; break;
		case OMAP_DSS_COLOR_RGB24U:
			m = 0x8; break;
		case OMAP_DSS_COLOR_RGB24P:
			m = 0x9; break;
		case OMAP_DSS_COLOR_YUV2:
			m = 0xa; break;
		case OMAP_DSS_COLOR_UYVY:
			m = 0xb; break;
		case OMAP_DSS_COLOR_ARGB32:
			m = 0xc; break;
		case OMAP_DSS_COLOR_RGBA32:
			m = 0xd; break;
		case OMAP_DSS_COLOR_RGBX32:
			m = 0xe; break;
		case OMAP_DSS_COLOR_XRGB16_1555:
			m = 0xf; break;
		default:
			BUG(); break;
		}
	} else {
		switch (color_mode) {
		case OMAP_DSS_COLOR_CLUT1:
			m = 0x0; break;
		case OMAP_DSS_COLOR_CLUT2:
			m = 0x1; break;
		case OMAP_DSS_COLOR_CLUT4:
			m = 0x2; break;
		case OMAP_DSS_COLOR_CLUT8:
			m = 0x3; break;
		case OMAP_DSS_COLOR_RGB12U:
			m = 0x4; break;
		case OMAP_DSS_COLOR_ARGB16:
			m = 0x5; break;
		case OMAP_DSS_COLOR_RGB16:
			m = 0x6; break;
		case OMAP_DSS_COLOR_ARGB16_1555:
			m = 0x7; break;
		case OMAP_DSS_COLOR_RGB24U:
			m = 0x8; break;
		case OMAP_DSS_COLOR_RGB24P:
			m = 0x9; break;
		case OMAP_DSS_COLOR_YUV2:
		case OMAP_DSS_COLOR_RGBX16:
			m = 0xa; break;
		case OMAP_DSS_COLOR_UYVY:
		case OMAP_DSS_COLOR_RGBA16:
			m = 0xb; break;
		case OMAP_DSS_COLOR_ARGB32:
			m = 0xc; break;
		case OMAP_DSS_COLOR_RGBA32:
			m = 0xd; break;
		case OMAP_DSS_COLOR_RGBX32:
			m = 0xe; break;
		case OMAP_DSS_COLOR_XRGB16_1555:
			m = 0xf; break;
		default:
			BUG(); break;
		}
	}

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), m, 4, 1);
}

void dispc_set_channel_out(enum omap_plane plane,
		enum omap_channel channel)
{
	int shift;
	u32 val;
	int chan = 0, chan2 = 0;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
	case OMAP_DSS_VIDEO3:
		shift = 16;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		switch (channel) {
		case OMAP_DSS_CHANNEL_LCD:
			chan = 0;
			chan2 = 0;
			break;
		case OMAP_DSS_CHANNEL_DIGIT:
			chan = 1;
			chan2 = 0;
			break;
		case OMAP_DSS_CHANNEL_LCD2:
			chan = 0;
			chan2 = 1;
			break;
		default:
			BUG();
		}

		val = FLD_MOD(val, chan, shift, shift);
		val = FLD_MOD(val, chan2, 31, 30);
	} else {
		val = FLD_MOD(val, channel, shift, shift);
	}
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

void dispc_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size)
{
	int shift;
	u32 val;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 6;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
	case OMAP_DSS_VIDEO3:
		shift = 14;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, burst_size, shift+1, shift);
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

void dispc_enable_gamma_table(bool enable)
{
	/*
	 * This is partially implemented to support only disabling of
	 * the gamma table.
	 */
	if (enable) {
		DSSWARN("Gamma table enabling for TV not yet supported");
		return;
	}

	REG_FLD_MOD(DISPC_CONFIG, enable, 9, 9);
}

void dispc_set_zorder(enum omap_plane plane,
			enum omap_overlay_zorder zorder)
{
	u32 val;

	if (!dss_has_feature(FEAT_OVL_ZORDER))
		return;
	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, zorder, 27, 26);
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

void dispc_enable_zorder(enum omap_plane plane, bool enable)
{
	u32 val;

	if (!dss_has_feature(FEAT_OVL_ZORDER))
		return;
	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, enable, 25, 25);
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

void dispc_enable_cpr(enum omap_channel channel, bool enable)
{
	u16 reg;

	if (channel == OMAP_DSS_CHANNEL_LCD)
		reg = DISPC_CONFIG;
	else if (channel == OMAP_DSS_CHANNEL_LCD2)
		reg = DISPC_CONFIG2;
	else
		return;

	REG_FLD_MOD(reg, enable, 15, 15);
}

void dispc_set_cpr_coef(enum omap_channel channel,
		struct omap_dss_cpr_coefs *coefs)
{
	u32 coef_r, coef_g, coef_b;

	if (channel != OMAP_DSS_CHANNEL_LCD && channel != OMAP_DSS_CHANNEL_LCD2)
		return;

	coef_r = FLD_VAL(coefs->rr, 31, 22) | FLD_VAL(coefs->rg, 20, 11) |
		FLD_VAL(coefs->rb, 9, 0);
	coef_g = FLD_VAL(coefs->gr, 31, 22) | FLD_VAL(coefs->gg, 20, 11) |
		FLD_VAL(coefs->gb, 9, 0);
	coef_b = FLD_VAL(coefs->br, 31, 22) | FLD_VAL(coefs->bg, 20, 11) |
		FLD_VAL(coefs->bb, 9, 0);

	dispc_write_reg(DISPC_CPR_COEF_R(channel), coef_r);
	dispc_write_reg(DISPC_CPR_COEF_G(channel), coef_g);
	dispc_write_reg(DISPC_CPR_COEF_B(channel), coef_b);
}

static void _dispc_set_vid_color_conv(enum omap_plane plane, bool enable)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));
	val = FLD_MOD(val, enable, 9, 9);
	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), val);
}

void dispc_enable_replication(enum omap_plane plane, bool enable)
{
	int bit;

	if (plane == OMAP_DSS_GFX)
		bit = 5;
	else
		bit = 10;

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable, bit, bit);
}

void dispc_set_lcd_size(enum omap_channel channel, u16 width, u16 height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(DISPC_SIZE_MGR(channel), val);
}

void dispc_set_digit_size(u16 width, u16 height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_DIGIT), val);
}

static void dispc_read_plane_fifo_sizes(void)
{
	u32 size;
	int plane;
	u8 start, end;

	dss_feat_get_reg_field(FEAT_REG_FIFOSIZE, &start, &end);

	for (plane = 0; plane < ARRAY_SIZE(dispc.fifo_size); ++plane) {
		size = FLD_GET(dispc_read_reg(DISPC_OVL_FIFO_SIZE_STATUS(plane)),
			start, end);
		dispc.fifo_size[plane] = size;
	}
}

u32 dispc_get_plane_fifo_size(enum omap_plane plane)
{
	return dispc.fifo_size[plane];
}

void dispc_setup_plane_fifo(enum omap_plane plane, u32 low, u32 high)
{
	u8 hi_start, hi_end, lo_start, lo_end;

	dss_feat_get_reg_field(FEAT_REG_FIFOHIGHTHRESHOLD, &hi_start, &hi_end);
	dss_feat_get_reg_field(FEAT_REG_FIFOLOWTHRESHOLD, &lo_start, &lo_end);

	DSSDBG("fifo(%d) low/high old %u/%u, new %u/%u\n",
			plane,
			REG_GET(DISPC_OVL_FIFO_THRESHOLD(plane),
				lo_start, lo_end),
			REG_GET(DISPC_OVL_FIFO_THRESHOLD(plane),
				hi_start, hi_end),
			low, high);

	/* preload to high threshold to avoid FIFO underflow */
	dispc_write_reg(DISPC_OVL_PRELOAD(plane), min(high, 0xfffu));

	dispc_write_reg(DISPC_OVL_FIFO_THRESHOLD(plane),
			FLD_VAL(high, hi_start, hi_end) |
			FLD_VAL(low, lo_start, lo_end));
}

void dispc_enable_fifomerge(bool enable)
{
	DSSDBG("FIFO merge %s\n", enable ? "enabled" : "disabled");
	REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 14, 14);
}

static void _dispc_set_fir(enum omap_plane plane,
				int hinc, int vinc,
				enum omap_color_component color_comp)
{
	u32 val;

	if (color_comp == DISPC_COLOR_COMPONENT_RGB_Y) {
		u8 hinc_start, hinc_end, vinc_start, vinc_end;

		dss_feat_get_reg_field(FEAT_REG_FIRHINC,
					&hinc_start, &hinc_end);
		dss_feat_get_reg_field(FEAT_REG_FIRVINC,
					&vinc_start, &vinc_end);
		val = FLD_VAL(vinc, vinc_start, vinc_end) |
				FLD_VAL(hinc, hinc_start, hinc_end);

		dispc_write_reg(DISPC_OVL_FIR(plane), val);
	} else {
		val = FLD_VAL(vinc, 28, 16) | FLD_VAL(hinc, 12, 0);
		dispc_write_reg(DISPC_OVL_FIR2(plane), val);
	}
}

static void _dispc_set_vid_accu0(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	u8 hor_start, hor_end, vert_start, vert_end;

	dss_feat_get_reg_field(FEAT_REG_HORIZONTALACCU, &hor_start, &hor_end);
	dss_feat_get_reg_field(FEAT_REG_VERTICALACCU, &vert_start, &vert_end);

	val = FLD_VAL(vaccu, vert_start, vert_end) |
			FLD_VAL(haccu, hor_start, hor_end);

	dispc_write_reg(DISPC_OVL_ACCU0(plane), val);
}

static void _dispc_set_vid_accu1(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	u8 hor_start, hor_end, vert_start, vert_end;

	dss_feat_get_reg_field(FEAT_REG_HORIZONTALACCU, &hor_start, &hor_end);
	dss_feat_get_reg_field(FEAT_REG_VERTICALACCU, &vert_start, &vert_end);

	val = FLD_VAL(vaccu, vert_start, vert_end) |
			FLD_VAL(haccu, hor_start, hor_end);

	dispc_write_reg(DISPC_OVL_ACCU1(plane), val);
}

static void _dispc_set_vid_accu2_0(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;

	val = FLD_VAL(vaccu, 26, 16) | FLD_VAL(haccu, 10, 0);
	dispc_write_reg(DISPC_OVL_ACCU2_0(plane), val);
}

static void _dispc_set_vid_accu2_1(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;

	val = FLD_VAL(vaccu, 26, 16) | FLD_VAL(haccu, 10, 0);
	dispc_write_reg(DISPC_OVL_ACCU2_1(plane), val);
}

static void _dispc_set_scale_param(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool five_taps, u8 rotation,
		enum omap_color_component color_comp)
{
	int fir_hinc, fir_vinc;
	int hscaleup, vscaleup;

	hscaleup = orig_width <= out_width;
	vscaleup = orig_height <= out_height;

	_dispc_set_scale_coef(plane, hscaleup, vscaleup, five_taps, color_comp);

	fir_hinc = 1024 * orig_width / out_width;
	fir_vinc = 1024 * orig_height / out_height;

	_dispc_set_fir(plane, fir_hinc, fir_vinc, color_comp);
}

static void _dispc_set_scaling_common(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	int accu0 = 0;
	int accu1 = 0;
	u32 l;
	u16 y_adjust = color_mode == OMAP_DSS_COLOR_NV12 ? 2 : 0;

	_dispc_set_scale_param(plane, orig_width, orig_height - y_adjust,
				out_width, out_height, five_taps,
				rotation, DISPC_COLOR_COMPONENT_RGB_Y);
	l = dispc_read_reg(DISPC_OVL_ATTRIBUTES(plane));

	/* RESIZEENABLE and VERTICALTAPS */
	l &= ~((0x3 << 5) | (0x1 << 21));
	l |= (orig_width != out_width) ? (1 << 5) : 0;
	l |= (orig_height != out_height) ? (1 << 6) : 0;
	l |= five_taps ? (1 << 21) : 0;

	/* VRESIZECONF and HRESIZECONF */
	if (dss_has_feature(FEAT_RESIZECONF)) {
		l &= ~(0x3 << 7);
		l |= (orig_width <= out_width) ? 0 : (1 << 7);
		l |= (orig_height <= out_height) ? 0 : (1 << 8);
	}

	/* LINEBUFFERSPLIT */
	if (dss_has_feature(FEAT_LINEBUFFERSPLIT)) {
		l &= ~(0x1 << 22);
		l |= five_taps ? (1 << 22) : 0;
	}

	dispc_write_reg(DISPC_OVL_ATTRIBUTES(plane), l);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	if (ilace && !fieldmode) {
		accu1 = 0;
		accu0 = ((1024 * orig_height / out_height) / 2) & 0x3ff;
		if (accu0 >= 1024/2) {
			accu1 = 1024/2;
			accu0 -= accu1;
		}
	}

	_dispc_set_vid_accu0(plane, 0, accu0);
	_dispc_set_vid_accu1(plane, 0, accu1);
}

static void _dispc_set_scaling_uv(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	int scale_x = out_width != orig_width;
	int scale_y = out_height != orig_height;
	u16 y_adjust = 0;

	if (!dss_has_feature(FEAT_HANDLE_UV_SEPARATE))
		return;
	if ((color_mode != OMAP_DSS_COLOR_YUV2 &&
			color_mode != OMAP_DSS_COLOR_UYVY &&
			color_mode != OMAP_DSS_COLOR_NV12)) {
		/* reset chroma resampling for RGB formats  */
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES2(plane), 0, 8, 8);
		return;
	}
	switch (color_mode) {
	case OMAP_DSS_COLOR_NV12:
		/* UV is subsampled by 2 vertically*/
		orig_height >>= 1;
		/* UV is subsampled by 2 horz.*/
		orig_width >>= 1;
		y_adjust = 1;
		break;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		/*For YUV422 with 90/270 rotation,
		 *we don't upsample chroma
		 */
		if (rotation == OMAP_DSS_ROT_0 ||
			rotation == OMAP_DSS_ROT_180)
			/* UV is subsampled by 2 hrz*/
			orig_width >>= 1;
		/* must use FIR for YUV422 if rotated */
		if (rotation != OMAP_DSS_ROT_0)
			scale_x = scale_y = true;
		break;
	default:
		BUG();
	}

	if (out_width != orig_width)
		scale_x = true;
	if (out_height != orig_height)
		scale_y = true;

	_dispc_set_scale_param(plane, orig_width, orig_height - y_adjust,
			out_width, out_height, five_taps,
				rotation, DISPC_COLOR_COMPONENT_UV);

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES2(plane),
		(scale_x || scale_y) ? 1 : 0, 8, 8);
	/* set H scaling */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), scale_x ? 1 : 0, 5, 5);
	/* set V scaling */
	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), scale_y ? 1 : 0, 6, 6);

	_dispc_set_vid_accu2_0(plane, 0x80, 0);
	_dispc_set_vid_accu2_1(plane, 0x80, 0);
}

static void _dispc_set_scaling(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, enum omap_color_mode color_mode,
		u8 rotation)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	_dispc_set_scaling_common(plane,
			orig_width, orig_height,
			out_width, out_height,
			ilace, five_taps,
			fieldmode, color_mode,
			rotation);

	_dispc_set_scaling_uv(plane,
		orig_width, orig_height,
		out_width, out_height,
		ilace, five_taps,
		fieldmode, color_mode,
		rotation);
}

static void _dispc_set_rotation_attrs(enum omap_plane plane, u8 rotation,
		bool mirroring, enum omap_color_mode color_mode,
		enum omap_dss_rotation_type type)
{
	bool row_repeat = false;
	int vidrot = 0;

	if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY) {

		if (mirroring) {
			switch (rotation) {
			case OMAP_DSS_ROT_0:
				vidrot = 2;
				break;
			case OMAP_DSS_ROT_90:
				vidrot = 1;
				break;
			case OMAP_DSS_ROT_180:
				vidrot = 0;
				break;
			case OMAP_DSS_ROT_270:
				vidrot = 3;
				break;
			}
		} else {
			switch (rotation) {
			case OMAP_DSS_ROT_0:
				vidrot = 0;
				break;
			case OMAP_DSS_ROT_90:
				vidrot = 1;
				break;
			case OMAP_DSS_ROT_180:
				vidrot = 2;
				break;
			case OMAP_DSS_ROT_270:
				vidrot = 3;
				break;
			}
		}

		if (rotation == OMAP_DSS_ROT_90 || rotation == OMAP_DSS_ROT_270)
			row_repeat = true;
		else
			row_repeat = false;
	}

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), vidrot, 13, 12);
	if (dss_has_feature(FEAT_ROWREPEATENABLE))
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane),
			row_repeat ? 1 : 0, 18, 18);

	if (color_mode == OMAP_DSS_COLOR_NV12) {
		/* this will never happen for GFX */
		/* 1D NV12 buffer is always non-rotated or vert. mirrored */
		bool doublestride = (rotation == OMAP_DSS_ROT_0 ||
				     rotation == OMAP_DSS_ROT_180) &&
				     type == OMAP_DSS_ROT_TILER;
		/* DOUBLESTRIDE */
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), doublestride, 22, 22);
	}
}

static int color_mode_to_bpp(enum omap_color_mode color_mode)
{
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
		return 1;
	case OMAP_DSS_COLOR_CLUT2:
		return 2;
	case OMAP_DSS_COLOR_CLUT4:
		return 4;
	case OMAP_DSS_COLOR_CLUT8:
	case OMAP_DSS_COLOR_NV12:
		return 8;
	case OMAP_DSS_COLOR_RGB12U:
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
	case OMAP_DSS_COLOR_RGBA16:
	case OMAP_DSS_COLOR_RGBX16:
	case OMAP_DSS_COLOR_ARGB16_1555:
	case OMAP_DSS_COLOR_XRGB16_1555:
		return 16;
	case OMAP_DSS_COLOR_RGB24P:
		return 24;
	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		return 32;
	default:
		BUG();
	}
}

static s32 pixinc(int pixels, u8 ps)
{
	if (pixels == 1)
		return 1;
	else if (pixels > 1)
		return 1 + (pixels - 1) * ps;
	else if (pixels < 0)
		return 1 - (-pixels + 1) * ps;
	else
		BUG();
}

static void calc_tiler_row_rotation(struct tiler_view_t *view,
		u16 width, int bpp, int y_decim,
		s32 *row_inc, unsigned *offset1, bool ilace)
{
	/* assume TB. We worry about swapping top/bottom outside of this call */

	if (ilace) {
		/* even and odd frames are interleaved */

		/* offset1 is always at an odd line */
		*offset1 = view->v_inc * (y_decim | 1);
		y_decim *= 2;
	}
	*row_inc = view->v_inc * y_decim + 1 - width * bpp;

	DSSDBG(" ps: %d/%d, width: %d/%d, offset1: %d,"
		" height: %d, row_inc:%d\n", view->bpp, bpp,
		view->width, width, *offset1, view->height, *row_inc);

	return;
}

static void calc_vrfb_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc)
{
	u8 ps;

	/* FIXME CLUT formats */
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
	case OMAP_DSS_COLOR_CLUT2:
	case OMAP_DSS_COLOR_CLUT4:
	case OMAP_DSS_COLOR_CLUT8:
		BUG();
		return;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		ps = 4;
		break;
	default:
		ps = color_mode_to_bpp(color_mode) / 8;
		break;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case OMAP_DSS_ROT_0:
	case OMAP_DSS_ROT_180:
		/*
		 * If the pixel format is YUV or UYVY divide the width
		 * of the image by 2 for 0 and 180 degree rotation.
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case OMAP_DSS_ROT_90:
	case OMAP_DSS_ROT_270:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;

		*row_inc = pixinc(1 + (screen_width - width) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case OMAP_DSS_ROT_0 + 4:
	case OMAP_DSS_ROT_180 + 4:
		/* If the pixel format is YUV or UYVY divide the width
		 * of the image by 2  for 0 degree and 180 degree
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case OMAP_DSS_ROT_90 + 4:
	case OMAP_DSS_ROT_270 + 4:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;
		*row_inc = pixinc(1 - (screen_width + width) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	default:
		BUG();
	}
}

static void calc_dma_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc, int x_decim, int y_decim)
{
	u8 ps;
	u16 fbw, fbh;

	/* FIXME CLUT formats */
	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
	case OMAP_DSS_COLOR_CLUT2:
	case OMAP_DSS_COLOR_CLUT4:
	case OMAP_DSS_COLOR_CLUT8:
		BUG();
		return;
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		if (cpu_is_omap44xx()) {
			/* on OMAP4 YUYV is handled as 32-bit data */
			ps = 4;
			screen_width /= 2;
			break;
		}
		/* fall through */
	default:
		ps = color_mode_to_bpp(color_mode) / 8;
		break;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/* width & height are overlay sizes, convert to fb sizes */

	if (rotation == OMAP_DSS_ROT_0 || rotation == OMAP_DSS_ROT_180) {
		fbw = width;
		fbh = height;
	} else {
		fbw = height;
		fbh = width;
	}

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case OMAP_DSS_ROT_0:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 + (y_decim * screen_width - fbw * x_decim) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(x_decim, ps);
		break;
	case OMAP_DSS_ROT_90:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;
	case OMAP_DSS_ROT_180:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-1 -
				(screen_width - fbw) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;
	case OMAP_DSS_ROT_270:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	/* mirroring */
	case OMAP_DSS_ROT_0 + 4:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * 2 - 1 +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;

	case OMAP_DSS_ROT_90 + 4:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	case OMAP_DSS_ROT_180 + 4:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 - screen_width * 2 -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case OMAP_DSS_ROT_270 + 4:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;

	default:
		BUG();
	}
}

static unsigned long calc_fclk_five_taps(enum omap_channel channel, u16 width,
		u16 height, u16 out_width, u16 out_height,
		enum omap_color_mode color_mode)
{
	u32 fclk = 0;
	/* FIXME venc pclk? */
	u64 tmp, pclk = dispc_pclk_rate(channel);

	if (cpu_is_omap44xx()) {
		/* do conservative TRM value on OMAP4 ES1.0 */
		if (omap_rev() == OMAP4430_REV_ES1_0)
			return pclk * DIV_ROUND_UP(width, out_width) *
					DIV_ROUND_UP(height, out_height);

		/* since 4430 ES2.0, fclk requirement only depends on width */
		pclk *= max(width, out_width);
		do_div(pclk, out_width);
		return pclk;
	}

	if (height > out_height) {
		/* FIXME get real display PPL */
		unsigned int ppl = 800;

		tmp = pclk * height * out_width;
		do_div(tmp, 2 * out_height * ppl);
		fclk = tmp;

		if (height > 2 * out_height) {
			if (ppl == out_width)
				return 0;

			tmp = pclk * (height - 2 * out_height) * out_width;
			do_div(tmp, 2 * out_height * (ppl - out_width));
			fclk = max(fclk, (u32) tmp);
		}
	}

	if (width > out_width) {
		tmp = pclk * width;
		do_div(tmp, out_width);
		fclk = max(fclk, (u32) tmp);

		if (color_mode == OMAP_DSS_COLOR_RGB24U)
			fclk <<= 1;
	}

	return fclk;
}

static unsigned long calc_fclk(enum omap_channel channel, u16 width,
		u16 height, u16 out_width, u16 out_height)
{
	unsigned int hf, vf;

	/* on OMAP4 three-tap and five-tap clock requirements are the same */
	if (cpu_is_omap44xx())
		return calc_fclk_five_taps(channel, width, height, out_width,
					out_height, 0);

	/*
	 * FIXME how to determine the 'A' factor
	 * for the no downscaling case ?
	 */

	if (width > 3 * out_width)
		hf = 4;
	else if (width > 2 * out_width)
		hf = 3;
	else if (width > out_width)
		hf = 2;
	else
		hf = 1;

	if (height > out_height)
		vf = 2;
	else
		vf = 1;

	/* FIXME venc pclk? */
	return dispc_pclk_rate(channel) * vf * hf;
}

int dispc_scaling_decision(u16 width, u16 height,
				u16 out_width, u16 out_height,
				enum omap_plane plane,
				enum omap_color_mode color_mode,
				enum omap_channel channel, u8 rotation,
				enum omap_dss_rotation_type type,
				u16 min_x_decim, u16 max_x_decim,
				u16 min_y_decim, u16 max_y_decim,
				u16 *x_decim, u16 *y_decim, bool *five_taps)
{
	int maxdownscale = cpu_is_omap24xx() ? 2 : 4;
	int bpp = color_mode_to_bpp(color_mode);

	/*
	 * For now only whole byte formats on OMAP4 can be predecimated.
	 * Later SDMA decimation support may be added
	 */
	bool can_decimate_x = cpu_is_omap44xx() && !(bpp & 7);
	bool can_decimate_y = can_decimate_x;

	bool can_scale = plane != OMAP_DSS_GFX;

	u16 in_width, in_height;
	unsigned long fclk = 0, fclk5 = 0;
	int min_factor, max_factor;	/* decimation search limits */
	int x, y;			/* decimation search variables */
	unsigned long fclk_max = dispc_fclk_rate();
	u16 y_decim_limit = type == OMAP_DSS_ROT_TILER ? 2 : 16;

	/* No decimation for bitmap formats */
	if (color_mode == OMAP_DSS_COLOR_CLUT1 ||
	    color_mode == OMAP_DSS_COLOR_CLUT2 ||
	    color_mode == OMAP_DSS_COLOR_CLUT4 ||
	    color_mode == OMAP_DSS_COLOR_CLUT8) {
		*x_decim = 1;
		*y_decim = 1;
		*five_taps = false;
		return 0;
	}

	/* restrict search region based on whether we can decimate */
	if (!can_decimate_x) {
		if (min_x_decim > 1)
			return -EINVAL;
		min_x_decim = max_x_decim = 1;
	} else {
		if (max_x_decim > 16)
			max_x_decim = 16;
	}

	if (!can_decimate_y) {
		if (min_y_decim > 1)
			return -EINVAL;
		min_y_decim = max_y_decim = 1;
	} else {
		if (max_y_decim > y_decim_limit)
			max_y_decim = y_decim_limit;
	}

	/*
	 * Find best supported quality.  In the search algorithm, we make use
	 * of the fact, that increased decimation in either direction will have
	 * lower quality.  However, we do not differentiate horizontal and
	 * vertical decimation even though they may affect quality differently
	 * given the exact geometry involved.
	 *
	 * Also, since the clock calculations are abstracted, we cannot make
	 * assumptions on how decimation affects the clock rates in our search.
	 *
	 * We search the whole search region in increasing layers from
	 * min_factor to max_factor.  In each layer we search in increasing
	 * factors alternating between x and y axis:
	 *
	 *   x:	1	2	3
	 * y:
	 * 1	1st |	3rd |	6th |
	 *	----+	    |	    |
	 * 2	2nd	4th |	8th |
	 *	------------+	    |
	 * 3	5th	7th	9th |
	*	--------------------+
	 */
	min_factor = min(min_x_decim, min_y_decim);
	max_factor = max(max_x_decim, max_y_decim);
	x = min_x_decim;
	y = min_y_decim;
	while (1) {
		if (x < min_x_decim || x > max_x_decim ||
			y < min_y_decim || y > max_y_decim)
			goto loop;

		in_width = DIV_ROUND_UP(width, x);
		in_height = DIV_ROUND_UP(height, y);

		if (in_width == out_width && in_height == out_height)
			break;

		if (!can_scale)
			goto loop;

		if (out_width * maxdownscale < in_width ||
			out_height * maxdownscale < in_height)
			goto loop;

		/* Use 5-tap filter unless must use 3-tap */
		if (!cpu_is_omap44xx())
			*five_taps = in_width <= 1024;
		else if (omap_rev() == OMAP4430_REV_ES1_0)
			*five_taps = in_width <= 1280;
		else
			*five_taps = true;

		/*
		 * Predecimation on OMAP4 still fetches the whole lines
		 * :TODO: How does it affect the required clock speed?
		 */
		fclk = calc_fclk(channel, in_width, in_height,
					out_width, out_height);
		fclk5 = *five_taps ?
			calc_fclk_five_taps(channel, in_width, in_height,
					out_width, out_height, color_mode) : 0;

		DSSDBG("%d*%d,%d*%d->%d,%d requires %lu(3T), %lu(5T) Hz\n",
			in_width, x, in_height, y, out_width, out_height,
			fclk, fclk5);

		/* for now we always use 5-tap unless 3-tap is required */
		if (*five_taps)
			fclk = fclk5;

		/* OMAP2/3 has a scaler size limitation */
		if (!cpu_is_omap44xx() && in_width > (1024 << !*five_taps))
			goto loop;

		DSSDBG("required fclk rate = %lu Hz\n", fclk);
		DSSDBG("current fclk rate = %lu Hz\n", fclk_max);

		if (fclk > fclk_max)
			goto loop;
		break;

loop:
		/* err if exhausted search region */
		if (x == max_x_decim && y == max_y_decim) {
			DSSERR("failed to set up scaling %u*%u to %u*%u, "
					"required fclk rate = %lu Hz, "
					"current = %lu Hz\n",
					width, height, out_width, out_height,
					fclk, fclk_max);
			return -EINVAL;
		}

		/* get to next factor */
		if (x == y) {
			x = min_factor;
			y++;
		} else {
			swap(x, y);
			if (x < y)
				x++;
		}
	}

	*x_decim = x;
	*y_decim = y;
	return 0;
}

int dispc_setup_plane(enum omap_plane plane,
		u32 paddr, u16 screen_width,
		u16 pos_x, u16 pos_y,
		u16 width, u16 height,
		u16 out_width, u16 out_height,
		enum omap_color_mode color_mode,
		bool ilace,
		int x_decim, int y_decim, bool five_taps,
		enum omap_dss_rotation_type rotation_type,
		u8 rotation, bool mirror,
		u8 global_alpha, u8 pre_mult_alpha,
		enum omap_channel channel, u32 puv_addr)
{
	const int maxdownscale = cpu_is_omap24xx() ? 2 : 4;
	bool fieldmode = 0;
	int cconv = 0;
	unsigned offset0, offset1;
	s32 row_inc;
	s32 pix_inc;
	u16 frame_height = height;
	unsigned int field_offset = 0;
	int pixpg = (color_mode &
		(OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY)) ? 2 : 1;
	unsigned long tiler_width, tiler_height;
	u32 fifo_high, fifo_low;

	DSSDBG("dispc_setup_plane %d, pa %x, sw %d, %d,%d, %d/%dx%d/%d -> "
	       "%dx%d, ilace %d, cmode %x, rot %d, mir %d chan %d %dtap\n",
	       plane, paddr, screen_width, pos_x, pos_y,
	       width, x_decim, height, y_decim,
	       out_width, out_height,
	       ilace, color_mode,
	       rotation, mirror, channel, five_taps ? 5 : 3);

	if (paddr == 0)
		return -EINVAL;

	if (ilace && height == out_height)
		fieldmode = 1;

	if (ilace) {
		if (fieldmode)
			height /= 2;
		pos_y /= 2;
		out_height /= 2;

		DSSDBG("adjusting for ilace: height %d, pos_y %d, "
				"out_height %d\n",
				height, pos_y, out_height);
	}

	if (!dss_feat_color_mode_supported(plane, color_mode))
		return -EINVAL;

	/* predecimate */

	/* adjust for group-of-pixels*/
	if (rotation & 1)
		height /= pixpg;
	else
		width /= pixpg;

	/* remember tiler block's size as we are reconstructing it */
	tiler_width  = width;
	tiler_height = height;

	width = DIV_ROUND_UP(width, x_decim);
	height = DIV_ROUND_UP(height, y_decim);

	/* NV12 width has to be even (height apparently does not) */
	if (color_mode == OMAP_DSS_COLOR_NV12)
		width &= ~1;

	if (plane == OMAP_DSS_GFX) {
		if (width != out_width || height != out_height)
			return -EINVAL;
	} else {
		/* video plane */

		if (out_width < width / maxdownscale)
			return -EINVAL;

		if (out_height < height / maxdownscale)
			return -EINVAL;

		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY ||
			color_mode == OMAP_DSS_COLOR_NV12)
			cconv = 1;
	}

	if (ilace && !fieldmode) {
		/*
		 * when downscaling the bottom field may have to start several
		 * source lines below the top field. Unfortunately ACCUI
		 * registers will only hold the fractional part of the offset
		 * so the integer part must be added to the base address of the
		 * bottom field.
		 */
		if (!height || height == out_height)
			field_offset = 0;
		else
			field_offset = height / out_height / 2;
	}

	/* Fields are independent but interleaved in memory. */
	if (fieldmode)
		field_offset = 1;

	/* default values */
	row_inc = pix_inc = 0x1;
	offset0 = offset1 = 0x0;

	/*
	 * :HACK: we piggy back on UV separate feature for TILER to avoid
	 * having to keep rebase our FEAT_ enum until they add TILER.
	 */
	if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
		/* set BURSTTYPE */
		bool use_tiler = rotation_type == OMAP_DSS_ROT_TILER;
		REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), use_tiler, 29, 29);
	}

	if (rotation_type == OMAP_DSS_ROT_TILER) {
		struct tiler_view_t view = {0};
		int bpp = color_mode_to_bpp(color_mode) / 8;
		/* tiler needs 0-degree width & height */
		if (rotation & 1)
			swap(tiler_width, tiler_height);

		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
		    color_mode == OMAP_DSS_COLOR_UYVY)
			tiler_width /= 2;

		tilview_create(&view, paddr, tiler_width, tiler_height);
		tilview_rotate(&view, rotation * 90);
		tilview_flip(&view, mirror, false);
		paddr = view.tsptr;

		/* we cannot do TB field interlaced in rotated view */
		pix_inc = 1 + (x_decim - 1) * bpp * pixpg;
		calc_tiler_row_rotation(&view, width * x_decim, bpp * pixpg,
					y_decim, &row_inc, &offset1, ilace);

		DSSDBG("w, h = %ld %ld\n", tiler_width, tiler_height);

		if (puv_addr) {
			tilview_create(&view, puv_addr, tiler_width / 2,
					       tiler_height / 2);
			tilview_rotate(&view, rotation * 90);
			tilview_flip(&view, mirror, false);
			puv_addr = view.tsptr;
		}

	} else if (rotation_type == OMAP_DSS_ROT_DMA) {
		calc_dma_rotation_offset(rotation, mirror,
				screen_width, width, frame_height, color_mode,
				fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc,
				x_decim, y_decim);
	} else {
		calc_vrfb_rotation_offset(rotation, mirror,
				screen_width, width, frame_height, color_mode,
				fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc);
	}

	/* adjust back to pixels */
	if (rotation & 1)
		height *= pixpg;
	else
		width *= pixpg;
	DSSDBG("offset0 %u, offset1 %u, row_inc %d, pix_inc %d\n",
			offset0, offset1, row_inc, pix_inc);

	_dispc_set_color_mode(plane, color_mode);

	_dispc_set_plane_ba0(plane, paddr + offset0);
	_dispc_set_plane_ba1(plane, paddr + offset1);

	if (OMAP_DSS_COLOR_NV12 == color_mode) {
		_dispc_set_plane_ba0_uv(plane, puv_addr + offset0);
		_dispc_set_plane_ba1_uv(plane, puv_addr + offset1);
	}


	_dispc_set_row_inc(plane, row_inc);
	_dispc_set_pix_inc(plane, pix_inc);

	DSSDBG("%d,%d %d*%dx%d*%d -> %dx%d\n", pos_x, pos_y, width, x_decim,
			height, y_decim, out_width, out_height);

	_dispc_set_plane_pos(plane, pos_x, pos_y);

	_dispc_set_pic_size(plane, width, height);

	if (plane != OMAP_DSS_GFX) {
		_dispc_set_scaling(plane, width, height,
				   out_width, out_height,
				   ilace, five_taps, fieldmode,
				   color_mode, rotation);
		_dispc_set_vid_size(plane, out_width, out_height);
		_dispc_set_vid_color_conv(plane, cconv);
	}

	_dispc_set_rotation_attrs(plane, rotation, mirror, color_mode,
							rotation_type);

	_dispc_set_pre_mult_alpha(plane, pre_mult_alpha);
	_dispc_setup_global_alpha(plane, global_alpha);

	if (cpu_is_omap44xx()) {
		fifo_low = dispc_calculate_threshold(plane, paddr + offset0,
				   puv_addr + offset0, width, height,
				   row_inc, pix_inc);
		fifo_high = dispc_get_plane_fifo_size(plane) - 1;
		dispc_setup_plane_fifo(plane, fifo_low, fifo_high);
	}

	return 0;
}

int dispc_enable_plane(enum omap_plane plane, bool enable)
{
	DSSDBG("dispc_enable_plane %d, %d\n", plane, enable);

	REG_FLD_MOD(DISPC_OVL_ATTRIBUTES(plane), enable ? 1 : 0, 0, 0);

	return 0;
}

static void dispc_disable_isr(void *data, u32 mask)
{
	struct completion *compl = data;
	complete(compl);
}

static void _enable_lcd_out(enum omap_channel channel, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, enable ? 1 : 0, 0, 0);
	else
		REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 0, 0);
}

void omap_dispc_set_irq_type(int channel, enum omap_dispc_irq_type type)
{
	if (type == OMAP_DISPC_IRQ_TYPE_VSYNC) {
		dispc.channel_irq[channel] = channel == OMAP_DSS_CHANNEL_LCD2 ?
			DISPC_IRQ_VSYNC2 : DISPC_IRQ_VSYNC;
	} else {
		dispc.channel_irq[channel] = channel == OMAP_DSS_CHANNEL_LCD2 ?
			DISPC_IRQ_FRAMEDONE2 : DISPC_IRQ_FRAMEDONE;
	}
}

static void dispc_enable_lcd_out(enum omap_channel channel, bool enable)
{
	struct completion frame_done_completion;
	bool is_on;
	int r;
	u32 irq;

	/* When we disable LCD output, we need to wait until frame is done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode */
	is_on = channel == OMAP_DSS_CHANNEL_LCD2 ?
			REG_GET(DISPC_CONTROL2, 0, 0) :
			REG_GET(DISPC_CONTROL, 0, 0);

	irq = dispc.channel_irq[channel];

	if (!enable && is_on) {
		init_completion(&frame_done_completion);

		r = omap_dispc_register_isr(dispc_disable_isr,
				&frame_done_completion, irq);

		if (r)
			DSSERR("failed to register FRAMEDONE isr\n");
	}

	_enable_lcd_out(channel, enable);

	if (!enable && is_on) {
		if (!wait_for_completion_timeout(&frame_done_completion,
					msecs_to_jiffies(100)))
			DSSERR("timeout waiting for FRAME DONE\n");

		r = omap_dispc_unregister_isr(dispc_disable_isr,
				&frame_done_completion, irq);

		if (r)
			DSSERR("failed to unregister FRAMEDONE isr\n");
	}
}

static void _enable_digit_out(bool enable)
{
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 1, 1);
}

static void dispc_enable_digit_out(enum omap_display_type type, bool enable)
{
	struct completion frame_done_completion;
	int r;

	if (REG_GET(DISPC_CONTROL, 1, 1) == enable)
		return;

	if (enable) {
		unsigned long flags;
		/* When we enable digit output, we'll get an extra digit
		 * sync lost interrupt, that we need to ignore */
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask &= ~DISPC_IRQ_SYNC_LOST_DIGIT;
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}

	/* When we disable digit output, we need to wait until fields are done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode. And when enabling, we need to
	 * wait for the extra sync losts */
	init_completion(&frame_done_completion);

	r = omap_dispc_register_isr(dispc_disable_isr, &frame_done_completion,
			DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_EVSYNC_ODD
						| DISPC_IRQ_FRAMEDONETV);
	if (r)
		DSSERR("failed to register EVSYNC isr\n");

	_enable_digit_out(enable);

	/* XXX I understand from TRM that we should only wait for the
	 * current field to complete. But it seems we have to wait
	 * for both fields */
	if (!wait_for_completion_timeout(&frame_done_completion,
				msecs_to_jiffies(100)))
		DSSERR("timeout waiting for EVSYNC\n");

	/* Don't wait for the odd field  in the case of HDMI */
	if (type != OMAP_DISPLAY_TYPE_HDMI) {
		if (!wait_for_completion_timeout(&frame_done_completion,
					msecs_to_jiffies(100)))
			DSSERR("timeout waiting for EVSYNC\n");
	}

	r = omap_dispc_unregister_isr(dispc_disable_isr,
			&frame_done_completion,
			DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_EVSYNC_ODD
						| DISPC_IRQ_FRAMEDONETV);
	if (r)
		DSSERR("failed to unregister EVSYNC isr\n");

	if (enable) {
		unsigned long flags;
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask = DISPC_IRQ_MASK_ERROR;
		if (dss_has_feature(FEAT_MGR_LCD2))
			dispc.irq_error_mask |= DISPC_IRQ_SYNC_LOST2;
		if (dss_has_feature(FEAT_OVL_VID3))
			dispc.irq_error_mask |= DISPC_IRQ_VID3_FIFO_UNDERFLOW;
		dispc_write_reg(DISPC_IRQSTATUS, DISPC_IRQ_SYNC_LOST_DIGIT);
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}
}

bool dispc_is_channel_enabled(enum omap_channel channel)
{
	if (channel == OMAP_DSS_CHANNEL_LCD)
		return !!REG_GET(DISPC_CONTROL, 0, 0);
	else if (channel == OMAP_DSS_CHANNEL_DIGIT)
		return !!REG_GET(DISPC_CONTROL, 1, 1);
	else if (channel == OMAP_DSS_CHANNEL_LCD2)
		return !!REG_GET(DISPC_CONTROL2, 0, 0);
	else
		BUG();
}

void dispc_enable_channel(enum omap_channel channel,
		enum omap_display_type type, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD ||
			channel == OMAP_DSS_CHANNEL_LCD2)
		dispc_enable_lcd_out(channel, enable);
	else if (channel == OMAP_DSS_CHANNEL_DIGIT)
		dispc_enable_digit_out(type, enable);
	else
		BUG();
}

void dispc_lcd_enable_signal_polarity(bool act_high)
{
	if (!dss_has_feature(FEAT_LCDENABLEPOL))
		return;

	REG_FLD_MOD(DISPC_CONTROL, act_high ? 1 : 0, 29, 29);
}

void dispc_lcd_enable_signal(bool enable)
{
	if (!dss_has_feature(FEAT_LCDENABLESIGNAL))
		return;

	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 28, 28);
}

void dispc_pck_free_enable(bool enable)
{
	if (!dss_has_feature(FEAT_PCKFREEENABLE))
		return;

	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 27, 27);
}

void dispc_enable_fifohandcheck(enum omap_channel channel, bool enable)
{
	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONFIG2, enable ? 1 : 0, 16, 16);
	else
		REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 16, 16);
}


void dispc_set_lcd_display_type(enum omap_channel channel,
		enum omap_lcd_display_type type)
{
	int mode;

	switch (type) {
	case OMAP_DSS_LCD_DISPLAY_STN:
		mode = 0;
		break;

	case OMAP_DSS_LCD_DISPLAY_TFT:
		mode = 1;
		break;

	default:
		BUG();
		return;
	}

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, mode, 3, 3);
	else
		REG_FLD_MOD(DISPC_CONTROL, mode, 3, 3);
}

void dispc_set_loadmode(enum omap_dss_load_mode mode)
{
	REG_FLD_MOD(DISPC_CONFIG, mode, 2, 1);
}


void dispc_set_default_color(enum omap_channel channel, u32 color)
{
	dispc_write_reg(DISPC_DEFAULT_COLOR(channel), color);
}

u32 dispc_get_default_color(enum omap_channel channel)
{
	u32 l;

	BUG_ON(channel != OMAP_DSS_CHANNEL_DIGIT &&
		channel != OMAP_DSS_CHANNEL_LCD &&
		channel != OMAP_DSS_CHANNEL_LCD2);

	l = dispc_read_reg(DISPC_DEFAULT_COLOR(channel));

	return l;
}

void dispc_set_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type type,
		u32 trans_key)
{
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, type, 11, 11);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, type, 13, 13);
	else /* OMAP_DSS_CHANNEL_LCD2 */
		REG_FLD_MOD(DISPC_CONFIG2, type, 11, 11);

	dispc_write_reg(DISPC_TRANS_COLOR(ch), trans_key);
}

void dispc_get_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type *type,
		u32 *trans_key)
{
	if (type) {
		if (ch == OMAP_DSS_CHANNEL_LCD)
			*type = REG_GET(DISPC_CONFIG, 11, 11);
		else if (ch == OMAP_DSS_CHANNEL_DIGIT)
			*type = REG_GET(DISPC_CONFIG, 13, 13);
		else if (ch == OMAP_DSS_CHANNEL_LCD2)
			*type = REG_GET(DISPC_CONFIG2, 11, 11);
		else
			BUG();
	}

	if (trans_key)
		*trans_key = dispc_read_reg(DISPC_TRANS_COLOR(ch));
}

void dispc_enable_trans_key(enum omap_channel ch, bool enable)
{
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 10, 10);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, enable, 12, 12);
	else /* OMAP_DSS_CHANNEL_LCD2 */
		REG_FLD_MOD(DISPC_CONFIG2, enable, 10, 10);
}
void dispc_enable_alpha_blending(enum omap_channel ch, bool enable)
{
	if (!dss_has_feature(FEAT_GLOBAL_ALPHA))
		return;

	/* :NOTE: compatibility mode is not supported on LCD2 */
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 18, 18);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		REG_FLD_MOD(DISPC_CONFIG, enable, 19, 19);
}
bool dispc_alpha_blending_enabled(enum omap_channel ch)
{
	bool enabled;

	if (!dss_has_feature(FEAT_GLOBAL_ALPHA))
		return false;

	if (ch == OMAP_DSS_CHANNEL_LCD)
		enabled = REG_GET(DISPC_CONFIG, 18, 18);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		enabled = REG_GET(DISPC_CONFIG, 19, 19);
	else if (ch == OMAP_DSS_CHANNEL_LCD2)
		enabled = false;
	else
		BUG();

	return enabled;
}


bool dispc_trans_key_enabled(enum omap_channel ch)
{
	bool enabled;

	if (ch == OMAP_DSS_CHANNEL_LCD)
		enabled = REG_GET(DISPC_CONFIG, 10, 10);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		enabled = REG_GET(DISPC_CONFIG, 12, 12);
	else if (ch == OMAP_DSS_CHANNEL_LCD2)
		enabled = REG_GET(DISPC_CONFIG2, 10, 10);
	else
		BUG();

	return enabled;
}

/* valid inputs for gamma are from 1 to 10 that map
  from 0.2 to 2.2 gamma values and 0 for disabled */
int dispc_enable_gamma(enum omap_channel ch, u8 gamma)
{
#ifdef CONFIG_ARCH_OMAP4
	bool enabled;
	u32 i, temp, channel;
	static bool enable[MAX_DSS_MANAGERS];

	enabled = enable[ch];

	switch (ch) {
	case OMAP_DSS_CHANNEL_LCD:
		channel = 0;
		break;
	case OMAP_DSS_CHANNEL_LCD2:
		channel = 1;
		break;
	case OMAP_DSS_CHANNEL_DIGIT:
		channel = 2;
		break;
	default:
		return -EINVAL;
	}

	if (gamma > NO_OF_GAMMA_TABLES || gamma < 0)
		return -EINVAL;

	if (gamma) {
		u8 *tablePtr = gamma_table[gamma - 1];

		for (i = 0; i < GAMMA_TBL_SZ; i++) {
			temp =  tablePtr[i];
			temp =  (i<<24)|(temp|(temp<<8)|(temp<<16));
			dispc_write_reg(DISPC_GAMMA_TABLE + (channel*4), temp);
		}
	}
	enabled = enabled & ~(1 << channel) | (gamma ? (1 << channel) : 0);
	REG_FLD_MOD(DISPC_CONFIG, (enabled & 1), 3, 3);
	REG_FLD_MOD(DISPC_CONFIG, !!(enabled & 6), 9, 9);

	enable[ch] = enabled;

#endif
	return 0;
}


void dispc_set_tft_data_lines(enum omap_channel channel, u8 data_lines)
{
	int code;

	switch (data_lines) {
	case 12:
		code = 0;
		break;
	case 16:
		code = 1;
		break;
	case 18:
		code = 2;
		break;
	case 24:
		code = 3;
		break;
	default:
		BUG();
		return;
	}

	if (channel == OMAP_DSS_CHANNEL_LCD2)
		REG_FLD_MOD(DISPC_CONTROL2, code, 9, 8);
	else
		REG_FLD_MOD(DISPC_CONTROL, code, 9, 8);
}

void dispc_set_parallel_interface_mode(enum omap_channel channel,
		enum omap_parallel_interface_mode mode)
{
	u32 l;
	int stallmode;
	int gpout0 = 1;
	int gpout1;

	switch (mode) {
	case OMAP_DSS_PARALLELMODE_BYPASS:
		stallmode = 0;
		gpout1 = 1;
		break;

	case OMAP_DSS_PARALLELMODE_RFBI:
		stallmode = 1;
		gpout1 = 0;
		break;

	case OMAP_DSS_PARALLELMODE_DSI:
		stallmode = 1;
		gpout1 = 1;
		break;

	default:
		BUG();
		return;
	}

	if (channel == OMAP_DSS_CHANNEL_LCD2) {
		l = dispc_read_reg(DISPC_CONTROL2);
		l = FLD_MOD(l, stallmode, 11, 11);
		dispc_write_reg(DISPC_CONTROL2, l);
	} else {
		l = dispc_read_reg(DISPC_CONTROL);
		l = FLD_MOD(l, stallmode, 11, 11);
		l = FLD_MOD(l, gpout0, 15, 15);
		l = FLD_MOD(l, gpout1, 16, 16);
		dispc_write_reg(DISPC_CONTROL, l);
	}
}

static bool _dispc_lcd_timings_ok(int hsw, int hfp, int hbp,
		int vsw, int vfp, int vbp)
{
	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		if (hsw < 1 || hsw > 64 ||
				hfp < 1 || hfp > 256 ||
				hbp < 1 || hbp > 256 ||
				vsw < 1 || vsw > 64 ||
				vfp < 0 || vfp > 255 ||
				vbp < 0 || vbp > 255)
			return false;
	} else {
		if (hsw < 1 || hsw > 256 ||
				hfp < 1 || hfp > 4096 ||
				hbp < 1 || hbp > 4096 ||
				vsw < 1 || vsw > 256 ||
				vfp < 0 || vfp > 4095 ||
				vbp < 0 || vbp > 4095)
			return false;
	}

	return true;
}

bool dispc_lcd_timings_ok(struct omap_video_timings *timings)
{
	return _dispc_lcd_timings_ok(timings->hsw, timings->hfp,
			timings->hbp, timings->vsw,
			timings->vfp, timings->vbp);
}

static void _dispc_set_lcd_timings(enum omap_channel channel, int hsw,
		int hfp, int hbp, int vsw, int vfp, int vbp)
{
	u32 timing_h, timing_v;

	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		timing_h = FLD_VAL(hsw-1, 5, 0) | FLD_VAL(hfp-1, 15, 8) |
			FLD_VAL(hbp-1, 27, 20);

		timing_v = FLD_VAL(vsw-1, 5, 0) | FLD_VAL(vfp, 15, 8) |
			FLD_VAL(vbp, 27, 20);
	} else {
		timing_h = FLD_VAL(hsw-1, 7, 0) | FLD_VAL(hfp-1, 19, 8) |
			FLD_VAL(hbp-1, 31, 20);

		timing_v = FLD_VAL(vsw-1, 7, 0) | FLD_VAL(vfp, 19, 8) |
			FLD_VAL(vbp, 31, 20);
	}

	dispc_write_reg(DISPC_TIMING_H(channel), timing_h);
	dispc_write_reg(DISPC_TIMING_V(channel), timing_v);
}

/* change name to mode? */
void dispc_set_lcd_timings(enum omap_channel channel,
		struct omap_video_timings *timings)
{
	unsigned xtot, ytot;
	unsigned long ht, vt;

	if (!_dispc_lcd_timings_ok(timings->hsw, timings->hfp,
				timings->hbp, timings->vsw,
				timings->vfp, timings->vbp))
		BUG();

	_dispc_set_lcd_timings(channel, timings->hsw, timings->hfp,
			timings->hbp, timings->vsw, timings->vfp,
			timings->vbp);

	dispc_set_lcd_size(channel, timings->x_res, timings->y_res);

	xtot = timings->x_res + timings->hfp + timings->hsw + timings->hbp;
	ytot = timings->y_res + timings->vfp + timings->vsw + timings->vbp;

	ht = (timings->pixel_clock * 1000) / xtot;
	vt = (timings->pixel_clock * 1000) / xtot / ytot;

	DSSDBG("channel %d xres %u yres %u\n", channel, timings->x_res,
			timings->y_res);
	DSSDBG("pck %u\n", timings->pixel_clock);
	DSSDBG("hsw %d hfp %d hbp %d vsw %d vfp %d vbp %d\n",
			timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	DSSDBG("hsync %luHz, vsync %luHz\n", ht, vt);
}

static void dispc_set_lcd_divisor(enum omap_channel channel, u16 lck_div,
		u16 pck_div)
{
	BUG_ON(lck_div < 1);
	BUG_ON(pck_div < 2);

	dispc_write_reg(DISPC_DIVISORo(channel),
			FLD_VAL(lck_div, 23, 16) | FLD_VAL(pck_div, 7, 0));
}

static void dispc_get_lcd_divisor(enum omap_channel channel, int *lck_div,
		int *pck_div)
{
	u32 l;
	l = dispc_read_reg(DISPC_DIVISORo(channel));
	*lck_div = FLD_GET(l, 23, 16);
	*pck_div = FLD_GET(l, 7, 0);
}

unsigned long dispc_fclk_rate(void)
{
	struct platform_device *dsidev;
	unsigned long r = 0;

	switch (dss_get_dispc_clk_source()) {
	case OMAP_DSS_CLK_SRC_FCK:
		r = clk_get_rate(dispc.dss_clk);
		break;
	case OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(0);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	case OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(1);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	default:
		BUG();
	}

	return r;
}

unsigned long dispc_lclk_rate(enum omap_channel channel)
{
	struct platform_device *dsidev;
	int lcd;
	unsigned long r;
	u32 l;

	l = dispc_read_reg(DISPC_DIVISORo(channel));

	lcd = FLD_GET(l, 23, 16);

	switch (dss_get_lcd_clk_source(channel)) {
	case OMAP_DSS_CLK_SRC_FCK:
		r = clk_get_rate(dispc.dss_clk);
		break;
	case OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(0);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	case OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC:
		dsidev = dsi_get_dsidev_from_id(1);
		r = dsi_get_pll_hsdiv_dispc_rate(dsidev);
		break;
	default:
		BUG();
	}

	return r / lcd;
}

unsigned long dispc_pclk_rate(enum omap_channel channel)
{
	int pcd;
	unsigned long r;
	u32 l;

	if (channel == OMAP_DSS_CHANNEL_LCD ||
	    channel == OMAP_DSS_CHANNEL_LCD2) {
		l = dispc_read_reg(DISPC_DIVISORo(channel));

		pcd = FLD_GET(l, 7, 0);

		r = dispc_lclk_rate(channel);

		return r / pcd;
	} else {
		struct omap_overlay_manager *mgr;
		mgr = omap_dss_get_overlay_manager(channel);
		if (!mgr || !mgr->device)
			return 0;

		return mgr->device->panel.timings.pixel_clock * 1000;
	}
}

void dispc_dump_clocks(struct seq_file *s)
{
	int lcd, pcd;
	u32 l;
	enum omap_dss_clk_source dispc_clk_src = dss_get_dispc_clk_source();
	enum omap_dss_clk_source lcd_clk_src;

	if (dispc_runtime_get())
		return;

	seq_printf(s, "- DISPC -\n");

	seq_printf(s, "dispc fclk source = %s (%s)\n",
			dss_get_generic_clk_source_name(dispc_clk_src),
			dss_feat_get_clk_source_name(dispc_clk_src));

	seq_printf(s, "fck\t\t%-16lu\n", dispc_fclk_rate());

	if (dss_has_feature(FEAT_CORE_CLK_DIV)) {
		seq_printf(s, "- DISPC-CORE-CLK -\n");
		l = dispc_read_reg(DISPC_DIVISOR);
		lcd = FLD_GET(l, 23, 16);

		seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
				(dispc_fclk_rate()/lcd), lcd);
	}
	seq_printf(s, "- LCD1 -\n");

	lcd_clk_src = dss_get_lcd_clk_source(OMAP_DSS_CHANNEL_LCD);

	seq_printf(s, "lcd1_clk source = %s (%s)\n",
		dss_get_generic_clk_source_name(lcd_clk_src),
		dss_feat_get_clk_source_name(lcd_clk_src));

	dispc_get_lcd_divisor(OMAP_DSS_CHANNEL_LCD, &lcd, &pcd);

	seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
			dispc_lclk_rate(OMAP_DSS_CHANNEL_LCD), lcd);
	seq_printf(s, "pck\t\t%-16lupck div\t%u\n",
			dispc_pclk_rate(OMAP_DSS_CHANNEL_LCD), pcd);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		seq_printf(s, "- LCD2 -\n");

		lcd_clk_src = dss_get_lcd_clk_source(OMAP_DSS_CHANNEL_LCD2);

		seq_printf(s, "lcd2_clk source = %s (%s)\n",
			dss_get_generic_clk_source_name(lcd_clk_src),
			dss_feat_get_clk_source_name(lcd_clk_src));

		dispc_get_lcd_divisor(OMAP_DSS_CHANNEL_LCD2, &lcd, &pcd);

		seq_printf(s, "lck\t\t%-16lulck div\t%u\n",
				dispc_lclk_rate(OMAP_DSS_CHANNEL_LCD2), lcd);
		seq_printf(s, "pck\t\t%-16lupck div\t%u\n",
				dispc_pclk_rate(OMAP_DSS_CHANNEL_LCD2), pcd);
	}

	dispc_runtime_put();
}

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
void dispc_dump_irqs(struct seq_file *s)
{
	unsigned long flags;
	struct dispc_irq_stats stats;

	spin_lock_irqsave(&dispc.irq_stats_lock, flags);

	stats = dispc.irq_stats;
	memset(&dispc.irq_stats, 0, sizeof(dispc.irq_stats));
	dispc.irq_stats.last_reset = jiffies;

	spin_unlock_irqrestore(&dispc.irq_stats_lock, flags);

	seq_printf(s, "period %u ms\n",
			jiffies_to_msecs(jiffies - stats.last_reset));

	seq_printf(s, "irqs %d\n", stats.irq_count);
#define PIS(x) \
	seq_printf(s, "%-20s %10d\n", #x, stats.irqs[ffs(DISPC_IRQ_##x)-1]);

	PIS(FRAMEDONE);
	PIS(VSYNC);
	PIS(EVSYNC_EVEN);
	PIS(EVSYNC_ODD);
	PIS(ACBIAS_COUNT_STAT);
	PIS(PROG_LINE_NUM);
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(GFX_END_WIN);
	PIS(PAL_GAMMA_MASK);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID1_END_WIN);
	PIS(VID2_FIFO_UNDERFLOW);
	PIS(VID2_END_WIN);
	if (dss_has_feature(FEAT_OVL_VID3)) {
		PIS(VID3_FIFO_UNDERFLOW);
		PIS(VID3_END_WIN);
	}
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
	PIS(WAKEUP);
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		PIS(FRAMEDONE2);
		PIS(VSYNC2);
		PIS(ACBIAS_COUNT_STAT2);
		PIS(SYNC_LOST2);
	}
#undef PIS
}
#endif

void dispc_dump_regs(struct seq_file *s)
{
	int i, o;
#define DUMPREG(r) seq_printf(s, "%-50s %08x\n", #r, dispc_read_reg(r))

	if (dispc_runtime_get())
		return;

	DUMPREG(DISPC_REVISION);
	DUMPREG(DISPC_SYSCONFIG);
	DUMPREG(DISPC_SYSSTATUS);
	DUMPREG(DISPC_IRQSTATUS);
	DUMPREG(DISPC_IRQENABLE);
	DUMPREG(DISPC_CONTROL);
	DUMPREG(DISPC_CONFIG);
	DUMPREG(DISPC_CAPABLE);
	DUMPREG(DISPC_DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_DEFAULT_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	DUMPREG(DISPC_TRANS_COLOR(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_TRANS_COLOR(OMAP_DSS_CHANNEL_DIGIT));
	DUMPREG(DISPC_LINE_STATUS);
	DUMPREG(DISPC_LINE_NUMBER);
	DUMPREG(DISPC_TIMING_H(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_TIMING_V(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_POL_FREQ(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_DIVISORo(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_GLOBAL_ALPHA))
		DUMPREG(DISPC_GLOBAL_ALPHA);
	DUMPREG(DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_DIGIT));
	DUMPREG(DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_LCD));
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		DUMPREG(DISPC_CONTROL2);
		DUMPREG(DISPC_CONFIG2);
		DUMPREG(DISPC_DEFAULT_COLOR(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_TRANS_COLOR(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_TIMING_H(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_TIMING_V(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_POL_FREQ(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_DIVISORo(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_SIZE_MGR(OMAP_DSS_CHANNEL_LCD2));
	}

	DUMPREG(DISPC_OVL_BA0(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_BA1(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_POSITION(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_SIZE(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_ATTRIBUTES(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_FIFO_THRESHOLD(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_FIFO_SIZE_STATUS(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_ROW_INC(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_PIXEL_INC(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_WINDOW_SKIP(OMAP_DSS_GFX));
	DUMPREG(DISPC_OVL_TABLE_BA(OMAP_DSS_GFX));

	DUMPREG(DISPC_DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD));
	DUMPREG(DISPC_DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD));

	if (dss_has_feature(FEAT_CPR)) {
		DUMPREG(DISPC_CPR_COEF_R(OMAP_DSS_CHANNEL_LCD));
		DUMPREG(DISPC_CPR_COEF_G(OMAP_DSS_CHANNEL_LCD));
		DUMPREG(DISPC_CPR_COEF_B(OMAP_DSS_CHANNEL_LCD));
	}
	if (dss_has_feature(FEAT_MGR_LCD2)) {
		DUMPREG(DISPC_DATA_CYCLE1(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_DATA_CYCLE2(OMAP_DSS_CHANNEL_LCD2));
		DUMPREG(DISPC_DATA_CYCLE3(OMAP_DSS_CHANNEL_LCD2));

		if (dss_has_feature(FEAT_CPR)) {
			DUMPREG(DISPC_CPR_COEF_R(OMAP_DSS_CHANNEL_LCD2));
			DUMPREG(DISPC_CPR_COEF_G(OMAP_DSS_CHANNEL_LCD2));
			DUMPREG(DISPC_CPR_COEF_B(OMAP_DSS_CHANNEL_LCD2));
		}
	}

	if (dss_has_feature(FEAT_PRELOAD))
		DUMPREG(DISPC_OVL_PRELOAD(OMAP_DSS_GFX));

	for (o = OMAP_DSS_VIDEO1; o <= OMAP_DSS_VIDEO3; o++) {
		if (o == OMAP_DSS_VIDEO3 && !dss_has_feature(FEAT_OVL_VID3))
			continue;

		DUMPREG(DISPC_OVL_BA0(o));
		DUMPREG(DISPC_OVL_BA1(o));
		DUMPREG(DISPC_OVL_POSITION(o));
		DUMPREG(DISPC_OVL_SIZE(o));
		DUMPREG(DISPC_OVL_ATTRIBUTES(o));
		DUMPREG(DISPC_OVL_FIFO_THRESHOLD(o));
		DUMPREG(DISPC_OVL_FIFO_SIZE_STATUS(o));
		DUMPREG(DISPC_OVL_ROW_INC(o));
		DUMPREG(DISPC_OVL_PIXEL_INC(o));
		DUMPREG(DISPC_OVL_FIR(o));
		DUMPREG(DISPC_OVL_PICTURE_SIZE(o));
		DUMPREG(DISPC_OVL_ACCU0(o));
		DUMPREG(DISPC_OVL_ACCU1(o));

		for (i = 0; i < 8; i++)
			DUMPREG(DISPC_OVL_FIR_COEF_H(o, i));

		for (i = 0; i < 8; i++)
			DUMPREG(DISPC_OVL_FIR_COEF_HV(o, i));

		for (i = 0; i < 5; i++)
			DUMPREG(DISPC_OVL_CONV_COEF(o, i));

		if (dss_has_feature(FEAT_FIR_COEF_V)) {
			for (i = 0; i < 8; i++)
				DUMPREG(DISPC_OVL_FIR_COEF_V(o, i));
		}

		if (dss_has_feature(FEAT_HANDLE_UV_SEPARATE)) {
			DUMPREG(DISPC_OVL_BA0_UV(o));
			DUMPREG(DISPC_OVL_BA1_UV(o));
			DUMPREG(DISPC_OVL_FIR2(o));
			DUMPREG(DISPC_OVL_ACCU2_0(o));
			DUMPREG(DISPC_OVL_ACCU2_1(o));

			for (i = 0; i < 8; i++)
				DUMPREG(DISPC_OVL_FIR_COEF_H2(o, i));

			for (i = 0; i < 8; i++)
				DUMPREG(DISPC_OVL_FIR_COEF_HV2(o, i));

			for (i = 0; i < 8; i++)
				DUMPREG(DISPC_OVL_FIR_COEF_V2(o, i));
		}
		if (dss_has_feature(FEAT_ATTR2))
			DUMPREG(DISPC_OVL_ATTRIBUTES2(o));

		if (dss_has_feature(FEAT_PRELOAD))
			DUMPREG(DISPC_OVL_PRELOAD(o));
	}

	dispc_runtime_put();
#undef DUMPREG
}

static void _dispc_set_pol_freq(enum omap_channel channel, bool onoff, bool rf,
		bool ieo, bool ipc, bool ihs, bool ivs, u8 acbi, u8 acb)
{
	u32 l = 0;

	DSSDBG("onoff %d rf %d ieo %d ipc %d ihs %d ivs %d acbi %d acb %d\n",
			onoff, rf, ieo, ipc, ihs, ivs, acbi, acb);

	l |= FLD_VAL(onoff, 17, 17);
	l |= FLD_VAL(rf, 16, 16);
	l |= FLD_VAL(ieo, 15, 15);
	l |= FLD_VAL(ipc, 14, 14);
	l |= FLD_VAL(ihs, 13, 13);
	l |= FLD_VAL(ivs, 12, 12);
	l |= FLD_VAL(acbi, 11, 8);
	l |= FLD_VAL(acb, 7, 0);

	dispc_write_reg(DISPC_POL_FREQ(channel), l);
}

void dispc_set_pol_freq(enum omap_channel channel,
		enum omap_panel_config config, u8 acbi, u8 acb)
{
	_dispc_set_pol_freq(channel, (config & OMAP_DSS_LCD_ONOFF) != 0,
			(config & OMAP_DSS_LCD_RF) != 0,
			(config & OMAP_DSS_LCD_IEO) != 0,
			(config & OMAP_DSS_LCD_IPC) != 0,
			(config & OMAP_DSS_LCD_IHS) != 0,
			(config & OMAP_DSS_LCD_IVS) != 0,
			acbi, acb);
}

/* with fck as input clock rate, find dispc dividers that produce req_pck */
void dispc_find_clk_divs(bool is_tft, unsigned long req_pck, unsigned long fck,
		struct dispc_clock_info *cinfo)
{
	u16 pcd_min = is_tft ? 2 : 3;
	unsigned long best_pck;
	u16 best_ld, cur_ld;
	u16 best_pd, cur_pd;

	best_pck = 0;
	best_ld = 0;
	best_pd = 0;

	for (cur_ld = 1; cur_ld <= 255; ++cur_ld) {
		unsigned long lck = fck / cur_ld;

		for (cur_pd = pcd_min; cur_pd <= 255; ++cur_pd) {
			unsigned long pck = lck / cur_pd;
			long old_delta = abs(best_pck - req_pck);
			long new_delta = abs(pck - req_pck);

			if (best_pck == 0 || new_delta < old_delta) {
				best_pck = pck;
				best_ld = cur_ld;
				best_pd = cur_pd;

				if (pck == req_pck)
					goto found;
			}

			if (pck < req_pck)
				break;
		}

		if (lck / pcd_min < req_pck)
			break;
	}

found:
	cinfo->lck_div = best_ld;
	cinfo->pck_div = best_pd;
	cinfo->lck = fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;
}

/* calculate clock rates using dividers in cinfo */
int dispc_calc_clock_rates(unsigned long dispc_fclk_rate,
		struct dispc_clock_info *cinfo)
{
	if (cinfo->lck_div > 255 || cinfo->lck_div == 0)
		return -EINVAL;
	if (cinfo->pck_div < 2 || cinfo->pck_div > 255)
		return -EINVAL;

	cinfo->lck = dispc_fclk_rate / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;

	return 0;
}

int dispc_set_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo)
{
	DSSDBG("lck = %lu (%u)\n", cinfo->lck, cinfo->lck_div);
	DSSDBG("pck = %lu (%u)\n", cinfo->pck, cinfo->pck_div);

	dispc_set_lcd_divisor(channel, cinfo->lck_div, cinfo->pck_div);

	return 0;
}

int dispc_get_clock_div(enum omap_channel channel,
		struct dispc_clock_info *cinfo)
{
	unsigned long fck;

	fck = dispc_fclk_rate();

	cinfo->lck_div = REG_GET(DISPC_DIVISORo(channel), 23, 16);
	cinfo->pck_div = REG_GET(DISPC_DIVISORo(channel), 7, 0);

	cinfo->lck = fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;

	return 0;
}

/* dispc.irq_lock has to be locked by the caller */
static void _omap_dispc_set_irqs(void)
{
	u32 mask;
	u32 old_mask;
	int i;
	struct omap_dispc_isr_data *isr_data;

	mask = dispc.irq_error_mask;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr == NULL)
			continue;

		mask |= isr_data->mask;
	}

	old_mask = dispc_read_reg(DISPC_IRQENABLE);
	/* clear the irqstatus for newly enabled irqs */
	dispc_write_reg(DISPC_IRQSTATUS, (mask ^ old_mask) & mask);

	dispc_write_reg(DISPC_IRQENABLE, mask);
}

int omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	int ret;
	unsigned long flags;
	struct omap_dispc_isr_data *isr_data;

	if (isr == NULL)
		return -EINVAL;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	/* check for duplicate entry */
	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr == isr && isr_data->arg == arg &&
				isr_data->mask == mask) {
			ret = -EINVAL;
			goto err;
		}
	}

	isr_data = NULL;
	ret = -EBUSY;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr != NULL)
			continue;

		isr_data->isr = isr;
		isr_data->arg = arg;
		isr_data->mask = mask;
		ret = 0;

		break;
	}

	if (ret)
		goto err;

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return 0;
err:
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_register_isr);

int omap_dispc_unregister_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	unsigned long flags;
	int ret = -EINVAL;
	struct omap_dispc_isr_data *isr_data;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr != isr || isr_data->arg != arg ||
				isr_data->mask != mask)
			continue;

		/* found the correct isr */

		isr_data->isr = NULL;
		isr_data->arg = NULL;
		isr_data->mask = 0;

		ret = 0;
		break;
	}

	if (ret == 0)
		_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_unregister_isr);

#ifdef DEBUG
static void print_irq_status(u32 status)
{
	if ((status & dispc.irq_error_mask) == 0)
		return;

	printk(KERN_DEBUG "DISPC IRQ: 0x%x: ", status);

#define PIS(x) \
	if (status & DISPC_IRQ_##x) \
		printk(#x " ");
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID2_FIFO_UNDERFLOW);
	if (dss_has_feature(FEAT_OVL_VID3))
		PIS(VID3_FIFO_UNDERFLOW);
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
	if (dss_has_feature(FEAT_MGR_LCD2))
		PIS(SYNC_LOST2);
#undef PIS

	printk("\n");
}
#endif

/* Called from dss.c. Note that we don't touch clocks here,
 * but we presume they are on because we got an IRQ. However,
 * an irq handler may turn the clocks off, so we may not have
 * clock later in the function. */
static irqreturn_t omap_dispc_irq_handler(int irq, void *arg)
{
	int i;
	u32 irqstatus, irqenable;
	u32 handledirqs = 0;
	u32 unhandled_errors;
	struct omap_dispc_isr_data *isr_data;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];

	spin_lock(&dispc.irq_lock);

	irqstatus = dispc_read_reg(DISPC_IRQSTATUS);
	irqenable = dispc_read_reg(DISPC_IRQENABLE);

	/* IRQ is not for us */
	if (!(irqstatus & irqenable)) {
		spin_unlock(&dispc.irq_lock);
		return IRQ_NONE;
	}

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock(&dispc.irq_stats_lock);
	dispc.irq_stats.irq_count++;
	dss_collect_irq_stats(irqstatus, dispc.irq_stats.irqs);
	spin_unlock(&dispc.irq_stats_lock);
#endif

#ifdef DEBUG
	if (dss_debug)
		print_irq_status(irqstatus);
#endif
	/* Ack the interrupt. Do it here before clocks are possibly turned
	 * off */
	dispc_write_reg(DISPC_IRQSTATUS, irqstatus);
	/* flush posted write */
	dispc_read_reg(DISPC_IRQSTATUS);

	/* make a copy and unlock, so that isrs can unregister
	 * themselves */
	memcpy(registered_isr, dispc.registered_isr,
			sizeof(registered_isr));

	spin_unlock(&dispc.irq_lock);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus) {
			isr_data->isr(isr_data->arg, irqstatus);
			handledirqs |= isr_data->mask;
		}
	}

	spin_lock(&dispc.irq_lock);

	unhandled_errors = irqstatus & ~handledirqs & dispc.irq_error_mask;

	if (unhandled_errors) {
		dispc.error_irqs |= unhandled_errors;

		dispc.irq_error_mask &= ~unhandled_errors;
		_omap_dispc_set_irqs();

		schedule_work(&dispc.error_work);
	}

	spin_unlock(&dispc.irq_lock);

	return IRQ_HANDLED;
}

static void dispc_error_worker(struct work_struct *work)
{
	int i;
	u32 errors;
	unsigned long flags;

	spin_lock_irqsave(&dispc.irq_lock, flags);
	errors = dispc.error_irqs;
	dispc.error_irqs = 0;
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	dispc_runtime_get();

	if (errors & DISPC_IRQ_GFX_FIFO_UNDERFLOW) {
		DSSERR("GFX_FIFO_UNDERFLOW, disabling GFX\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 0) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_VID1_FIFO_UNDERFLOW) {
		DSSERR("VID1_FIFO_UNDERFLOW, disabling VID1\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 1) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_VID2_FIFO_UNDERFLOW) {
		DSSERR("VID2_FIFO_UNDERFLOW, disabling VID2\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 2) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_VID3_FIFO_UNDERFLOW) {
		DSSERR("VID3_FIFO_UNDERFLOW, disabling VID3\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 3) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_SYNC_LOST) {
		struct omap_overlay_manager *manager = NULL;
		bool enable = false;

		DSSERR("SYNC_LOST, disabling LCD\n");

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);

			if (mgr->id == OMAP_DSS_CHANNEL_LCD) {
				if(!mgr->device->first_vsync){
					DSSERR("First SYNC_LOST.. ignoring \n");
					break;
				}

				manager = mgr;
				enable = mgr->device->state ==
						OMAP_DSS_DISPLAY_ACTIVE;
				mgr->device->driver->disable(mgr->device);
				break;
			}
		}

		if (manager) {
			struct omap_dss_device *dssdev = manager->device;
			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);

				if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
					continue;

				if (ovl->id != 0 && ovl->manager == manager)
					dispc_enable_plane(ovl->id, 0);
			}

			dispc_go(manager->id);
			mdelay(50);
			if (enable)
				dssdev->driver->enable(dssdev);
		}
	}

	if (errors & DISPC_IRQ_SYNC_LOST_DIGIT) {
		struct omap_overlay_manager *manager = NULL;
		bool enable = false;

		pr_err_ratelimited("SYNC_LOST_DIGIT, disabling TV\n");

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);

			if (mgr->id == OMAP_DSS_CHANNEL_DIGIT) {
				if(!mgr->device->first_vsync){
					DSSERR("First SYNC_LOST..TV ignoring\n");
				}

				manager = mgr;
				enable = mgr->device->state ==
						OMAP_DSS_DISPLAY_ACTIVE;
				mgr->device->sync_lost_error = 1;
				mgr->device->driver->disable(mgr->device);
				mgr->device->sync_lost_error = 0;
				break;
			}
		}

		if (manager) {
			struct omap_dss_device *dssdev = manager->device;
			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);

				if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
					continue;

				if (ovl->id != 0 && ovl->manager == manager)
					dispc_enable_plane(ovl->id, 0);
			}

			dispc_go(manager->id);
			mdelay(50);
			if (enable)
				dssdev->driver->enable(dssdev);
		}
	}

	if (errors & DISPC_IRQ_SYNC_LOST2) {
		struct omap_overlay_manager *manager = NULL;
		bool enable = false;

		DSSERR("SYNC_LOST for LCD2, disabling LCD2\n");

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);

			if (mgr->id == OMAP_DSS_CHANNEL_LCD2) {
				if(!mgr->device->first_vsync){
					DSSERR("First SYNC_LOST.. ignoring \n");
					break;
				}

				manager = mgr;
				enable = mgr->device->state ==
						OMAP_DSS_DISPLAY_ACTIVE;
				mgr->device->driver->disable(mgr->device);
				break;
			}
		}

		if (manager) {
			struct omap_dss_device *dssdev = manager->device;
			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);

				if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
					continue;

				if (ovl->id != 0 && ovl->manager == manager)
					dispc_enable_plane(ovl->id, 0);
			}

			dispc_go(manager->id);
			mdelay(50);
			if (enable)
				dssdev->driver->enable(dssdev);
		}
	}

	if (errors & DISPC_IRQ_OCP_ERR) {
		DSSERR("OCP_ERR\n");
		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);

			if (mgr->caps & OMAP_DSS_OVL_CAP_DISPC)
				mgr->device->driver->disable(mgr->device);
		}
	}

	spin_lock_irqsave(&dispc.irq_lock, flags);
	dispc.irq_error_mask |= errors;
	_omap_dispc_set_irqs();
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	dispc_runtime_put();
}

int omap_dispc_wait_for_irq_timeout(u32 irqmask, unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		return r;

	timeout = wait_for_completion_timeout(&completion, timeout);

	omap_dispc_unregister_isr(dispc_irq_wait_handler, &completion, irqmask);

	if (timeout == 0)
		return -ETIMEDOUT;

	if (timeout == -ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

int omap_dispc_wait_for_irq_interruptible_timeout(u32 irqmask,
		unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = dispc_runtime_get();
	if (r)
		return r;

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		goto done;

	timeout = wait_for_completion_interruptible_timeout(&completion,
			timeout);

	omap_dispc_unregister_isr(dispc_irq_wait_handler, &completion, irqmask);

	if (timeout == 0)
		r = -ETIMEDOUT;
	else  if (timeout == -ERESTARTSYS)
		r = timeout;

done:
	dispc_runtime_put();

	return r;
}

#ifdef CONFIG_OMAP2_DSS_FAKE_VSYNC
void dispc_fake_vsync_irq(void)
{
	u32 irqstatus = DISPC_IRQ_VSYNC;
	int i;

	WARN_ON(!in_interrupt());

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		struct omap_dispc_isr_data *isr_data;
		isr_data = &dispc.registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus)
			isr_data->isr(isr_data->arg, irqstatus);
	}
}
#endif

static void _omap_dispc_initialize_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	memset(dispc.registered_isr, 0, sizeof(dispc.registered_isr));

	dispc.irq_error_mask = DISPC_IRQ_MASK_ERROR;
	if (dss_has_feature(FEAT_MGR_LCD2))
		dispc.irq_error_mask |= DISPC_IRQ_SYNC_LOST2;
	if (dss_has_feature(FEAT_OVL_VID3))
		dispc.irq_error_mask |= DISPC_IRQ_VID3_FIFO_UNDERFLOW;
	/* there's SYNC_LOST_DIGIT waiting after enabling the DSS,
	 * so clear it */
	dispc_write_reg(DISPC_IRQSTATUS, dispc_read_reg(DISPC_IRQSTATUS));

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);
}

void dispc_enable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 2, 4, 3);	/* SIDLEMODE: smart idle */
}

void dispc_disable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 1, 4, 3);	/* SIDLEMODE: no idle */
}

static void _omap_dispc_initial_config(void)
{
	u32 l;

	/* Exclusively enable DISPC_CORE_CLK and set divider to 1 */
	if (dss_has_feature(FEAT_CORE_CLK_DIV)) {
		l = dispc_read_reg(DISPC_DIVISOR);
		/* Use DISPC_DIVISOR.LCD, instead of DISPC_DIVISOR1.LCD */
		l = FLD_MOD(l, 1, 0, 0);
		l = FLD_MOD(l, 1, 23, 16);
		dispc_write_reg(DISPC_DIVISOR, l);
	}

	/* for OMAP4 ERRATUM xxxx: Mstandby and disconnect protocol issue */
	if (cpu_is_omap44xx()) {
		l3_1_clkdm = clkdm_lookup("l3_1_clkdm");
		l3_2_clkdm = clkdm_lookup("l3_2_clkdm");
	}

	/* FUNCGATED */
	if (dss_has_feature(FEAT_FUNCGATED))
		REG_FLD_MOD(DISPC_CONFIG, 1, 9, 9);

	REG_FLD_MOD(DISPC_CONFIG, 1, 17, 17);

	/* L3 firewall setting: enable access to OCM RAM */
	/* XXX this should be somewhere in plat-omap */
	if (cpu_is_omap24xx())
		__raw_writel(0x402000b0, OMAP2_L3_IO_ADDRESS(0x680050a0));

	dispc_set_loadmode(OMAP_DSS_LOAD_FRAME_ONLY);

	dispc_read_plane_fifo_sizes();
}

/* DISPC HW IP initialisation */
static int omap_dispchw_probe(struct platform_device *pdev)
{
	u32 rev;
	int r = 0;
	struct resource *dispc_mem;
	struct clk *clk;

	dispc.pdev = pdev;

	clk = clk_get(&pdev->dev, "dss_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get dss_clk\n");
		r = PTR_ERR(clk);
		goto err_get_clk;
	}

	dispc.dss_clk = clk;

	spin_lock_init(&dispc.irq_lock);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	spin_lock_init(&dispc.irq_stats_lock);
	dispc.irq_stats.last_reset = jiffies;
#endif

	INIT_WORK(&dispc.error_work, dispc_error_worker);

	dispc_mem = platform_get_resource(dispc.pdev, IORESOURCE_MEM, 0);
	if (!dispc_mem) {
		DSSERR("can't get IORESOURCE_MEM DISPC\n");
		r = -EINVAL;
		goto err_ioremap;
	}
	dispc.base = ioremap(dispc_mem->start, resource_size(dispc_mem));
	if (!dispc.base) {
		DSSERR("can't ioremap DISPC\n");
		r = -ENOMEM;
		goto err_ioremap;
	}
	dispc.irq = platform_get_irq(dispc.pdev, 0);
	if (dispc.irq < 0) {
		DSSERR("platform_get_irq failed\n");
		r = -ENODEV;
		goto err_irq;
	}

	r = request_irq(dispc.irq, omap_dispc_irq_handler, IRQF_SHARED,
		"OMAP DISPC", dispc.pdev);
	if (r < 0) {
		DSSERR("request_irq failed\n");
		goto err_irq;
	}

	mutex_init(&dispc.runtime_lock);

	pm_runtime_enable(&pdev->dev);

	r = dispc_runtime_get();
	if (r)
		goto err_runtime_get;

	_omap_dispc_initial_config();

	_omap_dispc_initialize_irq();

	rev = dispc_read_reg(DISPC_REVISION);
	dev_dbg(&pdev->dev, "OMAP DISPC rev %d.%d\n",
	       FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	dispc_runtime_put();

	return 0;

err_runtime_get:
	pm_runtime_disable(&pdev->dev);
	free_irq(dispc.irq, dispc.pdev);
err_irq:
	iounmap(dispc.base);
err_ioremap:
	clk_put(dispc.dss_clk);
err_get_clk:
	return r;
}

static int omap_dispchw_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	clk_put(dispc.dss_clk);

	free_irq(dispc.irq, dispc.pdev);
	iounmap(dispc.base);
	return 0;
}

static struct platform_driver omap_dispchw_driver = {
	.probe          = omap_dispchw_probe,
	.remove         = omap_dispchw_remove,
	.driver         = {
		.name   = "omapdss_dispc",
		.owner  = THIS_MODULE,
	},
};

int dispc_init_platform_driver(void)
{
	return platform_driver_register(&omap_dispchw_driver);
}

void dispc_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omap_dispchw_driver);
}
