#ifndef _ARCH_ARM_PLAT_OMAP_DSSCOMP_H
#define _ARCH_ARM_PLAT_OMAP_DSSCOMP_H

#include <video/omapdss.h>

/* queuing operations */
typedef struct dsscomp_data *dsscomp_t;		/* handle */

dsscomp_t dsscomp_new(struct omap_overlay_manager *mgr);
u32 dsscomp_get_ovls(dsscomp_t comp);
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl);
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl);
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win);
int dsscomp_delayed_apply(dsscomp_t comp);
void dsscomp_drop(dsscomp_t c);

struct tiler_pa_info;
int dsscomp_gralloc_queue(struct dsscomp_setup_dispc_data *d,
			struct tiler_pa_info **pas,
			bool early_callback,
			void (*cb_fn)(void *, int), void *cb_arg);
#endif
