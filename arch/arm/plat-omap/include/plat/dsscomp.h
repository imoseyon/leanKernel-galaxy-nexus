#ifndef _ARCH_ARM_PLAT_OMAP_DSSCOMP_H
#define _ARCH_ARM_PLAT_OMAP_DSSCOMP_H

#include <video/omapdss.h>

/* queuing operations */
typedef struct dsscomp_data *dsscomp_t;		/* handle */

dsscomp_t dsscomp_new_sync_id(struct omap_overlay_manager *mgr, u32 sync_id);
u32 dsscomp_first_sync_id(struct omap_overlay_manager *mgr);
dsscomp_t dsscomp_find(struct omap_overlay_manager *mgr, u32 sync_id);
u32 dsscomp_get_ovls(dsscomp_t comp);
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl);
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl);
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win);
int dsscomp_apply(dsscomp_t comp);
int dsscomp_wait(dsscomp_t comp, enum dsscomp_wait_phase phase, int timeout);
void dsscomp_drop(dsscomp_t c);

#endif
