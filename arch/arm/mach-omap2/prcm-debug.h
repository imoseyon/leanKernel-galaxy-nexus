#ifndef __ARCH_ASM_MACH_OMAP2_PRCM_DEBUG_H
#define __ARCH_ASM_MACH_OMAP2_PRCM_DEBUG_H

#define PRCMDEBUG_LASTSLEEP	(1 << 0)
#define PRCMDEBUG_ON		(1 << 1)

#ifdef CONFIG_PM_DEBUG
extern void prcmdebug_dump(int flags);
#else
static inline void prcmdebug_dump(int flags) { }
#endif

#endif /* __ARCH_ASM_MACH_OMAP2_PRCM_DEBUG_H */
