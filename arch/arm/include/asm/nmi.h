#ifndef __ASM_ARM_NMI_H_
#define __ASM_ARM_NMI_H_

#include <linux/notifier.h>

extern struct atomic_notifier_head touch_watchdog_notifier_head;

static inline void touch_nmi_watchdog(void)
{
	atomic_notifier_call_chain(&touch_watchdog_notifier_head, 0, 0);
	touch_softlockup_watchdog();
}

#endif /* __ASM_ARM_NMI_H_ */
