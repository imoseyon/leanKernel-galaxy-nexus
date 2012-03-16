/* include/linux/sound_control.h */

#ifndef _LINUX_SOUND_CONTROL_H
#define _LINUX_SOUND_CONTROL_H

void soundcontrol_updatevolume(unsigned int volumeboost);
void soundcontrol_updateperf(bool highperf_enabled);
void soundcontrol_reportjack(int jack_type);

#endif
