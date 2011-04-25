/*
 * tcm_utils.h
 *
 * Utility functions for implementing TILER container managers.
 *
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TCM_UTILS_H
#define TCM_UTILS_H

#include "../tcm.h"

/* TCM_ALG_NAME must be defined to use the debug methods */

#ifdef DEBUG
#define IFDEBUG(x) x
#else
/* compile-check debug statements even if not DEBUG */
#define IFDEBUG(x) do { if (0) x; } while (0)
#endif

#define P(level, fmt, ...) \
	IFDEBUG(printk(level TCM_ALG_NAME ":%d:%s()" fmt "\n", \
			__LINE__, __func__, ##__VA_ARGS__))

#define P1(fmt, ...) P(KERN_NOTICE, fmt, ##__VA_ARGS__)
#define P2(fmt, ...) P(KERN_INFO, fmt, ##__VA_ARGS__)
#define P3(fmt, ...) P(KERN_DEBUG, fmt, ##__VA_ARGS__)

#define PA(level, msg, p_area) P##level(msg " (%03d %03d)-(%03d %03d)\n", \
	(p_area)->p0.x, (p_area)->p0.y, (p_area)->p1.x, (p_area)->p1.y)

/* assign coordinates to area */
static inline
void assign(struct tcm_area *a, u16 x0, u16 y0, u16 x1, u16 y1)
{
	a->p0.x = x0;
	a->p0.y = y0;
	a->p1.x = x1;
	a->p1.y = y1;
}

#endif
