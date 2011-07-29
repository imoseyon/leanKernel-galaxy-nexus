/*
 * Remote processor messaging
 *
 * Copyright(c) 2011 Texas Instruments. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LINUX_RPMSG_RESMGR_H
#define _LINUX_RPMSG_RESMGR_H

#define MAX_NUM_SDMA_CHANNELS	16

enum {
	RPRM_GPTIMER	= 0,
	RPRM_IVAHD	= 1,
	RPRM_IVASEQ0	= 2,
	RPRM_IVASEQ1	= 3,
	RPRM_L3BUS	= 4,
	RPRM_ISS	= 5,
	RPRM_FDIF	= 6,
	RPRM_SL2IF	= 7,
	RPRM_AUXCLK	= 8,
	RPRM_REGULATOR	= 9,
	RPRM_GPIO	= 10,
	RPRM_SDMA	= 11,
	RPRM_IPU	= 12,
	RPRM_DSP	= 13,
	RPRM_I2C	= 14,
	RPRM_MAX
};

enum {
	RPRM_CONNECT		= 0,
	RPRM_REQ_ALLOC		= 1,
	RPRM_REQ_FREE		= 2,
	RPRM_DISCONNECT		= 3,
	RPRM_REQ_CONSTRAINTS	= 4,
	RPRM_REL_CONSTRAINTS	= 5,
};

enum {
	RPRM_SCALE		= 0x1,
	RPRM_LATENCY		= 0x2,
	RPRM_BANDWIDTH		= 0x4,
};

struct rprm_request {
	u32 res_type;
	u32 acquire;
	u32 res_id;
	char data[];
} __packed;

struct rprm_ack {
	u32 ret;
	u32 res_type;
	u32 res_id;
	u32 base;
	char data[];
} __packed;

struct rprm_gpt {
	u32 id;
	u32 src_clk;
};

struct rprm_auxclk {
	u32 id;
	u32 clk_rate;
	u32 parent_src_clk;
	u32 parent_src_clk_rate;
};

struct rprm_regulator {
	u32 id;
	u32 min_uv;
	u32 max_uv;
};

struct rprm_gpio {
	u32 id;
};

/**
 * struct rprm_i2c - resource i2c
 * @id:	i2c id
 *
 * meant to store the i2c related information
 */
struct rprm_i2c {
	u32 id;
};

struct rprm_sdma {
	u32 num_chs;
	s32 channels[MAX_NUM_SDMA_CHANNELS];
};

struct rprm_constraints_data {
	u32 mask;
	long frequency;
	long bandwidth;
	long latency;
};

#endif /* _LINUX_RPMSG_RESMGR_H */
