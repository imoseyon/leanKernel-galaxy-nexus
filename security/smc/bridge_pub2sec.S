/**
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

.text

#define SMICODEPUB_IRQ_END   0xFE
#define SMICODEPUB_FIQ_END   0xFD
#define SMICODEPUB_RPC_END   0xFC

#define PUB2SEC_NOCST        0xFF
#define SMICODEPUB_NEWTASK   0x00

/*
 * RPC status:
 *  - 0: the secure world yielded due to an interrupt
 *  - 1: the secure world yielded on an RPC (no public thread is handling it)
 *  - 2: the secure world yielded on an RPC and the response is ready
 */
#define RPC_ADVANCEMENT_NONE		0
#define RPC_ADVANCEMENT_PENDING		1
#define RPC_ADVANCEMENT_FINISHED	2

#ifdef CONFIG_ARM_ERRATA_430973
#define INVALIDATE_BTB       MCR p15, 0, R0, c7, c5, 6
#else
#define INVALIDATE_BTB
#endif

schedule_secure_world:
	.global schedule_secure_world

	/* Save registers */
	push	{r4-r12, lr}

	/* Copy the Secure Service ID in r12 */
	mov     r12, r0

	cmp	r0, #SMICODEPUB_IRQ_END
	beq	return_from_irq

	cmp	r0, #SMICODEPUB_RPC_END
	beq	return_from_rpc

	mov	r6, #PUB2SEC_NOCST
	mov	r12, #SMICODEPUB_NEWTASK

	b	label_smc

return_from_rpc:
	ldr	r9, =g_RPC_parameters
	ldm	r9, {r0-r3}
	/* fall through */

return_from_irq:
	ldr	r10, =g_secure_task_id
	ldr	r6, [r10]

	b	label_smc

label_smc:
	INVALIDATE_BTB
	dsb
	dmb

#ifdef CONFIG_BENCH_SECURE_CYCLE
	/* Come from Non Secure: activate counter 1 (write to 0 are ignored) */
	mov	r4, #0x00000002

	/* Read Count Enable Set Register */
	mcr	p15, 0x0, r4, c9, c12, 1

	/* Come from Non Secure: stop counter 0 (write to 0 are ignored) */
	mov	r4, #0x00000001

	/* Write Count Enable Clear Register */
	mcr	p15, 0x0, r4, c9, c12, 2
#endif

	smc	#0
	b	service_end
	nop

#ifdef CONFIG_BENCH_SECURE_CYCLE
	/* Come from Secure: activate counter 0 (write to 0 are ignored) */
	mov	r4, #0x00000001

	/* Write Count Enable Set Register */
	mcr	p15, 0x0, r4, c9, c12, 1

	/* Come from Secure: stop counter 1 (write to 0 are ignored) */
	mov	r4, #0x00000002

	/* Write Count Enable Clear Register */
	mcr	p15, 0x0, r4, c9, c12, 2
#endif

	INVALIDATE_BTB
	ldr	r8, =g_secure_task_id
	str	r6, [r8]

	mov	r0, #0x00
	ldr	r8, =g_service_end
	str	r0, [r8]

	b	schedule_secure_world_exit

service_end:

schedule_secure_world_exit:
#ifdef CONFIG_BENCH_SECURE_CYCLE
	/* Come from Secure: activate counter 0 (write to 0 are ignored) */
	mov	r4, #0x00000001

	/* Write Count Enable Set Register */
	mcr	p15, 0x0, r4, c9, c12, 1

	/* Come from Secure: stop counter 1 (write to 0 are ignored) */
	mov	r4, #0x00000002

	/* Write Count Enable Clear Register */
	mcr	p15, 0x0, r4, c9, c12, 2
#endif

	INVALIDATE_BTB

	/* Restore registers */
	pop	{r4-r12, pc}

rpc_handler:
	.global rpc_handler

#ifdef CONFIG_BENCH_SECURE_CYCLE
	/* Come from Secure: activate counter 0 (write to 0 are ignored) */
	mov	r4, #0x00000001

	/* Write Count Enable Set Register */
	mcr	p15, 0x0, r4, c9, c12, 1

	/* Come from Secure: stop counter 1 (write to 0 are ignored) */
	mov	r4, #0x00000002

	/* Write Count Enable Clear Register */
	mcr	p15, 0x0, r4, c9, c12, 2
#endif
	INVALIDATE_BTB

	/* g_RPC_advancement = RPC_ADVANCEMENT_PENDING */
	ldr	r8, =g_RPC_advancement
	mov	r9, #RPC_ADVANCEMENT_PENDING
	str	r9, [r8]

	ldr	r8, =g_RPC_parameters
	stm	r8, {r0-r3}

	ldr	r8, =g_secure_task_id
	str	r6, [r8]

	mov	r0, #0x00
	ldr	r8, =g_service_end
	str	r0, [r8]

	/* Restore registers */
	pop	{r4-r12, pc}

#ifdef CONFIG_BENCH_SECURE_CYCLE

setup_counters:
	.global setup_counters

	push	{r14}

	mrc	p15, 0, r2, c9, c12, 0
	orr	r2, r2, #0x3
	mcr	p15, 0, r2, c9, c12, 0

	mrc	p15, 0, r2, c9, c12, 1
	orr	r2, r2, #0x80000000
	mcr	p15, 0, r2, c9, c12, 1

	pop	{pc}

run_code_speed:
	.global run_code_speed

	push	{r14}

	/*  Reset cycle counter */
	mov	r2, #0
	mcr	p15, 0, r2, c9, c13, 0

run_code_speed_loop:
	sub	r0, r0, #1
	cmp	r0, #0
	bne	run_code_speed_loop

	/* Read cycle counter */
	mrc	p15, 0, r0, c9, c13, 0

	pop	{pc}

run_data_speed:
	.global run_data_speed

	push	{r14}

	/* Reset cycle counter */
	mov	r2, #0
	mcr	p15, 0, r2, c9, c13, 0

run_data_speed_loop:
	sub	r0, r0, #1
	ldr	r2, [r1]
	cmp	r0, #0
	bne	run_data_speed_loop

	/* read cycle counter */
	mrc	p15, 0, r0, c9, c13, 0

	pop	{pc}

#endif

read_mpidr:
	.global read_mpidr
	mrc   p15, 0, r0, c0, c0, 5
	bx   lr
