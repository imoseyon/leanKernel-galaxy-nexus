/**
 * Copyright (c) 2010 Trusted Logic S.A.
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

#include <asm/div64.h>
#include <asm/system.h>
#include <asm/cputype.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>
#include <linux/cpu.h>

#include <asm/cacheflush.h>

#include "tf_defs.h"
#include "tf_comm.h"
#include "tf_util.h"
#include "tf_conn.h"
#include "tf_zebra.h"
#include "tf_crypto.h"

/*--------------------------------------------------------------------------
 * Internal constants
 *-------------------------------------------------------------------------- */

/* RPC commands */
#define RPC_CMD_YIELD	0x00
#define RPC_CMD_INIT	0x01
#define RPC_CMD_TRACE	0x02

/* RPC return values to secure world */
#define RPC_SUCCESS			0x00000000
#define RPC_ERROR_BAD_PARAMETERS	0xFFFF0006
#define RPC_ERROR_CONNECTION_PROTOCOL	0xFFFF3020

/*
 * RPC call status
 *
 * 0: the secure world yielded due to an interrupt
 * 1: the secure world yielded on an RPC (no public world thread is handling it)
 * 2: the secure world yielded on an RPC and the response to that RPC is now in
 *    place
 */
#define RPC_ADVANCEMENT_NONE		0
#define RPC_ADVANCEMENT_PENDING		1
#define RPC_ADVANCEMENT_FINISHED	2

u32 g_RPC_advancement;
u32 g_RPC_parameters[4] = {0, 0, 0, 0};
u32 g_secure_task_id;
u32 g_service_end;

/*
 * Secure ROMCode HAL API Identifiers
 */
#define API_HAL_SDP_RUNTIMEINIT_INDEX           0x04
#define API_HAL_LM_PALOAD_INDEX                 0x05
#define API_HAL_LM_PAUNLOADALL_INDEX            0x07
#define API_HAL_TASK_MGR_RPCINIT_INDEX          0x08
#define API_HAL_KM_GETSECUREROMCODECRC_INDEX    0x0B
#define API_HAL_SEC_L3_RAM_RESIZE_INDEX         0x17

#define API_HAL_RET_VALUE_OK	0x0

/* SE entry flags */
#define FLAG_START_HAL_CRITICAL     0x4
#define FLAG_IRQFIQ_MASK            0x3
#define FLAG_IRQ_ENABLE             0x2
#define FLAG_FIQ_ENABLE             0x1

#define SMICODEPUB_IRQ_END	0xFE
#define SMICODEPUB_FIQ_END	0xFD
#define SMICODEPUB_RPC_END	0xFC

#define SEC_RAM_SIZE_40KB	0x0000A000
#define SEC_RAM_SIZE_48KB	0x0000C000
#define SEC_RAM_SIZE_52KB	0x0000D000
#define SEC_RAM_SIZE_60KB	0x0000F000
#define SEC_RAM_SIZE_64KB	0x00010000

struct tf_ns_pa_info {
	void *certificate;
	void *parameters;
	void *results;
};

/*
 * AFY: I would like to remove the L0 buffer altogether:
 * - you can use the L1 shared buffer to pass the RPC parameters and results:
 *    I think these easily fit in 256 bytes and you can use the area at
 *    offset 0x2C0-0x3BF in the L1 shared buffer
 */
struct tf_init_buffer {
	u32 init_status;
	u32 protocol_version;
	u32 l1_shared_buffer_descr;
	u32 backing_store_addr;
	u32 backext_storage_addr;
	u32 workspace_addr;
	u32 workspace_size;
	u32 properties_length;
	u8 properties_buffer[1];
};

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock g_tf_wake_lock;
static u32 tf_wake_lock_count = 0;
#endif

static struct clockdomain *smc_l4_sec_clkdm;
static u32 smc_l4_sec_clkdm_use_count = 0;

static int __init tf_early_init(void)
{
	g_secure_task_id = 0;

	dprintk(KERN_INFO "SMC early init\n");

	smc_l4_sec_clkdm = clkdm_lookup("l4_secure_clkdm");
	if (smc_l4_sec_clkdm == NULL)
		return -EFAULT;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&g_tf_wake_lock, WAKE_LOCK_SUSPEND,
		TF_DEVICE_BASE_NAME);
#endif

	return 0;
}
early_initcall(tf_early_init);

/*
 * Function responsible for formatting parameters to pass from NS world to
 * S world
 */
u32 omap4_secure_dispatcher(u32 app_id, u32 flags, u32 nargs,
	u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	u32 ret;
	unsigned long iflags;
	u32 pub2sec_args[5] = {0, 0, 0, 0, 0};

	/*dprintk(KERN_INFO "omap4_secure_dispatcher: "
		"app_id=0x%08x, flags=0x%08x, nargs=%u\n",
		app_id, flags, nargs);*/

	/*if (nargs != 0)
		dprintk(KERN_INFO
		"omap4_secure_dispatcher: args=%08x, %08x, %08x, %08x\n",
		arg1, arg2, arg3, arg4);*/

	pub2sec_args[0] = nargs;
	pub2sec_args[1] = arg1;
	pub2sec_args[2] = arg2;
	pub2sec_args[3] = arg3;
	pub2sec_args[4] = arg4;

	/* Make sure parameters are visible to the secure world */
	dmac_flush_range((void *)pub2sec_args,
		(void *)(((u32)(pub2sec_args)) + 5*sizeof(u32)));
	outer_clean_range(__pa(pub2sec_args),
		__pa(pub2sec_args) + 5*sizeof(u32));
	wmb();

	/*
	 * Put L4 Secure clock domain to SW_WKUP so that modules are accessible
	 */
	tf_l4sec_clkdm_wakeup(false);

	local_irq_save(iflags);
#ifdef DEBUG
	BUG_ON((read_mpidr() & 0x00000003) != 0);
#endif
	/* proc_id is always 0 */
	ret = schedule_secure_world(app_id, 0, flags, __pa(pub2sec_args));
	local_irq_restore(iflags);

	/* Restore the HW_SUP on L4 Sec clock domain so hardware can idle */
	tf_l4sec_clkdm_allow_idle(false);

	/*dprintk(KERN_INFO "omap4_secure_dispatcher()\n");*/

	return ret;
}

/* Yields the Secure World */
int tf_schedule_secure_world(struct tf_comm *comm, bool prepare_exit)
{
	int status = 0;
	int ret;
	unsigned long iflags;
	u32 appli_id;

	tf_set_current_time(comm);

	local_irq_save(iflags);

	switch (g_RPC_advancement) {
	case  RPC_ADVANCEMENT_NONE:
		/* Return from IRQ */
		appli_id = SMICODEPUB_IRQ_END;
		if (prepare_exit)
			status = STATUS_PENDING;
		break;
	case  RPC_ADVANCEMENT_PENDING:
		/* nothing to do in this case */
		goto exit;
	default:
	case RPC_ADVANCEMENT_FINISHED:
		if (prepare_exit)
			goto exit;
		appli_id = SMICODEPUB_RPC_END;
		g_RPC_advancement = RPC_ADVANCEMENT_NONE;
		break;
	}

	g_service_end = 1;
	/* yield to the Secure World */
	ret = omap4_secure_dispatcher(appli_id, /* app_id */
	   0, 0,        /* flags, nargs */
	   0, 0, 0, 0); /* arg1, arg2, arg3, arg4 */
	if (g_service_end != 0) {
		dprintk(KERN_ERR "Service End ret=%X\n", ret);

		if (ret == 0) {
			dmac_flush_range((void *)comm->init_shared_buffer,
				(void *)(((u32)(comm->init_shared_buffer)) +
					PAGE_SIZE));
			outer_inv_range(__pa(comm->init_shared_buffer),
				__pa(comm->init_shared_buffer) +
				PAGE_SIZE);

			ret = ((struct tf_init_buffer *)
				(comm->init_shared_buffer))->init_status;

			dprintk(KERN_ERR "SMC PA failure ret=%X\n", ret);
			if (ret == 0)
				ret = -EFAULT;
		}
		clear_bit(TF_COMM_FLAG_PA_AVAILABLE, &comm->flags);
		omap4_secure_dispatcher(API_HAL_LM_PAUNLOADALL_INDEX,
			FLAG_START_HAL_CRITICAL, 0, 0, 0, 0, 0);
		status = ret;
	}

exit:
	local_irq_restore(iflags);

	return status;
}

/* Initializes the SE (SDP, SRAM resize, RPC handler) */
static int tf_se_init(struct tf_comm *comm,
	u32 sdp_backing_store_addr, u32 sdp_bkext_store_addr)
{
	int error;
	unsigned int crc;

	if (comm->se_initialized) {
		dprintk(KERN_INFO "tf_se_init: SE already initialized... "
			"nothing to do\n");
		return 0;
	}

	/* Secure CRC read */
	dprintk(KERN_INFO "tf_se_init: Secure CRC Read...\n");

	crc = omap4_secure_dispatcher(API_HAL_KM_GETSECUREROMCODECRC_INDEX,
		0, 0, 0, 0, 0, 0);
	printk(KERN_INFO "SMC: SecureCRC=0x%08X\n", crc);

	/*
	 * Flush caches before resize, just to be sure there is no
	 * pending public data writes back to SRAM that could trigger a
	 * security violation once their address space is marked as
	 * secure.
	 */
#define OMAP4_SRAM_PA   0x40300000
#define OMAP4_SRAM_SIZE 0xe000
	flush_cache_all();
	outer_flush_range(OMAP4_SRAM_PA,
			OMAP4_SRAM_PA + OMAP4_SRAM_SIZE);
	wmb();

	/* SRAM resize */
	dprintk(KERN_INFO "tf_se_init: SRAM resize (52KB)...\n");
	error = omap4_secure_dispatcher(API_HAL_SEC_L3_RAM_RESIZE_INDEX,
		FLAG_FIQ_ENABLE | FLAG_START_HAL_CRITICAL, 1,
		SEC_RAM_SIZE_52KB, 0, 0, 0);

	if (error == API_HAL_RET_VALUE_OK) {
		dprintk(KERN_INFO "tf_se_init: SRAM resize OK\n");
	} else {
		dprintk(KERN_ERR "tf_se_init: "
			"SRAM resize failed [0x%x]\n", error);
		goto error;
	}

	/* SDP init */
	dprintk(KERN_INFO "tf_se_init: SDP runtime init..."
		"(sdp_backing_store_addr=%x, sdp_bkext_store_addr=%x)\n",
		sdp_backing_store_addr, sdp_bkext_store_addr);
	error = omap4_secure_dispatcher(API_HAL_SDP_RUNTIMEINIT_INDEX,
		FLAG_FIQ_ENABLE | FLAG_START_HAL_CRITICAL, 2,
		sdp_backing_store_addr, sdp_bkext_store_addr, 0, 0);

	if (error == API_HAL_RET_VALUE_OK) {
		dprintk(KERN_INFO "tf_se_init: SDP runtime init OK\n");
	} else {
		dprintk(KERN_ERR "tf_se_init: "
			"SDP runtime init failed [0x%x]\n", error);
		goto error;
	}

	/* RPC init */
	dprintk(KERN_INFO "tf_se_init: RPC init...\n");
	error = omap4_secure_dispatcher(API_HAL_TASK_MGR_RPCINIT_INDEX,
		FLAG_START_HAL_CRITICAL, 1,
		(u32) (u32(*const) (u32, u32, u32, u32)) &rpc_handler, 0, 0, 0);

	if (error == API_HAL_RET_VALUE_OK) {
		dprintk(KERN_INFO "tf_se_init: RPC init OK\n");
	} else {
		dprintk(KERN_ERR "tf_se_init: "
			"RPC init failed [0x%x]\n", error);
		goto error;
	}

	comm->se_initialized = true;

	return 0;

error:
	return -EFAULT;
}

/* Check protocol version returned by the PA */
static u32 tf_rpc_init(struct tf_comm *comm)
{
	u32 protocol_version;
	u32 rpc_error = RPC_SUCCESS;

	dprintk(KERN_INFO "tf_rpc_init(%p)\n", comm);

	spin_lock(&(comm->lock));

	dmac_flush_range((void *)comm->init_shared_buffer,
		(void *)(((u32)(comm->init_shared_buffer)) + PAGE_SIZE));
	outer_inv_range(__pa(comm->init_shared_buffer),
		__pa(comm->init_shared_buffer) +  PAGE_SIZE);

	protocol_version = ((struct tf_init_buffer *)
				(comm->init_shared_buffer))->protocol_version;

	if ((GET_PROTOCOL_MAJOR_VERSION(protocol_version))
			!= TF_S_PROTOCOL_MAJOR_VERSION) {
		dprintk(KERN_ERR "SMC: Unsupported SMC Protocol PA Major "
			"Version (0x%02x, expected 0x%02x)!\n",
			GET_PROTOCOL_MAJOR_VERSION(protocol_version),
			TF_S_PROTOCOL_MAJOR_VERSION);
		rpc_error = RPC_ERROR_CONNECTION_PROTOCOL;
	} else {
		rpc_error = RPC_SUCCESS;
	}

	spin_unlock(&(comm->lock));

	register_smc_public_crypto_digest();
	register_smc_public_crypto_aes();

	return rpc_error;
}

static u32 tf_rpc_trace(struct tf_comm *comm)
{
	dprintk(KERN_INFO "tf_rpc_trace(%p)\n", comm);

#ifdef CONFIG_SECURE_TRACE
	spin_lock(&(comm->lock));
	printk(KERN_INFO "SMC PA: %s",
		comm->pBuffer->rpc_trace_buffer);
	spin_unlock(&(comm->lock));
#endif
	return RPC_SUCCESS;
}

/*
 * Handles RPC calls
 *
 * Returns:
 *  - RPC_NO if there was no RPC to execute
 *  - RPC_YIELD if there was a Yield RPC
 *  - RPC_NON_YIELD if there was a non-Yield RPC
 */

int tf_rpc_execute(struct tf_comm *comm)
{
	u32 rpc_command;
	u32 rpc_error = RPC_NO;

#ifdef DEBUG
	BUG_ON((read_mpidr() & 0x00000003) != 0);
#endif

	/* Lock the RPC */
	mutex_lock(&(comm->rpc_mutex));

	rpc_command = g_RPC_parameters[1];

	if (g_RPC_advancement == RPC_ADVANCEMENT_PENDING) {
		dprintk(KERN_INFO "tf_rpc_execute: "
			"Executing CMD=0x%x\n",
			g_RPC_parameters[1]);

		switch (rpc_command) {
		case RPC_CMD_YIELD:
			dprintk(KERN_INFO "tf_rpc_execute: "
				"RPC_CMD_YIELD\n");

			rpc_error = RPC_YIELD;
			g_RPC_parameters[0] = RPC_SUCCESS;
			break;

		case RPC_CMD_TRACE:
			rpc_error = RPC_NON_YIELD;
			g_RPC_parameters[0] = tf_rpc_trace(comm);
			break;

		default:
			if (tf_crypto_execute_rpc(rpc_command,
				comm->pBuffer->rpc_cus_buffer) != 0)
				g_RPC_parameters[0] = RPC_ERROR_BAD_PARAMETERS;
			else
				g_RPC_parameters[0] = RPC_SUCCESS;
			rpc_error = RPC_NON_YIELD;
			break;
		}
		g_RPC_advancement = RPC_ADVANCEMENT_FINISHED;
	}

	mutex_unlock(&(comm->rpc_mutex));

	dprintk(KERN_INFO "tf_rpc_execute: Return 0x%x\n",
		rpc_error);

	return rpc_error;
}

/*--------------------------------------------------------------------------
 * L4 SEC Clock domain handling
 *-------------------------------------------------------------------------- */

static DEFINE_SPINLOCK(clk_lock);
void tf_l4sec_clkdm_wakeup(bool wakelock)
{
	unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);
#ifdef CONFIG_HAS_WAKELOCK
	if (wakelock) {
		tf_wake_lock_count++;
		wake_lock(&g_tf_wake_lock);
	}
#endif
	smc_l4_sec_clkdm_use_count++;
	clkdm_wakeup(smc_l4_sec_clkdm);
	spin_unlock_irqrestore(&clk_lock, flags);
}

void tf_l4sec_clkdm_allow_idle(bool wakeunlock)
{
	unsigned long flags;
	spin_lock_irqsave(&clk_lock, flags);
	smc_l4_sec_clkdm_use_count--;
	if (smc_l4_sec_clkdm_use_count == 0)
		clkdm_allow_idle(smc_l4_sec_clkdm);
#ifdef CONFIG_HAS_WAKELOCK
	if (wakeunlock){
		tf_wake_lock_count--;
		if (tf_wake_lock_count == 0)
			wake_unlock(&g_tf_wake_lock);
	}
#endif
	spin_unlock_irqrestore(&clk_lock, flags);
}

/*--------------------------------------------------------------------------
 * Power management
 *-------------------------------------------------------------------------- */
 /*
 * Perform a Secure World shutdown operation.
 * The routine does not return if the operation succeeds.
 * the routine returns an appropriate error code if
 * the operation fails.
 */
int tf_pm_shutdown(struct tf_comm *comm)
{

	int error;
	union tf_command command;
	union tf_answer answer;

	dprintk(KERN_INFO "tf_pm_shutdown()\n");

	memset(&command, 0, sizeof(command));

	command.header.message_type = TF_MESSAGE_TYPE_MANAGEMENT;
	command.header.message_size =
			(sizeof(struct tf_command_management) -
				sizeof(struct tf_command_header))/sizeof(u32);

	command.management.command = TF_MANAGEMENT_SHUTDOWN;

	error = tf_send_receive(
		comm,
		&command,
		&answer,
		NULL,
		false);

	if (error != 0) {
		dprintk(KERN_ERR "tf_pm_shutdown(): "
			"tf_send_receive failed (error %d)!\n",
			error);
		return error;
	}

#ifdef CONFIG_TF_DRIVER_DEBUG_SUPPORT
	if (answer.header.error_code != 0)
		dprintk(KERN_ERR "tf_driver: shutdown failed.\n");
	else
		dprintk(KERN_INFO "tf_driver: shutdown succeeded.\n");
#endif

	return answer.header.error_code;
}


int tf_pm_hibernate(struct tf_comm *comm)
{
	struct tf_device *dev = tf_get_device();

	dprintk(KERN_INFO "tf_pm_hibernate()\n");

	/*
	 * As we enter in CORE OFF, the keys are going to be cleared.
	 * Reset the global key context.
	 * When the system leaves CORE OFF, this will force the driver to go
	 * through the secure world which will reconfigure the accelerators.
	 */
	dev->aes1_key_context = 0;
	dev->des_key_context = 0;
#ifndef CONFIG_SMC_KERNEL_CRYPTO
	dev->sham1_is_public = false;
#endif
	return 0;
}

#ifdef CONFIG_SMC_KERNEL_CRYPTO
#define DELAYED_RESUME_NONE	0
#define DELAYED_RESUME_PENDING	1
#define DELAYED_RESUME_ONGOING	2

static DEFINE_SPINLOCK(tf_delayed_resume_lock);
static int tf_need_delayed_resume = DELAYED_RESUME_NONE;

int tf_delayed_secure_resume(void)
{
	int ret;
	union tf_command message;
	union tf_answer answer;
	struct tf_device *dev = tf_get_device();

	spin_lock(&tf_delayed_resume_lock);
	if (likely(tf_need_delayed_resume == DELAYED_RESUME_NONE)) {
		spin_unlock(&tf_delayed_resume_lock);
		return 0;
	}

	if (unlikely(tf_need_delayed_resume == DELAYED_RESUME_ONGOING)) {
		spin_unlock(&tf_delayed_resume_lock);

		/*
		 * Wait for the other caller to actually finish the delayed
		 * resume operation
		 */
		while (tf_need_delayed_resume != DELAYED_RESUME_NONE)
			cpu_relax();

		return 0;
	}

	tf_need_delayed_resume = DELAYED_RESUME_ONGOING;
	spin_unlock(&tf_delayed_resume_lock);

	/*
	 * When the system leaves CORE OFF, HWA are configured as secure.  We
	 * need them as public for the Linux Crypto API.
	 */
	memset(&message, 0, sizeof(message));

	message.header.message_type = TF_MESSAGE_TYPE_MANAGEMENT;
	message.header.message_size =
		(sizeof(struct tf_command_management) -
			sizeof(struct tf_command_header))/sizeof(u32);
	message.management.command =
		TF_MANAGEMENT_RESUME_FROM_CORE_OFF;

	ret = tf_send_receive(&dev->sm, &message, &answer, NULL, false);
	if (ret) {
		printk(KERN_ERR "tf_pm_resume(%p): "
			"tf_send_receive failed (error %d)!\n",
			&dev->sm, ret);

		unregister_smc_public_crypto_digest();
		unregister_smc_public_crypto_aes();
		return ret;
	}

	if (answer.header.error_code) {
		unregister_smc_public_crypto_digest();
		unregister_smc_public_crypto_aes();
	}

	spin_lock(&tf_delayed_resume_lock);
	tf_need_delayed_resume = DELAYED_RESUME_NONE;
	spin_unlock(&tf_delayed_resume_lock);

	return answer.header.error_code;
}
#endif

int tf_pm_resume(struct tf_comm *comm)
{

	dprintk(KERN_INFO "tf_pm_resume()\n");
	#if 0
	{
		void *workspace_va;
		struct tf_device *dev = tf_get_device();
		workspace_va = ioremap(dev->workspace_addr,
			dev->workspace_size);
		printk(KERN_INFO
		"Read first word of workspace [0x%x]\n",
		*(uint32_t *)workspace_va);
	}
	#endif

#ifdef CONFIG_SMC_KERNEL_CRYPTO
	spin_lock(&tf_delayed_resume_lock);
	tf_need_delayed_resume = DELAYED_RESUME_PENDING;
	spin_unlock(&tf_delayed_resume_lock);
#endif
	return 0;
}

/*--------------------------------------------------------------------------
 * Initialization
 *-------------------------------------------------------------------------- */

int tf_init(struct tf_comm *comm)
{
	spin_lock_init(&(comm->lock));
	comm->flags = 0;
	comm->pBuffer = NULL;
	comm->init_shared_buffer = NULL;

	comm->se_initialized = false;

	init_waitqueue_head(&(comm->wait_queue));
	mutex_init(&(comm->rpc_mutex));

	if (tf_crypto_init() != PUBLIC_CRYPTO_OPERATION_SUCCESS)
		return -EFAULT;

	if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
		register_smc_public_crypto_digest();
		register_smc_public_crypto_aes();
	}

	return 0;
}

/* Start the SMC PA */
int tf_start(struct tf_comm *comm,
	u32 workspace_addr, u32 workspace_size,
	u8 *pa_buffer, u32 pa_size,
	u8 *properties_buffer, u32 properties_length)
{
	struct tf_init_buffer *init_shared_buffer = NULL;
	struct tf_l1_shared_buffer *l1_shared_buffer = NULL;
	u32 l1_shared_buffer_descr;
	struct tf_ns_pa_info pa_info;
	int ret;
	u32 descr;
	u32 sdp_backing_store_addr;
	u32 sdp_bkext_store_addr;
#ifdef CONFIG_SMP
	long ret_affinity;
	cpumask_t saved_cpu_mask;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;

	/* OMAP4 Secure ROM Code can only be called from CPU0. */
	cpu_set(0, local_cpu_mask);
	sched_getaffinity(0, &saved_cpu_mask);
	ret_affinity = sched_setaffinity(0, &local_cpu_mask);
	if (ret_affinity != 0)
		dprintk(KERN_ERR "sched_setaffinity #1 -> 0x%lX", ret_affinity);
#endif

	tf_l4sec_clkdm_wakeup(true);

	workspace_size -= SZ_1M;
	sdp_backing_store_addr = workspace_addr + workspace_size;
	workspace_size -= 0x20000;
	sdp_bkext_store_addr = workspace_addr + workspace_size;

	/*
	 * Implementation notes:
	 *
	 * 1/ The PA buffer (pa_buffer)is now owned by this function.
	 *    In case of error, it is responsible for releasing the buffer.
	 *
	 * 2/ The PA Info and PA Buffer will be freed through a RPC call
	 *    at the beginning of the PA entry in the SE.
	 */

	if (test_bit(TF_COMM_FLAG_PA_AVAILABLE, &comm->flags)) {
		dprintk(KERN_ERR "tf_start(%p): "
			"The SMC PA is already started\n", comm);

		ret = -EFAULT;
		goto error1;
	}

	if (sizeof(struct tf_l1_shared_buffer) != PAGE_SIZE) {
		dprintk(KERN_ERR "tf_start(%p): "
			"The L1 structure size is incorrect!\n", comm);
		ret = -EFAULT;
		goto error1;
	}

	ret = tf_se_init(comm, sdp_backing_store_addr,
		sdp_bkext_store_addr);
	if (ret != 0) {
		dprintk(KERN_ERR "tf_start(%p): "
			"SE initialization failed\n", comm);
		goto error1;
	}

	init_shared_buffer =
		(struct tf_init_buffer *)
			internal_get_zeroed_page(GFP_KERNEL);
	if (init_shared_buffer == NULL) {
		dprintk(KERN_ERR "tf_start(%p): "
			"Ouf of memory!\n", comm);

		ret = -ENOMEM;
		goto error1;
	}
	/* Ensure the page is mapped */
	__set_page_locked(virt_to_page(init_shared_buffer));

	l1_shared_buffer =
		(struct tf_l1_shared_buffer *)
			internal_get_zeroed_page(GFP_KERNEL);

	if (l1_shared_buffer == NULL) {
		dprintk(KERN_ERR "tf_start(%p): "
			"Ouf of memory!\n", comm);

		ret = -ENOMEM;
		goto error1;
	}
	/* Ensure the page is mapped */
	__set_page_locked(virt_to_page(l1_shared_buffer));

	dprintk(KERN_INFO "tf_start(%p): "
		"L0SharedBuffer={0x%08x, 0x%08x}\n", comm,
		(u32) init_shared_buffer, (u32) __pa(init_shared_buffer));
	dprintk(KERN_INFO "tf_start(%p): "
		"L1SharedBuffer={0x%08x, 0x%08x}\n", comm,
		(u32) l1_shared_buffer, (u32) __pa(l1_shared_buffer));

	descr = tf_get_l2_descriptor_common((u32) l1_shared_buffer,
			current->mm);
	l1_shared_buffer_descr = (
		((u32) __pa(l1_shared_buffer) & 0xFFFFF000) |
		(descr & 0xFFF));

	pa_info.certificate = (void *) __pa(pa_buffer);
	pa_info.parameters = (void *) __pa(init_shared_buffer);
	pa_info.results = (void *) __pa(init_shared_buffer);

	init_shared_buffer->l1_shared_buffer_descr = l1_shared_buffer_descr;

	init_shared_buffer->backing_store_addr = sdp_backing_store_addr;
	init_shared_buffer->backext_storage_addr = sdp_bkext_store_addr;
	init_shared_buffer->workspace_addr = workspace_addr;
	init_shared_buffer->workspace_size = workspace_size;

	init_shared_buffer->properties_length = properties_length;
	if (properties_length == 0) {
		init_shared_buffer->properties_buffer[0] = 0;
	} else {
		/* Test for overflow */
		if ((init_shared_buffer->properties_buffer +
			properties_length
				> init_shared_buffer->properties_buffer) &&
			(properties_length <=
				init_shared_buffer->properties_length)) {
				memcpy(init_shared_buffer->properties_buffer,
					properties_buffer,
					 properties_length);
		} else {
			dprintk(KERN_INFO "tf_start(%p): "
				"Configuration buffer size from userland is "
				"incorrect(%d, %d)\n",
				comm, (u32) properties_length,
				init_shared_buffer->properties_length);
			ret = -EFAULT;
			goto error1;
		}
	}

	dprintk(KERN_INFO "tf_start(%p): "
		"System Configuration (%d bytes)\n", comm,
		init_shared_buffer->properties_length);
	dprintk(KERN_INFO "tf_start(%p): "
		"Starting PA (%d bytes)...\n", comm, pa_size);

	/*
	 * Make sure all data is visible to the secure world
	 */
	dmac_flush_range((void *)init_shared_buffer,
		(void *)(((u32)init_shared_buffer) + PAGE_SIZE));
	outer_clean_range(__pa(init_shared_buffer),
		__pa(init_shared_buffer) + PAGE_SIZE);

	dmac_flush_range((void *)pa_buffer,
		(void *)(pa_buffer + pa_size));
	outer_clean_range(__pa(pa_buffer),
		__pa(pa_buffer) + pa_size);

	dmac_flush_range((void *)&pa_info,
		(void *)(((u32)&pa_info) + sizeof(struct tf_ns_pa_info)));
	outer_clean_range(__pa(&pa_info),
		__pa(&pa_info) + sizeof(struct tf_ns_pa_info));
	wmb();

	spin_lock(&(comm->lock));
	comm->init_shared_buffer = init_shared_buffer;
	comm->pBuffer = l1_shared_buffer;
	spin_unlock(&(comm->lock));
	init_shared_buffer = NULL;
	l1_shared_buffer = NULL;

	/*
	 * Set the OS current time in the L1 shared buffer first. The secure
	 * world uses it as itw boot reference time.
	 */
	tf_set_current_time(comm);

	/* Workaround for issue #6081 */
	if ((omap_rev() && 0xFFF000FF) == OMAP443X_CLASS)
		disable_nonboot_cpus();

	/*
	 * Start the SMC PA
	 */
	ret = omap4_secure_dispatcher(API_HAL_LM_PALOAD_INDEX,
		FLAG_IRQ_ENABLE | FLAG_FIQ_ENABLE | FLAG_START_HAL_CRITICAL, 1,
		__pa(&pa_info), 0, 0, 0);
	if (ret != API_HAL_RET_VALUE_OK) {
		printk(KERN_ERR "SMC: Error while loading the PA [0x%x]\n",
			ret);
		goto error2;
	}

	/* Loop until the first S Yield RPC is received */
loop:
	mutex_lock(&(comm->rpc_mutex));

	if (g_RPC_advancement == RPC_ADVANCEMENT_PENDING) {
		dprintk(KERN_INFO "tf_rpc_execute: "
			"Executing CMD=0x%x\n",
			g_RPC_parameters[1]);

		switch (g_RPC_parameters[1]) {
		case RPC_CMD_YIELD:
			dprintk(KERN_INFO "tf_rpc_execute: "
				"RPC_CMD_YIELD\n");
			set_bit(TF_COMM_FLAG_L1_SHARED_ALLOCATED,
				&(comm->flags));
			g_RPC_parameters[0] = RPC_SUCCESS;
			break;

		case RPC_CMD_INIT:
			dprintk(KERN_INFO "tf_rpc_execute: "
				"RPC_CMD_INIT\n");
			g_RPC_parameters[0] = tf_rpc_init(comm);
			break;

		case RPC_CMD_TRACE:
			g_RPC_parameters[0] = tf_rpc_trace(comm);
			break;

		default:
			g_RPC_parameters[0] = RPC_ERROR_BAD_PARAMETERS;
			break;
		}
		g_RPC_advancement = RPC_ADVANCEMENT_FINISHED;
	}

	mutex_unlock(&(comm->rpc_mutex));

	ret = tf_schedule_secure_world(comm, false);
	if (ret != 0) {
		printk(KERN_ERR "SMC: Error while loading the PA [0x%x]\n",
			ret);
		goto error2;
	}

	if (!test_bit(TF_COMM_FLAG_L1_SHARED_ALLOCATED, &(comm->flags)))
		goto loop;

	set_bit(TF_COMM_FLAG_PA_AVAILABLE, &comm->flags);
	wake_up(&(comm->wait_queue));
	ret = 0;

	#if 0
	{
		void *workspace_va;
		workspace_va = ioremap(workspace_addr, workspace_size);
		printk(KERN_INFO
		"Read first word of workspace [0x%x]\n",
		*(uint32_t *)workspace_va);
	}
	#endif

	/* Workaround for issue #6081 */
	if ((omap_rev() && 0xFFF000FF) == OMAP443X_CLASS)
		enable_nonboot_cpus();

	goto exit;

error2:
	/* Workaround for issue #6081 */
	if ((omap_rev() && 0xFFF000FF) == OMAP443X_CLASS)
		enable_nonboot_cpus();

	spin_lock(&(comm->lock));
	l1_shared_buffer = comm->pBuffer;
	init_shared_buffer = comm->init_shared_buffer;
	comm->pBuffer = NULL;
	comm->init_shared_buffer = NULL;
	spin_unlock(&(comm->lock));

error1:
	if (init_shared_buffer != NULL) {
		__clear_page_locked(virt_to_page(init_shared_buffer));
		internal_free_page((unsigned long) init_shared_buffer);
	}
	if (l1_shared_buffer != NULL) {
		__clear_page_locked(virt_to_page(l1_shared_buffer));
		internal_free_page((unsigned long) l1_shared_buffer);
	}

exit:
#ifdef CONFIG_SMP
	ret_affinity = sched_setaffinity(0, &saved_cpu_mask);
	if (ret_affinity != 0)
		dprintk(KERN_ERR "sched_setaffinity #2 -> 0x%lX", ret_affinity);
#endif

	tf_l4sec_clkdm_allow_idle(true);

	if (ret > 0)
		ret = -EFAULT;

	return ret;
}

void tf_terminate(struct tf_comm *comm)
{
	dprintk(KERN_INFO "tf_terminate(%p)\n", comm);

	spin_lock(&(comm->lock));

	tf_crypto_terminate();

	spin_unlock(&(comm->lock));
}
