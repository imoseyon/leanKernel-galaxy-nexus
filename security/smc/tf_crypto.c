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

#include "tf_defs.h"
#include "tf_util.h"
#include "tf_zebra.h"
#include "tf_crypto.h"
#include "tf_dma.h"

#define IO_ADDRESS OMAP2_L4_IO_ADDRESS

#define S_SUCCESS		0x00000000
#define S_ERROR_GENERIC		0xFFFF0000
#define S_ERROR_ACCESS_DENIED	0xFFFF0001
#define S_ERROR_BAD_FORMAT	0xFFFF0005
#define S_ERROR_BAD_PARAMETERS	0xFFFF0006
#define S_ERROR_OUT_OF_MEMORY	0xFFFF000C
#define S_ERROR_SHORT_BUFFER	0xFFFF0010
#define S_ERROR_UNREACHABLE	0xFFFF3013
#define S_ERROR_SERVICE		0xFFFF1000

#define CKR_OK			0x00000000

#define PUBLIC_CRYPTO_TIMEOUT_CONST	0x000FFFFF

#define RPC_AES1_CODE	PUBLIC_CRYPTO_HWA_AES1
#define RPC_DES_CODE	PUBLIC_CRYPTO_HWA_DES
#define RPC_SHA_CODE	PUBLIC_CRYPTO_HWA_SHA

#define RPC_CRYPTO_COMMAND_MASK	0x000003c0

#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR		0x200
#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_UNLOCK	0x000
#define RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_LOCK	0x001

#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT			0x240
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_AES1	RPC_AES1_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_DES		RPC_DES_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_LOCK_SHA		RPC_SHA_CODE
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_SUSPEND		0x010
#define RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_UNINSTALL	0x020

#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS			0x280
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_AES1	RPC_AES1_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_DES	RPC_DES_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_SHA	RPC_SHA_CODE
#define RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_RESUME		0x010

#define RPC_CLEAR_GLOBAL_KEY_CONTEXT			0x2c0
#define RPC_CLEAR_GLOBAL_KEY_CONTEXT_CLEARED_AES	0x001
#define RPC_CLEAR_GLOBAL_KEY_CONTEXT_CLEARED_DES	0x002

#define ENABLE_CLOCK	true
#define DISABLE_CLOCK	false

/*---------------------------------------------------------------------------*/
/*RPC IN/OUT structures for CUS implementation                               */
/*---------------------------------------------------------------------------*/

struct rpc_install_shortcut_lock_accelerator_out {
	u32 shortcut_id;
	u32 error;
};

struct rpc_install_shortcut_lock_accelerator_in {
	u32 device_context_id;
	u32 client_session;
	u32 command_id;
	u32 key_context;
	/**
	 *The identifier of the HWA accelerator that this shortcut uses!
	 *Possible values are:
	 *- 1 (RPC_AES1_CODE)
	 *- 4 (RPC_DES_CODE)
	 *- 8 (RPC_SHA_CODE)
	 **/
	u32 hwa_id;
	/**
	 *This field defines the algorithm, direction, mode, key size.
	 *It contains some of the bits of the corresponding "CTRL" register
	 *of the accelerator.
	 *
	 *More precisely:
	 *For AES1 accelerator, hwa_ctrl contains the following bits:
	 *- CTR (bit 6):
	 * when 1, selects CTR mode.
	 * when 0, selects CBC or ECB mode (according to CBC bit)
	 *- CBC (bit 5)
	 * when 1, selects CBC mode (but only if CTR=0)
	 * when 0, selects EBC mode (but only if CTR=0)
	 *- DIRECTION (bit 2)
	 *  0: decryption
	 *  1: encryption
	 *
	 *For the DES2 accelerator, hwa_ctrl contains the following bits:
	 *- CBC (bit 4): 1 for CBC, 0 for ECB
	 *- DIRECTION (bit 2): 0 for decryption, 1 for encryption
	 *
	 *For the SHA accelerator, hwa_ctrl contains the following bits:
	 *- ALGO (bit 2:1):
	 *  0x0: MD5
	 *  0x1: SHA1
	 *  0x2: SHA-224
	 *  0x3: SHA-256
	 **/
	u32 hwa_ctrl;
	union tf_crypto_operation_state operation_state;
};

struct rpc_lock_hwa_suspend_shortcut_out {
	union tf_crypto_operation_state operation_state;
};

struct rpc_lock_hwa_suspend_shortcut_in {
	u32 shortcut_id;
};

struct rpc_resume_shortcut_unlock_hwa_in {
	u32 shortcut_id;
	u32 aes1_key_context;
	u32 reserved;
	u32 des_key_context;
	union tf_crypto_operation_state operation_state;
};

/*------------------------------------------------------------------------- */
/*
 * tf_get_device_context(struct cus_context *cus)
 * search in the all the device context (connection_list) if the CUS context
 * specified by cus exist.
 *
 * If it is found, return the device context where the CUS context is.
 * If is is not found, return NULL.
 */
static struct tf_connection *tf_get_device_context(
	struct cus_context *cus)
{
	struct tf_connection *connection = NULL;
	struct cus_context *cusFromList = NULL;
	struct tf_device *dev = tf_get_device();

	spin_lock(&(dev->connection_list_lock));
	list_for_each_entry(connection, &(dev->connection_list),
		list) {
		spin_lock(&(connection->shortcut_list_lock));
		list_for_each_entry(cusFromList,
			&(connection->shortcut_list), list) {
			if ((u32)cusFromList == (u32)cus) {
				spin_unlock(&(connection->
					shortcut_list_lock));
				spin_unlock(&(dev->
					connection_list_lock));
				return connection;
			}
		}
		spin_unlock(&(connection->
			shortcut_list_lock));
	}
	spin_unlock(&(dev->connection_list_lock));

	/*cus does not exist */
	return NULL;
}

/*------------------------------------------------------------------------- */
/*
 * Get the shared memory from the memory block handle coming from secure.
 * Return NULL if it does not exist.
 */
static struct tf_shmem_desc *tf_get_shmem_from_block_handle(
	struct tf_connection *connection, u32 block)
{
	struct tf_shmem_desc *shmem_desc = NULL;

	mutex_lock(&(connection->shmem_mutex));

	list_for_each_entry(shmem_desc,
			&(connection->used_shmem_list), list) {
		if ((u32) shmem_desc->block_identifier ==
				(u32) block) {
			mutex_unlock(&(connection->shmem_mutex));
			return shmem_desc;
		}
	}

	/* block does not exist */
	mutex_unlock(&(connection->shmem_mutex));

	return NULL;
}

/*------------------------------------------------------------------------- */
/*
 * HWA public lock or unlock one HWA according algo specified by hwa_id
 */
void tf_crypto_lock_hwa(u32 hwa_id, bool do_lock)
{
	struct semaphore *s = NULL;
	struct tf_device *dev = tf_get_device();

	dprintk(KERN_INFO "[pid=%d] %s: hwa_id=0x%04X do_lock=%d\n",
		current->pid, __func__, hwa_id, do_lock);

	switch (hwa_id) {
	case RPC_AES1_CODE:
		s = &dev->aes1_sema;
		break;
	case RPC_DES_CODE:
		s = &dev->des_sema;
		break;
	default:
	case RPC_SHA_CODE:
		s = &dev->sha_sema;
		break;
	}

	if (do_lock == LOCK_HWA) {
		dprintk(KERN_INFO "tf_crypto_lock_hwa: "
			"Wait for HWAID=0x%04X\n", hwa_id);
		while (down_trylock(s))
			cpu_relax();
		dprintk(KERN_INFO "tf_crypto_lock_hwa: "
			"Locked on HWAID=0x%04X\n", hwa_id);
	} else {
		up(s);
		dprintk(KERN_INFO "tf_crypto_lock_hwa: "
			"Released for HWAID=0x%04X\n", hwa_id);
	}
}

/*------------------------------------------------------------------------- */
/*
 * HWAs public lock or unlock HWA's specified in the HWA H/A/D fields of RPC
 * command rpc_command
 */
static void tf_crypto_lock_hwas(u32 rpc_command, bool do_lock)
{
	dprintk(KERN_INFO
		"tf_crypto_lock_hwas: rpc_command=0x%08x do_lock=%d\n",
		rpc_command, do_lock);

	/* perform the locks */
	if (rpc_command & RPC_AES1_CODE)
		tf_crypto_lock_hwa(RPC_AES1_CODE, do_lock);

	if (rpc_command & RPC_DES_CODE)
		tf_crypto_lock_hwa(RPC_DES_CODE, do_lock);

	if (rpc_command & RPC_SHA_CODE)
		tf_crypto_lock_hwa(RPC_SHA_CODE, do_lock);
}

/*------------------------------------------------------------------------- */
/**
 *Initialize the public crypto DMA channels, global HWA semaphores and handles
 */
u32 tf_crypto_init(void)
{
	struct tf_device *dev = tf_get_device();
	u32 error = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	/* Initialize HWAs */
	tf_aes_init();
	tf_des_init();
	tf_digest_init();

	/*initialize the HWA semaphores */
	sema_init(&dev->aes1_sema, 1);
	sema_init(&dev->des_sema, 1);
	sema_init(&dev->sha_sema, 1);

	/*initialize the current key handle loaded in the AESn/DES HWA */
	dev->aes1_key_context = 0;
	dev->des_key_context = 0;
	dev->sham1_is_public = false;

	/*initialize the DMA semaphores */
	mutex_init(&dev->sm.dma_mutex);

	/*allocate DMA buffer */
	dev->dma_buffer_length = PAGE_SIZE * 16;
	dev->dma_buffer = dma_alloc_coherent(NULL,
		dev->dma_buffer_length,
		&(dev->dma_buffer_phys),
		GFP_KERNEL);
	if (dev->dma_buffer == NULL) {
		printk(KERN_ERR
			"tf_crypto_init: Out of memory for DMA buffer\n");
		error = S_ERROR_OUT_OF_MEMORY;
	}

	return error;
}

/*------------------------------------------------------------------------- */
/*
 *Initialize the device context CUS fields (shortcut semaphore and public CUS
 *list)
 */
void tf_crypto_init_cus(struct tf_connection *connection)
{
	/*initialize the CUS list in the given device context */
	spin_lock_init(&(connection->shortcut_list_lock));
	INIT_LIST_HEAD(&(connection->shortcut_list));
}

/*------------------------------------------------------------------------- */
/**
 *Terminate the public crypto (including DMA)
 */
void tf_crypto_terminate(void)
{
	struct tf_device *dev = tf_get_device();

	if (dev->dma_buffer != NULL) {
		dma_free_coherent(NULL, dev->dma_buffer_length,
			dev->dma_buffer,
			dev->dma_buffer_phys);
		dev->dma_buffer = NULL;
	}

	tf_digest_exit();
	tf_des_exit();
	tf_aes_exit();
}

/*------------------------------------------------------------------------- */
/*
 *Perform a crypto update operation.
 *THIS FUNCTION IS CALLED FROM THE IOCTL
 */
static bool tf_crypto_update(
	struct cus_context *cus,
	struct cus_params *params)
{
	bool status = true;
	dprintk(KERN_INFO
		"tf_crypto_update(%x): "\
		"HWAID=0x%x, In=%p, Out=%p, Len=%u\n",
		(uint32_t) cus, cus->hwa_id,
		params->input_data,
		params->output_data, params->input_data_length);

	/* Enable the clock and Process Data */
	switch (cus->hwa_id) {
	case RPC_AES1_CODE:
		tf_crypto_enable_clock(PUBLIC_CRYPTO_AES1_CLOCK_REG);
		cus->operation_state.aes.key_is_public = 0;
		cus->operation_state.aes.CTRL = cus->hwa_ctrl;
		status = tf_aes_update(
			&cus->operation_state.aes,
			params->input_data,
			params->output_data,
			params->input_data_length / AES_BLOCK_SIZE);
		tf_crypto_disable_clock(PUBLIC_CRYPTO_AES1_CLOCK_REG);
		break;

	case RPC_DES_CODE:
		tf_crypto_enable_clock(PUBLIC_CRYPTO_DES3DES_CLOCK_REG);
		status = tf_des_update(
			cus->hwa_ctrl,
			&cus->operation_state.des,
			params->input_data,
			params->output_data,
			params->input_data_length / DES_BLOCK_SIZE);
		tf_crypto_disable_clock(PUBLIC_CRYPTO_DES3DES_CLOCK_REG);
		break;

	case RPC_SHA_CODE:
		tf_crypto_enable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);
		cus->operation_state.sha.CTRL = cus->hwa_ctrl;
		status = tf_digest_update(
			&cus->operation_state.sha,
			params->input_data,
			params->input_data_length);
		tf_crypto_disable_clock(PUBLIC_CRYPTO_SHA2MD5_CLOCK_REG);
		break;

	default:
		BUG_ON(1);
		break;
	}

	dprintk(KERN_INFO "tf_crypto_update: Done\n");
	return status;
}

/*------------------------------------------------------------------------- */

/*
 *Check if the command must be intercepted by a CUS or not.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 *
 *inputs: struct tf_connection *connection : current device context
 *       tf_command_invoke_client_command *command : the command
 *       bool incrementuse_count : specify if the use_count must be incremented
 *output:
 * struct cus_context **cus_ctx : the public CUS
 *       if it is shortcuted
 *return: true or false
 *
 */
static bool tf_crypto_is_shortcuted_command(
	struct tf_connection *connection,
	struct tf_command_invoke_client_command *command,
	struct cus_context **cus_ctx,
	bool incrementuse_count)
{
	struct tf_device *dev = tf_get_device();
	struct cus_context *cus = NULL;
	*cus_ctx = NULL;

	dprintk(KERN_INFO "tf_crypto_is_shortcuted_command: "\
		"connection=0x%08x, command=0x%08x, "\
		"CltSession=0x%08x, CmdID=0x%08x\n",
		(uint32_t) connection, (uint32_t) command,
		(uint32_t) command->client_session,
		command->client_command_identifier);

	/*take shortcut_list_lock for the device context
	 *in which the message is sent <=> make sure that nobody is
	 *going to change data while processing */
	spin_lock(&(connection->shortcut_list_lock));

	/*lookup in the list of shortcuts attached to the device context for a
	 *shortcut context that contains the same client_session as the command
	 *and such that command_id is equal to client_command_identifier of the
	 *INVOKE_CLIENT_COMMAND message. If no such shortcut exists, take the
	 *standard path */
	list_for_each_entry(
	   cus, &(connection->shortcut_list), list) {
		dprintk(KERN_INFO
			"tf_crypto_is_shortcuted_command: "\
			"command_id = 0x%08x client_session = 0x%08x\n",
			cus->command_id, cus->client_session);

		if ((cus->client_session == command->client_session)
			&&
			(cus->command_id == command->
				client_command_identifier)) {
			dprintk(KERN_INFO
				"tf_crypto_is_shortcuted_command: "\
				"shortcut is identified\n");
			/*find a CUS : check if is suspended or not */
			if (cus->suspended) {
				/*
				 * suspended of the shortcut context is set to
				 * true, it means that the secure world has
				 * suspended the shortcut to perform an update
				 * on its own. In this case, take the standard
				 * path. This should happen very rarely because
				 * the client and the service should generally
				 * communicate to avoid such a collision
				 */
				dprintk(KERN_INFO "shortcut exists but "\
					"suspended\n");
				goto command_not_shortcutable;

			} else {
				dprintk(KERN_INFO "shortcut exists\n");
				/*For AES and DES/3DES operations,
				 *provisionally determine if the accelerator
				 *is loaded with the appropriate key before
				 *deciding to enter the accelerator critical
				 *section. In most cases, if some other thread
				 *or the secure world is currently using the
				 *accelerator, the key won't change.
				 *So, if the key doesn't match now, it is
				 *likely not to match later on, so we'd better
				 *not try to enter the critical section in this
				 *case: */

				if (cus->hwa_id == RPC_AES1_CODE &&
					cus->
					key_context != dev->
					aes1_key_context) {
					/*For AES operations, atomically read
					 *g_hAES1SSecureKeyContext and check if
					 *it is equal to key_context. If not,
					 *take the standard path  <=> do not
					 *shortcut */
					dprintk(KERN_INFO
						"shortcut exists but AES key "\
						"not correct\nkey_context="\
						"0x%08x vs 0x%08x\n",
						cus->key_context,
						dev->
						aes1_key_context);
					goto command_not_shortcutable;

				} else if (cus->hwa_id == RPC_DES_CODE
						&& cus->key_context !=
						dev->
						des_key_context) {
					/*
					 * For DES/3DES atomically read
					 * des_key_context and check if
					 * it is equal to key_context. If not,
					 * take the standard path <=> do not
					 * shortcut
					 */
					dprintk(KERN_INFO
						"shortcut exists but DES key "
						"not correct "
						"des_key_context = 0x%08x"
						" key_context0x%08x\n",
						(u32)dev->
						des_key_context,
						(u32)cus->key_context);
					goto command_not_shortcutable;
				} else if (cus->hwa_id == RPC_SHA_CODE
						&& !dev->sham1_is_public) {
					/*
					 * For digest operations, atomically
					 * read sham1_is_public and check if it
					 * is true. If not, no shortcut.
					 */
					 dprintk(KERN_INFO
						 "shortcut exists but SHAM1 "
						 "is not accessible in public");
					 goto command_not_shortcutable;
				}
			}

			dprintk(KERN_INFO "shortcut exists and enable\n");

			/*Shortcut has been found and context fits with
			 *thread => YES! the command can be shortcuted */

			/*
			 *set the pointer on the corresponding session
			 *(eq CUS context)
			 */
			*cus_ctx = cus;

			/*
			 *increment use_count if required
			 */
			if (incrementuse_count)
				cus->use_count++;

			/*
			 *release shortcut_list_lock
			 */
			spin_unlock(&(connection->
				shortcut_list_lock));
			return true;
		}
	}

 command_not_shortcutable:
	/*
	 *release shortcut_list_lock
	 */
	spin_unlock(&(connection->shortcut_list_lock));
	*cus_ctx = NULL;
	return false;
}

/*------------------------------------------------------------------------- */
/*
 * Pre-process the client command (crypto update operation), i.e., parse the
 * command message (decode buffers, etc.) THIS FUNCTION IS CALLED FROM THE USER
 * THREAD (ioctl).
 *
 * For incorrect messages, an error is returned and the message will be sent to
 * secure
 */
static bool tf_crypto_parse_command_message(struct tf_connection *connection,
	struct cus_context *cus,
	struct tf_command_invoke_client_command *command,
	struct cus_params *params)
{
	u32 param_type;
	u32 input_data_length;
	u32 output_data_length;
	u8 *input_data;
	u8 *output_data;
	struct tf_shmem_desc *input_shmem = NULL;
	struct tf_shmem_desc *output_shmem = NULL;

	dprintk(KERN_INFO
		"tf_crypto_parse_command_message(%p) : Session=0x%x\n",
		cus, cus->client_session);

	if (command->params[0].temp_memref.size == 0)
		return false;

	param_type = TF_GET_PARAM_TYPE(command->param_types, 0);
	switch (param_type) {
	case TF_PARAM_TYPE_MEMREF_TEMP_INPUT:
		if (command->params[0].temp_memref.descriptor == 0)
			return false;

		input_data = (u8 *) command->params[0].temp_memref.
			descriptor;
		input_data_length = command->params[0].temp_memref.size;

		break;

	case TF_PARAM_TYPE_MEMREF_INPUT:
		input_shmem = tf_get_shmem_from_block_handle(connection,
			command->params[0].memref.block);

		if (input_shmem == NULL)
			return false;
		atomic_inc(&input_shmem->ref_count);

		input_data = input_shmem->pBuffer +
			command->params[0].memref.offset;
		input_data_length = command->params[0].memref.size;

		break;

	default:
		return false;
	}

	if (cus->hwa_id != RPC_SHA_CODE) {
		if (command->params[1].temp_memref.size == 0)
			goto err0;

		/* We need an output buffer as well */
		param_type = TF_GET_PARAM_TYPE(command->param_types, 1);
		switch (param_type) {
		case TF_PARAM_TYPE_MEMREF_TEMP_OUTPUT:
			output_data =
				(u8 *) command->params[1].temp_memref.
					descriptor;
			output_data_length =
				command->params[1].temp_memref.size;

			break;

		case TF_PARAM_TYPE_MEMREF_OUTPUT:
			if (command->params[1].temp_memref.descriptor == 0)
				return false;

			output_shmem = tf_get_shmem_from_block_handle(
				connection, command->params[1].memref.block);
			if (output_shmem == NULL)
				goto err0;
			atomic_inc(&output_shmem->ref_count);

			output_data = output_shmem->pBuffer +
				command->params[1].memref.offset;
			output_data_length = command->params[1].memref.size;

			break;

		default:
			dprintk(KERN_ERR "tf_crypto_parse_command_message: "
				"Encrypt/decrypt operations require an output "
				"buffer\n");

			goto err0;
		}

		if (output_data_length < input_data_length) {
			dprintk(KERN_ERR "tf_crypto_parse_command_message: "
				"Short buffer: output_data_length = %d < "
				"input_data_length = %d\n",
				output_data_length, input_data_length);
			goto err1;
		}
	} else {
		output_data_length = 0;
		output_data = NULL;
	}

	/*
	 * Check if input length is compatible with the algorithm of the
	 * shortcut
	 */
	switch (cus->hwa_id) {
	case RPC_AES1_CODE:
		/* Must be multiple of the AES block size */
		if ((input_data_length % AES_BLOCK_SIZE) != 0) {
			dprintk(KERN_ERR
				"tf_crypto_parse_command_message(%p): "\
				"Input Data Length invalid [%d] for AES\n",
				cus, input_data_length);
			goto err1;
		}
		break;
	case RPC_DES_CODE:
		/* Must be multiple of the DES block size */
		if ((input_data_length % DES_BLOCK_SIZE) != 0) {
			dprintk(KERN_ERR
				"tf_crypto_parse_command_message(%p): "\
				"Input Data Length invalid [%d] for DES\n",
				cus, input_data_length);
			goto err1;
		}
		break;
	default:
		/* SHA operation: no constraint on data length */
		break;
	}

	params->input_data = input_data;
	params->input_data_length = input_data_length;
	params->input_shmem = input_shmem;
	params->output_data = output_data;
	params->output_data_length = output_data_length;
	params->output_shmem = output_shmem;

	return true;

err1:
	if (output_shmem)
		atomic_dec(&output_shmem->ref_count);
err0:
	if (input_shmem)
		atomic_dec(&input_shmem->ref_count);

	return false;
}

/*------------------------------------------------------------------------- */

/*
 *Post-process the client command (crypto update operation),
 *i.e. copy the result into the user output buffer and release the resources.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 */
static void tf_crypto_write_answer(
	struct cus_context *cus,
	struct cus_params *params,
	struct tf_answer_invoke_client_command *answer)
{
	u32 error = S_SUCCESS;

	dprintk(KERN_INFO
		"tf_crypto_write_answer(%p) : Session=0x%x\n",
		cus, cus->client_session);

	/* Generate the answer */
	answer->message_size =
		(sizeof(struct tf_answer_invoke_client_command) -
		 sizeof(struct tf_answer_header)) / 4;
	answer->message_type = TF_MESSAGE_TYPE_INVOKE_CLIENT_COMMAND;
	answer->error_origin = TF_ORIGIN_TRUSTED_APP;
	answer->operation_id = 0;
	answer->error_code = error;
	answer->answers[1].size.size = params->output_data_length;
}

/*------------------------------------------------------------------------- */

int tf_crypto_try_shortcuted_update(struct tf_connection *connection,
	struct tf_command_invoke_client_command *command,
	struct tf_answer_invoke_client_command *answer)
{
	struct cus_context *cus = NULL;

	if (tf_crypto_is_shortcuted_command(connection,
			(struct tf_command_invoke_client_command *) command,
			&cus, false)) {
		u32 hwa_id = cus->hwa_id;

		/* Lock HWA */
		tf_crypto_lock_hwa(hwa_id, LOCK_HWA);

		if (tf_crypto_is_shortcuted_command(connection,
				command,
				&cus, true)) {
			struct cus_params cus_params;

			memset(&cus_params, 0, sizeof(cus_params));

			if (!tf_crypto_parse_command_message(
					connection,
					cus,
					command,
					&cus_params)) {
				/* Decrement CUS context use count */
				cus->use_count--;

				/* Release HWA lock */
				tf_crypto_lock_hwa(cus->hwa_id,
					UNLOCK_HWA);

				return -1;
			}

			/* Perform the update in public <=> THE shortcut */
			if (!tf_crypto_update(cus, &cus_params)) {
				/* Decrement CUS context use count */
				cus->use_count--;

				/* Release HWA lock */
				tf_crypto_lock_hwa(cus->hwa_id,
					UNLOCK_HWA);

				return -1;
			}

			/* Write answer message */
			tf_crypto_write_answer(cus,
				&cus_params, answer);

			/* Decrement registered shmems use count if needed */
			if (cus_params.input_shmem)
				atomic_dec(&cus_params.input_shmem->ref_count);
			if (cus_params.output_shmem)
				atomic_dec(&cus_params.output_shmem->ref_count);

			/* Decrement CUS context use count */
			cus->use_count--;

			tf_crypto_lock_hwa(cus->hwa_id,
				UNLOCK_HWA);
		} else {
			tf_crypto_lock_hwa(hwa_id, UNLOCK_HWA);
			return -1;
		}
	} else {
		return -1;
	}

	return 0;
}

/*------------------------------------------------------------------------- */

void tf_crypto_wait_for_ready_bit_infinitely(u32 *reg, u32 bit)
{
	while (!(INREG32(reg) & bit))
		;
}

/*------------------------------------------------------------------------- */

u32 tf_crypto_wait_for_ready_bit(u32 *reg, u32 bit)
{
	u32 timeoutCounter = PUBLIC_CRYPTO_TIMEOUT_CONST;

	while ((!(INREG32(reg) & bit)) && ((--timeoutCounter) != 0))
		;

	if (timeoutCounter == 0)
		return PUBLIC_CRYPTO_ERR_TIMEOUT;

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*------------------------------------------------------------------------- */

static DEFINE_SPINLOCK(clk_lock);

void tf_crypto_disable_clock(uint32_t clock_paddr)
{
	u32 *clock_reg;
	u32 val;
	unsigned long flags;

	dprintk(KERN_INFO "tf_crypto_disable_clock: " \
		"clock_paddr=0x%08X\n",
		clock_paddr);

	/* Ensure none concurrent access when changing clock registers */
	spin_lock_irqsave(&clk_lock, flags);

	clock_reg = (u32 *)IO_ADDRESS(clock_paddr);

	val = __raw_readl(clock_reg);
	val &= ~(0x3);
	__raw_writel(val, clock_reg);

	/* Wait for clock to be fully disabled */
	while ((__raw_readl(clock_reg) & 0x30000) == 0)
		;

	spin_unlock_irqrestore(&clk_lock, flags);

	tf_l4sec_clkdm_allow_idle(true);
}

/*------------------------------------------------------------------------- */

void tf_crypto_enable_clock(uint32_t clock_paddr)
{
	u32 *clock_reg;
	u32 val;
	unsigned long flags;

	dprintk(KERN_INFO "tf_crypto_enable_clock: " \
		"clock_paddr=0x%08X\n",
		clock_paddr);

	tf_l4sec_clkdm_wakeup(true);

	/* Ensure none concurrent access when changing clock registers */
	spin_lock_irqsave(&clk_lock, flags);

	clock_reg = (u32 *)IO_ADDRESS(clock_paddr);

	val = __raw_readl(clock_reg);
	val |= 0x2;
	__raw_writel(val, clock_reg);

	/* Wait for clock to be fully enabled */
	while ((__raw_readl(clock_reg) & 0x30000) != 0)
		;

	spin_unlock_irqrestore(&clk_lock, flags);
}

/*------------------------------------------------------------------------- */
/*                     CUS RPCs                                             */
/*------------------------------------------------------------------------- */
/*
 * This RPC is used by the secure world to install a new shortcut.  Optionally,
 * for AES or DES/3DES operations, it can also lock the accelerator so that the
 * secure world can install a new key in it.
 */
static int tf_crypto_install_shortcut_lock_hwa(
	u32 rpc_command, void *rpc_shared_buffer)
{
	struct cus_context *cus = NULL;
	struct tf_connection *connection = NULL;

	/* Reference the input/ouput data */
	struct rpc_install_shortcut_lock_accelerator_out *install_cus_out =
		rpc_shared_buffer;
	struct rpc_install_shortcut_lock_accelerator_in *install_cus_in =
		rpc_shared_buffer;

	dprintk(KERN_INFO "tf_crypto_install_shortcut_lock_hwa: "
		"rpc_command=0x%08x; hwa_id=0x%08x\n",
		rpc_command, install_cus_in->hwa_id);

	connection = (struct tf_connection *)
		install_cus_in->device_context_id;

	if (connection == NULL) {
		dprintk(KERN_INFO
			"tf_crypto_install_shortcut_lock_hwa: "
			"DeviceContext 0x%08x does not exist, "
			"cannot create Shortcut\n",
			install_cus_in->device_context_id);
		install_cus_out->error = -1;
		return 0;
	}

	/*
	 * Allocate a shortcut context. If the allocation fails,
	 * return S_ERROR_OUT_OF_MEMORY error code
	 */
	cus = (struct cus_context *)
		internal_kmalloc(sizeof(*cus), GFP_KERNEL);
	if (cus == NULL) {
		dprintk(KERN_ERR
			"tf_crypto_install_shortcut_lock_hwa: "\
			"Out of memory for public session\n");
		install_cus_out->error = S_ERROR_OUT_OF_MEMORY;
		return 0;
	}

	memset(cus, 0, sizeof(*cus));

	/*setup the shortcut */
	cus->magic_number = CUS_CONTEXT_MAGIC;
	cus->client_session = install_cus_in->client_session;
	cus->command_id = install_cus_in->command_id;
	cus->hwa_id = install_cus_in->hwa_id;
	cus->hwa_ctrl = install_cus_in->hwa_ctrl;
	cus->key_context = install_cus_in->key_context;
	cus->use_count = 0;
	cus->suspended = false;

	memcpy(&cus->operation_state,
			 &install_cus_in->operation_state,
			 sizeof(union tf_crypto_operation_state));

	/*lock the shortcut_list_lock for this device context */
	spin_lock(&connection->shortcut_list_lock);

	/*Insert the shortcut in the list of shortcuts in the device context */
	list_add(&(cus->list), &(connection->shortcut_list));

	/*release shortcut_list_lock */
	spin_unlock(&connection->shortcut_list_lock);

	/*fill the output structure */
	install_cus_out->shortcut_id = (u32) cus;
	install_cus_out->error = S_SUCCESS;

	/*If the L bit is true, then:
	 * Enter the accelerator critical section. If an update is currently in
	 * progress on the accelerator (using g_hXXXKeyContext key), this will
	 * wait until the update has completed. This is call when secure wants
	 * to install a key in HWA, once it is done secure world will release
	 * the lock.  For SHA (activate shortcut is always called without LOCK
	 * fag):do nothing
	 */
	if ((rpc_command & RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR_LOCK) != 0) {
		/*Lock the HWA */
		tf_crypto_lock_hwa(cus->hwa_id, LOCK_HWA);
	}

	dprintk(KERN_INFO
		"tf_crypto_install_shortcut_lock_hwa: Done\n");

	return S_SUCCESS;
}

/*------------------------------------------------------------------------- */

/*
 * This RPC is used to perform one or several of the following operations
 * - Lock one or several accelerators for the exclusive use by the secure world,
 *   either because it is going to be switched to secure or because a new key is
 *   going to be loaded in the accelerator
 * - Suspend a shortcut, i.e., make it temporarily unavailable to the public
 *   world. This is used when a secure update is going to be performed on the
 *   operation. The answer to the RPC then contains the operation state
 *   necessary for the secure world to do the update.
 * - Uninstall the shortcut
 */
static int tf_crypto_lock_hwas_suspend_shortcut(
	u32 rpc_command, void *rpc_shared_buffer)
{
	u32 target_shortcut;
	struct cus_context *cus = NULL;
	struct tf_connection *connection = NULL;

	/*reference the input/ouput data */
	struct rpc_lock_hwa_suspend_shortcut_out *suspend_cus_out =
		rpc_shared_buffer;
	struct rpc_lock_hwa_suspend_shortcut_in *suspend_cus_in =
		rpc_shared_buffer;

	dprintk(KERN_INFO
		"tf_crypto_lock_hwas_suspend_shortcut: "\
		"suspend_cus_in=0x%08x; shortcut_id=0x%08x\n",
		suspend_cus_in->shortcut_id, (u32)suspend_cus_in);

	target_shortcut = suspend_cus_in->shortcut_id;

	/*lock HWAs */
	tf_crypto_lock_hwas(rpc_command, LOCK_HWA);

	/*if suspend_cus_in->shortcut_id != 0 and  if rpc_command.S != 0,
		then, suspend shortcut */
	if ((target_shortcut != 0) && ((rpc_command &
		RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_SUSPEND) != 0)) {
		/*reference the CUSContext */
		cus = (struct cus_context *)
			suspend_cus_in->shortcut_id;

		/*preventive check1: return if shortcut does not exist */
		connection = tf_get_device_context(cus);
		if (connection == NULL) {
			dprintk(KERN_INFO
			"tf_crypto_lock_hwas_suspend_shortcut: "\
			"shortcut_id=0x%08x does not exist, cannot suspend "\
			"Shortcut\n",
				suspend_cus_in->shortcut_id);
			return -1;
		}

loop_on_suspend:
		/*lock shortcut_list_lock associated with the
		 *device context */
		spin_lock(&connection->shortcut_list_lock);

		/*Suspend shortcut */
		cus->suspended = true;

		if (cus->use_count != 0) {
			/*release shortcut_list_lock */
			spin_unlock(&connection->
				shortcut_list_lock);
			schedule();
			goto loop_on_suspend;
		}

		/*Copy the operation state data stored in CUS Context into the
		 *answer to the RPC output assuming that HWA register has been
		 *saved at update time */
		memcpy(&suspend_cus_out->operation_state,
				 &cus->operation_state,
				 sizeof(union tf_crypto_operation_state));

		/*Uninstall shortcut if requiered  */
		if ((rpc_command &
		RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT_UNINSTALL) != 0) {
			dprintk(KERN_INFO
			"tf_crypto_lock_hwas_suspend_shortcut:"\
			"Uninstall 0x%08x\n",
				target_shortcut);
			list_del(&(cus->list));
			/*list_del only remove the item in the list, the
			 *memory must be free afterward */
			/*release the lock before calling internal_kfree */
			spin_unlock(&connection->
				shortcut_list_lock);
			if (cus != NULL)
				internal_kfree(cus);
			return 0;
		}

		/*release shortcut_list_lock */
		spin_unlock(&connection->shortcut_list_lock);
	}

	return 0;
}

/*------------------------------------------------------------------------- */

/*
 * This RPC is used to perform one or several of the following operations:
 * -	Resume a shortcut previously suspended
 * -	Inform the public driver of the new keys installed in the DES and AES
 *	accelerators
 * -	Unlock some of the accelerators
 */
static int tf_crypto_resume_shortcut_unlock_hwas(
	u32 rpc_command, void *rpc_shared_buffer)
{
	struct tf_device *dev = tf_get_device();
	struct tf_connection *connection = NULL;
	struct cus_context *cus = NULL;

	/*reference the input data */
	struct rpc_resume_shortcut_unlock_hwa_in *resume_cus_in =
		rpc_shared_buffer;

	dprintk(KERN_INFO
		"tf_crypto_resume_shortcut_unlock_hwas\n"
		"rpc_command=0x%08x\nshortcut_id=0x%08x\n",
		rpc_command, resume_cus_in->shortcut_id);

	/*if shortcut_id not 0 resume the shortcut and unlock HWA
		else only unlock HWA */
	if (resume_cus_in->shortcut_id != 0) {
		/*reference the CUSContext */
		cus = (struct cus_context *)
			resume_cus_in->shortcut_id;

		/*preventive check1: return if shortcut does not exist
		 *else, points to the public crypto monitor (inside the device
		 *context) */
		connection = tf_get_device_context(cus);
		if (connection == NULL) {
			dprintk(KERN_INFO
			"tf_crypto_resume_shortcut_unlock_hwas(...):"\
			"shortcut_id 0x%08x does not exist, cannot suspend "\
			"Shortcut\n",
				resume_cus_in->shortcut_id);
			return -1;
		}

		/*if S set and shortcut not yet suspended */
		if ((cus->suspended) &&
			 ((rpc_command &
			RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_RESUME) != 0)){
			/*Write operation_stateData in the shortcut context */
			memcpy(&cus->operation_state,
				&resume_cus_in->operation_state,
				sizeof(union tf_crypto_operation_state));
			/*resume the shortcut */
			cus->suspended = false;
		}
	}

	/*
	 * If A is set: Atomically set aes1_key_context to
	 * aes1_key_context
	 */
	if ((rpc_command &
		RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_AES1) != 0) {
		dev->aes1_key_context =
			resume_cus_in->aes1_key_context;
	}

	/*
	 * If D is set:
	 * Atomically set des_key_context to des_key_context
	 */
	if ((rpc_command &
		RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_DES) != 0) {
		dev->des_key_context =
					resume_cus_in->des_key_context;
	}

	/* H is never set by the PA: Atomically set sham1_is_public to true */
	dev->sham1_is_public = true;

	/* Unlock HWAs according rpc_command */
	tf_crypto_lock_hwas(rpc_command, UNLOCK_HWA);

	return 0;
}

/*------------------------------------------------------------------------- */

/*
 * This RPC is used to notify the public driver that the key in the AES, DES
 * accelerators has been cleared. This happens only when the key is no longer
 * referenced by any shortcuts. So, it is guaranteed that no-one has entered the
 * accelerators critical section and there is no need to enter it to implement
 * this RPC.
 */
static int tf_crypto_clear_global_key_context(
	u32 rpc_command, void *rpc_shared_buffer)
{
	struct tf_device *dev = tf_get_device();

	/*
	 * If A is set: Atomically set aes1_key_context to 0
	 */
	if ((rpc_command &
		RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_AES1) != 0) {
		dev->aes1_key_context = 0;
	}

	/*
	 *If D is set: Atomically set des_key_context to 0
	 */
	if ((rpc_command &
		RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS_UNLOCK_DES) != 0) {
		dev->des_key_context = 0;
	}

	return 0;
}

/*------------------------------------------------------------------------- */
/*
 * Execute a public crypto related RPC
 */

int tf_crypto_execute_rpc(u32 rpc_command, void *rpc_shared_buffer)
{
	switch (rpc_command & RPC_CRYPTO_COMMAND_MASK) {
	case RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR:
		dprintk(KERN_INFO "RPC_INSTALL_SHORTCUT_LOCK_ACCELERATOR\n");
		return tf_crypto_install_shortcut_lock_hwa(
			rpc_command, rpc_shared_buffer);

	case RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT:
		dprintk(KERN_INFO "RPC_LOCK_ACCELERATORS_SUSPEND_SHORTCUT\n");
		return tf_crypto_lock_hwas_suspend_shortcut(
			rpc_command, rpc_shared_buffer);

	case RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS:
		dprintk(KERN_INFO "RPC_RESUME_SHORTCUT_UNLOCK_ACCELERATORS\n");
		return tf_crypto_resume_shortcut_unlock_hwas(
			rpc_command, rpc_shared_buffer);

	case RPC_CLEAR_GLOBAL_KEY_CONTEXT:
		dprintk(KERN_INFO "RPC_CLEAR_GLOBAL_KEY_CONTEXT\n");
		return tf_crypto_clear_global_key_context(
			rpc_command, rpc_shared_buffer);
	}

	return -1;
}
