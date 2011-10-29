/*
 * hdcp_top.c
 *
 * HDCP interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Fabrice Olivero
 *	Fabrice Olivero <f-olivero@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/firmware.h>
#include "../../hdmi_ti_4xxx_ip.h"
#include "../dss/dss.h"
#include "hdcp.h"

struct hdcp hdcp;
struct hdcp_sha_in sha_input;

/* State machine / workqueue */
static void hdcp_wq_disable(void);
static void hdcp_wq_start_authentication(void);
static void hdcp_wq_check_r0(void);
static void hdcp_wq_step2_authentication(void);
static void hdcp_wq_authentication_failure(void);
static void hdcp_work_queue(struct work_struct *work);
static struct delayed_work *hdcp_submit_work(int event, int delay);
static void hdcp_cancel_work(struct delayed_work **work);

/* Callbacks */
static void hdcp_start_frame_cb(void);
static void hdcp_irq_cb(int hpd_low);

/* Control */
static long hdcp_enable_ctl(void __user *argp);
static long hdcp_disable_ctl(void);
static long hdcp_query_status_ctl(void __user *argp);
static long hdcp_encrypt_key_ctl(void __user *argp);

/* Driver */
static int __init hdcp_init(void);
static void __exit hdcp_exit(void);

struct completion hdcp_comp;
static DECLARE_WAIT_QUEUE_HEAD(hdcp_up_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(hdcp_down_wait_queue);

#define DSS_POWER

/*-----------------------------------------------------------------------------
 * Function: hdcp_request_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_request_dss(void)
{
#ifdef DSS_POWER
	hdcp.dss_state = dss_runtime_get();
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_user_space_task
 *-----------------------------------------------------------------------------
 */
int hdcp_user_space_task(int flags)
{
	int ret;

	DBG("Wait for user space task %x\n", flags);
	hdcp.hdcp_up_event = flags & 0xFF;
	hdcp.hdcp_down_event = flags & 0xFF;
	wake_up_interruptible(&hdcp_up_wait_queue);
	wait_event_interruptible(hdcp_down_wait_queue,
				 (hdcp.hdcp_down_event & 0xFF) == 0);
	ret = (hdcp.hdcp_down_event & 0xFF00) >> 8;

	DBG("User space task done %x\n", hdcp.hdcp_down_event);
	hdcp.hdcp_down_event = 0;

	return ret;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_release_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_release_dss(void)
{
#ifdef DSS_POWER
	if (hdcp.dss_state == 0)
		dss_runtime_put();
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_disable
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_disable(void)
{
	printk(KERN_INFO "HDCP: disabled\n");

	hdcp_cancel_work(&hdcp.pending_wq_event);
	hdcp_lib_disable();
	hdcp.pending_disable = 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_start_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_start_authentication(void)
{
	int status = HDCP_OK;

	hdcp.hdcp_state = HDCP_AUTHENTICATION_START;

	printk(KERN_INFO "HDCP: authentication start\n");

	/* Step 1 part 1 (until R0 calc delay) */
	status = hdcp_lib_step1_start();

	if (status == -HDCP_AKSV_ERROR) {
		hdcp_wq_authentication_failure();
	} else if (status == -HDCP_CANCELLED_AUTH) {
		DBG("Authentication step 1 cancelled.");
		return;
	} else if (status != HDCP_OK) {
		hdcp_wq_authentication_failure();
	} else {
		hdcp.hdcp_state = HDCP_WAIT_R0_DELAY;
		hdcp.auth_state = HDCP_STATE_AUTH_1ST_STEP;
		hdcp.pending_wq_event = hdcp_submit_work(HDCP_R0_EXP_EVENT,
							 HDCP_R0_DELAY);
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_check_r0
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_check_r0(void)
{
	int status = hdcp_lib_step1_r0_check();

	if (status == -HDCP_CANCELLED_AUTH) {
		DBG("Authentication step 1/R0 cancelled.");
		return;
	} else if (status < 0)
		hdcp_wq_authentication_failure();
	else {
		if (hdcp_lib_check_repeater_bit_in_tx()) {
			/* Repeater */
			printk(KERN_INFO "HDCP: authentication step 1 "
					 "successful - Repeater\n");

			hdcp.hdcp_state = HDCP_WAIT_KSV_LIST;
			hdcp.auth_state = HDCP_STATE_AUTH_2ND_STEP;

			hdcp.pending_wq_event =
				hdcp_submit_work(HDCP_KSV_TIMEOUT_EVENT,
						 HDCP_KSV_TIMEOUT_DELAY);
		} else {
			/* Receiver */
			printk(KERN_INFO "HDCP: authentication step 1 "
					 "successful - Receiver\n");

			hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
			hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;

			/* Restore retry counter */
			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
		}
	}
}


/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_step2_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_step2_authentication(void)
{
	int status = HDCP_OK;

	/* KSV list timeout is running and should be canceled */
	hdcp_cancel_work(&hdcp.pending_wq_event);

	status = hdcp_lib_step2();

	if (status == -HDCP_CANCELLED_AUTH) {
		DBG("Authentication step 2 cancelled.");
		return;
	} else if (status < 0)
		hdcp_wq_authentication_failure();
	else {
		printk(KERN_INFO "HDCP: (Repeater) authentication step 2 "
				 "successful\n");

		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;

		/* Restore retry counter */
		if (hdcp.en_ctrl->nb_retry == 0)
			hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
		else
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_authentication_failure
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_authentication_failure(void)
{
	if (hdcp.hdmi_state == HDMI_STOPPED) {
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
		return;
	}

	hdcp_lib_auto_ri_check(false);
	hdcp_lib_auto_bcaps_rdy_check(false);
	hdcp_lib_set_av_mute(AV_MUTE_SET);
	hdcp_lib_set_encryption(HDCP_ENC_OFF);

	hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp_lib_disable();
	hdcp.pending_disable = 0;

	if (hdcp.retry_cnt && (hdcp.hdmi_state != HDMI_STOPPED)) {
		if (hdcp.retry_cnt < HDCP_INFINITE_REAUTH) {
			hdcp.retry_cnt--;
			printk(KERN_INFO "HDCP: authentication failed - "
					 "retrying, attempts=%d\n",
							hdcp.retry_cnt);
		} else
			printk(KERN_INFO "HDCP: authentication failed - "
					 "retrying\n");

		hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
		hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;

		hdcp.pending_wq_event = hdcp_submit_work(HDCP_AUTH_REATT_EVENT,
							 HDCP_REAUTH_DELAY);
	} else {
		printk(KERN_INFO "HDCP: authentication failed - "
				 "HDCP disabled\n");
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
	}

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_work_queue
 *-----------------------------------------------------------------------------
 */
static void hdcp_work_queue(struct work_struct *work)
{
	struct hdcp_delayed_work *hdcp_w =
		container_of(work, struct hdcp_delayed_work, work.work);
	int event = hdcp_w->event;

	mutex_lock(&hdcp.lock);

	DBG("hdcp_work_queue() - START - %u hdmi=%d hdcp=%d auth=%d evt= %x %d"
	    " hdcp_ctrl=%02x",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF,
		RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__HDCP_CTRL));

	/* Clear pending_wq_event
	 * In case a delayed work is scheduled from the state machine
	 * "pending_wq_event" is used to memorize pointer on the event to be
	 * able to cancel any pending work in case HDCP is disabled
	 */
	if (event & HDCP_WORKQUEUE_SRC)
		hdcp.pending_wq_event = 0;

	/* First handle HDMI state */
	if (event == HDCP_START_FRAME_EVENT) {
		hdcp.pending_start = 0;
		hdcp.hdmi_state = HDMI_STARTED;
	}
	/**********************/
	/* HDCP state machine */
	/**********************/
	switch (hdcp.hdcp_state) {

	/* State */
	/*********/
	case HDCP_DISABLED:
		/* HDCP enable control or re-authentication event */
		if (event == HDCP_ENABLE_CTL) {
			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;

			if (hdcp.hdmi_state == HDMI_STARTED)
				hdcp_wq_start_authentication();
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		}

		break;

	/* State */
	/*********/
	case HDCP_ENABLE_PENDING:
		/* HDMI start frame event */
		if (event == HDCP_START_FRAME_EVENT)
			hdcp_wq_start_authentication();

		break;

	/* State */
	/*********/
	case HDCP_AUTHENTICATION_START:
		/* Re-authentication */
		if (event == HDCP_AUTH_REATT_EVENT)
			hdcp_wq_start_authentication();

		break;

	/* State */
	/*********/
	case HDCP_WAIT_R0_DELAY:
		/* R0 timer elapsed */
		if (event == HDCP_R0_EXP_EVENT)
			hdcp_wq_check_r0();

		break;

	/* State */
	/*********/
	case HDCP_WAIT_KSV_LIST:
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			printk(KERN_INFO "HDCP: Ri check failure\n");

			hdcp_wq_authentication_failure();
		}
		/* KSV list ready event */
		else if (event == HDCP_KSV_LIST_RDY_EVENT)
			hdcp_wq_step2_authentication();
		/* Timeout */
		else if (event == HDCP_KSV_TIMEOUT_EVENT) {
			printk(KERN_INFO "HDCP: BCAPS polling timeout\n");
			hdcp_wq_authentication_failure();
		}
		break;

	/* State */
	/*********/
	case HDCP_LINK_INTEGRITY_CHECK:
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			printk(KERN_INFO "HDCP: Ri check failure\n");
			hdcp_wq_authentication_failure();
		}
		break;

	default:
		printk(KERN_WARNING "HDCP: error - unknow HDCP state\n");
		break;
	}

	kfree(hdcp_w);
	hdcp_w = 0;
	if (event == HDCP_START_FRAME_EVENT)
		hdcp.pending_start = 0;
	if (event == HDCP_KSV_LIST_RDY_EVENT ||
	    event == HDCP_R0_EXP_EVENT) {
		hdcp.pending_wq_event = 0;
	}

	DBG("hdcp_work_queue() - END - %u hdmi=%d hdcp=%d auth=%d evt=%x %d ",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF);

	mutex_unlock(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_submit_work
 *-----------------------------------------------------------------------------
 */
static struct delayed_work *hdcp_submit_work(int event, int delay)
{
	struct hdcp_delayed_work *work;

	work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_ATOMIC);

	if (work) {
		INIT_DELAYED_WORK(&work->work, hdcp_work_queue);
		work->event = event;
		queue_delayed_work(hdcp.workqueue,
				   &work->work,
				   msecs_to_jiffies(delay));
	} else {
		printk(KERN_WARNING "HDCP: Cannot allocate memory to "
				    "create work\n");
		return 0;
	}

	return &work->work;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_cancel_work
 *-----------------------------------------------------------------------------
 */
static void hdcp_cancel_work(struct delayed_work **work)
{
	int ret = 0;

	if (*work) {
		ret = cancel_delayed_work(*work);
		if (ret != 1) {
			ret = cancel_work_sync(&((*work)->work));
			printk(KERN_INFO "Canceling work failed - "
					 "cancel_work_sync done %d\n", ret);
		}
		kfree(*work);
		*work = 0;
	}
}


/******************************************************************************
 * HDCP callbacks
 *****************************************************************************/

/*-----------------------------------------------------------------------------
 * Function: hdcp_3des_cb
 *-----------------------------------------------------------------------------
 */
static bool hdcp_3des_cb(void)
{
	DBG("hdcp_3des_cb() %u", jiffies_to_msecs(jiffies));

	if (!hdcp.hdcp_keys_loaded) {
		printk(KERN_ERR "%s: hdcp_keys not loaded = %d",
		       __func__, hdcp.hdcp_keys_loaded);
		return false;
	}

	/* Load 3DES key */
	if (hdcp_3des_load_key(hdcp.en_ctrl->key) != HDCP_OK) {
		printk(KERN_ERR "Error Loading  HDCP keys\n");
		return false;
	}
	return true;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_start_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_start_frame_cb(void)
{
	DBG("hdcp_start_frame_cb() %u", jiffies_to_msecs(jiffies));

	if (!hdcp.hdcp_keys_loaded) {
		DBG("%s: hdcp_keys not loaded = %d",
		    __func__, hdcp.hdcp_keys_loaded);
		return;
	}

	/* Cancel any pending work */
	if (hdcp.pending_start)
		hdcp_cancel_work(&hdcp.pending_start);
	if (hdcp.pending_wq_event)
		hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp.hpd_low = 0;
	hdcp.pending_disable = 0;
	hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	hdcp.pending_start = hdcp_submit_work(HDCP_START_FRAME_EVENT,
							HDCP_ENABLE_DELAY);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_irq_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_irq_cb(int status)
{
	DBG("hdcp_irq_cb() status=%x", status);

	if (!hdcp.hdcp_keys_loaded) {
		DBG("%s: hdcp_keys not loaded = %d",
		    __func__, hdcp.hdcp_keys_loaded);
		return;
	}

	/* Disable auto Ri/BCAPS immediately */
	if (((status & HDMI_RI_ERR) ||
	    (status & HDMI_BCAP) ||
	    (status & HDMI_HPD_LOW)) &&
	    (hdcp.hdcp_state != HDCP_ENABLE_PENDING)) {
		hdcp_lib_auto_ri_check(false);
		hdcp_lib_auto_bcaps_rdy_check(false);
	}

	/* Work queue execution not required if HDCP is disabled */
	/* TODO: ignore interrupts if they are masked (cannnot access UMASK
	 * here so should use global variable
	 */
	if ((hdcp.hdcp_state != HDCP_DISABLED) &&
	    (hdcp.hdcp_state != HDCP_ENABLE_PENDING)) {
		if (status & HDMI_HPD_LOW) {
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp_ddc_abort();
		}

		if (status & HDMI_RI_ERR) {
			hdcp_lib_set_av_mute(AV_MUTE_SET);
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp_submit_work(HDCP_RI_FAIL_EVENT, 0);
		}
		/* RI error takes precedence over BCAP */
		else if (status & HDMI_BCAP)
			hdcp_submit_work(HDCP_KSV_LIST_RDY_EVENT, 0);
	}

	if (status & HDMI_HPD_LOW) {
		hdcp.pending_disable = 1;	/* Used to exit on-going HDCP
						 * work */
		hdcp.hpd_low = 0;		/* Used to cancel HDCP works */
		hdcp_lib_disable();
		/* In case of HDCP_STOP_FRAME_EVENT, HDCP stop
		 * frame callback is blocked and waiting for
		 * HDCP driver to finish accessing the HW
		 * before returning
		 * Reason is to avoid HDMI driver to shutdown
		 * DSS/HDMI power before HDCP work is finished
		 */
		hdcp.hdmi_state = HDMI_STOPPED;
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_DISABLED;
	}
}

/******************************************************************************
 * HDCP control from ioctl
 *****************************************************************************/


/*-----------------------------------------------------------------------------
 * Function: hdcp_enable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_enable_ctl(void __user *argp)
{
	DBG("hdcp_ioctl() - ENABLE %u", jiffies_to_msecs(jiffies));

	if (hdcp.en_ctrl == 0) {
		hdcp.en_ctrl =
			kmalloc(sizeof(struct hdcp_enable_control),
							GFP_KERNEL);

		if (hdcp.en_ctrl == 0) {
			printk(KERN_WARNING
				"HDCP: Cannot allocate memory for HDCP"
				" enable control struct\n");
			return -EFAULT;
		}
	}

	if (copy_from_user(hdcp.en_ctrl, argp,
			   sizeof(struct hdcp_enable_control))) {
		printk(KERN_WARNING "HDCP: Error copying from user space "
				    "- enable ioctl\n");
		return -EFAULT;
	}

	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_ENABLE_CTL, 0) == 0)
		return -EFAULT;

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_disable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_disable_ctl(void)
{
	DBG("hdcp_ioctl() - DISABLE %u", jiffies_to_msecs(jiffies));

	hdcp_cancel_work(&hdcp.pending_start);
	hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp.pending_disable = 1;
	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_DISABLE_CTL, 0) == 0)
		return -EFAULT;

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_query_status_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_query_status_ctl(void __user *argp)
{
	uint32_t *status = (uint32_t *)argp;

	DBG("hdcp_ioctl() - QUERY %u", jiffies_to_msecs(jiffies));

	*status = hdcp.auth_state;

	return 0;
}

static int hdcp_wait_re_entrance;

/*-----------------------------------------------------------------------------
 * Function: hdcp_wait_event_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_wait_event_ctl(void __user *argp)
{
	struct hdcp_wait_control ctrl;

	DBG("hdcp_ioctl() - WAIT %u %d", jiffies_to_msecs(jiffies),
					 hdcp.hdcp_up_event);

	if (copy_from_user(&ctrl, argp,
			   sizeof(struct hdcp_wait_control))) {
		printk(KERN_WARNING "HDCP: Error copying from user space"
				    " - wait ioctl");
		return -EFAULT;
	}

	if (hdcp_wait_re_entrance == 0) {
		hdcp_wait_re_entrance = 1;
		wait_event_interruptible(hdcp_up_wait_queue,
					 (hdcp.hdcp_up_event & 0xFF) != 0);

		ctrl.event = hdcp.hdcp_up_event;

		if ((ctrl.event & 0xFF) == HDCP_EVENT_STEP2) {
			if (copy_to_user(ctrl.data, &sha_input,
						sizeof(struct hdcp_sha_in))) {
				printk(KERN_WARNING "HDCP: Error copying to "
						    "user space - wait ioctl");
				return -EFAULT;
			}
		}

		hdcp.hdcp_up_event = 0;
		hdcp_wait_re_entrance = 0;
	} else
		ctrl.event = HDCP_EVENT_EXIT;

	/* Store output data to output pointer */
	if (copy_to_user(argp, &ctrl,
			 sizeof(struct hdcp_wait_control))) {
		printk(KERN_WARNING "HDCP: Error copying to user space -"
				    " wait ioctl");
		return -EFAULT;
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_done_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_done_ctl(void __user *argp)
{
	uint32_t *status = (uint32_t *)argp;

	DBG("hdcp_ioctl() - DONE %u %d", jiffies_to_msecs(jiffies), *status);

	hdcp.hdcp_down_event &= ~(*status & 0xFF);
	hdcp.hdcp_down_event |= *status & 0xFF00;

	wake_up_interruptible(&hdcp_down_wait_queue);

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_encrypt_key_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_encrypt_key_ctl(void __user *argp)
{
	struct hdcp_encrypt_control *ctrl;
	uint32_t *out_key;

	DBG("hdcp_ioctl() - ENCRYPT KEY %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	if (hdcp.hdcp_state != HDCP_DISABLED) {
		printk(KERN_INFO "HDCP: Cannot encrypt keys while HDCP "
				   "is enabled\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	hdcp.hdcp_state = HDCP_KEY_ENCRYPTION_ONGOING;

	/* Encryption happens in ioctl / user context */
	ctrl = kmalloc(sizeof(struct hdcp_encrypt_control),
		       GFP_KERNEL);

	if (ctrl == 0) {
		printk(KERN_WARNING "HDCP: Cannot allocate memory for HDCP"
				    " encryption control struct\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	out_key = kmalloc(sizeof(uint32_t) *
					DESHDCP_KEY_SIZE, GFP_KERNEL);

	if (out_key == 0) {
		printk(KERN_WARNING "HDCP: Cannot allocate memory for HDCP "
				    "encryption output key\n");
		kfree(ctrl);
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	if (copy_from_user(ctrl, argp,
				sizeof(struct hdcp_encrypt_control))) {
		printk(KERN_WARNING "HDCP: Error copying from user space"
				    " - encrypt ioctl\n");
		kfree(ctrl);
		kfree(out_key);
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	hdcp_request_dss();

	/* Call encrypt function */
	hdcp_3des_encrypt_key(ctrl, out_key);

	hdcp_release_dss();

	hdcp.hdcp_state = HDCP_DISABLED;
	mutex_unlock(&hdcp.lock);

	/* Store output data to output pointer */
	if (copy_to_user(ctrl->out_key, out_key,
				sizeof(uint32_t)*DESHDCP_KEY_SIZE)) {
		printk(KERN_WARNING "HDCP: Error copying to user space -"
				    " encrypt ioctl\n");
		kfree(ctrl);
		kfree(out_key);
		return -EFAULT;
	}

	kfree(ctrl);
	kfree(out_key);
	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HDCP_ENABLE:
		return hdcp_enable_ctl(argp);

	case HDCP_DISABLE:
		return hdcp_disable_ctl();

	case HDCP_ENCRYPT_KEY:
		return hdcp_encrypt_key_ctl(argp);

	case HDCP_QUERY_STATUS:
		return hdcp_query_status_ctl(argp);

	case HDCP_WAIT_EVENT:
		return hdcp_wait_event_ctl(argp);

	case HDCP_DONE:
		return hdcp_done_ctl(argp);

	default:
		return -ENOTTY;
	} /* End switch */
}


/******************************************************************************
 * HDCP driver init/exit
 *****************************************************************************/

/*-----------------------------------------------------------------------------
 * Function: hdcp_mmap
 *-----------------------------------------------------------------------------
 */
static int hdcp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int status;

	DBG("hdcp_mmap() %lx %lx %lx\n", vma->vm_start, vma->vm_pgoff,
					 vma->vm_end - vma->vm_start);

	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	status = remap_pfn_range(vma, vma->vm_start,
				 HDMI_WP >> PAGE_SHIFT,
				 vma->vm_end - vma->vm_start,
				 vma->vm_page_prot);
	if (status) {
		DBG("mmap error %d\n", status);
		return -EAGAIN;
	}

	DBG("mmap succesfull\n");
	return 0;
}

static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.mmap = hdcp_mmap,
	.unlocked_ioctl = hdcp_ioctl,
};

struct miscdevice mdev;

static void hdcp_load_keys_cb(const struct firmware *fw, void *context)
{
	struct hdcp_enable_control *en_ctrl;

	if (!fw) {
		pr_err("HDCP: failed to load keys\n");
		return;
	}

	if (fw->size != sizeof(en_ctrl->key)) {
		pr_err("HDCP: encrypted key file wrong size %d\n", fw->size);
		return;
	}

	en_ctrl = kmalloc(sizeof(*en_ctrl), GFP_KERNEL);
	if (!en_ctrl) {
		pr_err("HDCP: can't allocated space for keys\n");
		return;
	}

	memcpy(en_ctrl->key, fw->data, sizeof(en_ctrl->key));
	en_ctrl->nb_retry = 20;

	hdcp.en_ctrl = en_ctrl;
	hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	hdcp.hdcp_state = HDCP_ENABLE_PENDING;
	hdcp.hdcp_keys_loaded = true;
	pr_info("HDCP: loaded keys\n");
}

static int hdcp_load_keys(void)
{
	int ret;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				      "hdcp.keys", mdev.this_device, GFP_KERNEL,
				      &hdcp, hdcp_load_keys_cb);
	if (ret < 0) {
		pr_err("HDCP: request_firmware_nowait failed: %d\n", ret);
		hdcp.hdcp_keys_loaded = false;
		return ret;
	}

	return 0;
}


/*-----------------------------------------------------------------------------
 * Function: hdcp_init
 *-----------------------------------------------------------------------------
 */
static int __init hdcp_init(void)
{
	DBG("hdcp_init() %u", jiffies_to_msecs(jiffies));

	/* Map HDMI WP address */
	hdcp.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!hdcp.hdmi_wp_base_addr) {
		printk(KERN_ERR "HDCP: HDMI WP IOremap error\n");
		return -EFAULT;
	}

	/* Map DESHDCP in kernel address space */
	hdcp.deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp.deshdcp_base_addr) {
		printk(KERN_ERR "HDCP: DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp.lock);

	mdev.minor = MISC_DYNAMIC_MINOR;
	mdev.name = "hdcp";
	mdev.mode = 0666;
	mdev.fops = &hdcp_fops;

	if (misc_register(&mdev)) {
		printk(KERN_ERR "HDCP: Could not add character driver\n");
		goto err_register;
	}

	mutex_lock(&hdcp.lock);

	/* Variable init */
	hdcp.en_ctrl  = 0;
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.pending_wq_event = 0;
	hdcp.retry_cnt = 0;
	hdcp.auth_state = HDCP_STATE_DISABLED;
	hdcp.pending_disable = 0;
	hdcp.hdcp_up_event = 0;
	hdcp.hdcp_down_event = 0;
	hdcp_wait_re_entrance = 0;
	hdcp.hpd_low = 0;

	spin_lock_init(&hdcp.spinlock);

	init_completion(&hdcp_comp);

	hdcp.workqueue = create_singlethread_workqueue("hdcp");
	if (hdcp.workqueue == NULL)
		goto err_add_driver;

	hdcp_request_dss();

	/* Register HDCP callbacks to HDMI library */
	if (omapdss_hdmi_register_hdcp_callbacks(&hdcp_start_frame_cb,
						 &hdcp_irq_cb,
						 &hdcp_3des_cb))
		hdcp.hdmi_state = HDMI_STARTED;
	else
		hdcp.hdmi_state = HDMI_STOPPED;

	hdcp_release_dss();

	mutex_unlock(&hdcp.lock);

	hdcp_load_keys();

	return 0;

err_add_driver:
	misc_deregister(&mdev);

err_register:
	mutex_destroy(&hdcp.lock);

	iounmap(hdcp.deshdcp_base_addr);

err_map_deshdcp:
	iounmap(hdcp.hdmi_wp_base_addr);

	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_exit
 *-----------------------------------------------------------------------------
 */
static void __exit hdcp_exit(void)
{
	DBG("hdcp_exit() %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	kfree(hdcp.en_ctrl);

	hdcp_request_dss();

	/* Un-register HDCP callbacks to HDMI library */
	omapdss_hdmi_register_hdcp_callbacks(0, 0, 0);

	hdcp_release_dss();

	misc_deregister(&mdev);

	/* Unmap HDMI WP / DESHDCP */
	iounmap(hdcp.hdmi_wp_base_addr);
	iounmap(hdcp.deshdcp_base_addr);

	destroy_workqueue(hdcp.workqueue);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Fabrice Olivero");
