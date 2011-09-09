/*
 * OMX offloading remote processor driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/sched.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg_omx.h>
#include <linux/completion.h>

#include <mach/tiler.h>

#ifdef CONFIG_ION_OMAP
#include <linux/ion.h>
#include <linux/omap_ion.h>

extern struct ion_device *omap_ion_device;
#endif

/* maximum OMX devices this driver can handle */
#define MAX_OMX_DEVICES		8

enum rpc_omx_map_info_type {
	RPC_OMX_MAP_INFO_NONE          = 0,
	RPC_OMX_MAP_INFO_ONE_BUF       = 1,
	RPC_OMX_MAP_INFO_TWO_BUF       = 2,
	RPC_OMX_MAP_INFO_THREE_BUF     = 3,
	RPC_OMX_MAP_INFO_MAX           = 0x7FFFFFFF
};

enum {
	OMX_SERVICE_DOWN,
	OMX_SERVICE_UP
};

struct rpmsg_omx_service {
	struct list_head next;
	struct cdev cdev;
	struct device *dev;
	struct rpmsg_channel *rpdev;
	int minor;
	struct list_head list;
	struct mutex lock;
	struct completion comp;
	int state;
#ifdef CONFIG_ION_OMAP
	struct ion_client *ion_client;
#endif
};

struct rpmsg_omx_instance {
	struct list_head next;
	struct rpmsg_omx_service *omxserv;
	struct sk_buff_head queue;
	struct mutex lock;
	wait_queue_head_t readq;
	struct completion reply_arrived;
	struct rpmsg_endpoint *ept;
	u32 dst;
	int state;
#ifdef CONFIG_ION_OMAP
	struct ion_client *ion_client;
#endif
};

static struct class *rpmsg_omx_class;
static dev_t rpmsg_omx_dev;

/* store all remote omx connection services (usually one per remoteproc) */
static DEFINE_IDR(rpmsg_omx_services);
static DEFINE_SPINLOCK(rpmsg_omx_services_lock);
static LIST_HEAD(rpmsg_omx_services_list);

#ifdef CONFIG_ION_OMAP
#ifdef CONFIG_PVR_SGX
#include "../gpu/pvr/ion.h"
#endif
#endif

/*
 * TODO: Need to do this using lookup with rproc, but rproc is not
 * visible to rpmsg_omx
 */
#define TILER_START	0x60000000
#define TILER_END	0x80000000
#define ION_1D_START	0xBA300000
#define ION_1D_END	0xBFD00000
#define ION_1D_VA	0x88000000
static u32 _rpmsg_pa_to_da(u32 pa)
{
	if (pa >= TILER_START && pa < TILER_END)
		return pa;
	else if (pa >= ION_1D_START && pa < ION_1D_END)
		return (pa - ION_1D_START + ION_1D_VA);
	else
		return 0;
}

static u32 _rpmsg_omx_buffer_lookup(struct rpmsg_omx_instance *omx, long buffer)
{
	phys_addr_t pa;
	u32 va;
#ifdef CONFIG_ION_OMAP
	struct ion_handle *handle;
	ion_phys_addr_t paddr;
	size_t unused;
	int fd;

	/* is it an ion handle? */
	handle = (struct ion_handle *)buffer;
	if (!ion_phys(omx->ion_client, handle, &paddr, &unused)) {
		pa = (phys_addr_t) paddr;
		goto to_va;
	}

#ifdef CONFIG_PVR_SGX
	/* how about an sgx buffer wrapping an ion handle? */
	{
		struct ion_client *pvr_ion_client;
		fd = buffer;
		handle = PVRSRVExportFDToIONHandle(fd, &pvr_ion_client);
		if (handle &&
			!ion_phys(pvr_ion_client, handle, &paddr, &unused)) {
			pa = (phys_addr_t)paddr;
			goto to_va;
		}
	}
#endif
#endif
	pa = (phys_addr_t) tiler_virt2phys(buffer);

#ifdef CONFIG_ION_OMAP
to_va:
#endif
	va = _rpmsg_pa_to_da(pa);
	return va;
}

static int _rpmsg_omx_map_buf(struct rpmsg_omx_instance *omx, char *packet)
{
	int ret = -EINVAL, offset = 0;
	long *buffer;
	char *data;
	enum rpc_omx_map_info_type maptype;
	u32 da = 0;

	data = (char *)((struct omx_packet *)packet)->data;
	maptype = *((enum rpc_omx_map_info_type *)data);

	/*Nothing to map*/
	if (maptype == RPC_OMX_MAP_INFO_NONE)
		return 0;
	if ((maptype != RPC_OMX_MAP_INFO_THREE_BUF) &&
		(maptype != RPC_OMX_MAP_INFO_TWO_BUF) &&
			(maptype != RPC_OMX_MAP_INFO_ONE_BUF))
		return ret;

	offset = *(int *)((int)data + sizeof(maptype));
	buffer = (long *)((int)data + offset);

	da = _rpmsg_omx_buffer_lookup(omx, *buffer);
	if (da) {
		*buffer = da;
		ret = 0;
	}

	if (!ret && (maptype >= RPC_OMX_MAP_INFO_TWO_BUF)) {
		buffer = (long *)((int)data + offset + sizeof(*buffer));
		if (*buffer != 0) {
			ret = -EIO;
			da = _rpmsg_omx_buffer_lookup(omx, *buffer);
			if (da) {
				*buffer = da;
				ret = 0;
			}
		}
	}

	if (!ret && maptype >= RPC_OMX_MAP_INFO_THREE_BUF) {
		buffer = (long *)((int)data + offset + 2*sizeof(*buffer));
		if (*buffer != 0) {
			ret = -EIO;
			da = _rpmsg_omx_buffer_lookup(omx, *buffer);
			if (da) {
				*buffer = da;
				ret = 0;
			}
		}
	}
	return ret;
}

static void rpmsg_omx_cb(struct rpmsg_channel *rpdev, void *data, int len,
							void *priv, u32 src)
{
	struct omx_msg_hdr *hdr = data;
	struct rpmsg_omx_instance *omx = priv;
	struct omx_conn_rsp *rsp;
	struct sk_buff *skb;
	char *skbdata;

	if (len < sizeof(*hdr) || hdr->len < len - sizeof(*hdr)) {
		dev_warn(&rpdev->dev, "%s: truncated message\n", __func__);
		return;
	}

	dev_dbg(&rpdev->dev, "%s: incoming msg src 0x%x type %d len %d\n",
					__func__, src, hdr->type, hdr->len);
#if 0
	print_hex_dump(KERN_DEBUG, "rpmsg_omx RX: ", DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
#endif

	switch (hdr->type) {
	case OMX_CONN_RSP:
		if (hdr->len < sizeof(*rsp)) {
			dev_warn(&rpdev->dev, "incoming empty response msg\n");
			break;
		}
		rsp = (struct omx_conn_rsp *) hdr->data;
		dev_info(&rpdev->dev, "conn rsp: status %d addr %d\n",
			       rsp->status, rsp->addr);
		omx->dst = rsp->addr;
		if (rsp->status)
			omx->state = OMX_FAIL;
		else
			omx->state = OMX_CONNECTED;
		complete(&omx->reply_arrived);
		break;
	case OMX_RAW_MSG:
		skb = alloc_skb(hdr->len, GFP_KERNEL);
		if (!skb) {
			dev_err(&rpdev->dev, "alloc_skb err: %u\n", hdr->len);
			break;
		}
		skbdata = skb_put(skb, hdr->len);
		memcpy(skbdata, hdr->data, hdr->len);

		mutex_lock(&omx->lock);
		skb_queue_tail(&omx->queue, skb);
		mutex_unlock(&omx->lock);
		/* wake up any blocking processes, waiting for new data */
		wake_up_interruptible(&omx->readq);
		break;
	default:
		dev_warn(&rpdev->dev, "unexpected msg type: %d\n", hdr->type);
		break;
	}
}

static int rpmsg_omx_connect(struct rpmsg_omx_instance *omx, char *omxname)
{
	struct omx_msg_hdr *hdr;
	struct omx_conn_req *payload;
	struct rpmsg_omx_service *omxserv = omx->omxserv;
	char connect_msg[sizeof(*hdr) + sizeof(*payload)] = { 0 };
	int ret;

	if (omx->state == OMX_CONNECTED) {
		dev_dbg(omxserv->dev, "endpoint already connected\n");
		return -EISCONN;
	}

	hdr = (struct omx_msg_hdr *)connect_msg;
	hdr->type = OMX_CONN_REQ;
	hdr->flags = 0;
	hdr->len = strlen(omxname) + 1;
	payload = (struct omx_conn_req *)hdr->data;
	strcpy(payload->name, omxname);

	init_completion(&omx->reply_arrived);

	/* send a conn req to the remote OMX connection service. use
	 * the new local address that was just allocated by ->open */
	ret = rpmsg_send_offchannel(omxserv->rpdev, omx->ept->addr,
			omxserv->rpdev->dst, connect_msg, sizeof(connect_msg));
	if (ret) {
		dev_err(omxserv->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	/* wait until a connection reply arrives or 5 seconds elapse */
	ret = wait_for_completion_interruptible_timeout(&omx->reply_arrived,
						msecs_to_jiffies(5000));
	if (omx->state == OMX_CONNECTED)
		return 0;

	if (omx->state == OMX_FAIL)
		return -ENXIO;

	if (ret) {
		dev_err(omxserv->dev, "premature wakeup: %d\n", ret);
		return -EIO;
	}

	return -ETIMEDOUT;
}

static
long rpmsg_omx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rpmsg_omx_instance *omx = filp->private_data;
	struct rpmsg_omx_service *omxserv = omx->omxserv;
	char buf[48];
	int ret = 0;

	dev_dbg(omxserv->dev, "%s: cmd %d, arg 0x%lx\n", __func__, cmd, arg);

	if (_IOC_TYPE(cmd) != OMX_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > OMX_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case OMX_IOCCONNECT:
		ret = copy_from_user(buf, (char __user *) arg, sizeof(buf));
		if (ret) {
			dev_err(omxserv->dev,
				"%s: %d: copy_from_user fail: %d\n", __func__,
				_IOC_NR(cmd), ret);
			ret = -EFAULT;
			break;
		}
		/* make sure user input is null terminated */
		buf[sizeof(buf) - 1] = '\0';
		ret = rpmsg_omx_connect(omx, buf);
		break;
#ifdef CONFIG_ION_OMAP
	case OMX_IOCIONREGISTER:
	{
		struct ion_fd_data data;
		if (copy_from_user(&data, (char __user *) arg, sizeof(data))) {
			dev_err(omxserv->dev,
				"%s: %d: copy_from_user fail: %d\n", __func__,
				_IOC_NR(cmd), ret);
			return -EFAULT;
		}
		data.handle = ion_import_fd(omx->ion_client, data.fd);
		if (IS_ERR(data.handle))
			data.handle = NULL;
		if (copy_to_user(&data, (char __user *) arg, sizeof(data))) {
			dev_err(omxserv->dev,
				"%s: %d: copy_to_user fail: %d\n", __func__,
				_IOC_NR(cmd), ret);
			return -EFAULT;
		}
		break;
	}
	case OMX_IOCIONUNREGISTER:
	{
		struct ion_fd_data data;
		if (copy_from_user(&data, (char __user *) arg, sizeof(data))) {
			dev_err(omxserv->dev,
				"%s: %d: copy_from_user fail: %d\n", __func__,
				_IOC_NR(cmd), ret);
			return -EFAULT;
		}
		ion_free(omx->ion_client, data.handle);
		if (copy_to_user(&data, (char __user *) arg, sizeof(data))) {
			dev_err(omxserv->dev,
				"%s: %d: copy_to_user fail: %d\n", __func__,
				_IOC_NR(cmd), ret);
			return -EFAULT;
		}
		break;
	}
#endif
	default:
		dev_warn(omxserv->dev, "unhandled ioctl cmd: %d\n", cmd);
		break;
	}

	return ret;
}

static int rpmsg_omx_open(struct inode *inode, struct file *filp)
{
	struct rpmsg_omx_service *omxserv;
	struct rpmsg_omx_instance *omx;

	omxserv = container_of(inode->i_cdev, struct rpmsg_omx_service, cdev);

	if (omxserv->state == OMX_SERVICE_DOWN)
		if (filp->f_flags & O_NONBLOCK ||
			      wait_for_completion_interruptible(&omxserv->comp))
			return -EBUSY;

	omx = kzalloc(sizeof(*omx), GFP_KERNEL);
	if (!omx)
		return -ENOMEM;

	mutex_init(&omx->lock);
	skb_queue_head_init(&omx->queue);
	init_waitqueue_head(&omx->readq);
	omx->omxserv = omxserv;
	omx->state = OMX_UNCONNECTED;

	/* assign a new, unique, local address and associate omx with it */
	omx->ept = rpmsg_create_ept(omxserv->rpdev, rpmsg_omx_cb, omx,
							RPMSG_ADDR_ANY);
	if (!omx->ept) {
		dev_err(omxserv->dev, "create ept failed\n");
		kfree(omx);
		return -ENOMEM;
	}
#ifdef CONFIG_ION_OMAP
	omx->ion_client = ion_client_create(omap_ion_device,
					    (1<< ION_HEAP_TYPE_CARVEOUT) |
					    (1 << OMAP_ION_HEAP_TYPE_TILER),
					    "rpmsg-omx");
#endif

	/* associate filp with the new omx instance */
	filp->private_data = omx;
	mutex_lock(&omxserv->lock);
	list_add(&omx->next, &omxserv->list);
	mutex_unlock(&omxserv->lock);

	dev_info(omxserv->dev, "local addr assigned: 0x%x\n", omx->ept->addr);

	return 0;
}

static int rpmsg_omx_release(struct inode *inode, struct file *filp)
{
	struct rpmsg_omx_instance *omx = filp->private_data;
	struct rpmsg_omx_service *omxserv = omx->omxserv;
	char kbuf[512];
	struct omx_msg_hdr *hdr = (struct omx_msg_hdr *) kbuf;
	struct omx_disc_req *disc_req = (struct omx_disc_req *)hdr->data;
	int use, ret;

	/* todo: release resources here */
	/*
	 * If state == fail, remote processor crashed, so don't send it
	 * any message.
	 */
	if (omx->state == OMX_FAIL)
		goto out;

	/* send a disconnect msg with the OMX instance addr */
	hdr->type = OMX_DISCONNECT;
	hdr->flags = 0;
	hdr->len = sizeof(struct omx_disc_req);
	disc_req->addr = omx->dst;
	use = sizeof(*hdr) + hdr->len;

	dev_info(omxserv->dev, "Disconnecting from OMX service at %d\n",
		omx->dst);

	/* send the msg to the remote OMX connection service */
	ret = rpmsg_send_offchannel(omxserv->rpdev, omx->ept->addr,
					omxserv->rpdev->dst, kbuf, use);
	if (ret) {
		dev_err(omxserv->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}
	rpmsg_destroy_ept(omx->ept);
out:
#ifdef CONFIG_ION_OMAP
	ion_client_destroy(omx->ion_client);
#endif
	mutex_lock(&omxserv->lock);
	list_del(&omx->next);
	mutex_unlock(&omxserv->lock);
	kfree(omx);

	return 0;
}

static ssize_t rpmsg_omx_read(struct file *filp, char __user *buf,
						size_t len, loff_t *offp)
{
	struct rpmsg_omx_instance *omx = filp->private_data;
	struct sk_buff *skb;
	int use;

	if (mutex_lock_interruptible(&omx->lock))
		return -ERESTARTSYS;

	if (omx->state == OMX_FAIL) {
		mutex_unlock(&omx->lock);
		return -ENXIO;
	}

	if (omx->state != OMX_CONNECTED) {
		mutex_unlock(&omx->lock);
		return -ENOTCONN;
	}

	/* nothing to read ? */
	if (skb_queue_empty(&omx->queue)) {
		mutex_unlock(&omx->lock);
		/* non-blocking requested ? return now */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		/* otherwise block, and wait for data */
		if (wait_event_interruptible(omx->readq,
				(!skb_queue_empty(&omx->queue) ||
				omx->state == OMX_FAIL)))
			return -ERESTARTSYS;
		if (mutex_lock_interruptible(&omx->lock))
			return -ERESTARTSYS;
	}

	if (omx->state == OMX_FAIL) {
		mutex_unlock(&omx->lock);
		return -ENXIO;
	}

	skb = skb_dequeue(&omx->queue);
	if (!skb) {
		mutex_unlock(&omx->lock);
		dev_err(omx->omxserv->dev, "err is rmpsg_omx racy ?\n");
		return -EIO;
	}

	mutex_unlock(&omx->lock);

	use = min(len, skb->len);

	if (copy_to_user(buf, skb->data, use)) {
		dev_err(omx->omxserv->dev, "%s: copy_to_user fail\n", __func__);
		use = -EFAULT;
	}

	kfree_skb(skb);
	return use;
}

static ssize_t rpmsg_omx_write(struct file *filp, const char __user *ubuf,
						size_t len, loff_t *offp)
{
	struct rpmsg_omx_instance *omx = filp->private_data;
	struct rpmsg_omx_service *omxserv = omx->omxserv;
	char kbuf[512];
	struct omx_msg_hdr *hdr = (struct omx_msg_hdr *) kbuf;
	int use, ret;

	if (omx->state != OMX_CONNECTED)
		return -ENOTCONN;

	/*
	 * for now, limit msg size to 512 bytes (incl. header).
	 * (note: rpmsg's limit is even tighter. this whole thing needs fixing)
	 */
	use = min(sizeof(kbuf) - sizeof(*hdr), len);

	/*
	 * copy the data. Later, number of copies can be optimized if found to
	 * be significant in real use cases
	 */
	if (copy_from_user(hdr->data, ubuf, use))
		return -EMSGSIZE;

	ret = _rpmsg_omx_map_buf(omx, hdr->data);
	if (ret < 0)
		return ret;

	hdr->type = OMX_RAW_MSG;
	hdr->flags = 0;
	hdr->len = use;

	use += sizeof(*hdr);

	ret = rpmsg_send_offchannel(omxserv->rpdev, omx->ept->addr,
						omx->dst, kbuf, use);
	if (ret) {
		dev_err(omxserv->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return use;
}

static
unsigned int rpmsg_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct rpmsg_omx_instance *omx = filp->private_data;
	unsigned int mask = 0;

	if (mutex_lock_interruptible(&omx->lock))
		return -ERESTARTSYS;

	poll_wait(filp, &omx->readq, wait);
	if (omx->state == OMX_FAIL) {
		mutex_unlock(&omx->lock);
		return -ENXIO;
	}

	if (!skb_queue_empty(&omx->queue))
		mask |= POLLIN | POLLRDNORM;

	/* implement missing rpmsg virtio functionality here */
	if (true)
		mask |= POLLOUT | POLLWRNORM;

	mutex_unlock(&omx->lock);

	return mask;
}

static const struct file_operations rpmsg_omx_fops = {
	.open		= rpmsg_omx_open,
	.release	= rpmsg_omx_release,
	.unlocked_ioctl	= rpmsg_omx_ioctl,
	.read		= rpmsg_omx_read,
	.write		= rpmsg_omx_write,
	.poll		= rpmsg_poll,
	.owner		= THIS_MODULE,
};

static int rpmsg_omx_probe(struct rpmsg_channel *rpdev)
{
	int ret, major, minor;
	struct rpmsg_omx_service *omxserv = NULL, *tmp;

	if (!idr_pre_get(&rpmsg_omx_services, GFP_KERNEL)) {
		dev_err(&rpdev->dev, "idr_pre_get failes\n");
		return -ENOMEM;
	}

	/* dynamically assign a new minor number */
	spin_lock(&rpmsg_omx_services_lock);
	ret = idr_get_new(&rpmsg_omx_services, omxserv, &minor);
	if (ret) {
		spin_unlock(&rpmsg_omx_services_lock);
		dev_err(&rpdev->dev, "failed to idr_get_new: %d\n", ret);
		return ret;
	}

	/* look for an already created omx service */
	list_for_each_entry(tmp, &rpmsg_omx_services_list, next) {
		if (tmp->minor == minor) {
			omxserv = tmp;
			idr_replace(&rpmsg_omx_services, omxserv, minor);
			break;
		}
	}
	spin_unlock(&rpmsg_omx_services_lock);
	if (omxserv)
		goto serv_up;

	omxserv = kzalloc(sizeof(*omxserv), GFP_KERNEL);
	if (!omxserv) {
		dev_err(&rpdev->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto rem_idr;
	}

	spin_lock(&rpmsg_omx_services_lock);
	idr_replace(&rpmsg_omx_services, omxserv, minor);
	spin_unlock(&rpmsg_omx_services_lock);
	INIT_LIST_HEAD(&omxserv->list);
	mutex_init(&omxserv->lock);
	init_completion(&omxserv->comp);

	list_add(&omxserv->next, &rpmsg_omx_services_list);

	major = MAJOR(rpmsg_omx_dev);

	cdev_init(&omxserv->cdev, &rpmsg_omx_fops);
	omxserv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&omxserv->cdev, MKDEV(major, minor), 1);
	if (ret) {
		dev_err(&rpdev->dev, "cdev_add failed: %d\n", ret);
		goto free_omx;
	}

	omxserv->dev = device_create(rpmsg_omx_class, &rpdev->dev,
			MKDEV(major, minor), NULL,
			"rpmsg-omx%d", minor);
	if (IS_ERR(omxserv->dev)) {
		ret = PTR_ERR(omxserv->dev);
		dev_err(&rpdev->dev, "device_create failed: %d\n", ret);
		goto clean_cdev;
	}
serv_up:
	omxserv->rpdev = rpdev;
	omxserv->minor = minor;
	omxserv->state = OMX_SERVICE_UP;
	dev_set_drvdata(&rpdev->dev, omxserv);
	complete_all(&omxserv->comp);

	dev_info(omxserv->dev, "new OMX connection srv channel: %u -> %u!\n",
						rpdev->src, rpdev->dst);
	return 0;

clean_cdev:
	cdev_del(&omxserv->cdev);
free_omx:
	kfree(omxserv);
rem_idr:
	spin_lock(&rpmsg_omx_services_lock);
	idr_remove(&rpmsg_omx_services, minor);
	spin_unlock(&rpmsg_omx_services_lock);
	return ret;
}

static void __devexit rpmsg_omx_remove(struct rpmsg_channel *rpdev)
{
	struct rpmsg_omx_service *omxserv = dev_get_drvdata(&rpdev->dev);
	int major = MAJOR(rpmsg_omx_dev);
	struct rpmsg_omx_instance *omx;

	dev_info(omxserv->dev, "rpmsg omx driver is removed\n");

	spin_lock(&rpmsg_omx_services_lock);
	idr_remove(&rpmsg_omx_services, omxserv->minor);
	spin_unlock(&rpmsg_omx_services_lock);

	mutex_lock(&omxserv->lock);
	/*
	 * If there is omx instrances that means it is a revovery.
	 * TODO: make sure it is a recovery.
	 */
	if (list_empty(&omxserv->list)) {
		device_destroy(rpmsg_omx_class, MKDEV(major, omxserv->minor));
		cdev_del(&omxserv->cdev);
		list_del(&omxserv->next);
		mutex_unlock(&omxserv->lock);
		kfree(omxserv);
		return;
	}
	/* If it is a recovery, don't clean the omxserv */
	init_completion(&omxserv->comp);
	omxserv->state = OMX_SERVICE_DOWN;
	list_for_each_entry(omx, &omxserv->list, next) {
		/* set omx instance to fail state */
		omx->state = OMX_FAIL;
		/* unblock any pending omx thread*/
		complete_all(&omx->reply_arrived);
		wake_up_interruptible(&omx->readq);
	}
	mutex_unlock(&omxserv->lock);
}

static void rpmsg_omx_driver_cb(struct rpmsg_channel *rpdev, void *data,
						int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "uhm, unexpected message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id rpmsg_omx_id_table[] = {
	{ .name	= "rpmsg-omx" },
	{ },
};
MODULE_DEVICE_TABLE(platform, rpmsg_omx_id_table);

static struct rpmsg_driver rpmsg_omx_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_omx_id_table,
	.probe		= rpmsg_omx_probe,
	.callback	= rpmsg_omx_driver_cb,
	.remove		= __devexit_p(rpmsg_omx_remove),
};

static int __init init(void)
{
	int ret;

	ret = alloc_chrdev_region(&rpmsg_omx_dev, 0, MAX_OMX_DEVICES,
							KBUILD_MODNAME);
	if (ret) {
		pr_err("alloc_chrdev_region failed: %d\n", ret);
		goto out;
	}

	rpmsg_omx_class = class_create(THIS_MODULE, KBUILD_MODNAME);
	if (IS_ERR(rpmsg_omx_class)) {
		ret = PTR_ERR(rpmsg_omx_class);
		pr_err("class_create failed: %d\n", ret);
		goto unreg_region;
	}

	return register_rpmsg_driver(&rpmsg_omx_driver);

unreg_region:
	unregister_chrdev_region(rpmsg_omx_dev, MAX_OMX_DEVICES);
out:
	return ret;
}
module_init(init);

static void __exit fini(void)
{
	struct rpmsg_omx_service *omxserv, *tmp;
	int major = MAJOR(rpmsg_omx_dev);

	unregister_rpmsg_driver(&rpmsg_omx_driver);
	list_for_each_entry_safe(omxserv, tmp, &rpmsg_omx_services_list, next) {
		device_destroy(rpmsg_omx_class, MKDEV(major, omxserv->minor));
		cdev_del(&omxserv->cdev);
		list_del(&omxserv->next);
		kfree(omxserv);
	}
	class_destroy(rpmsg_omx_class);
	unregister_chrdev_region(rpmsg_omx_dev, MAX_OMX_DEVICES);
}
module_exit(fini);

MODULE_DESCRIPTION("OMX offloading rpmsg driver");
MODULE_LICENSE("GPL v2");
