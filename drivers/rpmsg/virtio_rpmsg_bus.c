/*
 * Virtio-based remote processor messaging bus
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
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/rpmsg.h>

/**
 * struct virtproc_info - virtual remote processor info
 *
 * @vdev:	the virtio device
 * @rvq:	rx virtqueue (from pov of local processor)
 * @svq:	tx virtqueue (from pov of local processor)
 * @rbufs:	address of rx buffers
 * @sbufs:	address of tx buffers
 * @last_rbuf:	index of last rx buffer used
 * @last_sbuf:	index of last tx buffer used
 * @sim_base:	simulated base addr base to make virtio's virt_to_page happy
 * @svq_lock:	protects the tx virtqueue, to allow several concurrent senders
 * @num_bufs:	total number of buffers allocated for communicating with this
 *		virtual remote processor. half is used for rx and half for tx.
 * @buf_size:	size of buffers allocated for communications
 * @endpoints:	the set of local endpoints
 * @endpoints_lock: lock of the endpoints set
 * @sendq:	wait queue of sending contexts waiting for free rpmsg buffer
 * @ns_ept:	the bus's name service endpoint
 *
 * This structure stores the rpmsg state of a given virtio remote processor
 * device (there might be several virtio rproc devices for each physical
 * remote processor).
 */
struct virtproc_info {
	struct virtio_device *vdev;
	struct virtqueue *rvq, *svq;
	void *rbufs, *sbufs;
	int last_rbuf, last_sbuf;
	void *sim_base;
	struct mutex svq_lock;
	int num_bufs;
	int buf_size;
	struct idr endpoints;
	spinlock_t endpoints_lock;
	wait_queue_head_t sendq;
	struct rpmsg_endpoint *ns_ept;
};

#define to_rpmsg_channel(d) container_of(d, struct rpmsg_channel, dev)
#define to_rpmsg_driver(d) container_of(d, struct rpmsg_driver, drv)

/*
 * Local addresses are dynamically allocated on-demand.
 * We do not dynamically assign addresses from the low 1024 range,
 * in order to reserve that address range for predefined services.
 */
#define RPMSG_RESERVED_ADDRESSES	(1024)

/* Address 53 is reserved for advertising remote services */
#define RPMSG_NS_ADDR			(53)

/* show configuration fields */
#define rpmsg_show_attr(field, path, format_string)			\
static ssize_t								\
field##_show(struct device *dev,					\
			struct device_attribute *attr, char *buf)	\
{									\
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);		\
									\
	return sprintf(buf, format_string, rpdev->path);		\
}

rpmsg_show_attr(name, id.name, "%s\n");
rpmsg_show_attr(dst, dst, "0x%x\n");
rpmsg_show_attr(src, src, "0x%x\n");
rpmsg_show_attr(announce, announce ? "true" : "false", "%s\n");

/* unique (free running) numbering for rpmsg devices */
static unsigned int rpmsg_dev_index;

static ssize_t modalias_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);

	return sprintf(buf, RPMSG_DEVICE_MODALIAS_FMT "\n", rpdev->id.name);
}

static struct device_attribute rpmsg_dev_attrs[] = {
	__ATTR_RO(name),
	__ATTR_RO(modalias),
	__ATTR_RO(dst),
	__ATTR_RO(src),
	__ATTR_RO(announce),
	__ATTR_NULL
};

static inline int rpmsg_id_match(const struct rpmsg_channel *rpdev,
				  const struct rpmsg_device_id *id)
{
	if (strncmp(id->name, rpdev->id.name, RPMSG_NAME_SIZE))
		return 0;

	return 1;
}

/* match rpmsg channel and rpmsg driver */
static int rpmsg_dev_match(struct device *dev, struct device_driver *drv)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);
	struct rpmsg_driver *rpdrv = to_rpmsg_driver(drv);
	const struct rpmsg_device_id *ids = rpdrv->id_table;
	unsigned int i;

	for (i = 0; ids[i].name[0]; i++) {
		if (rpmsg_id_match(rpdev, &ids[i]))
			return 1;
	}

	return 0;
}

static int rpmsg_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);

	return add_uevent_var(env, "MODALIAS=" RPMSG_DEVICE_MODALIAS_FMT,
					rpdev->id.name);
}

static struct rpmsg_endpoint *__rpmsg_create_ept(struct virtproc_info *vrp,
		struct rpmsg_channel *rpdev,
		void (*cb)(struct rpmsg_channel *, void *, int, void *, u32),
		void *priv, u32 addr)
{
	int err, tmpaddr, request;
	struct rpmsg_endpoint *ept;
	struct device *dev = rpdev ? &rpdev->dev : &vrp->vdev->dev;

	if (!idr_pre_get(&vrp->endpoints, GFP_KERNEL))
		return NULL;

	ept = kzalloc(sizeof(*ept), GFP_KERNEL);
	if (!ept) {
		dev_err(dev, "failed to kzalloc a new ept\n");
		return NULL;
	}

	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;

	/* do we need to allocate a local address ? */
	request = addr == RPMSG_ADDR_ANY ? RPMSG_RESERVED_ADDRESSES : addr;

	spin_lock(&vrp->endpoints_lock);

	/* bind the endpoint to an rpmsg address (and allocate one if needed) */
	err = idr_get_new_above(&vrp->endpoints, ept, request, &tmpaddr);
	if (err) {
		dev_err(dev, "idr_get_new_above failed: %d\n", err);
		goto free_ept;
	}

	if (addr != RPMSG_ADDR_ANY && tmpaddr != addr) {
		dev_err(dev, "address 0x%x already in use\n", addr);
		goto rem_idr;
	}

	ept->addr = tmpaddr;

	spin_unlock(&vrp->endpoints_lock);

	return ept;

rem_idr:
	idr_remove(&vrp->endpoints, request);
free_ept:
	spin_unlock(&vrp->endpoints_lock);
	kfree(ept);
	return NULL;
}

struct rpmsg_endpoint *rpmsg_create_ept(struct rpmsg_channel *rpdev,
		void (*cb)(struct rpmsg_channel *, void *, int, void *, u32),
		void *priv, u32 addr)
{
	return __rpmsg_create_ept(rpdev->vrp, rpdev, cb, priv, addr);
}
EXPORT_SYMBOL(rpmsg_create_ept);

void rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct virtproc_info *vrp = ept->rpdev->vrp;

	spin_lock(&vrp->endpoints_lock);
	idr_remove(&vrp->endpoints, ept->addr);
	spin_unlock(&vrp->endpoints_lock);

	kfree(ept);
}
EXPORT_SYMBOL(rpmsg_destroy_ept);

static int rpmsg_dev_probe(struct device *dev)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);
	struct rpmsg_driver *rpdrv = to_rpmsg_driver(rpdev->dev.driver);
	struct virtproc_info *vrp = rpdev->vrp;
	struct rpmsg_endpoint *ept;
	int err;

	ept = rpmsg_create_ept(rpdev, rpdrv->callback, NULL, rpdev->src);
	if (!ept) {
		dev_err(dev, "failed to create endpoint\n");
		err = -ENOMEM;
		goto out;
	}

	rpdev->ept = ept;
	rpdev->src = ept->addr;

	err = rpdrv->probe(rpdev);
	if (err) {
		dev_err(dev, "%s: failed: %d\n", __func__, err);
		rpmsg_destroy_ept(ept);
		goto out;
	}

	/* need to tell remote processor's name service about this channel ? */
	if (rpdev->announce &&
			virtio_has_feature(vrp->vdev, VIRTIO_RPMSG_F_NS)) {
		struct rpmsg_ns_msg nsm;

		strncpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
		nsm.addr = rpdev->src;
		nsm.flags = RPMSG_NS_CREATE;

		err = rpmsg_sendto(rpdev, &nsm, sizeof(nsm), RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

out:
	return err;
}

static int rpmsg_dev_remove(struct device *dev)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);
	struct rpmsg_driver *rpdrv = to_rpmsg_driver(rpdev->dev.driver);
	struct virtproc_info *vrp = rpdev->vrp;
	int err = 0;

	/* tell remote processor's name service we're removing this channel */
	if (rpdev->announce &&
			virtio_has_feature(vrp->vdev, VIRTIO_RPMSG_F_NS)) {
		struct rpmsg_ns_msg nsm;

		strncpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
		nsm.addr = rpdev->src;
		nsm.flags = RPMSG_NS_DESTROY;

		err = rpmsg_sendto(rpdev, &nsm, sizeof(nsm), RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

	rpdrv->remove(rpdev);

	rpmsg_destroy_ept(rpdev->ept);

	return err;
}

static struct bus_type rpmsg_bus = {
	.name		= "rpmsg",
	.match		= rpmsg_dev_match,
	.dev_attrs	= rpmsg_dev_attrs,
	.uevent		= rpmsg_uevent,
	.probe		= rpmsg_dev_probe,
	.remove		= rpmsg_dev_remove,
};

int register_rpmsg_driver(struct rpmsg_driver *rpdrv)
{
	rpdrv->drv.bus = &rpmsg_bus;
	return driver_register(&rpdrv->drv);
}
EXPORT_SYMBOL(register_rpmsg_driver);

void unregister_rpmsg_driver(struct rpmsg_driver *rpdrv)
{
	driver_unregister(&rpdrv->drv);
}
EXPORT_SYMBOL(unregister_rpmsg_driver);

static void rpmsg_release_device(struct device *dev)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);

	kfree(rpdev);
}

/* match an rpmsg channel with channel info structs */
static int rpmsg_channel_match(struct device *dev, void *data)
{
	struct rpmsg_channel_info *chinfo = data;
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);

	if (chinfo->src != RPMSG_ADDR_ANY && chinfo->src != rpdev->src)
		return 0;

	if (chinfo->dst != RPMSG_ADDR_ANY && chinfo->dst != rpdev->dst)
		return 0;

	if (strncmp(chinfo->name, rpdev->id.name, RPMSG_NAME_SIZE))
		return 0;

	return 1;
}

static struct rpmsg_channel *rpmsg_create_channel(struct virtproc_info *vrp,
				struct rpmsg_channel_info *chinfo)
{
	struct rpmsg_channel *rpdev;
	struct device *tmp, *dev = &vrp->vdev->dev;
	int ret;

	/* make sure a similar channel doesn't already exist */
	tmp = device_find_child(dev, chinfo, rpmsg_channel_match);
	if (tmp) {
		dev_err(dev, "channel %s:%x:%x already exist\n",
				chinfo->name, chinfo->src, chinfo->dst);
		return NULL;
	}

	rpdev = kzalloc(sizeof(struct rpmsg_channel), GFP_KERNEL);
	if (!rpdev) {
		pr_err("kzalloc failed\n");
		return NULL;
	}

	rpdev->vrp = vrp;
	rpdev->src = chinfo->src;
	rpdev->dst = chinfo->dst;

	/*
	 * rpmsg server channels has predefined local address, and their
	 * existence needs to be announced remotely
	 */
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY ? true : false;

	strncpy(rpdev->id.name, chinfo->name, RPMSG_NAME_SIZE);

	/* very simple device indexing plumbing which just works (for now) */
	dev_set_name(&rpdev->dev, "rpmsg%d", rpmsg_dev_index++);

	rpdev->dev.parent = &vrp->vdev->dev;
	rpdev->dev.bus = &rpmsg_bus;
	rpdev->dev.release = rpmsg_release_device;

	ret = device_register(&rpdev->dev);
	if (ret) {
		dev_err(dev, "device_register failed: %d\n", ret);
		kfree(rpdev);
		return NULL;
	}

	return rpdev;
}

static void rpmsg_destroy_channel(struct rpmsg_channel *rpdev)
{
	device_unregister(&rpdev->dev);
}

static int rpmsg_destroy_channel_by_info(struct virtproc_info *vrp,
					struct rpmsg_channel_info *chinfo)
{
	struct virtio_device *vdev = vrp->vdev;
	struct device *dev;

	dev = device_find_child(&vdev->dev, chinfo, rpmsg_channel_match);
	if (!dev)
		return -EINVAL;

	rpmsg_destroy_channel(to_rpmsg_channel(dev));

	return 0;
}

/* minimal buf "allocator" that is just enough for now */
static void *get_a_buf(struct virtproc_info *vrp)
{
	unsigned int len;
	void *buf = NULL;

	/* make sure the descriptors are updated before reading */
	rmb();
	/* either pick the next unused buffer */
	if (vrp->last_sbuf < vrp->num_bufs / 2)
		buf = vrp->sbufs + vrp->buf_size * vrp->last_sbuf++;
	/* or recycle a used one */
	else
		buf = virtqueue_get_buf(vrp->svq, &len);

	return buf;
}

/* XXX: the blocking 'wait' mechanism hasn't been tested yet */
int rpmsg_send_offchannel_raw(struct rpmsg_channel *rpdev, u32 src, u32 dst,
					void *data, int len, bool wait)
{
	struct virtproc_info *vrp = rpdev->vrp;
	struct device *dev = &rpdev->dev;
	struct scatterlist sg;
	struct rpmsg_hdr *msg;
	int err;
	unsigned long offset;
	void *sim_addr;

	if (src == RPMSG_ADDR_ANY || dst == RPMSG_ADDR_ANY) {
		dev_err(dev, "invalid addr (src 0x%x, dst 0x%x)\n", src, dst);
		return -EINVAL;
	}

	/* the payload's size is currently limited */
	if (len > vrp->buf_size - sizeof(struct rpmsg_hdr)) {
		dev_err(dev, "message is too big (%d)\n", len);
		return -EMSGSIZE;
	}

	/*
	 * protect svq from simultaneous concurrent manipulations,
	 * and serialize the sending of messages
	 */
	if (mutex_lock_interruptible(&vrp->svq_lock))
		return -ERESTARTSYS;
	/* grab a buffer */
	msg = get_a_buf(vrp);
	if (!msg && !wait) {
		err = -ENOMEM;
		goto out;
	}

	/* no free buffer ? wait for one (but bail after 15 seconds) */
	if (!msg) {
		/* enable "tx-complete" interrupts before dozing off */
		virtqueue_enable_cb(vrp->svq);

		/*
		 * sleep until a free buffer is available or 15 secs elapse.
		 * the timeout period is not configurable because frankly
		 * i don't see why drivers need to deal with that.
		 * if later this happens to be required, it'd be easy to add.
		 */
		err = wait_event_interruptible_timeout(vrp->sendq,
					(msg = get_a_buf(vrp)),
					msecs_to_jiffies(15000));

		/* on success, suppress "tx-complete" interrupts again */
		virtqueue_disable_cb(vrp->svq);

		if (err < 0) {
			err = -ERESTARTSYS;
			goto out;
		}

		if (!msg) {
			dev_err(dev, "timeout waiting for buffer\n");
			err = -ETIMEDOUT;
			goto out;
		}
	}

	msg->len = len;
	msg->flags = 0;
	msg->src = src;
	msg->dst = dst;
	msg->unused = 0;
	memcpy(msg->data, data, len);

	dev_dbg(dev, "TX From 0x%x, To 0x%x, Len %d, Flags %d, Unused %d\n",
					msg->src, msg->dst, msg->len,
					msg->flags, msg->unused);
#if 0
	print_hex_dump(KERN_DEBUG, "rpmsg_virtio TX: ", DUMP_PREFIX_NONE, 16, 1,
					msg, sizeof(*msg) + msg->len, true);
#endif

	offset = ((unsigned long) msg) - ((unsigned long) vrp->rbufs);
	sim_addr = vrp->sim_base + offset;
	sg_init_one(&sg, sim_addr, sizeof(*msg) + len);

	/* add message to the remote processor's virtqueue */
	err = virtqueue_add_buf_gfp(vrp->svq, &sg, 1, 0, msg, GFP_KERNEL);
	if (err < 0) {
		dev_err(dev, "virtqueue_add_buf_gfp failed: %d\n", err);
		goto out;
	}
	/* descriptors must be written before kicking remote processor */
	wmb();

	/* tell the remote processor it has a pending message to read */
	virtqueue_kick(vrp->svq);

	err = 0;
out:
	mutex_unlock(&vrp->svq_lock);
	return err;
}
EXPORT_SYMBOL(rpmsg_send_offchannel_raw);

static void rpmsg_recv_done(struct virtqueue *rvq)
{
	struct rpmsg_hdr *msg;
	unsigned int len;
	struct rpmsg_endpoint *ept;
	struct scatterlist sg;
	unsigned long offset;
	void *sim_addr;
	struct virtproc_info *vrp = rvq->vdev->priv;
	struct device *dev = &rvq->vdev->dev;
	int err;

	/* make sure the descriptors are updated before reading */
	rmb();
	msg = virtqueue_get_buf(rvq, &len);
	if (!msg) {
		dev_err(dev, "uhm, incoming signal, but no used buffer ?\n");
		return;
	}

	dev_dbg(dev, "From: 0x%x, To: 0x%x, Len: %d, Flags: %d, Unused: %d\n",
					msg->src, msg->dst, msg->len,
					msg->flags, msg->unused);
#if 0
	print_hex_dump(KERN_DEBUG, "rpmsg_virtio RX: ", DUMP_PREFIX_NONE, 16, 1,
					msg, sizeof(*msg) + msg->len, true);
#endif

	/* fetch the callback of the appropriate user */
	spin_lock(&vrp->endpoints_lock);
	ept = idr_find(&vrp->endpoints, msg->dst);
	spin_unlock(&vrp->endpoints_lock);

	if (ept && ept->cb)
		ept->cb(ept->rpdev, msg->data, msg->len, ept->priv, msg->src);
	else
		dev_warn(dev, "msg received with no recepient\n");

	/* add the buffer back to the remote processor's virtqueue */
	offset = ((unsigned long) msg) - ((unsigned long) vrp->rbufs);
	sim_addr = vrp->sim_base + offset;
	sg_init_one(&sg, sim_addr, sizeof(*msg) + len);

	err = virtqueue_add_buf_gfp(vrp->rvq, &sg, 0, 1, msg, GFP_KERNEL);
	if (err < 0) {
		dev_err(dev, "failed to add a virtqueue buffer: %d\n", err);
		return;
	}
	/* descriptors must be written before kicking remote processor */
	wmb();

	/* tell the remote processor we added another available rx buffer */
	virtqueue_kick(vrp->rvq);
}

static void rpmsg_xmit_done(struct virtqueue *svq)
{
	struct virtproc_info *vrp = svq->vdev->priv;

	dev_dbg(&svq->vdev->dev, "%s\n", __func__);

	/* wake up potential processes that are waiting for a buffer */
	wake_up_interruptible(&vrp->sendq);
}

static void rpmsg_ns_cb(struct rpmsg_channel *rpdev, void *data, int len,
							void *priv, u32 src)
{
	struct rpmsg_ns_msg *msg = data;
	struct rpmsg_channel *newch;
	struct rpmsg_channel_info chinfo;
	struct virtproc_info *vrp = priv;
	struct device *dev = &vrp->vdev->dev;
	int ret;

#if 0
	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len, true);
#endif

	if (len != sizeof(*msg)) {
		dev_err(dev, "malformed ns msg (%d)\n", len);
		return;
	}

	/*
	 * the name service ept does _not_ belong to a real rpmsg channel,
	 * and is handled by the rpmsg bus itself.
	 * for sanity reasons, make sure a valid rpdev has _not_ sneaked
	 * in somehow.
	 */
	if (rpdev) {
		dev_err(dev, "anomaly: ns ept has an rpdev handle\n");
		return;
	}

	/* don't trust the remote processor for null terminating the name */
	msg->name[RPMSG_NAME_SIZE - 1] = '\0';

	dev_info(dev, "%sing channel %s addr 0x%x\n",
			msg->flags & RPMSG_NS_DESTROY ? "destroy" : "creat",
			msg->name, msg->addr);

	strncpy(chinfo.name, msg->name, sizeof(chinfo.name));
	chinfo.src = RPMSG_ADDR_ANY;
	chinfo.dst = msg->addr;

	if (msg->flags & RPMSG_NS_DESTROY) {
		ret = rpmsg_destroy_channel_by_info(vrp, &chinfo);
		if (ret)
			dev_err(dev, "rpmsg_destroy_channel failed: %d\n", ret);
	} else {
		newch = rpmsg_create_channel(vrp, &chinfo);
		if (!newch)
			dev_err(dev, "rpmsg_create_channel failed\n");
	}
}

static int rpmsg_probe(struct virtio_device *vdev)
{
	vq_callback_t *vq_cbs[] = { rpmsg_recv_done, rpmsg_xmit_done };
	const char *names[] = { "input", "output" };
	struct virtqueue *vqs[2];
	struct virtproc_info *vrp;
	void *addr;
	int err, i, num_bufs, buf_size, total_buf_size;
	struct rpmsg_channel_info *ch;

	vrp = kzalloc(sizeof(*vrp), GFP_KERNEL);
	if (!vrp)
		return -ENOMEM;

	vrp->vdev = vdev;

	idr_init(&vrp->endpoints);
	spin_lock_init(&vrp->endpoints_lock);
	mutex_init(&vrp->svq_lock);
	init_waitqueue_head(&vrp->sendq);

	/* We expect two virtqueues, rx and tx (in this order) */
	err = vdev->config->find_vqs(vdev, 2, vqs, vq_cbs, names);
	if (err)
		goto free_vi;

	vrp->rvq = vqs[0];
	vrp->svq = vqs[1];

	/* Platform must supply pre-allocated uncached buffers for now */
	vdev->config->get(vdev, VPROC_BUF_ADDR, &addr, sizeof(addr));
	vdev->config->get(vdev, VPROC_BUF_NUM, &num_bufs,
							sizeof(num_bufs));
	vdev->config->get(vdev, VPROC_BUF_SZ, &buf_size, sizeof(buf_size));

	total_buf_size = num_bufs * buf_size;

	dev_dbg(&vdev->dev, "%d buffers, size %d, addr 0x%x, total 0x%x\n",
		num_bufs, buf_size, (unsigned int) addr, total_buf_size);

	vrp->num_bufs = num_bufs;
	vrp->buf_size = buf_size;
	vrp->rbufs = addr;
	vrp->sbufs = addr + total_buf_size / 2;

	/* simulated addr base to make virt_to_page happy */
	vdev->config->get(vdev, VPROC_SIM_BASE, &vrp->sim_base,
							sizeof(vrp->sim_base));

	/* set up the receive buffers */
	for (i = 0; i < num_bufs / 2; i++) {
		struct scatterlist sg;
		void *tmpaddr = vrp->rbufs + i * buf_size;
		void *simaddr = vrp->sim_base + i * buf_size;

		sg_init_one(&sg, simaddr, buf_size);
		err = virtqueue_add_buf_gfp(vrp->rvq, &sg, 0, 1, tmpaddr,
								GFP_KERNEL);
		WARN_ON(err < 0); /* sanity check; this can't really happen */
	}

	/* tell the remote processor it can start sending data */
	virtqueue_kick(vrp->rvq);

	/* suppress "tx-complete" interrupts */
	virtqueue_disable_cb(vrp->svq);

	vdev->priv = vrp;

	dev_info(&vdev->dev, "rpmsg backend virtproc probed successfully\n");

	/* if supported by the remote processor, enable the name service */
	if (virtio_has_feature(vdev, VIRTIO_RPMSG_F_NS)) {
		vrp->ns_ept = __rpmsg_create_ept(vrp, NULL, rpmsg_ns_cb,
						vrp, RPMSG_NS_ADDR);
		if (!vrp->ns_ept) {
			dev_err(&vdev->dev, "failed to create the ns ept\n");
			err = -ENOMEM;
			goto vqs_del;
		}
	}

	/* look for platform-specific static channels */
	vdev->config->get(vdev, VPROC_STATIC_CHANNELS, &ch, sizeof(ch));

	for (i = 0; ch && ch[i].name[0]; i++)
		rpmsg_create_channel(vrp, &ch[i]);

	return 0;

vqs_del:
	vdev->config->del_vqs(vrp->vdev);
free_vi:
	kfree(vrp);
	return err;
}

static int rpmsg_remove_device(struct device *dev, void *data)
{
	struct rpmsg_channel *rpdev = to_rpmsg_channel(dev);

	rpmsg_destroy_channel(rpdev);

	return 0;
}

static void __devexit rpmsg_remove(struct virtio_device *vdev)
{
	struct virtproc_info *vrp = vdev->priv;
	int ret;

	ret = device_for_each_child(&vdev->dev, NULL, rpmsg_remove_device);
	if (ret)
		dev_warn(&vdev->dev, "can't remove rpmsg device: %d\n", ret);

	idr_remove_all(&vrp->endpoints);
	idr_destroy(&vrp->endpoints);

	vdev->config->del_vqs(vrp->vdev);

	kfree(vrp);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_RPMSG, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {
	VIRTIO_RPMSG_F_NS,
};

static struct virtio_driver virtio_ipc_driver = {
	.feature_table	= features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name	= KBUILD_MODNAME,
	.driver.owner	= THIS_MODULE,
	.id_table	= id_table,
	.probe		= rpmsg_probe,
	.remove		= __devexit_p(rpmsg_remove),
};

static int __init init(void)
{
	int ret;

	ret = bus_register(&rpmsg_bus);
	if (ret) {
		pr_err("failed to register rpmsg bus: %d\n", ret);
		return ret;
	}

	return register_virtio_driver(&virtio_ipc_driver);
}
module_init(init);

static void __exit fini(void)
{
	unregister_virtio_driver(&virtio_ipc_driver);
	bus_unregister(&rpmsg_bus);
}
module_exit(fini);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio-based remote processor messaging bus");
MODULE_LICENSE("GPL v2");
