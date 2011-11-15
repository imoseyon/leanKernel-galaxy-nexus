/*
 * Remote processor messaging transport (OMAP platform-specific bits)
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Authors: Ohad Ben-Cohen <ohad@wizery.com>
 *          Brian Swetland <swetland@google.com>
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

#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/interrupt.h>
#include <linux/virtio_ring.h>
#include <linux/rpmsg.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/memblock.h>
#include <linux/remoteproc.h>
#include <asm/io.h>

#include <plat/rpmsg.h>
#include <plat/mailbox.h>
#include <plat/remoteproc.h>

struct omap_rpmsg_vproc {
	struct virtio_device vdev;
	unsigned int vring[2]; /* mpu owns first vring, ipu owns the 2nd */
	unsigned int buf_addr;
	unsigned int buf_size; /* must be page-aligned */
	void *buf_mapped;
	char *mbox_name;
	char *rproc_name;
	struct omap_mbox *mbox;
	struct mutex lock;
	struct rproc *rproc;
	struct notifier_block nb;
	struct notifier_block rproc_nb;
	struct work_struct reset_work;
	bool slave_reset;
	struct omap_rpmsg_vproc *slave_next;
	struct virtqueue *vq[2];
	int base_vq_id;
	int num_of_vqs;
	struct rpmsg_channel_info *hardcoded_chnls;
};

#define to_omap_rpdev(vd) container_of(vd, struct omap_rpmsg_vproc, vdev)
static void rpmsg_reset_work(struct work_struct *work);

struct omap_rpmsg_vq_info {
	__u16 num;	/* number of entries in the virtio_ring */
	__u16 vq_id;	/* a globaly unique index of this virtqueue */
	void *addr;	/* address where we mapped the virtio ring */
	struct omap_rpmsg_vproc *rpdev;
};

/*
 * For now, allocate 256 buffers of 512 bytes for each side. each buffer
 * will then have 16B for the msg header and 496B for the payload.
 * This will require a total space of 256KB for the buffers themselves, and
 * 3 pages for every vring (the size of the vring depends on the number of
 * buffers it supports).
 */
#define RPMSG_NUM_BUFS		(512)
#define RPMSG_BUF_SIZE		(512)
#define RPMSG_BUFS_SPACE	(RPMSG_NUM_BUFS * RPMSG_BUF_SIZE)

/*
 * The alignment between the consumer and producer parts of the vring.
 * Note: this is part of the "wire" protocol. If you change this, you need
 * to update your BIOS image as well
 */
#define RPMSG_VRING_ALIGN	(4096)

/* With 256 buffers, our vring will occupy 3 pages */
#define RPMSG_RING_SIZE	((DIV_ROUND_UP(vring_size(RPMSG_NUM_BUFS / 2, \
				RPMSG_VRING_ALIGN), PAGE_SIZE)) * PAGE_SIZE)

/* The total IPC space needed to communicate with a remote processor */
#define RPMSG_IPC_MEM	(RPMSG_BUFS_SPACE + 2 * RPMSG_RING_SIZE)

/* provide drivers with platform-specific details */
static void omap_rpmsg_get(struct virtio_device *vdev, unsigned int request,
		   void *buf, unsigned len)
{
	struct omap_rpmsg_vproc *rpdev = to_omap_rpdev(vdev);
	void *presult;
	int iresult;

	switch (request) {
	case VPROC_BUF_ADDR:
		/* user data is at stake so bugs here cannot be tolerated */
		BUG_ON(len != sizeof(rpdev->buf_mapped));
		memcpy(buf, &rpdev->buf_mapped, len);
		break;
	case VPROC_SIM_BASE:
		/* user data is at stake so bugs here cannot be tolerated */
		BUG_ON(len != sizeof(presult));
		/*
		 * calculate a simulated base address to make virtio's
		 * virt_to_page() happy.
		 */
		presult = __va(rpdev->buf_addr);
		memcpy(buf, &presult, len);
		break;
	case VPROC_BUF_NUM:
		/* user data is at stake so bugs here cannot be tolerated */
		BUG_ON(len != sizeof(iresult));
		iresult = RPMSG_NUM_BUFS;
		memcpy(buf, &iresult, len);
		break;
	case VPROC_BUF_SZ:
		/* user data is at stake so bugs here cannot be tolerated */
		BUG_ON(len != sizeof(iresult));
		iresult = RPMSG_BUF_SIZE;
		memcpy(buf, &iresult, len);
		break;
	case VPROC_STATIC_CHANNELS:
		/* user data is at stake so bugs here cannot be tolerated */
		BUG_ON(len != sizeof(rpdev->hardcoded_chnls));
		memcpy(buf, &rpdev->hardcoded_chnls, len);
		break;
	default:
		dev_err(&vdev->dev, "invalid request: %d\n", request);
	}
}

/* kick the remote processor, and let it know which virtqueue to poke at */
static void omap_rpmsg_notify(struct virtqueue *vq)
{
	struct omap_rpmsg_vq_info *rpvq = vq->priv;
	int ret;
	int count = 5;

	pr_debug("sending mailbox msg: %d\n", rpvq->vq_id);
	do {
		rproc_last_busy(rpvq->rpdev->rproc);
		mutex_lock(&rpvq->rpdev->lock);
		if (rpvq->rpdev->mbox)
			break;
		mutex_unlock(&rpvq->rpdev->lock);
	} while (--count);
	if (!count) {
		pr_err("mbox handle is NULL\n");
		return;
	}
	/* send the index of the triggered virtqueue as the mailbox payload */
	ret = omap_mbox_msg_send(rpvq->rpdev->mbox, rpvq->vq_id);
	if (ret)
		pr_err("ugh, omap_mbox_msg_send() failed: %d\n", ret);
	mutex_unlock(&rpvq->rpdev->lock);
}

static int omap_rpmsg_mbox_callback(struct notifier_block *this,
					unsigned long index, void *data)
{
	mbox_msg_t msg = (mbox_msg_t) data;
	struct omap_rpmsg_vproc *rpdev;

	rpdev = container_of(this, struct omap_rpmsg_vproc, nb);

	pr_debug("mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_CRASH:
		pr_err("%s has just crashed !\n", rpdev->rproc_name);
		rproc_error_notify(rpdev->rproc);
		break;
	case RP_MBOX_ECHO_REPLY:
		pr_info("received echo reply from %s !\n", rpdev->rproc_name);
		break;
	case RP_MBOX_PENDING_MSG:
		/*
		 * a new inbound message is waiting in our own vring (index 0).
		 * Let's pretend the message explicitly contained the vring
		 * index number and handle it generically
		 */
		msg = rpdev->base_vq_id;
		/* intentional fall-through */
	default:
		/* ignore vq indices which are clearly not for us */
		if (msg < rpdev->base_vq_id)
			break;

		msg -= rpdev->base_vq_id;

		/*
		 * Currently both PENDING_MSG and explicit-virtqueue-index
		 * messaging are supported.
		 * Whatever approach is taken, at this point 'msg' contains
		 * the index of the vring which was just triggered.
		 */
		if (msg < rpdev->num_of_vqs)
			vring_interrupt(msg, rpdev->vq[msg]);
	}

	return NOTIFY_DONE;
}

static void rpmsg_reset_devices(struct omap_rpmsg_vproc *rpdev)
{
	/* wait until previous reset requests have finished */
	flush_work_sync(&rpdev->reset_work);
	schedule_work(&rpdev->reset_work);
}

static int rpmsg_rproc_error(struct omap_rpmsg_vproc *rpdev)
{
	pr_err("Fatal error in %s\n", rpdev->rproc_name);
#ifdef CONFIG_OMAP_RPMSG_RECOVERY
	if (rpdev->slave_reset)
		return NOTIFY_DONE;
	rpmsg_reset_devices(rpdev);
#endif

	return NOTIFY_DONE;
}

static int rpmsg_rproc_suspend(struct omap_rpmsg_vproc *rpdev)
{
	if (virtqueue_more_used(rpdev->vq[0]))
		return NOTIFY_BAD;
	return NOTIFY_DONE;
}

static int rpmsg_rproc_pos_suspend(struct omap_rpmsg_vproc *rpdev)
{
	mutex_lock(&rpdev->lock);
	if (rpdev->mbox) {
		omap_mbox_put(rpdev->mbox, &rpdev->nb);
		rpdev->mbox = NULL;
	}
	mutex_unlock(&rpdev->lock);

	return NOTIFY_DONE;
}

static int rpmsg_rproc_resume(struct omap_rpmsg_vproc *rpdev)
{
	mutex_lock(&rpdev->lock);
	if (!rpdev->mbox)
		rpdev->mbox = omap_mbox_get(rpdev->mbox_name, &rpdev->nb);
	mutex_unlock(&rpdev->lock);

	return NOTIFY_DONE;
}

static int rpmsg_rproc_secure(struct omap_rpmsg_vproc *rpdev, bool s)
{
	pr_err("%s: %s secure mode\n", rpdev->rproc_name, s ? "enter" : "exit");
	if (rpdev->slave_reset)
		return NOTIFY_DONE;
	rpmsg_reset_devices(rpdev);

	return NOTIFY_DONE;
}

static int rpmsg_rproc_events(struct notifier_block *this,
				unsigned long type, void *data)
{
	struct omap_rpmsg_vproc *rpdev = container_of(this,
				struct omap_rpmsg_vproc, rproc_nb);

	switch (type) {
	case RPROC_ERROR:
		return rpmsg_rproc_error(rpdev);
	case RPROC_PRE_SUSPEND:
		return rpmsg_rproc_suspend(rpdev);
	case RPROC_POS_SUSPEND:
		return rpmsg_rproc_pos_suspend(rpdev);
	case RPROC_RESUME:
		return rpmsg_rproc_resume(rpdev);
	case RPROC_SECURE:
		return rpmsg_rproc_secure(rpdev, !!data);
	}
	return NOTIFY_DONE;
}

static struct virtqueue *rp_find_vq(struct virtio_device *vdev,
				    unsigned index,
				    void (*callback)(struct virtqueue *vq),
				    const char *name)
{
	struct omap_rpmsg_vproc *rpdev = to_omap_rpdev(vdev);
	struct omap_rpmsg_vq_info *rpvq;
	struct virtqueue *vq;
	int err;

	rpvq = kmalloc(sizeof(*rpvq), GFP_KERNEL);
	if (!rpvq)
		return ERR_PTR(-ENOMEM);

	/* ioremap'ing normal memory, so we cast away sparse's complaints */
	rpvq->addr = (__force void *) ioremap_nocache(rpdev->vring[index],
							RPMSG_RING_SIZE);
	if (!rpvq->addr) {
		err = -ENOMEM;
		goto free_rpvq;
	}

	memset(rpvq->addr, 0, RPMSG_RING_SIZE);

	pr_debug("vring%d: phys 0x%x, virt 0x%x\n", index, rpdev->vring[index],
					(unsigned int) rpvq->addr);

	vq = vring_new_virtqueue(RPMSG_NUM_BUFS / 2, RPMSG_VRING_ALIGN, vdev,
				rpvq->addr, omap_rpmsg_notify, callback, name);
	if (!vq) {
		pr_err("vring_new_virtqueue failed\n");
		err = -ENOMEM;
		goto unmap_vring;
	}

	rpdev->vq[index] = vq;
	vq->priv = rpvq;
	/* system-wide unique id for this virtqueue */
	rpvq->vq_id = rpdev->base_vq_id + index;
	rpvq->rpdev = rpdev;
	mutex_init(&rpdev->lock);

	return vq;

unmap_vring:
	/* iounmap normal memory, so make sparse happy */
	iounmap((__force void __iomem *) rpvq->addr);
free_rpvq:
	kfree(rpvq);
	return ERR_PTR(err);
}

static void omap_rpmsg_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;
	struct omap_rpmsg_vproc *rpdev = to_omap_rpdev(vdev);

	rproc_event_unregister(rpdev->rproc, &rpdev->rproc_nb);

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		struct omap_rpmsg_vq_info *rpvq = vq->priv;
		vring_del_virtqueue(vq);
		kfree(rpvq);
	}

	if (rpdev->mbox)
		omap_mbox_put(rpdev->mbox, &rpdev->nb);

	if (rpdev->rproc)
		rproc_put(rpdev->rproc);

	iounmap(rpdev->buf_mapped);
}

static int omap_rpmsg_find_vqs(struct virtio_device *vdev, unsigned nvqs,
		       struct virtqueue *vqs[],
		       vq_callback_t *callbacks[],
		       const char *names[])
{
	struct omap_rpmsg_vproc *rpdev = to_omap_rpdev(vdev);
	int i, err;

	/* we maintain two virtqueues per remote processor (for RX and TX) */
	if (nvqs != 2)
		return -EINVAL;

	for (i = 0; i < nvqs; ++i) {
		vqs[i] = rp_find_vq(vdev, i, callbacks[i], names[i]);
		if (IS_ERR(vqs[i])) {
			err = PTR_ERR(vqs[i]);
			goto error;
		}
	}

	rpdev->num_of_vqs = nvqs;

	/* ioremap'ing normal memory, so we cast away sparse's complaints */
	rpdev->buf_mapped = (__force void *) ioremap_nocache(rpdev->buf_addr,
							rpdev->buf_size);
	if (!rpdev->buf_mapped) {
		pr_err("ioremap failed\n");
		err = -ENOMEM;
		goto error;
	}

	/* for now, use mailbox's notifiers. later that can be optimized */
	rpdev->nb.notifier_call = omap_rpmsg_mbox_callback;
	rpdev->mbox = omap_mbox_get(rpdev->mbox_name, &rpdev->nb);
	if (IS_ERR(rpdev->mbox)) {
		pr_err("failed to get mailbox %s\n", rpdev->mbox_name);
		err = -EINVAL;
		goto unmap_buf;
	}

	pr_debug("buf: phys 0x%x, virt 0x%x\n", rpdev->buf_addr,
					(unsigned int) rpdev->buf_mapped);

	/* tell the M3 we're ready. hmm. do we really need this msg */
	err = omap_mbox_msg_send(rpdev->mbox, RP_MBOX_READY);
	if (err) {
		pr_err("ugh, omap_mbox_msg_send() failed: %d\n", err);
		goto put_mbox;
	}

	/* send it the physical address of the mapped buffer + vrings, */
	/* this should be moved to the resource table logic */
	err = omap_mbox_msg_send(rpdev->mbox, (mbox_msg_t) rpdev->buf_addr);
	if (err) {
		pr_err("ugh, omap_mbox_msg_send() failed: %d\n", err);
		goto put_mbox;
	}

	/* ping the remote processor. this is only for sanity-sake;
	 * there is no functional effect whatsoever */
	err = omap_mbox_msg_send(rpdev->mbox, RP_MBOX_ECHO_REQUEST);
	if (err) {
		pr_err("ugh, omap_mbox_msg_send() failed: %d\n", err);
		goto put_mbox;
	}

	/* now load the firmware, and take the M3 out of reset */
	rpdev->rproc = rproc_get(rpdev->rproc_name);
	if (!rpdev->rproc) {
		pr_err("failed to get rproc %s\n", rpdev->rproc_name);
		err = -EINVAL;
		goto put_mbox;
	}
	/* register for remoteproc events */
	rpdev->rproc_nb.notifier_call = rpmsg_rproc_events;
	rproc_event_register(rpdev->rproc, &rpdev->rproc_nb);

	return 0;

put_mbox:
	omap_mbox_put(rpdev->mbox, &rpdev->nb);
unmap_buf:
	/* iounmap normal memory, so make sparse happy */
	iounmap((__force void __iomem *)rpdev->buf_mapped);
error:
	omap_rpmsg_del_vqs(vdev);
	return err;
}

/*
 * should be nice to add firmware support for these handlers.
 * for now provide them so virtio doesn't crash
 */
static u8 omap_rpmsg_get_status(struct virtio_device *vdev)
{
	return 0;
}

static void omap_rpmsg_set_status(struct virtio_device *vdev, u8 status)
{
	dev_dbg(&vdev->dev, "new status: %d\n", status);
}

static void omap_rpmsg_reset(struct virtio_device *vdev)
{
	dev_dbg(&vdev->dev, "reset !\n");
}

static u32 omap_rpmsg_get_features(struct virtio_device *vdev)
{
	/* for now, use hardcoded bitmap. later this should be provided
	 * by the firmware itself */
	return (1 << VIRTIO_RPMSG_F_NS);
}

static void omap_rpmsg_finalize_features(struct virtio_device *vdev)
{
	/* Give virtio_ring a chance to accept features */
	vring_transport_features(vdev);
}

static void omap_rpmsg_vproc_release(struct device *dev)
{
	/* this handler is provided so driver core doesn't yell at us */
}

static void rpmsg_reset_work(struct work_struct *work)
{
	struct omap_rpmsg_vproc *rpdev =
		container_of(work, struct omap_rpmsg_vproc, reset_work);
	struct omap_rpmsg_vproc *tmp;
	int ret;

	for (tmp = rpdev; tmp; tmp = tmp->slave_next) {
		pr_err("reseting virtio device %d\n", tmp->vdev.index);
		unregister_virtio_device(&tmp->vdev);
	}
	for (tmp = rpdev; tmp; tmp = tmp->slave_next) {
		memset(&tmp->vdev.dev, 0, sizeof(struct device));
		tmp->vdev.dev.release = omap_rpmsg_vproc_release;
		ret = register_virtio_device(&tmp->vdev);
		if (ret)
			pr_err("error creating virtio device %d\n", ret);
	}
}

static struct virtio_config_ops omap_rpmsg_config_ops = {
	.get_features	= omap_rpmsg_get_features,
	.finalize_features = omap_rpmsg_finalize_features,
	.get		= omap_rpmsg_get,
	.find_vqs	= omap_rpmsg_find_vqs,
	.del_vqs	= omap_rpmsg_del_vqs,
	.reset		= omap_rpmsg_reset,
	.set_status	= omap_rpmsg_set_status,
	.get_status	= omap_rpmsg_get_status,
};

static struct rpmsg_channel_info omap_ipuc0_hardcoded_chnls[] = {
	{ "rpmsg-resmgr", 100, RPMSG_ADDR_ANY },
	{ "rpmsg-server-sample", 137, RPMSG_ADDR_ANY },
	{ },
};

static struct rpmsg_channel_info omap_ipuc1_hardcoded_chnls[] = {
	{ "rpmsg-resmgr", 100, RPMSG_ADDR_ANY },
	{ },
};

static struct omap_rpmsg_vproc omap_rpmsg_vprocs[] = {
	/* ipu_c0's rpmsg backend */
	{
		.vdev.id.device	= VIRTIO_ID_RPMSG,
		.vdev.config	= &omap_rpmsg_config_ops,
		.mbox_name	= "mailbox-1",
		.rproc_name	= "ipu",
		.base_vq_id	= 0,
		.hardcoded_chnls = omap_ipuc0_hardcoded_chnls,
		.slave_next	= &omap_rpmsg_vprocs[1],
	},
	/* ipu_c1's rpmsg backend */
	{
		.vdev.id.device	= VIRTIO_ID_RPMSG,
		.vdev.config	= &omap_rpmsg_config_ops,
		.mbox_name	= "mailbox-1",
		.rproc_name	= "ipu",
		.base_vq_id	= 2,
		.hardcoded_chnls = omap_ipuc1_hardcoded_chnls,
		.slave_reset	= true,
	},
};

static int __init omap_rpmsg_ini(void)
{
	int i, ret = 0;
	phys_addr_t paddr = omap_ipu_get_mempool_base(
						OMAP_RPROC_MEMPOOL_STATIC);
	phys_addr_t psize = omap_ipu_get_mempool_size(
						OMAP_RPROC_MEMPOOL_STATIC);

	for (i = 0; i < ARRAY_SIZE(omap_rpmsg_vprocs); i++) {
		struct omap_rpmsg_vproc *rpdev = &omap_rpmsg_vprocs[i];

		if (psize < RPMSG_IPC_MEM) {
			pr_err("out of carveout memory: %d (%d)\n", psize, i);
			return -ENOMEM;
		}

		/*
		 * vring buffers are expected to be present at the beginning
		 * of the chosen remoteproc pool
		 */
		rpdev->buf_addr = paddr;
		rpdev->buf_size = RPMSG_BUFS_SPACE;
		rpdev->vring[0] = paddr + RPMSG_BUFS_SPACE;
		rpdev->vring[1] = paddr + RPMSG_BUFS_SPACE + RPMSG_RING_SIZE;
		INIT_WORK(&rpdev->reset_work, rpmsg_reset_work);

		paddr += RPMSG_IPC_MEM;
		psize -= RPMSG_IPC_MEM;

		pr_debug("rpdev%d: buf 0x%x, vring0 0x%x, vring1 0x%x\n", i,
			rpdev->buf_addr, rpdev->vring[0], rpdev->vring[1]);

		rpdev->vdev.dev.release = omap_rpmsg_vproc_release;

		ret = register_virtio_device(&rpdev->vdev);
		if (ret) {
			pr_err("failed to register rpdev: %d\n", ret);
			break;
		}
	}

	return ret;
}
module_init(omap_rpmsg_ini);

static void __exit omap_rpmsg_fini(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(omap_rpmsg_vprocs); i++) {
		struct omap_rpmsg_vproc *rpdev = &omap_rpmsg_vprocs[i];

		unregister_virtio_device(&rpdev->vdev);
	}
}
module_exit(omap_rpmsg_fini);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote processor messaging virtio device");
