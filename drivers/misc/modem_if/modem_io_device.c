/* /linux/drivers/misc/modem_if/modem_io_device.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"


#define HDLC_START	0x7F
#define HDLC_END	0x7E
#define SIZE_OF_HDLC_START	1
#define SIZE_OF_HDLC_END	1
#define MAX_RXDATA_SIZE	0x1000	/* 4*1024 */

static const char hdlc_start[1] = { HDLC_START };
static const char hdlc_end[1] = { HDLC_END };

struct fmt_hdr {
	u16 len;
	u8 control;
} __attribute__ ((packed));

struct raw_hdr {
	u32 len;
	u8 channel;
	u8 control;
} __attribute__ ((packed));

struct rfs_hdr {
	u32 len;
	u8 cmd;
	u8 id;
} __attribute__ ((packed));

static int rx_iodev_skb(struct io_device *iod);

static int get_header_size(struct io_device *iod)
{
	switch (iod->format) {
	case IPC_FMT:
		return sizeof(struct fmt_hdr);

	case IPC_RAW:
	case IPC_MULTI_RAW:
		return sizeof(struct raw_hdr);

	case IPC_RFS:
		return sizeof(struct rfs_hdr);

	case IPC_BOOT:
		/* minimum size for transaction align */
		return 4;

	default:
		return 0;
	}
}

static int get_hdlc_size(struct io_device *iod, char *buf)
{
	struct fmt_hdr *fmt_header;
	struct raw_hdr *raw_header;
	struct rfs_hdr *rfs_header;

	pr_debug("[MODEM_IF] buf : %02x %02x %02x (%d)\n", *buf, *(buf + 1),
				*(buf + 2), __LINE__);

	switch (iod->format) {
	case IPC_FMT:
		fmt_header = (struct fmt_hdr *)buf;
		return fmt_header->len;
	case IPC_RAW:
	case IPC_MULTI_RAW:
		raw_header = (struct raw_hdr *)buf;
		return raw_header->len;
	case IPC_RFS:
		rfs_header = (struct rfs_hdr *)buf;
		return rfs_header->len;
	default:
		break;
	}
	return 0;
}

static void *get_header(struct io_device *iod, struct misc_data_io *misc_data,
			char *frame_header_buf)
{
	struct fmt_hdr *fmt_h;
	struct raw_hdr *raw_h;
	struct rfs_hdr *rfs_h;

	switch (iod->format) {
	case IPC_FMT:
		fmt_h = (struct fmt_hdr *)frame_header_buf;

		fmt_h->len = misc_data->size + sizeof(struct fmt_hdr);
		fmt_h->control = 0;

		return (void *)frame_header_buf;

	case IPC_RAW:
	case IPC_MULTI_RAW:
		raw_h = (struct raw_hdr *)frame_header_buf;

		raw_h->len = misc_data->size + sizeof(struct raw_hdr);
		raw_h->channel = misc_data->id;
		raw_h->control = 0;

		return (void *)frame_header_buf;

	case IPC_RFS:
		rfs_h = (struct rfs_hdr *)frame_header_buf;

		rfs_h->len = misc_data->size + sizeof(struct raw_hdr);
		rfs_h->cmd = misc_data->cmd;
		rfs_h->id = misc_data->id;

		return (void *)frame_header_buf;

	default:
		return 0;
	}
}

static int rx_hdlc_head_start_check(char *buf)
{
	if (strncmp(buf, hdlc_start, sizeof(hdlc_start))) {
		pr_err("[MODEM_IF] Wrong HDLC start: 0x%x\n", *buf);
		return -EBADMSG;
	}
	return sizeof(hdlc_start);
}

static int rx_hdlc_tail_check(char *buf)
{
	if (strncmp(buf, hdlc_end, sizeof(hdlc_end))) {
		pr_err("[MODEM_IF] Wrong HDLC end: 0x%x\n", *buf);
		return -EBADMSG;
	}
	return sizeof(hdlc_end);
}

/* remove hdlc header and store IPC header */
static int rx_hdlc_head_check(struct io_device *iod, char *buf, unsigned rest)
{
	struct header_data *hdr = &iod->h_data;
	int head_size = get_header_size(iod);
	int done_len = 0;
	int len = 0;

	/* first frame, remove start header 7F */
	if (!hdr->start) {
		len = rx_hdlc_head_start_check(buf);
		if (len < 0)
			return len; /*Wrong hdlc start*/

		pr_debug("[MODEM_IF] check len : %d, rest : %d (%d)\n", len,
					rest, __LINE__);

		memcpy(&hdr->start, hdlc_start, len);
		hdr->len = 0;

		/* debug print */
		switch (iod->format) {
		case IPC_FMT:
		case IPC_RAW:
		case IPC_MULTI_RAW:
		case IPC_RFS:
			/* TODO: print buf...  */
			break;

		case IPC_CMD:
		case IPC_BOOT:
		default:
			break;
		}
		buf += len;
		done_len += len;
		rest -= len; /* rest, call by value */
	}

	pr_debug("[MODEM_IF] check len : %d, rest : %d (%d)\n", len, rest,
				__LINE__);

	/* store the IPC header to iod priv */
	if (hdr->len < head_size) {
		len = min(rest, head_size - hdr->len);
		memcpy(hdr->hdr + hdr->len, buf, len);
		hdr->len += len;
		done_len += len;
	}

	pr_debug("[MODEM_IF] check done_len : %d, rest : %d (%d)\n", done_len,
				rest, __LINE__);
	return done_len;
}

/* alloc skb and copy dat to skb */
static int rx_hdlc_data_check(struct io_device *iod, char *buf, unsigned rest)
{
	struct header_data *hdr = &iod->h_data;
	struct sk_buff *skb = iod->skb_recv;
	int head_size = get_header_size(iod);
	int data_size = get_hdlc_size(iod, hdr->hdr) - head_size;
	int alloc_size = min(data_size, MAX_RXDATA_SIZE);
	int len;
	int done_len = 0;
	int rest_len = data_size - hdr->flag_len;
	struct sk_buff *skb_new;

	pr_debug("[MODEM_IF] head_size : %d, data_size : %d (%d)\n", head_size,
				data_size, __LINE__);

	/* first payload data - alloc skb */
	if (!skb) {
		switch (iod->format) {
		case IPC_RFS:
			alloc_size = min(data_size, (int)rest) + head_size;
			alloc_size = min(alloc_size, MAX_RXDATA_SIZE);
			skb = alloc_skb(alloc_size, GFP_ATOMIC);
			if (unlikely(!skb))
				return -ENOMEM;
			/* copy the RFS haeder to skb->data */
			memcpy(skb_put(skb, head_size), hdr->hdr, head_size);
			break;
		default:
			skb = alloc_skb(alloc_size, GFP_ATOMIC);
			if (unlikely(!skb))
				return -ENOMEM;
			break;
		}
		iod->skb_recv = skb;
	}

	/* if recv packet size is larger than user space */
	while (rest_len > MAX_RXDATA_SIZE) {
		len = MAX_RXDATA_SIZE - skb->len;
		memcpy(skb_put(skb, len), buf, len);
		buf += len;
		done_len += len;
		rest -= len;
		rest_len -= len;
		rx_iodev_skb(iod);

		alloc_size = min(rest_len, MAX_RXDATA_SIZE);
		skb_new = alloc_skb(alloc_size, GFP_ATOMIC);
		if (unlikely(!skb_new))
			return -ENOMEM;
		skb = iod->skb_recv = skb_new;
	}

	/* copy data to skb */
	len = min(rest, alloc_size - skb->len);
	pr_debug("[MODEM_IF] rest : %d, alloc_size : %d , len : %d (%d)\n",
				rest, alloc_size, skb->len, __LINE__);

	memcpy(skb_put(skb, len), buf, len);
	done_len += len;
	hdr->flag_len += done_len;

	return done_len;
}

static int rx_iodev_skb_raw(struct io_device *iod)
{
	int err;
	struct sk_buff *skb = iod->skb_recv;
	struct net_device *ndev;

	switch (iod->io_typ) {
	case IODEV_MISC:
		skb_queue_tail(&iod->sk_rx_q, iod->skb_recv);
		wake_up(&iod->wq);
		return 0;

	case IODEV_NET:
		ndev = iod->ndev;
		if (!ndev)
			return NET_RX_DROP;

		skb->dev = ndev;
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += skb->len;
		skb->protocol = htons(ETH_P_IP);

		err = netif_rx(skb);
		if (err != NET_RX_SUCCESS)
			dev_err(&ndev->dev, "rx error: %d\n", err);
		return err;

	default:
		pr_err("[MODEM_IF] wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}
}

static int rx_multipdp(struct io_device *iod)
{
	int ret;
	u8 ch;
	struct raw_hdr *raw_header = (struct raw_hdr *)&iod->h_data.hdr;
	struct io_raw_devices *io_raw_devs =
				(struct io_raw_devices *)iod->private_data;
	struct io_device *real_iod;

	ch = raw_header->channel;
	real_iod = io_raw_devs->raw_devices[ch];

	real_iod->skb_recv = iod->skb_recv;
	ret = rx_iodev_skb_raw(real_iod);
	if (ret < 0)
		pr_err("[MODEM_IF] failed!\n");
	return ret;
}

/* de-mux function draft */
static int rx_iodev_skb(struct io_device *iod)
{
	switch (iod->format) {
	case IPC_MULTI_RAW:
		return rx_multipdp(iod);

	case IPC_FMT:
	case IPC_RFS:
	default:
		skb_queue_tail(&iod->sk_rx_q, iod->skb_recv);

		pr_debug("[MODEM_IF] wake up fmt,rfs skb\n");
		wake_up(&iod->wq);
		return 0;
	}
}

static int rx_hdlc_packet(struct io_device *iod, const char *data,
		unsigned recv_size)
{
	unsigned rest = recv_size;
	char *buf = (char *)data;
	int err;
	int len;

	if (rest <= 0)
		goto exit;

	pr_info("[MODEM_IF] RX_SIZE=%d\n", rest);

next_frame:
	err = len = rx_hdlc_head_check(iod, buf, rest);
	if (err < 0)
		goto exit; /* buf++; rest--; goto next_frame; */
	pr_debug("[MODEM_IF] check len : %d, rest : %d (%d)\n", len, rest,
				__LINE__);

	buf += len;
	rest -= len;
	if (rest <= 0)
		goto exit;

	err = len = rx_hdlc_data_check(iod, buf, rest);
	if (err < 0)
		goto exit;
	pr_debug("[MODEM_IF] check len : %d, rest : %d (%d)\n", len, rest,
				__LINE__);

	buf += len;
	rest -= len;
	if (rest <= 0)
		goto exit;

	err = len = rx_hdlc_tail_check(buf);
	if (err < 0)
		goto exit;
	pr_debug("[MODEM_IF] check len : %d, rest : %d (%d)\n", len, rest,
				__LINE__);

	buf += len;
	rest -= len;
	if (rest < 0)
		goto exit;

	err = rx_iodev_skb(iod);
	if (err < 0)
		goto exit;

	/* initialize header & skb */
	iod->skb_recv = NULL;
	memset(&iod->h_data, 0x00, sizeof(struct header_data));

	if (rest)
		goto next_frame;

exit:
	/* free buffers. mipi-hsi re-use recv buf */

	if (rest < 0)
		err = -ERANGE;

	if (err < 0 && iod->skb_recv) {
		dev_kfree_skb_any(iod->skb_recv);
		iod->skb_recv = NULL;
	}
	/* clear headers */
	memset(&iod->h_data, 0x00, sizeof(struct header_data));
	return err;
}

/* called from link device when a packet arrives for this io device */
static int io_dev_recv_data_from_link_dev(struct io_device *iod,
			const char *data, unsigned int len)
{
	struct sk_buff *skb;
	int err;

	switch (iod->format) {
	case IPC_FMT:
	case IPC_RAW:
	case IPC_RFS:
	case IPC_MULTI_RAW:
		err = rx_hdlc_packet(iod, data, len);
		if (err < 0)
			pr_err("[MODEM_IF] fail process hdlc fram\n");
		return err;

	case IPC_CMD:
		/* TODO- handle flow control command from CP */
		return 0;

	case IPC_BOOT:
		/* save packet to sk_buff */
		skb = alloc_skb(len, GFP_ATOMIC);
		if (!skb) {
			pr_err("[MODEM_IF] fail alloc skb (%d)\n", __LINE__);
			return -ENOMEM;
		}

		pr_debug("[MODEM_IF] boot len : %d\n", len);

		memcpy(skb_put(skb, len), data, len);
		skb_queue_tail(&iod->sk_rx_q, skb);
		pr_debug("[MODEM_IF] skb len : %d\n", skb->len);

		wake_up(&iod->wq);
		return len;

	default:
		return -EINVAL;
	}
}

/* inform the IO device that the modem is now online or offline or
 * crashing or whatever...
 */
static void io_dev_modem_state_changed(struct io_device *iod,
			enum modem_state state)
{
}

static int misc_open(struct inode *inode, struct file *filp)
{
	struct io_device *iod = to_io_device(filp->private_data);
	filp->private_data = (void *)iod;
	return 0;
}

static unsigned int misc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;

	poll_wait(filp, &iod->wq, wait);

	if ((!skb_queue_empty(&iod->sk_rx_q))
				&& (iod->mc->phone_state != STATE_OFFLINE))
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long _arg)
{
	void __user *arg = (void *) _arg;
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct misc_data_io misc_data;
	int frame_len = 0;
	char frame_header_buf[sizeof(struct raw_hdr)];
	struct sk_buff *skb;

	pr_debug("[MODEM_IF] misc_ioctl : 0x%x\n", cmd);

	switch (cmd) {
	case IOCTL_MODEM_SEND:

		/* TODO - check here flow control for only raw data */

		if (copy_from_user(&misc_data, arg, sizeof(misc_data)) != 0)
			return -EFAULT;

		frame_len = misc_data.size + SIZE_OF_HDLC_START
			+ get_header_size(iod) + SIZE_OF_HDLC_END;
		skb = alloc_skb(frame_len, GFP_ATOMIC);
		if (!skb) {
			pr_err("[MODEM_IF] fail alloc skb (%d)\n", __LINE__);
			return -ENOMEM;
		}

		switch (iod->format) {
		case IPC_BOOT:
			if (copy_from_user(skb_put(skb, misc_data.size),
				misc_data.data, misc_data.size) != 0)
				return -EFAULT;
			break;

		case IPC_RFS:
			memcpy(skb_put(skb, SIZE_OF_HDLC_START), hdlc_start,
					SIZE_OF_HDLC_START);
			if (copy_from_user(skb_put(skb, misc_data.size),
					misc_data.data, misc_data.size) != 0)
				return -EFAULT;
			memcpy(skb_put(skb, SIZE_OF_HDLC_END), hdlc_end,
						SIZE_OF_HDLC_END);
			break;

		default:
			memcpy(skb_put(skb, SIZE_OF_HDLC_START), hdlc_start,
					SIZE_OF_HDLC_START);
			memcpy(skb_put(skb, get_header_size(iod)),
				get_header(iod, &misc_data, frame_header_buf),
				get_header_size(iod));
			if (copy_from_user(skb_put(skb, misc_data.size),
					misc_data.data, misc_data.size) != 0)
				return -EFAULT;
			memcpy(skb_put(skb, SIZE_OF_HDLC_END), hdlc_end,
						SIZE_OF_HDLC_END);
			break;
		}

		/* send data with sk_buff, link device will put sk_buff
		 * into the specific sk_buff_q and run work-q to send data
		 */
		return iod->link->send(iod->link, iod, skb);

	case IOCTL_MODEM_RECV:
		if (copy_from_user(&misc_data, arg, sizeof(misc_data)) != 0)
			return -EFAULT;

		skb = skb_dequeue(&iod->sk_rx_q);
		if (!skb) {
			pr_info("[MODEM_IF] no data from sk_rx_q\n");
			return 0;
		}

		if (skb->len > MAX_RXDATA_SIZE) {
			pr_err("[MODEM_IF] skb len is too big = %d!(%d)",
					MAX_RXDATA_SIZE, __LINE__);
			dev_kfree_skb_any(skb);
			return -EFAULT;
		}
		misc_data.size = skb->len;
		pr_debug("[MODEM_IF] skb len : %d\n", skb->len);

		if (copy_to_user(misc_data.data, skb->data, skb->len) != 0)
			return -EFAULT;
		dev_kfree_skb_any(skb);

		if (copy_to_user(arg, &misc_data, sizeof(misc_data)) != 0)
			return -EFAULT;

		pr_debug("[MODEM_IF] copy to user : %d\n", misc_data.size);

		return misc_data.size;

	case IOCTL_MODEM_ON:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_ON\n");
		return iod->mc->ops.modem_on(iod->mc);

	case IOCTL_MODEM_OFF:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_OFF\n");
		return iod->mc->ops.modem_off(iod->mc);

	case IOCTL_MODEM_RESET:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_RESET\n");
		return iod->mc->ops.modem_reset(iod->mc);

	case IOCTL_MODEM_BOOT_ON:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_BOOT_ON\n");
		return iod->mc->ops.modem_boot_on(iod->mc);

	case IOCTL_MODEM_BOOT_OFF:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_BOOT_OFF\n");
		return iod->mc->ops.modem_boot_off(iod->mc);

	case IOCTL_MODEM_START:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_START\n");
		return iod->link->init_comm(iod->link, iod);

	case IOCTL_MODEM_STATUS:
		pr_info("[MODEM_IF] misc_ioctl : IOCTL_MODEM_START\n");
		return iod->mc->phone_state;

	default:
		return -EINVAL;
	}
}

static const struct file_operations misc_io_fops = {
	.owner = THIS_MODULE,
	.open = misc_open,
	.poll = misc_poll,
	.unlocked_ioctl = misc_ioctl,
};

static int vnet_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static int vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int ret;
	struct raw_hdr hd;
	struct sk_buff *skb_new;
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;

	hd.len = skb->len + sizeof(hd);
	hd.control = 0;
	hd.channel = iod->id & 0x1F;

	skb_new = skb_copy_expand(skb, sizeof(hd) + sizeof(hdlc_start),
				sizeof(hdlc_end), GFP_ATOMIC);
	if (!skb_new) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}

	memcpy(skb_push(skb_new, sizeof(hd)), &hd, sizeof(hd));
	memcpy(skb_push(skb_new, sizeof(hdlc_start)), hdlc_start,
				sizeof(hdlc_start));
	memcpy(skb_put(skb_new, sizeof(hdlc_end)), hdlc_end, sizeof(hdlc_end));

	ret = iod->link->send(iod->link, iod, skb_new);
	if (ret < 0) {
		netif_stop_queue(ndev);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_BUSY;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static struct net_device_ops vnet_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_stop,
	.ndo_start_xmit = vnet_xmit,
};

static void vnet_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &vnet_ops;
	ndev->type = ARPHRD_PPP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->tx_queue_len = 1000;
	ndev->mtu = ETH_DATA_LEN;
	ndev->watchdog_timeo = 5 * HZ;
}

int init_io_device(struct io_device *iod)
{
	int ret = 0;
	struct vnet *vnet;

	/* get modem state from modem control device */
	iod->modem_state_changed = io_dev_modem_state_changed;
	/* get data from link device */
	iod->recv = io_dev_recv_data_from_link_dev;

	INIT_LIST_HEAD(&iod->list);

	/* register misc or net drv */
	switch (iod->io_typ) {
	case IODEV_MISC:
		init_waitqueue_head(&iod->wq);
		skb_queue_head_init(&iod->sk_rx_q);

		iod->miscdev.minor = MISC_DYNAMIC_MINOR;
		iod->miscdev.name = iod->name;
		iod->miscdev.fops = &misc_io_fops;

		ret = misc_register(&iod->miscdev);
		if (ret)
			pr_err("failed to register misc io device : %s\n",
						iod->name);

		break;

	case IODEV_NET:
		iod->ndev = alloc_netdev(0, iod->name, vnet_setup);
		if (!iod->ndev) {
			pr_err("failed to alloc netdev\n");
			return -ENOMEM;
		}

		ret = register_netdev(iod->ndev);
		if (ret)
			free_netdev(iod->ndev);

		pr_err("%s: %d(iod:0x%p)", __func__, __LINE__, iod);
		vnet = netdev_priv(iod->ndev);
		pr_err("%s: %d(vnet:0x%p)", __func__, __LINE__, vnet);
		vnet->iod = iod;

		break;

	case IODEV_DUMMY:
		break;

	default:
		pr_err("wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}

	pr_info("[MODEM_IF] %s(%d) : init_io_device() done : %d\n",
				iod->name, iod->io_typ, ret);
	return ret;
}

