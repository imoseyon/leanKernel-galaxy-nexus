/* /linux/drivers/new_modem_if/link_dev_usb.c
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_link_device_usb.h"

static irqreturn_t usb_resume_irq(int irq, void *data);
static void usb_rx_complete(struct urb *urb);

static int usb_attach_io_dev(struct link_device *ld,
			struct io_device *iod)
{
	struct usb_link_device *usb_ld = to_usb_link_device(ld);

	iod->link = ld;

	/* list up io devices */
	list_add(&iod->list, &usb_ld->list_of_io_devices);

	return 0;
}

static int usb_init_communication(struct link_device *ld,
			struct io_device *iod)
{
	return 0;
}

static int usb_rx_submit(struct usb_link_device *usb_ld,
					struct if_usb_devdata *pipe_data,
					gfp_t gfp_flags)
{
	int ret;
	struct urb *urb;

	urb = pipe_data->urb;

	urb->transfer_flags = 0;
	usb_fill_bulk_urb(urb, usb_ld->usbdev,
					pipe_data->rx_pipe, pipe_data->rx_buf,
					pipe_data->rx_buf_size, usb_rx_complete,
					(void *)pipe_data);

	ret = usb_submit_urb(urb, gfp_flags);
	if (ret)
		pr_err("%s: submit urb fail with ret (%d)\n", __func__, ret);

	return ret;
}

static void usb_rx_complete(struct urb *urb)
{
	struct if_usb_devdata *pipe_data = urb->context;
	struct usb_link_device *usb_ld = usb_get_intfdata(pipe_data->data_intf);
	struct io_device *iod;
	int iod_format = IPC_FMT;
	int ret;

	switch (urb->status) {
	case 0:
		if (!urb->actual_length)
			goto re_submit;
		/* call iod recv */
		/* how we can distinguish boot ch with fmt ch ?? */
		switch (pipe_data->format) {
		case IF_USB_FMT_EP:
			iod_format = IPC_FMT;
			break;
		case IF_USB_RAW_EP:
			iod_format = IPC_MULTI_RAW;
			break;
		case IF_USB_RFS_EP:
			iod_format = IPC_RFS;
			break;
		default:
			break;
		}

		list_for_each_entry(iod, &usb_ld->list_of_io_devices, list) {
			/* during boot stage fmt end point */
			/* shared with boot io device */
			/* when we use fmt device only, at boot and ipc exchange
				it can be reduced to 1 device */
			if (iod_format == IPC_FMT &&
				unlikely(iod->mc->phone_state == STATE_BOOTING))
				iod_format = IPC_BOOT;

			if (iod->format == iod_format) {
				ret = iod->recv(iod,
						(char *)urb->transfer_buffer,
						urb->actual_length);
				if (ret < 0)
					pr_err("%s: io device recv error (%d)\n",
								__func__, ret);
				break;
			}
		}
re_submit:
		usb_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		pr_debug("%s: RX complete Status(%d)\n", __func__, urb->status);
		break;
	case -EOVERFLOW:
		pr_err("%s: RX overflow\n", __func__);
		break;
	case -EILSEQ:
		break;
	default:
		break;
	}
}

static int usb_send(struct link_device *ld, struct io_device *iod,
			struct sk_buff *skb)
{
	struct sk_buff_head *txq;

	switch (iod->format) {
	case IPC_RAW:
		txq = &ld->sk_raw_tx_q;
		break;

	case IPC_FMT:
	case IPC_RFS:
	case IPC_BOOT:
	default:
		txq = &ld->sk_fmt_tx_q;
		break;
	}

	/* save io device into cb area */
	*((struct io_device **)skb->cb) = iod;
	/* en queue skb data */
	skb_queue_tail(txq, skb);

	if (!work_pending(&ld->tx_delayed_work.work))
		queue_delayed_work(ld->tx_wq, &ld->tx_delayed_work, 0);

	return skb->len;
}

static void usb_tx_complete(struct urb *urb)
{
	int ret = 0;
	struct sk_buff *skb = urb->context;

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	default:
		pr_err("%s:TX error (%d)\n", __func__, urb->status);
	}

	usb_mark_last_busy(urb->dev);
	ret = pm_runtime_put(&urb->dev->dev);
	if (ret < 0)
		pr_debug("%s pm_runtime_put failed : ret(%d)\n", __func__, ret);
	usb_free_urb(urb);
	dev_kfree_skb_any(skb);
}

static int usb_tx_urb_with_skb(struct usb_link_device *usb_ld,
		struct sk_buff *skb, struct if_usb_devdata *pipe_data)
{
	int ret;
	struct urb *urb;
	struct usb_device *usbdev = usb_ld->usbdev;

	if (!usbdev)
		return -ENODEV;

	usb_mark_last_busy(usbdev);
	pm_runtime_get_noresume(&usbdev->dev);

	if (pm_runtime_suspended(&usbdev->dev)) {
		usb_ld->resume_status = AP_INITIATED_RESUME;
		SET_SLAVE_WAKEUP(usb_ld->pdata, 1);

		if (!wait_event_interruptible_timeout(usb_ld->l2_wait,
			(usbdev->dev.power.runtime_status == RPM_ACTIVE),
			2000)) {
			pr_err("Modem wakeup timeout %d\n", 2000);
			SET_SLAVE_WAKEUP(usb_ld->pdata, 0);
			pm_runtime_put(&usbdev->dev);
			return -1;
		}
		pr_debug("wait_q done (runtime_status=%d)\n",
				usbdev->dev.power.runtime_status);
		SET_SLAVE_WAKEUP(usb_ld->pdata, 0);
	}

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		pr_err("%s alloc urb error\n", __func__);
		if (pm_runtime_put_sync(&usbdev->dev) < 0)
			pr_debug("pm_runtime_put_sync fail\n");
		return -ENOMEM;
	}

	urb->transfer_flags = URB_ZERO_PACKET;
	usb_fill_bulk_urb(urb, usbdev, pipe_data->tx_pipe, skb->data,
			skb->len, usb_tx_complete, (void *)skb);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret < 0) {
		pr_err("%s usb_submit_urb with ret(%d)\n", __func__, ret);
		if (pm_runtime_put_sync(&usbdev->dev) < 0)
			pr_debug("pm_runtime_put_sync fail\n");
		return ret;
	}

	return 0;
}

static void usb_tx_work(struct work_struct *work)
{
	int ret = 0;
	struct link_device *ld =
		container_of(work, struct link_device, tx_delayed_work.work);
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct io_device *iod;
	struct sk_buff *skb;
	struct if_usb_devdata *pipe_data;

	while (ld->sk_fmt_tx_q.qlen || ld->sk_raw_tx_q.qlen) {
		/* send skb from fmt_txq and raw_txq,
		 * one by one for fair flow control */
		skb = skb_dequeue(&ld->sk_fmt_tx_q);
		if (skb) {
			iod = *((struct io_device **)skb->cb);
			switch (iod->format) {
			case IPC_BOOT:
			case IPC_FMT:
				/* boot device uses same intf with fmt*/
				pipe_data = &usb_ld->devdata[IF_USB_FMT_EP];
				break;
			case IPC_RFS:
				pipe_data = &usb_ld->devdata[IF_USB_RFS_EP];
				break;
			default:
				/* wrong packet for fmt tx q , drop it */
				pipe_data =  NULL;
				break;
			}

			if (!pipe_data) {
				dev_kfree_skb_any(skb);
				continue;
			}

			ret = usb_tx_urb_with_skb(usb_ld, skb, pipe_data);
			if (ret < 0) {
				pr_err("%s usb_tx_urb_with_skb for iod(%d)\n",
						__func__, iod->format);
				skb_queue_head(&ld->sk_fmt_tx_q, skb);
				return;
			}
		}

		skb = skb_dequeue(&ld->sk_raw_tx_q);
		if (skb) {
			iod = *((struct io_device **)skb->cb);
			switch (iod->format) {
			case IPC_RAW:
				pipe_data = &usb_ld->devdata[IF_USB_RAW_EP];
				break;
			default:
				/* wrong packet for raw tx q , drop it */
				pipe_data =  NULL;
				break;
			}

			if (!pipe_data) {
				dev_kfree_skb_any(skb);
				continue;
			}

			ret = usb_tx_urb_with_skb(usb_ld, skb, pipe_data);
			if (ret < 0) {
				pr_err("%s usb_tx_urb_with_skb for iod(%d)\n",
						__func__, iod->format);
				skb_queue_head(&ld->sk_fmt_tx_q, skb);
				return;
			}
		}
	}
}

static int if_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_link_device *usb_ld = usb_get_intfdata(intf);
	int i;

	pr_debug("%s: cnt=%d\n", __func__, usb_ld->suspend_count);

	if (!usb_ld->suspend_count++) {
		pr_debug("%s\n", __func__);

		for (i = 0; i < IF_USB_DEVNUM_MAX; i++)
			usb_kill_urb(usb_ld->devdata[i].urb);
	}

	return 0;
}

static int if_usb_resume(struct usb_interface *intf)
{
	int i;
	int ret;
	struct usb_link_device *usb_ld = usb_get_intfdata(intf);

	pr_debug("%s: cnt=%d\n", __func__, usb_ld->suspend_count);

	if (!--usb_ld->suspend_count) {
		pr_debug("%s\n", __func__);

		for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
			ret = usb_rx_submit(usb_ld, &usb_ld->devdata[i],
					GFP_KERNEL);
			if (ret < 0) {
				pr_err("%s: usb_rx_submit error with (%d)\n",
						__func__, ret);
				return ret;
			}
		}

		SET_SLAVE_WAKEUP(usb_ld->pdata, 1);
	}

	return 0;
}

static int if_usb_reset_resume(struct usb_interface *intf)
{
	int ret;

	pr_debug("%s\n", __func__);
	ret = if_usb_resume(intf);
	return ret;
}

static struct usb_device_id if_usb_ids[] = {
	{ USB_DEVICE(0x04e8, 0x6999), /* CMC221 LTE Modem */
	/*.driver_info = 0,*/
	},
	{ } /* terminating entry */
};
MODULE_DEVICE_TABLE(usb, if_usb_ids);

static struct usb_driver if_usb_driver;
static void if_usb_disconnect(struct usb_interface *intf)
{
	struct usb_link_device *usb_ld  = usb_get_intfdata(intf);
	struct usb_device *usbdev = usb_ld->usbdev;
	int dev_id = intf->altsetting->desc.bInterfaceNumber;
	struct device *ppdev;

	SET_HOST_ACTIVE(usb_ld->pdata, 0);

	usb_set_intfdata(intf, NULL);

	if (usb_ld->devdata[dev_id].disconnected)
		return;

	if (usb_ld->if_usb_connected) {
		disable_irq_wake(usb_ld->pdata->irq_host_wakeup);
		free_irq(usb_ld->pdata->irq_host_wakeup, usb_ld);
	}
	usb_ld->if_usb_connected = 0;
	usb_ld->flow_suspend = 1;

	dev_dbg(&usbdev->dev, "%s\n", __func__);
	usb_ld->dev_count--;
	usb_ld->devdata[dev_id].disconnected = 1;
	usb_driver_release_interface(&if_usb_driver,
			usb_ld->devdata[dev_id].data_intf);

	ppdev = usbdev->dev.parent->parent;
	/*pm_runtime_forbid(ppdev);*/ /*ehci*/

	usb_kill_urb(usb_ld->devdata[dev_id].urb);

	if (usb_ld->dev_count == 0) {
		usb_put_dev(usbdev);
		usb_ld->usbdev = NULL;

		cancel_delayed_work_sync(&usb_ld->ld.tx_delayed_work);
		cancel_work_sync(&usb_ld->post_resume_work);
		if (!usb_ld->driver_info) {
			/*TODO:check the Phone ACTIVE pin*/
#ifdef AIRPLAIN_MODE_TEST
			if (lte_airplain_mode == 0) {
				printk(KERN_INFO "%s reconnect_work\n",
							__func__);
				usb_ld->reconnect_cnt = 5;
				schedule_delayed_work(&usb_ld->reconnect_work,
						10);
			}
#endif
			/*wake_unlock_pm(svn);*/
		}
	}

}

static int __devinit if_usb_probe(struct usb_interface *intf,
					const struct usb_device_id *id)
{
	struct usb_host_interface *data_desc;
	struct usb_link_device *usb_ld =
			(struct usb_link_device *)id->driver_info;
	struct usb_interface *data_intf;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct device *dev;

	int i;
	int dev_id;
	int err;

	/* To detect usb device order probed */
	dev_id = intf->cur_altsetting->desc.bInterfaceNumber;

	if (dev_id >= IF_USB_DEVNUM_MAX) {
		dev_err(&intf->dev, "Device id %d cannot support\n",
								dev_id);
		return -EINVAL;
	}

	pr_info("%s: probe dev_id=%d usb_device_id(0x%p), usb_ld (0x%p)\n",
				__func__, dev_id, id, usb_ld);

	usb_ld->usbdev = usbdev;
	usb_ld->driver_info = (unsigned long)id->driver_info;

	if (!usb_ld) {
		dev_err(&intf->dev,
		"if_usb device doesn't be allocated\n");
		err = ENOMEM;
		goto out;
	}

	usb_get_dev(usbdev);

	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		data_intf = usb_ifnum_to_if(usbdev, i);

		/* remap endpoint of RAW to no.1 for LTE modem */
		if (i == 0)
			i = 1;
		else if (i == 1)
			i = 0;


		usb_ld->devdata[i].data_intf = data_intf;
		data_desc = data_intf->cur_altsetting;

		/* Endpoints */
		if (usb_pipein(data_desc->endpoint[0].desc.bEndpointAddress)) {
			usb_ld->devdata[i].rx_pipe = usb_rcvbulkpipe(usbdev,
				data_desc->endpoint[0].desc.bEndpointAddress);
			usb_ld->devdata[i].tx_pipe = usb_sndbulkpipe(usbdev,
				data_desc->endpoint[1].desc.bEndpointAddress);
		} else {
			usb_ld->devdata[i].rx_pipe = usb_rcvbulkpipe(usbdev,
				data_desc->endpoint[1].desc.bEndpointAddress);
			usb_ld->devdata[i].tx_pipe = usb_sndbulkpipe(usbdev,
				data_desc->endpoint[0].desc.bEndpointAddress);
		}


		/* remap endpoint of RAW to no.1 for LTE modem */
		if (i == 0)
			i = 1;
		else if (i == 1)
			i = 0;


		if (i == 0) {
			usb_set_intfdata(data_intf, usb_ld);
			usb_ld->dev_count++;

			dev_err(&usbdev->dev, "USB IF USB device found\n");

			pm_suspend_ignore_children(&data_intf->dev, true);
		} else {
			err = usb_driver_claim_interface(&if_usb_driver,
					data_intf, usb_ld);
			if (err < 0) {
				pr_err("%s - failed to cliam usb interface\n",
						__func__);
				goto out;
			}

			usb_set_intfdata(data_intf, usb_ld);
			usb_ld->dev_count++;

			pm_suspend_ignore_children(&data_intf->dev, true);
		}

		usb_ld->devdata[i].disconnected = 0;
	}

	/* temporary call reset_resume */
	if_usb_reset_resume(data_intf);

	SET_HOST_ACTIVE(usb_ld->pdata, 1);

	if (gpio_get_value(usb_ld->pdata->gpio_phone_active)) {
		pm_runtime_set_autosuspend_delay(&usbdev->dev, 1000);
		dev = &usb_ld->usbdev->dev;
		if (dev->parent) {
			dev_dbg(&usbdev->dev, "if_usb Runtime PM Start!!\n");
			usb_enable_autosuspend(usb_ld->usbdev);
		}
		usb_ld->if_usb_connected = 1;
		usb_ld->flow_suspend = 0;

		err = request_threaded_irq(usb_ld->pdata->irq_host_wakeup,
			NULL,
			usb_resume_irq,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"modem_usb_wake",
			usb_ld);
		if (err)
			pr_err("Failed to allocate an interrupt(%d)\n",
					usb_ld->pdata->irq_host_wakeup);

		enable_irq_wake(usb_ld->pdata->irq_host_wakeup);
	}

	return 0;

out:
	usb_set_intfdata(intf, NULL);
	return err;
}

static void if_usb_free_pipe_data(struct usb_link_device *usb_ld)
{
	int i;
	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		kfree(usb_ld->devdata[i].rx_buf);
		usb_kill_urb(usb_ld->devdata[i].urb);
	}
}

static irqreturn_t usb_resume_irq(int irq, void *data)
{
	int ret;
	struct usb_link_device *usb_ld = data;
	int val;
	struct device *dev;

	val = gpio_get_value(usb_ld->pdata->gpio_host_wakeup);
	irq_set_irq_type(irq, val ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	dev = &usb_ld->usbdev->dev;

	pr_debug("< H-WUP %d\n", val);

	if (val) {
		wake_lock(&usb_ld->wakelock);
		device_lock(dev);
		if (dev->power.is_prepared || dev->power.is_suspended) {
			pm_runtime_get_noresume(dev);
			ret = 0;
		}
		else {
			ret = pm_runtime_get_sync(dev);
		}
		device_unlock(dev);
		if (ret < 0) {
			pr_err("%s pm_runtime_get fail (%d)\n", __func__, ret);
			return IRQ_HANDLED;
		}
	} else {
		wake_lock_timeout(&usb_ld->wakelock, 100);
		if (usb_ld->resume_status == AP_INITIATED_RESUME)
			wake_up(&usb_ld->l2_wait);
		SET_SLAVE_WAKEUP(usb_ld->pdata, 0);
		usb_ld->resume_status = CP_INITIATED_RESUME;
		pm_runtime_put(&usb_ld->usbdev->dev);
	}

	return IRQ_HANDLED;
}

static int if_usb_init(struct link_device *ld)
{
	int ret;
	int i;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;

	/* give it to probe, or global variable needed */
	if_usb_ids[0].driver_info = (unsigned long)usb_ld;

	ret = usb_register(&if_usb_driver);
	if (ret) {
		pr_err("usb_register_driver() fail : %d\n", ret);
		return ret;
	}

	/* allocate rx buffer for usb receive */
	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		pipe_data = &usb_ld->devdata[i];
		pipe_data->format = i;
		if (i == IF_USB_RFS_EP)
			pipe_data->rx_buf_size = 256 * 1024;
		else
			pipe_data->rx_buf_size = 16 * 1024;

		pipe_data->rx_buf = kmalloc(pipe_data->rx_buf_size,
						GFP_DMA | GFP_KERNEL);
		if (!pipe_data->rx_buf) {
			if_usb_free_pipe_data(usb_ld);
			ret = -ENOMEM;
			break;
		}

		pipe_data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pipe_data->urb) {
			pr_err("%s: alloc urb fail\n", __func__);
			if_usb_free_pipe_data(usb_ld);
			return -ENOMEM;
		}
	}
	init_waitqueue_head(&usb_ld->l2_wait);

	return ret;
}

struct link_device *usb_create_link_device(void *data)
{
	int ret;
	struct modem_data *pdata;
	struct platform_device *pdev = (struct platform_device *)data;
	struct usb_link_device *usb_ld;
	struct link_device *ld;

	pdata = pdev->dev.platform_data;

	usb_ld = kzalloc(sizeof(struct usb_link_device), GFP_KERNEL);
	if (!usb_ld)
		return NULL;

	INIT_LIST_HEAD(&usb_ld->list_of_io_devices);
	skb_queue_head_init(&usb_ld->ld.sk_fmt_tx_q);
	skb_queue_head_init(&usb_ld->ld.sk_raw_tx_q);

	ld = &usb_ld->ld;
	usb_ld->pdata = pdata;

	ld->name = "usb";
	ld->attach = usb_attach_io_dev;
	ld->init_comm = usb_init_communication;
	ld->send = usb_send;
	ld->com_state = COM_NONE;

	/*ld->tx_wq = create_singlethread_workqueue("usb_tx_wq");*/
	ld->tx_wq = alloc_workqueue("usb_tx_wq",
		WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);

	if (!ld->tx_wq) {
		pr_err("fail to create work Q.\n");
		return NULL;
	}

	usb_ld->pdata->irq_host_wakeup = platform_get_irq(pdev, 1);
	wake_lock_init(&usb_ld->wakelock, WAKE_LOCK_SUSPEND, "modem_usb_link");
	wake_lock_init(&usb_ld->writelock, WAKE_LOCK_SUSPEND, "modem_usb_write");

	INIT_DELAYED_WORK(&ld->tx_delayed_work, usb_tx_work);

	ret = if_usb_init(ld);
	if (ret)
		return NULL;

	return (void *)ld;
}

static struct usb_driver if_usb_driver = {
	.name =		"if_usb_driver",
	.probe =		if_usb_probe,
	.disconnect =	if_usb_disconnect,
	.id_table =	if_usb_ids,
	.suspend =	if_usb_suspend,
	.resume =	if_usb_resume,
	.reset_resume =	if_usb_reset_resume,
	.supports_autosuspend = 1,
};

static void __exit if_usb_exit(void)
{
	usb_deregister(&if_usb_driver);
}

