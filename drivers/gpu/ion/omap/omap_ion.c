/*
 * drivers/gpu/omap/omap_ion.c
 *
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/err.h>
#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../ion_priv.h"
#include "omap_ion_priv.h"

struct ion_device *omap_ion_device;
EXPORT_SYMBOL(omap_ion_device);

int num_heaps;
struct ion_heap **heaps;
struct ion_heap *tiler_heap;
static struct ion_heap *nonsecure_tiler_heap;

int omap_ion_tiler_alloc(struct ion_client *client,
			 struct omap_ion_tiler_alloc_data *data)
{
	return omap_tiler_alloc(tiler_heap, client, data);
}

int omap_ion_nonsecure_tiler_alloc(struct ion_client *client,
			 struct omap_ion_tiler_alloc_data *data)
{
	if (!nonsecure_tiler_heap)
		return -ENOMEM;
	return omap_tiler_alloc(nonsecure_tiler_heap, client, data);
}

long omap_ion_ioctl(struct ion_client *client, unsigned int cmd,
		    unsigned long arg)
{
	switch (cmd) {
	case OMAP_ION_TILER_ALLOC:
	{
		struct omap_ion_tiler_alloc_data data;
		int ret;

		if (!tiler_heap) {
			pr_err("%s: Tiler heap requested but no tiler "
			       "heap exists on this platform\n", __func__);
			return -EINVAL;
		}
		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;
		ret = omap_ion_tiler_alloc(client, &data);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &data,
				 sizeof(data)))
			return -EFAULT;
		break;
	}
	default:
		pr_err("%s: Unknown custom ioctl\n", __func__);
		return -ENOTTY;
	}
	return 0;
}

int omap_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	int err;
	int i;

	num_heaps = pdata->nr;

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	omap_ion_device = ion_device_create(omap_ion_ioctl);
	if (IS_ERR_OR_NULL(omap_ion_device)) {
		kfree(heaps);
		return PTR_ERR(omap_ion_device);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		if (heap_data->type == OMAP_ION_HEAP_TYPE_TILER) {
			heaps[i] = omap_tiler_heap_create(heap_data);
			if (heap_data->id == OMAP_ION_HEAP_NONSECURE_TILER)
				nonsecure_tiler_heap = heaps[i];
			else
				tiler_heap = heaps[i];
		} else {
			heaps[i] = ion_heap_create(heap_data);
		}
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(omap_ion_device, heaps[i]);
		pr_info("%s: adding heap %s of type %d with %lx@%x\n",
			__func__, heap_data->name, heap_data->type,
			heap_data->base, heap_data->size);

	}

	platform_set_drvdata(pdev, omap_ion_device);
	return 0;
err:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i]) {
			if (heaps[i]->type == OMAP_ION_HEAP_TYPE_TILER)
				omap_tiler_heap_destroy(heaps[i]);
			else
				ion_heap_destroy(heaps[i]);
		}
	}
	kfree(heaps);
	return err;
}

int omap_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		if (heaps[i]->type == OMAP_ION_HEAP_TYPE_TILER)
			omap_tiler_heap_destroy(heaps[i]);
		else
			ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}

static struct platform_driver ion_driver = {
	.probe = omap_ion_probe,
	.remove = omap_ion_remove,
	.driver = { .name = "ion-omap4" }
};

static int __init ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void __exit ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

