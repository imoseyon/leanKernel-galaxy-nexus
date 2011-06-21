/*
 * Remote processor machine-specific module for OMAP4
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include <plat/iommu.h>
#include <plat/omap_device.h>
#include <plat/remoteproc.h>

struct omap_rproc_priv {
	struct iommu *iommu;
	int (*iommu_cb)(struct rproc *, u64, u32);
};

static int
omap_rproc_map(struct device *dev, struct iommu *obj, u32 da, u32 pa, u32 size)
{
	struct iotlb_entry e;
	u32 all_bits;
	u32 pg_size[] = {SZ_16M, SZ_1M, SZ_64K, SZ_4K};
	int size_flag[] = {MMU_CAM_PGSZ_16M, MMU_CAM_PGSZ_1M,
		MMU_CAM_PGSZ_64K, MMU_CAM_PGSZ_4K};
	int i, ret;

	while (size) {
		/*
		 * To find the max. page size with which both PA & VA are
		 * aligned
		 */
		all_bits = pa | da;
		for (i = 0; i < 4; i++) {
			if ((size >= pg_size[i]) &&
				((all_bits & (pg_size[i] - 1)) == 0)) {
				break;
			}
		}

		memset(&e, 0, sizeof(e));

		e.da = da;
		e.pa = pa;
		e.valid = 1;
		e.pgsz = size_flag[i];
		e.endian = MMU_RAM_ENDIAN_LITTLE;
		e.elsz = MMU_RAM_ELSZ_32;

		ret = iopgtable_store_entry(obj, &e);
		if (ret) {
			dev_err(dev, "iopgtable_store_entry fail: %d\n", ret);
			return ret;
		}

		size -= pg_size[i];
		da += pg_size[i];
		pa += pg_size[i];
	}

	return 0;
}


static int omap_rproc_iommu_isr(struct iommu *iommu, u32 da, u32 errs, void *p)
{
	struct rproc *rproc = p;
	struct omap_rproc_priv *rpp = rproc->priv;
	int ret = -EIO;

	if (rpp && rpp->iommu_cb)
		ret = rpp->iommu_cb(rproc, (u64)da, errs);

	return ret;
}

static int omap_rproc_iommu_init(struct rproc *rproc,
		 int (*callback)(struct rproc *rproc, u64 fa, u32 flags))
{
	struct device *dev = rproc->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	int ret, i;
	struct iommu *iommu;
	struct omap_rproc_priv *rpp;

	rpp = kmalloc(sizeof(*rpp), GFP_KERNEL);
	if (!rpp)
		return -ENOMEM;

	iommu_set_isr(pdata->iommu_name, omap_rproc_iommu_isr, rproc);
	iommu = iommu_get(pdata->iommu_name);
	if (IS_ERR_OR_NULL(iommu)) {
		dev_err(dev, "iommu_get error: %ld\n", PTR_ERR(iommu));
		ret = PTR_ERR(iommu);
		goto err_mmu;
	}

	rpp->iommu = iommu;
	rpp->iommu_cb = callback;
	rproc->priv = rpp;

	for (i = 0; rproc->memory_maps[i].size; i++) {
		const struct rproc_mem_entry *me = &rproc->memory_maps[i];

		ret = omap_rproc_map(dev, iommu, me->da, me->pa, me->size);
		if (ret)
			goto err_map;
	}
	return 0;
err_map:
	iommu_put(iommu);
err_mmu:
	kfree(rpp);
	return ret;
}

static inline int omap_rproc_start(struct rproc *rproc, u64 bootaddr)
{
	struct platform_device *pdev = to_platform_device(rproc->dev);

	return omap_device_enable(pdev);
}

static int omap_rproc_iommu_exit(struct rproc *rproc)
{
	struct omap_rproc_priv *rpp = rproc->priv;

	iommu_put(rpp->iommu);
	kfree(rpp);

	return 0;
}

static inline int omap_rproc_stop(struct rproc *rproc)
{
	struct platform_device *pdev = to_platform_device(rproc->dev);

	return omap_device_shutdown(pdev);
}

static struct rproc_ops omap_rproc_ops = {
	.start = omap_rproc_start,
	.stop = omap_rproc_stop,
	.iommu_init = omap_rproc_iommu_init,
	.iommu_exit = omap_rproc_iommu_exit,
};

static int omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;

	return rproc_register(&pdev->dev, pdata->name, &omap_rproc_ops,
				pdata->firmware, pdata->memory_maps,
				THIS_MODULE);
}

static int __devexit omap_rproc_remove(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;

	return rproc_unregister(pdata->name);
}

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = __devexit_p(omap_rproc_remove),
	.driver = {
		.name = "omap-rproc",
		.owner = THIS_MODULE,
	},
};

static int __init omap_rproc_init(void)
{
	return platform_driver_register(&omap_rproc_driver);
}
/* must be ready in time for device_initcall users */
subsys_initcall(omap_rproc_init);

static void __exit omap_rproc_exit(void)
{
	platform_driver_unregister(&omap_rproc_driver);
}
module_exit(omap_rproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
