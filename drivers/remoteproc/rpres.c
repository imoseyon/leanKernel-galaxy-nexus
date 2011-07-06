/*
 * Remote processor resources
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <plat/omap_device.h>
#include <plat/rpres.h>

static LIST_HEAD(rpres_list);
static DEFINE_SPINLOCK(rpres_lock);

static struct rpres *__find_by_name(const char *name)
{
	struct rpres *obj;

	list_for_each_entry(obj, &rpres_list, next)
		if (!strcmp(obj->name, name))
			return obj;
	return NULL;
}

struct rpres *rpres_get(const char *name)
{
	int ret;
	struct rpres *r;
	struct rpres_platform_data *pdata;

	spin_lock(&rpres_lock);
	r = __find_by_name(name);
	spin_unlock(&rpres_lock);
	if (!r)
		return ERR_PTR(-ENOENT);

	mutex_lock(&r->lock);
	if (r->state == RPRES_ACTIVE) {
		pr_err("%s:resource already active\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	pdata = r->pdev->dev.platform_data;
	ret = pdata->ops->start(r->pdev);
	if (!ret)
		r->state = RPRES_ACTIVE;
out:
	mutex_unlock(&r->lock);
	if (ret)
		return ERR_PTR(ret);
	return r;
}
EXPORT_SYMBOL(rpres_get);

void rpres_put(struct rpres *obj)
{
	struct rpres_platform_data *pdata = obj->pdev->dev.platform_data;
	mutex_lock(&obj->lock);
	if (obj->state == RPRES_INACTIVE) {
		pr_err("%s:resource already inactive\n", __func__);
	} else {
		pdata->ops->stop(obj->pdev);
		obj->state = RPRES_INACTIVE;
	}
	mutex_unlock(&obj->lock);
}
EXPORT_SYMBOL(rpres_put);

int rpres_set_constraints(struct rpres *obj, enum rpres_constraint type, long val)
{
	int ret;
	struct rpres_platform_data *pdata = obj->pdev->dev.platform_data;
	struct platform_device *pdev = obj->pdev;
	static const char *cname[] = {"scale", "latency", "bandwidth"};
	int (*func)(struct platform_device *, long);

	switch (type) {
	case RPRES_CONSTRAINT_SCALE:
		func = pdata->ops->scale_dev;
		break;
	case RPRES_CONSTRAINT_LATENCY:
		func = pdata->ops->set_lat;
		break;
	case RPRES_CONSTRAINT_BANDWIDTH:
		func = pdata->ops->set_bw;
		break;
	default:
		dev_err(&pdev->dev, "%s: invalid constraint %d\n",
			__func__, type);
		return -EINVAL;
	}

	if (!func) {
		dev_err(&pdev->dev, "%s: No %s constraint\n",
			__func__, cname[type]);
		return -EINVAL;
	}

	mutex_lock(&obj->lock);
	if (obj->state == RPRES_INACTIVE) {
		mutex_unlock(&obj->lock);
		pr_err("%s: resource inactive\n", __func__);
		return -EPERM;
	}

	dev_dbg(&pdev->dev, "set %s constraint %ld\n", cname[type], val);
	ret = func(pdev, val);
	if (ret)
		dev_err(&pdev->dev, "%s: error setting constraint %s\n",
				__func__, cname[type]);
	mutex_unlock(&obj->lock);

	return ret;
}
EXPORT_SYMBOL(rpres_set_constraints);

static int rpres_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpres_platform_data *pdata = dev->platform_data;
	struct rpres *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	obj->pdev = pdev;
	obj->name = pdata->name;
	obj->state = RPRES_INACTIVE;
	mutex_init(&obj->lock);

	spin_lock(&rpres_lock);
	list_add_tail(&obj->next, &rpres_list);
	spin_unlock(&rpres_lock);

	return 0;
}

static int __devexit rpres_remove(struct platform_device *pdev)
{
	struct rpres_platform_data *pdata = pdev->dev.platform_data;
	struct rpres *obj;

	spin_lock(&rpres_lock);
	obj = __find_by_name(pdata->name);
	if (!obj) {
		spin_unlock(&rpres_lock);
		dev_err(&pdev->dev, "fail to remove %s\n", pdata->name);
		return -ENOENT;
	}
	list_del(&obj->next);
	spin_unlock(&rpres_lock);

	kfree(obj);

	return 0;
}

static struct platform_device_id rpres_id_table[] = {
	{
		.name	= "iva",
	},
	{
		.name	= "fdif",
	},
	{
		.name	= "rpres",
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, rpres_id_table);

static struct platform_driver omap_rpres_driver = {
	.id_table	= rpres_id_table,
	.probe		= rpres_probe,
	.remove		= __devexit_p(rpres_remove),
	.driver		= {
		.name = "rpres",
		.owner = THIS_MODULE,
	},
};

static int __init rpres_init(void)
{
	return platform_driver_register(&omap_rpres_driver);
}
late_initcall(rpres_init);

static void __exit rpres_exit(void)
{
	platform_driver_unregister(&omap_rpres_driver);
}
module_exit(rpres_exit);

MODULE_LICENSE("GPL v2");
