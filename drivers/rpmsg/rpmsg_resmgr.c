/*
 * Remote processor resource manager
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Miguel Vadillo <vadillo@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/slab.h>
#include <linux/rpmsg.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/remoteproc.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/rpmsg_resmgr.h>
#include <linux/pm_runtime.h>
#include <plat/dmtimer.h>
#include <plat/rpres.h>
#include <plat/clock.h>
#include <plat/dma.h>
#include <plat/i2c.h>
#include <plat/omap_hwmod.h>

#define NAME_SIZE	50
#define REGULATOR_MAX	1
#define NUM_SRC_CLK	3
#define AUX_CLK_MIN	0
#define AUX_CLK_MAX	5
#define GPTIMERS_MAX	11
#define MHZ		1000000
#define MAX_MSG		(sizeof(struct rprm_ack) + sizeof(struct rprm_sdma))

static struct dentry *rprm_dbg;

static char *regulator_name[] = {
	"cam2pwr"
};

static char *clk_src_name[] = {
	"sys_clkin_ck",
	"dpll_core_m3x2_ck",
	"dpll_per_m3x2_ck",
};

static const char const *rnames[] = {
	[RPRM_GPTIMER]		= "GP Timer",
	[RPRM_L3BUS]		= "L3 bus",
	[RPRM_IVAHD]		= "IVA HD",
	[RPRM_IVASEQ0]		= "IVA SEQ0",
	[RPRM_IVASEQ1]		= "IVA SEQ1",
	[RPRM_ISS]		= "ISS",
	[RPRM_SL2IF]		= "SL2IF",
	[RPRM_FDIF]		= "FDIF",
	[RPRM_AUXCLK]		= "AUXCLK",
	[RPRM_REGULATOR]	= "REGULATOR",
	[RPRM_GPIO]		= "GPIO",
	[RPRM_SDMA]		= "SDMA",
	[RPRM_IPU]		= "IPU",
	[RPRM_DSP]		= "DSP",
	[RPRM_I2C]		= "I2C",
};

static const char *rname(u32 type) {
	if (type >= RPRM_MAX)
		return "(invalid)";
	return rnames[type];
}

struct rprm_elem {
	struct list_head next;
	u32 src;
	u32 type;
	u32 id;
	void *handle;
	u32 base;
	struct rprm_constraints_data *constraints;
	char res[];
};

struct rprm {
	struct list_head res_list;
	struct idr conn_list;
	struct idr id_list;
	struct mutex lock;
	struct dentry *dbg_dir;
};

struct rprm_auxclk_depot {
	struct clk *aux_clk;
	struct clk *src;
};

struct rprm_regulator_depot {
	struct regulator *reg_p;
	u32 orig_uv;
};

static struct rprm_constraints_data def_data = {
	.frequency	= 0,
	.bandwidth	= -1,
	.latency	= -1,
};

static int _get_rprm_size(u32 type)
{
	switch (type) {
	case RPRM_GPTIMER:
		return sizeof(struct rprm_gpt);
	case RPRM_AUXCLK:
		return sizeof(struct rprm_auxclk);
	case RPRM_REGULATOR:
		return sizeof(struct rprm_regulator);
	case RPRM_GPIO:
		return sizeof(struct rprm_gpio);
	case RPRM_SDMA:
		return sizeof(struct rprm_sdma);
	case RPRM_I2C:
		return sizeof(struct rprm_i2c);
	}
	return 0;
}

static int rprm_gptimer_request(struct rprm_elem *e, struct rprm_gpt *obj)
{
	int ret;
	struct omap_dm_timer *gpt;

	if (obj->id > GPTIMERS_MAX) {
		pr_err("Invalid gptimer %u\n", obj->id);
		return -EINVAL;
	}

	gpt = omap_dm_timer_request_specific(obj->id);
	if (!gpt)
		return -EBUSY;

	ret = omap_dm_timer_set_source(gpt, obj->src_clk);
	if (!ret)
		e->handle = gpt;
	else
		omap_dm_timer_free(gpt);

	return ret;
}

static void rprm_gptimer_release(struct omap_dm_timer *obj)
{
	omap_dm_timer_free(obj);
}

static int rprm_auxclk_request(struct rprm_elem *e, struct rprm_auxclk *obj)
{
	int ret;
	char clk_name[NAME_SIZE];
	char src_clk_name[NAME_SIZE];
	struct rprm_auxclk_depot *acd;
	struct clk *src_parent;

	if ((obj->id < AUX_CLK_MIN) || (obj->id > AUX_CLK_MAX)) {
		pr_err("Invalid aux_clk %d\n", obj->id);
		return -EINVAL;
	}

	/* Create auxclks depot */
	acd = kmalloc(sizeof(*acd), GFP_KERNEL);
	if (!acd)
		return -ENOMEM;

	sprintf(clk_name, "auxclk%d_ck", obj->id);
	acd->aux_clk = clk_get(NULL, clk_name);
	if (!acd->aux_clk) {
		pr_err("%s: unable to get clock %s\n", __func__, clk_name);
		ret = -EIO;
		goto error;
	}

	if (unlikely(acd->aux_clk->usecount))
		pr_warn("There are other users of %d clk\n", obj->id);

	sprintf(src_clk_name, "auxclk%d_src_ck", obj->id);
	acd->src = clk_get(NULL, src_clk_name);
	if (!acd->src) {
		pr_err("%s: unable to get clock %s\n", __func__, src_clk_name);
		ret = -EIO;
		goto error_aux;
	}

	src_parent = clk_get(NULL, clk_src_name[obj->parent_src_clk]);
	if (!src_parent) {
		pr_err("%s: unable to get parent clock %s\n", __func__,
					clk_src_name[obj->parent_src_clk]);
		ret = -EIO;
		goto error_aux_src;
	}

	ret = clk_set_rate(src_parent, (obj->parent_src_clk_rate * MHZ));
	if (ret) {
		pr_err("%s: rate not supported by %s\n", __func__,
					clk_src_name[obj->parent_src_clk]);
		goto error_aux_src_parent;
	}

	ret = clk_set_parent(acd->src, src_parent);
	if (ret) {
		pr_err("%s: unable to set clk %s as parent of aux_clk %s\n",
			__func__,
			clk_src_name[obj->parent_src_clk],
			src_clk_name);
		goto error_aux_src_parent;
	}

	ret = clk_enable(acd->src);
	if (ret) {
		pr_err("%s: error enabling %s\n", __func__, src_clk_name);
		goto error_aux_src_parent;
	}

	ret = clk_set_rate(acd->aux_clk, (obj->clk_rate * MHZ));
	if (ret) {
		pr_err("%s: rate not supported by %s\n", __func__, clk_name);
		goto error_aux_enable;
	}

	ret = clk_enable(acd->aux_clk);
	if (ret) {
		pr_err("%s: error enabling %s\n", __func__, clk_name);
		goto error_aux_enable;
	}
	clk_put(src_parent);

	e->handle = acd;

	return 0;
error_aux_enable:
	clk_disable(acd->src);
error_aux_src_parent:
	clk_put(src_parent);
error_aux_src:
	clk_put(acd->src);
error_aux:
	clk_put(acd->aux_clk);
error:
	kfree(acd);

	return ret;
}

static void rprm_auxclk_release(struct rprm_auxclk_depot *obj)
{
	clk_disable((struct clk *)obj->aux_clk);
	clk_put((struct clk *)obj->aux_clk);
	clk_disable((struct clk *)obj->src);
	clk_put((struct clk *)obj->src);

	kfree(obj);
}

static
int rprm_regulator_request(struct rprm_elem *e, struct rprm_regulator *obj)
{
	int ret;
	struct rprm_regulator_depot *rd;
	char *reg_name;

	if (obj->id > REGULATOR_MAX) {
		pr_err("Invalid regulator %d\n", obj->id);
		return -EINVAL;
	}

	/* Create regulator depot */
	rd = kmalloc(sizeof(*rd), GFP_KERNEL);
	if (!rd)
		return -ENOMEM;

	reg_name = regulator_name[obj->id - 1];
	rd->reg_p = regulator_get_exclusive(NULL, reg_name);
	if (IS_ERR_OR_NULL(rd->reg_p)) {
		pr_err("%s: error providing regulator %s\n", __func__, reg_name);
		ret = -EINVAL;
		goto error;
	}

	rd->orig_uv = regulator_get_voltage(rd->reg_p);

	ret = regulator_set_voltage(rd->reg_p, obj->min_uv, obj->max_uv);
	if (ret) {
		pr_err("%s: error setting %s voltage\n", __func__, reg_name);
		goto error_reg;
	}

	ret = regulator_enable(rd->reg_p);
	if (ret) {
		pr_err("%s: error enabling %s ldo\n", __func__, reg_name);
		goto error_reg;
	}

	e->handle = rd;

	return 0;

error_reg:
	regulator_put(rd->reg_p);
error:
	kfree(rd);

	return ret;
}

static void rprm_regulator_release(struct rprm_regulator_depot *obj)
{
	int ret;

	ret = regulator_disable(obj->reg_p);
	if (ret) {
		pr_err("%s: error disabling ldo\n", __func__);
		return;
	}

	/* Restore orginal voltage */
	ret = regulator_set_voltage(obj->reg_p, obj->orig_uv, obj->orig_uv);
	if (ret) {
		pr_err("%s: error restoring voltage\n", __func__);
		return;
	}

	regulator_put(obj->reg_p);
	kfree(obj);
}

static int rprm_gpio_request(struct rprm_elem *e, struct rprm_gpio *obj)
{
	int ret;
	struct rprm_gpio *gd;

	/* Create gpio depot */
	gd = kmalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd)
		return -ENOMEM;

	ret = gpio_request(obj->id , "rpmsg_resmgr");
	if (ret) {
		pr_err("%s: error providing gpio %d\n", __func__, obj->id);
		return ret;
	}

	e->handle = memcpy(gd, obj, sizeof(*obj));

	return ret;
}

static void rprm_gpio_release(struct rprm_gpio *obj)
{
	gpio_free(obj->id);
	kfree(obj);
}

static int rprm_sdma_request(struct rprm_elem *e, struct rprm_sdma *obj)
{
	int ret;
	int sdma;
	int i;
	struct rprm_sdma *sd;

	/* Create sdma depot */
	sd = kmalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	if (obj->num_chs > MAX_NUM_SDMA_CHANNELS) {
		pr_err("Not able to provide %u channels\n", obj->num_chs);
		return -EINVAL;
	}

	for (i = 0; i < obj->num_chs; i++) {
		ret = omap_request_dma(0, "rpmsg_resmgr", NULL, NULL, &sdma);
		if (ret) {
			pr_err("Error providing sdma channel %d\n", ret);
			goto err;
		}
		obj->channels[i] = sdma;
		pr_debug("Providing sdma ch %d\n", sdma);
	}

	e->handle = memcpy(sd, obj, sizeof(*obj));

	return 0;
err:
	while (i--)
		omap_free_dma(obj->channels[i]);
	kfree(sd);
	return ret;
}

static void rprm_sdma_release(struct rprm_sdma *obj)
{
	int i = obj->num_chs;

	while (i--) {
		omap_free_dma(obj->channels[i]);
		pr_debug("Releasing sdma ch %d\n", obj->channels[i]);
	}
	kfree(obj);
}

static int rprm_i2c_request(struct rprm_elem *e, struct rprm_i2c *obj)
{
	struct device *i2c_dev;
	struct i2c_adapter *adapter;
	char i2c_name[NAME_SIZE];
	int ret = -EINVAL;

	sprintf(i2c_name, "i2c%d", obj->id);
	i2c_dev = omap_hwmod_name_get_dev(i2c_name);
	if (IS_ERR_OR_NULL(i2c_dev)) {
		pr_err("%s: unable to lookup %s\n", __func__, i2c_name);
		return ret;
	}

	adapter = i2c_get_adapter(obj->id);
	if (!adapter) {
		pr_err("%s: could not get i2c%d adapter\n", __func__, obj->id);
		return -EINVAL;
	}
	i2c_detect_ext_master(adapter);
	i2c_put_adapter(adapter);

	ret = pm_runtime_get_sync(i2c_dev);
	/*
	 * pm_runtime_get_sync can return 1 in case it is already active,
	 * change it to 0 to indicate success.
	 */
	ret -= ret == 1;
	if (!ret)
		e->handle = i2c_dev;
	else
		dev_warn(i2c_dev, "%s: failed get sync %d\n", __func__, ret);

	return ret;
}

static int rprm_i2c_release(struct device *i2c_dev)
{
	int ret = -EINVAL;

	if (IS_ERR_OR_NULL(i2c_dev)) {
		pr_err("%s: invalid device passed\n", __func__);
		return ret;
	}

	ret = pm_runtime_put_sync(i2c_dev);
	if (ret)
		dev_warn(i2c_dev, "%s: failed put sync %d\n", __func__, ret);

	return ret;

}

static const char *_get_rpres_name(int type)
{
	switch (type) {
	case RPRM_IVAHD:
		return "rpres_iva";
	case RPRM_IVASEQ0:
		return "rpres_iva_seq0";
	case RPRM_IVASEQ1:
		return "rpres_iva_seq1";
	case RPRM_ISS:
		return "rpres_iss";
	case RPRM_FDIF:
		return "rpres_fdif";
	case RPRM_SL2IF:
		return "rpres_sl2if";
	}
	return "";
}

static int _rpres_set_constraints(struct rprm_elem *e, u32 type, long val)
{
	switch (type) {
	case RPRM_SCALE:
		return rpres_set_constraints(e->handle,
					     RPRES_CONSTRAINT_SCALE,
					     val);
	case RPRM_LATENCY:
		return rpres_set_constraints(e->handle,
					     RPRES_CONSTRAINT_LATENCY,
					     val);
	case RPRM_BANDWIDTH:
		return rpres_set_constraints(e->handle,
					     RPRES_CONSTRAINT_BANDWIDTH,
					     val);
	}
	pr_err("Invalid constraint\n");
	return -EINVAL;
}

static int _rproc_set_constraints(struct rprm_elem *e, u32 type, long val)
{
	switch (type) {
	case RPRM_SCALE:
		return rproc_set_constraints(e->handle,
					     RPROC_CONSTRAINT_SCALE,
					     val);
	case RPRM_LATENCY:
		return rproc_set_constraints(e->handle,
					     RPROC_CONSTRAINT_LATENCY,
					     val);
	case RPRM_BANDWIDTH:
		return rproc_set_constraints(e->handle,
					     RPROC_CONSTRAINT_BANDWIDTH,
					     val);
	}
	pr_err("Invalid constraint\n");
	return -EINVAL;
}

static
int _set_constraints(struct rprm_elem *e, struct rprm_constraints_data *c)
{
	int ret = -EINVAL;
	u32 mask = 0;
	int (*_set_constraints_func)(struct rprm_elem *, u32 type, long val);

	switch (e->type) {
	case RPRM_IVAHD:
	case RPRM_ISS:
	case RPRM_FDIF:
		_set_constraints_func = _rpres_set_constraints;
		break;
	case RPRM_IPU:
		_set_constraints_func = _rproc_set_constraints;
		break;
	default:
		return -EINVAL;
	}

	if (c->mask & RPRM_SCALE) {
		ret = _set_constraints_func(e, RPRM_SCALE, c->frequency);
		if (ret)
			goto err;
		mask |= RPRM_SCALE;
		e->constraints->frequency = c->frequency;
	}

	if (c->mask & RPRM_LATENCY) {
		ret = _set_constraints_func(e, RPRM_LATENCY, c->latency);
		if (ret)
			goto err;
		mask |= RPRM_LATENCY;
		e->constraints->latency = c->latency;
	}

	if (c->mask & RPRM_BANDWIDTH) {
		ret = _set_constraints_func(e, RPRM_BANDWIDTH, c->bandwidth);
		if (ret)
			goto err;
		mask |= RPRM_BANDWIDTH;
		e->constraints->bandwidth = c->bandwidth;
	}
err:
	c->mask = mask;
	return ret;
}

static int rprm_set_constraints(struct rprm *rprm, u32 addr, int res_id,
			   void *data, bool set)
{
	int ret = 0;
	struct rprm_elem *e;

	mutex_lock(&rprm->lock);
	if (!idr_find(&rprm->conn_list, addr)) {
		ret = -ENOTCONN;
		goto out;
	}

	e = idr_find(&rprm->id_list, res_id);
	if (!e || e->src != addr) {
		ret = -ENOENT;
		goto out;
	}

	if (!e->constraints) {
		pr_warn("No constraints\n");
		ret = -EINVAL;
		goto out;
	}

	if (set) {
		ret = _set_constraints(e, data);
		if (!ret) {
			e->constraints->mask |=
				((struct rprm_constraints_data *)data)->mask;
			goto out;
		}
	}
	def_data.mask = ((struct rprm_constraints_data *)data)->mask;
	if (def_data.mask) {
		_set_constraints(e, &def_data);
		e->constraints->mask &=
				~((struct rprm_constraints_data *)data)->mask;
	}
out:
	mutex_unlock(&rprm->lock);
	return ret;
}


static int rprm_rpres_request(struct rprm_elem *e, int type)
{
	const char *res_name = _get_rpres_name(type);
	struct rpres *res;

	e->constraints = kzalloc(sizeof(*(e->constraints)), GFP_KERNEL);
	if (!(e->constraints))
		return -ENOMEM;

	res = rpres_get(res_name);

	if (IS_ERR(res)) {
		pr_err("%s: error requesting %s\n", __func__, res_name);
		kfree(e->constraints);
		return PTR_ERR(res);
	}
	e->handle = res;

	return 0;
}

static void rprm_rpres_release(struct rpres *res)
{
	rpres_put(res);
}

static int rprm_rproc_request(struct rprm_elem *e, char *name)
{
	struct rproc *rp;

	e->constraints = kzalloc(sizeof(*(e->constraints)), GFP_KERNEL);
	if (!(e->constraints))
		return -ENOMEM;

	rp = rproc_get(name);
	if (IS_ERR(rp)) {
		pr_debug("Error requesting %s\n", name);
		kfree(e->constraints);
		return PTR_ERR(rp);
	}
	e->handle = rp;

	return 0;
}

static void rprm_rproc_release(struct rproc *rp)
{
	rproc_put(rp);
}

static int _resource_free(struct rprm_elem *e)
{
	int ret = 0;
	if (e->constraints && e->constraints->mask) {
		def_data.mask = e->constraints->mask;
		_set_constraints(e, &def_data);
	}
	kfree(e->constraints);

	switch (e->type) {
	case RPRM_GPTIMER:
		rprm_gptimer_release(e->handle);
		break;
	case RPRM_IVAHD:
	case RPRM_IVASEQ0:
	case RPRM_IVASEQ1:
	case RPRM_ISS:
	case RPRM_SL2IF:
	case RPRM_FDIF:
		rprm_rpres_release(e->handle);
		break;
	case RPRM_IPU:
	case RPRM_DSP:
		rprm_rproc_release(e->handle);
		break;
	case RPRM_AUXCLK:
		rprm_auxclk_release(e->handle);
		break;
	case RPRM_I2C:
		ret = rprm_i2c_release(e->handle);
		break;
	case RPRM_REGULATOR:
		rprm_regulator_release(e->handle);
		break;
	case RPRM_GPIO:
		rprm_gpio_release(e->handle);
		break;
	case RPRM_SDMA:
		rprm_sdma_release(e->handle);
		break;
	case RPRM_L3BUS:
		/* ignore silently */
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int rprm_resource_free(struct rprm *rprm, u32 addr, int res_id)
{
	int ret = 0;
	struct rprm_elem *e;

	mutex_lock(&rprm->lock);
	if (!idr_find(&rprm->conn_list, addr)) {
		ret = -ENOTCONN;
		goto out;
	}

	e = idr_find(&rprm->id_list, res_id);
	if (!e || e->src != addr) {
		ret = -ENOENT;
		goto out;
	}
	idr_remove(&rprm->id_list, res_id);
	list_del(&e->next);
out:
	mutex_unlock(&rprm->lock);

	if (!ret) {
		ret = _resource_free(e);
		kfree(e);
	}

	return ret;
}

static int _resource_alloc(struct rprm_elem *e, int type, void *data)
{
	int ret = 0;

	switch (type) {
	case RPRM_GPTIMER:
		ret = rprm_gptimer_request(e, data);
		break;
	case RPRM_IVAHD:
	case RPRM_IVASEQ0:
	case RPRM_IVASEQ1:
	case RPRM_ISS:
	case RPRM_SL2IF:
	case RPRM_FDIF:
		ret = rprm_rpres_request(e, type);
		break;
	case RPRM_IPU:
		ret = rprm_rproc_request(e, "ipu");
		break;
	case RPRM_DSP:
		ret = rprm_rproc_request(e, "dsp");
		break;
	case RPRM_AUXCLK:
		ret = rprm_auxclk_request(e, data);
		break;
	case RPRM_I2C:
		ret = rprm_i2c_request(e, data);
		break;
	case RPRM_REGULATOR:
		ret = rprm_regulator_request(e, data);
		break;
	case RPRM_GPIO:
		ret = rprm_gpio_request(e, data);
		break;
	case RPRM_SDMA:
		ret = rprm_sdma_request(e, data);
		break;
	case RPRM_L3BUS:
		/* ignore silently; */
		break;
	default:
		pr_err("%s: invalid source %d!\n", __func__, type);
		ret = -EINVAL;
	}

	return ret;
}

static int rprm_resource_alloc(struct rprm *rprm, u32 addr, int *res_id,
				int type, void *data)
{
	struct rprm_elem *e;
	int ret;
	int rlen = _get_rprm_size(type);

	e = kzalloc(sizeof(*e) + rlen, GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	ret = _resource_alloc(e, type, data);
	if (ret) {
		pr_err("%s: request for %d (%s) failed: %d\n", __func__,
				type, rname(type), ret);
		goto err_res_alloc;
	}

	mutex_lock(&rprm->lock);
	if (!idr_find(&rprm->conn_list, addr)) {
		pr_err("%s: addr %d not connected!\n", __func__, addr);
		ret = -ENOTCONN;
		goto err;
	}
	/*
	 * Create a resource id to avoid sending kernel address to the
	 * remote processor.
	 */
	if (!idr_pre_get(&rprm->id_list, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto err;
	}
	ret = idr_get_new(&rprm->id_list, e, res_id);
	if (ret)
		goto err;

	e->type = type;
	e->src = addr;
	e->id = *res_id;
	memcpy(e->res, data, rlen);
	list_add(&e->next, &rprm->res_list);
	mutex_unlock(&rprm->lock);

	return 0;
err:
	mutex_unlock(&rprm->lock);
	_resource_free(e);
err_res_alloc:
	kfree(e);

	return ret;
}

static int rprm_disconnect_client(struct rprm *rprm, u32 addr)
{
	struct rprm_elem *e, *tmp;
	int ret;

	mutex_lock(&rprm->lock);
	if (!idr_find(&rprm->conn_list, addr)) {
		ret = -ENOTCONN;
		goto out;
	}
	list_for_each_entry_safe(e, tmp, &rprm->res_list, next) {
		if (e->src == addr) {
			_resource_free(e);
			idr_remove(&rprm->id_list, e->id);
			list_del(&e->next);
			kfree(e);
		}
	}

	idr_remove(&rprm->conn_list, addr);
out:
	mutex_unlock(&rprm->lock);

	return 0;
}

static int rpmsg_connect_client(struct rprm *rprm, u32 addr)
{
	int ret;
	int tid;

	mutex_lock(&rprm->lock);
	if (idr_find(&rprm->conn_list, addr)) {
		pr_err("Connection already opened\n");
		ret = -EISCONN;
		goto out;
	}
	if (!idr_pre_get(&rprm->conn_list, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto out;
	}
	ret = idr_get_new_above(&rprm->conn_list, &rprm->res_list, addr, &tid);
	BUG_ON(addr != tid);
out:
	mutex_unlock(&rprm->lock);

	return ret;
}

static void rprm_cb(struct rpmsg_channel *rpdev, void *data, int len,
			void *priv, u32 src)
{
	int ret;
	struct device *dev = &rpdev->dev;
	struct rprm *rprm = dev_get_drvdata(dev);
	struct rprm_request *req = data;
	char ack_msg[MAX_MSG];
	struct rprm_ack *ack = (void *)ack_msg;
	int r_sz = 0;

	if (len < sizeof(*req)) {
		dev_err(dev, "Bad message\n");
		return;
	}

	dev_dbg(dev, "resource type %d\n"
		"request type %d\n"
		"res_id %d",
		req->res_type, req->acquire, req->res_id);

	switch (req->acquire) {
	case RPRM_CONNECT:
		ret = rpmsg_connect_client(rprm, src);
		if (ret)
			dev_err(dev, "connection failed! ret %d\n", ret);
		break;
	case RPRM_REQ_ALLOC:
		r_sz = len - sizeof(*req);
		if (r_sz != _get_rprm_size(req->res_type)) {
			r_sz = 0;
			ret = -EINVAL;
			break;
		}
		ret = rprm_resource_alloc(rprm, src, &req->res_id,
					req->res_type, req->data);
		if (ret)
			dev_err(dev, "resource allocation failed! ret %d\n",
				ret);
		break;
	case RPRM_REQ_FREE:
		ret = rprm_resource_free(rprm, src, req->res_id);
		if (ret)
			dev_err(dev, "resource release failed! ret %d\n", ret);
		return;
	case RPRM_DISCONNECT:
		ret = rprm_disconnect_client(rprm, src);
		if (ret)
			dev_err(dev, "disconnection failed ret %d\n", ret);
		return;
	case RPRM_REQ_CONSTRAINTS:
		r_sz = len - sizeof(*req);
		if (r_sz != sizeof(struct rprm_constraints_data)) {
			r_sz = 0;
			ret = -EINVAL;
			break;
		}
		ret = rprm_set_constraints(rprm, src, req->res_id,
						req->data, true);
		if (ret)
			dev_err(dev, "set constraints failed! ret %d\n", ret);
		break;
	case RPRM_REL_CONSTRAINTS:
		ret = rprm_set_constraints(rprm, src, req->res_id,
						req->data, false);
		if (ret)
			dev_err(dev, "rel constraints failed! ret %d\n", ret);
		return;
	default:
		dev_err(dev, "Unknow request\n");
		ret = -EINVAL;
	}

	ack->ret = ret;
	ack->res_type = req->res_type;
	ack->res_id = req->res_id;
	memcpy(ack->data, req->data, r_sz);

	ret = rpmsg_sendto(rpdev, ack, sizeof(*ack) + r_sz, src);
	if (ret)
		dev_err(dev, "rprm ack failed: %d\n", ret);
}

static int _printf_gptimer_args(char *buf, struct rprm_gpt *obj)
{
	return sprintf(buf,
		"Id:%d\n"
		"Source:%d\n",
		obj->id, obj->src_clk);
}

static int _printf_auxclk_args(char *buf, struct rprm_auxclk *obj)
{
	return sprintf(buf,
		"Id:%d\n"
		"Rate:%2d\n"
		"ParentSrc:%d\n"
		"ParentSrcRate:%d\n",
		obj->id, obj->clk_rate, obj->parent_src_clk,
		obj->parent_src_clk_rate);
}

static int _printf_regulator_args(char *buf, struct rprm_regulator *obj)
{
	return sprintf(buf,
		"Id:%d\n"
		"min_uV:%d\n"
		"max_uV:%d\n",
		obj->id, obj->min_uv, obj->max_uv);
}

static int _printf_gpio_args(char *buf, struct rprm_gpio *obj)
{
	return sprintf(buf, "Id:%d\n", obj->id);
}

static int _printf_i2c_args(char *buf, struct rprm_i2c *obj)
{
	return sprintf(buf, "Id:%d\n", obj->id);
}

static int _printf_sdma_args(char *buf, struct rprm_sdma *obj)
{
	int i, ret = 0;
	ret += sprintf(buf, "NumChannels:%d\n", obj->num_chs);
	for (i = 0 ; i < obj->num_chs; i++)
		ret += sprintf(buf + ret, "Channel[%d]:%d\n", i,
			obj->channels[i]);
	return ret;
}

static int _print_res_args(char *buf, struct rprm_elem *e)
{
	void *res = (void *)e->res;

	switch (e->type) {
	case RPRM_GPTIMER:
		return _printf_gptimer_args(buf, res);
	case RPRM_AUXCLK:
		return _printf_auxclk_args(buf, res);
	case RPRM_I2C:
		return _printf_i2c_args(buf, res);
	case RPRM_REGULATOR:
		return _printf_regulator_args(buf, res);
	case RPRM_GPIO:
		return _printf_gpio_args(buf, res);
	case RPRM_SDMA:
		return _printf_sdma_args(buf, res);
	}
	return 0;
}

static int _printf_constraints_args(char *buf, struct rprm_elem *e)
{
	return sprintf(buf,
		"Mask:0x%x\n"
		"Frequency:%ld\n"
		"Latency:%ld\n"
		"Bandwidth:%ld\n",
		e->constraints->mask, e->constraints->frequency,
		e->constraints->latency, e->constraints->bandwidth);
}

static ssize_t rprm_dbg_read(struct file *filp, char __user *userbuf,
			 size_t count, loff_t *ppos)
{
	struct rprm *rprm = filp->private_data;
	struct rprm_elem *e;
	char res[512];
	int total = 0, c, tmp;
	loff_t p = 0, pt;

	list_for_each_entry(e, &rprm->res_list, next) {
		c = sprintf(res,
			"\nResource Name:%s\n"
			"Source address:%d\n",
			rnames[e->type], e->src);

		if (_get_rprm_size(e->type))
			c += _print_res_args(res + c, e);

		if (e->constraints && e->constraints->mask)
			c += _printf_constraints_args(res + c, e);

		p += c;
		if (*ppos >= p)
			continue;
		pt = c - p + *ppos;
		tmp = simple_read_from_buffer(userbuf + total, count, &pt,
						 res, c);
		total += tmp;
		*ppos += tmp;
		if (tmp - c)
			break;
	}

	return total;
}

static int rprm_dbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations rprm_dbg_ops = {
	.read = rprm_dbg_read,
	.open = rprm_dbg_open,
	.llseek	= generic_file_llseek,
};

static int rprm_probe(struct rpmsg_channel *rpdev)
{
	struct rprm *rprm;

	rprm = kmalloc(sizeof(*rprm), GFP_KERNEL);
	if (!rprm)
		return -ENOMEM;

	mutex_init(&rprm->lock);
	INIT_LIST_HEAD(&rprm->res_list);
	idr_init(&rprm->conn_list);
	idr_init(&rprm->id_list);
	dev_set_drvdata(&rpdev->dev, rprm);

	rprm->dbg_dir = debugfs_create_dir(dev_name(&rpdev->dev), rprm_dbg);
	if (!rprm->dbg_dir)
		dev_err(&rpdev->dev, "can't create debugfs dir\n");

	debugfs_create_file("resources", 0400, rprm->dbg_dir, rprm,
							&rprm_dbg_ops);

	return 0;
}

static void __devexit rprm_remove(struct rpmsg_channel *rpdev)
{
	struct rprm *rprm = dev_get_drvdata(&rpdev->dev);
	struct rprm_elem *e, *tmp;

	dev_info(&rpdev->dev, "Enter %s\n", __func__);

	if (rprm->dbg_dir)
		debugfs_remove_recursive(rprm->dbg_dir);

	mutex_lock(&rprm->lock);

	/* clean up remaining resources */
	list_for_each_entry_safe(e, tmp, &rprm->res_list, next) {
		_resource_free(e);
		list_del(&e->next);
		kfree(e);
	}
	idr_remove_all(&rprm->id_list);
	idr_destroy(&rprm->id_list);
	idr_remove_all(&rprm->conn_list);
	idr_destroy(&rprm->conn_list);

	mutex_unlock(&rprm->lock);

	kfree(rprm);
}

static struct rpmsg_device_id rprm_id_table[] = {
	{
		.name	= "rpmsg-resmgr",
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, rprm_id_table);

static struct rpmsg_driver rprm_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rprm_id_table,
	.probe		= rprm_probe,
	.callback	= rprm_cb,
	.remove		= __devexit_p(rprm_remove),
};

static int __init init(void)
{
	int r;

	if (debugfs_initialized()) {
		rprm_dbg = debugfs_create_dir(KBUILD_MODNAME, NULL);
		if (!rprm_dbg)
			pr_err("Error creating rprm debug directory\n");
	}
	r = register_rpmsg_driver(&rprm_driver);
	if (r && rprm_dbg)
		debugfs_remove_recursive(rprm_dbg);

	return r;
}

static void __exit fini(void)
{
	if (rprm_dbg)
		debugfs_remove_recursive(rprm_dbg);
	unregister_rpmsg_driver(&rprm_driver);
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("Remote Processor Resource Manager");
MODULE_LICENSE("GPL v2");
