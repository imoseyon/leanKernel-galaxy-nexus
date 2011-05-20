/*
 * Samsung s6e8aa0 panel support
 *
 * Copyright 2011 Google, Inc.
 * Author: Erik Gilling <konkers@google.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <plat/display.h>

#include <linux/platform_data/panel-s6e8aa0.h>

/* DSI Command Virtual channel */
#define CMD_VC_CHANNEL 1

#define DRIVER_NAME "s6e8aa0_i2c"
#define DEVICE_NAME "s6e8aa0_i2c"

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h);

static struct omap_video_timings s6e8aa0_timings = {
	.x_res = 720,
	.y_res = 1280,
	.pixel_clock = 80842,
	.hfp = 158,
	.hsw = 2,
	.hbp = 160,
	.vfp = 13,
	.vsw = 1,
	.vbp = 2,
};

struct s6e8aa0_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned long hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long hw_guard_wait;	/* max guard time in jiffies */

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

	bool cabc_broken;
	unsigned cabc_mode;

	bool force_update;
	struct omap_video_timings *timings;

	struct panel_s6e8aa0_data *pdata;
};

const u8 s6e8aa0_init_pre[] = {
	0xF0,
	0x5A,
	0x5A,
};

const u8 s6e8aa0_init_panel[] = {
	0xF8,
	0x25,
	0x34,
	0x00,
	0x00,
	0x00,
	0x8D,
	0x00,
	0x43,
	0x6E,
	0x10,
	0x27,
	0x00,
	0x00,
	0x10,
	0x00,
	0x00,
	0x20,
	0x02,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x02,
	0x08,
	0x08,
	0x23,
	0x23,
	0xC0,
	0xC1,
	0x01,
	0x81,
	0xC1,
	0x00,
	0xC8,
	0xC1,
	0xD3,
	0x01,
};

const u8 s6e8aa0_init_display[] = {
	0xF2,
	0x80,
	0x03,
	0x0D,
};

const u8 s6e8aa0_init_gamma[] = {
	0xFA,
	0x01,
	0x0F,
	0x0F,
	0x0F,
	0xEE,
	0xB4,
	0xEE,
	0xCB,
	0xC2,
	0xC4,
	0xDA,
	0xD7,
	0xD5,
	0xAE,
	0xAF,
	0xA7,
	0xC0,
	0xC1,
	0xBB,
	0x00,
	0x9F,
	0x00,
	0x95,
	0x00,
	0xD4,
};

const u8 s6e8aa0_init_post0[] = {
	0xF6,
	0x00,
	0x02,
	0x00,
};

const u8 s6e8aa0_init_post1[] = {
	0xB6,
	0x0C,
	0x02,
	0x03,
	0x32,
	0xFF,
	0x44,
	0x44,
	0xC0,
	0x00,
};

const u8 s6e8aa0_init_post2[] = {
	0xD9,
	0x14,
	0x40,
	0x0C,
	0xCB,
	0xCE,
	0x6E,
	0xC4,
	0x0F,
	0x40,
	0x40,
	0xCE,
	0x00,
	0x60,
	0x19,
};

static int s6e8aa0_write(struct omap_dss_device *dssdev, u8 val)
{
	return dsi_vc_dcs_write(1, &val, 1);
}

static int s6e8aa0_write_reg(struct omap_dss_device *dssdev, u8 reg, u8 val)
{
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;

	return dsi_vc_dcs_write(1, buf, 2);
}

static int s6e8aa0_write_block(struct omap_dss_device *dssdev, const u8 *data, int len)
{
	// XXX: dsi_vc_dsc_write should take a const u8 *
	int ret;
	msleep(10);  // XxX: why do we have to wait

	ret = dsi_vc_dcs_write(1, (u8 *)data, len);
	msleep(10);  // XxX: why do we have to wait
	return ret;
}

/***********************
*** DUMMY FUNCTIONS ****
***********************/

static int s6e8aa0_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return 0;
}

static u8 s6e8aa0_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int s6e8aa0_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static bool s6e8aa0_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;
}

static void s6e8aa0_get_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void s6e8aa0_set_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int s6e8aa0_check_timings(struct omap_dss_device *dssdev,
			     struct omap_video_timings *timings)
{
	return 0;
}

static void s6e8aa0_get_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->rotate == 0 || s6->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static int s6e8aa0_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static int s6e8aa0_hw_reset(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(s6->pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(s6->pdata->reset_gpio, 1);
	msleep(40);

	return 0;
}

static int s6e8aa0_probe(struct omap_dss_device *dssdev)
{
	int ret = 0;
	struct s6e8aa0_data *s6 = NULL;

	dev_dbg(&dssdev->dev, "s6e8aa0_probe\n");

	if (dssdev->data == NULL) {
		dev_err(&dssdev->dev, "no platform data!\n");
		return -EINVAL;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = s6e8aa0_timings;

	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	s6 = kzalloc(sizeof(*s6), GFP_KERNEL);
	if (!s6)
		return -ENOMEM;

	s6->dssdev = dssdev;
	s6->pdata = dssdev->data;

	ret = gpio_request(s6->pdata->reset_gpio, "s6e8aa0_reset");
	if (ret < 0) {
		dev_err(&dssdev->dev, "gpio_request %d failed!\n", s6->pdata->reset_gpio);
		goto err;
	}
	gpio_direction_output(s6->pdata->reset_gpio, 1);

	mutex_init(&s6->lock);

	atomic_set(&s6->do_update, 0);

	dev_set_drvdata(&dssdev->dev, s6);

	if (cpu_is_omap44xx())
		s6->force_update = true;

	dev_dbg(&dssdev->dev, "s6e8aa0_probe\n");
	return ret;

err:
	kfree(s6);

	return ret;
}

static void s6e8aa0_remove(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	kfree(s6);
}

/**
 * s6e8aa0_config - Configure S6E8AA0
 *
 * Initial configuration for S6E8AA0 configuration registers, PLL...
 */
static void s6e8aa0_config(struct omap_dss_device *dssdev)
{
	s6e8aa0_write_block(dssdev, s6e8aa0_init_pre, ARRAY_SIZE(s6e8aa0_init_pre));

	s6e8aa0_write(dssdev, 0x11);

	s6e8aa0_write_block(dssdev, s6e8aa0_init_panel, ARRAY_SIZE(s6e8aa0_init_panel));
	s6e8aa0_write_block(dssdev, s6e8aa0_init_display, ARRAY_SIZE(s6e8aa0_init_display));
	s6e8aa0_write_block(dssdev, s6e8aa0_init_gamma, ARRAY_SIZE(s6e8aa0_init_gamma));

	s6e8aa0_write_reg(dssdev, 0xF7, 0x01);

	s6e8aa0_write_block(dssdev, s6e8aa0_init_post0, ARRAY_SIZE(s6e8aa0_init_post0));
	s6e8aa0_write_block(dssdev, s6e8aa0_init_post1, ARRAY_SIZE(s6e8aa0_init_post1));
	s6e8aa0_write_block(dssdev, s6e8aa0_init_post2, ARRAY_SIZE(s6e8aa0_init_post1));

	msleep(250); //XXX: find minimum time

	s6e8aa0_write(dssdev, 0x29);
}

static int s6e8aa0_power_on(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int ret = 0;

	if (s6->enabled != 1) {
		if (s6->pdata->set_power)
			s6->pdata->set_power(true);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		/* reset s6e8aa0 bridge */
		s6e8aa0_hw_reset(dssdev);
		msleep(10);

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		/* XXX */
		msleep(100);
		s6e8aa0_config(dssdev);

		dsi_video_mode_enable(dssdev, 0x3E); /* DSI_DT_PXLSTREAM_24BPP_PACKED; */

		s6->enabled = 1;
	}

err:
	return ret;
}

static void s6e8aa0_power_off(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	gpio_set_value(s6->pdata->reset_gpio, 0);
	msleep(10);

	s6->enabled = 0;
	omapdss_dsi_display_disable(dssdev, 0, 0);

	if (s6->pdata->set_power)
		s6->pdata->set_power(false);

}

static int s6e8aa0_start(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&s6->lock);

	dsi_bus_lock();

	r = s6e8aa0_power_on(dssdev);

	dsi_bus_unlock();

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dssdev->manager->enable(dssdev->manager);
	}

	mutex_unlock(&s6->lock);

	return r;
}

static void s6e8aa0_stop(struct omap_dss_device *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&s6->lock);

	dssdev->manager->disable(dssdev->manager);

	dsi_bus_lock();

	s6e8aa0_power_off(dssdev);

	dsi_bus_unlock();

	mutex_unlock(&s6->lock);
}

static void s6e8aa0_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int s6e8aa0_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return s6e8aa0_start(dssdev);
}

static void s6e8aa0_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock();
}

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);
	int r;
	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&s6->lock);

	dsi_bus_lock();

	if (!s6->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	/* We use VC(0) for VideoPort Data and VC(1) for commands */
	r = omap_dsi_update(dssdev, 0, x, y, w, h, s6e8aa0_framedone_cb, dssdev);
	if (r)
		goto err;

	dsi_bus_unlock();
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&s6->lock);
	return 0;
err:
	dsi_bus_unlock();
	mutex_unlock(&s6->lock);
	return r;
}

static int s6e8aa0_sync(struct omap_dss_device *dssdev)
{
	/* TODO? */
	return 0;
}

static int s6e8aa0_set_update_mode(struct omap_dss_device *dssdev,
			       enum omap_dss_update_mode mode)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode s6e8aa0_get_update_mode(struct omap_dss_device
						     *dssdev)
{
	struct s6e8aa0_data *s6 = dev_get_drvdata(&dssdev->dev);

	if (s6->force_update)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

#ifdef CONFIG_PM
static int s6e8aa0_resume(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "resume\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return s6e8aa0_start(dssdev);
}

static int s6e8aa0_suspend(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "suspend\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}
#endif

static struct omap_dss_driver s6e8aa0_driver = {
	.probe = s6e8aa0_probe,
	.remove = s6e8aa0_remove,

	.enable = s6e8aa0_enable,
	.disable = s6e8aa0_disable,
#ifdef CONFIG_PM
	.suspend = s6e8aa0_suspend,
	.resume = s6e8aa0_resume,
#endif

	.set_update_mode = s6e8aa0_set_update_mode,
	.get_update_mode = s6e8aa0_get_update_mode,

	.update = s6e8aa0_update,
	.sync = s6e8aa0_sync,

	.get_resolution = s6e8aa0_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	/* dummy entry start */
	.enable_te = s6e8aa0_enable_te,
	.set_rotate = s6e8aa0_rotate,
	.get_rotate = s6e8aa0_get_rotate,
	.set_mirror = s6e8aa0_mirror,
	.get_mirror = s6e8aa0_get_mirror,
	/* dummy entry end */

	.get_timings = s6e8aa0_get_timings,
	.set_timings = s6e8aa0_set_timings,
	.check_timings = s6e8aa0_check_timings,

	.driver = {
		   .name = "s6e8aa0",
		   .owner = THIS_MODULE,
		   },
};

static int __init s6e8aa0_init(void)
{
	omap_dss_register_driver(&s6e8aa0_driver);;
	return 0;
}

static void __exit s6e8aa0_exit(void)
{
	omap_dss_unregister_driver(&s6e8aa0_driver);
}

module_init(s6e8aa0_init);
module_exit(s6e8aa0_exit);
