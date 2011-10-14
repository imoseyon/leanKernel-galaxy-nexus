/*
 * hdmi_panel.c
 *
 * HDMI library support functions for TI OMAP4 processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors:	Mythri P k <mythripk@ti.com>
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <video/omapdss.h>
#include <linux/switch.h>

#include "dss.h"

#include <video/hdmi_ti_4xxx_ip.h>

static struct {
	struct mutex hdmi_lock;
	struct switch_dev hpd_switch;
} hdmi;

static ssize_t hdmi_deepcolor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	r = omapdss_hdmi_get_deepcolor();
	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}

static ssize_t hdmi_deepcolor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deep_color;
	int r = kstrtoul(buf, 0, &deep_color);
	if (r || deep_color > 2)
		return -EINVAL;
	omapdss_hdmi_set_deepcolor(deep_color);
	return size;
}

static DEVICE_ATTR(deepcolor, S_IRUGO | S_IWUSR, hdmi_deepcolor_show,
							hdmi_deepcolor_store);

static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_panel_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;

	/*
	 * Initialize the timings to 640 * 480
	 * This is only for framebuffer update not for TV timing setting
	 * Setting TV timing will be done only on enable
	 */
	dssdev->panel.timings.x_res = 640;
	dssdev->panel.timings.y_res = 480;

	/* sysfs entry to provide user space control to set deepcolor mode */
	if (device_create_file(&dssdev->dev, &dev_attr_deepcolor))
		DSSERR("failed to create sysfs file\n");

	DSSDBG("hdmi_panel_probe x_res= %d y_res = %d\n",
		dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);
	return 0;
}

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
	device_remove_file(&dssdev->dev, &dev_attr_deepcolor);
}

static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	DSSDBG("ENTER hdmi_panel_enable\n");

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	r = omapdss_hdmi_display_enable(dssdev);
	if (r) {
		DSSERR("failed to power on\n");
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		omapdss_hdmi_display_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&hdmi.hdmi_lock);
}

static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	hdmi_panel_hpd_handler(0);

	omapdss_hdmi_display_disable(dssdev);
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	hdmi_panel_hpd_handler(hdmi_get_current_hpd());

	return r;
}

enum {
	HPD_STATE_OFF,
	HPD_STATE_START,
	HPD_STATE_EDID_TRYLAST = HPD_STATE_START + 5,
};

static struct hpd_worker_data {
	struct delayed_work dwork;
	atomic_t state;
} hpd_work;
static struct workqueue_struct *my_workq;

static void hdmi_hotplug_detect_worker(struct work_struct *work)
{
	struct hpd_worker_data *d = container_of(work, typeof(*d), dwork.work);
	struct omap_dss_device *dssdev = NULL;
	int state = atomic_read(&d->state);

	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}
	dssdev = omap_dss_find_device(NULL, match);

	pr_err("in hpd work %d, state=%d\n", state, dssdev->state);
	if (dssdev == NULL)
		return;

	mutex_lock(&hdmi.hdmi_lock);
	if (state == HPD_STATE_OFF) {
		switch_set_state(&hdmi.hpd_switch, 0);
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			mutex_unlock(&hdmi.hdmi_lock);
			dssdev->driver->disable(dssdev);
			mutex_lock(&hdmi.hdmi_lock);
		}
		goto done;
	} else {
		if (state == HPD_STATE_START) {
			mutex_unlock(&hdmi.hdmi_lock);
			dssdev->driver->enable(dssdev);
			mutex_lock(&hdmi.hdmi_lock);
		} else if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE ||
			   hdmi.hpd_switch.state) {
			/* powered down after enable - skip EDID read */
			goto done;
		} else if (hdmi_read_edid(&dssdev->panel.timings)) {
			/* get monspecs from edid */
			hdmi_get_monspecs(&dssdev->panel.monspecs);
			pr_info("panel size %d by %d\n",
					dssdev->panel.monspecs.max_x,
					dssdev->panel.monspecs.max_y);
			dssdev->panel.width_in_um =
					dssdev->panel.monspecs.max_x * 10000;
			dssdev->panel.height_in_um =
					dssdev->panel.monspecs.max_y * 10000;
			switch_set_state(&hdmi.hpd_switch, 1);
			goto done;
		} else if (state == HPD_STATE_EDID_TRYLAST){
			pr_info("Failed to read EDID after %d times. Giving up.", state - HPD_STATE_START);
			goto done;
		}
		if (atomic_add_unless(&d->state, 1, HPD_STATE_OFF))
			queue_delayed_work(my_workq, &d->dwork, msecs_to_jiffies(60));
	}
done:
	mutex_unlock(&hdmi.hdmi_lock);
}

int hdmi_panel_hpd_handler(int hpd)
{
	__cancel_delayed_work(&hpd_work.dwork);
	atomic_set(&hpd_work.state, hpd ? HPD_STATE_START : HPD_STATE_OFF);
	queue_delayed_work(my_workq, &hpd_work.dwork, msecs_to_jiffies(hpd ? 40 : 30));
	return 0;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	mutex_lock(&hdmi.hdmi_lock);

	*timings = dssdev->panel.timings;

	mutex_unlock(&hdmi.hdmi_lock);
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_set_timings\n");

	mutex_lock(&hdmi.hdmi_lock);

	dssdev->panel.timings = *timings;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		omapdss_hdmi_display_set_timing(dssdev);

	mutex_unlock(&hdmi.hdmi_lock);
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	int r = 0;

	DSSDBG("hdmi_check_timings\n");

	mutex_lock(&hdmi.hdmi_lock);

	r = omapdss_hdmi_display_check_timing(dssdev, timings);
	if (r) {
		DSSERR("Timing cannot be applied\n");
		goto err;
	}
err:
	mutex_unlock(&hdmi.hdmi_lock);
	return r;
}

static int hdmi_get_modedb(struct omap_dss_device *dssdev,
			   struct fb_videomode *modedb, int modedb_len)
{
	struct fb_monspecs *specs = &dssdev->panel.monspecs;
	if (specs->modedb_len < modedb_len)
		modedb_len = specs->modedb_len;
	memcpy(modedb, specs->modedb, sizeof(*modedb) * modedb_len);
	return modedb_len;
}

static struct omap_dss_driver hdmi_driver = {
	.probe		= hdmi_panel_probe,
	.remove		= hdmi_panel_remove,
	.enable		= hdmi_panel_enable,
	.disable	= hdmi_panel_disable,
	.suspend	= hdmi_panel_suspend,
	.resume		= hdmi_panel_resume,
	.get_timings	= hdmi_get_timings,
	.set_timings	= hdmi_set_timings,
	.check_timings	= hdmi_check_timings,
	.get_modedb	= hdmi_get_modedb,
	.set_mode	= omapdss_hdmi_display_set_mode,
	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};

int hdmi_panel_init(void)
{
	mutex_init(&hdmi.hdmi_lock);
	hdmi.hpd_switch.name = "hdmi";
	switch_dev_register(&hdmi.hpd_switch);

	my_workq = create_singlethread_workqueue("hdmi_hotplug");
	INIT_DELAYED_WORK(&hpd_work.dwork, hdmi_hotplug_detect_worker);
	omap_dss_register_driver(&hdmi_driver);

	return 0;
}

void hdmi_panel_exit(void)
{
	destroy_workqueue(my_workq);
	omap_dss_unregister_driver(&hdmi_driver);

	switch_dev_unregister(&hdmi.hpd_switch);
}
