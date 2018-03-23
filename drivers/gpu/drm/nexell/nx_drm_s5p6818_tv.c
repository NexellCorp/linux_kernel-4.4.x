/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Sungwoo, Park <swpark@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/component.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/nexell_drm.h>

#include <video/of_display_timing.h>

#include "nx_drm_drv.h"

struct tvout_context {
	struct nx_drm_connector connector;
	bool local_timing;
	bool plugged;
	bool enabled;
	int crtc_pipe; /* dpc num */
	unsigned int possible_crtcs_mask; /* dpc enable mask */
};

#define ctx_to_display(c)	((struct nx_drm_display *)(c->connector.display))
#define property_read(n, s, v)	of_property_read_u32(n, s, &v)

/**
 * internal util functions
 */
static int tvout_parse_dt(struct device *dev, struct tvout_context *ctx)
{
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct device_node *node = dev->of_node;
	struct device_node *np;
	struct display_timing timing;
	int ret;

	DRM_INFO("Load TV-OUT property\n");

	property_read(node, "crtc-pipe", ctx->crtc_pipe);
	property_read(node, "crtcs-possible-mask", ctx->possible_crtcs_mask);

	np = of_graph_get_remote_port_parent(node);
	display->panel_node = np;
	if (!np) {
		DRM_INFO("not use remote panel node (%s)\n", node->full_name);

		ret = of_get_display_timing(node, "display-timing", &timing);
		if (ret == 0) {
			videomode_from_timing(&timing, &display->vm);
			ctx->local_timing = true;
		}
	}
	of_node_put(np);

	np = of_find_node_by_name(node, "dp_control");
	if (!np) {
		DRM_ERROR("failed to find panel's control node (%s)\n",
			  node->full_name);
		return -EINVAL;
	}

	nx_drm_display_setup(display, np, display->panel_type);
	return 0;
}

/**
 * callbacks of struct nx_drm_connect_drv_ops
 */
static bool tvout_detect(struct device *dev, struct drm_connector *conn)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	enum nx_panel_type panel_type = nx_panel_get_type(display);

	if (display->panel_node == NULL && ctx->local_timing == false) {
		DRM_ERROR("can't find timing for panel %s\n",
			  nx_panel_get_name(panel_type));
		return false;
	}

	display->is_connected = ctx->plugged;
	return ctx->plugged;
}

static bool tvout_is_connected(struct device *dev)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);

	return ctx->plugged;
}

/**
 * return value
 * count of supported modes
 */
static int tvout_get_modes(struct device *dev, struct drm_connector *conn)
{
	struct drm_display_mode *mode;
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);

	mode = drm_mode_create(conn->dev);
	if (!mode) {
		DRM_ERROR("failed to drm_mode_create\n");
		return 0;
	}

	drm_display_mode_from_videomode(&display->vm, mode);

	drm_mode_set_name(mode);
	drm_mode_probed_add(conn, mode);

	DRM_DEBUG_KMS("exit, (%dx%d, flags=0x%x)\n",
		      mode->hdisplay, mode->vdisplay, mode->flags);
	return 1;
}

static int tvout_valid_mode(struct device *dev, struct drm_display_mode *mode)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);

	drm_display_mode_to_videomode(mode, &display->vm);

	DRM_DEBUG_KMS("ha:%d, hf:%d, hb:%d, hs:%d\n",
		display->vm.hactive, display->vm.hfront_porch,
		display->vm.hback_porch, display->vm.hsync_len);
	DRM_DEBUG_KMS("va:%d, vf:%d, vb:%d, vs:%d\n",
		display->vm.vactive, display->vm.vfront_porch,
		display->vm.vback_porch, display->vm.vsync_len);
	DRM_DEBUG_KMS("flags:0x%x\n", display->vm.flags);

	return MODE_OK;
}

static void tvout_set_mode(struct device *dev, struct drm_display_mode *mode)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	if (ops->set_mode)
		ops->set_mode(display, mode, 0);
}

static void tvout_enable(struct device *dev)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_connector *nx_connector = &ctx->connector;
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	if (nx_connector->suspended && ops->resume)
		ops->resume(display);

	if (ops->prepare)
		ops->prepare(display);

	drm_panel_prepare(display->panel);
	drm_panel_enable(display->panel);

	if (ops->enable)
		ops->enable(display);
}

static void tvout_disable(struct device *dev)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_connector *nx_connector = &ctx->connector;
	struct nx_drm_display *display = ctx_to_display(ctx);
	struct nx_drm_display_ops *ops = display->ops;

	if (nx_connector->suspended && ops->suspend)
		ops->suspend(display);

	drm_panel_unprepare(display->panel);
	drm_panel_disable(display->panel);

	if (ops->unprepare)
		ops->unprepare(display);

	if (ops->disable)
		ops->disable(display);
}

static struct nx_drm_connect_drv_ops tvout_connector_ops = {
	.detect = tvout_detect,
	.is_connected = tvout_is_connected,
	.get_modes = tvout_get_modes,
	.valid_mode = tvout_valid_mode,
	.set_mode = tvout_set_mode,
	.enable = tvout_enable,
	.disable = tvout_disable,
};

/**
 * callbacks of struct component_ops
 */
static int tvout_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct drm_connector *connector = &ctx->connector.connector;
	enum nx_panel_type panel_type = nx_panel_get_type(ctx_to_display(ctx));
	int ret;

	DRM_INFO("Bind %s panel\n", nx_panel_get_name(panel_type));

	ret = nx_drm_connector_attach(drm, connector, ctx->crtc_pipe,
				      ctx->possible_crtcs_mask, panel_type);
	if (ret < 0) {
		dev_err(dev, "failed to nx_drm_connector_attach\n");
		return ret;
	}

	return 0;
}

static void tvout_unbind(struct device *dev, struct device *master, void *data)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct drm_connector *connector = &ctx->connector.connector;

	if (connector)
		nx_drm_connector_detach(connector);
}

static const struct component_ops tvout_comp_ops = {
	.bind = tvout_bind,
	.unbind = tvout_unbind,
};

/**
 * sysfs interface
 */
static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev;
	long enable;
	int ret;

	BUG_ON(!ctx);

	drm_dev = ctx->connector.connector.dev;

	ret = kstrtoul(buf, 10, &enable);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtoul(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (enable == 1 && ctx->plugged == false) {
		ctx->plugged = true;
		drm_helper_hpd_irq_event(drm_dev);
	} else if (enable == 0 && ctx->plugged == true) {
		ctx->plugged = false;
		drm_helper_hpd_irq_event(drm_dev);
	}

	return n;
}

static DEVICE_ATTR_WO(enable);

/**
 * platform_driver interface
 */
static int tvout_probe(struct platform_device *pdev)
{
	struct tvout_context *ctx;
	struct device *dev = &pdev->dev;
	struct nx_drm_display_ops *ops;
	int ret;

	DRM_DEBUG_KMS("enter (%s)\n", dev_name(dev));

	ret = device_create_file(dev, &dev_attr_enable);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file\n", __func__);
		return ret;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_out;
	}

	ctx->connector.dev = dev;
	ctx->connector.ops = &tvout_connector_ops;
	ctx->plugged = false;
	ctx->enabled = false;

	ctx->connector.display =
		nx_drm_display_get(dev, dev->of_node,&ctx->connector.connector,
				   NX_PANEL_TYPE_TV);
	if (!ctx->connector.display) {
		dev_err(dev, "failed to get display for TVOUT\n");
		ret = -ENODEV;
		goto err_out;
	}

	ops = ctx_to_display(ctx)->ops;
	if (ops->open) {
		ret = ops->open(ctx_to_display(ctx), ctx->crtc_pipe);
		if (ret) {
			dev_err(dev, "failed to display_ops open\n");
			goto err_out;
		}
	}

	ret = tvout_parse_dt(dev, ctx);
	if (ret)
		goto err_out;

	dev_set_drvdata(dev, ctx);
	component_add(dev, &tvout_comp_ops);
	return 0;

err_out:
	DRM_ERROR("failed to probe %s\n", dev_name(dev));
	device_remove_file(dev, &dev_attr_enable);
	return ret;
}

static int tvout_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct nx_drm_display_ops *ops;

	if (!ctx)
		return 0;

	component_del(dev, &tvout_comp_ops);

	ops = ctx_to_display(ctx)->ops;
	if (ops->close)
		ops->close(ctx_to_display(ctx), ctx->crtc_pipe);

	nx_drm_display_put(dev, ctx_to_display(ctx));

	device_remove_file(dev, &dev_attr_enable);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tvout_suspend(struct device *dev)
{
	return 0;
}

static int tvout_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops tvout_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(tvout_suspend, tvout_resume)
};

static const struct of_device_id tvout_of_match[] = {
	{
		.compatible = "nexell,s5pxx18-drm-tv",
		.data = (void *)NX_PANEL_TYPE_TV
	},
	{}
};
MODULE_DEVICE_TABLE(of, tvout_of_match);

static struct platform_driver nx_tvout_driver = {
	.probe = tvout_probe,
	.remove = tvout_remove,
	.driver = {
		.name = "nexell,display_drm_tv",
		.owner = THIS_MODULE,
		.of_match_table = tvout_of_match,
		.pm = &tvout_pm,
	},
};

void panel_tv_init(void)
{
	platform_driver_register(&nx_tvout_driver);
}

void panel_tv_exit(void)
{
	platform_driver_unregister(&nx_tvout_driver);
}
