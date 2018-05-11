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
	struct tvout_control_param param;
};

static const char *type_table[TVOUT_TYPE_MAX] = {
	"NTSC-M",
	"NTSC-N",
	"NTSC-4.43",
	"PAL-M",
	"PAL-N",
	"PAL-BGHI",
	"PSEUDO-PAL",
	"PSEUDO-NTSC",
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
	const char *type;
	s32 sval;
	u32 val;
	struct tvout_control_param *param = &ctx->param;

	DRM_INFO("Load TV-OUT property\n");

	property_read(node, "crtc-pipe", ctx->crtc_pipe);
	property_read(node, "crtcs-possible-mask", ctx->possible_crtcs_mask);

	if (of_property_read_string(node, "type", &type) == 0) {
		int i;

		for (i = 0; i < TVOUT_TYPE_MAX; i++) {
			if (!strncmp(type, type_table[i], strlen(type))) {
				param->type = i;
				break;
			}
		}
		if (i == TVOUT_TYPE_MAX) {
			DRM_ERROR("invalid type value: %s, set to NTSC-M\n",
				  type);
			param->type = TVOUT_TYPE_NTSC_M;
		}
	}

	switch (param->type) {
	case TVOUT_TYPE_NTSC_M:
	case TVOUT_TYPE_NTSC_443:
	case TVOUT_TYPE_PAL_M:
	case TVOUT_TYPE_PSEUDO_PAL:
		param->hsw = 32;
		param->hbp = 90;
		param->hfp = 16;
		param->vactive = 480;
		param->vsw = 3;
		param->vbp = 15;
		param->vfp = 4;
		param->hsos = 63;
		param->hsoe = 1715;
		param->vsos = 0;
		param->vsoe = 3;
		break;
	default:
		param->hsw = 42;
		param->hbp = 90;
		param->hfp = 12;
		param->vactive = 576;
		param->vsw = 2;
		param->vbp = 21;
		param->vfp = 3;
		param->hsos = 83;
		param->hsoe = 1727;
		param->vsos = 0;
		param->vsoe = 2;
		break;
	}

	param->pedestal = of_property_read_bool(node, "pedestal");

	if (of_property_read_s32(node, "sch", &sval) == 0)
		param->sch = sval;

	if (of_property_read_s32(node, "hue", &sval) == 0)
		param->hue = sval;

	if (of_property_read_s32(node, "saturation", &sval) == 0)
		param->saturation = sval;

	if (of_property_read_s32(node, "contrast", &sval) == 0)
		param->contrast = sval;

	if (of_property_read_u32(node, "bright", &val) == 0)
		param->bright = val;

	if (of_property_read_u32(node, "fscadj", &val) == 0)
		param->fscadj = val;

	if (of_property_read_u32(node, "ybw", &val) == 0)
		param->ybw = val;

	if (of_property_read_u32(node, "cbw", &val) == 0)
		param->cbw = val;

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

	display->priv = &ctx->param;
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

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);

	BUG_ON(!ctx);
	return sprintf(buf, "%s\n", ctx->plugged ? "1" : "0");
}

static DEVICE_ATTR(enable, 0644, enable_show, enable_store);

static ssize_t type_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	struct nx_drm_display *display = ctx_to_display(ctx);
	int i;

	for (i = 0; i < TVOUT_TYPE_MAX; i++) {
		if (!strncmp(buf, type_table[i], strlen(type_table[i]))) {
			param->type = i;
			break;
		}
	}
	if (i == TVOUT_TYPE_MAX) {
		dev_err(dev, "invalid type value: %s\n", buf);
		dev_err(dev, "valid type is as below\n");
		for (i = 0; i < TVOUT_TYPE_MAX; i++)
			dev_err(dev, "%s\n", type_table[i]);
	} else {
		switch (param->type) {
		case TVOUT_TYPE_NTSC_M:
		case TVOUT_TYPE_NTSC_443:
		case TVOUT_TYPE_PAL_M:
		case TVOUT_TYPE_PSEUDO_PAL:
			param->hsw = 32;
			param->hbp = 90;
			param->hfp = 16;
			param->vactive = 480;
			param->vsw = 3;
			param->vbp = 15;
			param->vfp = 4;
			param->hsos = 63;
			param->hsoe = 1715;
			param->vsos = 0;
			param->vsoe = 3;
			break;
		default:
			param->hsw = 42;
			param->hbp = 90;
			param->hfp = 12;
			param->vactive = 576;
			param->vsw = 2;
			param->vbp = 21;
			param->vfp = 3;
			param->hsos = 83;
			param->hsoe = 1727;
			param->vsos = 0;
			param->vsoe = 2;
			break;
		}
	}
	display->vm.vactive = param->vactive;

	return n;
}

static ssize_t type_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%s\n", type_table[param->type]);
}

static DEVICE_ATTR(type, 0644, type_show, type_store);

static ssize_t sch_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= -128 && val <= 127) {
		param->sch = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_sch(param);
	} else
		dev_err(dev, "%s: -128 ~ 127 is valid value\n", __func__);

	return n;
}

static ssize_t sch_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->sch);
}

static DEVICE_ATTR(sch, 0644, sch_show, sch_store);

static ssize_t hue_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= -128 && val <= 127) {
		param->hue = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_hue(param);
	} else
		dev_err(dev, "%s: -128 ~ 127 is valid value\n", __func__);

	return n;
}

static ssize_t hue_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->hue);
}

static DEVICE_ATTR(hue, 0644, hue_show, hue_store);

static ssize_t saturation_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= -128 && val <= 127) {
		param->saturation = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_saturation(param);
	} else
		dev_err(dev, "%s: -128 ~ 127 is valid value\n", __func__);

	return n;
}

static ssize_t saturation_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->saturation);
}

static DEVICE_ATTR(saturation, 0644, saturation_show, saturation_store);

static ssize_t contrast_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= -128 && val <= 0) {
		param->contrast = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_contrast(param);
	} else
		dev_err(dev, "%s: -128 ~ 0 is valid value\n", __func__);

	return n;
}

static ssize_t contrast_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->contrast);
}

static DEVICE_ATTR(contrast, 0644, contrast_show, contrast_store);

static ssize_t bright_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= 0 && val <= 127) {
		param->bright = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_bright(param);
	} else
		dev_err(dev, "%s: 0 ~ 127 is valid value\n", __func__);

	return n;
}

static ssize_t bright_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->bright);
}

static DEVICE_ATTR(bright, 0644, bright_show, bright_store);

static ssize_t fscadj_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= 0 && val <= 65535) {
		param->bright = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_fscadj(param);
	} else
		dev_err(dev, "%s: 0 ~ 65535 is valid value\n", __func__);

	return n;
}

static ssize_t fscadj_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->fscadj);
}

static DEVICE_ATTR(fscadj, 0644, fscadj_show, fscadj_store);

static ssize_t ybw_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= 0 && val <= 2) {
		param->ybw = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_ybw(param);
	} else
		dev_err(dev, "%s: 0 ~ 2 is valid value\n", __func__);

	return n;
}

static ssize_t ybw_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->ybw);
}

static DEVICE_ATTR(ybw, 0644, ybw_show, ybw_store);

static ssize_t cbw_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= 0 && val <= 2) {
		param->cbw = val;

		if (ctx->plugged && param->ops != NULL)
			param->ops->set_cbw(param);
	} else
		dev_err(dev, "%s: 0 ~ 2 is valid value\n", __func__);

	return n;
}

static ssize_t cbw_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->cbw);
}

static DEVICE_ATTR(cbw, 0644, cbw_show, cbw_store);

static ssize_t pedestal_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t n)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;
	long val;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret) {
		dev_err(dev, "%s: failed to kstrtol(), ret %d\n", __func__,
			ret);
		return 0;
	}

	if (val >= 0 && val <= 1)
		param->pedestal = val;
	else
		dev_err(dev, "%s: 0 ~ 1 is valid value\n", __func__);

	return n;
}

static ssize_t pedestal_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct tvout_context *ctx = dev_get_drvdata(dev);
	struct tvout_control_param *param = &ctx->param;

	return sprintf(buf, "%d\n", param->pedestal);
}

static DEVICE_ATTR(pedestal, 0644, pedestal_show, pedestal_store);

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
		dev_err(dev, "%s: failed to device_create_file for enable\n",
			__func__);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_type);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for type\n",
			__func__);
		goto err_out;
	}

	ret = device_create_file(dev, &dev_attr_sch);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for sch\n",
			__func__);
		goto err_attr_type;
	}

	ret = device_create_file(dev, &dev_attr_hue);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for hue\n",
			__func__);
		goto err_attr_sch;
	}

	ret = device_create_file(dev, &dev_attr_saturation);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for saturation\n",
			__func__);
		goto err_attr_hue;
	}

	ret = device_create_file(dev, &dev_attr_contrast);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for contrast\n",
			__func__);
		goto err_attr_saturation;
	}

	ret = device_create_file(dev, &dev_attr_bright);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for bright\n",
			__func__);
		goto err_attr_contrast;
	}

	ret = device_create_file(dev, &dev_attr_fscadj);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for fscadj\n",
			__func__);
		goto err_attr_bright;
	}

	ret = device_create_file(dev, &dev_attr_ybw);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for ybw\n",
			__func__);
		goto err_attr_fscadj;
	}

	ret = device_create_file(dev, &dev_attr_cbw);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for cbw\n",
			__func__);
		goto err_attr_ybw;
	}

	ret = device_create_file(dev, &dev_attr_pedestal);
	if (ret < 0) {
		dev_err(dev, "%s: failed to device_create_file for pedestal\n",
			__func__);
		goto err_attr_cbw;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto err_attr_pedestal;
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
		goto err_attr_pedestal;
	}

	ops = ctx_to_display(ctx)->ops;
	if (ops->open) {
		ret = ops->open(ctx_to_display(ctx), ctx->crtc_pipe);
		if (ret) {
			dev_err(dev, "failed to display_ops open\n");
			goto err_attr_pedestal;
		}
	}

	ret = tvout_parse_dt(dev, ctx);
	if (ret)
		goto err_out;

	dev_set_drvdata(dev, ctx);
	component_add(dev, &tvout_comp_ops);
	return 0;

err_attr_pedestal:
	device_remove_file(dev, &dev_attr_pedestal);
err_attr_cbw:
	device_remove_file(dev, &dev_attr_cbw);
err_attr_ybw:
	device_remove_file(dev, &dev_attr_ybw);
err_attr_fscadj:
	device_remove_file(dev, &dev_attr_fscadj);
err_attr_bright:
	device_remove_file(dev, &dev_attr_bright);
err_attr_contrast:
	device_remove_file(dev, &dev_attr_contrast);
err_attr_saturation:
	device_remove_file(dev, &dev_attr_saturation);
err_attr_hue:
	device_remove_file(dev, &dev_attr_hue);
err_attr_sch:
	device_remove_file(dev, &dev_attr_sch);
err_attr_type:
	device_remove_file(dev, &dev_attr_type);
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
