/*
 * MIPI-DSI based s6d7aa1881099a LCD panel driver.
 *
 * Copyright (c) 2017 Nexell Co., Ltd
 *
 * Sungwoo Park <swpark@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/backlight.h>

#define PX0701C_WIDTH_MM		107  /* real: 94.2 */
#define PX0701C_HEIGHT_MM		172 /* real: 150.72 */

#define HFP_ALLO         256
#define HBP_ALLO         158
#define HSA_ALLO         5
#define VFP_ALLO         15
#define VBP_ALLO         10
#define VSA_ALLO         4

#define HACTIVE     800
#define VACTIVE     1280

struct s6d7aa {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data supplies[2];
	int reset_gpio;
	int enable_gpio;
	u32 power_on_delay;
	u32 reset_delay;
	u32 init_delay;
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
	bool is_power_on;

	struct backlight_device *bl_dev;
	int error;
};

static inline struct s6d7aa *panel_to_s6d7aa(struct drm_panel *panel)
{
	return container_of(panel, struct s6d7aa, panel);
}

static int __maybe_unused s6d7aa_clear_error(struct s6d7aa *ctx)
{
	int ret = ctx->error;

	ctx->error = 0;
	return ret;
}

#ifndef CONFIG_DRM_CHECK_PRE_INIT
static void _dcs_write(struct s6d7aa *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing dcs seq: %*ph\n", ret,
			(int)len, data);
		ctx->error = ret;
	}
}
#endif

static int __maybe_unused _dcs_read(struct s6d7aa *ctx, u8 cmd, void *data,
				    size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->error < 0)
		return ctx->error;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}
#ifndef CONFIG_DRM_CHECK_PRE_INIT

#define _dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	_dcs_write(ctx, d, ARRAY_SIZE(d));\
})
static void _set_sequence(struct s6d7aa *ctx)
{
	/* SEQ_INIT1_1 */
	_dcs_write_seq_static(ctx, 0xF0,
			0x5A, 0x5A);

	/* SEQ_INIT1_2 */
    _dcs_write_seq_static(ctx, 0xF1,
			0x5A, 0x5A);

	/* SEQ_INIT1_3 */
    _dcs_write_seq_static(ctx, 0xFC,
			0xA5, 0xA5);

	/* SEQ_INIT2_OTP */
    _dcs_write_seq_static(ctx, 0xD0,
			0x00, 0x10);

	/* SEQ_INIT3_RES_SEL */
    _dcs_write_seq_static(ctx, 0xB1,
			0x10);

	/* SEQ_INIT3_DIR_SEL */
    _dcs_write_seq_static(ctx, 0xB2,
			0x14, 0x22, 0x2F, 0x04);

	/* SEQ_INIT3_ASGT0 */
    _dcs_write_seq_static(ctx, 0xB5,
			0x01);

	/* SEQ_INIT3_ASGT1 */
    _dcs_write_seq_static(ctx, 0xEE,
			0x00, 0x00, 0x1F, 0x00, 0x00,
			0x00, 0x1F, 0x00);

	/* SEQ_INIT3_ASGT2 */
    _dcs_write_seq_static(ctx, 0xEF,
			0x56, 0x34, 0x43, 0x65, 0x90,
			0x80, 0x24, 0x80, 0x00, 0x91,
			0x11, 0x11, 0x11);

	/* SEQ_INIT3_ASGP */
    _dcs_write_seq_static(ctx, 0xF7,
			0x04, 0x08, 0x09, 0x0A, 0x0B,
			0x0C, 0x0D, 0x0E, 0x0F, 0x16,
			0x17, 0x10, 0x01, 0x01, 0x01,
			0x01, 0x04, 0x08, 0x09, 0x0A,
			0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
			0x16, 0x17, 0x10, 0x01, 0x01,
			0x01, 0x01);

	/* SEQ_INIT3_PORCH_CONT */
    _dcs_write_seq_static(ctx, 0xF2,
			0x02, 0x08, 0x08, 0x40, 0x10);

	/* SEQ_INIT3_OUTPUT_CONT */
    _dcs_write_seq_static(ctx, 0xF6,
			0x60, 0x25, 0x26, 0x00, 0x00,
			0x00);

	/* SEQ_INIT3_GMMA0 */
    _dcs_write_seq_static(ctx, 0xFA,
			0x04, 0x35, 0x07, 0x0B, 0x12,
			0x0B, 0x10, 0x16, 0x1A, 0x24,
			0x2C, 0x33, 0x3B, 0x3B, 0x33,
			0x34, 0x33);

	/* SEQ_INIT3_GMMA1 */
    _dcs_write_seq_static(ctx, 0xFB,
			0x04, 0x35, 0x07, 0x0B, 0x12,
			0x0B, 0x10, 0x16, 0x1A, 0x24,
			0x2C, 0x33, 0x3B, 0x3B, 0x33,
			0x34, 0x33);

	/* SEQ_INIT3_POWER_CONTR */
    _dcs_write_seq_static(ctx, 0xF3,
			0x01, 0xC4, 0xE0, 0x62, 0xD4,
			0x83, 0x37, 0x3C, 0x24, 0x00);

	/* SEQ_INIT3_POWER_SEQ_CONTR */
    _dcs_write_seq_static(ctx, 0xF4,
			0x00, 0x02, 0x03, 0x26, 0x03,
			0x02, 0x09, 0x00, 0x07, 0x16,
			0x16, 0x03, 0x00, 0x08, 0x08,
			0x03, 0x19, 0x1C, 0x12, 0x1C,
			0x1D, 0x1E, 0x1A, 0x09, 0x01,
			0x04, 0x02, 0x61, 0x74, 0x75,
			0x72, 0x83, 0x80, 0x80, 0xF0);

	/* SEQ_INIT3_VOL_SETTING */
    _dcs_write_seq_static(ctx, 0xB0,
			0x01);

	/* SEQ_INIT3_POWER_SEQ */
    _dcs_write_seq_static(ctx, 0xF5,
			0x2F, 0x2F, 0x5F, 0xAB, 0x98,
			0x52, 0x0F, 0x33, 0x43, 0x04,
			0x59, 0x54, 0x52, 0x05, 0x40,
			0x40, 0x5D, 0x59, 0x40);

	/* SEQ_INIT3_WATCHDOG0 */
    _dcs_write_seq_static(ctx, 0xBC,
			0x01, 0x4E, 0x08);

	/* SEQ_INIT3_WATCHDOG1 */
    _dcs_write_seq_static(ctx, 0xE1,
			0x03, 0x10, 0x1C, 0xA0, 0x10);

	/* SEQ_INIT3_DDI_ANALOG */
    _dcs_write_seq_static(ctx, 0xFD,
			0x16, 0x10, 0x11, 0x20, 0x09);

	/* SEQ_TE */
	_dcs_write_seq_static(ctx, 0x35,
			0x00);

	/* SEQ_SLEEPOUT */
	_dcs_write_seq_static(ctx, 0x11);

	msleep(30);

	/* SEQ_INIT3_BC_CTRL_ENABLE */
    _dcs_write_seq_static(ctx, 0xC3,
			0x40, 0x00, 0x28);

    msleep(150);

	/* SEQ_DISPLAY_ON */
	_dcs_write_seq_static(ctx, 0x29);

    mdelay(200);
}
#endif

static int s6d7aa_power_on(struct s6d7aa *ctx)
{
#ifndef CONFIG_DRM_CHECK_PRE_INIT
	int ret;

	if (ctx->is_power_on)
		return 0;

	gpio_direction_output(ctx->reset_gpio, 1);
	gpio_set_value(ctx->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	msleep(ctx->power_on_delay);
	gpio_set_value(ctx->reset_gpio, 0);
	msleep(ctx->reset_delay);
	gpio_set_value(ctx->reset_gpio, 1);
	msleep(ctx->init_delay);
#endif
	ctx->is_power_on = true;

	return 0;
}

static int s6d7aa_power_off(struct s6d7aa *ctx)
{
	if (!ctx->is_power_on)
		return 0;

	/* gpio_set_value(ctx->reset_gpio, 0); */
	/* usleep_range(5000, 6000); */
        /*  */
	/* regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies); */
	/* ctx->is_power_on = false; */

	return 0;
}

static int s6d7aa_enable(struct drm_panel *panel)
{

	struct s6d7aa *ctx = panel_to_s6d7aa(panel);

	if (ctx->bl_dev) {
		ctx->bl_dev->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->bl_dev);
	}

	if (ctx->enable_gpio > 0) {
		gpio_direction_output(ctx->enable_gpio, 1);
		gpio_set_value(ctx->enable_gpio, 1);
	}

	return 0;
}

static int s6d7aa_disable(struct drm_panel *panel)
{
	struct s6d7aa *ctx = panel_to_s6d7aa(panel);

	if (ctx->bl_dev) {
		ctx->bl_dev->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->bl_dev);
	}

	if (ctx->enable_gpio > 0)
		gpio_set_value(ctx->enable_gpio, 0);

	return 0;
}

static int s6d7aa_prepare(struct drm_panel *panel)
{
	struct s6d7aa *ctx = panel_to_s6d7aa(panel);
	int ret;

	ret = s6d7aa_power_on(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to power on\n");
		return ret;
	}

#ifndef CONFIG_DRM_CHECK_PRE_INIT
	_set_sequence(ctx);
	ret = ctx->error;
	if (ret < 0) {
		dev_err(ctx->dev, "failed to set_sequence\n");
		return ret;
	}
#endif

	return 0;
}

static int s6d7aa_unprepare(struct drm_panel *panel)
{
	struct s6d7aa *ctx = panel_to_s6d7aa(panel);

	return s6d7aa_power_off(ctx);
}

static const struct drm_display_mode default_mode = {
	.clock = (int)(((HACTIVE + HBP_ALLO + HFP_ALLO + HSA_ALLO) *
				(VACTIVE + VBP_ALLO + VFP_ALLO + VSA_ALLO) * 60)/1000),
	.hdisplay = HACTIVE,
	.hsync_start = HACTIVE + HBP_ALLO,	/* hactive + hbackporch */
	.hsync_end = HACTIVE + HBP_ALLO + HSA_ALLO,	/* hsync_start + hsyncwidth */
	.htotal = HACTIVE + HBP_ALLO + HSA_ALLO + HFP_ALLO,	/* hsync_end + hfrontporch */
	.vdisplay = VACTIVE,
	.vsync_start = VACTIVE + VBP_ALLO,	/* vactive + vbackporch */
	.vsync_end = VACTIVE + VBP_ALLO + VSA_ALLO,	/* vsync_start + vsyncwidth */
	.vtotal = VACTIVE + VBP_ALLO + VSA_ALLO + VFP_ALLO,	/* vsync_end + vfrontporch */
	.vrefresh = 60,			/* Hz */
};

/**
 * HACK
 * return value
 * 1: success
 * 0: failure
 */
static int s6d7aa_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct s6d7aa *ctx = panel_to_s6d7aa(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);

	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	drm_mode_set_name(mode);
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs s6d7aa_drm_funcs = {
	.enable = s6d7aa_enable,
	.disable = s6d7aa_disable,
	.prepare = s6d7aa_prepare,
	.unprepare = s6d7aa_unprepare,
	.get_modes = s6d7aa_get_modes,
};

static int s6d7aa_parse_dt(struct s6d7aa *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *np = dev->of_node;

	of_property_read_u32(np, "power-on-delay", &ctx->power_on_delay);
	of_property_read_u32(np, "reset-delay", &ctx->reset_delay);
	of_property_read_u32(np, "init-delay", &ctx->init_delay);

	return 0;
}

static int s6d7aa_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *backlight;
	struct s6d7aa *ctx;
	int ret;

	if (!drm_panel_connected("s6d7aa"))
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(struct s6d7aa), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	ctx->is_power_on = false;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
		| MIPI_DSI_MODE_VIDEO_HFP
		| MIPI_DSI_MODE_VIDEO_HBP
		| MIPI_DSI_MODE_VIDEO_HSA
		| MIPI_DSI_MODE_VSYNC_FLUSH
//		| MIPI_DSI_MODE_VIDEO_SYNC_PULSE
//		| MIPI_DSI_MODE_VIDEO_AUTO_VERT
//		| MIPI_DSI_MODE_VIDEO_BURST
//		| MIPI_DSI_CLOCK_NON_CONTINUOUS
//		| MIPI_DSI_MODE_LPM
		;
/*
    dsi->mode_flags = MIPI_DSI_MODE_VIDEO
//        | MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
//        | MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_VSYNC_FLUSH
		| MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_EOT_PACKET;
*/


//	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = s6d7aa_parse_dt(ctx);
	if (ret)
		return ret;

	/* TODO: mapping real power name */
/*
	ctx->supplies[0].supply = "vdd3";
	ctx->supplies[1].supply = "vci";
*/

    ctx->supplies[0].supply = "vci";
    ctx->supplies[1].supply = "vdd3";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		dev_warn(dev, "failed to get regulators: %d\n", ret);

	ctx->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	if (ctx->reset_gpio < 0) {
		dev_err(dev, "cannot get reset-gpio %d\n", ctx->reset_gpio);
		return -EINVAL;
	}

	ret = devm_gpio_request(dev, ctx->reset_gpio, "reset-gpio");
	if (ret) {
		dev_err(dev, "failed to request reset-gpio\n");
		return ret;
	}

	ctx->enable_gpio = of_get_named_gpio(dev->of_node, "enable-gpio", 0);
	if (ctx->enable_gpio < 0)
		dev_warn(dev, "cannot get enable-gpio %d\n", ctx->enable_gpio);
	else {
		ret = devm_gpio_request(dev, ctx->enable_gpio, "enable-gpio");
		if (ret) {
			dev_err(dev, "failed to request enable-gpio\n");
			return ret;
		}
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->bl_dev = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->bl_dev)
			return -EPROBE_DEFER;
	}

	ctx->width_mm = PX0701C_WIDTH_MM;
	ctx->height_mm = PX0701C_HEIGHT_MM;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &s6d7aa_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0) {
		backlight_device_unregister(ctx->bl_dev);
		return ret;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		backlight_device_unregister(ctx->bl_dev);
		drm_panel_remove(&ctx->panel);
	}
	return ret;
}

static int s6d7aa_remove(struct mipi_dsi_device *dsi)
{
	struct s6d7aa *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	backlight_device_unregister(ctx->bl_dev);
	s6d7aa_power_off(ctx);

	return 0;
}

static void s6d7aa_shutdown(struct mipi_dsi_device *dsi)
{
	struct s6d7aa *ctx = mipi_dsi_get_drvdata(dsi);

	s6d7aa_power_off(ctx);
}

static const struct of_device_id s6d7aa_of_match[] = {
	{ .compatible = "s6d7aa" },
	{ }
};
MODULE_DEVICE_TABLE(of, s6d7aa_of_match);

static struct mipi_dsi_driver s6d7aa_driver = {
	.probe = s6d7aa_probe,
	.remove = s6d7aa_remove,
	.shutdown = s6d7aa_shutdown,
	.driver = {
		.name = "panel-s6d7aa",
		.of_match_table = s6d7aa_of_match,
	},
};

module_mipi_dsi_driver(s6d7aa_driver);

MODULE_AUTHOR("Sungwoo Park <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("MIPI-SDI based s6d7aa series LCD Panel Driver");
MODULE_LICENSE("GPL v2");
