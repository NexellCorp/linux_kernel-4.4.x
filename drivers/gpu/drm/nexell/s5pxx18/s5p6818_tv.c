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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/reset.h>

#include <linux/io.h>

#include <dt-bindings/tieoff/s5p6818-tieoff.h>

#include "s5pxx18_drv.h"
#include "s5pxx18_hdmi.h"
#include "s5pxx18_reg_hdmi.h"

static void __iomem *hdmi_base;
#define	hdmi_write(r, v)	writel(v, hdmi_base + r)
#define	hdmi_read(r)		readl(hdmi_base + r)

static int tvout_ops_open(struct nx_drm_display *display, int pipe)
{
	struct nx_tvout_dev *tvout = display->context;
	struct nx_control_res *res = &tvout->control.res;

	hdmi_base = res->sub_bases[0];
	return 0;
}

static bool wait_for_hdmi_phy_ready(void)
{
	bool is_hdmi_phy_ready = false;

	while (!is_hdmi_phy_ready) {
		if (hdmi_read(HDMI_PHY_STATUS_0) & 0x01)
			is_hdmi_phy_ready = true;
	}

	return is_hdmi_phy_ready;
}

static int hdmi_reset(struct reset_control *rsc[], int num)
{
	int count = (num - 1);	/* skip hdmi phy reset */
	int i;

	pr_debug("%s: resets %d\n", __func__, num);
	if (num <= 0) {
		pr_err("%s: resets num (currently %d) must be bigger than 0\n",
		       __func__, num);
		return -EINVAL;
	}

	for (i = 0; count > i; i++)
		reset_control_assert(rsc[i]);

	mdelay(1);

	for (i = 0; count > i; i++)
		reset_control_deassert(rsc[i]);

	return 0;
}

static int hdmi_clock_on_27MHz(struct nx_control_res *res)
{
	nx_tieoff_set(res->tieoffs[0][0], res->tieoffs[0][1]);
	nx_disp_top_clkgen_set_clock_pclk_mode(hdmi_clkgen, nx_pclkmode_always);

	hdmi_write(HDMI_PHY_REG7C, (0<<7));
	hdmi_write(HDMI_PHY_REG7C, (0<<7));
	hdmi_write(HDMI_PHY_REG04, (0<<4));
	hdmi_write(HDMI_PHY_REG04, (0<<4));
	hdmi_write(HDMI_PHY_REG24, (1<<7));
	hdmi_write(HDMI_PHY_REG24, (1<<7));
	hdmi_write(HDMI_PHY_REG04, 0xD1);
	hdmi_write(HDMI_PHY_REG04, 0xD1);
	hdmi_write(HDMI_PHY_REG08, 0x22);
	hdmi_write(HDMI_PHY_REG08, 0x22);
	hdmi_write(HDMI_PHY_REG0C, 0x51);
	hdmi_write(HDMI_PHY_REG0C, 0x51);
	hdmi_write(HDMI_PHY_REG10, 0x40);
	hdmi_write(HDMI_PHY_REG10, 0x40);
	hdmi_write(HDMI_PHY_REG14, 0x8);
	hdmi_write(HDMI_PHY_REG14, 0x8);
	hdmi_write(HDMI_PHY_REG18, 0xFC);
	hdmi_write(HDMI_PHY_REG18, 0xFC);
	hdmi_write(HDMI_PHY_REG1C, 0xE0);
	hdmi_write(HDMI_PHY_REG1C, 0xE0);
	hdmi_write(HDMI_PHY_REG20, 0x98);
	hdmi_write(HDMI_PHY_REG20, 0x98);
	hdmi_write(HDMI_PHY_REG24, 0xE8);
	hdmi_write(HDMI_PHY_REG24, 0xE8);
	hdmi_write(HDMI_PHY_REG28, 0xCB);
	hdmi_write(HDMI_PHY_REG28, 0xCB);
	hdmi_write(HDMI_PHY_REG2C, 0xD8);
	hdmi_write(HDMI_PHY_REG2C, 0xD8);
	hdmi_write(HDMI_PHY_REG30, 0x45);
	hdmi_write(HDMI_PHY_REG30, 0x45);
	hdmi_write(HDMI_PHY_REG34, 0xA0);
	hdmi_write(HDMI_PHY_REG34, 0xA0);
	hdmi_write(HDMI_PHY_REG38, 0xAC);
	hdmi_write(HDMI_PHY_REG38, 0xAC);
	hdmi_write(HDMI_PHY_REG3C, 0x80);
	hdmi_write(HDMI_PHY_REG3C, 0x80);
	hdmi_write(HDMI_PHY_REG40, 0x6);
	hdmi_write(HDMI_PHY_REG40, 0x6);
	hdmi_write(HDMI_PHY_REG44, 0x80);
	hdmi_write(HDMI_PHY_REG44, 0x80);
	hdmi_write(HDMI_PHY_REG48, 0x9);
	hdmi_write(HDMI_PHY_REG48, 0x9);
	hdmi_write(HDMI_PHY_REG4C, 0x84);
	hdmi_write(HDMI_PHY_REG4C, 0x84);
	hdmi_write(HDMI_PHY_REG50, 0x5);
	hdmi_write(HDMI_PHY_REG50, 0x5);
	hdmi_write(HDMI_PHY_REG54, 0x22);
	hdmi_write(HDMI_PHY_REG54, 0x22);
	hdmi_write(HDMI_PHY_REG58, 0x24);
	hdmi_write(HDMI_PHY_REG58, 0x24);
	hdmi_write(HDMI_PHY_REG5C, 0x86);
	hdmi_write(HDMI_PHY_REG5C, 0x86);
	hdmi_write(HDMI_PHY_REG60, 0x54);
	hdmi_write(HDMI_PHY_REG60, 0x54);
	hdmi_write(HDMI_PHY_REG64, 0xE4);
	hdmi_write(HDMI_PHY_REG64, 0xE4);
	hdmi_write(HDMI_PHY_REG68, 0x24);
	hdmi_write(HDMI_PHY_REG68, 0x24);
	hdmi_write(HDMI_PHY_REG6C, 0x0);
	hdmi_write(HDMI_PHY_REG6C, 0x0);
	hdmi_write(HDMI_PHY_REG70, 0x0);
	hdmi_write(HDMI_PHY_REG70, 0x0);
	hdmi_write(HDMI_PHY_REG74, 0x0);
	hdmi_write(HDMI_PHY_REG74, 0x0);
	hdmi_write(HDMI_PHY_REG78, 0x1);
	hdmi_write(HDMI_PHY_REG78, 0x1);
	hdmi_write(HDMI_PHY_REG7C, 0x80);
	hdmi_write(HDMI_PHY_REG7C, 0x80);
	hdmi_write(HDMI_PHY_REG7C, (1<<7));
	hdmi_write(HDMI_PHY_REG7C, (1<<7));

	if (!wait_for_hdmi_phy_ready())
		return -ETIMEDOUT;

	return 0;
}

static void dac_power_control(u32 enable)
{
	nx_tieoff_set(NX_TIEOFF_Inst_DAC_PD, enable);
}

static void dac_set_full_scale_output_voltage(int val)
{
	nx_tieoff_set(NX_TIEOFF_Inst_DAC_FS, val);
}

static int enable_clock_source(struct nx_drm_display *display, bool enable)
{
	struct nx_tvout_dev *tvout = display->context;
	struct nx_control_res *res = &tvout->control.res;
	struct nx_control_info *ctrl = &tvout->control.ctrl;

	if (ctrl->clk_src_lv0 == 4) {
		int ret = 0;

		ret = hdmi_reset(res->sub_resets, res->num_sub_resets);
		if (ret < 0) {
			pr_err("%s: failed to hdmi_reset\n", __func__);
			return -EINVAL;
		}

		if (enable) {
			ret = hdmi_clock_on_27MHz(res);
			if (ret != 0) {
				pr_err("hdmi 27MHz clock set error!\n");
				return -1;
			}
		}
	}

	return 0;
}

static int tvout_set_color(struct tvout_control_param *param)
{
	nx_dpc_set_video_encoder_color_control(1, param->sch,
					       param->hue,
					       param->saturation,
					       param->contrast,
					       param->bright);
	return 0;
}

static int tvout_set_fscadj(struct tvout_control_param *param)
{
	nx_dpc_set_video_encoder_fscadjust(1, param->fscadj);
	return 0;
}

static int tvout_set_bandwidth(struct tvout_control_param *param)
{
	nx_dpc_set_video_encoder_bandwidth(1, param->ybw, param->cbw);
	return 0;
}

static struct tvout_control_ops ctrl_ops = {
	.set_sch = tvout_set_color,
	.set_hue = tvout_set_color,
	.set_contrast = tvout_set_color,
	.set_saturation = tvout_set_color,
	.set_bright = tvout_set_color,
	.set_fscadj = tvout_set_fscadj,
	.set_ybw = tvout_set_bandwidth,
	.set_cbw = tvout_set_bandwidth,
};

static int tvout_ops_enable(struct nx_drm_display *display)
{
	struct nx_tvout_dev *tvout = display->context;
	struct nx_control_info *ctrl = &tvout->control.ctrl;
	int module = tvout->control.module;
	unsigned int out_format = ctrl->out_format;
	struct tvout_control_param *param =
		(struct tvout_control_param *)display->priv;

	param->ops = &ctrl_ops;

	DRM_DEBUG_KMS("[TVOUT] type %d\n", param->type);
	DRM_DEBUG_KMS("[TVOUT] sch %d\n", param->sch);
	DRM_DEBUG_KMS("[TVOUT] hue %d\n", param->hue);
	DRM_DEBUG_KMS("[TVOUT] saturation %d\n", param->saturation);
	DRM_DEBUG_KMS("[TVOUT] contrast %d\n", param->contrast);
	DRM_DEBUG_KMS("[TVOUT] bright %d\n", param->bright);
	DRM_DEBUG_KMS("[TVOUT] fscadj %d\n", param->fscadj);
	DRM_DEBUG_KMS("[TVOUT] ybw %d\n", param->ybw);
	DRM_DEBUG_KMS("[TVOUT] cbw %d\n", param->cbw);
	DRM_DEBUG_KMS("[TVOUT] pedestal %d\n", param->pedestal);

	enable_clock_source(display, true);

	nx_dpc_set_clock_divisor_enable(module, false);
	nx_dpc_set_clock_out_enb(module, 0, false);
	nx_dpc_set_clock_out_enb(module, 1, false);
	nx_dpc_set_clock_divisor(module, 0, ctrl->clk_div_lv0);
	nx_dpc_set_clock_source(module, 0, ctrl->clk_src_lv0);
	nx_dpc_set_clock_out_inv(module, 0, ctrl->clk_inv_lv0);
	nx_dpc_set_clock_divisor(module, 1, ctrl->clk_div_lv1);
	nx_dpc_set_clock_source(module, 1, ctrl->clk_src_lv1);
	nx_dpc_set_clock_out_enb(module, 1, true);

	nx_dpc_set_mode(module, out_format, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0);
	nx_dpc_set_hsync(module, 720, param->hsw, param->hfp, param->hbp, 0);
	nx_dpc_set_vsync(module, param->vactive/2, param->vsw, param->vfp,
			 param->vbp, 0, param->vactive/2, param->vsw,
			 param->vfp, param->vbp);
	nx_dpc_set_luma_gain(module, 30);
	nx_dpc_set_vsync_offset(module, 0, 0, 0, 0);
	nx_dpc_set_delay(module, 0, 12, 12, 12);
	nx_dpc_set_dither(module, 0, 0, 0);

	nx_mlc_set_top_control_parameter(module, 1, 1, 1, 0);
	nx_mlc_set_screen_size(module, 720, param->vactive);
	nx_mlc_set_srammode(module, topmlc, sleepmode);
	nx_mlc_set_srammode(module, topmlc, run);
	nx_mlc_set_background(module, 0xff0000);
	nx_mlc_set_rgb0layer_control_parameter(module, 1, 0,
					       0, 0x0,
					       0, 0x0,
					       0, 0x0,
					       rgbfmt_a8b8g8r8,
					       locksize_16);
	nx_mlc_set_layer_reg_finish(module, topmlc);
	nx_mlc_set_layer_reg_finish(module, rgb0);

	nx_dpc_set_reg_flush(module);
	nx_dpc_set_dpc_enable(module, 1);
	nx_dpc_set_clock_divisor_enable(module, 1);

	nx_dpc_set_video_encoder_power_down(module, false);
	nx_dpc_set_encenable(module, true);
	nx_dpc_set_encoder_dacpower_enable(module, 0x30);

	mdelay(33);

	nx_dpc_set_video_encoder_mode(module, param->type, param->pedestal);
	nx_dpc_set_video_encoder_fscadjust(module, param->fscadj);
	nx_dpc_set_video_encoder_bandwidth(module, param->ybw, param->cbw);
	nx_dpc_set_video_encoder_color_control(module,
					       param->sch,
					       param->hue,
					       param->saturation,
					       param->contrast,
					       param->bright);
	nx_dpc_set_video_encoder_timing(module, param->hsos, param->hsoe,
					param->vsos, param->vsoe);
	nx_dpc_set_encoder_shcphase_control(module, 0x3f);
	nx_dpc_set_encoder_timing_config_reg(module, 7);
	nx_dpc_set_encoder_dacoutput_select(module, 1, 2, 4, 5, 0, 0);

	dac_set_full_scale_output_voltage(0); /* Ratio 60/60, 100% */
	dac_power_control(1);

	return 0;
}

static int tvout_ops_disable(struct nx_drm_display *display)
{
	struct nx_tvout_dev *tvout = display->context;
	int module = tvout->control.module;

	nx_dpc_set_dpc_enable(module, 0);
	nx_dpc_set_clock_divisor_enable(module, 0);
	enable_clock_source(display, false);
	dac_power_control(0);

	return 0;
}

static int tvout_ops_set_mode(struct nx_drm_display *display,
			struct drm_display_mode *mode, unsigned int flags)
{
	nx_display_mode_to_sync(mode, display);
	return 0;
}

static int tvout_ops_resume(struct nx_drm_display *display)
{
	nx_display_resume_resource(display);
	return 0;
}

static struct nx_drm_display_ops tvout_ops = {
	.open = tvout_ops_open,
	.enable = tvout_ops_enable,
	.disable = tvout_ops_disable,
	.set_mode = tvout_ops_set_mode,
	.resume = tvout_ops_resume,
};

void *nx_drm_display_tvout_get(struct device *dev,
			struct device_node *node,
			struct nx_drm_display *display)
{
	struct nx_tvout_dev *tvout;

	tvout = kzalloc(sizeof(*tvout), GFP_KERNEL);
	if (!tvout)
		return NULL;

	display->context = tvout;
	display->ops = &tvout_ops;

	return &tvout->control;
}
