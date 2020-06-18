/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: junghyun, kim <jhkim@nexell.co.kr>
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
#ifndef _S5PXX18_SOC_MLC_H_
#define _S5PXX18_SOC_MLC_H_

#include "s5pxx18_soc_disp.h"

#define NUMBER_OF_MLC_MODULE 2
#define PHY_BASEADDR_MLC0	0xC0102000
#define PHY_BASEADDR_MLC1	0xC0102400

#define	PHY_BASEADDR_MLC_LIST	\
		{ PHY_BASEADDR_MLC0, PHY_BASEADDR_MLC1 }

struct nx_mlc_register_set {
	u32 mlccontrolt;
	u32 mlcscreensize;
	u32 mlcbgcolor;
	struct {
		u32 mlcleftright;
		u32 mlctopbottom;
		u32 mlcinvalidleftright0;
		u32 mlcinvalidtopbottom0;
		u32 mlcinvalidleftright1;
		u32 mlcinvalidtopbottom1;
		u32 mlccontrol;
		int32_t mlchstride;
		int32_t mlcvstride;
		u32 mlctpcolor;
		u32 mlcinvcolor;
		u32 mlcaddress;
		u32 __reserved0;
	} mlcrgblayer[2];
	struct {
		u32 mlcleftright;
		u32 mlctopbottom;
		u32 mlccontrol;
		u32 mlcvstride;
		u32 mlctpcolor;

		u32 mlcinvcolor;
		u32 mlcaddress;
		u32 mlcaddresscb;
		u32 mlcaddresscr;
		int32_t mlcvstridecb;
		int32_t mlcvstridecr;
		u32 mlchscale;
		u32 mlcvscale;
		u32 mlcluenh;
		u32 mlcchenh[4];
	} mlcvideolayer;
	struct {
		u32 mlcleftright;
		u32 mlctopbottom;
		u32 mlcinvalidleftright0;
		u32 mlcinvalidtopbottom0;
		u32 mlcinvalidleftright1;
		u32 mlcinvalidtopbottom1;
		u32 mlccontrol;
		int32_t mlchstride;
		int32_t mlcvstride;
		u32 mlctpcolor;
		u32 mlcinvcolor;
		u32 mlcaddress;
	} mlcrgblayer2;
	u32 mlcpaletetable2;
	u32 mlcgammacont;
	u32 mlcrgammatablewrite;
	u32 mlcggammatablewrite;
	u32 mlcbgammatablewrite;
	u32 yuvlayergammatable_red;
	u32 yuvlayergammatable_green;
	u32 yuvlayergammatable_blue;

	u32 dimctrl;
	u32 dimlut0;
	u32 dimlut1;
	u32 dimbusyflag;
	u32 dimprdarrr0;
	u32 dimprdarrr1;
	u32 dimram0rddata;
	u32 dimram1rddata;
	u32 __reserved2[(0x3c0 - 0x12c) / 4];
	u32 mlcclkenb;
};

enum nx_mlc_priority {
	nx_mlc_priority_videofirst = 0ul,
	nx_mlc_priority_videosecond = 1ul,
	nx_mlc_priority_videothird = 2ul,
	nx_mlc_priority_videofourth = 3ul
};

enum nx_mlc_rgbfmt {
	nx_mlc_rgbfmt_r5g6b5 = 0x44320000ul,
	nx_mlc_rgbfmt_b5g6r5 = 0xc4320000ul,
	nx_mlc_rgbfmt_x1r5g5b5 = 0x43420000ul,
	nx_mlc_rgbfmt_x1b5g5r5 = 0xc3420000ul,
	nx_mlc_rgbfmt_x4r4g4b4 = 0x42110000ul,
	nx_mlc_rgbfmt_x4b4g4r4 = 0xc2110000ul,
	nx_mlc_rgbfmt_x8r3g3b2 = 0x41200000ul,
	nx_mlc_rgbfmt_x8b3g3r2 = 0xc1200000ul,
	nx_mlc_rgbfmt_a1r5g5b5 = 0x33420000ul,
	nx_mlc_rgbfmt_a1b5g5r5 = 0xb3420000ul,
	nx_mlc_rgbfmt_a4r4g4b4 = 0x22110000ul,
	nx_mlc_rgbfmt_a4b4g4r4 = 0xa2110000ul,
	nx_mlc_rgbfmt_a8r3g3b2 = 0x11200000ul,
	nx_mlc_rgbfmt_a8b3g3r2 = 0x91200000ul,
	nx_mlc_rgbfmt_r8g8b8 = 0x46530000ul,
	nx_mlc_rgbfmt_b8g8r8 = 0xc6530000ul,
	nx_mlc_rgbfmt_x8r8g8b8 = 0x46530000ul,
	nx_mlc_rgbfmt_x8b8g8r8 = 0xc6530000ul,
	nx_mlc_rgbfmt_a8r8g8b8 = 0x06530000ul,
	nx_mlc_rgbfmt_a8b8g8r8 = 0x86530000ul
};

enum nx_mlc_yuvfmt {
	nx_mlc_yuvfmt_420 = 0ul << 16,
	nx_mlc_yuvfmt_422 = 1ul << 16,
	nx_mlc_yuvfmt_444 = 3ul << 16,
	nx_mlc_yuvfmt_yuyv = 2ul << 16,
	nx_mlc_yuvfmt_422_cbcr = 4ul << 16,
	nx_mlc_yuvfmt_420_cbcr = 5ul << 16,
};

#define FMT_VID_YUV_TO_YVU (0x1<<31)

#ifdef __arm
#pragma diag_default 66
#endif

int nx_mlc_initialize(void);
u32 nx_mlc_get_number_of_module(void);
u32 nx_mlc_get_physical_address(u32 module_index);
u32 nx_mlc_get_size_of_register_set(void);
void nx_mlc_set_base_address(u32 module_index, void *base_address);
void *nx_mlc_get_base_address(u32 module_index);
int nx_mlc_open_module(u32 module_index);
int nx_mlc_close_module(u32 module_index);
int nx_mlc_check_busy(u32 module_index);
int nx_mlc_can_power_down(u32 module_index);
void nx_mlc_set_clock_pclk_mode(u32 module_index, enum nx_pclkmode mode);
enum nx_pclkmode nx_mlc_get_clock_pclk_mode(u32 module_index);
void nx_mlc_set_clock_bclk_mode(u32 module_index, enum nx_bclkmode mode);
enum nx_bclkmode nx_mlc_get_clock_bclk_mode(u32 module_index);

void nx_mlc_set_top_power_mode(u32 module_index, int bpower);
int nx_mlc_get_top_power_mode(u32 module_index);
void nx_mlc_set_top_sleep_mode(u32 module_index, int bsleep);
int nx_mlc_get_top_sleep_mode(u32 module_index);
void nx_mlc_set_top_dirty_flag(u32 module_index);
int nx_mlc_get_top_dirty_flag(u32 module_index);
void nx_mlc_set_mlc_enable(u32 module_index, int benb);
int nx_mlc_get_mlc_enable(u32 module_index);
void nx_mlc_set_field_enable(u32 module_index, int benb);
int nx_mlc_get_field_enable(u32 module_index);
void nx_mlc_set_layer_priority(u32 module_index,
				      enum nx_mlc_priority priority);
void nx_mlc_set_screen_size(u32 module_index, u32 width, u32 height);
void nx_mlc_get_screen_size(u32 module_index, u32 *pwidth,
				   u32 *pheight);
void nx_mlc_set_background(u32 module_index, u32 color);

void nx_mlc_set_dirty_flag(u32 module_index, u32 layer);
int nx_mlc_get_dirty_flag(u32 module_index, u32 layer);
void nx_mlc_set_layer_enable(u32 module_index, u32 layer, int benb);
int nx_mlc_get_layer_enable(u32 module_index, u32 layer);
void nx_mlc_set_lock_size(u32 module_index, u32 layer, u32 locksize);
void nx_mlc_set_alpha_blending(u32 module_index, u32 layer, int benb,
				      u32 alpha);
void nx_mlc_set_transparency(u32 module_index, u32 layer, int benb,
				    u32 color);
void nx_mlc_set_color_inversion(u32 module_index, u32 layer, int benb,
				       u32 color);
u32 nx_mlc_get_extended_color(u32 module_index, u32 color,
				     enum nx_mlc_rgbfmt format);
void nx_mlc_set_format_rgb(u32 module_index, u32 layer,
				  enum nx_mlc_rgbfmt format);
void nx_mlc_set_format_yuv(u32 module_index, enum nx_mlc_yuvfmt format);
void nx_mlc_set_position(u32 module_index, u32 layer, int32_t sx,
				int32_t sy, int32_t ex, int32_t ey);
void nx_mlc_set_dither_enable_when_using_gamma(u32 module_index,
						      int benable);
int nx_mlc_get_dither_enable_when_using_gamma(u32 module_index);
void nx_mlc_set_gamma_priority(u32 module_index, int bvideolayer);
int nx_mlc_get_gamma_priority(u32 module_index);

void nx_mlc_set_rgblayer_invalid_position(u32 module_index, u32 layer,
						 u32 region, int32_t sx,
						 int32_t sy, int32_t ex,
						 int32_t ey, int benb);
void nx_mlc_set_rgblayer_stride(u32 module_index, u32 layer,
				       int32_t hstride, int32_t vstride);
void nx_mlc_set_rgblayer_address(u32 module_index, u32 layer, u32 addr);
void nx_mlc_set_rgblayer_gama_table_power_mode(u32 module_index,
						      int bred, int bgreen,
						      int bblue);
void nx_mlc_get_rgblayer_gama_table_power_mode(u32 module_index,
						      int *pbred, int *pbgreen,
						      int *pbblue);
void nx_mlc_set_rgblayer_gama_table_sleep_mode(u32 module_index,
						      int bred, int bgreen,
						      int bblue);
void nx_mlc_get_rgblayer_gama_table_sleep_mode(u32 module_index,
						      int *pbred, int *pbgreen,
						      int *pbblue);
void nx_mlc_set_rgblayer_rgamma_table(u32 module_index, u32 dwaddress,
					     u32 dwdata);
void nx_mlc_set_rgblayer_ggamma_table(u32 module_index, u32 dwaddress,
					     u32 dwdata);
void nx_mlc_set_rgblayer_bgamma_table(u32 module_index, u32 dwaddress,
					     u32 dwdata);
void nx_mlc_set_rgblayer_gamma_enable(u32 module_index, int benable);
int nx_mlc_get_rgblayer_gamma_enable(u32 module_index);

void nx_mlc_set_video_layer_stride(u32 module_index, int32_t lu_stride,
					  int32_t cb_stride, int32_t cr_stride);
void nx_mlc_set_video_layer_address(u32 module_index, u32 lu_addr,
					   u32 cb_addr, u32 cr_addr);
void nx_mlc_set_video_layer_address_yuyv(u32 module_index, u32 addr,
						int32_t stride);
void nx_mlc_set_video_layer_scale_factor(u32 module_index, u32 hscale,
						u32 vscale, int bhlumaenb,
						int bhchromaenb, int bvlumaenb,
						int bvchromaenb);
void nx_mlc_set_video_layer_scale_filter(u32 module_index, int bhlumaenb,
						int bhchromaenb, int bvlumaenb,
						int bvchromaenb);
void nx_mlc_get_video_layer_scale_filter(u32 module_index,
						int *bhlumaenb,
						int *bhchromaenb,
						int *bvlumaenb,
						int *bvchromaenb);
void nx_mlc_set_video_layer_scale(u32 module_index, u32 sw, u32 sh,
					 u32 dw, u32 dh, int bhlumaenb,
					 int bhchromaenb, int bvlumaenb,
					 int bvchromaenb);
void nx_mlc_set_video_layer_luma_enhance(u32 module_index, u32 contrast,
						int32_t brightness);
void nx_mlc_set_video_layer_brightness(u32 module_index, int32_t brightness);
void nx_mlc_set_video_layer_contrast(u32 module_index, int32_t contrast);
void nx_mlc_set_video_layer_chroma_enhance(u32 module_index,
						  u32 quadrant, int32_t cb_a,
						  int32_t cb_b, int32_t cr_a,
						  int32_t cr_b);
void nx_mlc_set_video_layer_hue(u32 module_index,int32_t hue);
void nx_mlc_set_video_layer_saturation(u32 module_index,int32_t saturation);
void nx_mlc_set_video_layer_line_buffer_power_mode(u32 module_index,
							  int benable);
int nx_mlc_get_video_layer_line_buffer_power_mode(u32 module_index);
void nx_mlc_set_video_layer_line_buffer_sleep_mode(u32 module_index,
							  int benable);
int nx_mlc_get_video_layer_line_buffer_sleep_mode(u32 module_index);
void nx_mlc_set_video_layer_gamma_enable(u32 module_index, int benable);
int nx_mlc_get_video_layer_gamma_enable(u32 module_index);

void nx_mlc_set_gamma_table_poweroff(u32 module_index, int enb);

enum mlc_rgbfmt {
	rgbfmt_r5g6b5 = 0,
	rgbfmt_x1r5g5b5 = 1,
	rgbfmt_x4r4g4b4 = 2,
	rgbfmt_x8r3g3b2 = 3,
	rgbfmt_x8l8 = 4,
	rgbfmt_l16 = 5,
	rgbfmt_a1r5g5b5 = 6,
	rgbfmt_a4r4g4b4 = 7,
	rgbfmt_a8r3g3b2 = 8,
	rgbfmt_a8l8 = 9,
	rgbfmt_r8g8b8 = 10,
	rgbfmt_x8r8g8b8 = 11,
	rgbfmt_a8r8g8b8 = 12,
	rgbfmt_g8r8_g8b8 = 13,
	rgbfmt_r8g8_b8g8 = 14,
	rgbfmt_b5g6r5 = 15,
	rgbfmt_x1b5g5r5 = 16,
	rgbfmt_x4b4g4r4 = 17,
	rgbfmt_x8b3g3r2 = 18,
	rgbfmt_a1b5g5r5 = 19,
	rgbfmt_a4b4g4r4 = 20,
	rgbfmt_a8b3g3r2 = 21,
	rgbfmt_b8g8r8 = 22,
	rgbfmt_x8b8g8r8 = 23,
	rgbfmt_a8b8g8r8 = 24,
	rgbfmt_g8b8_g8r8 = 25,
	rgbfmt_b8g8_r8g8 = 26,
	rgbfmt_pataletb = 27
};

enum latyername {
	topmlc = 0,
	rgb0 = 1,
	rgb1 = 2,
	rgb2 = 3,
	video = 4
};

enum srammode {
	poweroff = 0,
	sleepmode = 2,
	run = 3
};

enum locksizesel {
	locksize_4 = 0,
	locksize_8 = 1,
	locksize_16 = 2
};

enum g3daddrchangeallowed {
	prim = 0,
	secon = 1,
	primorsecon = 2,
	primandsecon = 3
};

void nx_mlc_set_rgb0layer_control_parameter(u32 module_index,
						   int layer_enable,
						   int grp3denable,
						   int tp_enable,
						   u32 transparency_color,
						   int inv_enable,
						   u32 inverse_color,
						   int blend_enable,
						   u8 alpha_value,
						   enum mlc_rgbfmt rbgformat,
						   enum locksizesel
						   lock_size_select);

u32 nx_mlc_get_rgbformat(enum mlc_rgbfmt rbgformat);
void nx_mlc_set_rgb1layer_control_parameter(u32 module_index,
						   int layer_enable,
						   int grp3denable,
						   int tp_enable,
						   u32 transparency_color,
						   int inv_enable,
						   u32 inverse_color,
						   int blend_enable,
						   u8 alpha_value,
						   enum mlc_rgbfmt rbgformat,
						   enum locksizesel
						   lock_size_select);

void nx_mlc_set_rgb2layer_control_parameter(u32 module_index,
						   int layer_enable,
						   int grp3denable,
						   int tp_enable,
						   u32 transparency_color,
						   int inv_enable,
						   u32 inverse_color,
						   int blend_enable,
						   u8 alpha_value,
						   enum mlc_rgbfmt rbgformat,
						   enum locksizesel
						   lock_size_select);

void nx_mlc_set_video_layer_control_parameter(u32 module_index,
						     int layer_enable,
						     int tp_enable,
						     u32 transparency_color,
						     int inv_enable,
						     u32 inverse_color,
						     int blend_enable,
						     u8 alpha_value,
						     enum nx_mlc_yuvfmt
						     yuvformat);

void nx_mlc_set_srammode(u32 module_index, enum latyername layer_name,
				enum srammode sram_mode);

void nx_mlc_set_layer_reg_finish(u32 module_index,
					enum latyername layer_name);

void nx_mlc_set_video_layer_coordinate(u32 module_index,
					      int vfilterenable,
					      int hfilterenable,
					      int vfilterenable_c,
					      int hfilterenable_c,
					      u16 video_layer_with,
					      u16 video_layer_height,
					      int16_t left, int16_t right,
					      int16_t top, int16_t bottom);

void nx_mlc_set_video_layer_filter_scale(u32 module_index, u32 hscale,
						u32 vscale);
void nx_mlcsetgammasrammode(u32 module_index, enum srammode sram_mode);
void nx_mlc_set_gamma_control_parameter(u32 module_index,
					       int rgbgammaenb, int yuvgammaenb,
					       int yuvalphaarray,
					       int dither_enb);

void nx_mlc_set_layer_alpha256(u32 module_index, u32 layer, int alpha);
int nx_mlc_is_under_flow(u32 module_index);

struct nx_mlc_gamma_table_parameter {
	u32 r_table[256];
	u32 g_table[256];
	u32 b_table[256];
	u32 ditherenb;
	u32 alphaselect;
	u32 yuvgammaenb;
	u32 rgbgammaenb;
	u32 allgammaenb;
};

void nx_mlc_set_gamma_table(u32 module_index, int enb,
		struct nx_mlc_gamma_table_parameter *p_gammatable);
void nx_mlc_get_rgblayer_stride(u32 module_index, u32 layer,
				       int32_t *hstride, int32_t *vstride);
void nx_mlc_get_rgblayer_address(u32 module_index, u32 layer,
					u32 *phys_address);
void nx_mlc_get_position(u32 module_index, u32 layer, int *left,
				int *top, int *right, int *bottom);
void nx_mlc_get_video_layer_address_yuyv(u32 module_index, u32 *address,
						u32 *stride);
void nx_mlc_get_video_layer_address(u32 module_index, u32 *lu_address,
					   u32 *cb_address, u32 *cr_address);
void nx_mlc_get_video_layer_stride(u32 module_index, u32 *lu_stride,
					  u32 *cb_stride, u32 *cr_stride);
void nx_mlc_get_video_layer_stride(u32 module_index, u32 *lu_stride,
					  u32 *cb_stride, u32 *cr_stride);
void nx_mlc_get_video_position(u32 module_index, int *left, int *top,
				      int *right, int *bottom);
void nx_mlc_set_top_control_parameter(u32 module_index, int field_enable,
				      int mlc_enable, u8 priority, int layer);

#endif
