/*
 * Copyright (C) 2017  Nexell Co., Ltd.
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
#ifndef __NXS_HDMI_PHY_H__
#define __NXS_HDMI_PHY_H__

#define PHY_CONFIG_SIZE		28

struct hdmi_phy_conf {
	int pixel_clock;
	const u8 *pixel08;
	const u8 *pixel10;
};

static const u8 hdmi2_phy_25200_8[PHY_CONFIG_SIZE] = {
	0xB1, 0x3F, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x7A, 0x02, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_25200_10[PHY_CONFIG_SIZE] = {
	0xF1, 0x69, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x62, 0x02, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_27000_8[PHY_CONFIG_SIZE] = {
	0xB2, 0x87, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x39, 0x02, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_27000_10[PHY_CONFIG_SIZE] = {
	0xF2, 0xE1, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x2D, 0x02, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_27027_8[PHY_CONFIG_SIZE] = {
	0xF1, 0x5A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x72, 0x02, 0x0F, 0x22, 0x02, 0x64, 0x00, 0x92, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_27027_10[PHY_CONFIG_SIZE] = {
	0xF2, 0xE1, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x2D, 0x02, 0x7F, 0x82, 0x05, 0x50, 0x00, 0x92, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_36000_8[PHY_CONFIG_SIZE] = {
	0xB1, 0x5A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x55, 0x02, 0x03, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_40000_8[PHY_CONFIG_SIZE] = {
	0xB1, 0x64, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x4D, 0x02, 0x0B, 0x0C, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_54000_8[PHY_CONFIG_SIZE] = {
	0x71, 0x5A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x72, 0x12, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_54000_10[PHY_CONFIG_SIZE] = {
	0x72, 0xE1, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x2D, 0x12, 0x0B, 0x20, 0x05, 0x01, 0x00, 0x80, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_54054_8[PHY_CONFIG_SIZE] = {
	0x71, 0x5A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x72, 0x12, 0x0F, 0x22, 0x02, 0x64, 0x00, 0x92, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_54054_10[PHY_CONFIG_SIZE] = {
	0x72, 0xE1, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x2D, 0x12, 0x7F, 0x82, 0x05, 0x50, 0x00, 0x92, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_59400_8[PHY_CONFIG_SIZE] = {
	0x71, 0x63, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x67, 0x12, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_59400_10[PHY_CONFIG_SIZE] = {
	0x31, 0x3E, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x53, 0x12, 0x0B, 0x20, 0x05, 0x40, 0x00, 0x90, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_65000_8[PHY_CONFIG_SIZE] = {
	0x71, 0x6C, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x5F, 0x12, 0x0B, 0x22, 0x04, 0x3C, 0x00, 0xA8, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_71000_8[PHY_CONFIG_SIZE] = {
	0x71, 0x76, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x57, 0x12, 0x03, 0x06, 0x02, 0x3B, 0x00, 0xA0, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_74250_8[PHY_CONFIG_SIZE] = {
	0x31, 0x3E, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x53, 0x12, 0x0B, 0x20, 0x05, 0x40, 0x00, 0x90, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_74250_10[PHY_CONFIG_SIZE] = {
	0x32, 0x9B, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x21, 0x12, 0x17, 0x82, 0x01, 0x40, 0x00, 0x94, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40,
};

static const u8 hdmi2_phy_83500_8[PHY_CONFIG_SIZE] = {
	0x51, 0x68, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x4A, 0x12, 0x1B, 0x48, 0x06, 0x4A, 0x00, 0xA5, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_108000_8[PHY_CONFIG_SIZE] = {
	0x31, 0x5A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x72, 0x22, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_108000_10[PHY_CONFIG_SIZE] = {
	0x32, 0xE1, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x2D, 0x22, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_118800_8[PHY_CONFIG_SIZE] = {
	0x31, 0x63, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x67, 0x22, 0x17, 0x34, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_146250_8[PHY_CONFIG_SIZE] = {
	0x31, 0x7A, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x54, 0x22, 0x03, 0x06, 0x02, 0x7A, 0x00, 0x98, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_148500_8[PHY_CONFIG_SIZE] = {
	0x11, 0x3E, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x53, 0x22, 0x0B, 0x20, 0x05, 0x40, 0x00, 0x90, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_148500_10[PHY_CONFIG_SIZE] = {
	0x11, 0x4D, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x42, 0x22, 0x0F, 0x42, 0x01, 0x40, 0x00, 0xAC, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_162000_8[PHY_CONFIG_SIZE] = {
	0x12, 0x87, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x26, 0x22, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_296703_8[PHY_CONFIG_SIZE] = {
	0x11, 0x7C, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x53, 0x32, 0x0B, 0x22, 0x05, 0x5B, 0x00, 0xC4, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40,
};

static const u8 hdmi2_phy_297000_8[PHY_CONFIG_SIZE] = {
	0x11, 0x7C, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x03, 0x53, 0x32, 0x0B, 0x22, 0x05, 0x40, 0x00, 0xA0, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

static const u8 hdmi2_phy_594000_8[PHY_CONFIG_SIZE] = {
	0x01, 0x7C, 0xBC, 0x24, 0x1F, 0x65, 0x2A, 0x14, 0x03, 0x05,
	0x83, 0x29, 0x32, 0x03, 0x06, 0x02, 0x1F, 0x00, 0x8C, 0x80,
	0x00, 0x2F, 0x80, 0x00, 0x00, 0x03, 0x00, 0x40
};

#define	DEFINE_HDMI_CLOCK(pixelclock, p08, p10)	\
	static const struct hdmi_phy_conf hdmi2_phy_##pixelclock = \
	{ .pixel_clock = pixelclock, .pixel08 = p08, .pixel08 = p10, }

DEFINE_HDMI_CLOCK(25200000,  hdmi2_phy_25200_8, hdmi2_phy_25200_10);
DEFINE_HDMI_CLOCK(27000000,  hdmi2_phy_27000_8, hdmi2_phy_27000_10);
DEFINE_HDMI_CLOCK(27027000,  hdmi2_phy_27027_8, hdmi2_phy_27027_10);
DEFINE_HDMI_CLOCK(36000000,  hdmi2_phy_36000_8, NULL);
DEFINE_HDMI_CLOCK(40000000,  hdmi2_phy_40000_8, NULL);
DEFINE_HDMI_CLOCK(54000000,  hdmi2_phy_54000_8, hdmi2_phy_54000_10);
DEFINE_HDMI_CLOCK(54054000,  hdmi2_phy_54054_8, hdmi2_phy_54054_10);
DEFINE_HDMI_CLOCK(59400000,  hdmi2_phy_59400_8, hdmi2_phy_59400_10);
DEFINE_HDMI_CLOCK(65000000,  hdmi2_phy_65000_8, NULL);
DEFINE_HDMI_CLOCK(71000000,  hdmi2_phy_71000_8, NULL);
DEFINE_HDMI_CLOCK(74250000,  hdmi2_phy_74250_8, hdmi2_phy_74250_10);
DEFINE_HDMI_CLOCK(83500000,  hdmi2_phy_83500_8, NULL);
DEFINE_HDMI_CLOCK(108000000, hdmi2_phy_108000_8, hdmi2_phy_108000_10);
DEFINE_HDMI_CLOCK(118800000, hdmi2_phy_118800_8, NULL);
DEFINE_HDMI_CLOCK(146250000, hdmi2_phy_146250_8, NULL);
DEFINE_HDMI_CLOCK(148500000, hdmi2_phy_148500_8, hdmi2_phy_148500_10);
DEFINE_HDMI_CLOCK(162000000, hdmi2_phy_162000_8, NULL);
DEFINE_HDMI_CLOCK(296703000, hdmi2_phy_296703_8, NULL);
DEFINE_HDMI_CLOCK(297000000, hdmi2_phy_297000_8, NULL);
DEFINE_HDMI_CLOCK(594000000, hdmi2_phy_594000_8, NULL);

/*
 * HDMI support pixelclocks
 */
static const struct hdmi_phy_conf *hdmi2_phy_conf[] = {
	&hdmi2_phy_25200000,
	&hdmi2_phy_27000000,
	&hdmi2_phy_27027000,
	&hdmi2_phy_36000000,
	&hdmi2_phy_40000000,
	&hdmi2_phy_54000000,
	&hdmi2_phy_54054000,
	&hdmi2_phy_59400000,
	&hdmi2_phy_65000000,
	&hdmi2_phy_71000000,
	&hdmi2_phy_74250000,
	&hdmi2_phy_83500000,
	&hdmi2_phy_108000000,
	&hdmi2_phy_118800000,
	&hdmi2_phy_146250000,
	&hdmi2_phy_148500000,
	&hdmi2_phy_162000000,
	&hdmi2_phy_296703000,
	&hdmi2_phy_297000000,
	&hdmi2_phy_594000000,
};

struct hdmi_preferred_conf {
	const char *name;
	const struct hdmi_phy_conf *conf;
	struct videomode vm;
	int hz;
};

/*
 * videomode
 * - pixelclock
 * - hactive, hsync_len, hback_porch, hfront_porch
 * - vactive, vsync_len, vback_porch, vfront_porch
 *
 */
#define PHYCONF(_s, _p, _ha, _hs, _hb, _hf, _va, _vs, _vb, _vf, _hz) {	\
	.name = _s, .conf = &hdmi2_phy_##_p,	\
	.vm = {	.pixelclock = _p,	\
	.hactive = _ha, .hsync_len = _hs, .hback_porch = _hb,	\
	.hfront_porch = _hf,	\
	.vactive = _va, .vsync_len = _vs, .vback_porch = _vb,	\
	.vfront_porch = _vf,	\
	}, .hz = _hz, }


/*
 * HDMI preferred sync informations
 * support to no ECID or default HDMI resoultion.
 */
static const struct hdmi_preferred_conf hdmi_preferred_confs[] = {
/* CEA */
PHYCONF("720x480p@60", 27027000, 720, 62, 60, 16, 480, 6, 30, 9, 60),
PHYCONF("720x576p@50", 27000000, 720, 64, 68, 12, 576, 5, 39, 5, 50),
PHYCONF("1280x720p@50", 74250000, 1280, 40, 220, 440, 720, 5, 20, 5, 50),
PHYCONF("1280x720p@60", 74250000, 1280, 40, 220, 110, 720, 5, 20, 5, 60),
PHYCONF("1920x1080p@50", 148500000, 1920, 44, 148, 528, 1080, 5, 36, 4, 50),
PHYCONF("1920x1080p@60", 148500000, 1920, 44, 148, 88, 1080, 5, 36, 4, 60),
/* VESA */
PHYCONF("1024x768p@60", 65000000, 1024, 136, 160, 24, 768, 6, 29, 3, 60),
};

#endif
