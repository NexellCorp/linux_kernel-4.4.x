/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
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
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "tp2825.h"

/* #define DEBUG_TP2825 */
#ifdef DEBUG_TP2825
#define vmsg(a...)  pr_err(a)
#else
#define vmsg(a...)
#endif

struct nx_resolution {
	uint32_t width;
	uint32_t height;
	uint32_t interval[2];
};

static struct nx_resolution supported_resolutions[] = {	
	{
		.width	= 960,
		.height = 576,
		.interval[0] = 25,
		.interval[1] = 30,
	},
	{
		.width	= 960,
		.height = 480,
		.interval[0] = 25,
		.interval[1] = 30,
	},
	{
		.width	= 1280,
		.height = 720,
		.interval[0] = 25,
		.interval[1] = 30,
	}
};

struct tp2825_state {
	struct media_pad pad;
	struct v4l2_subdev sd;
	bool first;

	struct i2c_client *i2c_client;

	__u32 width;
	__u32 height;

	int old_mode;
	int gpio_reset;
	int out_port;
	int vdelay;
};

struct reg_val {
	uint8_t reg;
	uint8_t val;
};

#define END_MARKER {0xff, 0xff}

static struct tp2825_state _state;
static int mode;
static int revision;

/* PAL HALF -> 960x576i */
static struct reg_val _sensor_init_data_960x576i[] = {
	/* video */
	{0x40, 0x00},
	{0x07, 0xc0},
	{0x0b, 0xc0},
	{0x39, 0x8c},
	{0x4d, 0x03},
	{0x4e, 0x37},
	/* PTZ */
	{0xc8, 0x21},
	{0x7e, 0x01},
	{0xb9, 0x01},
	/* Data Set */
	/* {0x02, 0xcf}, */
	{0x15, 0x13},
	{0x16, 0x68},
	{0x17, 0x80},
	{0x18, 0x17},
	{0x19, 0x20},
	{0x1a, 0x17},
	{0x1c, 0x09},
	{0x1d, 0x48},
	{0x0c, 0x53},
	{0x0d, 0x11},
	{0x20, 0xb0},
	{0x26, 0x02},
	{0x2b, 0x70},
	{0x2d, 0x60},
	{0x2e, 0x5e},
	{0x30, 0x7a},
	{0x31, 0x4a},
	{0x32, 0x4d},
	{0x33, 0xf0},
	{0x35, 0x65},
	{0x39, 0x84},
	END_MARKER
};

/* PAL -> 1920x576i */
static struct reg_val _sensor_init_data_1920x576i[] = {
	/* video */
	{0x40, 0x00},
	{0x07, 0xc0},
	{0x0b, 0xc0},
	{0x39, 0x8c},
	{0x4d, 0x03},
	{0x4e, 0x17},
	/* PTZ */
	{0xc8, 0x21},
	{0x7e, 0x01},
	{0xb9, 0x01},
	/* Data Set */
	{0x02, 0xcf},
	{0x15, 0x13},
	{0x16, 0x68},
	{0x17, 0x80},
	{0x18, 0x17},
	{0x19, 0x20},
	{0x1a, 0x17},
	{0x1c, 0x09},
	{0x1d, 0x48},
	{0x0c, 0x53},
	{0x0d, 0x11},
	{0x20, 0xb0},
	{0x26, 0x02},
	{0x2b, 0x70},
	{0x2d, 0x60},
	{0x2e, 0x5e},
	{0x30, 0x7a},
	{0x31, 0x4a},
	{0x32, 0x4d},
	{0x33, 0xf0},
	{0x35, 0x25},
	{0x39, 0x84},
	END_MARKER
};

/* NTSC -> 960x480i */
static struct reg_val _sensor_init_data_960x480i[] = {
	/* video */
	{0x40, 0x00},
	{0x07, 0xc0},
	{0x0b, 0xc0},
	{0x39, 0x8c},
	{0x4d, 0x03},
	{0x4e, 0x17},
	/* PTZ */
	{0xc8, 0x21},
	{0x7e, 0x01},
	{0xb9, 0x01},
	/* Data Set */
	{0x02, 0xcf},
	{0x15, 0x13},
	{0x16, 0x4e},
	{0x17, 0x80},
	{0x18, 0x13},
	{0x19, 0xf0},
	{0x1a, 0x07},
	{0x1c, 0x09},
	{0x1d, 0x38},
	{0x0c, 0x53},
	{0x0d, 0x10},
	{0x20, 0xa0},
	{0x26, 0x12},
	{0x2b, 0x70},
	{0x2d, 0x68},
	{0x2e, 0x5e},
	{0x30, 0x62},
	{0x31, 0xbb},
	{0x32, 0x96},
	{0x33, 0xc0},
	{0x35, 0x45},
	{0x39, 0x84},
	{0xfa, 0x02},
	END_MARKER
};

/* NTSC -> 1920x480i */
static struct reg_val _sensor_init_data_1920x480i[] = {
	/* video */
	{0x40, 0x00},
	{0x07, 0xc0},
	{0x0b, 0xc0},
	{0x39, 0x8c},
	{0x4d, 0x03},
	{0x4e, 0x17},
	/* PTZ */
	{0xc8, 0x21},
	{0x7e, 0x01},
	{0xb9, 0x01},
	/* Data Set */
	{0x02, 0xcf},
	{0x15, 0x13},
	{0x16, 0x4e},
	{0x17, 0x80},
	{0x18, 0x13},
	{0x19, 0xf0},
	{0x1a, 0x07},
	{0x1c, 0x09},
	{0x1d, 0x38},
	{0x0c, 0x53},
	{0x0d, 0x10},
	{0x20, 0xa0},
	{0x26, 0x12},
	{0x2b, 0x70},
	{0x2d, 0x68},
	{0x2e, 0x5e},
	{0x30, 0x62},
	{0x31, 0xbb},
	{0x32, 0x96},
	{0x33, 0xc0},
	{0x35, 0x25},
	{0x39, 0x84},
	END_MARKER
};

unsigned char tbl_tp2825_1080p25_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x03, 0xD3, 0x80, 0x29, 0x38, 0x48, 0x00, 0x0A, 0x50
};

unsigned char tbl_tp2825_1080p30_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x03, 0xD3, 0x80, 0x29, 0x38, 0x47, 0x00, 0x08, 0x98
};

unsigned char tbl_tp2825_720p25_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x16, 0x00, 0x19, 0xD0, 0x25, 0x00, 0x0F, 0x78
};

unsigned char tbl_tp2825_720p30_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x16, 0x00, 0x19, 0xD0, 0x25, 0x00, 0x0C, 0xE4
};

unsigned char tbl_tp2825_720p50_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x16, 0x00, 0x19, 0xD0, 0x25, 0x00, 0x07, 0xBC
};

unsigned char tbl_tp2825_720p60_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x16, 0x00, 0x19, 0xD0, 0x25, 0x00, 0x06, 0x72
};

unsigned char tbl_tp2825_PAL_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x5f, 0xbc, 0x17, 0x20, 0x17, 0x00, 0x09, 0x48
};

unsigned char tbl_tp2825_NTSC_raster[] = {
	/* Start address 0x15, Size = 9B */
	0x13, 0x5f, 0xbc, 0x13, 0xf0, 0x07, 0x00, 0x09, 0x38
};

/**
 * util functions
 */
#define I2C_RETRY_CNT 3
static unsigned char tp28xx_byte_read(struct i2c_client *client, u8 addr)
{
	int i;
	int ret;

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, addr);
		if (ret < 0)
			pr_err("## %s() fail! reg:0x%x, ret:%d, i:%d\n",
				__func__, addr, ret, i);
		else
			break;
	}

	return ret;
}

static void tp28xx_byte_write(struct i2c_client *client, u8 addr, u8 val)
{
	int i;
	int ret;

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, addr, val);
		if (ret < 0)
			pr_err("## %s() fail! reg:0x%x, val:0x%x, ret:%d, i:%d\n",
				__func__, addr, val, ret, i);
		else
			break;
	}

	return;
}

static void tp2825_write_table(struct i2c_client *client,
	unsigned char addr,
	unsigned char *tbl_ptr,
	unsigned char tbl_cnt)
{
	unsigned char i = 0;

	for (i = 0; i < tbl_cnt; i++)
		tp28xx_byte_write(client, (addr + i), *(tbl_ptr + i));
}

static void tp2825_set_work_mode_1080p25(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_1080p25_raster, 9);
}

static void tp2825_set_work_mode_1080p30(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_1080p30_raster, 9);
}
static void tp2825_set_work_mode_720p25(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_720p25_raster, 9);
}

static void tp2825_set_work_mode_720p30(struct i2c_client *client)
{
	struct tp2825_state *me = &_state;

	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_720p30_raster, 9);
	if (me->vdelay)
		tp28xx_byte_write(client, 0x18, (u8)me->vdelay);

}

static void tp2825_set_work_mode_720p50(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_720p50_raster, 9);
}

static void tp2825_set_work_mode_720p60(struct i2c_client *client)
{
	struct tp2825_state *me = &_state;

	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_720p60_raster, 9);

	if (me->vdelay)
		tp28xx_byte_write(client, 0x18, (u8)me->vdelay);
}

static void tp2825_set_work_mode_PAL(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_PAL_raster, 9);
}

static void tp2825_set_work_mode_NTSC(struct i2c_client *client)
{
	/* Start address 0x15, Size = 9B */
	tp2825_write_table(client, 0x15, tbl_tp2825_NTSC_raster, 9);
}


static void TP2825_NTSC_DataSet(struct i2c_client *client)
{
	unsigned int tmp;

	tp28xx_byte_write(client, 0x0c, 0x43);
	tp28xx_byte_write(client, 0x0d, 0x10);
	tp28xx_byte_write(client, 0x20, 0xa0);
	tp28xx_byte_write(client, 0x26, 0x12);
	tp28xx_byte_write(client, 0x2b, 0x50);
	tp28xx_byte_write(client, 0x2d, 0x68);
	tp28xx_byte_write(client, 0x2e, 0x5e);
	tp28xx_byte_write(client, 0x30, 0x62);
	tp28xx_byte_write(client, 0x31, 0xbb);
	tp28xx_byte_write(client, 0x32, 0x96);
	tp28xx_byte_write(client, 0x33, 0xc0);
	tp28xx_byte_write(client, 0x35, 0x25);
	tp28xx_byte_write(client, 0x39, 0x84);
	tp28xx_byte_write(client, 0x2c, 0x2a);
	tp28xx_byte_write(client, 0x27, 0x2d);
	tp28xx_byte_write(client, 0x28, 0x00);
	tp28xx_byte_write(client, 0x13, 0x00);

	tmp = tp28xx_byte_read(client, 0x14);
	tmp &= 0x9f;
	tp28xx_byte_write(client, 0x14, tmp);
}

static void TP2825_PAL_DataSet(struct i2c_client *client)
{
	unsigned int tmp;

	tp28xx_byte_write(client, 0x0c, 0x53);
	tp28xx_byte_write(client, 0x0d, 0x11);
	tp28xx_byte_write(client, 0x20, 0xb0);
	tp28xx_byte_write(client, 0x26, 0x02);
	tp28xx_byte_write(client, 0x2b, 0x50);
	tp28xx_byte_write(client, 0x2d, 0x60);
	tp28xx_byte_write(client, 0x2e, 0x5e);
	tp28xx_byte_write(client, 0x30, 0x7a);
	tp28xx_byte_write(client, 0x31, 0x4a);
	tp28xx_byte_write(client, 0x32, 0x4d);
	tp28xx_byte_write(client, 0x33, 0xf0);
	tp28xx_byte_write(client, 0x35, 0x25);
	tp28xx_byte_write(client, 0x39, 0x84);
	tp28xx_byte_write(client, 0x2c, 0x2a);
	tp28xx_byte_write(client, 0x27, 0x2d);
	tp28xx_byte_write(client, 0x28, 0x00);
	tp28xx_byte_write(client, 0x13, 0x00);

	tmp = tp28xx_byte_read(client, 0x14);
	tmp &= 0x9f;
	tp28xx_byte_write(client, 0x14, tmp);
}

static void TP2825_V1_DataSet(struct i2c_client *client)
{
	unsigned int tmp;

	tp28xx_byte_write(client, 0x0c, 0x03);
	tp28xx_byte_write(client, 0x0d, 0x10);
	tp28xx_byte_write(client, 0x20, 0x60);
	tp28xx_byte_write(client, 0x26, 0x02);
	tp28xx_byte_write(client, 0x2b, 0x58);
	tp28xx_byte_write(client, 0x2d, 0x30);
	tp28xx_byte_write(client, 0x2e, 0x70);
	tp28xx_byte_write(client, 0x30, 0x48);
	tp28xx_byte_write(client, 0x31, 0xbb);
	tp28xx_byte_write(client, 0x32, 0x2e);
	tp28xx_byte_write(client, 0x33, 0x90);
	tp28xx_byte_write(client, 0x35, 0x05);
	tp28xx_byte_write(client, 0x39, 0x8C);
	tp28xx_byte_write(client, 0x2c, 0x0a);
	tp28xx_byte_write(client, 0x27, 0x2d);
	tp28xx_byte_write(client, 0x28, 0x00);
	tp28xx_byte_write(client, 0x13, 0x00);

	tmp = tp28xx_byte_read(client, 0x14);
	tmp &= 0x9f;
	tp28xx_byte_write(client, 0x14, tmp);
}

static void TP2825_V2_DataSet(struct i2c_client *client)
{
	unsigned int tmp;

	tp28xx_byte_write(client, 0x0c, 0x03);
	tp28xx_byte_write(client, 0x0d, 0x10);
	tp28xx_byte_write(client, 0x20, 0x60);
	tp28xx_byte_write(client, 0x26, 0x02);
	tp28xx_byte_write(client, 0x2b, 0x58);
	tp28xx_byte_write(client, 0x2d, 0x30);
	tp28xx_byte_write(client, 0x2e, 0x70);
	tp28xx_byte_write(client, 0x30, 0x48);
	tp28xx_byte_write(client, 0x31, 0xbb);
	tp28xx_byte_write(client, 0x32, 0x2e);
	tp28xx_byte_write(client, 0x33, 0x90);
	tp28xx_byte_write(client, 0x35, 0x25);
	tp28xx_byte_write(client, 0x39, 0x88);
	tp28xx_byte_write(client, 0x2c, 0x0a);
	tp28xx_byte_write(client, 0x27, 0x2d);
	tp28xx_byte_write(client, 0x28, 0x00);
	tp28xx_byte_write(client, 0x13, 0x00);

	tmp = tp28xx_byte_read(client, 0x14);
	tmp &= 0x9f;
	tp28xx_byte_write(client, 0x14, tmp);
}

static void TP2825_A720P30_DataSet(struct i2c_client *client)
{
	unsigned char tmp;

	tmp = tp28xx_byte_read(client, 0x14);
	tmp |= 0x40;
	tp28xx_byte_write(client, 0x14, tmp);
	tp28xx_byte_write(client, 0x2d, 0x48);
	tp28xx_byte_write(client, 0x2e, 0x5e);
	tp28xx_byte_write(client, 0x30, 0x27);
	tp28xx_byte_write(client, 0x31, 0x72);
	tp28xx_byte_write(client, 0x32, 0x80);
	tp28xx_byte_write(client, 0x33, 0x77);
	tp28xx_byte_write(client, 0x07, 0x80);
}

static void TP2825_A720P25_DataSet(struct i2c_client *client)
{
	unsigned char tmp;

	tmp = tp28xx_byte_read(client, 0x14);
	tmp |= 0x40;
	tp28xx_byte_write(client, 0x14, tmp);
	tp28xx_byte_write(client, 0x2d, 0x48);
	tp28xx_byte_write(client, 0x2e, 0x5e);
	tp28xx_byte_write(client, 0x30, 0x27);
	tp28xx_byte_write(client, 0x31, 0x88);
	tp28xx_byte_write(client, 0x32, 0x04);
	tp28xx_byte_write(client, 0x33, 0x23);
	tp28xx_byte_write(client, 0x07, 0x80);
}

static void TP2825_A1080P30_DataSet(struct i2c_client *client)
{
	unsigned char tmp;

	tmp = tp28xx_byte_read(client, 0x14);
	tmp |= 0x60;
	tp28xx_byte_write(client, 0x14, tmp);
	tp28xx_byte_write(client, 0x2d, 0x45);
	tp28xx_byte_write(client, 0x2e, 0x50);
	tp28xx_byte_write(client, 0x30, 0x29);
	tp28xx_byte_write(client, 0x31, 0x65);
	tp28xx_byte_write(client, 0x32, 0x78);
	tp28xx_byte_write(client, 0x33, 0x16);
	tp28xx_byte_write(client, 0x07, 0x80);
}

static void TP2825_A1080P25_DataSet(struct i2c_client *client)
{
	unsigned char tmp;

	tmp = tp28xx_byte_read(client, 0x14);
	tmp |= 0x60;
	tp28xx_byte_write(client, 0x14, tmp);
	tp28xx_byte_write(client, 0x2d, 0x45);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x29);
	tp28xx_byte_write(client, 0x31, 0x61);
	tp28xx_byte_write(client, 0x32, 0x78);
	tp28xx_byte_write(client, 0x33, 0x16);
	tp28xx_byte_write(client, 0x07, 0x80);
}

static void TP2825_C1080P25_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0xa0);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x54);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x41);
	tp28xx_byte_write(client, 0x31, 0x82);
	tp28xx_byte_write(client, 0x32, 0x27);
	tp28xx_byte_write(client, 0x33, 0xa6);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_C720P25_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0x74);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x42);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x48);
	tp28xx_byte_write(client, 0x31, 0x67);
	tp28xx_byte_write(client, 0x32, 0x6f);
	tp28xx_byte_write(client, 0x33, 0x31);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_C1080P30_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0x80);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x47);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x41);
	tp28xx_byte_write(client, 0x31, 0x82);
	tp28xx_byte_write(client, 0x32, 0x27);
	tp28xx_byte_write(client, 0x33, 0xa6);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_C720P30_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0x60);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x37);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x48);
	tp28xx_byte_write(client, 0x31, 0x67);
	tp28xx_byte_write(client, 0x32, 0x6f);
	tp28xx_byte_write(client, 0x33, 0x31);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_C720P50_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0x74);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x42);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x41);
	tp28xx_byte_write(client, 0x31, 0x82);
	tp28xx_byte_write(client, 0x32, 0x27);
	tp28xx_byte_write(client, 0x33, 0xa6);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_C720P60_DataSet(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x13, 0x40);
	tp28xx_byte_write(client, 0x20, 0x60);
	tp28xx_byte_write(client, 0x2b, 0x60);
	tp28xx_byte_write(client, 0x2d, 0x37);
	tp28xx_byte_write(client, 0x2e, 0x40);
	tp28xx_byte_write(client, 0x30, 0x41);
	tp28xx_byte_write(client, 0x31, 0x82);
	tp28xx_byte_write(client, 0x32, 0x27);
	tp28xx_byte_write(client, 0x33, 0xa6);
	tp28xx_byte_write(client, 0x28, 0x04);
	tp28xx_byte_write(client, 0x07, 0x80);
	tp28xx_byte_write(client, 0x27, 0x5a);
}

static void TP2825_PTZ_init(struct i2c_client *client)
{
	tp28xx_byte_write(client, 0x40, 0x00);
	tp28xx_byte_write(client, 0xc9, 0x00);
	tp28xx_byte_write(client, 0xca, 0x00);
	tp28xx_byte_write(client, 0xcb, 0x05);
	tp28xx_byte_write(client, 0xcc, 0x06);
	tp28xx_byte_write(client, 0xcd, 0x08);
	tp28xx_byte_write(client, 0xce, 0x09);
	tp28xx_byte_write(client, 0xcf, 0x03);
	tp28xx_byte_write(client, 0xd0, 0x48);
	tp28xx_byte_write(client, 0xd1, 0x34);
	tp28xx_byte_write(client, 0xd2, 0x60);
	tp28xx_byte_write(client, 0xd3, 0x10);
	tp28xx_byte_write(client, 0xd4, 0x04);
	tp28xx_byte_write(client, 0xd5, 0xf0);
	tp28xx_byte_write(client, 0xd6, 0xd8);
	tp28xx_byte_write(client, 0xd7, 0x17);
	tp28xx_byte_write(client, 0x7E, 0x01);
}

static void TP2825_output(struct i2c_client *client)
{
	struct tp2825_state *me = &_state;

	tp28xx_byte_write(client, 0x4C, 0x00);
	tp28xx_byte_write(client, 0x4D, 0x03); /* both port enable */

	if (mode == TP2825_720P25V2
		|| mode == TP2825_720P30V2
		|| mode == TP2825_PAL
		|| mode == TP2825_PAL_HALF
		|| mode == TP2825_NTSC
		|| mode == TP2825_NTSC_HALF) {
		if (me->out_port == 2)
			tp28xx_byte_write(client, 0x4E, 0x2f);
		else
			tp28xx_byte_write(client, 0x4E, 0x15);
	} else {
		if (me->out_port == 2)
			tp28xx_byte_write(client, 0x4E, 0x03);
		else
			tp28xx_byte_write(client, 0x4E, 0x01);
	}
}

static int tp2825_set_video_mode(struct i2c_client *client,
	unsigned char mode,
	unsigned char std
)
{
	int err = 0;
	unsigned int tmp;

	if (std == STD_HDA_DEFAULT)
		std = STD_HDA;

	switch (mode) {
	case TP2825_HALF1080P25:
	case TP2825_1080P25:
		vmsg("## [%s()] TP2825_1080P25\n", __func__);
		tp2825_set_work_mode_1080p25(client);
		tp28xx_byte_write(client, 0x02, 0xC8);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);

		if (std == STD_HDA) {
			TP2825_A1080P25_DataSet(client);
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C1080P25_DataSet(client);
			if (std == STD_HDC) { /* HDC 1080p25 position adjust */
				tp28xx_byte_write(client, 0x15, 0x13);
				tp28xx_byte_write(client, 0x16, 0x84);
			}
		}
		break;

	case TP2825_HALF1080P30:
	case TP2825_1080P30:
		vmsg("## [%s()] TP2825_1080P30\n", __func__);
		tp2825_set_work_mode_1080p30(client);
		tp28xx_byte_write(client, 0x02, 0xC8);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);

		if (std == STD_HDA) {
			TP2825_A1080P30_DataSet(client);
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C1080P30_DataSet(client);
			if (std == STD_HDC) { /* HDC 1080p30 position adjust */
				tp28xx_byte_write(client, 0x15, 0x13);
				tp28xx_byte_write(client, 0x16, 0x44);
			}
		}
		break;

	case TP2825_HALF720P25:
	case TP2825_720P25:
		vmsg("## [%s()] TP2825_720P25\n", __func__);
		tp2825_set_work_mode_720p25(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);
		break;

	case TP2825_HALF720P30:
	case TP2825_720P30:
		vmsg("## [%s()] TP2825_720P30\n", __func__);
		tp2825_set_work_mode_720p30(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);
		break;

	case TP2825_HALF720P50:
	case TP2825_720P50:
		vmsg("## [%s()] TP2825_720P50\n", __func__);
		tp2825_set_work_mode_720p50(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);

		if (std == STD_HDA) {
			;
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C720P50_DataSet(client);
			if (std == STD_HDC) /* HDC 720p50 position adjust */
				tp28xx_byte_write(client, 0x16, 0x40);
		}
		break;

	case TP2825_HALF720P60:
	case TP2825_720P60:
		vmsg("## [%s()] TP2825_720P60\n", __func__);
		tp2825_set_work_mode_720p60(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp &= 0xFB;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V1_DataSet(client);

		if (std == STD_HDA) {
			;
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C720P60_DataSet(client);
			if (std == STD_HDC) /* HDC 720p60 position adjust */
				tp28xx_byte_write(client, 0x16, 0x02);
		}
		break;

	case TP2825_720P30V2:
		vmsg("## [%s()] TP2825_720P30V2\n", __func__);
		tp2825_set_work_mode_720p60(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V2_DataSet(client);
		if (std == STD_HDA) {
			TP2825_A720P30_DataSet(client);
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C720P30_DataSet(client);
			if (std == STD_HDC) /* HDC 720p30 position adjust */
				tp28xx_byte_write(client, 0x16, 0x02);
		}
		break;

	case TP2825_720P25V2:
		vmsg("## [%s()] TP2825_720P25V2\n", __func__);
		tp2825_set_work_mode_720p50(client);
		tp28xx_byte_write(client, 0x02, 0xCA);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_V2_DataSet(client);
		if (std == STD_HDA) {
			TP2825_A720P25_DataSet(client);
		} else if (std == STD_HDC || std == STD_HDC_DEFAULT) {
			TP2825_C720P25_DataSet(client);
			if (std == STD_HDC) /* HDC 720p25 position adjust */
				tp28xx_byte_write(client, 0x16, 0x40);
		}
		break;

	case TP2825_PAL:
		vmsg("## [%s()] TP2825_PAL\n", __func__);
		tp2825_set_work_mode_PAL(client);
		tp28xx_byte_write(client, 0x02, 0xCF);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_PAL_DataSet(client);
		break;

	case TP2825_PAL_HALF:
		vmsg("## [%s()] TP2825_PAL_HALF\n", __func__);
		tp2825_set_work_mode_PAL(client);
		tp28xx_byte_write(client, 0x02, 0xCF);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_PAL_DataSet(client);
		break;

	case TP2825_NTSC:
		vmsg("## [%s()] TP2825_NTSC\n", __func__);
		tp2825_set_work_mode_NTSC(client);
		tp28xx_byte_write(client, 0x02, 0xCF);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_NTSC_DataSet(client);
		break;

	case TP2825_NTSC_HALF:
		vmsg("## [%s()] TP2825_NTSC_HALF\n", __func__);
		tp2825_set_work_mode_NTSC(client);
		tp28xx_byte_write(client, 0x02, 0xCF);
		tmp = tp28xx_byte_read(client, 0x4E);
		tmp |= 0x04;
		tp28xx_byte_write(client, 0x4E, tmp);
		TP2825_NTSC_DataSet(client);
		break;

	default:
		vmsg("## [%s()] default:\n", __func__);
		err = -1;
		break;
	}

	return err;
}

static void tp2825_reset_default(struct i2c_client *client)
{
	unsigned int tmp;

	tp28xx_byte_write(client, 0x26, 0x01);
	tp28xx_byte_write(client, 0x07, 0xC0);
	tp28xx_byte_write(client, 0x0B, 0xC0);
	tp28xx_byte_write(client, 0x22, 0x35);

	tmp = tp28xx_byte_read(client, 0x06);
	tmp &= 0xfb;
	tp28xx_byte_write(client, 0x06, tmp);
}

static void tp2825_comm_init(struct i2c_client *client, int chip)
{
	unsigned int val;

	vmsg("## [%s():%s:%d\t] chip:0x%x\n",
		__func__, strrchr(__FILE__, '/')+1, __LINE__, chip);

	if (chip == TP2825) {
		tp2825_reset_default(client);
		tp2825_set_video_mode(client, mode, STD_TVI);

		/* MUX output */
		TP2825_output(client);
		TP2825_PTZ_init(client);

		/* soft reset */
		val = tp28xx_byte_read(client, 0x06);
		tp28xx_byte_write(client, 0x06, 0x80|val);
	}
}

#ifdef DEBUG_TP2825
static void tp2825_info_print(struct tp2825_state *me)
{
	int val;
	int val1;

	mdelay(1000);

	vmsg("#### Video Input Status Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x01);
	if (val & 1<<7)
		vmsg("## Video loss\n");
	else
		vmsg("## Video present\n");

	if (val & 1<<6)
		vmsg("## Vertical PLL Lock\n");
	else
		vmsg("## Vertical PLL Not Lock\n");

	if (val & 1<<5)
		vmsg("## Horizontal PLL Lock\n");
	else
		vmsg("## Horizontal PLL Not Lock\n");

	if (val & 1<<4)
		vmsg("## Carrier PLL Lock\n");
	else
		vmsg("## Carrier PLL Not Lock\n");

	if (val & 1<<3)
		vmsg("## Video detected\n");
	else
		vmsg("## No Video\n");

	if (val & 1<<2)
		vmsg("## Detected or 50Hz(SD)\n");
	else
		vmsg("## None(EQ or 50Hz Detect)\n");

	if (val & 1<<1)
		vmsg("## Progessive video\n");
	else
		vmsg("## Interlaced video\n");

	if (val & 1<<0)
		vmsg("## No carrier detected\n");
	else
		vmsg("## carrier detected\n");


	vmsg("#### Decoding Control Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x02);
	if (val & 1<<7)
		vmsg("## Output mode : 8bit\n");
	else
		vmsg("## Output mode : 16bit\n");

	if (val & 1<<6)
		vmsg("## Y/C order : C first\n");
	else
		vmsg("## Y/C order : Y first\n");

	if (val & 1<<5)
		vmsg("## Output limit : Y=16-235, Cb/Cr=16-240\n");
	else
		vmsg("## Output limit : 1-254\n");

	if (val & 1<<3)
		vmsg("## MD656 : BT.656 format\n");
	else
		vmsg("## MD656 : BT.1120 format\n");

	if (val & 1<<2)
		vmsg("## SD mode : SD mode only\n");
	else
		vmsg("## SD mode : HD mode\n");

	if (val & 1<<1)
		vmsg("## 720p decoding\n");
	else
		vmsg("## 1080p decoding\n");

	if (val & 1<<0)
		vmsg("## Interlace mode decoding\n");
	else
		vmsg("## Progressive mode decoding\n");

	vmsg("#### Detection Status Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x03);
	if (val & 1<<3)
		vmsg("## TVI v2.0\n");
	else
		vmsg("## TVI v1.0\n");

	switch (val & 0x07) {
	case 0:
		vmsg("## 720p/60\n");
		break;
	case 1:
		vmsg("## 720p/50\n");
		break;
	case 2:
		vmsg("## 1080p/30\n");
		break;
	case 3:
		vmsg("## 1080p/25\n");
		break;
	case 4:
		vmsg("## 720p/30\n");
		break;
	case 5:
		vmsg("## 720p/25\n");
		break;
	case 6:
		vmsg("## SD\n");
		break;
	}

	vmsg("#### Comb Filter and SD Format control Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x0D);
	switch (val & 0x07) {
	case 0:
		vmsg("## NTSC-M\n");
		break;
	case 1:
		vmsg("## PAL-B\n");
		break;
	case 2:
		vmsg("## PAL-M\n");
		break;
	case 3:
		vmsg("## PAL-N\n");
		break;
	case 4:
		vmsg("## PAL-60\n");
		break;
	case 5:
		vmsg("## NTSC 4.43\n");
		break;

	default:
		vmsg("## Not supported\n");
		break;
	}

	vmsg("#### Output H/V Control Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x15);
	val1 = tp28xx_byte_read(me->i2c_client, 0x16);
	vmsg("## H-Delay : %d\n", ((val&0x30)<<4 | val1));

	val = tp28xx_byte_read(me->i2c_client, 0x1a);
	val1 = tp28xx_byte_read(me->i2c_client, 0x17);
	vmsg("## H-Active : %d\n", ((val&0x0f)<<8 | val1));

	val1 = tp28xx_byte_read(me->i2c_client, 0x18);
	vmsg("## V-Delay : %d\n", (val1));

	val = tp28xx_byte_read(me->i2c_client, 0x1a);
	val1 = tp28xx_byte_read(me->i2c_client, 0x19);
	vmsg("## V-Active : %d\n", ((val&0xf0)<<4 | val1));


	vmsg("#### H length Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x1c);
	val1 = tp28xx_byte_read(me->i2c_client, 0x1d);
	vmsg("## Number of Pixels per line : %d\n",
			((val&0x1f)<<8 | val1));

	vmsg("#### Clamping Control Register ########\n");
	val = tp28xx_byte_read(me->i2c_client, 0x26);
	if (val & 1<<7)
		vmsg("## Clamp : Disable\n");
	else
		vmsg("## Clamp : Enable\n");

	if (val & 1<<6)
		vmsg("## Clamp Current : 2X\n");
	else
		vmsg("## Clamp Current : 1X\n");
}
#endif

/*
*static void tp2825_power_clt(struct v4l2_subdev *sd, int enable)
*{
*	struct i2c_client *client = v4l2_get_subdevdata(sd);
*	struct tp2825_state *pdata = &_state;
*
*	int ret;
*
*	if (enable) {
*		if(gpio_is_valid(pdata->gpio_reset)) {
*			ret = gpio_direction_output(pdata->gpio_reset, 1);
*			if (ret) {
*				dev_err(&client->dev,
*				"%s: gpio_reset didn't output high\n",
*				__func__);
*			}
*		}
*
*		mdelay(1000);
*	} else {
*		if(gpio_is_valid(pdata->gpio_reset)) {
*			ret = gpio_direction_output(pdata->gpio_reset, 0);
*			if (ret) {
*				dev_err(&client->dev,
*				"%s: gpio_reset didn't output low.\n",
*				__func__);
*			}
*		}
*
*		mdelay(10);
*	}
*}
*/

static int tp2825_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tp2825_state *me = &_state;
	struct reg_val *reg_val;

	if (enable) {
		if (me->first) {
#ifndef CONFIG_V4L2_INIT_LEVEL_UP
			revision = tp28xx_byte_read(me->i2c_client, 0xfc);
			if (revision == 0x00) {
				revision = 0x01;
				vmsg("## [%s():%s:%d\t] TP2825B\n",
				__func__, strrchr(__FILE__, '/')+1, __LINE__);
			} else if (revision == 0xc0) {
				revision = 0x00;
				vmsg("## [%s():%s:%d\t] TP2825A\n",
				__func__, strrchr(__FILE__, '/')+1, __LINE__);
			}

			vmsg("## [%s():%s:%d\t] width:%d, height:%d\n",
				__func__, strrchr(__FILE__, '/')+1, __LINE__,
				me->width, me->height);
#endif
			if (mode == TP2825_NTSC) {
				vmsg("## [%s()] TP2825_NTSC\n", __func__);
				reg_val = _sensor_init_data_1920x480i;
				while (reg_val->reg != 0xff) {
					tp28xx_byte_write(me->i2c_client,
						reg_val->reg, reg_val->val);
					reg_val++;
				}
			} else if (mode == TP2825_NTSC_HALF) {
				vmsg("## [%s()] TP2825_NTSC_HALF\n", __func__);
				reg_val = _sensor_init_data_960x480i;
				while (reg_val->reg != 0xff) {
					tp28xx_byte_write(me->i2c_client,
						reg_val->reg, reg_val->val);
					reg_val++;
				}
			} else if (mode == TP2825_PAL) {
				vmsg("## [%s()] TP2825_PAL\n", __func__);
				reg_val = _sensor_init_data_1920x576i;
				while (reg_val->reg != 0xff) {
					tp28xx_byte_write(me->i2c_client,
						reg_val->reg, reg_val->val);
					reg_val++;
				}
			} else if (mode == TP2825_PAL_HALF) {
				vmsg("## [%s()] TP2825_PAL_HALF\n", __func__);
				reg_val = _sensor_init_data_960x576i;
				while (reg_val->reg != 0xff) {
					tp28xx_byte_write(me->i2c_client,
						reg_val->reg, reg_val->val);
					reg_val++;
				}
			} else if (mode == TP2825_NONE) {
				vmsg("## [%s()] TP2825_NONE\n", __func__);
			} else {
				tp2825_comm_init(me->i2c_client, 0x2825);
			}

			me->old_mode = mode;

#ifdef DEBUG_TP2825
			tp2825_info_print(me);
#endif
		}
	}

	return 0;
}

static int tp2825_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct tp2825_state *me = &_state;

	vmsg("%s\n", __func__);

	me->width = mf->width;
	me->height = mf->height;

	/* NTSC -> 1920x480i */
	if (me->width == 1920 && me->height == 480)
		mode = TP2825_NTSC;
	/* NTSC -> 960x480i */
	else if (me->width == 960 && me->height == 480)
		mode = TP2825_NTSC_HALF;
	/* PAL -> 1920x576i */
	else if (me->width == 1920 && me->height == 576)
		mode = TP2825_PAL;
	/* PAL -> 960x576i */
	else if (me->width == 960 && me->height == 576)
		mode = TP2825_PAL_HALF;
	/* TVI(FHD) -> 1920x1080p */
	else if (me->width == 1920 && me->height == 1080)
		mode = TP2825_1080P30;
	/* TVI(HD) -> 1280x720p */
	else if (me->width == 1280 && me->height == 720)
		mode = TP2825_720P30V2;
	else
		mode = TP2825_NONE;

	if (mode == me->old_mode
		|| mode == TP2825_NONE)
		me->first = false;
	else
		me->first = true;

	return 0;
}

static int tp2825_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *frame)
{
	vmsg("%s, index:%d\n", __func__, frame->index);

	if (frame->index >= ARRAY_SIZE(supported_resolutions))
		return -ENODEV;

	frame->max_width = supported_resolutions[frame->index].width;
	frame->max_height = supported_resolutions[frame->index].height;

	vmsg("%s, max_width:%d, max_height:%d\n", __func__,
		frame->max_width, frame->max_height);

	return 0;
}

static int tp2825_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum
				      *frame)
{
	int i;

	vmsg("%s, %s interval\n", __func__, (frame->index) ? "max" : "min");

	for (i = 0; i < ARRAY_SIZE(supported_resolutions); i++) {
		if ((frame->width == supported_resolutions[i].width) &&
		    (frame->height == supported_resolutions[i].height)) {
			frame->interval.numerator = 1;
			frame->interval.denominator =
				supported_resolutions[i].interval[frame->index];
			vmsg("[%s] width:%d, height:%d, interval:%d\n",
			     __func__, frame->width, frame->height,
			     frame->interval.denominator);
			return false;
		}
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops tp2825_subdev_pad_ops = {
	.set_fmt = tp2825_s_fmt,
	.enum_frame_size = tp2825_enum_frame_size,
	.enum_frame_interval = tp2825_enum_frame_interval,
};

static const struct v4l2_subdev_video_ops tp2825_subdev_video_ops = {
	.s_stream = tp2825_s_stream,
};

static const struct v4l2_subdev_ops tp2825_ops = {
	.video = &tp2825_subdev_video_ops,
	.pad   = &tp2825_subdev_pad_ops,
};

static int tp2825_parse_dt(struct device *dev, struct tp2825_state *pdata)
{
	struct device_node *np = dev->of_node;
	u32 val;

	/*
	*pdata->gpio_reset = of_get_named_gpio(np, "gpio_reset", 0);
	*if (devm_gpio_request_one(dev, pdata->gpio_reset,
	*		GPIOF_OUT_INIT_HIGH, "tp2825 reset") < 0)
	*	dev_err(dev, "tp2825 reset : failed to gpio %d set to high\n",
	*		pdata->gpio_reset);
	*/

	if (!of_property_read_u32(np, "out_port", &val))
		pdata->out_port = (int)val;
	else
		pdata->out_port = 0;

	if (!of_property_read_u32(np, "vdelay", &val))
		pdata->vdelay = (int)val;
	else
		pdata->vdelay = 0;

	vmsg("## %s() out_port:%d\n", __func__, pdata->out_port);
	vmsg("## %s() vdelay:%d\n", __func__, pdata->vdelay);

	return 0;
}

static int tp2825_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct tp2825_state *state = &_state;
	int ret;

	sd = &state->sd;
	strcpy(sd->name, "tp2825");

	v4l2_i2c_subdev_init(sd, client, &tp2825_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &state->pad, 0);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s: failed to media_entity_init()\n", __func__);
		return ret;
	}

	i2c_set_clientdata(client, sd);
	state->i2c_client = client;
	state->first = true;

	ret = tp2825_parse_dt(&client->dev, state);
	if (ret)
		dev_err(&client->dev, "Failed to parse DT!\n");

	return 0;
}

static int tp2825_remove(struct i2c_client *client)
{
	struct tp2825_state *state = &_state;

	/*
	* if(gpio_is_valid(state->gpio_reset))
	*	gpio_free(state->gpio_reset);
	*/

	v4l2_device_unregister_subdev(&state->sd);
	return 0;
}

static const struct i2c_device_id tp2825_id[] = {
	{ "tp2825", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, tp2825_id);

static struct i2c_driver tp2825_i2c_driver = {
	.driver = {
		.name = "tp2825",
	},
	.probe = tp2825_probe,
	.remove = tp2825_remove,
	.id_table = tp2825_id,
};

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
static int __init tp2825_mod_init(void)
{
	return i2c_add_driver(&tp2825_i2c_driver);
}

static void __exit tp2825_mod_exit(void)
{
	i2c_del_driver(&tp2825_i2c_driver);
}

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
subsys_initcall(tp2825_mod_init);
#else
module_init(tp2825_mod_init);
#endif

module_exit(tp2825_mod_exit);
#else
module_i2c_driver(tp2825_i2c_driver);
#endif

MODULE_DESCRIPTION("TP2825 Camera Sensor Driver");
MODULE_AUTHOR("<jkchoi@nexell.co.kr>");
MODULE_LICENSE("GPL");
