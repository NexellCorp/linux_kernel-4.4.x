/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: NXFAE (nxfae@nexell.co.kr)
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
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/uaccess.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

MODULE_DESCRIPTION("TP2912 TVI encoder drvier");
MODULE_LICENSE("GPL");

struct tp2912_state {
    struct v4l2_subdev sd;
    struct v4l2_ctrl_handler hdl;
	u8 *resol;
    v4l2_std_id std;
};

/* dump all tp2912 register [FF:00]*/
int reg_dump = 0;

static const u8 tp2912_init_reg[] = {
    0x21, 0xbb,
    0x45, 0x8b,
};

static const u8 TP2910_DataSet[] = {
//	0x03, 0x4a,
	0x16, 0xeb,
	0x19, 0xf0,
	0x1a, 0x10,
	0x1c, 0x55,
	0x1d, 0x76,
	0x20, 0x48,
	0x21, 0xbb,
	0x22, 0x2e,
	0x23, 0x8b,
	0x41, 0x00,
	0x42, 0x12,
	0x43, 0x07,
	0x44, 0x49,
	0x45, 0xcb,
};

static const u8 TP2912_1080P30_DataSet[] = {
	0x02, 0x83,
	0x05, 0x40,
	0x08, 0x58,
	0x09, 0x2c,
	0x0a, 0x2c,
	0x0b, 0x00,
	0x0c, 0xc0,
	0x0d, 0xc0,
	0x0f, 0x98,
	0x10, 0x08,
	0x11, 0x6c,
	0x1c, 0xa6,
	0x1d, 0xe6,
	0x1e, 0xf8,
};

static const u8 TP2912_1080P25_DataSet[] = {
	0x02, 0x93,
	0x05, 0x40,
	0x08, 0x10,
	0x09, 0x2c,
	0x0a, 0x2c,
	0x0b, 0x20,
	0x0c, 0xc0,
	0x0d, 0xc0,
	0x0f, 0x50,
	0x10, 0x2a,
	0x11, 0x24,
	0x1c, 0xa6,
	0x1d, 0xe6,
	0x1e, 0xf8,
};

static const u8 TP2912_720P60_DataSet[] = {
	0x07, 0xc1,
	0x43, 0x17,
	0x02, 0x8b,
	0x08, 0x6e,
	0x11, 0x83,
	0x12, 0x05,
	0x1c, 0xa6,
	0x1d, 0xe6,
	0x1e, 0xf8,
};

static const u8 TP2912_720P30_DataSet[] = {
	0x02, 0x8b,
	0x08, 0xe0,
	0x09, 0x28,
	0x0a, 0x28,
	0x0b, 0x45,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0f, 0xe4,
	0x10, 0x6c,
	0x11, 0xf5,
	0x12, 0x05,
};

static const u8 TP2912_720P50_DataSet[] = {
	0x02, 0x9b,
	0x08, 0xb8,
	0x0b, 0x15,
	0x0f, 0xbc,
	0x10, 0x17,
	0x11, 0xcd,
	0x12, 0x05,
	0x1c, 0xa6,
	0x1d, 0xe6,
	0x1e, 0xf8,
};


static const u8 TP2912_NTSC_DataSet[] = {
	0x02, 0x49,
	0x05, 0x24,
	0x07, 0x81,
	0x08, 0x20,
	0x09, 0x42,
	0x0a, 0x28,
	0x0b, 0x05,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0e, 0x3c,
	0x0f, 0x5a,
	0x10, 0x03,
	0x11, 0x16,
	0x12, 0x04,
	0x15, 0x2a,
	0x1b, 0x77,
	0x1c, 0xaa,
	0x1d, 0xf0,
	0x1e, 0xa9,
	0x41, 0xc1,
	0x45, 0x8c,
};

static const u8 TP2912_PAL_DataSet[] = {
	0x02, 0x1a,
	0x03, 0x43,
	0x05, 0x20,
	0x07, 0x81,
	0x08, 0x20,
	0x09, 0x42,
	0x0a, 0x28,
	0x0b, 0x05,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0e, 0x3c,
	0x0f, 0x60,
	0x10, 0x03,
	0x11, 0x20,
	0x12, 0x04,
	0x13, 0x3f,
	0x15, 0x2c,
	0x1b, 0x77,
	0x1c, 0xaa,
	0x1d, 0xf0,
	0x1e, 0x76,
	0x41, 0xc1,
	0x45, 0x8c,
};

static const u8 TP2910_1080P30_DataSet[] = {
	0x02, 0x87,
	0x05, 0x40,
	0x08, 0x58,
	0x09, 0x2c,
	0x0a, 0x2c,
	0x0b, 0x00,
	0x0c, 0xc0,
	0x0d, 0xc0,
	0x0f, 0x98,
	0x10, 0x08,
	0x11, 0x6c,
};


static const u8 TP2910_1080P25_DataSet[] = {
	0x02, 0x97,
	0x05, 0x40,
	0x08, 0x10,
	0x09, 0x2c,
	0x0a, 0x2c,
	0x0b, 0x20,
	0x0c, 0xc0,
	0x0d, 0xc0,
	0x0f, 0x50,
	0x10, 0x2a,
	0x11, 0x24,
};


static const u8 TP2910_720P60_DataSet[] = {
	0x02, 0x8b,
    0x07, 0x81,
	0x08, 0x6e,
	0x09, 0x28,
	0x0a, 0x28,
	0x0b, 0x05,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0f, 0x72,
	0x10, 0x06,
	0x11, 0x83,
	0x12, 0x05,
    0x3e, 0x18,
    0x3c, 0x50,
};

static const u8 TP2910_720P50_DataSet[] = {
	0x02, 0x9b,
	0x08, 0xb8,
	0x09, 0x28,
	0x0a, 0x28,
	0x0b, 0x15,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0f, 0xbc,
	0x10, 0x17,
	0x11, 0xcd,
	0x12, 0x05,
};

static const u8 TP2910_720P30_DataSet[] = {
	0x02, 0x8b,
	0x08, 0xe0,
	0x09, 0x28,
	0x0a, 0x28,
	0x0b, 0x45,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0f, 0xe4,
	0x10, 0x6c,
	0x11, 0xf5,
	0x12, 0x05,
};

static const u8 TP2910_720P25_DataSet[] = {
	0x02, 0x9f,
	0x08, 0x74,
	0x09, 0x28,
	0x0a, 0x28,
	0x0b, 0x65,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0f, 0x78,
	0x10, 0x9f,
	0x11, 0x89,
	0x12, 0x05,
};

static const u8 TP2910_NTSC_DataSet[] = {
	0x02, 0x09,
	0x05, 0x24,
	0x07, 0x81,
	0x08, 0x20,
	0x09, 0x42,
	0x0a, 0x28,
	0x0b, 0x05,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0e, 0x3a,
	0x0f, 0x5a,
	0x10, 0x03,
	0x11, 0x16,
	0x12, 0x04,
	0x1b, 0x97,
	0x1c, 0x81,
	0x1d, 0xb6,
};

static const u8 TP2910_PAL_DataSet[] = {
	0x02, 0x09,
	0x05, 0x24,
	0x07, 0x81,
	0x08, 0x20,
	0x09, 0x42,
	0x0a, 0x28,
	0x0b, 0x05,
	0x0c, 0x04,
	0x0d, 0x04,
	0x0e, 0x3a,
	0x0f, 0x5a,
	0x10, 0x03,
	0x11, 0x16,
	0x12, 0x04,
	0x1b, 0x97,
	0x1c, 0x81,
	0x1d, 0xb6,
};


static const u8 TP2910_AHD1080P30_DataSet[] = {		
	0x05, 0x4a,
	0x08, 0xc0,
	0x09, 0x94,
	0x0a, 0x94,
	0x0b, 0x10,
	0x0c, 0x02,
	0x0d, 0x64,
	0x0e, 0x94,
	0x0f, 0x98,
	0x10, 0x08,
	0x11, 0x28,
	0x12, 0x00,
	0x20, 0x29,
	0x21, 0x65,
	0x22, 0x78,
	0x23, 0x16,
	0x29, 0x35,
};


static const u8 TP2910_AHD1080P25_DataSet[] = {		
	0x05, 0x4a,
	0x08, 0xc0,
	0x09, 0x94,
	0x0a, 0x94,
	0x0b, 0x10,
	0x0c, 0x02,
	0x0d, 0x64,
	0x0e, 0x94,
	0x0f, 0x50,
	0x10, 0x1a,
	0x11, 0xe0,
	0x12, 0x00,
	0x20, 0x29,
	0x21, 0x61,
	0x22, 0x78,
	0x23, 0x16,
	0x29, 0x35,
};

static const u8 TP2910_AHD720P30_DataSet[] = {		
	0x02, 0x8f,
	0x05, 0x60,
	0x09, 0x80,
	0x0b, 0x05,
	0x0e, 0x80,
	0x0f, 0x72,
	0x10, 0x06,
	0x20, 0x27,
	0x21, 0x72,
	0x22, 0x80,
	0x23, 0x77,
	0x29, 0x35,
};

static const u8 TP2910_AHD720P25_DataSet[] = {		
	0x02, 0x9f,
	0x08, 0xb8,
	0x09, 0x8c,
	0x0b, 0x15,
	0x0f, 0xbc,
	0x10, 0x17,
	0x11, 0xcd,
	0x20, 0x27,
	0x21, 0x88,
	0x22, 0x04,
	0x23, 0x23,
	0x29, 0x35,
};

static const u8 TP2910_TVI_UpData_DataSet[] = {		//26clocks/bit & 34bits/line
	0x57, 0x04,
	0x58, 0xec,
	0x59, 0x4e,
	0x5a, 0x0d,
	0x5d, 0x00,
	0x5c, 0x00,
	0x5d, 0x0c,
	0x5c, 0x0b,
	0x5e, 0x21,	// disable VBI data recive
	0x6b, 0xa0,
	0x6d, 0x04,
	0x94, 0x10,
	0x3b, 0x15,
};

static const u8 TP2910_AHD1080_UpData_DataSet[] = {		
	0x57, 0x03,
	0x58, 0x75,
	0x59, 0x9f,
	0x5a, 0x13,
	0x5c, 0x11,
	0x5d, 0x12,
	0x5e, 0x97,
	0x7d, 0x13,
	0x7e, 0x14,
};

static const u8 is_voltage_mode[] = {
	0x3e, 0x18,
};

static const u8 is_tvi20[] = {
	0x3c, 0x90,
};

static const u8 is_other[] = {
	0x3c, 0x50,
};

static const u8 is_resol_ntsc_pal[] = {
	0x3c, 0x90,
	0x3e, 0x1c,
};

static const u8 is_resol_720p30_720p25[] = {
	0x3c, 0x90,
};

static const u8 enable_slice[] = {
	//0x6b, 0xa0, // not use VBI
	//0x6d, 0x04, // not use VBI
	0x94, 0x10,
	0x3b, 0x15,
};

static const u8 techpoint_1[] = {
	0x42, 0x52,
	0x43, 0x07,
	0x44, 0x49,
	0x02, 0x8b,
	0x03, 0x4a,
	0x07, 0xc1,
	0x11, 0x83,
	0x12, 0x05,
	0x1c, 0xa6,
	0x1d, 0xe6,
	0x1e, 0xf8,
	0x20, 0x24,
	0x21, 0x5d,
	0x22, 0x17,
	0x23, 0x45,
	0x3c, 0x90,
	0x3e, 0x1c,
	0x45, 0x9c,
	0xfb, 0x03,
	0x43, 0x47,
};

static const u8 techpoint_2[] = {	
	0x43, 0x07,
	0xfc, 0x12,
	0x06, 0x80,
};
	

static int tp2912_setstd(struct v4l2_subdev *sd, v4l2_std_id std);


static inline struct tp2912_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tp2912_state, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct tp2912_state, hdl)->sd;
}

static inline int tp2912_read(struct v4l2_subdev *sd, u8 reg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    return i2c_smbus_read_byte_data(client, reg);
}

static inline int tp2912_write(struct v4l2_subdev *sd, u8 reg, u8 value)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    return i2c_smbus_write_byte_data(client, reg, value);
}

/* 720p30Hz */
static int techpoint_init(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;
	
	for (i = 0; i < ARRAY_SIZE(techpoint_1); i += 2) {
		err = tp2912_write(sd, techpoint_1[i],
					techpoint_1[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing\n");
			return err;
		}
	}
	usleep_range(100000, 110000);
	for (i = 0; i < ARRAY_SIZE(techpoint_2); i += 2) {
		err = tp2912_write(sd, techpoint_2[i],
					techpoint_2[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing\n");
			return err;
		}
	}

	return 0;
}

static int tp2912_initialize(struct v4l2_subdev *sd)
{
	int err = 0;
	int dev_id = 0;
	int i;
	struct tp2912_state *state = to_state(sd);

	err = techpoint_init(sd);
	if (err)
		return err;

	if(reg_dump) {
		for (i=0; i<0x100; i++) {
			dev_id = tp2912_read(&state->sd, i);
			v4l2_err(sd, "addr = 0x%02x : 0x%02x\n", i, dev_id);  
		}
	}
	return err;
}


int tp2912_s_register(struct v4l2_subdev *sd, 
							const struct v4l2_dbg_register *reg)
{
	int err = 0;
	//unsigned int addr, val;

	err = tp2912_initialize(sd);
	if (err)
		return err;
	
	#if 0
	while (1) {
		if ((reg->reg == 0xFF) && (reg->val == 0xFF))
			break;

		addr = (unsigned int)reg->reg;
		val  = (unsigned int)reg->val;

		err = tp2912_write(sd, addr, val);
		if (err<0) {
			v4l2_err(sd, "Error setting register, write failed\n");
			v4l2_err(sd, "Address : [0x%0X], Value : [0x%0X]\n",
				(unsigned int)reg->reg, (unsigned int)reg->val);
			err = -1;
		}
		reg++;
	}
	#endif

	return err;
}

#if 0
static int tp2912_set_pll_startup(struct v4l2_subdev *sd)
{
    u8 pll_con;
    u8 err; 

    pll_con = tp2912_read(sd, 0x43);
    err = tp2912_write(sd, 0x43, (pll_con&0xbf));
    if (err) { 
        v4l2_err(sd, "Error %s %d\n", __func__, __LINE__);
        return err;
    }
    usleep_range(500000, 510000);
    err = tp2912_write(sd, 0x43, pll_con);
    if (err) { 
        v4l2_err(sd, "Error %s %d\n", __func__, __LINE__);
        return err;
    }
    return err;
}
#endif

#if 0
static int set_tp2912_ntsc(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;

	/*NTSC*/
	for (i = 0; i < ARRAY_SIZE(TP2912_NTSC_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_NTSC_DataSet[i],
					TP2912_NTSC_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}

	for (i = 0; i < ARRAY_SIZE(is_voltage_mode); i += 2) {
		err = tp2912_write(sd, is_voltage_mode[i],
					is_voltage_mode[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}

	for (i = 0; i < ARRAY_SIZE(is_resol_ntsc_pal); i += 2) {
		err = tp2912_write(sd, is_resol_ntsc_pal[i],
					is_resol_ntsc_pal[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}
	return err;
}
#endif

#if 0
static int set_tp2912_pal(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;

	/*PAL*/
	for (i = 0; i < ARRAY_SIZE(TP2912_PAL_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_PAL_DataSet[i],
					TP2912_PAL_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}

	for (i = 0; i < ARRAY_SIZE(is_resol_ntsc_pal); i += 2) {
		err = tp2912_write(sd, is_resol_ntsc_pal[i],
					is_resol_ntsc_pal[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}
	return err;
}
#endif

#if 0
static int set_tp2912_720p60(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(TP2912_720P60_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_720P60_DataSet[i],
					TP2912_720P60_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing %s : %d\n" ,__func__, __LINE__);
			return err;
		}
	}
	return err;
}
#endif

#if 0
static int set_tp2912_720p30_TVI20(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(TP2912_720P60_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_720P60_DataSet[i],
					TP2912_720P60_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing %s : %d\n" ,__func__, __LINE__);
			return err;
		}
	}
	for (i = 0; i < ARRAY_SIZE(is_voltage_mode); i += 2) {
		err = tp2912_write(sd, is_voltage_mode[i],
					is_voltage_mode[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}
	for (i = 0; i < ARRAY_SIZE(is_tvi20); i += 2) {
		err = tp2912_write(sd, is_tvi20[i],
					is_tvi20[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}
	return err;
}
#endif

#if 0
static int set_tp2912_720p30(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;
	
	for (i = 0; i < ARRAY_SIZE(TP2912_720P30_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_720P30_DataSet[i],
					TP2912_720P30_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing %s : %d\n" ,__func__, __LINE__);
			return err;
		}
	}
	
	for (i = 0; i < ARRAY_SIZE(is_voltage_mode); i += 2) {
		err = tp2912_write(sd, is_voltage_mode[i],
					is_voltage_mode[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}
	
	for (i = 0; i < ARRAY_SIZE(is_other); i += 2) {
		err = tp2912_write(sd, is_other[i],
					is_other[i+1]);
		if (err) {
			v4l2_err(sd, "Error %s : %d\n", __func__, __LINE__);
			return err;
		}
	}

	return err;
}
#endif

static int tp2912_setstd(struct v4l2_subdev *sd, v4l2_std_id std)
{
	int err = 0;

	switch(std) {
		case V4L2_STD_NTSC:
			//set_tp2912_ntsc(sd);
			break;

		case V4L2_STD_PAL:
			//set_tp2912_pal(sd);
			break;

		case V4L2_STD_720P_30:
			//set_tp2912_720p30_TVI20(sd);
			/* 720p30 */
			err = techpoint_init(sd); 
			break;

		default:
			/* 720p30 */
			err = techpoint_init(sd);
	}
	return err;
}

static int tp2912_s_std_output(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct tp2912_state *state = to_state(sd);
	int err = 0;

	if (state->std == std)
		return 0;

	err = tp2912_setstd(sd, std);
	if (!err)
		state->std = std;

	return err;
}


static const struct v4l2_subdev_video_ops tp2912_video_ops = {
	.s_std_output = tp2912_s_std_output,
};

static const struct v4l2_subdev_core_ops tp2912_core_ops = {
	//.log_status = tp2912_log_status,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.s_register = tp2912_s_register,
};

static const struct v4l2_subdev_ops tp2912_ops = {
	.core	= &tp2912_core_ops,
	.video	= &tp2912_video_ops,
};

static int tp2912_probe(struct i2c_client *client, 
                    const struct i2c_device_id *id)
{
    int err = 0;
    
    struct tp2912_state *state;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENODEV;
    
    v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);
    
    state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;
	
	state->std = V4L2_STD_720P_30;

    v4l2_i2c_subdev_init(&state->sd, client, &tp2912_ops);

    return err; 
}

static int tp2912_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id tp2912_id[] = {
	{"tp2912", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tp2912_id);

static struct i2c_driver tp2912_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tp2912",
	},
	.probe		= tp2912_probe,
	.remove		= tp2912_remove,
	.id_table	= tp2912_id,
};
module_i2c_driver(tp2912_driver);

