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
#define TEST 1

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


static const u8 TP2912_720P30_DataSet[] = {
	0x02, 0x8b,
	0x03, 0x4a,
	0x07, 0xc1,
	0x08, 0x6e,
	0x11, 0x83,
	0x12, 0x05,
	0x1c, 0xA6,
	0x1d, 0xe6,
	0x1e, 0xf8,
	0x20, 0x24,
	0x21, 0x5d,
	0x22, 0x17,
	0x23, 0x45,
	0x45, 0x8b,
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
static int tp2912_Reg720p30(struct v4l2_subdev *sd)
{
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(TP2912_720P30_DataSet); i += 2) {
		err = tp2912_write(sd, TP2912_720P30_DataSet[i],
					TP2912_720P30_DataSet[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing\n");
			return err;
		}
	}

	return 0;
}


static int tp2912_setstd(struct v4l2_subdev *sd, v4l2_std_id std)
{
	int err = 0;

	switch(std) {

		case V4L2_STD_720P_30:
			/* 720p30 */
			err = tp2912_Reg720p30(sd);
			break;
		default:
			break;
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
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
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

	tp2912_setstd(&state->sd, V4L2_STD_720P_30);

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
#ifdef CONFIG_OF
static const struct of_device_id tp2912_of_match[] = {
	{ .compatible = "techpoint, tp2912" },
	{ },
};
MODULE_DEVICE_TABLE(of, tp2912_of_match);
#endif

static struct i2c_driver tp2912_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tp2912",
		.of_match_table = of_match_ptr(tp2912_of_match),
	},
	.probe		= tp2912_probe,
	.remove		= tp2912_remove,
	.id_table	= tp2912_id,
};
module_i2c_driver(tp2912_driver);

