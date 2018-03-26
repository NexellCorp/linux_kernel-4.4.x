/*
 * Samsung Sensor Driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 * Author: Sooman Jeong <sm5.jeong@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define SENSOR_NAME "ZN240"

#define ZN240_PAGE	0x00

struct zn240_state {
	struct v4l2_subdev	subdev;
	struct v4l2_mbus_framefmt fmt;
	struct media_pad	pad;	/* for media device pad */
	struct i2c_client	*client;
};

struct reg_value {
	u8 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

static const struct reg_value zn240_init_setting[] = {
};

static const struct reg_value zn240_test_pattern_on[] = {
	{0x00, 0x01, 0},
	{0x7f, 0x03, 0},
	{0x93, 0xe0, 0},
};

static const struct reg_value zn240_test_parttern_off[] = {
	{0x00, 0x01, 0},
	{0x7f, 0x01, 0},
	{0x93, 0x60, 0},
};

static inline struct zn240_state *to_state
	(struct v4l2_subdev *subdev)
{
	struct zn240_state *state =
		container_of(subdev, struct zn240_state, subdev);
	return state;
}

static inline struct i2c_client *to_client
	(struct v4l2_subdev *subdev)
{
	return (struct i2c_client *)v4l2_get_subdevdata(subdev);
}

static int sensor_zn240_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

static int sensor_zn240_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	*val = buf[0];
	return 0;
}

static int sensor_zn240_load_regs(struct v4l2_subdev *subdev,
			    const struct reg_value *regs, int size)
{
	unsigned int i;
	u32 delay_ms;
	u8 reg_addr;
	u8 val;
	int ret = 0;
	struct i2c_client *client = to_client(subdev);

	for (i = 0; i < size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;

		ret = sensor_zn240_write_reg(client, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000*delay_ms, 1000*delay_ms+100);
	}

	return ret;
}

static int sensor_zn240_init(struct v4l2_subdev *subdev, u32 val)
{
	int ret = 0;
	struct i2c_client *client = to_client(subdev);
	u8 id_h = 0, id_l = 0;

	WARN_ON(!subdev);

	dev_info(&client->dev, "%s start\n", __func__);

	/* Set page 0 */
	ret = sensor_zn240_write_reg(client, ZN240_PAGE, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "failed to set page 0\n");
		return ret;
	}

	ret = sensor_zn240_read_reg(client, 0x01, &id_h);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read id_h\n");
		return ret;
	}

	ret = sensor_zn240_read_reg(client, 0x02, &id_l);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read id_l\n");
		return ret;
	}

	dev_info(&client->dev, "id=0x%02x, 0x%02x\n", id_h, id_l);

	/* init member */
	ret = sensor_zn240_load_regs(subdev, zn240_init_setting,
				ARRAY_SIZE(zn240_init_setting));
	if (ret < 0) {
		dev_err(&client->dev, "init_reg failed\n");
		return ret;
	}

	dev_info(&client->dev, "%s end\n", __func__);

	msleep(100);

	return ret;
}

static int sensor_zn240_s_stream(struct v4l2_subdev *subdev,
	int enable)
{
	int ret = 0;
	struct i2c_client *client = to_client(subdev);

	dev_info(&client->dev, "%s %d\n", __func__, enable);

	if (enable) {
		ret = sensor_zn240_init(subdev, 1);
		if (ret) {
			dev_err(&client->dev, "stream_on is fail(%d)", ret);
			goto p_err;
		}
	}

p_err:
	return ret;
}

static int sensor_zn240_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int sensor_zn240_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int sensor_zn240_subdev_registered(struct v4l2_subdev *sd)
{
	return 0;
}

static void sensor_zn240_subdev_unregistered(struct v4l2_subdev *sd)
{
}

static int sensor_zn240_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int sensor_zn240_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int sensor_zn240_s_param(struct v4l2_subdev *sd,
	struct v4l2_streamparm *param)
{
	return 0;
}
static struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt		= sensor_zn240_s_fmt,
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init		= sensor_zn240_init,
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = sensor_zn240_s_stream,
	.s_parm = sensor_zn240_s_param,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad	= &pad_ops,
};

static const struct v4l2_subdev_internal_ops internal_ops = {
	.open = sensor_zn240_subdev_open,
	.close = sensor_zn240_subdev_close,
	.registered = sensor_zn240_subdev_registered,
	.unregistered = sensor_zn240_subdev_unregistered,
};

static const struct media_entity_operations media_ops = {
	.link_setup = sensor_zn240_link_setup,
};

int sensor_zn240_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct v4l2_subdev *subdev_module;
	struct zn240_state *zn240_state;

	zn240_state = devm_kzalloc(&client->dev, sizeof(struct zn240_state),
		GFP_KERNEL);
	if (!zn240_state) {
		return -ENOMEM;
	}

	subdev_module = &zn240_state->subdev;
	snprintf(subdev_module->name, V4L2_SUBDEV_NAME_SIZE,
		"%s", SENSOR_NAME);

	v4l2_i2c_subdev_init(subdev_module, client, &subdev_ops);

	zn240_state->pad.flags = MEDIA_PAD_FL_SOURCE;

	subdev_module->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	subdev_module->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	subdev_module->internal_ops = &internal_ops;
	subdev_module->entity.ops = &media_ops;

	ret = media_entity_init(&subdev_module->entity, 1,
		&zn240_state->pad, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s, failed\n", __func__);
		return ret;
	}

	zn240_state->client = client;

	dev_info(&client->dev, "(%d)\n", ret);
	return ret;
}

static int sensor_zn240_remove(struct i2c_client *client)
{
	int ret = 0;
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	return ret;
}

static const struct i2c_device_id sensor_zn240_idt[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

static struct i2c_driver sensor_zn240_driver = {
	.driver = {
		.name	= SENSOR_NAME,
	},
	.probe	= sensor_zn240_probe,
	.remove	= sensor_zn240_remove,
	.id_table = sensor_zn240_idt
};

module_i2c_driver(sensor_zn240_driver);

MODULE_AUTHOR("Sooman Jeong <sm5.jeong@samsung.com>");
MODULE_DESCRIPTION("Sensor zn240 driver");
MODULE_LICENSE("GPL v2");
