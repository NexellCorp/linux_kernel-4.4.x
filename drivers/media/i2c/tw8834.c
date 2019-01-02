/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Allan, park <allan.park@nexell.co.kr>
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
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <media/tw8834.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include "tw8834_reg.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

enum {
	TW8834_IN_PAD = 0,
	TW8834_OUT_PAD,
	TW8834_NUM_PADS,
};
enum {
	TW8834_INPUT_CVBS = 0,
	TW8834_INPUT_DTV,
	TW8834_NUM_INPUTS,
};
#define TW8834_GPIO_INT_NAME        "irq"
#define TW8834_GPIO_RST_NAME        "reset"

/* tw8834 data structure */
struct tw8834 {
	struct v4l2_subdev sd;
	struct i2c_client *i2c_client;
	struct media_pad pad[TW8834_NUM_PADS];
	struct v4l2_mbus_framefmt format[TW8834_NUM_PADS];
	int input;
	int is_streaming;
	int (*set_power)(int on);
	struct gpio_desc *gpiod_rst;
	struct gpio_desc *gpiod_int;
};
static inline struct tw8834 *to_tw8834(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tw8834, sd);
}

/* i2c transfer functions */
static s32 tw8834_i2c_read_byte(struct tw8834 *tw8834,
				u8 command)
{
	struct i2c_client *client = tw8834->i2c_client;
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}
static s32 tw8834_i2c_write_byte(struct tw8834 *tw8834,
				 u8 command,
				 u8 value)
{
	struct i2c_client *client = tw8834->i2c_client;
	union i2c_smbus_data data;
	int err;
	int i;

	data.byte = value;

	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		dev_err(&client->dev, "error writing %02x, %02x, %02x\n",
				client->addr, command, value);
	return err;
}
static int tw8834_write_reg_array(struct tw8834 *tw8834,
				  struct reg_cfg *array, int nb)
{
	s32 ret;
	int i;
	struct v4l2_subdev *sd = &tw8834->sd;
	struct i2c_client *client = tw8834->i2c_client;

	for (i = 0; i < nb; i++) {
		ret = tw8834_i2c_write_byte(tw8834,
					    array[i].reg,
					    array[i].val);
		if (ret < 0) {
			dev_err(&client->dev, "failed to write 0x%X to reg 0x%X\n",
				 array[i].val, array[i].reg);
			return ret;
		}
	}
	return 0;
}
static int tw8834_read_register(struct tw8834 *tw8834,
				u16 reg)
{
	int ret;
	int ret;
	u8 page = (reg >> 8);

	ret = tw8834_i2c_write_byte(tw8834, 0xFF, page);
	if (ret < 0)
		return ret;

	ret = tw8834_i2c_read_byte(tw8834, (u8)(reg & 0xFF));

	return ret;
}
static int tw8834_write_register(struct tw8834 *tw8834,
				 u16 reg,
				 u8 value)
{
	int ret;
	u8 page = (reg >> 8);

	ret = tw8834_i2c_write_byte(tw8834, 0xFF, page);
	if (ret < 0)
		return ret;

	ret = tw8834_i2c_write_byte(tw8834, (u8)(reg & 0xFF), value);
	return ret;
}
/* video operations */
static int tw8834_s_routing(struct v4l2_subdev *sd, u32 in, u32 out, u32 config)
{
	struct tw8834 *tw8834 = to_tw8834(sd);

	if (!tw8834 || in >= TW8834_NUM_INPUTS)
		return -EINVAL;

	tw8834->input = in;
	return 0;
}
static int tw8834_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tw8834 *tw8834 = to_tw8834(sd);
	int ret = 0;

	if (enable) {

		if (tw8834->set_power)
			tw8834->set_power(TW8834_POWER_INPUT_ENABLE);
	} else {
		if (tw8834->set_power)
			tw8834->set_power(TW8834_POWER_INPUT_DISABLE);
	}

	tw8834->is_streaming = !!enable;
	return ret;
}
int tw8834_subscribe_event(struct v4l2_subdev *subdev,
			   struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	return v4l2_src_change_event_subscribe(fh, sub);
}
int tw8834_unsubscribe_event(struct v4l2_subdev *subdev,
			     struct v4l2_fh *fh,
			     struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}
static const struct v4l2_subdev_core_ops tw8834_core_ops = {
	.subscribe_event = tw8834_subscribe_event,
	.unsubscribe_event = tw8834_unsubscribe_event,
};
static const struct v4l2_subdev_video_ops tw8834_video_ops = {
	.s_routing = tw8834_s_routing,
};

static const struct v4l2_subdev_ops tw8834_ops = {
	.core = &tw8834_core_ops,
	.video = &tw8834_video_ops,
};

static int tw8834_get_gpio_config(struct tw8834 *tw8834)
{
	int error;
	struct device *dev;
	struct gpio_desc *gpiod;

	if (!tw8834->i2c_client)
		return -EINVAL;

	dev = &tw8834->i2c_client->dev;

	/* Get the interrupt GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, TW8834_GPIO_INT_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
					TW8834_GPIO_INT_NAME, error);
		return error;
	}

	tw8834->gpiod_int = gpiod;

	/* Get the reset line GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, TW8834_GPIO_RST_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
					TW8834_GPIO_RST_NAME, error);
		return error;
	}

	tw8834->gpiod_rst = gpiod;

	return 0;

}
static int tw8834_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tw8834 *tw8834;
	struct v4l2_subdev *sd;
	int ret = -ENODEV;
	struct v4l2_mbus_framefmt *format;
	struct reg_cfg reg;


	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}
	dev_info(&client->dev, "detecting tw8834 client on address 0x%x\n",
			client->addr << 1);
	tw8834 = kzalloc(sizeof(*tw8834), GFP_KERNEL);
	if (!tw8834) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	tw8834->i2c_client = client;
	sd = &tw8834->sd;
	v4l2_i2c_subdev_init(sd, client, &tw8834_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	tw8834->pad[TW8834_IN_PAD].flags = MEDIA_PAD_FL_SINK;
	tw8834->pad[TW8834_OUT_PAD].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 2, tw8834->pad, 0);
	if (ret < 0) {
		dev_err(&client->dev, "failed to init media entity\n");
		goto emedia;
	}

	ret = tw8834_get_gpio_config(tw8834);

	if (ret < 0)
		dev_err(&client->dev, "gpio_config failed\n");

	/* reset */
	gpiod_direction_output(tw8834->gpiod_rst, 0);
	msleep(20);
	gpiod_direction_output(tw8834->gpiod_rst, 1);

	gpiod_direction_output(tw8834->gpiod_int, 0);

	reg.reg = 0xFF;
	reg.val = 0x00;
	ret = tw8834_i2c_write_byte(tw8834, reg.reg, reg.val);
	if (ret < 0) {
		dev_err(&client->dev, "impossible to write to tw8834\n");
		goto echipident;
	}
	reg.reg = 0x00;
	ret = tw8834_i2c_read_byte(tw8834, reg.reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read chip id\n");
		goto echipident;
	}
	reg.val = ret;
	if (reg.val != 0x34) {
		dev_err(&client->dev, "read 0x%x in chip ident reg\n", reg.val);
		ret = -ENODEV;
		goto echipident;
	}

	tw8834_write_reg_array(tw8834,
			       tw8834_common_cfg,
			       ARRAY_SIZE(tw8834_common_cfg));
		return 0;
echipident:
	media_entity_cleanup(&sd->entity);
emedia:
	kfree(tw8834);
enomem:
ei2c:
	return ret;
}
static int tw8834_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw8834 *tw8834 = to_tw8834(sd);

	if (client->irq > 0)
		free_irq(client->irq, tw8834);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	kfree(tw8834);
	return 0;
}
#ifdef CONFIG_PM
int tw8834_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
int tw8834_resume(struct i2c_client *client)
{
	return 0;
}
#endif
/* ----------------------------------------------------------------------- */
static const struct i2c_device_id tw8834_id[] = {
	{ "tw8834", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tw8834_id);

#ifdef CONFIG_OF
static const struct of_device_id tw8834_of_match[] = {
	{ .compatible = "intersil, tw8834" },
	{ },
};
MODULE_DEVICE_TABLE(of, tw8834_of_match);
#endif

static struct i2c_driver tw8834_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tw8834",
		.of_match_table = of_match_ptr(tw8834_of_match),
	},
	.probe = tw8834_probe,
	.remove = tw8834_remove,
#ifdef CONFIG_PM
	.suspend = tw8834_suspend,
	.resume = tw8834_resume,
#endif
	.id_table = tw8834_id,
};
module_i2c_driver(tw8834_driver);


MODULE_DESCRIPTION("TW8834 decoder Driver");
MODULE_AUTHOR("<allan.park@nexell.co.kr>");
MODULE_LICENSE("GPL");
