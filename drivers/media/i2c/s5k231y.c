/*
 * Samsung Dynamic Vision Sensor driver
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define SENSOR_NAME "S5K231Y"

#define DEBUG
#include <linux/device.h>

enum {
	INTERVAL_MIN = 0,
	INTERVAL_MAX,
	INTERVAL,
};

struct nx_resolution {
	uint32_t width;
	uint32_t height;
	uint32_t interval[INTERVAL];
};

static struct nx_resolution supported_resolutions[] = {
	{
		.width	= 1024,//640,
		.height = 64,//480,
		.interval[INTERVAL_MIN] = 15,
		.interval[INTERVAL_MAX] = 30,
	}
};

struct s5k231y_state {
	struct v4l2_subdev	subdev;
	struct v4l2_mbus_framefmt fmt;
	struct media_pad	pad;	/* for media device pad */
	struct i2c_client	*client;
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};
static const struct reg_value s5k231y_init_setting_vga_800[] = {
	{0x3908, 0x40, 0}, /* set dphy_auto_enable */
	{0x000d, 0x24, 1}, /* re-write bias */
	{0x3500, 0x01, 0}, /* bypass actdcs*/
	{0x3600, 0x00, 0}, /* shist off */
	{0x3300, 0x01, 0}, /* bypass crop */
	{0x3043, 0x01, 0}, /* bypass esp */
	{0x001c, 0x08, 0}, /* low sensitivity */
	{0x001e, 0x00, 0}, /* low sensitivity */
	{0x3255, 0x0c, 0},
	{0x000f, 0x05, 0},
	{0x301e, 0x00, 0}, /* clr para_out_en_r */
	{0x300F, 0xA0, 0}, /* PLL_M : 1000MHz --> 800MHz */
	{0x3924, 0x0D, 0}, /* lpx(0), band(13) */
	{0x3000, 0x02, 1}, /* set dvs mode to active mode*/
	{0x3266, 0x08, 0}, /* boot sequence */
	{0x3255, 0x0c, 0}, /* enable global hold and global reset */
	{0x000f, 0x07, 0},
	{0x0010, 0xff, 0},
	{0x0013, 0x03, 0}, /* default value : 0x01 */
	{0x0014, 0x7f, 0}, /* default value : 0x02 */
	{0x0015, 0x00, 0}, /* disable optical black */
	{0x000c, 0x14, 0},
	{0x000b, 0x06, 0},
	{0x0012, 0x7d, 0},
	{0x001c, 0x00, 0}, /* Dummy NUM ON, Default : 04 */
	{0x001e, 0x08, 0}, /* Dummy NUM nOFF, Default : 04*/
	{0x3248, 0x00, 0},
	{0x323c, 0x04, 0},
	{0x3240, 0x00, 0},
	{0x3241, 0x00, 0},
	{0x3242, 0x02, 0},
	{0x3243, 0x32, 0},
	{0x3244, 0x00, 0},
	{0x3245, 0x00, 0},
	{0x3246, 0x00, 0},
	{0x3247, 0x32, 0},
	{0x324b, 0x0a, 0},
	{0x324c, 0x08, 0},
	{0x324d, 0x14, 0},
	{0x324e, 0x04, 0},
	{0x324f, 0x08, 0},
	{0x3251, 0x0f, 0},
	{0x3253, 0x0c, 0},
	{0x3254, 0x10, 0},
	{0x3256, 0x00, 0},
	{0x3257, 0x00, 0},
	{0x3258, 0x01, 0},
	{0x3259, 0x32, 1},
	{0x3238, 0x01, 0}, /* reset timestamp */
	{0x3238, 0x00, 0}, /* restart timestamp */
	{0x390b, 0xF0, 0}, /* set use_big_endian */
	{0x3901, 0x1e, 0}, /* set sci2_header : id(yuv422-8bit) */
	{0x391a, 0x10, 0}, /* set frm_cnt *CHECK* */
	{0x3900, 0xF0, 0}, /* set mipi_enable | 4-lane | use_fram */
	{0x000d, 0x04, 0},
	{0x0015, 0x00, 0},
	{0x0008, 0x3b, 0},
	{0x000b, 0x04, 0},
	{0x001c, 0x00, 0},
	{0x0012, 0x7d, 0},
	{0x0014, 0x7f, 0},
	{0x001e, 0x00, 0},
	{0x3255, 0x0d, 0},
};
static const struct reg_value s5k231y_init_setting_vga_500[] = {
#if 0
	{0x3908, 0x40, 0}, /* set dphy_auto_enable */
	{0x000d, 0x24, 1}, /* re-write bias */
	{0x3500, 0x01, 0}, /* bypass actdcs*/
	{0x3600, 0x00, 0}, /* shist off */
	{0x3300, 0x01, 0}, /* bypass crop */
	{0x3043, 0x01, 0}, /* bypass esp */
	{0x001c, 0x08, 0}, /* low sensitivity */
	{0x001e, 0x00, 0}, /* low sensitivity */
	{0x3255, 0x0c, 0},
	{0x000f, 0x05, 0},
	{0x301e, 0x00, 0}, /* clr para_out_en_r */
	{0x3010, 0x01, 0}, /* PLL_S : 1000MHz --> 500MHz */
	{0x3924, 0x08, 0}, /* lpx(0), band(8) */
	{0x3000, 0x02, 1}, /* set dvs mode to active mode*/
	{0x3266, 0x08, 0}, /* boot sequence */
	{0x3255, 0x0c, 0}, /* enable global hold and global reset */
	{0x000f, 0x07, 0},
	{0x0010, 0xff, 0},
	{0x0013, 0x03, 0}, /* default value : 0x01 */
	{0x0014, 0x7f, 0}, /* default value : 0x02 */
	{0x0015, 0x00, 0}, /* disable optical black */
	{0x000c, 0x14, 0},
	{0x000b, 0x06, 0},
	{0x0012, 0x7d, 0},
	{0x001c, 0x00, 0}, /* Dummy NUM ON, Default : 04 */
	{0x001e, 0x08, 0}, /* Dummy NUM nOFF, Default : 04*/
	{0x3248, 0x00, 0},
	{0x323c, 0x04, 0},
	{0x3240, 0x00, 0},
	{0x3241, 0x00, 0},
	{0x3242, 0x02, 0},
	{0x3243, 0x32, 0},
	{0x3244, 0x00, 0},
	{0x3245, 0x00, 0},
	{0x3246, 0x00, 0},
	{0x3247, 0x32, 0},
	{0x324b, 0x0a, 0},
	{0x324c, 0x08, 0},
	{0x324d, 0x14, 0},
	{0x324e, 0x04, 0},
	{0x324f, 0x08, 0},
	{0x3251, 0x0f, 0},
	{0x3253, 0x0c, 0},
	{0x3254, 0x10, 0},
	{0x3256, 0x00, 0},
	{0x3257, 0x00, 0},
	{0x3258, 0x01, 0},
	{0x3259, 0x32, 1},
	{0x3238, 0x01, 0}, /* reset timestamp */
	{0x3238, 0x00, 0}, /* restart timestamp */
	{0x390b, 0x08, 0}, /* set use_big_endian */
	{0x3901, 0x1e, 0}, /* set sci2_header : id(yuv422-8bit) */
	{0x391a, 0x10, 0}, /* set frm_cnt *CHECK* */
	{0x3900, 0xF0, 0}, /* set mipi_enable | 4-lane | use_fram */
#else
	{0x3908, 0x40, 0}, /* set dphy_auto_enable */
	{0x000d, 0x24, 1}, /* re-write bias */
	{0x3500, 0x01, 0}, /* bypass actdcs*/
	{0x3600, 0x00, 0}, /* shist off */
	{0x3300, 0x01, 0}, /* bypass crop */
	{0x3043, 0x01, 0}, /* bypass esp */
	{0x001c, 0x08, 0}, /* low sensitivity */
	{0x001e, 0x00, 0}, /* low sensitivity */
	{0x000f, 0x05, 0},
	{0x301e, 0x00, 0}, /* clr para_out_en_r */
	{0x300d, 0x03, 0},
	{0x300e, 0x00, 0},
	{0x300f, 0x5a, 0},
	{0x3010, 0x00, 0}, /* PLL_S : 0 */
	{0x3011, 0xa0, 0}, /* change clock freq: 1000 -> 500 MHz for 2-lane */
	{0x3000, 0x02, 0}, /* Set DVS_MODE to Active Mode */
	{0x3924, 0x0a, 1}, /* lpx(0), band(a) */
	{0x3266, 0x08, 0}, /* boot sequence */
	{0x0010, 0xff, 0},
	{0x0013, 0x03, 0}, /* default value : 0x01 */
	{0x0014, 0x7f, 0}, /* default value : 0x02 */
	{0x0015, 0x00, 0}, /* disable optical black */
	{0x000c, 0x14, 0},
	{0x3255, 0x0d, 0}, /* enable global hold and global reset */
	{0x000f, 0x05, 0},
	{0x0013, 0x03, 0}, /* Enable Bias Buffer */
	{0x0014, 0x7f, 0}, /* Enable Bias Buffer */
	{0x000d, 0x04, 0},
	{0x000b, 0x04, 0}, /* ON MSB */
	{0x001c, 0x07, 0}, /* ON LSB */
	{0x0018, 0x00, 0}, /* AMP */
	{0x0012, 0x7d, 0}, /* nOFF MSB */
	{0x001e, 0x06, 1}, /* nOFF LSB */
	{0x3238, 0x01, 0}, /* reset timestamp */
	{0x3238, 0x00, 0}, /* restart timestamp */
	{0x390b, 0x08, 0}, /* set use_big_endian */
	{0x3922, 0x80, 0}, /* strength default = 0 */
	{0x3923, 0x01, 0}, /* strength default = 0 */
	{0x3901, 0x1e, 0}, /* set sci2_header : id(yuv422-8bit) */
	{0x391a, 0x10, 0}, /*  set frm_cnt *CHECK* */
	{0x3900, 0xb0, 0}, /* set mipi_enable | 2-lane | use_fram */
	{0x3234, 0x1d, 0}, /* Timestamp */
	{0x323d, 0x64, 0}, /* FINE_MAX_CNT */
	{0x3240, 0x09, 0}, /* ED (31,333.333 usec) */
	{0x3241, 0x01, 0}, /* ED */
	{0x3242, 0x8f, 0}, /* ED */
	{0x3243, 0x64, 0}, /* ED */
	{0x325c, 0x01, 0}, /* READ_FIXED (ON) / 0:OFF, 1:ON */
	{0x325d, 0xea, 0}, /* READ_FIXED (2,000 usec) */
	{0x325e, 0x60, 0}, /* READ_FIXED */
	{0x3248, 0x00, 0}, /* GRS (on Detection time) */	
	{0x3244, 0x00, 0}, /* GH2GRS (1CLK) */ 
	{0x3245, 0x00, 0}, /* GH2GRS */
	{0x3246, 0x00, 0}, /* GH2GRS */
	{0x3247, 0x01, 0}, /* GH2GRS */
	{0x3256, 0x00, 0}, /* GRS (2CLK) */
	{0x3257, 0x00, 0}, /* GRS */
	{0x3258, 0x00, 0}, /* GRS */
	{0x3259, 0x02, 0}, /* GRS */
	{0x3255, 0x0d, 0}, /* G.HOLD[0](ON), G.RST[1](OFF) (0C:OFF/OFF, 0D:ON/OFF, OE:OFF/ON, 0F:ON/ON) */
	{0x323c, 0x03, 0}, /* GH2SEL (100ns) */
	{0x324b, 0x06, 0}, /* NEXT_GH (200ns */
	{0x324c, 0x06, 0}, /* SELW (200ns */
	{0x324e, 0x03, 0}, /* SEL2AY_r (100ns) */
	{0x324f, 0x06, 0}, /* SEL2AY_f (200ns) */
	{0x3253, 0x09, 0}, /* SEL2R_r (300ns) */
	{0x3254, 0x0c, 0}, /* SEL2R_f (400ns) */
	{0x3261, 0x00, 0}, /* NEXT_SEL (500ns) */
	{0x3262, 0x0f, 0}, /* NEXT_SEL */
	{0x3251, 0x0a, 0}, /* NEXT_SEL-5 */
	{0x3000, 0x02, 0}, /* set DVS_MODE to Active Mode */
#endif
};

/*
 * if you want to force all the events to be on,
 * put this at the end of the table that you're using
 * {0x000d, 0x04, 0},
 * {0x0015, 0x00, 0},
 * {0x0008, 0x3b, 0},
 * {0x000b, 0x04, 0},
 * {0x001c, 0x00, 0},
 * {0x0012, 0x7d, 0},
 * {0x0014, 0x7f, 0},
 * {0x001e, 0x00, 0},
 * {0x3255, 0x0d, 0},
 */

/* SR shared this setting with us */
static const struct reg_value s5krd4_init_setting_vga_500_4lane_60hz[] = {
	{0x3900, 0xf0, 0}, /* 20:3900=F0  MIPI 4-lane */
	{0x3010, 0x01, 0}, /* 20:3010=01  PLL_S: 1000MHz --> 500MHz */
	{0x3234, 0x31, 0}, /* 20:3234=31  Timestamp (50MHz Operation Clock) */
	{0x323d, 0x32, 0}, /* 20:323D=32  Fine Count (50MHz Operation Clock) */
	{0x391a, 0x10, 0}, /* set frm_cnt *CHECK* */
	{0x3240, 0x0f, 0}, /* 20:3240=0F  ED (15707us) FRAME_RATE */
	{0x3241, 0x02, 0}, /* 20:3241=02  ED */
	{0x3242, 0xc2, 0}, /* 20:3242=C2  ED */
	{0x3243, 0x32, 0}, /* 20:3243=32  ED */
	{0x325c, 0x01, 0}, /* 20:325C=01  READ_FIXED (ON) */
	{0x325d, 0xbb, 0}, /* 20:325D=BB  READ_FIXED (960us) */
	{0x325e, 0x80, 0}, /* 20:325E=80  READ_FIXED  T = 15707 + 960 = 16667(usec) <--> 60FPS */
	{0x3248, 0x00, 0}, /* 20:3248=00  GRS (on Detection Time) GLOBAL_RESET */
	{0x3244, 0x00, 0}, /* 20:3244=00  GH2GRS (1us) */
	{0x3245, 0x00, 0}, /* 20:3245=00  GH2GRS */
	{0x3246, 0x00, 0}, /* 20:3246=00  GH2GRS */
	{0x3247, 0x32, 0}, /* 20:3247=32  GH2GRS */
	{0x3256, 0x00, 0}, /* 20:3256=00  GRS (2us) */
	{0x3257, 0x00, 0}, /* 20:3257=00  GRS */
	{0x3258, 0x01, 0}, /* 20:3258=01  GRS */
	{0x3259, 0x32, 0}, /* 20:3259=32  GRS */
	{0x3255, 0x0d, 0}, /* 20:3255=0D  G.HOLD(ON), G.RST(OFF) */
	{0x323c, 0x04, 0}, /* 20:323C=04  GH2SEL (80ns) READ_TIMING */
	{0x324b, 0x0a, 0}, /* 20:324B=0A  NEXT_GH (200ns) */
	{0x324c, 0x06, 0}, /* 20:324C=06  SELW (120ns) */
	{0x324e, 0x04, 0}, /* 20:324E=04  SEL2AY_r (80ns) */
	{0x324f, 0x06, 0}, /* 20:324F=06  SEL2AY_f (120ns) */
	{0x3253, 0x08, 0}, /* 20:3253=08  SEL2R_r (160ns) */
	{0x3254, 0x0a, 0}, /* 20:3254=0A  SEL2R_f (200ns) */
	{0x3261, 0x00, 0}, /* 20:3261=00  NEXT_SEL (300ns) */
	{0x3262, 0x0f, 0}, /* 20:3262=0F  NEXT_SEL */
	{0x3251, 0x0a, 0}, /* 20:3251=0A  NEXT_SEL-5 */
	{0x000b, 0x04, 0}, /* 20:000B=04  ON_MSB [1] = 0  Sensitivity (MAX) */
	{0x001c, 0x00, 0}, /* 20:001C=00  ON_LSB [3:0] = 0 (8~0) */
	{0x0012, 0x7f, 0}, /* 20:0012=7F  OFF_MSB [1] = 1 */
	{0x001e, 0x08, 0}, /* 20:001E=08  OSS_LSB [3:0] = 8 (0~8) */
};

static inline struct s5k231y_state *to_state
	(struct v4l2_subdev *subdev)
{
	struct s5k231y_state *state =
		container_of(subdev, struct s5k231y_state, subdev);
	return state;
}

static inline struct i2c_client *to_client
	(struct v4l2_subdev *subdev)
{
	return (struct i2c_client *)v4l2_get_subdevdata(subdev);
}

static int sensor_231y_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

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

static int sensor_231y_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

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

static int sensor_231y_load_regs(struct v4l2_subdev *subdev,
			    const struct reg_value *regs, int size)
{
	unsigned int i;
	u32 delay_ms;
	u16 reg_addr;
	u8 val;
	int ret = 0;
	struct i2c_client *client = to_client(subdev);

	for (i = 0; i < size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;

		ret = sensor_231y_write_reg(client, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000*delay_ms, 1000*delay_ms+100);
	}

	return ret;
}

static int sensor_231y_init(struct v4l2_subdev *subdev, u32 val)
{
	int ret = 0;
	struct i2c_client *client = to_client(subdev);
	u8 id = 0;

	WARN_ON(!subdev);

	dev_info(&client->dev, "%s start\n", __func__);

	ret = sensor_231y_read_reg(client, 0x3052, &id);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read id\n");
		printk(KERN_ALERT "failed to read id\n");
		goto p_err;
	}

	dev_info(&client->dev, "id=0x%02x\n", id);

	/* init member */
	ret = sensor_231y_load_regs(subdev, s5k231y_init_setting_vga_500,
				ARRAY_SIZE(s5k231y_init_setting_vga_500));
	if (ret < 0) {
		printk(KERN_ALERT "init_reg failed\n");
		goto p_err;
	}

	ret = sensor_231y_read_reg(client, 0x001c, &id);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read id\n");
		goto p_err;
	}
	dev_info(&client->dev, "id=0x%02x\n", id);
	dev_info(&client->dev, "%s end\n", __func__);

	msleep(100);

p_err:
	return ret;
}

static int sensor_231y_s_stream(struct v4l2_subdev *subdev,
	int enable)
{
	int ret = 0;
	struct i2c_client *client = to_client(subdev);

	dev_info(&client->dev, "%s %d\n", __func__, enable);

	if (enable) {
		ret = sensor_231y_init(subdev, 1);
		if (ret) {
			dev_err(&client->dev, "stream_on is fail(%d)", ret);
			goto p_err;
		}
	}

p_err:
	return ret;
}

static int sensor_231y_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int sensor_231y_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int sensor_231y_subdev_registered(struct v4l2_subdev *sd)
{
	return 0;
}

static void sensor_231y_subdev_unregistered(struct v4l2_subdev *sd)
{
}

static int sensor_231y_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int sensor_231y_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int sensor_231y_enum_fsize(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *frame)
{
	if (frame->index >= ARRAY_SIZE(supported_resolutions))
		return -ENODEV;

	frame->max_width = supported_resolutions[frame->index].width;
	frame->max_height = supported_resolutions[frame->index].height;

	return 0;
}

static int sensor_231y_enum_finterval(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *frame)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_resolutions); i++) {
		if ((frame->width == supported_resolutions[i].width) &&
		    (frame->height == supported_resolutions[i].height)) {
			frame->interval.numerator = 1;
			frame->interval.denominator =
				supported_resolutions[i].interval[frame->index];
			return false;
		}
	}
	return -EINVAL;
}

static struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt		= sensor_231y_s_fmt,
	.enum_frame_size	= sensor_231y_enum_fsize,
	.enum_frame_interval	= sensor_231y_enum_finterval,
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init		= sensor_231y_init,
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = sensor_231y_s_stream,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad	= &pad_ops,
};

static const struct v4l2_subdev_internal_ops internal_ops = {
	.open = sensor_231y_subdev_open,
	.close = sensor_231y_subdev_close,
	.registered = sensor_231y_subdev_registered,
	.unregistered = sensor_231y_subdev_unregistered,
};

static const struct media_entity_operations media_ops = {
	.link_setup = sensor_231y_link_setup,
};

int sensor_231y_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct v4l2_subdev *subdev_module;
	struct s5k231y_state *s5k231y_state;

	s5k231y_state = devm_kzalloc(&client->dev, sizeof(struct s5k231y_state),
		GFP_KERNEL);
	if (!s5k231y_state) {
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_module = &s5k231y_state->subdev;
	snprintf(subdev_module->name, V4L2_SUBDEV_NAME_SIZE,
		"%s", SENSOR_NAME);

	v4l2_i2c_subdev_init(subdev_module, client, &subdev_ops);

	s5k231y_state->pad.flags = MEDIA_PAD_FL_SOURCE;

	subdev_module->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	subdev_module->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	subdev_module->internal_ops = &internal_ops;
	subdev_module->entity.ops = &media_ops;

	ret = media_entity_init(&subdev_module->entity, 1,
		&s5k231y_state->pad, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s, failed\n", __func__);
		return ret;
	}

	s5k231y_state->client = client;

p_err:
	dev_info(&client->dev, "(%d)\n", ret);
	return ret;
}

static int sensor_231y_remove(struct i2c_client *client)
{
	int ret = 0;
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	return ret;
}

static const struct i2c_device_id sensor_231y_idt[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

static struct i2c_driver sensor_231y_driver = {
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= sensor_231y_probe,
	.remove	= sensor_231y_remove,
	.id_table = sensor_231y_idt
};

module_i2c_driver(sensor_231y_driver);

MODULE_AUTHOR("Hwansoon Sung <hs2704.sung@samsung.com>");
MODULE_DESCRIPTION("Sensor 231y driver");
MODULE_LICENSE("GPL v2");
