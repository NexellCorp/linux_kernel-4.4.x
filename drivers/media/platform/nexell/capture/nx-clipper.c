/*
 * Copyright (C) 2016  Nexell Co., Ltd.
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
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/irqreturn.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/semaphore.h>
#include <linux/v4l2-mediabus.h>
#include <linux/i2c.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <dt-bindings/media/nexell-vip.h>
#include <linux/kthread.h>

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
#include <linux/pm_qos.h>
#include <linux/soc/nexell/cpufreq.h>
#endif

#include "../nx-v4l2.h"
#include "../nx-video.h"
#include "nx-vip-primitive.h"
#include "nx-vip.h"

#define NX_CLIPPER_DEV_NAME	"nx-clipper"

/* #define DEBUG_SYNC */
#ifdef DEBUG_SYNC
#include <linux/timer.h>

#define DEBUG_SYNC_TIMEOUT_MS	(1000)
#endif

#ifdef CONFIG_CLIPPER_USE_DQTIMER
#include <linux/timer.h>
#include <linux/delay.h>
#define DQ_TIMEOUT_MS		CONFIG_CLIPPER_DQTIMER_TIMEOUT
#endif

#define DEFAULT_DUTY_CYCLE	50
#define NS_IN_HZ (1000000000UL)
#define TO_PERIOD_NS(freq)	(NS_IN_HZ/(freq))
#define TO_DUTY_NS(duty, freq)  (duty ? TO_PERIOD_NS(freq)/(100/duty) : 0)

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
struct task_struct *g_ClipperThread;
#endif

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
static struct pm_qos_request nx_clipper_qos;

static void nx_clipper_qos_update(int val)
{
	if (!pm_qos_request_active(&nx_clipper_qos))
		pm_qos_add_request(&nx_clipper_qos, PM_QOS_BUS_THROUGHPUT, val);
	else
		pm_qos_update_request(&nx_clipper_qos, val);
}

static struct pm_qos_request nx_clipper_qos_cpu_online;

static void nx_clipper_qos_cpu_online_update(int val)
{
	if (!pm_qos_request_active(&nx_clipper_qos_cpu_online))
		pm_qos_add_request(&nx_clipper_qos_cpu_online,
			PM_QOS_CPU_ONLINE_MIN, val);
	else
		pm_qos_update_request(&nx_clipper_qos_cpu_online, val);
}
#endif

enum {
	NX_CLIPPER_PAD_SINK,
	NX_CLIPPER_PAD_SOURCE_MEM,
	NX_CLIPPER_PAD_SOURCE_DECIMATOR,
	NX_CLIPPER_PAD_MAX
};

struct gpio_action_unit {
	int value;
	int delay_ms;
};

struct nx_capture_enable_gpio_action {
	int gpio_num;
	int count;
	/* alloc dynamically by count */
	struct gpio_action_unit *units;
};

struct nx_capture_enable_pmic_action {
	int enable;
	int delay_ms;
};

struct nx_capture_enable_clock_action {
	int enable;
	int delay_ms;
};

struct nx_capture_power_action {
	int type;
	/**
	 * nx_capture_enable_gpio_action or
	 * nx_capture_enable_pmic_action or
	 * nx_capture_enable_clock_action
	 */
	void *action;
};

struct nx_capture_power_seq {
	int count;
	/* alloc dynamically by count */
	struct nx_capture_power_action *actions;
};

struct nx_v4l2_i2c_board_info {
	int     i2c_adapter_id;
	int	i2c_addr;
	struct i2c_board_info board_info;
};

enum {
	STATE_IDLE = 0,
	STATE_MEM_RUNNING  = (1 << 0),
	STATE_CLIP_RUNNING = (1 << 1),
	STATE_MEM_STOPPING = (1 << 2),
};

struct nx_clipper {
	u32 module;
	u32 logical;
	u32 logical_num;
	u32 interface_type;
	s32 clk_src;
	u32 clk_freq;
	u32 external_sync;
	u32 padclk_sel;
	u32 h_syncpolarity;
	u32 v_syncpolarity;
	u32 h_frontporch;
	u32 h_syncwidth;
	u32 h_backporch;
	u32 v_frontporch;
	u32 v_backporch;
	u32 v_syncwidth;
	u32 clock_invert;
	u32 port;
	u32 interlace;
	u32 irq_count;
	int regulator_nr;
	char **regulator_names;
	u32 *regulator_voltages;
	struct pwm_device *pwm;
	u32 clock_rate;
	struct nx_capture_power_seq enable_seq;
	struct nx_capture_power_seq disable_seq;
	struct nx_v4l2_i2c_board_info sensor_info;

	struct v4l2_subdev subdev;
	struct media_pad pads[NX_CLIPPER_PAD_MAX];

	struct v4l2_rect crop;
	u32 bus_fmt; /* data_order */
	u32 width;
	u32 height;

	struct nx_dma_buf buf;

	atomic_t state;
	struct completion stop_done;
	struct semaphore s_stream_sem;
	bool sensor_enabled;
	struct media_pad sensor_pad;


	struct platform_device *pdev;

	struct tasklet_struct work;
	struct list_head done_bufs;
	struct nx_video_buffer_object vbuf_obj;
	struct nx_v4l2_irq_entry *irq_entry;
	u32 mem_fmt;
	bool buffer_underrun;

#ifdef DEBUG_SYNC
	struct timer_list timer;
#endif

#ifdef CONFIG_CLIPPER_USE_DQTIMER
	struct timer_list dq_timer;
	spinlock_t lock;
#endif

	/* for suspend */
	struct nx_video_buffer *last_buf;

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
	struct workqueue_struct *w_queue;
	struct delayed_work w_delay;
#endif
};

static int register_irq_handler(struct nx_clipper *me);

#ifdef DEBUG_SYNC
/* DEBUG_SYNC */
static void debug_sync(unsigned long priv)
{
	struct nx_clipper *me = (struct nx_clipper *)priv;

	dev_err(&me->pdev->dev, "VCOUNT: %d, HCOUNT: %d\n",
		nx_vip_get_ver_count(me->module),
		nx_vip_get_hor_count(me->module));

	mod_timer(&me->timer,
		jiffies + msecs_to_jiffies(DEBUG_SYNC_TIMEOUT_MS));
}
#endif

/**
 * parse device tree
 */
static int parse_sensor_i2c_board_info_dt(struct device_node *np,
				      struct device *dev, struct nx_clipper *me)
{
	const char *name;
	u32 adapter;
	u32 addr;
	struct nx_v4l2_i2c_board_info *info = &me->sensor_info;

	if (of_property_read_string(np, "i2c_name", &name)) {
		dev_err(dev, "failed to get sensor i2c name\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "i2c_adapter", &adapter)) {
		dev_err(dev, "failed to get sensor i2c adapter\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "addr", &addr)) {
		dev_err(dev, "failed to get sensor i2c addr\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "real_addr", &info->i2c_addr))
				info->i2c_addr = 0;

	strlcpy(info->board_info.type, name, sizeof(info->board_info.type));
	info->board_info.addr = addr;
	info->i2c_adapter_id = adapter;

	return 0;
}

static int parse_sensor_dt(struct device_node *np, struct device *dev,
			   struct nx_clipper *me)
{
	int ret;
	u32 type;

	if (of_property_read_u32(np, "type", &type)) {
		dev_err(dev, "failed to get dt sensor type\n");
		return -EINVAL;
	}

	if (type == NX_CAPTURE_SENSOR_I2C) {
		ret = parse_sensor_i2c_board_info_dt(np, dev, me);
		if (ret)
			return ret;
	} else {
		dev_err(dev, "sensor type %d is not supported\n",
			type);
		return -EINVAL;
	}

	return 0;
}

static int find_action_mark(u32 *p, int length, u32 mark)
{
	int i;

	for (i = 0; i < length; i++) {
		if (p[i] == mark)
			return i;
	}
	return -1;
}

static int find_action_start(u32 *p, int length)
{
	return find_action_mark(p, length, NX_ACTION_START);
}

static int find_action_end(u32 *p, int length)
{
	return find_action_mark(p, length, NX_ACTION_END);
}

static int get_num_of_enable_action(u32 *array, int count)
{
	u32 *p = array;
	int action_num = 0;
	int next_index = 0;
	int length = count;

	while (length > 0) {
		next_index = find_action_start(p, length);
		if (next_index < 0)
			break;
		p += next_index;
		length -= next_index;
		if (length <= 0)
			break;

		next_index = find_action_end(p, length);
		if (next_index <= 0) {
			pr_err("failed to find_action_end, check power node of dts\n");
			return 0;
		}
		p += next_index;
		length -= next_index;
		action_num++;
	}

	return action_num;
}

static u32 *get_next_action_unit(u32 *array, int count)
{
	u32 *p = array;
	int next_index = find_action_start(p, count);

	if (next_index >= 0)
		return p + next_index;
	return NULL;
}

static u32 get_action_type(u32 *array)
{
	return array[1];
}

static int make_enable_gpio_action(u32 *start, u32 *end,
				   struct nx_capture_power_action *action)
{
	struct nx_capture_enable_gpio_action *gpio_action;
	struct gpio_action_unit *unit;
	int i;
	u32 *p;
	/* start_marker, type, gpio num */
	int unit_count = end - start - 1 - 1 - 1;

	if ((unit_count <= 0) || (unit_count % 2)) {
		pr_err("invalid unit_count %d of gpio action\n", unit_count);
		return -EINVAL;
	}
	unit_count /= 2;

	gpio_action = kzalloc(sizeof(*gpio_action), GFP_KERNEL);
	if (!gpio_action) {
		WARN_ON(1);
		return -ENOMEM;
	}

	gpio_action->count = unit_count;
	gpio_action->units = kcalloc(unit_count, sizeof(*unit), GFP_KERNEL);
	if (!gpio_action->units) {
		WARN_ON(1);
		kfree(gpio_action);
		return -ENOMEM;
	}

	gpio_action->gpio_num = start[2];

	p = &start[3];
	for (i = 0; i < unit_count; i++) {
		unit = &gpio_action->units[i];
		unit->value = *p;
		p++;
		unit->delay_ms = *p;
		p++;
	}

	action->type = NX_ACTION_TYPE_GPIO;
	action->action = gpio_action;

	return 0;
}

static int make_enable_pmic_action(u32 *start, u32 *end,
				   struct nx_capture_power_action *action)
{
	struct nx_capture_enable_pmic_action *pmic_action;
	int entry_count = end - start - 1 - 1; /* start_marker, type */

	if ((entry_count <= 0) || (entry_count % 2)) {
		pr_err("invalid entry_count %d of pmic action\n", entry_count);
		return -EINVAL;
	}

	pmic_action = kzalloc(sizeof(*pmic_action), GFP_KERNEL);
	if (!pmic_action) {
		WARN_ON(1);
		return -ENOMEM;
	}

	pmic_action->enable = start[2];
	pmic_action->delay_ms = start[3];

	action->type = NX_ACTION_TYPE_PMIC;
	action->action = pmic_action;

	return 0;
}

static int make_enable_clock_action(u32 *start, u32 *end,
				    struct nx_capture_power_action *action)
{
	struct nx_capture_enable_clock_action *clock_action;
	int entry_count = end - start - 1 - 1; /* start_marker, type */

	if ((entry_count <= 0) || (entry_count % 2)) {
		pr_err("invalid entry_count %d of clock action\n", entry_count);
		return -EINVAL;
	}

	clock_action = kzalloc(sizeof(*clock_action), GFP_KERNEL);
	if (!clock_action) {
		WARN_ON(1);
		return -ENOMEM;
	}

	clock_action->enable = start[2];
	clock_action->delay_ms = start[3];

	action->type = NX_ACTION_TYPE_CLOCK;
	action->action = clock_action;

	return 0;
}

static int make_enable_action(u32 *array, int count,
			      struct nx_capture_power_action *action)
{
	u32 *p = array;
	int end_index = find_action_end(p, count);

	if (end_index <= 0) {
		pr_err("parse dt for v4l2 capture error: can't find action end\n");
		return -EINVAL;
	}

	switch (get_action_type(p)) {
	case NX_ACTION_TYPE_GPIO:
		return make_enable_gpio_action(p, p + end_index, action);
	case NX_ACTION_TYPE_PMIC:
		return make_enable_pmic_action(p, p + end_index, action);
	case NX_ACTION_TYPE_CLOCK:
		return make_enable_clock_action(p, p + end_index, action);
	default:
		pr_err("parse dt v4l2 capture: invalid type 0x%x\n",
		       get_action_type(p));
		return -EINVAL;
	}
}

static void free_enable_seq_actions(struct nx_capture_power_seq *seq)
{
	int i;
	struct nx_capture_power_action *action;

	for (i = 0; i < seq->count; i++) {
		action = &seq->actions[i];
		if (action->action) {
			if (action->type == NX_ACTION_TYPE_GPIO) {
				struct nx_capture_enable_gpio_action *
					gpio_action = action->action;
				kfree(gpio_action->units);
			}
			kfree(action->action);
		}
	}

	kfree(seq->actions);
	seq->count = 0;
	seq->actions = NULL;
}

static int parse_capture_power_seq(struct device_node *np,
				   char *node_name,
				   struct nx_capture_power_seq *seq)
{
	int count = of_property_count_elems_of_size(np, node_name, 4);

	if (count > 0) {
		u32 *p;
		int ret = 0;
		struct nx_capture_power_action *action;
		int action_nums;
		u32 *array = kcalloc(count, sizeof(u32), GFP_KERNEL);

		if (!array) {
			WARN_ON(1);
			return -ENOMEM;
		}

		of_property_read_u32_array(np, node_name, array, count);
		action_nums = get_num_of_enable_action(array, count);
		if (action_nums <= 0) {
			pr_err("parse_dt v4l2 capture: no actions in %s\n",
			       node_name);
			return -ENOENT;
		}

		seq->actions = kcalloc(count, sizeof(*action), GFP_KERNEL);
		if (!seq->actions) {
			WARN_ON(1);
			return -ENOMEM;
		}
		seq->count = action_nums;

		p = array;
		action = seq->actions;
		while (action_nums--) {
			p = get_next_action_unit(p, count - (p - array));
			if (!p) {
				pr_err("failed to get_next_action_unit(%d/%d)\n",
				       seq->count, action_nums);
				free_enable_seq_actions(seq);
				return -EINVAL;
			}

			ret = make_enable_action(p, count - (p - array),
						 action);
			if (ret != 0) {
				free_enable_seq_actions(seq);
				return ret;
			}

			p++;
			action++;
		}
	}

	return 0;
}

static int parse_power_dt(struct device_node *np, struct device *dev,
			  struct nx_clipper *me)
{
	int ret = 0;

	if (of_find_property(np, "enable_seq", NULL))
		ret = parse_capture_power_seq(np, "enable_seq",
					      &me->enable_seq);

	if (ret)
		return ret;

	if (of_find_property(np, "disable_seq", NULL))
		ret = parse_capture_power_seq(np, "disable_seq",
					      &me->disable_seq);

	return ret;
}

static int parse_clock_dt(struct device_node *np, struct device *dev,
			  struct nx_clipper *me)
{
	me->pwm = devm_of_pwm_get(dev, np, NULL);
	if (!IS_ERR(me->pwm)) {
		unsigned int period = pwm_get_period(me->pwm);
		unsigned int duty_cycle =
			TO_DUTY_NS(DEFAULT_DUTY_CYCLE, period);

		pwm_config(me->pwm, duty_cycle, TO_PERIOD_NS(period));
		dev_info(dev, "[%s] name:%s, period:%d, duty_cycle:%d\n",
				__func__, me->pwm->label, me->pwm->period,
				me->pwm->duty_cycle);
		pwm_enable(me->pwm);
	} else
		me->pwm = NULL;

	return 0;
}

static int nx_clipper_parse_dt(struct device *dev, struct nx_clipper *me)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct device_node *child_node = NULL;

	if (of_property_read_u32(np, "module", &me->module)) {
		dev_err(dev, "failed to get dt module\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "logical", &me->logical))
		me->logical = 0;

	if (me->logical == 1) {
		if (of_property_read_u32(np, "logical_num", &me->logical_num)) {
			dev_err(dev, "failed to get dt logical_num\n");
			return -EINVAL;
		}
	}

	if (of_property_read_u32(np, "interface_type", &me->interface_type)) {
		dev_err(dev, "failed to get dt interface_type\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "clock_source", &me->clk_src))
		me->clk_src = -1;

	if (of_property_read_u32(np, "clock_frequency", &me->clk_freq))
		me->clk_freq = 0;

	if (me->interface_type == NX_CAPTURE_INTERFACE_MIPI_CSI) {
		/* mipi use always same config, so ignore user config */
		if (me->module != 0) {
			dev_err(dev, "module of mipi-csi must be 0 but, current %d\n",
				me->module);
			return -EINVAL;
		}
		me->port = 1;
		me->padclk_sel = 0;
		me->h_syncpolarity = 0;
		me->v_syncpolarity = 0;
#ifdef CONFIG_ARCH_S5P4418
		me->h_frontporch = 8;
		me->h_syncwidth = 7;
		me->h_backporch = 7;
		me->v_frontporch = 1;
		me->v_syncwidth = 8;
		me->v_backporch = 1;
		me->clock_invert = 0;
#else
		me->h_frontporch = 4;
		me->h_syncwidth = 4;
		me->h_backporch = 4;
		me->v_frontporch = 1;
		me->v_syncwidth = 1;
		me->v_backporch = 1;
		me->clock_invert = 0;
#endif
		me->bus_fmt = NX_VIN_CBY0CRY1;
		me->interlace = 0;
	} else {
		if (of_property_read_u32(np, "port", &me->port)) {
			dev_err(dev, "failed to get dt port\n");
			return -EINVAL;
		}

		if (of_property_read_u32(np, "external_sync",
					 &me->external_sync)) {
			dev_err(dev, "failed to get dt external_sync\n");
			return -EINVAL;
		}

		if (me->external_sync == 0) {
			/* when 656, porch value is always same, so ignore user
			 * config
			 */
			me->padclk_sel = 0;
			me->h_syncpolarity = 0;
			me->v_syncpolarity = 0;
			me->h_frontporch = 7;
			me->h_syncwidth = 1;
			me->h_backporch = 10;
			me->v_frontporch = 0;
			me->v_syncwidth = 2;
			me->v_backporch = 3;
		} else {
			if (of_property_read_u32(np, "padclk_sel",
						 &me->padclk_sel))
				me->padclk_sel = 0;
			if (of_property_read_u32(np, "h_syncpolarity",
						 &me->h_syncpolarity))
				me->h_syncpolarity = 0;
			if (of_property_read_u32(np, "v_syncpolarity",
						 &me->v_syncpolarity))
				me->v_syncpolarity = 0;
			if (of_property_read_u32(np, "h_frontporch",
						 &me->h_frontporch)) {
				dev_err(dev, "failed to get dt h_frontporch\n");
				return -EINVAL;
			}
			if (of_property_read_u32(np, "h_syncwidth",
						 &me->h_syncwidth)) {
				dev_err(dev, "failed to get dt h_syncwidth\n");
				return -EINVAL;
			}
			if (of_property_read_u32(np, "h_backporch",
						 &me->h_backporch)) {
				dev_err(dev, "failed to get dt h_backporch\n");
				return -EINVAL;
			}
			if (of_property_read_u32(np, "v_frontporch",
						 &me->v_frontporch)) {
				dev_err(dev, "failed to get dt v_frontporch\n");
				return -EINVAL;
			}
			if (of_property_read_u32(np, "v_syncwidth",
						 &me->v_syncwidth)) {
				dev_err(dev, "failed to get dt v_syncwidth\n");
				return -EINVAL;
			}
			if (of_property_read_u32(np, "v_backporch",
						 &me->v_backporch)) {
				dev_err(dev, "failed to get dt v_backporch\n");
				return -EINVAL;
			}
		}

		/* optional */
		of_property_read_u32(np, "interlace", &me->interlace);
	}

	/* common property */
	if (of_property_read_u32(np, "data_order", &me->bus_fmt)) {
		dev_err(dev, "failed to get dt data_order\n");
		return -EINVAL;
	}

	me->regulator_nr = of_property_count_strings(np, "regulator_names");
	if (me->regulator_nr > 0) {
		int i;
		const char *name;
		me->regulator_names = devm_kcalloc(dev,
						   me->regulator_nr,
						   sizeof(char *),
						   GFP_KERNEL);
		if (!me->regulator_names) {
			WARN_ON(1);
			return -ENOMEM;
		}

		me->regulator_voltages = devm_kcalloc(dev,
						      me->regulator_nr,
						      sizeof(u32),
						      GFP_KERNEL);
		if (!me->regulator_voltages) {
			WARN_ON(1);
			return -ENOMEM;
		}


		for (i = 0; i < me->regulator_nr; i++) {
			if (of_property_read_string_index(np, "regulator_names",
							  i, &name)) {
				dev_err(&me->pdev->dev, "failed to read regulator %d name\n", i);
				return -EINVAL;
			}
			me->regulator_names[i] = (char *)name;
		}

		of_property_read_u32_array(np, "regulator_voltages",
					   me->regulator_voltages,
					   me->regulator_nr);
	}

	child_node = of_find_node_by_name(np, "sensor");
	if (!child_node) {
		dev_err(dev, "failed to get sensor node\n");
		return -EINVAL;
	}
	ret = parse_sensor_dt(child_node, dev, me);
	if (ret)
		return ret;

	child_node = of_find_node_by_name(np, "power");
	if (child_node)  {
		ret = parse_power_dt(child_node, dev, me);
		if (ret)
			return ret;

		ret = parse_clock_dt(child_node, dev, me);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * sensor power enable
 */
static int apply_gpio_action(struct device *dev, int gpio_num,
		  struct gpio_action_unit *unit)
{
	int ret;
	char label[64] = {0, };
	struct device_node *np;
	int gpio;

	np = dev->of_node;
	gpio = of_get_named_gpio(np, "gpios", gpio_num);

	snprintf(label, sizeof(label), "v4l2-cam #pwr gpio %d", gpio);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "invalid gpio %d set to %d\n", gpio, unit->value);
		return -EINVAL;
	}

	ret = devm_gpio_request_one(dev, gpio,
				    unit->value ?
				    GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
				    label);
	if (ret < 0) {
		dev_err(dev, "failed to gpio %d set to %d\n",
			gpio, unit->value);
		return ret;
	}

	if (unit->delay_ms > 0)
		mdelay(unit->delay_ms);

	devm_gpio_free(dev, gpio);

	return 0;
}

static int do_gpio_action(struct nx_clipper *me,
				 struct nx_capture_enable_gpio_action *action)
{
	int ret;
	struct gpio_action_unit *unit;
	int i;

	for (i = 0; i < action->count; i++) {
		unit = &action->units[i];
		ret = apply_gpio_action(&me->pdev->dev, action->gpio_num, unit);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int do_pmic_action(struct nx_clipper *me,
				 struct nx_capture_enable_pmic_action *action)
{
	int ret;
	int i;
	struct regulator *power;

	for (i = 0; i < me->regulator_nr; i++) {
		power = devm_regulator_get(&me->pdev->dev,
					   me->regulator_names[i]);
		if (IS_ERR(power)) {
			dev_err(&me->pdev->dev, "failed to get power %s\n",
				me->regulator_names[i]);
			return -ENODEV;
		}

		if (regulator_can_change_voltage(power)) {
			ret = regulator_set_voltage(power,
						    me->regulator_voltages[i],
						    me->regulator_voltages[i]);
			if (ret) {
				devm_regulator_put(power);
				dev_err(&me->pdev->dev,
					"can't set voltages(index: %d)\n", i);
				return ret;
			}
		}

		ret = 0;
		if (action->enable && !regulator_is_enabled(power)) {
			ret = regulator_enable(power);
		} else if (!action->enable && regulator_is_enabled(power)) {
			ret = regulator_disable(power);
		}

		devm_regulator_put(power);

		if (ret) {
			dev_err(&me->pdev->dev, "failed to power %s %s\n",
				me->regulator_names[i],
				action->enable ? "enable" : "disable");
			return ret;
		}
	}

	return 0;
}

static int do_clock_action(struct nx_clipper *me,
				  struct nx_capture_enable_clock_action *action)
{
	if (me->pwm)
		pwm_enable(me->pwm);

	if (action->delay_ms > 0)
		mdelay(action->delay_ms);

	return 0;
}

static int enable_sensor_power(struct nx_clipper *me, bool enable)
{
	struct nx_capture_power_seq *seq = NULL;

	if (enable && !me->sensor_enabled)
		seq = &me->enable_seq;
	else if (!enable && me->sensor_enabled)
		seq = &me->disable_seq;

	if (seq) {
		int i;
		struct nx_capture_power_action *action;

		for (i = 0; i < seq->count; i++) {
			action = &seq->actions[i];
			switch (action->type) {
			case NX_ACTION_TYPE_GPIO:
				do_gpio_action(me, action->action);
				break;
			case NX_ACTION_TYPE_PMIC:
				do_pmic_action(me, action->action);
				break;
			case NX_ACTION_TYPE_CLOCK:
				do_clock_action(me, action->action);
				break;
			default:
				pr_warn("unknown action type %d\n",
					action->type);
				break;
			}
		}
	}

	me->sensor_enabled = enable;
	return 0;
}

/**
 * buffer operations
 */

static int alloc_dma_buffer(struct nx_clipper *me)
{
	if (me->buf.addr == NULL) {
		struct nx_video_buffer *buf;
		u32 y_size, cbcr_size;

		buf = nx_video_get_next_buffer(&me->vbuf_obj, false);
		if (!buf) {
			dev_warn(&me->pdev->dev, "can't get next buffer\n");
			return -ENOENT;
		}
		if (me->buf.format == MEDIA_BUS_FMT_YVYU12_1X24) {
			y_size = buf->dma_addr[2] - buf->dma_addr[0];
			cbcr_size = buf->dma_addr[1] - buf->dma_addr[2];
		} else {
			y_size = buf->dma_addr[1] - buf->dma_addr[0];
			cbcr_size = buf->dma_addr[2] - buf->dma_addr[1];
		}
		me->buf.size = y_size + (cbcr_size * 2);
		me->buf.addr = dma_alloc_coherent(&me->pdev->dev,
				me->buf.size,
				&me->buf.handle[0], GFP_KERNEL);
		if (me->buf.addr == NULL) {
			dev_warn(&me->pdev->dev,
					"failed to alloc dma buffer\n");
			return -ENOMEM;
		}
		me->buf.stride[0] = buf->stride[0];
		me->buf.stride[1] = buf->stride[1];
		me->buf.handle[1] = me->buf.handle[0] + y_size;
		me->buf.handle[2] = me->buf.handle[1] + cbcr_size;
	}

	return 0;
}

static void free_dma_buffer(struct nx_clipper *me)
{
	if (me->buf.addr) {
		dma_free_coherent(&me->pdev->dev,
				me->buf.size,
				me->buf.addr,
				me->buf.handle[0]);
		me->buf.handle[0] = me->buf.handle[1] = me->buf.handle[2] = 0;
		me->buf.stride[0] = me->buf.stride[1] = 0;
		me->buf.addr = NULL;
	}
}

static int update_buffer(struct nx_clipper *me)
{
	struct nx_video_buffer *buf;

	buf = nx_video_get_next_buffer(&me->vbuf_obj, false);
	if (!buf) {
		dev_warn(&me->pdev->dev, "can't get next buffer\n");
		return -ENOENT;
	}

	nx_vip_set_clipper_addr(me->module, me->mem_fmt,
				me->crop.width, me->crop.height,
				buf->dma_addr[0], buf->dma_addr[1],
				buf->dma_addr[2], buf->stride[0],
				buf->stride[1]);
	me->last_buf = buf;


#ifdef DEBUG_SYNC
	dev_dbg(&me->pdev->dev, "%s: module : %d, crop width : %d\n",
		__func__, me->module, me->crop.width);
	dev_dbg(&me->pdev->dev, "crop height : %d\n", me->crop.height);

	dev_dbg(&me->pdev->dev, "%s: DMA Addr 0 : 0x%X, DMA Addr 1 : 0x%X\n",
		__func__, buf->dma_addr[0], buf->dma_addr[1]);
	dev_dbg(&me->pdev->dev, " DMA Addr2 : 0x%X\n", buf->dma_addr[2]);

	dev_dbg(&me->pdev->dev, "%s: Stride[0] : 0x%X, Stride[1] : 0x%X\n",
		__func__, buf->stride[0], buf->stride[1]);

	mod_timer(&me->timer, jiffies +
		msecs_to_jiffies(DEBUG_SYNC_TIMEOUT_MS));
#endif
	return 0;
}

static void install_timer(struct nx_clipper *me)
{
#ifdef CONFIG_CLIPPER_USE_DQTIMER
	mod_timer(&me->dq_timer,
		  jiffies + msecs_to_jiffies(DQ_TIMEOUT_MS));
#endif
}

static int handle_buffer_done(struct nx_clipper *me)
{
	struct nx_video_buffer *buf = NULL;

	while (!list_empty(&me->done_bufs)) {
		buf = list_first_entry(&me->done_bufs,
				struct nx_video_buffer, list);
		if (buf) {
			buf->consumer_index++;
			buf->cb_buf_done(buf);
			list_del_init(&buf->list);
		}
	}
	return 0;
}

static int handle_buffer_underrun(struct nx_clipper *me)
{
	if (me->buf.addr) {
		nx_vip_set_clipper_addr(me->module, me->mem_fmt,
					me->crop.width, me->crop.height,
					me->buf.handle[0], me->buf.handle[1],
					me->buf.handle[2], me->buf.stride[0],
					me->buf.stride[1]);
	}
	return 0;
}

static void add_buffer_to_handler(struct nx_clipper *me,
		struct nx_video_buffer *done_buf)
{
	list_add_tail(&done_buf->list, &me->done_bufs);
	tasklet_schedule(&me->work);
}

static void process_buffer(struct nx_clipper *me, bool is_timer)
{
#ifdef CONFIG_CLIPPER_USE_DQTIMER
	unsigned long flags;
#endif

#ifdef CONFIG_CLIPPER_USE_DQTIMER
	spin_lock_irqsave(&me->lock, flags);
#endif
	if (NX_ATOMIC_READ(&me->state) & STATE_MEM_STOPPING) {
		if (is_timer)
			nx_vip_force_stop(me->module, VIP_CLIPPER);
		complete(&me->stop_done);
	} else {
		if (!me->buffer_underrun) {
			struct nx_video_buffer *done_buf = NULL;
			struct nx_video_buffer_object *obj = &me->vbuf_obj;
			int buf_count;

			done_buf = nx_video_get_next_buffer(obj, true);
			buf_count = nx_video_get_buffer_count(obj);
			if (buf_count >= 1) {
				update_buffer(me);
			} else {
				handle_buffer_underrun(me);
				me->buffer_underrun = true;
			}

			if (done_buf)
				add_buffer_to_handler(me, done_buf);
		} else {
			int buf_count
				= nx_video_get_buffer_count(&me->vbuf_obj);

			if (buf_count >= 1) {
				update_buffer(me);
				me->buffer_underrun = false;
			}
		}

		install_timer(me);
	}
#ifdef CONFIG_CLIPPER_USE_DQTIMER
	spin_unlock_irqrestore(&me->lock, flags);
#endif
}

#ifdef CONFIG_CLIPPER_USE_DQTIMER
static void handle_dq_timeout(unsigned long priv)
{
	struct nx_clipper *me = (struct nx_clipper *)priv;

	dev_info(&me->pdev->dev, "[CLI %d] DQTimeout\n", me->module);
	process_buffer(me, true);
}
#endif

static void init_buffer_handler(struct nx_clipper *me)
{
	INIT_LIST_HEAD(&me->done_bufs);
	tasklet_init(&me->work, (void*)handle_buffer_done,
			(long unsigned int)me);
}

static void deinit_buffer_handler(struct nx_clipper *me)
{
	struct nx_video_buffer *buf = NULL;

	tasklet_kill(&me->work);
	while (!list_empty(&me->done_bufs)) {
		buf = list_entry(me->done_bufs.next,
					struct nx_video_buffer, list);
		if (buf) {
			buf->cb_buf_done(buf);
			list_del_init(&buf->list);
		} else
			break;
	}
	list_del_init(&me->done_bufs);
}

static void unregister_irq_handler(struct nx_clipper *me)
{
	if (me->irq_entry) {
		nx_vip_unregister_irq_entry(me->module, VIP_CLIPPER,
				me->irq_entry);
		kfree(me->irq_entry);
		me->irq_entry = NULL;
	}
}

static irqreturn_t nx_clipper_irq_handler(void *data)
{
	struct nx_clipper *me = data;

	bool interlace = me->interlace;
	bool do_process = true;

	if (NX_ATOMIC_READ(&me->state) & STATE_MEM_STOPPING) {
		process_buffer(me, false);
		return IRQ_HANDLED;
	}

	if (interlace) {
		bool is_odd = nx_vip_get_field_status(me->module);

		if (me->irq_count == 0) {
			if (!is_odd) /* odd */
				me->irq_count++;
		} else {
			if (is_odd) /* even */
				me->irq_count++;
		}

		if (me->irq_count == 2)
			me->irq_count = 0;
		else
			do_process = false;
	}

	if (do_process)
		process_buffer(me, false);

	return IRQ_HANDLED;
}

static int register_irq_handler(struct nx_clipper *me)
{
	struct nx_v4l2_irq_entry *irq_entry = me->irq_entry;

	if (!irq_entry) {
		irq_entry = kzalloc(sizeof(*irq_entry), GFP_KERNEL);
		if (!irq_entry) {
			WARN_ON(1);
			return -ENOMEM;
		}
		me->irq_entry = irq_entry;
	}
	irq_entry->irqs = VIP_OD_INT;
	irq_entry->priv = me;
	irq_entry->handler = nx_clipper_irq_handler;
	return nx_vip_register_irq_entry(me->module, VIP_CLIPPER, irq_entry);
}

static int clipper_buffer_queue(struct nx_video_buffer *buf, void *data)
{
	struct nx_clipper *me = data;

	nx_video_add_buffer(&me->vbuf_obj, buf);

	return 0;
}

static int handle_video_connection(struct nx_clipper *me, bool connected)
{
	int ret = 0;

	if (connected)
		ret = nx_video_register_buffer_consumer(&me->vbuf_obj,
							clipper_buffer_queue,
							me);
	else
		nx_video_unregister_buffer_consumer(&me->vbuf_obj);

	return ret;
}

static struct v4l2_subdev *get_remote_source_subdev(struct nx_clipper *me)
{
	struct media_pad *pad =
		media_entity_remote_pad(&me->pads[NX_CLIPPER_PAD_SINK]);
	if (!pad) {
		dev_err(&me->pdev->dev, "can't find remote source device\n");
		return NULL;
	}
	return media_entity_to_v4l2_subdev(pad->entity);
}

static void set_vip(struct nx_clipper *me)
{
	u32 module = me->module;
	bool is_mipi = me->interface_type == NX_CAPTURE_INTERFACE_MIPI_CSI;

	if (me->clk_src >= 0) {
		nx_vip_clock_config(module, me->clk_src, me->clk_freq);
		nx_vip_clock_enable(module, true);
		nx_vip_reset(module);
	}
	nx_vip_set_input_port(module, me->port);
	nx_vip_set_field_mode(module, false, nx_vip_fieldsel_bypass,
			      me->interlace, false);

	if (is_mipi) {
		nx_vip_set_data_mode(module, me->bus_fmt, 16);
		nx_vip_set_dvalid_mode(module, true, true, true);
		nx_vip_set_hvsync_for_mipi(module,
					   me->width * 2,
					   me->height,
					   me->h_syncwidth,
					   me->h_frontporch,
					   me->h_backporch,
					   me->v_syncwidth,
					   me->v_frontporch,
					   me->v_backporch);
	} else {
		nx_vip_set_data_mode(module, me->bus_fmt, 8);
		nx_vip_set_dvalid_mode(module, false, false, false);
		nx_vip_set_hvsync(module,
				  me->external_sync,
				  me->width * 2,
				  me->interlace ?
				  me->height >> 1 : me->height,
				  me->padclk_sel,
				  me->h_syncpolarity,
				  me->v_syncpolarity,
				  me->h_syncwidth,
				  me->h_frontporch,
				  me->h_backporch,
				  me->v_syncwidth,
				  me->v_frontporch,
				  me->v_backporch);
	}

	nx_vip_set_fiforeset_mode(module, nx_vip_fiforeset_all);

	nx_vip_set_clip_region(module,
			       me->crop.left,
			       me->crop.top,
			       me->crop.left + me->crop.width,
			       me->interlace ?
			       (me->crop.top + me->crop.height) >> 1 :
			       (me->crop.top + me->crop.height));
}

/**
 * v4l2 subdev ops
 */
static int nx_clipper_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote;
	u32 module = me->module;
	char *hostname = (char *)v4l2_get_subdev_hostdata(sd);
	bool is_host_video = false;

	dev_info(&me->pdev->dev, "[CLI %d] enable %d\n", me->module, enable);

	me->irq_count = 0;
	remote = get_remote_source_subdev(me);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	if (!hostname)
		return -EEXIST;

	if (!strncmp(hostname, "VIDEO", 5))
		is_host_video = true;

	ret = down_interruptible(&me->s_stream_sem);

	if (enable) {
		if (NX_ATOMIC_READ(&me->state) & STATE_MEM_STOPPING) {
			int timeout = 50; /* 5 second */

			dev_info(&me->pdev->dev, "wait clipper stopping\n");
			while (NX_ATOMIC_READ(&me->state) &
			       STATE_MEM_STOPPING) {
				msleep(100);
				timeout--;
				if (timeout == 0) {
					dev_err(&me->pdev->dev, "timeout for waiting clipper stop\n");
					break;
				}
			}
		}

		if (!(NX_ATOMIC_READ(&me->state) &
		      (STATE_MEM_RUNNING | STATE_CLIP_RUNNING))) {
			if (is_host_video &&
					nx_vip_is_running(me->module, VIP_CLIPPER)) {
				pr_err("VIP%d Clipper is already running\n",
						me->module);
				nx_video_clear_buffer(&me->vbuf_obj);
				ret = -EBUSY;
				goto UP_AND_OUT;
			}

			if ((me->crop.width == 0) || (me->crop.height == 0)) {
				me->crop.left = 0;
				me->crop.top = 0;
				me->crop.width = me->width;
				me->crop.height = me->height;
			}

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
			nx_clipper_qos_update(NX_BUS_CLK_VIP_KHZ);
			nx_clipper_qos_cpu_online_update(1);
#endif
			set_vip(me);
			ret = enable_sensor_power(me, true);
			if (ret) {
				WARN_ON(1);
				goto UP_AND_OUT;
			}
			ret = v4l2_subdev_call(remote, video, s_stream, 1);
			if (ret) {
				dev_err(&me->pdev->dev,
					"failed to s_stream %d\n", enable);
				nx_video_clear_buffer_queued(&me->vbuf_obj);
				goto UP_AND_OUT;
			}
		}

		if (is_host_video) {
			nx_vip_set_clipper_format(module, me->mem_fmt);
			ret = register_irq_handler(me);
			if (ret) {
				WARN_ON(1);
				goto UP_AND_OUT;
			}

#ifdef CONFIG_CLIPPER_USE_DQTIMER
			setup_timer(&me->dq_timer, handle_dq_timeout, (long)me);
#endif
			ret = update_buffer(me);
			if (ret) {
				WARN_ON(1);
				goto UP_AND_OUT;
			}
			alloc_dma_buffer(me);
			init_buffer_handler(me);
			nx_vip_run(me->module, VIP_CLIPPER);
			install_timer(me);
			NX_ATOMIC_SET_MASK(STATE_MEM_RUNNING, &me->state);
		} else
		NX_ATOMIC_SET_MASK(STATE_CLIP_RUNNING, &me->state);
	} else {
		if (!(NX_ATOMIC_READ(&me->state) &
		      (STATE_MEM_RUNNING | STATE_CLIP_RUNNING)))
			goto UP_AND_OUT;

		if (is_host_video &&
		    (NX_ATOMIC_READ(&me->state) & STATE_MEM_RUNNING)) {
			NX_ATOMIC_SET_MASK(STATE_MEM_STOPPING, &me->state);
			nx_vip_stop(module, VIP_CLIPPER);
			wait_for_completion_timeout(&me->stop_done, HZ);
			NX_ATOMIC_CLEAR_MASK(STATE_MEM_STOPPING, &me->state);
#ifdef CONFIG_CLIPPER_USE_DQTIMER
			while (timer_pending(&me->dq_timer)) {
				mdelay(DQ_TIMEOUT_MS);
				dev_info(&me->pdev->dev,
					 "[CLI %d] wait timer done\n", me->module);
			}
#endif

			unregister_irq_handler(me);
			me->buffer_underrun = false;
			free_dma_buffer(me);
			nx_video_clear_buffer(&me->vbuf_obj);
			deinit_buffer_handler(me);
			NX_ATOMIC_CLEAR_MASK(STATE_MEM_RUNNING, &me->state);
			dev_info(&me->pdev->dev, "[CLI %d] stop done\n",
				 me->module);
		}
		memset(&me->crop, 0, sizeof(me->crop));

		if (!is_host_video)
			if (NX_ATOMIC_READ(&me->state) == STATE_IDLE)
				goto UP_AND_OUT;

		if ((!nx_vip_is_running(me->module, VIP_DECIMATOR)) &&
				(!(NX_ATOMIC_READ(&me->state) &
				   STATE_MEM_RUNNING))) {
			if (me->clk_src >= 0)
				nx_vip_clock_enable(me->module, false);
			v4l2_subdev_call(remote, video, s_stream, 0);
			enable_sensor_power(me, false);
			NX_ATOMIC_CLEAR_MASK(STATE_CLIP_RUNNING, &me->state);

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
			nx_clipper_qos_update(NX_BUS_CLK_IDLE_KHZ);
			nx_clipper_qos_cpu_online_update(-1);
#endif
		}
	}

UP_AND_OUT:
	up(&me->s_stream_sem);

	return ret;
}

static int nx_clipper_g_crop(struct v4l2_subdev *sd,
			     struct v4l2_crop *crop)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);
	int err;

	err = v4l2_subdev_call(remote, video, g_crop, crop);
	if (!err) {
		pr_debug("[%s] crop %d:%d:%d:%d\n", __func__, crop->c.left,
				crop->c.top, crop->c.width, crop->c.height);
	}
	return err;
}

static int nx_clipper_s_crop(struct v4l2_subdev *sd,
			     const struct v4l2_crop *crop)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);

	if ((NX_ATOMIC_READ(&me->state) &
				(STATE_MEM_RUNNING | STATE_CLIP_RUNNING))) {
		if ((me->crop.width != crop->c.width) ||
				(me->crop.height != crop->c.height) ||
				(me->crop.left != crop->c.left) ||
				(me->crop.top != crop->c.top))
			return -EINVAL;
	}

	if (crop->c.left >= me->width || crop->c.top >= me->height)
		return -EINVAL;
	if ((crop->c.left + crop->c.width) > me->width ||
		(crop->c.top + crop->c.height) > me->height)
		return -EINVAL;

	/* me->crop = crop->c; */
	memcpy(&me->crop, &crop->c, sizeof(struct v4l2_rect));
	return 0;
}

static int nx_clipper_g_parm(struct v4l2_subdev *sd,
			     struct v4l2_streamparm *param)
{
	int err;
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	err = v4l2_subdev_call(remote, video, g_parm, param);

	if (err) {
		param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		param->parm.capture.timeperframe.numerator = 1001;
		param->parm.capture.timeperframe.denominator = 30000;
		param->parm.capture.readbuffers = 1;
	}

	return 0;
}

static int nx_clipper_s_parm(struct v4l2_subdev *sd,
			     struct v4l2_streamparm *param)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, video, s_parm, param);
}

static int nx_clipper_g_ctrl(struct v4l2_subdev *sd,
			     struct v4l2_control *ctrl)
{
	int ret = 0;

	if (ctrl->id == V4L2_CID_MIN_BUFFERS_FOR_CAPTURE)
		ctrl->value = 8;
	else
		ret = -EINVAL;

	return ret;
}

static int nx_clipper_s_ctrl(struct v4l2_subdev *sd,
			     struct v4l2_control *ctrl)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, core, s_ctrl, ctrl);
}

/**
 * called by VIDIOC_SUBDEV_S_CROP
 */
static int nx_clipper_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_selection *sel)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);

	memcpy(&sel->r, &me->crop, sizeof(struct v4l2_rect));
	return 0;
}

static int nx_clipper_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_selection *sel)
{
	/*struct nx_clipper *me = v4l2_get_subdevdata(sd);

	memcpy(&me->crop, &sel->r, sizeof(struct v4l2_rect));*/
	return 0;
}

static int nx_clipper_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *format)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	u32 pad = format->pad;

	if (pad == 0) {
		/* get bus format */
		u32 mbus_fmt;
		int ret = nx_vip_find_mbus_format(me->bus_fmt, &mbus_fmt);

		if (ret) {
			dev_err(&me->pdev->dev, "can't get mbus_fmt for bus\n");
			return ret;
		}
		format->format.code = mbus_fmt;
		format->format.width = me->width;
		format->format.height = me->height;
	}
	else if (pad == 1) {
		/* get mem format */
		u32 mem_fmt;
		int ret = nx_vip_find_mbus_mem_format(me->mem_fmt, &mem_fmt);

		if (ret) {
			dev_err(&me->pdev->dev, "can't get mbus_fmt for mem\n");
			return ret;
		}
		format->format.code = mem_fmt;
		format->format.width = me->width;
		format->format.height = me->height;
	}
	else {
		dev_err(&me->pdev->dev, "%d is invalid pad value for get_fmt\n",
			pad);
		return -EINVAL;
	}

	return 0;
}

static int nx_clipper_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *format)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	u32 pad = format->pad;
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	me->buf.format = format->format.code;
	if (pad == 0) {
		/* set bus format */
		u32 nx_bus_fmt;
		int ret = nx_vip_find_nx_bus_format(format->format.code,
						    &nx_bus_fmt);
		if (ret) {
			dev_err(&me->pdev->dev, "Unsupported bus format %d\n",
			       format->format.code);
			return ret;
		}
		me->bus_fmt = nx_bus_fmt;
		me->width = format->format.width;
		me->height = format->format.height;
	}
	else if (pad == 1) {
		struct v4l2_subdev_format fmt;
		/* set memory format */
		u32 nx_mem_fmt;
		int ret = nx_vip_find_nx_mem_format(format->format.code,
						    &nx_mem_fmt);
		if (ret) {
			dev_err(&me->pdev->dev, "Unsupported mem format %d\n",
			       format->format.code);
			return ret;
		}
		me->mem_fmt = nx_mem_fmt;
		me->width = format->format.width;
		me->height = format->format.height;
		memset(&fmt, 0, sizeof(fmt));
		fmt.format.width = me->width;
		fmt.format.height = me->height;
		fmt.which = format->which;

		return v4l2_subdev_call(remote, pad, set_fmt, NULL, &fmt);
	}
	else {
		dev_err(&me->pdev->dev, "%d is invalid pad value for set_fmt\n",
			pad);
		return -EINVAL;
	}

	return 0;
}

static int nx_clipper_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_size_enum
					     *frame)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	pr_debug("[%s]\n", __func__);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, pad, enum_frame_size, NULL, frame);
}

static int nx_clipper_enum_frame_interval(struct v4l2_subdev *sd,
			      		  struct v4l2_subdev_pad_config *cfg,
			      		  struct v4l2_subdev_frame_interval_enum
						*frame)
{
	struct nx_clipper *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	pr_debug("[%s]\n", __func__);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, pad, enum_frame_interval, NULL, frame);
}

static const struct v4l2_subdev_video_ops nx_clipper_video_ops = {
	.s_stream = nx_clipper_s_stream,
	.g_crop = nx_clipper_g_crop,
	.s_crop = nx_clipper_s_crop,
	.g_parm = nx_clipper_g_parm,
	.s_parm = nx_clipper_s_parm,
};

static const struct v4l2_subdev_pad_ops nx_clipper_pad_ops = {
	.get_selection = nx_clipper_get_selection,
	.set_selection = nx_clipper_set_selection,
	.get_fmt = nx_clipper_get_fmt,
	.set_fmt = nx_clipper_set_fmt,
	.enum_frame_size = nx_clipper_enum_frame_size,
	.enum_frame_interval = nx_clipper_enum_frame_interval,
};

static const struct v4l2_subdev_core_ops nx_clipper_core_ops = {
	.g_ctrl = nx_clipper_g_ctrl,
	.s_ctrl = nx_clipper_s_ctrl,
};

static const struct v4l2_subdev_ops nx_clipper_subdev_ops = {
	.video = &nx_clipper_video_ops,
	.pad = &nx_clipper_pad_ops,
	.core = &nx_clipper_core_ops,
};

/**
 * media_entity_operations
 */
static int nx_clipper_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote,
				 u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct nx_clipper *me = v4l2_get_subdevdata(sd);

	switch (local->index | media_entity_type(remote->entity)) {
	case NX_CLIPPER_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		pr_debug("clipper sink %s\n",
			 flags & MEDIA_LNK_FL_ENABLED ?
			 "connected" : "disconnected");
		break;
	case NX_CLIPPER_PAD_SOURCE_DECIMATOR | MEDIA_ENT_T_V4L2_SUBDEV:
		pr_debug("clipper source decimator %s\n",
			 flags & MEDIA_LNK_FL_ENABLED ?
			 "connected" : "disconnected");
		break;
	case NX_CLIPPER_PAD_SOURCE_MEM | MEDIA_ENT_T_DEVNODE:
		pr_debug("clipper source mem %s\n",
			 flags & MEDIA_LNK_FL_ENABLED ?
			 "connected" : "disconnected");
		handle_video_connection(me, flags & MEDIA_LNK_FL_ENABLED ?
					true : false);
		break;
	}

	return 0;
}

static const struct media_entity_operations nx_clipper_media_ops = {
	.link_setup = nx_clipper_link_setup,
};

/**
 * initialization
 */
static void init_me(struct nx_clipper *me)
{
	NX_ATOMIC_SET(&me->state, STATE_IDLE);
	init_completion(&me->stop_done);
	sema_init(&me->s_stream_sem, 1);
	nx_video_init_vbuf_obj(&me->vbuf_obj);
}

static int init_v4l2_subdev(struct nx_clipper *me)
{
	int ret;
	struct v4l2_subdev *sd = &me->subdev;
	struct media_pad *pads = me->pads;
	struct media_entity *entity = &sd->entity;
	int module = me->module;

	v4l2_subdev_init(sd, &nx_clipper_subdev_ops);

	if (me->logical)
		module = me->module*VIP_MAX_LOGICAL_DEV + me->logical_num +
			VIP_LOGICAL_START;
	snprintf(sd->name, sizeof(sd->name), "%s%d",
		NX_CLIPPER_DEV_NAME, module);
	v4l2_set_subdevdata(sd, me);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[NX_CLIPPER_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[NX_CLIPPER_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;
	pads[NX_CLIPPER_PAD_SOURCE_DECIMATOR].flags = MEDIA_PAD_FL_SOURCE;

	entity->ops = &nx_clipper_media_ops;
	ret = media_entity_init(entity, NX_CLIPPER_PAD_MAX, pads, 0);
	if (ret < 0) {
		dev_err(&me->pdev->dev, "failed to media_entity_init\n");
		return ret;
	}

	return 0;
}

static struct camera_sensor_info {
	int is_mipi;
	int interlaced;
	char name[V4L2_SUBDEV_NAME_SIZE];
} camera_sensor_info[VIP_MAX_DEV_NUM];

static ssize_t camera_sensor_show_common(struct device *dev,
	struct device_attribute *attr, char **buf, u32 module)
{
	struct attribute *at;

	at = &attr->attr;

	if (!strlen(camera_sensor_info[module].name))
		return scnprintf(*buf, PAGE_SIZE, "no exist");
	else
		return scnprintf(*buf, PAGE_SIZE, "is_mipi:%d,interlaced:%d,name:%s",
				 camera_sensor_info[module].is_mipi,
				 camera_sensor_info[module].interlaced,
				 camera_sensor_info[module].name);
}

static ssize_t camera_sensor_show0(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 0);
}

static ssize_t camera_sensor_show1(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 1);
}

static ssize_t camera_sensor_show2(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 2);
}

static ssize_t camera_sensor_show3(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 3);
}

static ssize_t camera_sensor_show4(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 4);
}

static ssize_t camera_sensor_show5(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 5);
}

static ssize_t camera_sensor_show6(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 6);
}

static ssize_t camera_sensor_show7(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 7);
}

static ssize_t camera_sensor_show8(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 8);
}

static ssize_t camera_sensor_show9(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 9);
}

static ssize_t camera_sensor_show10(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 10);
}

static ssize_t camera_sensor_show11(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return camera_sensor_show_common(dev, attr, &buf, 11);
}

static struct device_attribute camera_sensor0_attr =
__ATTR(info, 0644, camera_sensor_show0, NULL);
static struct device_attribute camera_sensor1_attr =
__ATTR(info, 0644, camera_sensor_show1, NULL);
static struct device_attribute camera_sensor2_attr =
__ATTR(info, 0644, camera_sensor_show2, NULL);
static struct device_attribute camera_sensor3_attr =
__ATTR(info, 0644, camera_sensor_show3, NULL);
static struct device_attribute camera_sensor4_attr =
__ATTR(info, 0644, camera_sensor_show4, NULL);
static struct device_attribute camera_sensor5_attr =
__ATTR(info, 0644, camera_sensor_show5, NULL);
static struct device_attribute camera_sensor6_attr =
__ATTR(info, 0644, camera_sensor_show6, NULL);
static struct device_attribute camera_sensor7_attr =
__ATTR(info, 0644, camera_sensor_show7, NULL);
static struct device_attribute camera_sensor8_attr =
__ATTR(info, 0644, camera_sensor_show8, NULL);
static struct device_attribute camera_sensor9_attr =
__ATTR(info, 0644, camera_sensor_show9, NULL);
static struct device_attribute camera_sensor10_attr =
__ATTR(info, 0644, camera_sensor_show10, NULL);
static struct device_attribute camera_sensor11_attr =
__ATTR(info, 0644, camera_sensor_show11, NULL);

static struct attribute *camera_sensor_attrs[] = {
	&camera_sensor0_attr.attr,
	&camera_sensor1_attr.attr,
	&camera_sensor2_attr.attr,
	&camera_sensor3_attr.attr,
	&camera_sensor4_attr.attr,
	&camera_sensor5_attr.attr,
	&camera_sensor6_attr.attr,
	&camera_sensor7_attr.attr,
	&camera_sensor8_attr.attr,
	&camera_sensor9_attr.attr,
	&camera_sensor10_attr.attr,
	&camera_sensor11_attr.attr,
};

static int create_sysfs_for_camera_sensor(struct nx_clipper *me,
					  struct nx_v4l2_i2c_board_info *info)
{
	int ret;
	struct kobject *kobj;
	char kobject_name[25] = {0, };
	char sensor_name[V4L2_SUBDEV_NAME_SIZE];
	int module = me->module;

	memset(sensor_name, 0, V4L2_SUBDEV_NAME_SIZE);
	snprintf(sensor_name, sizeof(sensor_name), "%s %d-%04x",
		info->board_info.type,
		info->i2c_adapter_id,
		info->board_info.addr);

	if (me->logical)
		module = me->module*VIP_MAX_LOGICAL_DEV + me->logical_num +
			VIP_LOGICAL_START;
	strlcpy(camera_sensor_info[module].name, sensor_name,
		V4L2_SUBDEV_NAME_SIZE);
	camera_sensor_info[module].is_mipi =
		me->interface_type == NX_CAPTURE_INTERFACE_MIPI_CSI;
	camera_sensor_info[module].interlaced = me->interlace;

	snprintf(kobject_name, sizeof(kobject_name), "camerasensor%d",
			module);
	kobj = kobject_create_and_add(kobject_name, &platform_bus.kobj);
	if (!kobj) {
		dev_err(&me->pdev->dev, "failed to kobject_create for module %d-%d-%d\n",
			me->module, me->logical, me->logical_num);
		return -EINVAL;
	}

	ret = sysfs_create_file(kobj, camera_sensor_attrs[module]);
	if (ret) {
		dev_err(&me->pdev->dev, "failed to sysfs_create_file for module %d-%d-%d\n",
			me->module, me->logical, me->logical_num);
		kobject_put(kobj);
	}
	return 0;
}

static int init_sensor_media_entity(struct nx_clipper *me,
				    struct v4l2_subdev *sd)
{
	if (!sd->entity.links) {
		struct media_pad *pad = &me->sensor_pad;

		pad->flags = MEDIA_PAD_FL_SOURCE;
		sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
		return media_entity_init(&sd->entity, 1, pad, 0);
	}

	return 0;
}

static int setup_link(struct media_pad *src, struct media_pad *dst)
{
	struct media_link *link;

	link = media_entity_find_link(src, dst);
	if (link == NULL)
		return -ENODEV;

	return __media_entity_setup_link(link, MEDIA_LNK_FL_ENABLED);
}

static int register_sensor_subdev(struct nx_clipper *me)
{
	int ret;
	struct i2c_adapter *adapter;
	struct v4l2_subdev *sensor;
	struct i2c_client *client;
	struct media_entity *input;
	u32 pad;
	struct nx_v4l2_i2c_board_info *info = &me->sensor_info;

	adapter = i2c_get_adapter(info->i2c_adapter_id);
	if (!adapter) {
		dev_err(&me->pdev->dev, "unable to get sensor i2c adapter\n");
		return -ENODEV;
	}

	request_module(I2C_MODULE_PREFIX "%s", info->board_info.type);
	client = i2c_new_device(adapter, &info->board_info);
	if (client == NULL || client->dev.driver == NULL) {
		ret = -ENODEV;
		goto error;
	}

	if (!try_module_get(client->dev.driver->owner)) {
		ret = -ENODEV;
		goto error;
	}
	sensor = i2c_get_clientdata(client);
	sensor->host_priv = info;

	ret = create_sysfs_for_camera_sensor(me, info);
	if (ret)
		goto error;

	ret = init_sensor_media_entity(me, sensor);
	if (ret) {
		dev_err(&me->pdev->dev, "failed to init sensor media entity\n");
		goto error;
	}

	ret  = nx_v4l2_register_subdev(sensor);
	if (ret) {
		dev_err(&me->pdev->dev, "failed to register subdev sensor\n");
		goto error;
	}

	if (me->interface_type == NX_CAPTURE_INTERFACE_MIPI_CSI) {
		struct v4l2_subdev *mipi_csi;

		mipi_csi = nx_v4l2_get_subdev("nx-csi");
		if (!mipi_csi) {
			dev_err(&me->pdev->dev, "can't get mipi_csi subdev\n");
			goto error;
		}

		ret = media_entity_create_link(&mipi_csi->entity, 1,
					       &me->subdev.entity, 0, 0);
		if (ret < 0) {
			dev_err(&me->pdev->dev,
				"failed to create link from csi to clipper\n");
			goto error;
		}

		ret = setup_link(&mipi_csi->entity.pads[1],
				 &me->subdev.entity.pads[0]);
		if (ret)
			BUG();

		input = &mipi_csi->entity;
		pad = 0;
	} else {
		input = &me->subdev.entity;
		pad = NX_CLIPPER_PAD_SINK;
	}

	ret = media_entity_create_link(&sensor->entity, 0, input, pad, 0);
	if (ret < 0)
		dev_err(&me->pdev->dev,
			"failed to create link from sensor\n");

	ret = setup_link(&sensor->entity.pads[0], &input->pads[pad]);
	if (ret)
		BUG();

error:
	if (client && ret < 0)
		i2c_unregister_device(client);

	return ret;
}

static void unregister_v4l2(struct nx_clipper *me)
{
	if (me->vbuf_obj.video) {
		nx_video_cleanup(me->vbuf_obj.video);
		me->vbuf_obj.video = NULL;
	}
	v4l2_device_unregister_subdev(&me->subdev);
}

static int register_v4l2(struct nx_clipper *me)
{
	int ret;
	char dev_name[64] = {0, };
	struct media_entity *entity = &me->subdev.entity;
	struct nx_video *video;
	int module = me->module;

	ret = nx_v4l2_register_subdev(&me->subdev);
	if (ret)
		BUG();

	ret = register_sensor_subdev(me);
	if (ret) {
		dev_info(&me->pdev->dev, "can't register sensor subdev\n");
		unregister_v4l2(me);
		return ret;
	}

	if (me->logical)
		module = me->module*VIP_MAX_LOGICAL_DEV + me->logical_num +
			VIP_LOGICAL_START;
	snprintf(dev_name, sizeof(dev_name), "VIDEO CLIPPER%d",
			module);
	video = nx_video_create(dev_name, NX_VIDEO_TYPE_CAPTURE,
				    nx_v4l2_get_v4l2_device(),
				    nx_v4l2_get_alloc_ctx());
	if (!video)
		BUG();


	ret = media_entity_create_link(entity, NX_CLIPPER_PAD_SOURCE_MEM,
				       &video->vdev.entity, 0, 0);
	if (ret < 0)
		BUG();

	me->vbuf_obj.video = video;

	ret = setup_link(&entity->pads[NX_CLIPPER_PAD_SOURCE_MEM],
			 &video->vdev.entity.pads[0]);
	if (ret)
		BUG();

	return 0;
}

/**
 * pm ops
 */
#ifdef CONFIG_PM_SLEEP
static int nx_clipper_suspend(struct device *dev)
{
	struct nx_clipper *me;

	me = dev_get_drvdata(dev);
	if (me) {
		if (NX_ATOMIC_READ(&me->state) & STATE_MEM_RUNNING) {
			struct v4l2_subdev *remote;

			remote = get_remote_source_subdev(me);
			if (remote) {
				v4l2_subdev_call(remote, video, s_stream, 0);
				enable_sensor_power(me, false);
				nx_vip_stop(me->module, VIP_CLIPPER);
			}
		}

		nx_vip_reset(me->module);
		nx_vip_clock_enable(me->module, false);
	}

	return 0;
}

static int nx_clipper_resume(struct device *dev)
{
	struct nx_clipper *me;

	me = dev_get_drvdata(dev);
	if (me) {
		nx_vip_clock_enable(me->module, true);
		nx_vip_reset(me->module);

		if (NX_ATOMIC_READ(&me->state) & STATE_MEM_RUNNING) {
			struct v4l2_subdev *remote;

			remote = get_remote_source_subdev(me);
			if (remote) {
				struct nx_video_buffer *buf;

				set_vip(me);
				enable_sensor_power(me, true);
				v4l2_subdev_call(remote, video, s_stream, 1);
				nx_vip_set_clipper_format(me->module,
							  me->mem_fmt);
				if (!me->buffer_underrun) {
					buf = me->last_buf;
					nx_vip_set_clipper_addr(me->module,
								me->mem_fmt,
								me->crop.width,
								me->crop.height,
								buf->dma_addr[0],
								buf->dma_addr[1],
								buf->dma_addr[2],
								buf->stride[0],
								buf->stride[1]);
					nx_vip_run(me->module, VIP_CLIPPER);
				}
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops nx_clipper_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nx_clipper_suspend, nx_clipper_resume)
};
#endif

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
static int init_clipper_th(void *args)
{
	int ret = 0;
	struct nx_clipper *me = args;

	if (!nx_vip_is_valid(me->module)) {
		dev_err(&me->pdev->dev, "NX VIP %d is not valid\n", me->module);
		return -ENODEV;
	}

	init_me(me);

	ret = init_v4l2_subdev(me);
	if (ret)
		return ret;

	ret = register_v4l2(me);
	if (ret)
		return ret;

	return ret;
}

static void init_clipper_work(struct work_struct *work)
{
	struct nx_clipper *me = container_of(work,
				struct nx_clipper, w_delay.work);
	int ret = 0;

	if (!nx_vip_is_valid(me->module)) {
		dev_err(&me->pdev->dev, "NX VIP %d is not valid\n", me->module);
		return;
	}

	init_me(me);

	ret = init_v4l2_subdev(me);
	if (ret)
		return;

	ret = register_v4l2(me);
	if (ret)
		return;

}
#endif

/**
 * platform driver
 */
static int nx_clipper_probe(struct platform_device *pdev)
{
	int ret;
	struct nx_clipper *me;
	struct device *dev = &pdev->dev;

	me = devm_kzalloc(dev, sizeof(*me), GFP_KERNEL);
	if (!me) {
		WARN_ON(1);
		return -ENOMEM;
	}
	me->pdev = pdev;
	ret = nx_clipper_parse_dt(dev, me);
	if (ret) {
		dev_err(dev, "failed to parse dt\n");
		return ret;
	}
#ifndef CONFIG_V4L2_INIT_LEVEL_UP
	if (!nx_vip_is_valid(me->module)) {
		dev_err(dev, "NX VIP %d is not valid\n", me->module);
		return -ENODEV;
	}

	init_me(me);

	ret = init_v4l2_subdev(me);
	if (ret)
		return ret;

	ret = register_v4l2(me);
	if (ret)
		return ret;
#else
	if (me->module == 1) {
		if (g_ClipperThread == NULL)
			g_ClipperThread = kthread_run(init_clipper_th,
				me, "KthreadForNxClipper");
	}

	if (me->module == 0) {
		me->w_queue = create_singlethread_workqueue("clipper_wqueue");
		INIT_DELAYED_WORK(&me->w_delay, init_clipper_work);

		queue_delayed_work(me->w_queue, &me->w_delay,
							msecs_to_jiffies(2000));
	}
#endif

	me->buffer_underrun = false;
	me->buf.addr = NULL;
	platform_set_drvdata(pdev, me);

#ifdef DEBUG_SYNC
	setup_timer(&me->timer, debug_sync, (long)me);
#endif

#ifdef CONFIG_CLIPPER_USE_DQTIMER
	spin_lock_init(&me->lock);
#endif
	return 0;
}

static int nx_clipper_remove(struct platform_device *pdev)
{
	struct nx_clipper *me = platform_get_drvdata(pdev);

	if (unlikely(!me))
		return 0;

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
	if (me->module == 1) {
		cancel_delayed_work(&me->w_delay);
		flush_workqueue(me->w_queue);
		destroy_workqueue(me->w_queue);
	}
#endif

	unregister_v4l2(me);

	return 0;
}

static struct platform_device_id nx_clipper_id_table[] = {
	{ NX_CLIPPER_DEV_NAME, 0 },
	{},
};

static const struct of_device_id nx_clipper_dt_match[] = {
	{ .compatible = "nexell,nx-clipper" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_clipper_dt_match);

static struct platform_driver nx_clipper_driver = {
	.probe		= nx_clipper_probe,
	.remove		= nx_clipper_remove,
	.id_table	= nx_clipper_id_table,
	.driver		= {
		.name	= NX_CLIPPER_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nx_clipper_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm	= &nx_clipper_pm_ops,
#endif
	},
};

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
static int __init nx_clipper_init(void)
{
	return platform_driver_register(&nx_clipper_driver);
}

static void __exit nx_clipper_exit(void)
{
	platform_driver_unregister(&nx_clipper_driver);
}
subsys_initcall(nx_clipper_init);
module_exit(nx_clipper_exit);
#else
module_platform_driver(nx_clipper_driver);
#endif

MODULE_AUTHOR("swpark <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell S5Pxx18 series SoC V4L2 capture clipper driver");
MODULE_LICENSE("GPL");
