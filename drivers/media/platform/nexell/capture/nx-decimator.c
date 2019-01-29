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
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/irqreturn.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/semaphore.h>
#include <linux/v4l2-mediabus.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/dma-mapping.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <dt-bindings/media/nexell-vip.h>

#include "../nx-v4l2.h"
#include "../nx-video.h"
#include "nx-vip-primitive.h"
#include "nx-vip.h"

#define NX_DECIMATOR_DEV_NAME	"nx-decimator"

#ifdef CONFIG_DECIMATOR_USE_DQTIMER
#include <linux/timer.h>
#include <linux/delay.h>
#define DQ_TIMEOUT_MS		CONFIG_DECIMATOR_DQTIMER_TIMEOUT
#endif

enum {
	NX_DECIMATOR_PAD_SINK,
	NX_DECIMATOR_PAD_SOURCE_MEM,
	NX_DECIMATOR_PAD_MAX
};

enum {
	STATE_IDLE = 0,
	STATE_RUNNING  = (1 << 0),
	STATE_STOPPING = (1 << 1),
};

struct nx_decimator {
	u32 module;
	u32 logical;
	u32 logical_num;

	struct v4l2_subdev subdev;
	struct media_pad pads[NX_DECIMATOR_PAD_MAX];
	struct nx_dma_buf buf;

	u32 width;
	u32 height;
	u32 clip_width;
	u32 clip_height;

	atomic_t state;
	struct completion stop_done;
	struct semaphore s_stream_sem;

	struct platform_device *pdev;

	struct tasklet_struct work;
	struct list_head done_bufs;
	struct nx_video_buffer_object vbuf_obj;
	struct nx_v4l2_irq_entry *irq_entry;
	u32 mem_fmt;

	bool buffer_underrun;

#ifdef CONFIG_DECIMATOR_USE_DQTIMER
	struct timer_list dq_timer;
	spinlock_t lock;
#endif
};

static int register_irq_handler(struct nx_decimator *me);

static int alloc_dma_buffer(struct nx_decimator *me)
{
	if (me->buf.addr == NULL) {
		int y_size, cbcr_size;
		struct nx_video_buffer *buf;

		buf = nx_video_get_next_buffer(&me->vbuf_obj, false);
		if (!buf) {
			dev_err(&me->pdev->dev, "can't get next buffer\n");
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
			dev_err(&me->pdev->dev,
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

static void free_dma_buffer(struct nx_decimator *me)
{
	if (me->buf.addr) {
		dma_free_coherent(&me->pdev->dev, me->buf.size,
				me->buf.addr,
				me->buf.handle[0]);
		me->buf.handle[0] = me->buf.handle[1] =
			me->buf.handle[2] = 0;
		me->buf.stride[0] = me->buf.stride[1] = 0;
		me->buf.addr = NULL;
	}
}

static int handle_buffer_done(struct nx_decimator *me)
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

static void init_buffer_handler(struct nx_decimator *me)
{
	INIT_LIST_HEAD(&me->done_bufs);
	tasklet_init(&me->work, (void*)handle_buffer_done,
			(long unsigned int)me);
}

static void add_buffer_to_handler(struct nx_decimator *me,
		struct nx_video_buffer *done_buf)
{
	list_add_tail(&done_buf->list, &me->done_bufs);
	tasklet_schedule(&me->work);
}

static void deinit_buffer_handler(struct nx_decimator *me)
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

static int handle_buffer_underrun(struct nx_decimator *me)
{
	if (me->buf.addr) {
		nx_vip_set_decimator_addr(me->module, me->mem_fmt,
					me->width, me->height,
					me->buf.handle[0], me->buf.handle[1],
					me->buf.handle[2], me->buf.stride[0],
					me->buf.stride[1]);
	}
	return 0;
}

static int update_buffer(struct nx_decimator *me)
{
	struct nx_video_buffer *buf;

	buf = nx_video_get_next_buffer(&me->vbuf_obj, false);
	if (!buf) {
		dev_err(&me->pdev->dev, "can't get next buffer\n");
		return -ENOENT;
	}

	nx_vip_set_decimator_addr(me->module, me->mem_fmt,
				me->width, me->height,
				buf->dma_addr[0], buf->dma_addr[1],
				buf->dma_addr[2], buf->stride[0],
				buf->stride[1]);


	return 0;
}

static void install_timer(struct nx_decimator *me)
{
#ifdef CONFIG_DECIMATOR_USE_DQTIMER
	mod_timer(&me->dq_timer,
		  jiffies + msecs_to_jiffies(DQ_TIMEOUT_MS));
#endif
}

static void process_buffer(struct nx_decimator *me, bool is_timer)
{
	if (NX_ATOMIC_READ(&me->state) & STATE_STOPPING) {
		if (is_timer)
			nx_vip_force_stop(me->module, VIP_DECIMATOR);
		complete(&me->stop_done);
	} else {
#ifdef CONFIG_DECIMATOR_USE_DQTIMER
		unsigned long flags;
#endif

#ifdef CONFIG_DECIMATOR_USE_DQTIMER
		spin_lock_irqsave(&me->lock, flags);
#endif
		if (!me->buffer_underrun) {
			struct nx_video_buffer *done = NULL;
			struct nx_video_buffer_object *obj = &me->vbuf_obj;
			int buf_count;

			done = nx_video_get_next_buffer(obj, true);
			buf_count = nx_video_get_buffer_count(obj);
			if (buf_count >= 1) {
				update_buffer(me);
			} else {
				handle_buffer_underrun(me);
				me->buffer_underrun = true;
			}
			if (done)
				add_buffer_to_handler(me, done);
		} else {
			int buf_count =
				nx_video_get_buffer_count(&me->vbuf_obj);

			if (buf_count >= 1) {
				update_buffer(me);
				me->buffer_underrun = false;
			}
		}

		install_timer(me);
#ifdef CONFIG_DECIMATOR_USE_DQTIMER
		spin_unlock_irqrestore(&me->lock, flags);
#endif
	}
}

#ifdef CONFIG_DECIMATOR_USE_DQTIMER
static void handle_dq_timeout(unsigned long priv)
{
	struct nx_decimator *me = (struct nx_decimator *)priv;

	dev_info(&me->pdev->dev,  "[DEC %d] DQTimeout\n", me->module);
	process_buffer(me, true);
}
#endif


static void unregister_irq_handler(struct nx_decimator *me)
{
	if (me->irq_entry) {
		nx_vip_unregister_irq_entry(me->module, VIP_DECIMATOR,
				me->irq_entry);
		kfree(me->irq_entry);
		me->irq_entry = NULL;
	}
}

static irqreturn_t nx_decimator_irq_handler(void *data)
{
	struct nx_decimator *me = data;

	process_buffer(me, false);

	return IRQ_HANDLED;
}

static int register_irq_handler(struct nx_decimator *me)
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
	irq_entry->handler = nx_decimator_irq_handler;

	return nx_vip_register_irq_entry(me->module, VIP_DECIMATOR, irq_entry);
}

static int decimator_buffer_queue(struct nx_video_buffer *buf, void *data)
{
	struct nx_decimator *me = data;

	nx_video_add_buffer(&me->vbuf_obj, buf);

	return 0;
}

static int handle_video_connection(struct nx_decimator *me, bool connected)
{
	int ret = 0;

	if (connected)
		ret = nx_video_register_buffer_consumer(&me->vbuf_obj,
							decimator_buffer_queue,
							me);
	else
		nx_video_unregister_buffer_consumer(&me->vbuf_obj);

	return ret;
}

static struct v4l2_subdev *get_remote_source_subdev(struct nx_decimator *me)
{
	struct media_pad *pad =
		media_entity_remote_pad(&me->pads[NX_DECIMATOR_PAD_SINK]);
	if (!pad) {
		dev_err(&me->pdev->dev, "can't find remote source device\n");
		return NULL;
	}
	return media_entity_to_v4l2_subdev(pad->entity);
}

static void set_vip(struct nx_decimator *me, u32 clip_width, u32 clip_height)
{
	nx_vip_set_decimation(me->module, clip_width, clip_height,
			      me->width, me->height);
	nx_vip_set_decimator_format(me->module, me->mem_fmt);
}

static int setup_link(struct media_pad *src, struct media_pad *dst)
{
	struct media_link *link;

	link = media_entity_find_link(src, dst);
	if (link == NULL)
		return -ENODEV;

	return __media_entity_setup_link(link, MEDIA_LNK_FL_ENABLED);
}

/**
 * v4l2 subdev ops
 */
static int nx_decimator_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	u32 module = me->module;
	struct v4l2_subdev *remote;
	void *hostdata_back;

	remote = get_remote_source_subdev(me);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	dev_info(&me->pdev->dev,  "[DEC %d] enable %d\n", me->module, enable);

	ret = down_interruptible(&me->s_stream_sem);
	if (enable) {
		if (NX_ATOMIC_READ(&me->state) & STATE_STOPPING) {
			int timeout = 50; /* 5 second */

			dev_info(&me->pdev->dev,  "wait decimator stopping\n");
			while (NX_ATOMIC_READ(&me->state) &
			       STATE_STOPPING) {
				msleep(100);
				timeout--;
				if (timeout == 0) {
					dev_err(&me->pdev->dev, "timeout for waiting decimator stop\n");
					break;
				}
			}
		}
		if (!(NX_ATOMIC_READ(&me->state) & STATE_RUNNING)) {
			if (nx_vip_is_running(me->module, VIP_DECIMATOR)) {
				dev_err(&me->pdev->dev, "VIP%d Decimator is already running\n",
						me->module);
				nx_video_clear_buffer(&me->vbuf_obj);
				ret = -EBUSY;
				goto UP_AND_OUT;
			}

			hostdata_back = v4l2_get_subdev_hostdata(remote);
			v4l2_set_subdev_hostdata(remote, NX_DECIMATOR_DEV_NAME);
			ret = v4l2_subdev_call(remote, video, s_stream, 1);
			v4l2_set_subdev_hostdata(remote, hostdata_back);
			if (ret) {
				dev_err(&me->pdev->dev,
					"failed to s_stream %d\n", enable);
				nx_video_clear_buffer(&me->vbuf_obj);
				goto UP_AND_OUT;
			}
			set_vip(me, me->width, me->height);
			ret = register_irq_handler(me);
			if (ret) {
				WARN_ON(1);
				goto UP_AND_OUT;
			}
#ifdef CONFIG_DECIMATOR_USE_DQTIMER
			setup_timer(&me->dq_timer, handle_dq_timeout, (long)me);
#endif
			update_buffer(me);
			alloc_dma_buffer(me);
			init_buffer_handler(me);
			nx_vip_run(me->module, VIP_DECIMATOR);
			install_timer(me);
			NX_ATOMIC_SET_MASK(STATE_RUNNING, &me->state);
		}
	} else {
		if (NX_ATOMIC_READ(&me->state) & STATE_RUNNING) {
			NX_ATOMIC_SET_MASK(STATE_STOPPING, &me->state);
			nx_vip_stop(module, VIP_DECIMATOR);
			wait_for_completion_timeout(&me->stop_done, HZ);
			NX_ATOMIC_CLEAR_MASK(STATE_STOPPING, &me->state);
#ifdef CONFIG_DECIMATOR_USE_DQTIMER
			while (timer_pending(&me->dq_timer)) {
				mdelay(DQ_TIMEOUT_MS);
				dev_info(&me->pdev->dev,  "[DEC %d] wait timer done\n",
					 me->module);
			}
#endif
			unregister_irq_handler(me);
			me->buffer_underrun = false;
			free_dma_buffer(me);
			nx_video_clear_buffer(&me->vbuf_obj);
			deinit_buffer_handler(me);
			hostdata_back = v4l2_get_subdev_hostdata(remote);
			v4l2_set_subdev_hostdata(remote, NX_DECIMATOR_DEV_NAME);
			v4l2_subdev_call(remote, video, s_stream, 0);
			v4l2_set_subdev_hostdata(remote, hostdata_back);
			NX_ATOMIC_CLEAR_MASK(STATE_RUNNING, &me->state);
			dev_info(&me->pdev->dev,  "[DEC %d] stop done\n",
				 me->module);
		}
	}

UP_AND_OUT:
	up(&me->s_stream_sem);

	return ret;
}

/**
 * called by VIDIOC_SUBDEV_S_CROP
 */
static int nx_decimator_get_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_selection *sel)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	return v4l2_subdev_call(remote, pad, get_selection, cfg, sel);
}

static int nx_decimator_set_selection(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_selection *sel)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);
	int ret;

	ret = v4l2_subdev_call(remote, pad, set_selection, cfg, sel);
	if (ret < 0) {
		dev_err(&me->pdev->dev, "failed to remote set_selection!\n");
		return ret;
	}

	me->width = sel->r.width;
	me->height = sel->r.height;

	return 0;
}

static int nx_decimator_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *format)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);

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

	return 0;
}

static int nx_decimator_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *format)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);
	/* set memory format */
	u32 nx_mem_fmt;

	int ret = nx_vip_find_nx_mem_format(format->format.code,
					    &nx_mem_fmt);
	if (ret) {
		dev_err(&me->pdev->dev, "Unsupported mem format %d\n",
		       format->format.code);
		return ret;
	}
	me->buf.format = format->format.code;
	me->mem_fmt = nx_mem_fmt;
	me->width = format->format.width;
	me->height = format->format.height;
	format->pad = 1;
	ret = v4l2_subdev_call(remote, pad, set_fmt, NULL, format);
	return ret;
}

static int nx_decimator_enum_frame_size(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_size_enum
						*frame)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	pr_debug("[%s]\n", __func__);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, pad, enum_frame_size, NULL, frame);
}

static int nx_decimator_enum_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_interval_enum
					*frame)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);

	pr_debug("[%s]\n", __func__);
	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, pad, enum_frame_interval, NULL, frame);
}

static int nx_decimator_g_crop(struct v4l2_subdev *sd,
			     struct v4l2_crop *crop)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);
	int err;

	err = v4l2_subdev_call(remote, video, g_crop, crop);
	if (!err) {
		pr_debug("[%s] crop %d:%d:%d:%d\n", __func__, crop->c.left,
				crop->c.top, crop->c.width, crop->c.height);
	}
	return err;
}

static int nx_decimator_s_crop(struct v4l2_subdev *sd,
			     const struct v4l2_crop *crop)
{
	struct nx_decimator *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_source_subdev(me);
	int ret;

	if (me->width < (crop->c.width - crop->c.left) ||
			me->height < (crop->c.height - crop->c.top)) {
		dev_err(&me->pdev->dev, "Invalid scaledown size.\n");
		dev_err(&me->pdev->dev, "The size must be less than");
		dev_err(&me->pdev->dev, " w(%d)xh(%d)\n",
			me->width, me->height);

		return -EINVAL;
	}

	ret = v4l2_subdev_call(remote, video, s_crop, crop);
	if (!ret) {
		pr_debug("[%s] crop %d:%d:%d:%d\n", __func__, crop->c.left,
				crop->c.top, crop->c.width, crop->c.height);
		me->width = crop->c.width;
		me->height = crop->c.height;
	}
	return 0;
}

static const struct v4l2_subdev_video_ops nx_decimator_video_ops = {
	.s_stream = nx_decimator_s_stream,
	.g_crop = nx_decimator_g_crop,
	.s_crop = nx_decimator_s_crop,
};

static const struct v4l2_subdev_pad_ops nx_decimator_pad_ops = {
	.get_selection = nx_decimator_get_selection,
	.set_selection = nx_decimator_set_selection,
	.get_fmt = nx_decimator_get_fmt,
	.set_fmt = nx_decimator_set_fmt,
	.enum_frame_size = nx_decimator_enum_frame_size,
	.enum_frame_interval = nx_decimator_enum_frame_interval,
};

static const struct v4l2_subdev_ops nx_decimator_subdev_ops = {
	.video = &nx_decimator_video_ops,
	.pad = &nx_decimator_pad_ops,
};
/**
 * media_entity_operations
 */
static int nx_decimator_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote,
				 u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct nx_decimator *me = v4l2_get_subdevdata(sd);

	switch (local->index | media_entity_type(remote->entity)) {
	case NX_DECIMATOR_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		pr_debug("decimator sink %s\n",
			 flags & MEDIA_LNK_FL_ENABLED ?
			 "connected" : "disconnected");
		break;
	case NX_DECIMATOR_PAD_SOURCE_MEM | MEDIA_ENT_T_DEVNODE:
		pr_debug("decimator source mem %s\n",
			 flags & MEDIA_LNK_FL_ENABLED ?
			 "connected" : "disconnected");
		handle_video_connection(me, flags & MEDIA_LNK_FL_ENABLED ?
					true : false);
		break;
	}

	return 0;
}

static const struct media_entity_operations nx_decimator_media_ops = {
	.link_setup = nx_decimator_link_setup,
};

/**
 * initialization
 */
static void init_me(struct nx_decimator *me)
{
	NX_ATOMIC_SET(&me->state, STATE_IDLE);
	init_completion(&me->stop_done);
	sema_init(&me->s_stream_sem, 1);
	nx_video_init_vbuf_obj(&me->vbuf_obj);
}

static int init_v4l2_subdev(struct nx_decimator *me)
{
	int ret;
	struct v4l2_subdev *sd = &me->subdev;
	struct media_pad *pads = me->pads;
	struct media_entity *entity = &sd->entity;

	v4l2_subdev_init(sd, &nx_decimator_subdev_ops);
	if (me->logical)
		snprintf(sd->name, sizeof(sd->name), "%s%d%s%d", NX_DECIMATOR_DEV_NAME,
				me->module, "-logical", me->logical_num);
	else
		snprintf(sd->name, sizeof(sd->name), "%s%d", NX_DECIMATOR_DEV_NAME,
				me->module);
	v4l2_set_subdevdata(sd, me);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[NX_DECIMATOR_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[NX_DECIMATOR_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;

	entity->ops = &nx_decimator_media_ops;
	ret = media_entity_init(entity, NX_DECIMATOR_PAD_MAX, pads, 0);
	if (ret < 0) {
		dev_err(&me->pdev->dev, "failed to media_entity_init\n");
		return ret;
	}

	return 0;
}

static int register_v4l2(struct nx_decimator *me)
{
	int ret;
	char dev_name[64] = {0, };
	struct media_entity *entity = &me->subdev.entity;
	struct nx_video *video;
	struct v4l2_subdev *clipper;

	ret = nx_v4l2_register_subdev(&me->subdev);
	if (ret)
		BUG();
	if (me->logical)
		snprintf(dev_name, sizeof(dev_name), "VIDEO DECIMATOR%d%s%d",
			me->module, " LOGICAL", me->logical_num);
	else
		snprintf(dev_name, sizeof(dev_name), "VIDEO DECIMATOR%d",
			me->module);	
	video = nx_video_create(dev_name, NX_VIDEO_TYPE_CAPTURE,
				    nx_v4l2_get_v4l2_device(),
				    nx_v4l2_get_alloc_ctx());
	if (!video)
		BUG();

	ret = media_entity_create_link(entity, NX_DECIMATOR_PAD_SOURCE_MEM,
				       &video->vdev.entity, 0, 0);
	if (ret < 0)
		BUG();

	me->vbuf_obj.video = video;

	ret = setup_link(&entity->pads[NX_DECIMATOR_PAD_SOURCE_MEM],
			&video->vdev.entity.pads[0]);
	if (ret)
		BUG();

	memset(dev_name, 0x0, sizeof(dev_name));
	if (me->logical)
		snprintf(dev_name, sizeof(dev_name), "nx-clipper%d%s%d", me->module,
				"-logical", me->logical_num);
	else
		snprintf(dev_name, sizeof(dev_name), "nx-clipper%d", me->module);
	clipper = nx_v4l2_get_subdev(dev_name);
	if (!clipper) {
		dev_err(&me->pdev->dev, "can't get clipper(%s) subdev\n",
				dev_name);
		return -1;
	}

	ret = media_entity_create_link(&clipper->entity, 2, entity, 0, 0);
	if (ret)
		BUG();

	ret = setup_link(&clipper->entity.pads[2],
			&entity->pads[NX_DECIMATOR_PAD_SINK]);

	return 0;
}

static void unregister_v4l2(struct nx_decimator *me)
{
	if (me->vbuf_obj.video) {
		nx_video_cleanup(me->vbuf_obj.video);
		me->vbuf_obj.video = NULL;
	}
	v4l2_device_unregister_subdev(&me->subdev);
}

static int nx_decimator_parse_dt(struct platform_device *pdev,
				 struct nx_decimator *me)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

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
	return 0;
}

/**
 * platform driver
 */
static int nx_decimator_probe(struct platform_device *pdev)
{
	int ret;
	struct nx_decimator *me;
	struct device *dev = &pdev->dev;

	me = devm_kzalloc(dev, sizeof(*me), GFP_KERNEL);
	if (!me) {
		WARN_ON(1);
		return -ENOMEM;
	}

	ret = nx_decimator_parse_dt(pdev, me);
	if (ret)
		return ret;

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

	me->pdev = pdev;
	me->buf.addr = NULL;
	me->buffer_underrun = false;
	platform_set_drvdata(pdev, me);

#ifdef CONFIG_DECIMATOR_USE_DQTIMER
	spin_lock_init(&me->lock);
#endif
	return 0;
}

static int nx_decimator_remove(struct platform_device *pdev)
{
	struct nx_decimator *me = platform_get_drvdata(pdev);

	if (unlikely(!me))
		return 0;

	unregister_v4l2(me);
	return 0;
}

static struct platform_device_id nx_decimator_id_table[] = {
	{ NX_DECIMATOR_DEV_NAME, 0 },
	{},
};

static const struct of_device_id nx_decimator_dt_match[] = {
	{ .compatible = "nexell,nx-decimator" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_decimator_dt_match);

static struct platform_driver nx_decimator_driver = {
	.probe		= nx_decimator_probe,
	.remove		= nx_decimator_remove,
	.id_table	= nx_decimator_id_table,
	.driver		= {
		.name	= NX_DECIMATOR_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nx_decimator_dt_match),
	},
};

module_platform_driver(nx_decimator_driver);

MODULE_AUTHOR("swpark <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell S5Pxx18 series SoC V4L2 capture decimator driver");
MODULE_LICENSE("GPL");
