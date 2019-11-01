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
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>

#include "nx_drm_drv.h"
#include "nx_drm_gem.h"
#include "nx_drm_fb.h"

/*
 * for multiple framebuffers.
 */
static int  fb_align_rgb = 1;
MODULE_PARM_DESC(fb_align, "frame buffer's align (0~4096)");

module_param_named(fb_align, fb_align_rgb, int, 0600);

static void nx_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;

	DRM_DEBUG_KMS("crtc.%d\n", nx_crtc->pipe);

	if (ops && ops->enable)
		ops->enable(crtc);

	drm_crtc_vblank_on(crtc);
}

static void nx_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;

	DRM_DEBUG_KMS("crtc.%d\n", nx_crtc->pipe);

	drm_crtc_vblank_off(crtc);

	if (ops && ops->disable)
		ops->disable(crtc);
}

static int nx_drm_crtc_atomic_check(struct drm_crtc *crtc,
				     struct drm_crtc_state *state)
{
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;

	DRM_DEBUG_KMS("crtc.%d enable:%d, active:%d, changed:%d\n",
		nx_crtc->pipe, crtc->state->enable,
		crtc->state->active, crtc->state->active_changed);

	if (!state->enable)
		return 0;

	if (ops && ops->atomic_check)
		return ops->atomic_check(crtc, state);

	return 0;
}

#define	is_changed(s)	\
		(s->mode_changed ||	\
		s->active_changed ||	\
		s->connectors_changed ? true : false)

static void nx_drm_crtc_atomic_begin(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;
	struct drm_device *drm;
	struct drm_pending_vblank_event *e;

	DRM_DEBUG_KMS("crtc.%d enable:%d, active:%d, %s, %s\n",
		nx_crtc->pipe, crtc->state->enable, crtc->state->active,
		is_changed(crtc->state) ? "changed" : "no change",
		crtc->state->event ? "event" : "no event");

	/* page flip event */
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);
		drm = crtc->dev;
		e = crtc->state->event;
		list_add_tail(&e->base.link, &drm->vblank_event_list);
	}

	if (!crtc->state->enable ||
		!crtc->state->active)
		return;

	if (!is_changed(crtc->state))
		return;

	to_nx_plane(crtc->primary)->align = fb_align_rgb;

	if (ops && ops->begin)
		ops->begin(crtc);
}

static void nx_drm_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;

	DRM_DEBUG_KMS("crtc.%d enable:%d, active:%d, changed:%d\n",
		nx_crtc->pipe, crtc->state->enable,
		crtc->state->active, crtc->state->active_changed);

	if (!crtc->state->enable)
		return;

	if (ops->flush)
		ops->flush(crtc);
}

static struct drm_crtc_helper_funcs nx_crtc_helper_funcs = {
	.enable	= nx_drm_crtc_enable,
	.disable = nx_drm_crtc_disable,
	.atomic_check = nx_drm_crtc_atomic_check,
	.atomic_begin = nx_drm_crtc_atomic_begin,
	.atomic_flush = nx_drm_crtc_atomic_flush,
};

static void nx_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct nx_drm_private *private = crtc->dev->dev_private;
	struct nx_drm_crtc *nx_crtc = to_nx_crtc(crtc);
	struct nx_drm_crtc_ops *ops = nx_crtc->ops;
	int pipe = nx_crtc->pipe;

	DRM_DEBUG_KMS("crtc.%d, %p\n", pipe, to_nx_crtc(private->crtcs[pipe]));

	if (ops && ops->destroy)
		ops->destroy(crtc);

	drm_crtc_cleanup(crtc);

	kfree(to_nx_crtc(private->crtcs[pipe]));
	private->crtcs[pipe] = NULL;
}

static struct drm_crtc_funcs nx_crtc_funcs = {
	.page_flip = drm_atomic_helper_page_flip,
	.set_config = drm_atomic_helper_set_config,
	.destroy = nx_drm_crtc_destroy,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static int of_get_port_nums(struct drm_device *drm,
			int *pipe, int pipe_size)
{
	struct device *dev = &drm->platformdev->dev;
	struct device_node *parent = dev->of_node;
	struct device_node *node, *port;
	int num = 0;

	node = of_get_child_by_name(parent, "ports");
	if (node)
		parent = node;

	for_each_child_of_node(parent, port) {
		u32 port_id = 0;

		if (of_node_cmp(port->name, "port") != 0)
			continue;
		if (of_property_read_u32(port, "reg", &port_id))
			continue;

		pipe[num] = port_id;
		num++;

		if (num > (pipe_size - 1))
			break;
	}

	of_node_put(port);
	of_node_put(node);

	return num;
}

int nx_drm_crtc_create(struct drm_device *drm)
{
	struct nx_drm_private *private;
	struct nx_drm_crtc *nx_crtc;
	int pipes[MAX_CRTCS], num_crtcs = 0;
	int align = fb_align_rgb;
	int i = 0, ret = 0;

	/* get ports with 'reg' property */
	num_crtcs = of_get_port_nums(drm, pipes, ARRAY_SIZE(pipes));

	if (align <= PAGE_SIZE && align > 0)
		fb_align_rgb = align;
	else
		fb_align_rgb = 1;

	DRM_INFO("num of crtcs %d, FB align %d\n", num_crtcs, fb_align_rgb);

	private = drm->dev_private;

	/* setup crtc's planes */
	for (i = 0; num_crtcs > i; i++) {
		int pipe = pipes[i];	/* from reg property */

		nx_crtc = kzalloc(sizeof(struct nx_drm_crtc), GFP_KERNEL);
		if (!nx_crtc)
			goto err_crtc;

		nx_crtc->pipe = pipe;
		nx_crtc->irq = INVALID_IRQ;

		ret = nx_drm_crtc_init(drm, &nx_crtc->crtc, pipe);
		if (ret < 0) {
			if (ret == -ENOENT) {
				kfree(nx_crtc);
				continue;
			}
			goto err_crtc;
		}

		ret = nx_drm_planes_create(drm,
				&nx_crtc->crtc, pipe, &nx_crtc_funcs);
		if (ret < 0) {
			if (ret == -ENOENT) {
				kfree(nx_crtc);
				continue;
			}
		}
		drm_crtc_helper_add(&nx_crtc->crtc, &nx_crtc_helper_funcs);

		private->crtcs[i] = &nx_crtc->crtc;
		private->num_crtcs++;
		DRM_INFO("crtc[%d]: pipe.%d\n", i, pipe);
	}

	return 0;

err_crtc:
	for (i = 0; private->num_crtcs > i; i++) {
		nx_crtc = to_nx_crtc(private->crtcs[i]);
		kfree(nx_crtc);
	}

	return ret;
}

