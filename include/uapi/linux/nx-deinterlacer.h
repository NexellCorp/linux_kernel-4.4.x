/*
 * Copyright (C) 2018  Nexell Co., Ltd.
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

#ifndef __UAPI_NX_DEINTERLACER_H
#define __UAPI_NX_DEINTERLACER_H

#include <linux/ioctl.h>

#if defined(__cplusplus)
extern "c" {
#endif

#define MAX_BUFFER_PLANES	3
#define SRC_BUFFER_COUNT	3
#define DST_BUFFER_COUNT	1

#define IOC_NX_MAGIC	0x6e78	/* nx */

enum {
	IOCTL_DEINTERLACE_SET_AND_RUN	= _IO(IOC_NX_MAGIC, 1),
};

enum nx_deinter_src_type {
	SRC_TYPE_MIPI = 0ul,
	SRC_TYPE_PARALLEL
};

enum nx_deinter_src_field {
	FIELD_EVEN = 0ul,
	FIELD_ODD = 1ul
};

enum {
	ACT_COPY = 0ul,
	ACT_DIRECT,
	ACT_DIRECT_FD,
	ACT_THREAD,
} ACT_MODE;

enum {
	FRAME_SRC = 1ul,
	FRAME_DST
};

enum nx_deinter_mode {
	SINGLE_FRAME = 0ul,
	DOUBLE_FRAME
};

enum {
	PLANE2 = 2ul,
	PLANE3,
};

struct frame_data {
	int frame_num;
	int plane_num;
	int frame_type;
	int frame_factor;

	union {
		struct {
			unsigned char *virt[MAX_BUFFER_PLANES];
			unsigned long sizes[MAX_BUFFER_PLANES];
			unsigned long src_stride[MAX_BUFFER_PLANES];
			unsigned long dst_stride[MAX_BUFFER_PLANES];

			int fds[MAX_BUFFER_PLANES];
			unsigned long phys[MAX_BUFFER_PLANES];
		} plane3;

		struct {
			unsigned char *virt[MAX_BUFFER_PLANES-1];
			unsigned long sizes[MAX_BUFFER_PLANES-1];
			unsigned long src_stride[MAX_BUFFER_PLANES-1];
			unsigned long dst_stride[MAX_BUFFER_PLANES-1];

			int fds[MAX_BUFFER_PLANES-1];
			unsigned long phys[MAX_BUFFER_PLANES-1];
		} plane2;
	};
};

struct frame_data_info {
	int command;
	int width;
	int height;
	int plane_mode;
	enum nx_deinter_src_type src_type;
	enum nx_deinter_src_field src_field;

	struct frame_data dst_bufs[DST_BUFFER_COUNT];
	struct frame_data src_bufs[SRC_BUFFER_COUNT];
};

#if defined(__cplusplus)
}
#endif

#endif
