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

#ifndef _NX_DEINTERLACER_H
#define _NX_DEINTERLACER_H

#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/mutex.h>

#define MAX_BUFFER_PLANES	3
#define SRC_BUFFER_COUNT	3
#define DST_BUFFER_COUNT	1

enum nx_deinter_src_type {
	SRC_TYPE_MIPI = 0ul,
	SRC_TYPE_PARALLEL
};

enum nx_deinter_field {
	EVEN = 0ul,
	ODD = 1ul
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
	enum nx_deinter_field src_field;

	struct frame_data dst_bufs[DST_BUFFER_COUNT];
	struct frame_data src_bufs[SRC_BUFFER_COUNT];
};

enum {
	PROCESSING_STOP,
	PROCESSING_START
};
#endif
