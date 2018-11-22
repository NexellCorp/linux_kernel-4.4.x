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

#define IOC_NX_MAGIC	0x6e78	/* nx */

enum {
	IOCTL_DEINTERLACE_SET_AND_RUN	= _IO(IOC_NX_MAGIC, 1),
	IOCTL_DEINTERLACE_RUNNING	= _IO(IOC_NX_MAGIC, 2),
	IOCTL_SRC_QBUF			= _IO(IOC_NX_MAGIC, 3),
	IOCTL_SRC_DQBUF			= _IO(IOC_NX_MAGIC, 4),
	IOCTL_DST_QBUF			= _IO(IOC_NX_MAGIC, 5),
	IOCTL_DST_DQBUF			= _IO(IOC_NX_MAGIC, 6),
	IOCTL_SET_CONTEXT               = _IO(IOC_NX_MAGIC, 7),
};
#endif
