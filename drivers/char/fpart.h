/*
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

#ifndef _FPART_H
#define _FPART_H

#define FDEBUG	1/* 0 */

#if FDEBUG
#define DBG(fmt...)	printk(fmt)
#else
#define DBG(fmt...)
#endif

#define HIDDEN_OFFSET		(512*0)	/* 0th page */
#define HIDDEN_PAGE_SIZE	(512)
#define HIDDEN_PAGE_NUM		((512*2)/512) /* 1K */
#define HIDDEN_PACKET_SIZE			20


static long fpart_ioctl(struct file *, unsigned int, unsigned long);
static int fpart_open(struct inode *, struct file *);
static int fpart_release(struct inode*, struct file*);

#define HIDDEN_PARTITION_GET _IOR('T', 0x10, unsigned long)
#define HIDDEN_PARTITION_SET _IOW('T', 0x11, unsigned long)

struct packet_command {
	unsigned char	cmd[HIDDEN_PACKET_SIZE];
	unsigned char	buffer[1024];
	unsigned int	buflen;
};

#endif
