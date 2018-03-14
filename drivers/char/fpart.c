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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/blkpg.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/buffer_head.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/completion.h>

#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/memory.h>
#include <asm/dma.h>

#include "fpart.h"

#define DEV_NAME    "fpart"

static int major;
static struct class *fpart_class;
static struct device *fpart_dev;
static struct packet_command fhidden;
struct completion fpart_completion;

static char path[30];
static unsigned char data1[512];
#define DEBUG 1
#define PARTITION_READ_DEBUG 0
#define USERSPACE_READ_DEBUG 0

static void fpart_get_path(char *path)
{
	sprintf(path, "/dev/mmcblk0p%d", 5);
}

static int get_data(void *output_data)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int ret, err, count, i;

	loff_t offset = HIDDEN_OFFSET;

	oldfs = get_fs();
	set_fs(get_ds());
	fpart_get_path(path);
	filp = filp_open(path, O_RDWR, GFP_KERNEL);

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		goto out;
	}
	for (count = 0; count < HIDDEN_PAGE_NUM; count++) {
		offset = HIDDEN_OFFSET + HIDDEN_PAGE_SIZE*count;
		ret = vfs_read(filp, (char *)&data1[0], 512, &offset);

	if (ret < 0)
		goto err;

		memcpy(fhidden.buffer+(HIDDEN_PAGE_SIZE*count), data1, HIDDEN_PAGE_SIZE);
	}

	filp_close(filp, NULL);
	set_fs(oldfs);
	return 0;

err:
	filp_close(filp, NULL);
	set_fs(oldfs);

out:
	return -1;

}

static int set_data(void *output_data)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int ret, err, count, i;
	unsigned char temp_data[512];

	loff_t offset = HIDDEN_OFFSET;

	oldfs = get_fs();
	set_fs(get_ds());

	fpart_get_path(path);
	filp = filp_open(path, O_RDWR, GFP_KERNEL);

	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		goto out;
	}
	char *tt;

	tt = (char *)output_data;

	for (count = 0; count < HIDDEN_PAGE_NUM; count++) {
		offset = HIDDEN_OFFSET + HIDDEN_PAGE_SIZE*count;
		memcpy(temp_data, (char *)(output_data + HIDDEN_PAGE_SIZE*count), HIDDEN_PAGE_SIZE);
		ret = vfs_write(filp, temp_data, HIDDEN_PAGE_SIZE, &offset);
		if (ret < 0)
			goto err;

	}

	filp_close(filp, NULL);
	set_fs(oldfs);
	return 0;

err:
	filp_close(filp, NULL);
	set_fs(oldfs);

out:
	return -1;
}

int fpart_get_data(void *data)
{
	get_data(data);
	complete_and_exit(&fpart_completion, 0);
}

int fpart_set_data(void *data)
{
	int ret;

	ret = set_data(data);
	complete_and_exit(&fpart_completion, 0);
	return ret;
}

static long fpart_ioctl(struct file *filp, unsigned int cmd, unsigned long user)
{
	int res, i;
	u8 data[1024*2];
	void __user *userp = (void __user *) user;
	struct task_struct *tsk;

	switch (cmd) {
	case HIDDEN_PARTITION_GET:
		tsk = kthread_run(fpart_get_data, NULL, "fhidden");
		if (IS_ERR(tsk)) {
			complete(&fpart_completion);
			res = -1;
		} else {
			wait_for_completion(&fpart_completion);
			res = 0;
		}
		if (copy_to_user(user, (const void *)fhidden.buffer, sizeof(fhidden.buffer)))
			return -EFAULT;

		break;
	case HIDDEN_PARTITION_SET:
		if (copy_from_user((void *)&data[0], (const void *)user, sizeof(data)))
			return -EFAULT;

		tsk = kthread_run(fpart_set_data, &data[0], "output_setting");
		if (IS_ERR(tsk))
			complete(&fpart_completion);
		else
			wait_for_completion(&fpart_completion);
		break;

	default:
		res = -1;
		break;
}

	return res;
}

static int fpart_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);
	return 0;
}

static int fpart_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;
}

const static struct file_operations fpart_fops = {
	.owner  = THIS_MODULE,
	.open   = fpart_open,
	.release    = fpart_release,
	.read   = NULL,
	.write  = NULL,
	.unlocked_ioctl = fpart_ioctl
};

static int __init fpart_init(void)
{
	void *ptr_err;
	char data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	struct task_struct *tsk;

	init_completion(&fpart_completion);
	major = register_chrdev(0, DEV_NAME, &fpart_fops);
	if (major < 0)
		return major;

	fpart_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(fpart_class))
		goto err2;

	fpart_dev = device_create(fpart_class, NULL, MKDEV(major, 0), NULL, DEV_NAME);
	if (IS_ERR(fpart_dev))
		goto err;

	return 0;

err:
	class_destroy(fpart_class);
err2:
	unregister_chrdev(major, DEV_NAME);
	return PTR_ERR(ptr_err);
}

static void __exit fpart_exit(void)
{
	device_destroy(fpart_class, MKDEV(major, 0));
	class_unregister(fpart_class);
	class_destroy(fpart_class);
	unregister_chrdev(major, DEV_NAME);
}

module_init(fpart_init);
module_exit(fpart_exit);

MODULE_AUTHOR("FMS");
MODULE_DESCRIPTION("FMS fpart Driver");
MODULE_LICENSE("GPL");
