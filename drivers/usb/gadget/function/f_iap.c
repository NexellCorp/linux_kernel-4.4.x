/*
 * f_iap.c - USB peripheral iap configuration driver

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* #define VERBOSE_DEBUG */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include "f_iap.h"
#include "u_iap.h"

#define WRITE_BUFLEN (4096)
#define READ_BUFLEN  (4096 * 2)
#define WAIT_TIMEOUT (100)       //100ms

/* Platform specific changes for freescale sabre boards */
#define FREESCALE_SABREAUTO
#ifdef FREESCALE_SABREAUTO
#define ENABLE_INTERRUPT_EP
#endif

/* Platform specific Fixes for nvidia t3 board */
#ifdef NVIDIA_T3
#define ENABLE_INTERRUPT_EP
#define NVIDIA_T3_DISCONN_FIX
#endif

/* Platform specific Fixes for Renesas */
#ifdef RENESAS
#define ENABLE_INTERRUPT_EP
#endif

/* Platform specific Fixes for TI J6 JellyBean */
#ifdef TI_J6_JB
#define J6_JB_FIXES
#endif

/* Platform specific Fixes for nvidia k1 board */
#ifdef NVIDIA_K1
#define ENABLE_INTERRUPT_EP
#endif

#ifdef NVIDIA_T3_DISCONN_FIX
static struct android_dev *_android_dev = NULL;
#endif

static int major, minors;
static struct class *ipodg_class;
static int iap_setup(struct usb_function *f,const struct usb_ctrlrequest *ctrl);
struct work_struct work;
#ifdef J6_JB_FIXES
extern int modeswitch;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
int giap_setup(struct usb_gadget *g, int count);
void giap_cleanup(void);
#endif

struct f_iap{
	struct cdev			    cdev;
	spinlock_t			    spinlock;
	wait_queue_head_t		read_queue;
	wait_queue_head_t		write_queue;
	struct mutex			lock_write;
	struct mutex			lock_read;
	bool				    write_pending;
	struct usb_request		*req_in_ep;
	struct usb_request		*req_out_ep;
	struct usb_request		*req_intr_ep;
	struct usb_function		func;
	struct usb_ep		    *in_ep;
	struct usb_ep		    *out_ep;
	struct usb_ep           *intr_ep;
	size_t count;
	int online;
	int error;
	bool                    connected;
	bool                    disconnected;
	atomic_t read_excl;
	struct list_head tx_idle;
	int rx_done;
#ifdef NVIDIA_T3_DISCONN_FIX
	struct device           *dev;
#endif
};

struct f_iap *g_ss;

static inline struct f_iap *func_to_ss(struct usb_function *f)
{
	return container_of(f, struct f_iap, func);
}

static struct usb_interface_descriptor ipod_out_intf = {
	.bLength =		sizeof ipod_out_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

#ifdef ENABLE_INTERRUPT_EP
	.bNumEndpoints =	3,
#else
	.bNumEndpoints =	2,
#endif
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =   USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol =   0x00,
	.iInterface =           0x00,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

#ifdef ENABLE_INTERRUPT_EP
static struct usb_endpoint_descriptor fs_interrupt_desc= {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(16),
	.bInterval =		1 ,
};
#endif

static struct usb_endpoint_descriptor fs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_ipod_out_descs[] = {
	(struct usb_descriptor_header *) &ipod_out_intf,
	(struct usb_descriptor_header *) &fs_source_desc,
	(struct usb_descriptor_header *) &fs_sink_desc,
#ifdef ENABLE_INTERRUPT_EP
	(struct usb_descriptor_header *) &fs_interrupt_desc,
#endif
	NULL,
};

/* high speed support: */

#ifdef ENABLE_INTERRUPT_EP
static struct usb_endpoint_descriptor hs_interrupt_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(16),
	.bInterval =		1,
};
#endif

static struct usb_endpoint_descriptor hs_source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *hs_ipod_out_descs[] = {
	(struct usb_descriptor_header *) &ipod_out_intf,
	(struct usb_descriptor_header *) &hs_source_desc,
	(struct usb_descriptor_header *) &hs_sink_desc,
#ifdef ENABLE_INTERRUPT_EP
	(struct usb_descriptor_header *) &hs_interrupt_desc,
#endif
	NULL,
};

static void disable_ipod_ep(struct usb_composite_dev *cdev, struct usb_ep *ep)
{
	int			value;

	if (ep->driver_data) {
		value = usb_ep_disable(ep);
		if (value < 0)
			DBG(cdev, "disable %s --> %d\n",
					ep->name, value);
		ep->driver_data = NULL;
	}
        return;
}

void disable_ipod_endpoints(struct usb_composite_dev *cdev,
		struct usb_ep *in, struct usb_ep *out, struct usb_ep *intr)
{
	disable_ipod_ep(cdev, in);
	disable_ipod_ep(cdev, out);
#ifdef ENABLE_INTERRUPT_EP
	if (intr)
		disable_ipod_ep(cdev, intr);
#endif
	return;
}

static void f_ipodg_req_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct f_iap *ipodg = (struct f_iap *)ep->driver_data;

	if (req->status != 0) {
		if (req->status == -ECONNRESET)		/* Request was cancelled */
			usb_ep_fifo_flush(ep);
	}

	spin_lock(&ipodg->spinlock);
	ipodg->rx_done = 1;
	spin_unlock(&ipodg->spinlock);

	wake_up_interruptible(&ipodg->read_queue);
}

static ssize_t f_ipodg_read(struct file *file, char __user *buffer,
		size_t count, loff_t *ptr)
{
	struct f_iap	*ipodg     = file->private_data;
	ssize_t status = -ENOMEM;

	if (!count)
		return 0;

	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	mutex_lock(&ipodg->lock_read);

	/* If the driver is not online or is in error then wait.
	 * TODO: Should we not just return with an error? */
	spin_lock_irq(&ipodg->spinlock);

	while (!(ipodg->online || ipodg->error)) {
		spin_unlock_irq(&ipodg->spinlock);
		status = wait_event_interruptible(ipodg->read_queue,
				(ipodg->online || ipodg->error));
		if (status < 0) {
			goto done;
		} else {
			spin_lock_irq(&ipodg->spinlock);
		}
	}

	if (ipodg->error) {
		status = -EIO;
		spin_unlock_irq(&ipodg->spinlock);
		goto done;
	}


requeue_req:
	/* queue a request */
	ipodg->req_out_ep->status   = 0;
	ipodg->req_out_ep->length   = count;
	ipodg->req_out_ep->complete = f_ipodg_req_complete_out;
	ipodg->req_out_ep->context  = ipodg;
	ipodg->rx_done = 0;

	spin_unlock_irq(&ipodg->spinlock);

	status = usb_ep_queue(ipodg->out_ep, ipodg->req_out_ep, GFP_ATOMIC);
	if (status < 0) {
		ERROR(ipodg->func.config->cdev, "usb_ep_queue error on bulk out endpoint: %d\n", status);
		goto done;
	}

	/* wait for a request to complete */
	spin_lock_irq(&ipodg->spinlock);

	while (ipodg->rx_done == 0) {
		spin_unlock_irq(&ipodg->spinlock);
#ifdef RENESAS
		status = wait_event_interruptible_timeout(ipodg->read_queue, (ipodg->rx_done == 1),WAIT_TIMEOUT);
#else
		status = wait_event_interruptible(ipodg->read_queue, (ipodg->rx_done == 1));
#endif

		if (ipodg->req_out_ep->status < 0) {
			/* See if the device is still is connected state */
			if(ipodg->connected)
			{
				usb_ep_fifo_flush(ipodg->out_ep);
				usb_ep_dequeue(ipodg->out_ep, ipodg->req_out_ep);
			}
			goto done;
		}

#ifdef RENESAS
		if (status <= 0 ) {
#else
        if (status != 0) {
#endif
			DBG (ipodg->func.config->cdev, "Wait interrupt failed %d %d %d\n",
					status, ipodg->req_out_ep->status, ipodg->rx_done);
			if (status == -ERESTARTSYS && ipodg->rx_done == 1) {
				printk(KERN_DEBUG "In read line = %d\n", __LINE__);
				ipodg->req_out_ep->status = 0;
			} else {
				usb_ep_fifo_flush(ipodg->out_ep);
				usb_ep_dequeue(ipodg->out_ep, ipodg->req_out_ep);
				goto done;
			}
		}
		spin_lock_irq(&ipodg->spinlock);
	}
	ipodg->rx_done = 0;

	spin_unlock_irq(&ipodg->spinlock);

	/* If we got a 0-len packet, throw it back and try again. */
	if (ipodg->req_out_ep->actual == 0) {
		if (ipodg->req_out_ep->status == 0) {
			DBG (ipodg->func.config->cdev, "zero packet\n");
			goto requeue_req;
		} else {
			usb_ep_fifo_flush(ipodg->out_ep);
			usb_ep_dequeue(ipodg->out_ep, ipodg->req_out_ep);
			status = -EIO;
			goto done;
		}
	}

	pr_debug("rx %p %d\n", ipodg->req_out_ep, ipodg->req_out_ep->actual);

	if (ipodg->req_out_ep->status == 0) {
		status = (ipodg->req_out_ep->actual < count) ? ipodg->req_out_ep->actual : count;
		if (copy_to_user(buffer, ipodg->req_out_ep->buf, status) != 0) {
			status = -EFAULT;
			goto done;
		}
	} else {
		ERROR (ipodg->func.config->cdev, "Read error\n");
		status = -EIO;
	}

done:
	mutex_unlock(&ipodg->lock_read);
	return status;
}

static void f_ipodg_req_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct f_iap *ipodg = (struct f_iap *)ep->driver_data;

	if (req->status != 0) {
		if (req->status == -ECONNRESET)		/* Request was cancelled */
			usb_ep_fifo_flush(ep);
	}

	spin_lock(&ipodg->spinlock);
	ipodg->write_pending = 0;
	spin_unlock(&ipodg->spinlock);
	wake_up_interruptible(&ipodg->write_queue);
}

static ssize_t f_ipodg_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *offp)
{
	struct f_iap *ipodg  = file->private_data;
	ssize_t status = -ENOMEM;

	if (!access_ok(VERIFY_READ, buffer, count))
	{
		printk(KERN_ALERT "access denied.\n");
		printk("\nipogd_write exit %d\n", __LINE__);
		return -EFAULT;
	}

	mutex_lock(&ipodg->lock_write);

#ifdef ENABLE_INTERRUPT_EP
	spin_lock_irq(&ipodg->spinlock);
	ipodg->req_intr_ep->status   = 0;
	ipodg->req_intr_ep->zero     = 1;
	ipodg->req_intr_ep->length   = 0;
	ipodg->req_intr_ep->complete = f_ipodg_req_complete_in;
	ipodg->req_intr_ep->context  = ipodg;
	ipodg->write_pending = 1;
	spin_unlock_irq(&ipodg->spinlock);

	status = usb_ep_queue(ipodg->intr_ep, ipodg->req_intr_ep, GFP_ATOMIC);
	if (status < 0) {
		ipodg->write_pending = 0;
		ERROR(ipodg->func.config->cdev, "usb_ep_queue error on interrupt endpoint: %d\n", status);
		mutex_unlock(&ipodg->lock_write);
		return status;
	}
#endif
	while (ipodg->write_pending) {
		mutex_unlock(&ipodg->lock_write);

		status = wait_event_interruptible(ipodg->write_queue, (ipodg->write_pending == 0));
		if (status == -ERESTARTSYS) {
			ipodg->req_intr_ep->status = 0;
		} else if (status != 0) {
			DBG(ipodg->func.config->cdev, "wait_event_interruptible int: %d\n", status);
			return status;
		}

		mutex_lock(&ipodg->lock_write);
	}

	status = copy_from_user(ipodg->req_in_ep->buf, buffer, (WRITE_BUFLEN < count)? WRITE_BUFLEN:count);
	if (status != 0) {
		ERROR(ipodg->func.config->cdev, "copy_from_user error\n");
		mutex_unlock(&ipodg->lock_write);
		return -EINVAL;
	}

	spin_lock_irq(&ipodg->spinlock);
	ipodg->req_in_ep->length = WRITE_BUFLEN;
	ipodg->req_in_ep->status   = 0;
	ipodg->req_in_ep->zero     = 0;
	ipodg->req_in_ep->length   = (ipodg->req_in_ep->length < count)?ipodg->req_in_ep->length:count;
	ipodg->req_in_ep->complete = f_ipodg_req_complete_in;
	ipodg->req_in_ep->context  = ipodg;
	ipodg->write_pending = 1;
	spin_unlock_irq(&ipodg->spinlock);

	status = usb_ep_queue(ipodg->in_ep, ipodg->req_in_ep, GFP_ATOMIC);
	if (status < 0) {
		ipodg->write_pending = 0;
		ERROR(ipodg->func.config->cdev, "usb_ep_queue error on bulk in endpoint: %d\n", status);
		mutex_unlock(&ipodg->lock_write);
		return status;
	}

	while (ipodg->write_pending) {
		mutex_unlock(&ipodg->lock_write);
		status = wait_event_interruptible_timeout(ipodg->write_queue, (ipodg->write_pending == 0),WAIT_TIMEOUT);
		if (status == -ERESTARTSYS) {
			ipodg->req_in_ep->status = 0;
		} else if (status <= 0) {
			DBG(ipodg->func.config->cdev, "wait_event_interruptible bulk in: %d\n", status);
			return status;
		}

		mutex_lock(&ipodg->lock_write);
	}

	mutex_unlock(&ipodg->lock_write);
	return count;
}

static int f_ipodg_release(struct inode *inode, struct file *fd)
{
	fd->private_data = NULL;
	printk(KERN_ALERT "in %s   %s \n",__FILE__,__func__);

	return 0;
}

static int f_ipodg_open(struct inode *inode, struct file *fd)
{
	struct f_iap *ipodg =
		container_of(inode->i_cdev, struct f_iap, cdev);

	fd->private_data = ipodg;
	printk(KERN_ALERT "in %s   %s \n",__FILE__,__func__);
	ipodg->error = 0;
	return 0;
}

const struct file_operations f_ipodg_fops = {
	.owner		= THIS_MODULE,
	.open		= f_ipodg_open,
	.release	= f_ipodg_release,
	.write		= f_ipodg_write,
	.read		= f_ipodg_read,

};

static void handle_node(void *tss)
{
	struct f_iap *ss = g_ss;
	dev_t dev;

	if(ss->connected)
	{
                /* Added to fix renesas disconnection issue.
                   Can be used for others also
                */
		if(!ss->disconnected)
		{
			/* Delete the node if not deleted */
			device_destroy(ipodg_class, MKDEV(major, 0));
			cdev_del(&ss->cdev);
			printk("Device node deleted\n");

		}
		/* create char device */
		cdev_init(&ss->cdev, &f_ipodg_fops);
		dev = MKDEV(major, 0);
		cdev_add(&ss->cdev, dev, 1);
#ifdef NVIDIA_T3_DISCONN_FIX
		ss->dev = device_create(ipodg_class, NULL, dev, NULL, "%s%d", "ipodout", 0);
#else
		device_create(ipodg_class, NULL, dev, NULL, "%s%d", "ipodout", 0);
#endif
		ss->disconnected = 0;

#ifdef DISCONN_FIX
		if(_android_dev != NULL) {
			printk(KERN_INFO "[IAP_GADGET]: In func:%s lineno: %d\n", __func__, __LINE__);
			_android_dev->dev = ss->dev;
			usb_composite_register_dev(_android_dev);
		}
#endif
		printk("ipodout device node created\n");
	}
	else
	{
		if(!ss->disconnected) {
			device_destroy(ipodg_class, MKDEV(major, 0));
			cdev_del(&ss->cdev);
			ss->disconnected = 1;
			printk("ipodout device node deleted\n");
		} else {
			printk("ipodout device already deleted !!\n");
		}
	}
}

static int iap_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_iap	*ss = func_to_ss(f);
	int	status, id;
	struct usb_ep		*in_ep, *out_ep;
#ifdef ENABLE_INTERRUPT_EP
	struct usb_ep       *intr_ep;
#endif
	printk(KERN_ALERT "%s   %s \n",__FILE__,__func__);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	/* allocate string ID(s) */
	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	strings_iap[0].id = id;

	ipod_out_intf.iInterface = id;
#endif
	/* allocate interface ID(s) */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	ipod_out_intf.bInterfaceNumber = status;

	printk("Interface id is:: %d\n",status);

	/* allocate instance-specific endpoints */
	status = -ENODEV;
	in_ep = usb_ep_autoconfig(c->cdev->gadget, &fs_source_desc);
	if (!in_ep)
		goto fail;
	in_ep->driver_data = c->cdev;	/* claim */
	ss->in_ep = in_ep;

	/* preallocate request and buffer */
	status = -ENOMEM;
	ss->req_in_ep = usb_ep_alloc_request(ss->in_ep, GFP_KERNEL);
	if (!ss->req_in_ep)
		goto fail;

	ss->req_in_ep->length = WRITE_BUFLEN;
	ss->req_in_ep->buf = kmalloc(ss->req_in_ep->length, GFP_KERNEL);
	if (!ss->req_in_ep->buf)
		goto fail;

	status = -ENODEV;
	out_ep = usb_ep_autoconfig(c->cdev->gadget, &fs_sink_desc);
	if (!out_ep)
		goto fail;
	out_ep->driver_data = c->cdev;	/* claim */
	ss->out_ep = out_ep;

	/* preallocate request and buffer */
	status = -ENOMEM;
	ss->req_out_ep = usb_ep_alloc_request(ss->out_ep, GFP_KERNEL);
	if (!ss->req_out_ep)
		goto fail;

	ss->req_out_ep->length = READ_BUFLEN;
	ss->req_out_ep->buf = kmalloc(ss->req_out_ep->length, GFP_KERNEL);
	if (!ss->req_out_ep->buf)
		goto fail;
#ifdef ENABLE_INTERRUPT_EP
	status = -ENODEV;
	intr_ep = usb_ep_autoconfig(c->cdev->gadget, &fs_interrupt_desc);
	if (!intr_ep)
		goto fail;
	intr_ep->driver_data = c->cdev;	/* claim */
	ss->intr_ep = intr_ep;

	/* preallocate request and buffer */
	status = -ENOMEM;
	ss->req_intr_ep = usb_ep_alloc_request(ss->intr_ep, GFP_KERNEL);
	if (!ss->req_intr_ep)
		goto fail;

	ss->req_intr_ep->length = 64;
	ss->req_intr_ep->buf = kmalloc(ss->req_intr_ep->length, GFP_KERNEL);
	if (!ss->req_intr_ep->buf)
		goto fail;
#endif
	/* copy descriptors */
	f->hs_descriptors = usb_copy_descriptors(hs_ipod_out_descs);
	if (!f->hs_descriptors)
		goto fail;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_source_desc.bEndpointAddress =
			fs_source_desc.bEndpointAddress;
		hs_sink_desc.bEndpointAddress =
			fs_sink_desc.bEndpointAddress;
#ifdef ENABLE_INTERRUPT_EP
		hs_interrupt_desc.bEndpointAddress =
			fs_interrupt_desc.bEndpointAddress;
#endif
		f->hs_descriptors = usb_copy_descriptors(hs_ipod_out_descs);
		if (!f->hs_descriptors)
			goto fail;
	}

#ifdef NVIDIA_T3_DISCONN_FIX
	printk(KERN_INFO "[IAP_GADGET]: In func:%s lineno: %d\n", __func__, __LINE__);
	_android_dev = (struct android_dev *) kzalloc(sizeof(struct android_dev), GFP_KERNEL);
#endif
	printk("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, ss->in_ep->name, ss->out_ep->name);

	mutex_init(&ss->lock_write);
	mutex_init(&ss->lock_read);
	spin_lock_init(&ss->spinlock);
	init_waitqueue_head(&ss->write_queue);
	init_waitqueue_head(&ss->read_queue);
	atomic_set(&ss->read_excl, 0);

	ss->connected    = 0;
	ss->disconnected = 1;
	g_ss = ss;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	status = giap_setup(c->cdev->gadget, 1);
	if(status < 0)
		goto fail;
#endif

	INIT_WORK(&work, handle_node);
	return 0;

fail:
	if (ss->req_in_ep != NULL) {
		kfree(ss->req_in_ep->buf);
		if (ss->in_ep != NULL)
			usb_ep_free_request(ss->in_ep, ss->req_in_ep);
	}
	if (ss->req_out_ep != NULL) {
		kfree(ss->req_out_ep->buf);
		if (ss->in_ep != NULL)
			usb_ep_free_request(ss->in_ep, ss->req_out_ep);
	}
	if (ss->req_intr_ep != NULL) {
		kfree(ss->req_intr_ep->buf);
		if (ss->in_ep != NULL)
			usb_ep_free_request(ss->in_ep, ss->req_intr_ep);
	}

	return status;
}

static void iap_unbind(struct usb_configuration *c, struct usb_function *f)
{

	struct f_iap	*ss = func_to_ss(f);
	printk(KERN_ALERT "in %s   %s \n",__FILE__,__func__);

	ss->online = 0;
	ss->error = 1;
	wake_up_interruptible(&ss->read_queue);

#ifdef NVIDIA_T3_DISCONN_FIX
	printk(KERN_INFO "[IAP_GADGET]: cancel_delayed_work_sync .....\n");
	cancel_delayed_work_sync(&disconnect_work);
	_android_dev->dev = NULL;
#endif
	schedule_work(&work);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	giap_cleanup();
#else
	kfree(func_to_ss(f));
#endif
#ifdef NVIDIA_T3_DISCONN_FIX
	printk(KERN_INFO "[IAP_GADGET]: In func:%s lineno: %d\n", __func__, __LINE__);
	kfree(_android_dev);
	_android_dev = NULL;
#endif

	kfree(func_to_ss(f));
}

static void disable_ipod_out(struct f_iap *ss)
{
	struct usb_composite_dev	*cdev;

	printk(KERN_ALERT "in %s   %s \n",__FILE__,__func__);

	cdev = ss->func.config->cdev;
	ss->connected = 0;
	g_ss = ss;
#ifdef ENABLE_INTERRUPT_EP
	disable_ipod_endpoints(cdev, ss->in_ep, ss->out_ep, ss->intr_ep);
#else
	disable_ipod_endpoints(cdev, ss->in_ep, ss->out_ep, NULL);
#endif
#ifdef J6_JB_FIXES
	modeswitch &= ~(1<<2);
#endif
#ifdef NVIDIA_T3_DISCONN_FIX
	printk(KERN_INFO "[IAP_GADGET]: cancel_delayed_work_sync .....\n");
	cancel_delayed_work_sync(&disconnect_work);
	_android_dev->dev = NULL;
#endif
	ss->write_pending = 0;
	wake_up_interruptible(&ss->write_queue);
	ss->rx_done = 1;
	wake_up_interruptible(&ss->read_queue);
	schedule_work(&work);

	VDBG(cdev, "%s disabled\n", ss->func.name);
}

static int enable_ipod_out(struct usb_composite_dev *cdev,struct usb_function *f,struct f_iap *ss)
{
	int        result = 0;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 0, 35)
	const struct usb_endpoint_descriptor    *src, *sink, *interrupt;
#endif
	printk(KERN_ALERT "%s   %s \n",__FILE__,__func__);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 35)
	if ( config_ep_by_speed(cdev->gadget,f,ss->in_ep) ||
	     config_ep_by_speed(cdev->gadget,f,ss->out_ep)
#ifdef ENABLE_INTERRUPT_EP
	     ||
	     config_ep_by_speed(cdev->gadget,f,ss->intr_ep)
#endif
	   ){
		goto fail;
	}
#else
	src = ep_choose(cdev->gadget, &hs_source_desc, &fs_source_desc);
	sink = ep_choose(cdev->gadget, &hs_sink_desc, &fs_sink_desc);
#ifdef ENABLE_INTERRUPT_EP
	interrupt = ep_choose(cdev->gadget, &hs_interrupt_desc, &fs_interrupt_desc);
#endif
#endif

	/* Enable interrupt endpoint if needed */
#ifdef ENABLE_INTERRUPT_EP
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 35)
	result = usb_ep_enable(ss->intr_ep);
#else
	result = usb_ep_enable(ss->intr_ep, interrupt);
#endif
	if (result < 0)
		goto fail;
	ss->intr_ep->driver_data = ss;
#endif

	/* Enable bulk-in endpoint */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 35)
	result = usb_ep_enable(ss->in_ep);
#else
	result = usb_ep_enable(ss->in_ep, src);
#endif
	if (result < 0)
		goto fail;
	ss->in_ep->driver_data = ss;

	/* Enable bulk-out endpoint */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 35)
	result = usb_ep_enable(ss->out_ep);
#else
	result = usb_ep_enable(ss->out_ep, sink);
#endif
	if (result < 0)
		goto fail;
	ss->out_ep->driver_data = ss;

#ifdef J6_JB_FIXES
	modeswitch|=1<<2;
#endif
	ss->connected = 1;
	schedule_work(&work);
#ifdef NVIDIA_T3_DISCONN_FIX
	if(_android_dev != NULL) {
		printk(KERN_INFO "[IAP_GADGET]: In func:%s lineno: %d\n", __func__, __LINE__);
		_android_dev->dev = ss->dev;
		usb_composite_register_dev(_android_dev);
	}
#endif
	ss->rx_done = 1;
	wake_up_interruptible(&ss->read_queue);

	DBG(cdev, "%s enabled\n", ss->func.name);
fail :
	return result;
}

static int iap_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct f_iap	*ss = func_to_ss(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	printk(KERN_ALERT "%s   %s \n",__FILE__,__func__);
	ss->online = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up_interruptible(&ss->read_queue);
	wake_up_interruptible(&ss->write_queue);
	// we know alt is zero
	if (ss->in_ep->driver_data)
		disable_ipod_out(ss);
	return enable_ipod_out(cdev,f, ss);
}

static void iap_disable(struct usb_function *f)
{
	struct f_iap	*ss = func_to_ss(f);

	printk(KERN_ALERT "%s   %s \n",__FILE__,__func__);
	ss->online = 0;
	ss->error = 1;

	disable_ipod_out(ss);

	wake_up_interruptible(&ss->read_queue);
	ss->rx_done = 1;
	wake_up_interruptible(&ss->read_queue);
	return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
static void iap_free(struct usb_function *f)
{
	struct f_iap *iap;
	struct f_iap_opts *opts;

	iap = func_to_ss(f);
	opts = container_of(f->fi, struct f_iap_opts, func_inst);
	kfree(iap);
	mutex_lock(&opts->lock);
	opts->refcnt--;
	mutex_unlock(&opts->lock);
}

static struct usb_function *iap_alloc(struct usb_function_instance *fi)
{
	struct f_iap	*ss;
	struct f_iap_opts  *opts;

	printk(KERN_ALERT "%s   %s \n",__FILE__,__func__);
	ss = kzalloc(sizeof (*ss), GFP_KERNEL);
	if (!ss)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_iap_opts, func_inst);
	mutex_lock(&opts->lock);
	opts->refcnt++;
	mutex_unlock(&opts->lock);

	ss->func.name = "source/sink";
	ss->func.fs_descriptors = fs_ipod_out_descs;
	ss->func.strings = iap_strings;
	ss->func.bind = iap_bind;
	ss->func.unbind = iap_unbind;
	ss->func.set_alt = iap_set_alt;
	ss->func.disable = iap_disable;
	ss->func.suspend = iap_disable;
	ss->func.setup = iap_setup;
	ss->func.free_func = iap_free;

	return &ss->func;
}

static void iap_free_inst(struct usb_function_instance *f)
{
	struct f_iap_opts *opts;

	opts = container_of(f, struct f_iap_opts, func_inst);
	kfree(opts);
}

static struct usb_function_instance *iap_alloc_inst(void)
{
	struct f_iap_opts *opts;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = iap_free_inst;

	return &opts->func_inst;
}

#else
static int __ref iap_bind_config(struct usb_configuration *c)
{
	struct f_iap	*ss;
	int			status,id;

	printk(KERN_ALERT "in %s   %s \n",__FILE__,__func__);

	/* allocate string ID(s) */
	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	strings_iap[0].id = id;

	ipod_out_intf.iInterface = id;

	ss = kzalloc(sizeof *ss, GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->func.name = "source/sink";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 13)
	ss->func.fs_descriptors = fs_ipod_out_descs;
#else
	ss->func.descriptors = fs_ipod_out_descs;
#endif
	ss->func.bind = iap_bind;
	ss->func.unbind = iap_unbind;
	ss->func.set_alt = iap_set_alt;
	ss->func.disable = iap_disable;
	ss->func.setup = iap_setup;

	status = usb_add_function(c, &ss->func);
	if (status)
		kfree(ss);
	return status;
}
#endif

static int iap_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev	*cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/* composite driver infrastructure handles everything except
	 * the two control test requests.
	 */
	printk(KERN_ALERT "%d       %d \n",ctrl->bRequestType,ctrl->bRequest);
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

		/*
		 * These are the same vendor-specific requests supported by
		 * Intel's USB 2.0 compliance test devices.  We exceed that
		 * device spec by allowing multiple-packet requests.
		 *
		 * NOTE:  the Control-OUT data stays in req->buf ... better
		 * would be copying it into a scratch buffer, so that other
		 * requests may safely intervene.
		 */

		case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8):

			printk(KERN_ALERT "IN brequest11\n");
			break;

		case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8):


			printk(KERN_ALERT "IN brequest22\n");
			break;



		case 0x5b:	/* control WRITE test -- fill the buffer */
			if (ctrl->bRequestType != (USB_DIR_OUT|USB_TYPE_VENDOR))
				goto unknown;
			if (w_value || w_index)
				break;
			/* just read that many bytes into the buffer */
			if (w_length > req->length)
				break;
			value = w_length;
			break;
		case 0x5c:	/* control READ test -- return the buffer */
			if (ctrl->bRequestType != (USB_DIR_IN|USB_TYPE_VENDOR))
				goto unknown;
			if (w_value || w_index)
				break;
			/* expect those bytes are still in the buffer; send back */
			if (w_length > req->length)
				break;
			value = w_length;
			break;

		default:
unknown:
			VDBG(cdev, "unknown control req%02x.%02x v%04x i%04x l%d\n",
				ctrl->bRequestType, ctrl->bRequest,
				w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		VDBG(cdev, "source/sink req%02x.%02x v%04x i%04x l%d\n",
				ctrl->bRequestType, ctrl->bRequest,
				w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		//if (value < 0)
		//ERROR(cdev, "source/sinkc response, err %d\n",
		//		value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}


int giap_setup(struct usb_gadget *g, int count)
{
	int status;
	dev_t dev;

	ipodg_class = class_create(THIS_MODULE, "ipodout");

	status = alloc_chrdev_region(&dev, 0, count, "ipodout");
	if (!status) {
		major = MAJOR(dev);
		minors = count;
	}

	return status;
}

void giap_cleanup(void)
{
	if (major) {
		unregister_chrdev_region(MKDEV(major, 0), minors);
		major = minors = 0;
	}

	class_destroy(ipodg_class);
	ipodg_class = NULL;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
DECLARE_USB_FUNCTION_INIT(iap, iap_alloc_inst, iap_alloc);
#endif
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Akshay Madbhavi");
