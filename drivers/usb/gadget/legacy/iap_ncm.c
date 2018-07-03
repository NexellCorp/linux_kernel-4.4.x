/*
 * iap_ncm.c -- Multifunction Composite driver for iap and ncm
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Samsung Electronics
 * Author: Michal Nazarewicz (m.nazarewicz@samsung.com)
 *
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


#include <linux/kernel.h>
#include <linux/module.h>

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif

#define DRIVER_DESC		"Multifunction Composite Gadget"

/***************************** All the files... *****************************/

/*
 * kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */

/* Add the Allgo iPod Out and CDC-NCM files for interfaces. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)

#include <linux/usb/composite.h>
#include "u_iap.h"
#include "u_ether.h"
#include "u_ncm.h"

#else

/* 
 * With Kernels of version <= 3.1.10, libcomposite can not be built
 * as a seperate module and be run-time linked to driver modules.
 * 
 * For lower versions, function drivers and composite drivers have to
 * be compile time linked. 
 * Note : This kernel version needs to be updated if a greater version 
 * is found be not supporting libcomposite.
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 1, 10)
#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#endif

/* 
 * Kernel versions >= 3.14.43 support usb function registration
 * with gadget framework using usb_function_driver structure. This 
 * facilitates building of individual functions as modules. 
 * Composite driver can use these functions using usb_get_function_instance 
 * and usb_get_function.
 *
 * For version < 3.14.43, function specific APIs have to compile time
 * linked with composite driver.
 */
#include "f_iap.c"
#include "f_ncm.c"
#include "u_ether.c"

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
/* Module parameters */
USB_GADGET_COMPOSITE_OPTIONS();
USB_ETHERNET_MODULE_PARAMETERS();
#else
static u8 hostaddr[ETH_ALEN];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
static struct eth_dev *dev_eth;
#endif
#endif

/* Modules parameter for IAP interfae subclass */
static unsigned subClass = 0xf0; /* Defaults to MFi Accessory subclass */
module_param(subClass, uint, 0);
MODULE_PARM_DESC(subClass, "Interface Subclass for IAP interafce (Ex:0xf0 for IAP MFi Subclass)");

#define MULTI_VENDOR_NUM	0x1d6b	/* Linux Foundation */
#define MULTI_PRODUCT_NUM	0x0104	/* Multifunction Composite Gadget */

/* Device Descriptor */
static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16(0x0200),

	.bDeviceClass =		USB_CLASS_MISC /* 0xEF */,
	.bDeviceSubClass =	2,
	.bDeviceProtocol =	1,

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(MULTI_VENDOR_NUM),
	.idProduct =		cpu_to_le16(MULTI_PRODUCT_NUM),
};


static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &(struct usb_otg_descriptor){
		.bLength =		sizeof(struct usb_otg_descriptor),
		.bDescriptorType =	USB_DT_OTG,

		/*
		 * REVISIT SRP-only hardware is possible, although
		 * it would not be called "OTG" ...
		 */
#ifdef ALLGO_DEFINE_HNP
		.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
#endif
	},
	NULL,
};

static struct usb_string strings_dev[] = {
	[0].s = "Multifunction with IAP",
	[1].s = "Multifunction with NCM",
	{  } /* end of list */
};

static struct usb_gadget_strings *dev_strings[] = {
	&(struct usb_gadget_strings){
		.language	= 0x0409,	/* en-us */
		.strings	= strings_dev,
	},
	NULL,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
/* NCM function instance and function */
static struct usb_function_instance *fi_ncm;
static struct usb_function *f_ncm;

/* IAP function instance and function */
static struct usb_function_instance *fi_iap;
static struct usb_function *f_iap;
#endif

/*
 * Overwrites Interface subclass value in IAP interface descriptor with 
 * what is passed as subClass module parameter.
 */ 
static void set_iap_interface_subclass(struct usb_configuration *c)
{
	struct usb_function     *f = c->interface[0]; /* Interface 0 is IAP interface */
	struct usb_descriptor_header **descriptors;

	if(gadget_is_dualspeed(c->cdev->gadget))
		descriptors = f->hs_descriptors;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	else
		descriptors = f->fs_descriptors;
#endif

	for (; *descriptors; ++descriptors)
	{
		struct usb_interface_descriptor *interface;

		if ((*descriptors)->bDescriptorType == USB_DT_INTERFACE)
		{
			interface = (struct usb_interface_descriptor *)*descriptors;
			interface->bInterfaceSubClass = (__u8)subClass;
                        printk(KERN_INFO "Set IAP interface subclass to 0x%x\n",(__u8)subClass); 
			break;
		}
	}
        
	return;
}

static __init int iap_ncm_do_config(struct usb_configuration *c)
{
	int ret;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
        c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	f_iap = usb_get_function(fi_iap);
	if (IS_ERR(f_iap))
		return PTR_ERR(f_iap);

	ret = usb_add_function(c, f_iap);
	if (ret < 0) {
		usb_remove_function(c, f_iap);
		goto exit;
	}

	f_ncm = usb_get_function(fi_ncm);
	if (IS_ERR(f_ncm))
		return PTR_ERR(f_ncm);

	ret = usb_add_function(c, f_ncm);
	if (ret < 0)
		usb_remove_function(c, f_ncm);
#else
    ret = iap_bind_config(c);
    if (ret < 0)
	goto exit;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
    ret = ncm_bind_config(c, hostaddr,dev_eth);
#else
    ret = ncm_bind_config(c, hostaddr);
#endif
    if (ret < 0)
	goto exit;
#endif
        /* Set IAP interface subclass to MFi Interface subclass */
        set_iap_interface_subclass(c);
exit:
	return ret;
}

static struct usb_configuration iap_ncm_config = {
	/* .label = f(hardware) */
	.label			= "Allgo Digital iPod Out",
	.strings	    = iap_strings,
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
	.MaxPower		= 2,
#else
	.bMaxPower              = 2,
#endif
};

static int iap_ncm_config_register(struct usb_composite_dev *cdev)
{

	return usb_add_config(cdev, &iap_ncm_config, iap_ncm_do_config);
}

static int __init iap_ncm_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int status;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	struct f_ncm_opts	*ncm_opts;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	fi_iap = usb_get_function_instance("iap");
	if (IS_ERR(fi_iap))
		return PTR_ERR(fi_iap);

	fi_ncm = usb_get_function_instance("ncm");
	if (IS_ERR(fi_ncm))
		return PTR_ERR(fi_ncm);

	ncm_opts = container_of(fi_ncm, struct f_ncm_opts, func_inst);

	gether_set_qmult(ncm_opts->net, qmult);
	if (!gether_set_host_addr(ncm_opts->net, host_addr))
		pr_info("using host ethernet address: %s", host_addr);
	if (!gether_set_dev_addr(ncm_opts->net, dev_addr))
		pr_info("using self ethernet address: %s", dev_addr);
#else
	/* Setup iPOD Out */
	status = giap_setup(cdev->gadget, 1);
	if (status < 0)
		return status;

	/* set up network link layer */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
	dev_eth = gether_setup(cdev->gadget, hostaddr);
	if (IS_ERR(dev_eth))
		return PTR_ERR(dev_eth);
#else
	status = gether_setup(cdev->gadget, hostaddr);
	if (status < 0)
		return status;
#endif
#endif

	/* allocate string IDs */
	status = usb_string_ids_tab(cdev, strings_dev);
	if (unlikely(status < 0))
		goto fail2;

	/* register configurations */
	status = iap_ncm_config_register(cdev);
	if (unlikely(status < 0))
		goto fail2;

	/* we're done */
	dev_info(&gadget->dev, DRIVER_DESC "\n");
	return 0;

	/* error recovery */
fail2:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	usb_put_function_instance(fi_iap);
	usb_put_function_instance(fi_ncm);
#else
	giap_cleanup();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
        gether_cleanup(dev_eth);
#else
	gether_cleanup();
#endif
#endif

	return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
static int __exit iap_ncm_unbind(struct usb_composite_dev *dev)
{
	if (!IS_ERR_OR_NULL(f_ncm))
		usb_put_function(f_ncm);
	if (!IS_ERR_OR_NULL(fi_ncm))
		usb_put_function_instance(fi_ncm);
	if (!IS_ERR_OR_NULL(f_iap))
		usb_put_function(f_iap);
	if (!IS_ERR_OR_NULL(fi_iap))
		usb_put_function_instance(fi_iap);

	return 0;
}
#else
static int __exit multi_unbind(struct usb_composite_dev *cdev)
{
	giap_cleanup();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 31)
        gether_cleanup(dev_eth);
#else
	gether_cleanup();
#endif
	return 0;
}
#endif

static __refdata struct usb_composite_driver iap_ncm_driver = {
	.name	     = "iap_ncm",
	.dev	     = &device_desc,
	.strings	        = dev_strings,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)
	.bind           = iap_ncm_bind,
	.unbind         = iap_ncm_unbind,
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 13)
	.bind           = iap_ncm_bind,
	.unbind		= __exit_p(multi_unbind),
#else
	.unbind		= __exit_p(multi_unbind),
	.iProduct	= DRIVER_DESC,
#endif

	.needs_serial	= 1,

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 35)
	.max_speed  = USB_SPEED_HIGH,
#endif
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
module_usb_composite_driver(iap_ncm_driver);
#else
static int __init iap_ncm_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 13)
        return usb_composite_probe(&iap_ncm_driver);
#else
	return usb_composite_probe(&iap_ncm_driver, iap_ncm_bind);
#endif
}
module_init(iap_ncm_init);

static void __exit iap_ncm_exit(void)
{
	usb_composite_unregister(&iap_ncm_driver);
}
module_exit(iap_ncm_exit);
#endif

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("GUESS???!");
MODULE_LICENSE("GPL");
