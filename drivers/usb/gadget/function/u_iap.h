/*
 * u_iap.h
 *
 * Utility definitions for the ncm function
 *
 * Copyright (c) 2015 AllGo Embedded Systems
 *
 * Author: Akshay Madbhavi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef U_IAP_H
#define U_IAP_H

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 43)

struct f_iap_opts {
	struct usb_function_instance	func_inst;
	bool				bound;

	/*
	 * Read/write access to configfs attributes is handled by configfs.
	 *
	 * This is to protect the data from concurrent access by read/write
	 * and create symlink/remove symlink.
	 */
	struct mutex			lock;
	int				refcnt;
};
#endif

/* function-specific strings: */
static struct usb_string strings_iap[] = {
	[0].s = "iAP Interface",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_iap = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_iap,
};

static struct usb_gadget_strings *iap_strings[] = {
	&stringtab_iap,
	NULL,
};

#endif /* U_IAP_H */
