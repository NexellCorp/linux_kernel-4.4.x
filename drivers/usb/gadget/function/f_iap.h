/*
 * This header declares the utility functions used by "Gadget Ipodout", plus
 * interfaces to its  single-configuration function drivers.
 */

#ifndef __G_IPOD_H
#define __G_IPOD_H

#include <linux/usb/composite.h>
#include <linux/types.h>


#define USB_DIO_UNION_TYPE		0x06	/* union_desc */

#define STRING_IAP_IDX	0
#define STRING_CTRL_IDX	0
#define STRING_MAC_IDX	1
#define STRING_DATA_IDX	2
#define STRING_IAD_IDX	3



/* global state */
extern unsigned buflen;
//extern const struct usb_descriptor_header *otg_desc[];

/* configuration-specific linkup */
int ipodout_add(struct usb_composite_dev *cdev, bool autoresume);

/* "Union Functional Descriptor" from DIO  */
struct usb_dio_union_desc {
	__u8	bLength;
	__u8	bDescriptorType;
	__u8	bDescriptorSubType;

	__u8	bMasterInterface0;
	__u8	bSlaveInterface0;
	/* ... and there could be other slave interfaces */
} __attribute__ ((packed));

#endif /* __G_IPOD_H */
