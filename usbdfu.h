#ifndef LINUX_USB_DFU_DSCAO__
#define LINUX_USB_DFU_DSCAO__
/*
 * usbdfu.h
 *
 * Copyright (c) 2017 Dashi Cao        <dscao999@hotmail.com, caods1@lenovo.com>
 *
 * USB Abstract Control Model driver for USB Device Firmware Upgrade
 *
*/
#include <linux/cdev.h>
#include <linux/usb.h>

#define USB_DFU_DETACH		0
#define USB_DFU_DNLOAD		1
#define USB_DFU_UPLOAD		2
#define USB_DFU_GETSTATUS	3
#define USB_DFU_CLRSTATUS	4
#define USB_DFU_GETSTATE	5
#define USB_DFU_ABORT		6

#define USB_DFU_SUBCLASS	0x01
#define USB_DFU_PROTO_RUNTIME	0x01
#define USB_DFU_PROTO_DFUMODE	0x02

#define USB_DFU_FUNC_DSCLEN	0x09
#define USB_DFU_FUNC_DSCTYP	0x21
#define USB_DFU_ERROR_CODE	65535

#define USB_DFU_INTERFACE_INFO(v, cl, sc, pr) \
        .match_flags = USB_DEVICE_ID_MATCH_VENDOR | \
			USB_DEVICE_ID_MATCH_INT_INFO, \
	.idVendor = (v), \
        .bInterfaceClass = (cl), \
        .bInterfaceSubClass = (sc), \
        .bInterfaceProtocol = (pr)

#define DFUDEV_NAME	"dfu"

struct dfufdsc {
	__u8 len;
	__u8 dsctyp;
	__u8 attr;
	__le16 tmout;
	__le16 xfersize;
	__le16 ver;
} __packed;

struct dfu_status {
	__u8 bStatus;
	__u8 wmsec[3];
	__u8 bState;
	__u8 istr;
} __packed;

enum dfu_state {
	appIDLE = 0,
	appDETACH = 1,
	dfuIDLE = 2,
	dfuDNLOAD_SYNC = 3,
	dfuDNLOAD_BUSY = 4,
	dfuDNLOAD_IDLE = 5,
	dfuMANIFEST_SYNC = 6,
	dfuMANIFEST = 7,
	dfuMANIFEST_WAIT_RESET = 8,
	dfuUPLOAD_IDLE = 9,
	dfuERROR = 10
};


struct dfu_control {
	struct urb *urb;
	struct completion urbdone;
	int status;
	struct usb_ctrlrequest req;
	int pipe;
	void *buff;
	int len;
	int nxfer;
	union {
		unsigned long ocupy[4];
		struct dfu_status dfuStatus;
		__u8 dfuState;
	};
};

struct dfu_feature {
	int reset:1;
};

struct dfu_device {
	struct mutex dfulock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct device *sysdev;
	struct device_attribute tachattr;
	struct device_attribute attrattr;
	struct device_attribute tmoutattr;
	struct device_attribute xsizeattr;
	struct device_attribute statattr;
	dev_t devnum;
	void *databuf;
	struct dfu_control *opctrl, *stctrl;
	int dma;
	int index;
	int attr;
	int dettmout;
	int xfersize;
	int proto;
	int intfnum;
	struct cdev dfu_cdev;
};

extern struct class *dfu_class;

int dfu_submit_urb(const struct dfu_device *dfudev, struct dfu_control *ctrl);
int dfu_prepare(struct dfu_device **dfudevp, struct usb_interface *intf,
                        const struct usb_device_id *d);
void dfu_cleanup(struct dfu_device *dfudev);

#endif /* LINUX_USB_DFU_DSCAO__ */
