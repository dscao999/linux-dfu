#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/cdev.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Class Driver");

#define MAX_DFUS	8

#define USB_DFU_DETACH		0
#define USB_DFU_GETSTATUS	3
#define USB_DFU_GETSTATE	5

#define USB_DFU_SUBCLASS	0x01
#define USB_DFU_PROTO_RUNTIME	0x01
#define USB_DFU_PROTO_DFUMODE	0x02

#define USB_DFU_FUNC_DSCLEN	0x09
#define USB_DFU_FUNC_DSCTYP	0x21

#define USB_URB_TIMEOUT	500   /* milliseconds */

static const struct usb_device_id dfu_ids[] = {
	{ .match_flags = USB_DEVICE_ID_MATCH_INT_INFO |
			USB_DEVICE_ID_MATCH_VENDOR,
	  .idVendor = 0x1cbe,
	  .bInterfaceClass = USB_CLASS_APP_SPEC,
	  .bInterfaceSubClass = USB_DFU_SUBCLASS,
	  .bInterfaceProtocol = USB_DFU_PROTO_DFUMODE },
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC, USB_DFU_SUBCLASS,
		USB_DFU_PROTO_RUNTIME) },
	{ }
};
MODULE_DEVICE_TABLE(usb, dfu_ids);

typedef unsigned char u8;

struct dfufdsc {
	u8 len;
	u8 dsctyp;
	u8 attr;
	u16 tmout;
	u16 xfersize;
	u16 ver;
} __packed;

struct dfu_status {
	u8 bStatus;
	u8 ptmout[3];
	u8 bState;
	u8 istr;
} __packed;

static const char *DFUDEV_NAME = "dfu";
static dev_t dfu_devnum;
static struct class *dfu_class;

struct dfu_device {
	struct mutex dfulock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct device_attribute devattr;
	int ctrl_pipe;
	int ctrl_status;
	int attr;
	int dettmout;
	int xfersize;
	int runtime;
	int intfnum;

	struct cdev dfu_cdev;
};

struct dfu_control {
	struct urb *urb;
	struct completion urbdone;
	volatile int status;
	struct usb_ctrlrequest req;
	int pipe;
	u8 *buff;
	int len;
};

static void dfu_ctrlurb_done(struct urb *urb)
{
	struct dfu_control *ctrl;

	ctrl = urb->context;
	ctrl->status = urb->status;
	complete(&ctrl->urbdone);
}

static int dfu_submit_urb(struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	unsigned long jiff_wait;
	int retusb;

	ctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ctrl->urb) {
		dev_err(&dfudev->intf->dev, "Cannot allocate USB URB\n");
		return -ENOMEM;
	}
	usb_fill_control_urb(ctrl->urb, dfudev->usbdev, ctrl->pipe,
			(u8 *)&ctrl->req, ctrl->buff, ctrl->len,
			dfu_ctrlurb_done, ctrl);
	init_completion(&ctrl->urbdone);
	retusb = usb_submit_urb(ctrl->urb, GFP_KERNEL);
	if (retusb == 0) {
		jiff_wait = msecs_to_jiffies(USB_URB_TIMEOUT);
		if (!wait_for_completion_timeout(&ctrl->urbdone, jiff_wait)) {
			usb_unlink_urb(ctrl->urb);
			wait_for_completion(&ctrl->urbdone);
			dev_err(&dfudev->intf->dev, "Control URB cancelled\n");
		}
	} else
		dev_err(&dfudev->intf->dev, "Cannot submit URB: %d\n", retusb);
	usb_free_urb(ctrl->urb);

	return ctrl->status;
}

static int dfu_status(struct dfu_device *dfudev)
{
	int retv;
	struct dfu_control ctrl;

	ctrl.req.bRequestType = 0xa1;
	ctrl.req.bRequest = USB_DFU_GETSTATUS;
	ctrl.req.wValue = 0;
	ctrl.req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl.req.wLength = cpu_to_le16(6);
	ctrl.pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);

	return retv;
}

static ssize_t dfu_switch(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	int tmout;
	struct dfu_control ctrl;

	dfudev = container_of(attr, struct dfu_device, devattr);

	if (count > 0 && *buf == '-' && (*(buf+1) == '\n' || *(buf+1) == 0)) {
		ctrl.req.wIndex = cpu_to_le16(dfudev->intfnum);
		ctrl.buff = NULL;
		ctrl.len = 0;
		if (dfudev->runtime) {
			tmout = dfudev->dettmout > 2000 ?
					2000 : dfudev->dettmout;
			ctrl.req.bRequestType = 0x21;
			ctrl.req.bRequest = USB_DFU_DETACH;
			ctrl.req.wValue = cpu_to_le16(tmout);
			ctrl.req.wLength = 0;
			ctrl.pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
			dfu_submit_urb(dfudev, &ctrl);
		} else {
			u8 bState;

			ctrl.req.bRequestType = 0xa1;
			ctrl.req.bRequest = USB_DFU_GETSTATE;
			ctrl.req.wValue = 0;
			ctrl.req.wLength = cpu_to_le16(1);
			ctrl.pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
			ctrl.buff = &bState;
			ctrl.len = 1;
			if (dfu_submit_urb(dfudev, &ctrl) == 0)
				dev_info(&dfudev->intf->dev, "DFU State: %d\n",
						(int)bState);
		}
	} else
		dev_err(dev, "Invalid Command: %c\n", *buf);

	return count;
}

static ssize_t dfu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;
	int retv;
	const char *fmt = "Attribute: %#02.2x Timeout: %d Transfer Size: %d\n";

	retv = 0;
	dfudev = container_of(attr, struct dfu_device, devattr);
	retv = snprintf(buf, 128, fmt, dfudev->attr, dfudev->dettmout,
			dfudev->xfersize);
	return retv;
}

static int dfu_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv, dfufdsc_len;
	struct usb_interface_descriptor *intfdsc;
	struct dfu_device *dfudev;
	struct dfufdsc *dfufdsc;

	dfufdsc = (struct dfufdsc *)intf->cur_altsetting->extra;
	dfufdsc_len = intf->cur_altsetting->extralen;
	if (!dfufdsc || dfufdsc_len != USB_DFU_FUNC_DSCLEN ||
			dfufdsc->dsctyp != USB_DFU_FUNC_DSCTYP) {
		dev_err(&intf->dev, "Invalid DFU functional descriptor\n");
		return -ENODEV;
	}

	dfudev = kmalloc(sizeof(struct dfu_device), GFP_KERNEL);
	if (!dfudev)
		return -ENOMEM;

	dfudev->attr = dfufdsc->attr;
	dfudev->dettmout = le16_to_cpu(dfufdsc->tmout);
	dfudev->xfersize = le16_to_cpu(dfufdsc->xfersize);
	dfudev->intf = intf;
	dfudev->usbdev = interface_to_usbdev(intf);
	dfudev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	mutex_init(&dfudev->dfulock);

	retv = 0;
	intfdsc = &intf->cur_altsetting->desc;
	if (intfdsc->bInterfaceProtocol == USB_DFU_PROTO_RUNTIME) {
		dfudev->devattr.attr.name = "detach";
		dfudev->runtime = 1;
	} else {
		dfudev->devattr.attr.name = "attach";
		dfudev->runtime = 0;
	}
	dfudev->devattr.attr.mode = S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->devattr.show = dfu_show;
	dfudev->devattr.store = dfu_switch;
	retv = device_create_file(&intf->dev, &dfudev->devattr);
	if (retv == 0)
		usb_set_intfdata(intf, dfudev);
	else {
		dev_err(&intf->dev, "Cannot create sysfs file \"detach\"\n");
		goto err_10;
	}

	if (retv < 0)
		goto err_10;

	return retv;

err_10:
	kfree(dfudev);
	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dev;

	dev = usb_get_intfdata(intf);
	device_remove_file(&intf->dev, &dev->devattr);
	usb_set_intfdata(intf, NULL);
	kfree(dev);
}

static struct usb_driver dfu_driver = {
	.name = "usbdfu",
	.probe = dfu_probe,
	.disconnect = dfu_disconnect,
	.id_table = dfu_ids,
};

static int __init usbdfu_init(void)
{
	int retv;

	retv = usb_register(&dfu_driver);
	if (retv) {
		pr_err("Cannot register USB DFU driver: %d\n", retv);
		return retv;
	}
	retv = alloc_chrdev_region(&dfu_devnum, 0, MAX_DFUS, DFUDEV_NAME);
	if (retv != 0) {
		pr_err("Cannot allocate a char major number: %d\n", retv);
		goto err_10;
	}
	dfu_class = class_create(THIS_MODULE, DFUDEV_NAME);
	if (IS_ERR(dfu_class)) {
		retv = -ENOMEM;
		pr_err("Cannot create DFU class, Out of Memory!\n");
		goto err_20;
	}


	return retv;

err_20:
	unregister_chrdev_region(dfu_devnum, MAX_DFUS);
err_10:
	usb_deregister(&dfu_driver);
	return retv;
}

static void __exit usbdfu_exit(void)
{
	class_destroy(dfu_class);
	unregister_chrdev_region(dfu_devnum, MAX_DFUS);
	usb_deregister(&dfu_driver);
}

module_init(usbdfu_init);
module_exit(usbdfu_exit);
