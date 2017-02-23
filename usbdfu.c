#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Class Driver");

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

struct dfu_device {
	struct mutex devlock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct device_attribute devattr;
	struct usb_ctrlrequest ctrl_req;
	struct urb *ctrl_urb;
	int ctrl_pipe;
	int ctrl_status;
	struct completion ctrl_done;
	int attr;
	int dettmout;
	int xfersize;
	int runtime;
	int intfnum;
};

static void dfu_ctrlurb_done(struct urb *urb)
{
	struct dfu_device *dfudev;

	dfudev = urb->context;
	dfudev->ctrl_status = urb->status;
	complete(&dfudev->ctrl_done);
}

static int dfu_issue_control(struct dfu_device *dfudev, u8 *buff, int len)
{
	unsigned long jiff_wait;
	int retusb;

	usb_fill_control_urb(dfudev->ctrl_urb, dfudev->usbdev,
			dfudev->ctrl_pipe, (u8 *)&dfudev->ctrl_req, buff, len,
				dfu_ctrlurb_done, dfudev);
	dfudev->ctrl_status = -65535;
	init_completion(&dfudev->ctrl_done);
	retusb = usb_submit_urb(dfudev->ctrl_urb, GFP_KERNEL);
	if (retusb == 0) {
		jiff_wait = msecs_to_jiffies(USB_URB_TIMEOUT);
		if (!wait_for_completion_timeout(&dfudev->ctrl_done,
				jiff_wait)) {
			usb_unlink_urb(dfudev->ctrl_urb);
			wait_for_completion(&dfudev->ctrl_done);
			dev_err(&dfudev->intf->dev, "Control URB cancelled\n");
		}
	} else
		dev_err(&dfudev->intf->dev, "Cannot submit URB: %d\n", retusb);

	return dfudev->ctrl_status;
}

static int submit_detach(struct dfu_device *dfudev)
{
	int tmout, retusb;

	tmout = dfudev->dettmout > 2000 ? 2000 : dfudev->dettmout;
	dfudev->ctrl_req.bRequestType = 0x21;
	dfudev->ctrl_req.bRequest = USB_DFU_DETACH;
	dfudev->ctrl_req.wValue = cpu_to_le16(tmout);
	dfudev->ctrl_req.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->ctrl_req.wLength = 0;
	dfudev->ctrl_pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	retusb = dfu_issue_control(dfudev, NULL, 0);
	return retusb;
}

static int dfu_state(struct dfu_device *dfudev)
{
	u8 state;
	int retv;

	dfudev->ctrl_req.bRequestType = 0xa1;
	dfudev->ctrl_req.bRequest = USB_DFU_GETSTATE;
	dfudev->ctrl_req.wValue = 0;
	dfudev->ctrl_req.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->ctrl_req.wLength = cpu_to_le16(1);
	dfudev->ctrl_pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);

	retv = dfu_issue_control(dfudev, &state, sizeof(state));
	if (retv == 0)
		retv = state;
	else
		dev_err(&dfudev->intf->dev, "Cannot get DFU State\n");
	return retv;
}

static int dfu_status(struct dfu_device *dfudev)
{
	int retv;
	struct dfu_status dfust;

	dfudev->ctrl_req.bRequestType = 0xa1;
	dfudev->ctrl_req.bRequest = USB_DFU_GETSTATUS;
	dfudev->ctrl_req.wValue = 0;
	dfudev->ctrl_req.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->ctrl_req.wLength = cpu_to_le16(6);
	dfudev->ctrl_pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);

	retv = dfu_issue_control(dfudev, (u8 *)&dfust, sizeof(dfust));
	if (retv == 0)
		retv = dfust.bStatus;
	else
		dev_err(&dfudev->intf->dev, "Cannot get DFU State\n");
	return retv;
}

static ssize_t dfu_switch(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	int retv, dfustat;

	retv = 0;
	dfudev = container_of(attr, struct dfu_device, devattr);
	dfudev->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->ctrl_urb) {
		dev_err(&dfudev->intf->dev,
				"Cannot allocate USB URB\n");
		return count;
	}

	if (count > 0 && *buf == '-' && (*(buf+1) == '\n' || *(buf+1) == 0)) {
		if (mutex_trylock(&dfudev->devlock)) {
			if (dfudev->runtime)
				retv = submit_detach(dfudev);
			else {
				dfustat = dfu_state(dfudev);
				dev_info(&dfudev->intf->dev, "DFU State: %d\n",
						dfustat);
			}
			mutex_unlock(&dfudev->devlock);
		} else
			dev_err(dev, "Device busy\n");
	} else
		dev_err(dev, "Invalid Command: %c\n", *buf);

	usb_free_urb(dfudev->ctrl_urb);
	dfudev->ctrl_urb = NULL;
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

	dfudev->ctrl_urb = NULL;
	dfudev->attr = dfufdsc->attr;
	dfudev->dettmout = le16_to_cpu(dfufdsc->tmout);
	dfudev->xfersize = le16_to_cpu(dfufdsc->xfersize);
	dfudev->intf = intf;
	dfudev->usbdev = interface_to_usbdev(intf);
	dfudev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	mutex_init(&dfudev->devlock);

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
		kfree(dfudev);
	}

	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dev;

	dev = usb_get_intfdata(intf);
	device_remove_file(&intf->dev, &dev->devattr);
	usb_set_intfdata(intf, NULL);
	while (mutex_lock_interruptible(&dev->devlock) != 0)
		dev_err(&intf->dev, "Unable to get DFU lock\n");
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
	if (retv)
		pr_err("Cannot register USB DFU driver: %d\n", retv);
	return retv;
}

static void __exit usbdfu_exit(void)
{
	usb_deregister(&dfu_driver);
}

module_init(usbdfu_init);
module_exit(usbdfu_exit);
