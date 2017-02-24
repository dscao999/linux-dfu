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

#define USB_DFU_DETACH		0
#define USB_DFU_GETSTATUS	3
#define USB_DFU_GETSTATE	5

#define USB_DFU_SUBCLASS	0x01
#define USB_DFU_PROTO_RUNTIME	0x01
#define USB_DFU_PROTO_DFUMODE	0x02

#define USB_DFU_FUNC_DSCLEN	0x09
#define USB_DFU_FUNC_DSCTYP	0x21

static int max_dfus = 8;
module_param(max_dfus, int, S_IRUGO);
static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, S_IRUGO | S_IWUSR);

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
static dev_t dfu_devnum = 0;
static struct class *dfu_class = NULL;

static atomic_t dfu_index = ATOMIC_INIT(-1);

struct dfu_device {
	struct mutex dfulock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct device *sysdev;
	struct device_attribute devattr;
	dev_t devnum;
	int index;
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

static int dfu_open(struct inode *inode, struct file *file)
{
	struct dfu_device *dfudev;

	dfudev = container_of(inode->i_cdev, struct dfu_device, dfu_cdev);
	file->private_data = dfudev;
	if (mutex_lock_interruptible(&dfudev->dfulock)) {
		dev_err(&dfudev->intf->dev, "Device busy\n");
		return -EBUSY;
	}
	return 0;
}

static int dfu_release(struct inode *inode, struct file *file)
{
	struct dfu_device *dfudev;

	dfudev = file->private_data;
	mutex_unlock(&dfudev->dfulock);
	return 0;
}

static struct file_operations dfu_fops = {
	.owner		= THIS_MODULE,
	.open		= dfu_open,
	.release	= dfu_release,
	.read		= NULL, /*dfu_upload,*/
	.write		= NULL, /*dfu_download,*/
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
		jiff_wait = msecs_to_jiffies(urb_timeout);
		if (!wait_for_completion_timeout(&ctrl->urbdone, jiff_wait)) {
			usb_unlink_urb(ctrl->urb);
			wait_for_completion(&ctrl->urbdone);
			dev_err(&dfudev->intf->dev, "Control URB cancelled\n");
		}
	} else
		dev_err(&dfudev->intf->dev, "Cannot submit URB: %d\n", retusb);
	usb_free_urb(ctrl->urb);
	if (ctrl->status)
		dev_err(&dfudev->intf->dev, "URB request failed: %d\n",
				ctrl->status);

	return ctrl->status;
}

static int dfu_get_status(struct dfu_device *dfudev, struct dfu_status *st)
{
	int retv;
	struct dfu_control ctrl;

	ctrl.req.bRequestType = 0xa1;
	ctrl.req.bRequest = USB_DFU_GETSTATUS;
	ctrl.req.wValue = 0;
	ctrl.req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl.req.wLength = cpu_to_le16(6);
	ctrl.pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl.buff = (u8 *)st;
	ctrl.len = 6;
	retv = dfu_submit_urb(dfudev, &ctrl);

	return retv;
}

static void do_switch(struct dfu_device *dfudev, struct dfu_control *ctrl)
{ 
	int tmout;

	tmout = dfudev->dettmout > 2000 ?
				2000 : dfudev->dettmout;
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DETACH;
	ctrl->req.wValue = cpu_to_le16(tmout);
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	dfu_submit_urb(dfudev, ctrl);
}

static int dfu_get_state(struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	u8 bState;
	int retv;

	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = USB_DFU_GETSTATE;
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(1);
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &bState;
	ctrl->len = 1;
	retv = dfu_submit_urb(dfudev, ctrl);
	if (retv == 0)
		return bState;
	else
		return retv;
}

static ssize_t dfu_switch(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct dfu_control ctrl;
	int state;

	dfudev = container_of(attr, struct dfu_device, devattr);

	if (count > 0 && *buf == '-' && (*(buf+1) == '\n' || *(buf+1) == 0)) {
		ctrl.req.wIndex = cpu_to_le16(dfudev->intfnum);
		ctrl.buff = NULL;
		ctrl.len = 0;
		if (dfudev->runtime)
			do_switch(dfudev, &ctrl);
		else {
			state = dfu_get_state(dfudev, &ctrl);
			if (state >= 0)
				dev_info(&dfudev->intf->dev, "DFU State: %d\n",
						state);
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

	retv = 0;
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
	dfudev->index = atomic_inc_return(&dfu_index);
	if (dfudev->index >= max_dfus) {
		dev_err(&intf->dev, "Maximum supported USB DFU reached: %d\n",
			max_dfus);
		goto err_10;
	}
	dfudev->attr = dfufdsc->attr;
	dfudev->dettmout = le16_to_cpu(dfufdsc->tmout);
	dfudev->xfersize = le16_to_cpu(dfufdsc->xfersize);
	dfudev->intf = intf;
	dfudev->usbdev = interface_to_usbdev(intf);
	dfudev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	dfudev->devnum = MKDEV(MAJOR(dfu_devnum), dfudev->index);
	mutex_init(&dfudev->dfulock);

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
	if (retv != 0) {
		dev_err(&intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_10;
	}
	usb_set_intfdata(intf, dfudev);
	if (dfudev->runtime)
		return retv;

	cdev_init(&dfudev->dfu_cdev, &dfu_fops);
	dfudev->dfu_cdev.owner = THIS_MODULE;
	retv = cdev_add(&dfudev->dfu_cdev, dfudev->devnum, 1);
	if (retv) {
		dev_err(&dfudev->intf->dev, "Cannot register device: %d\n",
				retv);
		goto err_20;
	}
	dfudev->sysdev = device_create(dfu_class, NULL, dfudev->devnum, dfudev, 
			"dfu%d", dfudev->index);
	if (IS_ERR(dfudev->sysdev)) {
		retv = (int)PTR_ERR(dfudev->sysdev);
		dev_err(&dfudev->intf->dev, "Cannot create device file: %d\n",
				retv);
		goto err_30;
	}

	return retv;

err_30:
	cdev_del(&dfudev->dfu_cdev);
err_20:
	device_remove_file(&intf->dev, &dfudev->devattr);
	usb_set_intfdata(intf, NULL);
err_10:
	atomic_dec(&dfu_index);
	kfree(dfudev);
	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dfudev;

	dfudev = usb_get_intfdata(intf);
	if (!dfudev->runtime) {
		device_destroy(dfu_class, dfudev->devnum);
		dfudev->sysdev = NULL;
		cdev_del(&dfudev->dfu_cdev);
	}
	device_remove_file(&intf->dev, &dfudev->devattr);
	usb_set_intfdata(intf, NULL);
	atomic_dec(&dfu_index);
	kfree(dfudev);
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

	retv = alloc_chrdev_region(&dfu_devnum, 0, max_dfus, DFUDEV_NAME);
	if (retv != 0) {
		pr_err("Cannot allocate a char major number: %d\n", retv);
		goto err_00;
	}
	dfu_class = class_create(THIS_MODULE, DFUDEV_NAME);
	if (IS_ERR(dfu_class)) {
		retv = (int)PTR_ERR(dfu_class);
		pr_err("Cannot create DFU class, Out of Memory!\n");
		goto err_10;
	}
	retv = usb_register(&dfu_driver);
	if (retv) {
		pr_err("Cannot register USB DFU driver: %d\n", retv);
		goto err_20;
	}

	return retv;

err_20:
	class_destroy(dfu_class);
err_10:
	unregister_chrdev_region(dfu_devnum, max_dfus);
err_00:
	return retv;
}

static void __exit usbdfu_exit(void)
{
	usb_deregister(&dfu_driver);
	class_destroy(dfu_class);
	unregister_chrdev_region(dfu_devnum, max_dfus);
}

module_init(usbdfu_init);
module_exit(usbdfu_exit);
