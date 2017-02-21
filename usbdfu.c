#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Class Driver");

#define USB_DFU_SUBCLASS	0x01
#define USB_DFU_PROTO_RUNTIME	0x01
#define USB_DFU_PROTO_DFUMODE	0x02
#define USB_DFU_FUNC_DSCLEN	0x09
#define USB_DFU_FUNC_DSCTYP	0x21

typedef unsigned char u8;

struct __attribute__((packed)) dfufdsc {
	u8 len;
	u8 dsctyp;
	u8 attr;
	u16 tmout;
	u16 xfersize;
	u16 ver;
};

struct dfu_device {
	struct usb_device *usbdev;
	struct usb_interface *intf;
	int attr;
	int dettmout;
	int xfersize;
	int runtime;
	struct device_attribute devattr;
};

static ssize_t dfu_detach(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	return 0;
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
	dfudev->runtime = 0;
	dfudev->devattr.attr.name = "detach";
	dfudev->devattr.attr.mode = S_IWUSR;
	dfudev->devattr.show = NULL;
	dfudev->devattr.store = dfu_detach;

	retv = 0;
	intfdsc = &intf->cur_altsetting->desc;
	if (intfdsc->bInterfaceProtocol == USB_DFU_PROTO_RUNTIME) {
		dfudev->runtime = 1;
	}
	usb_set_intfdata(intf, dfudev);
	printk(KERN_INFO "Attributes: %02X, Timeout: %d, Buffer size: %d\n",
		dfudev->attr, dfudev->dettmout, dfudev->xfersize);

	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	void *dev;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	kfree(dev);
}

static const struct usb_device_id dfu_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC, USB_DFU_SUBCLASS,
		USB_DFU_PROTO_RUNTIME) },
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC, USB_DFU_SUBCLASS,
		USB_DFU_PROTO_DFUMODE) },
	{}
};
MODULE_DEVICE_TABLE(usb, dfu_ids);

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
		printk(KERN_ERR "Cannot register USB DFU driver\n");
	return retv;
}

static void __exit usbdfu_exit(void)
{
	usb_deregister(&dfu_driver);
}

module_init(usbdfu_init);
module_exit(usbdfu_exit);
