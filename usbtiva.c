/*
 * usbdfu.c
 *
 * Copyright (c) 2017 Dashi Cao        <dscao999@hotmail.com, caods1@lenovo.com>
 *
 * USB Abstract Control Model driver for USB Device Firmware Upgrade
 *
*/
#include <stdarg.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Class Driver");

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

#define DFUDEV_NAME	"dfu"

#define USB_VENDOR_LUMINARY 0x1cbe
#define USB_PRODUCT_STELLARIS_DFU 0x0ff

static int max_dfus = 8;
module_param(max_dfus, int, S_IRUGO);
static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, S_IRUGO | S_IWUSR);
static int detach_timeout = 2000; /* 2 seconds */
module_param(detach_timeout, int, S_IRUGO | S_IWUSR);

static const struct usb_device_id dfu_ids[] = {
	{ .match_flags = USB_DEVICE_ID_MATCH_INT_INFO |
			USB_DEVICE_ID_MATCH_VENDOR,
	  .idVendor = USB_VENDOR_LUMINARY,
	  .bInterfaceClass = USB_CLASS_APP_SPEC,
	  .bInterfaceSubClass = USB_DFU_SUBCLASS,
	  .bInterfaceProtocol = USB_DFU_PROTO_DFUMODE },
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC, USB_DFU_SUBCLASS,
		USB_DFU_PROTO_RUNTIME) },
	{ }
};
MODULE_DEVICE_TABLE(usb, dfu_ids);

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

static dev_t dfu_devnum;
static struct class *dfu_class;

static atomic_t dfu_index = ATOMIC_INIT(-1);

struct dfu_cmd {
	__u8 cmdata[8];
} __packed;

struct dfu_info {
	__le16 blksize;
	__le16 numblks;
	__le32 partinfo1;
	__le32 partinfo0;
	__le32 addrHigh;
	__le32 addrLow;
} __packed;

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
		unsigned long ocupy;
		struct dfu_status dfuStatus;
		struct dfu_cmd cmd;
		struct dfu_info info;
		__u8 dfuState;
		struct {
			__le16 usMarker;
			__le16 usVersion;
		} tiva;
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
	struct device_attribute actattr;
	struct device_attribute dfuattr;
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

	union {
		int feat;
		struct dfu_feature ftus;
	};
};

static void dfu_ctrlurb_done(struct urb *urb)
{
	struct dfu_control *ctrl;

	ctrl = urb->context;
	ctrl->status = urb->status;
	ctrl->nxfer = urb->actual_length;
	complete(&ctrl->urbdone);
}

static int dfu_get_status(const struct dfu_device *dfudev,
		struct dfu_control *ctrl);

static void dfu_urb_timeout(const struct dfu_device *dfudev,
		struct dfu_control *ctrl)
{
	int retv;

	retv = USB_DFU_ERROR_CODE;
	usb_unlink_urb(ctrl->urb);
	wait_for_completion(&ctrl->urbdone);
	if (ctrl->req.bRequest != USB_DFU_ABORT)
		dev_err(&dfudev->intf->dev,
			"URB req type: %2.2x, req: %2.2x cancelled\n",
			(int)ctrl->req.bRequestType,
			(int)ctrl->req.bRequest);
}


static int dfu_submit_urb(const struct dfu_device *dfudev,
		struct dfu_control *ctrl)
{
	int retusb, retv, alloc;
	unsigned long jiff_wait;

	alloc = 0;
	if (ctrl->urb == NULL) {
		ctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!ctrl->urb) {
			dev_err(&dfudev->intf->dev, "Cannot allocate URB\n");
			return -ENOMEM;
		}
		alloc = 1;
	}
	usb_fill_control_urb(ctrl->urb, dfudev->usbdev, ctrl->pipe,
			(__u8 *)&ctrl->req, ctrl->buff, ctrl->len,
			dfu_ctrlurb_done, ctrl);
	init_completion(&ctrl->urbdone);
	ctrl->status = 65535;
	retusb = usb_submit_urb(ctrl->urb, GFP_KERNEL);
	if (retusb == 0) {
		jiff_wait = msecs_to_jiffies(urb_timeout);
		if (!wait_for_completion_timeout(&ctrl->urbdone, jiff_wait))
			dfu_urb_timeout(dfudev, ctrl);
	} else
		dev_err(&dfudev->intf->dev,
			"URB type: %2.2x, req: %2.2x submit failed: %d\n",
			(int)ctrl->req.bRequestType,
			(int)ctrl->req.bRequest, retusb);
	if (alloc) {
		usb_free_urb(ctrl->urb);
		ctrl->urb = NULL;
	}
	retv = ACCESS_ONCE(ctrl->status);
	if (retv && ctrl->req.bRequest != USB_DFU_ABORT)
		dev_err(&dfudev->intf->dev,
			"URB type: %2.2x, req: %2.2x request failed: %d\n",
			(int)ctrl->req.bRequestType, (int)ctrl->req.bRequest,
			ctrl->status);

	return ctrl->status;
}

static int dfu_clr_status(const struct dfu_device *dfudev, struct dfu_control *ctrl);
static int dfu_get_state(const struct dfu_device *dfudev, struct dfu_control *ctrl);

static int dfu_upload_area(const struct dfu_device *dfudev, struct dfu_control *ctrl,
	int start, int size)
{
	int blknum, retv;

	blknum = start / 1024;
	ctrl->cmd.cmdata[0] = 0x02; /* DFU_CMD_READ */
	ctrl->cmd.cmdata[1] = 0;
	ctrl->cmd.cmdata[2] = blknum & 0x0ff;
	ctrl->cmd.cmdata[3] = (blknum >> 8) & 0x0ff;
	ctrl->cmd.cmdata[4] = 0x0ff & size;
	ctrl->cmd.cmdata[5] = 0x0ff & (size >> 8);
	ctrl->cmd.cmdata[6] = 0x0ff & (size >> 16);
	ctrl->cmd.cmdata[7] = size >> 24;
	ctrl->buff = &ctrl->cmd;
	ctrl->len = sizeof(ctrl->cmd);
	
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wValue = 0;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wLength = cpu_to_le16(ctrl->len);
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->urb = NULL;
	retv = dfu_submit_urb(dfudev, ctrl);
	return retv;
}

static int dfu_open(struct inode *inode, struct file *file)
{
	struct dfu_device *dfudev;
	int state, retv;
	struct dfu_control *ctrl;

	retv = 0;
	dfudev = container_of(inode->i_cdev, struct dfu_device, dfu_cdev);
	file->private_data = dfudev;
	if (mutex_lock_interruptible(&dfudev->dfulock))
		return -EBUSY;

	dfudev->databuf = kmalloc(dfudev->xfersize+2*sizeof(struct dfu_control),
					GFP_KERNEL);
	if (!dfudev->databuf) {
		retv = -ENOMEM;
		goto err_10;
	}
	dfudev->opctrl = dfudev->databuf + dfudev->xfersize;
	dfudev->stctrl = dfudev->opctrl + 1;
	ctrl = dfudev->stctrl;
	state = dfu_get_state(dfudev, ctrl);
	if (state != dfuIDLE) {
		dev_err(&dfudev->intf->dev, "Bad Initial State: %d\n", state);
		retv =  -ETXTBSY;
		goto err_20;
	}

	return retv;

err_20:
	kfree(dfudev->databuf);
err_10:
	mutex_unlock(&dfudev->dfulock);
	return retv;
}

static int dfu_abort(struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_ABORT;
	ctrl->req.wValue = 0;
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	ctrl->urb = NULL;
	return dfu_submit_urb(dfudev, ctrl);
}

static int dfu_release(struct inode *inode, struct file *filp)
{
	struct dfu_device *dfudev;
	struct dfu_control *stctrl;
	int retv;

	dfudev = filp->private_data;
	stctrl = dfudev->stctrl;
	retv = dfu_get_state(dfudev, stctrl);
	if (retv == dfuERROR)
		dfu_clr_status(dfudev, stctrl);
	else if (retv != dfuIDLE)
		dfu_abort(dfudev, stctrl);
	mdelay(100);
	retv = dfu_get_state(dfudev, stctrl);
	if (retv != dfuIDLE)
		dev_err(&dfudev->intf->dev, "Need Reset! Stuck in State: %d\n",
				retv);
	kfree(dfudev->databuf);
	mutex_unlock(&dfudev->dfulock);
	return 0;
}

ssize_t dfu_upload(struct file *filp, char __user *buff, size_t count,
			loff_t *f_pos)
{
	struct dfu_device *dfudev;
	int blknum, retv, numb, nbytes;
	struct dfu_control *opctrl, *stctrl;
	int dfust, dma;
	dma_addr_t dmabuf;
	struct device *ctrler;

	dfudev = filp->private_data;
	if (count % dfudev->xfersize != 0)
		return -EINVAL;
	opctrl = dfudev->opctrl;
	stctrl = dfudev->stctrl;
	dfust = dfu_get_state(dfudev, stctrl);
	if (*f_pos == 0 && dfust != dfuIDLE) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		return -EINVAL;
	}
	if (*f_pos != 0) {
		if (dfust == dfuIDLE)
			return 0;
		if (dfust != dfuUPLOAD_IDLE) {
			dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n",
					dfust);
			return -EINVAL;
		}
	}
	if (!access_ok(VERIFY_WRITE, buff, count))
		return -EFAULT;
	if (count == 0)
		return 0;

	opctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->opctrl->urb)
		return -ENOMEM;

	ctrler = dfudev->usbdev->bus->controller;
	dmabuf = ~0;
	dma = 0;
	opctrl->req.bRequestType = 0xa1;
	opctrl->req.bRequest = USB_DFU_UPLOAD;
	opctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	opctrl->req.wLength = cpu_to_le16(dfudev->xfersize);
	opctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	opctrl->buff = dfudev->databuf;
	opctrl->len = dfudev->xfersize;
	if (dfudev->dma) {
		dmabuf = dma_map_single(ctrler, opctrl->buff, opctrl->len,
					DMA_FROM_DEVICE);
		if (!dma_mapping_error(ctrler, dmabuf)) {
			dma = 1;
			opctrl->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
			opctrl->urb->transfer_dma = dmabuf;
		}
	}

	numb = 0;
	blknum = *f_pos / dfudev->xfersize;
	do {
		opctrl->nxfer = 0;
		opctrl->req.wValue = cpu_to_le16(blknum);
		retv = dfu_submit_urb(dfudev, opctrl);
		dfu_get_status(dfudev, stctrl);
		dfust = stctrl->dfuStatus.bState;
		if ((dfust != dfuUPLOAD_IDLE && dfust != dfuIDLE) ||
				retv != 0) {
			dev_err(&dfudev->intf->dev, "Bad state in uploading: %d\n",
					dfust);
			break;
		}
		dma_sync_single_for_cpu(ctrler, dmabuf, opctrl->len, DMA_FROM_DEVICE);
		numb += ACCESS_ONCE(opctrl->nxfer);
		nbytes = copy_to_user(buff+numb, dfudev->databuf, opctrl->nxfer);
		blknum++;
	} while (opctrl->nxfer == opctrl->len && numb < count);
	*f_pos += numb;

	if (dma)
		dma_unmap_single(ctrler, dmabuf, opctrl->len,
				DMA_FROM_DEVICE);
	usb_free_urb(opctrl->urb);
	return numb;
}

static const struct file_operations dfu_fops = {
	.owner		= THIS_MODULE,
	.open		= dfu_open,
	.release	= dfu_release,
	.read		= dfu_upload,
	.write		= NULL, /*dfu_download,*/
};

static int dfu_get_status(const struct dfu_device *dfudev,
		struct dfu_control *ctrl)
{
	int retv;

	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = USB_DFU_GETSTATUS;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(6);
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->dfuStatus;
	ctrl->len = 6;
	ctrl->urb = NULL;
	retv = dfu_submit_urb(dfudev, ctrl);

	return retv;
}

static int dfu_reset(const struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	ctrl->ocupy = 0;
	ctrl->cmd.cmdata[0] = 0x07; /*DFU_CMD_RESET;*/
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(sizeof(struct dfu_cmd));
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	ctrl->urb = NULL;
	return dfu_submit_urb(dfudev, ctrl);
}

static int dfu_do_switch(struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	int tmout, retusb;

	tmout = dfudev->dettmout > detach_timeout ?
				detach_timeout : dfudev->dettmout;
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DETACH;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = cpu_to_le16(tmout);
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	ctrl->urb = NULL;
	retusb = dfu_submit_urb(dfudev, ctrl);
	if (retusb == 0 && (dfudev->attr & 0x08) == 0)
		dev_info(&dfudev->intf->dev, "Need reset to switch to DFU\n");
	return retusb;
}

static int dfu_clr_status(const struct dfu_device *dfudev,
		struct dfu_control *ctrl)
{
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_CLRSTATUS;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	ctrl->urb = NULL;
	return dfu_submit_urb(dfudev, ctrl);
}

static int dfu_get_state(const struct dfu_device *dfudev,
		struct dfu_control *ctrl)
{
	int retv;

	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = USB_DFU_GETSTATE;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(1);
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->dfuState;
	ctrl->len = 1;
	ctrl->urb = NULL;
	retv = dfu_submit_urb(dfudev, ctrl);
	if (retv == 0)
		retv = ctrl->dfuState;
	return retv;
}

static ssize_t dfu_switch(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct dfu_control *ctrl;

	dfudev = container_of(attr, struct dfu_device, actattr);

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	if (count > 0 && *buf == '-' && (*(buf+1) == '\n' || *(buf+1) == 0)) {
		if (dfudev->proto == 1)
			dfu_do_switch(dfudev, ctrl);
		else if (dfudev->ftus.reset)
			dfu_reset(dfudev, ctrl);
	} else
		dev_err(dev, "Invalid Command: %c\n", *buf);

	kfree(ctrl);
	return count;
}

static ssize_t dfu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;
	int retv, bst;
	struct dfu_control *ctrl;
	const char *fmt = "Attribute: %#02.2x Timeout: %d Transfer Size: %d ";

	retv = 0;
	dfudev = container_of(attr, struct dfu_device, actattr);
	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	
	retv = snprintf(buf, 128, fmt, dfudev->attr, dfudev->dettmout,
			dfudev->xfersize);
	if (dfudev->proto == 1)
		retv += snprintf(buf+retv, 128-retv, "\n");
	else {
		bst = dfu_get_state(dfudev, ctrl);
		retv += snprintf(buf+retv, 128-retv, "Current State: %d\n",
				bst);
	}
	kfree(ctrl);
	return retv;
}

static ssize_t dfu_size(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;
	int retv, bst;
	struct dfu_control *ctrl;
	int blksize, numblks, addrh, addrl;

	retv = 0;
	dfudev = container_of(attr, struct dfu_device, dfuattr);
	if (!mutex_trylock(&dfudev->dfulock))
		return -EBUSY;
	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl) {
		retv = -ENOMEM;
		goto exit_10;
	}
	bst = dfu_get_state(dfudev, ctrl);
	if (bst != dfuIDLE) {
		retv = -ETXTBSY;
		dev_err(&dfudev->intf->dev, "Bad DFU State: %d\n", bst);
		goto exit_20;
	}
	ctrl->ocupy = 0;
	ctrl->cmd.cmdata[0] = 0x05; /*DFU_CMD_INFO;*/

	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(sizeof(struct dfu_cmd));
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->cmd;
	ctrl->len = sizeof(struct dfu_cmd);
	ctrl->urb = NULL;
	retv = dfu_submit_urb(dfudev, ctrl);
	if (retv)
		goto exit_20;
	retv = dfu_get_status(dfudev, ctrl);
	if (retv != 0 || ctrl->dfuStatus.bState != dfuIDLE)
		goto exit_20;

	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = USB_DFU_UPLOAD;
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(sizeof(struct dfu_info));
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->info;
	ctrl->len = sizeof(struct dfu_info);
	ctrl->urb = NULL;
	retv = dfu_submit_urb(dfudev, ctrl);
	if (retv)
		goto exit_20;
	blksize = le16_to_cpu(ctrl->info.blksize);
	numblks = le16_to_cpu(ctrl->info.numblks);
	addrh = le16_to_cpu(ctrl->info.addrHigh);
	addrl = le16_to_cpu(ctrl->info.addrLow);
	retv = snprintf(buf, 128, "block size: %d, number of blocks: %d ",
		blksize, numblks);
	retv += snprintf(buf+retv, 128, "Addr High: %8.8X, Addr Low: %8.8X\n",
			addrh, addrl);

exit_20:
	kfree(ctrl);
exit_10:
	mutex_unlock(&dfudev->dfulock);
	return retv;
}

static int dfu_setup_stellaris(struct dfu_device *dfudev)
{
	struct dfu_control *ctrl;
	int retv, retusb;

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	retv = 0;
	ctrl->tiva.usMarker = 0;
	ctrl->tiva.usVersion = 0;
	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = 0x42; /* Tiva DFU Request */
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0x23;
	ctrl->req.wLength = 4;
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->tiva;
	ctrl->len = 4;
	ctrl->urb = NULL;
	retusb = dfu_submit_urb(dfudev, ctrl);
	if (ctrl->tiva.usVersion != 0)
		dfudev->ftus.reset = 1;
	dev_info(&dfudev->intf->dev, "Support of Tiva: %d\n", (int)dfudev->ftus.reset);

	kfree(ctrl);
	return retv;
}

static int dfu_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv, dfufdsc_len;
	struct usb_interface_descriptor *intfdsc;
	struct dfu_device *dfudev;
	struct dfufdsc *dfufdsc;
	__u16 idv, idp;
	

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
		retv = -ENODEV;
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
	if (dfudev->usbdev->bus->controller->dma_mask)
		dfudev->dma = 1;
	else
		dfudev->dma = 0;
	mutex_init(&dfudev->dfulock);

	intfdsc = &intf->cur_altsetting->desc;
	if (intfdsc->bInterfaceProtocol == USB_DFU_PROTO_RUNTIME) {
		dfudev->actattr.attr.name = "detach";
		dfudev->proto = 1;
	} else {
		dfudev->actattr.attr.name = "attach";
		dfudev->proto = 2;
	}
	dfudev->actattr.attr.mode = S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->actattr.show = dfu_show;
	dfudev->actattr.store = dfu_switch;
	usb_set_intfdata(intf, dfudev);
	retv = device_create_file(&intf->dev, &dfudev->actattr);
	if (retv != 0) {
		dev_err(&intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_10;
	}
	if (dfudev->proto == 1)
		return retv;

	cdev_init(&dfudev->dfu_cdev, &dfu_fops);
	dfudev->dfu_cdev.owner = THIS_MODULE;
	retv = cdev_add(&dfudev->dfu_cdev, dfudev->devnum, 1);
	if (retv) {
		dev_err(&dfudev->intf->dev, "Cannot register device: %d\n",
				retv);
		goto err_20;
	}
	dfudev->sysdev = device_create(dfu_class, &intf->dev,
			dfudev->devnum, dfudev, DFUDEV_NAME"%d", dfudev->index);
	if (IS_ERR(dfudev->sysdev)) {
		retv = (int)PTR_ERR(dfudev->sysdev);
		dev_err(&dfudev->intf->dev, "Cannot create device file: %d\n",
				retv);
		goto err_30;
	}

	dfudev->feat = 0;
	idv = le16_to_cpu(dfudev->usbdev->descriptor.idVendor);
	idp = le16_to_cpu(dfudev->usbdev->descriptor.idProduct);
	if (idv == USB_VENDOR_LUMINARY && idp == USB_PRODUCT_STELLARIS_DFU)
		retv = dfu_setup_stellaris(dfudev);

	if (retv)
		goto err_40;

	return retv;

err_40:
	device_destroy(dfu_class, dfudev->devnum);
err_30:
	cdev_del(&dfudev->dfu_cdev);
err_20:
	device_remove_file(&intf->dev, &dfudev->actattr);
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
	if (dfudev->proto == 2) {
		device_destroy(dfu_class, dfudev->devnum);
		cdev_del(&dfudev->dfu_cdev);
	}
	device_remove_file(&intf->dev, &dfudev->actattr);
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
