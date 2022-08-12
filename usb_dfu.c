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
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#define USB_DFU_DETACH		0
#define USB_DFU_DNLOAD		1
#define USB_DFU_UPLOAD		2
#define USB_DFU_GETSTATUS	3
#define USB_DFU_CLRSTATUS	4
#define USB_DFU_GETSTATE	5
#define USB_DFU_ABORT		6

#define USB_DFU_SUBCLASS	1
#define USB_DFU_PROTO_RUNTIME	1
#define USB_DFU_PROTO_DFUMODE	2

#define USB_DFU_FUNC_DSCLEN	9
#define USB_DFU_FUNC_DSCTYP	0x21

#define USB_DFU_FUNC_DOWN	0x21
#define USB_DFU_FUNC_UP		0xa1
#define USB_DFU_ERROR_CODE	255

#define CAN_DOWNLOAD	1
#define CAN_UPLOAD	2
#define CAN_MANIFEST	4
#define CAN_DETACH	8

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

struct dfu_device {
	struct mutex lock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct device *sysdev;
	struct usb_ctrlrequest prireq, auxreq;
	struct completion urbdone;
	struct urb *urb;
	int nxfer;
	int intfnum;
	int dettmout;
	int xfersize;
	int proto;
	int dma;
	volatile int resp;
	struct dfu_status status;
	__u8 cap;
	__u8 state;
};

static void dfu_urb_done(struct urb *urb)
{
	struct dfu_device *dfudev;

	dfudev = urb->context;
	dfudev->resp = urb->status;
	dfudev->nxfer = urb->actual_length;
	complete(&dfudev->urbdone);
}

static void dfu_urb_timeout(struct dfu_device *dfudev)
{
	usb_unlink_urb(dfudev->urb);
	wait_for_completion(&dfudev->urbdone);
}

int dfu_submit_urb(struct dfu_device *dfudev, int primary, int tmout,
		void *datbuf, int buflen)
{
	int retusb, retv, pipe;
	unsigned long jiff_wait;
	struct usb_ctrlrequest *req;

	if (primary)
		req = &dfudev->prireq;
	else
		req = &dfudev->auxreq;
	pipe = -1;
	if (req->bRequestType == USB_DFU_FUNC_DOWN)
		pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	else if (req->bRequestType == USB_DFU_FUNC_UP)
		pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	usb_fill_control_urb(dfudev->urb, dfudev->usbdev, pipe,
			(unsigned char *)req,
			datbuf, buflen, dfu_urb_done, dfudev);
	init_completion(&dfudev->urbdone);
	dfudev->resp = USB_DFU_ERROR_CODE;
	dfudev->nxfer = 0;
	retusb = usb_submit_urb(dfudev->urb, GFP_KERNEL);
	if (retusb != 0) {
		dev_err(&dfudev->intf->dev,
			"URB type: %2.2x, req: %2.2x submit failed: %d\n",
			(int)req->bRequestType, (int)req->bRequest, retusb);
		return retusb;
	}

	jiff_wait = msecs_to_jiffies(tmout);
	if (!wait_for_completion_timeout(&dfudev->urbdone, jiff_wait)) {
		dfu_urb_timeout(dfudev);
		dev_err(&dfudev->intf->dev,
			"URB req type: %2.2x, req: %2.2x timeout\n",
			(int)req->bRequestType, (int)req->bRequest);
		return dfudev->resp;
	}
	retv = READ_ONCE(dfudev->resp);
	if (retv && (retv != -EPROTO ||
				req->bRequest != USB_DFU_DETACH ||
				(dfudev->cap & CAN_DETACH) == 0)) {
		dev_err(&dfudev->intf->dev,
			"URB type: %2.2x, req: %2.2x request failed: %d\n",
			(int)req->bRequestType, (int)req->bRequest, retv);
	}

	return retv;
}

#define BLKSIZE	1024
#define DFUDEV_NAME "dfu"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Driver");


static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, 0644);
MODULE_PARM_DESC(urb_timeout, "USB urb completion timeout. "
	"Default: 200 milliseconds.");

/*static const struct usb_device_id dfu_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC,
			USB_DFU_SUBCLASS,
			USB_DFU_PROTO_RUNTIME)
	},
	{ USB_INTERFACE_INFO(USB_CLASS_APP_SPEC,
			USB_DFU_SUBCLASS,
			USB_DFU_PROTO_DFUMODE)
	},
	{ }
}; */
static const struct usb_device_id dfu_ids[] = {
	{ USB_DEVICE_INTERFACE_CLASS(0x1cbe, 0x00ff, USB_CLASS_APP_SPEC) },
	{ USB_DEVICE_INTERFACE_CLASS(0x1cbe, 0x00fd, USB_CLASS_APP_SPEC) },
	{}
};

MODULE_DEVICE_TABLE(usb, dfu_ids);

static inline int dfu_abort(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->auxreq.bRequest = USB_DFU_ABORT;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = 0;
	dfudev->auxreq.wLength = 0;
	return dfu_submit_urb(dfudev, 0, urb_timeout, NULL, 0);
}

static inline int dfu_detach(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->auxreq.bRequest = USB_DFU_DETACH;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = dfudev->dettmout > 5000? 5000 : dfudev->dettmout;
	dfudev->auxreq.wLength = 0;
	return dfu_submit_urb(dfudev, 0, urb_timeout, NULL, 0);
}

static inline int dfu_get_status(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_UP;
	dfudev->auxreq.bRequest = USB_DFU_GETSTATUS;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = 0;
	dfudev->auxreq.wLength = cpu_to_le16(6);
	return dfu_submit_urb(dfudev, 0, urb_timeout,
			&dfudev->status, sizeof(dfudev->status));
}

static inline int dfu_get_state(struct dfu_device *dfudev)
{
	int retv;

	dfudev->auxreq.bRequestType = USB_DFU_FUNC_UP;
	dfudev->auxreq.bRequest = USB_DFU_GETSTATE;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = 0;
	dfudev->auxreq.wLength = cpu_to_le16(1);
	retv = dfu_submit_urb(dfudev, 0, urb_timeout,
			&dfudev->state, sizeof(dfudev->state));
	if (retv == 0)
		retv = dfudev->state;
	return retv;
}

static inline int dfu_clr_status(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->auxreq.bRequest = USB_DFU_CLRSTATUS;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = 0;
	dfudev->auxreq.wLength = 0;
	return dfu_submit_urb(dfudev, 0, urb_timeout, NULL, 0);
}

static inline int dfu_finish_dnload(struct dfu_device *dfudev)
{
	dfudev->prireq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->prireq.bRequest = USB_DFU_DNLOAD;
	dfudev->prireq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->prireq.wValue = 0;
	dfudev->prireq.wLength = 0;
	return dfu_submit_urb(dfudev, 1, urb_timeout, NULL, 0);
}

/*static int dfu_open(struct inode *inode, struct file *filp)
{
	struct dfu1_device *dfudev;
	int state, retv, buflen;
	struct dfu_device *ctrl;
	void *datbuf;

	retv = 0;
	dfudev = container_of(inode->i_cdev, struct dfu1_device, cdev);
	filp->private_data = dfudev;
	if (mutex_lock_interruptible(&dfudev->lock))
		return -EBUSY;

	buflen = altrim(dfudev->xfersize, 4) + 2*sizeof(struct dfu_control);
	datbuf = kmalloc(buflen, GFP_KERNEL);
	if (!datbuf) {
		retv = -ENOMEM;
		goto err_10;
	}
	dfudev->datbuf = datbuf;
	dfudev->opctrl = datbuf + altrim(dfudev->xfersize, 4);
	dfudev->opctrl->datbuf = datbuf;
	dfudev->opctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->opctrl->dfurb) {
		retv = -ENOMEM;
		goto err_20;
	}
	dfudev->opctrl->usbdev = dfudev->usbdev;
	dfudev->opctrl->intf = dfudev->intf;
	dfudev->opctrl->intfnum = dfudev->intfnum;
	
	dfudev->stctrl = dfudev->opctrl + 1;
	dfudev->stctrl->datbuf = NULL;
	dfudev->stctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->stctrl->dfurb) {
		retv = -ENOMEM;
		goto err_25;
	}
	dfudev->stctrl->usbdev = dfudev->usbdev;
	dfudev->stctrl->intf = dfudev->intf;
	dfudev->stctrl->intfnum = dfudev->intfnum;

	ctrl = dfudev->stctrl;
	state = dfu_get_state(ctrl);
	if (state != dfuIDLE) {
		dev_err(&dfudev->intf->dev, "Bad Initial State: %d\n", state);
		retv =  -EBUSY;
		goto err_30;
	}

	return retv;

err_30:
	usb_free_urb(dfudev->stctrl->dfurb);
err_25:
	usb_free_urb(dfudev->opctrl->dfurb);
err_20:
	kfree(datbuf);
err_10:
	mutex_unlock(&dfudev->lock);
	return retv;
}

static int dfu_release(struct inode *inode, struct file *filp)
{
	struct dfu1_device *dfudev;
	struct dfu_control *stctrl;
	int retv;

	dfudev = filp->private_data;
	stctrl = dfudev->stctrl;
	retv = dfu_get_state(stctrl);
	if (retv == dfuDNLOAD_IDLE)
		dfu_finish_dnload(stctrl);
	else if (retv == dfuERROR)
		dfu_clr_status(stctrl);
	else if (retv != dfuIDLE)
		dfu_abort(stctrl);
	msleep(100);
	retv = dfu_get_state(stctrl);
	if (retv != dfuIDLE)
		dev_err(&dfudev->intf->dev, "Need Reset! Stuck in State: %d\n",
				retv);
	usb_free_urb(stctrl->dfurb);
	usb_free_urb(dfudev->opctrl->dfurb);
	kfree(dfudev->datbuf);
	mutex_unlock(&dfudev->lock);
	filp->private_data = NULL;
	return 0;
}

static ssize_t dfu_upload(struct file *filp, char __user *buff, size_t count,
			loff_t *f_pos)
{
	struct dfu1_device *dfudev;
	int blknum, numb;
	struct dfu_control *opctrl, *stctrl;
	int dfust, dma, len;
	dma_addr_t dmabuf;
	struct device *ctrler;

	if (count == 0)
		return 0;
	dfudev = filp->private_data;
	opctrl = dfudev->opctrl;
	stctrl = dfudev->stctrl;
	dfust = dfu_get_state(stctrl);
	if (dfust != dfuIDLE && dfust != dfuUPLOAD_IDLE) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		return -EINVAL;
	}
	if (*f_pos != 0 && dfust == dfuIDLE)
		return 0;

	if (!access_ok(buff, count))
		return -EFAULT;

	ctrler = dfudev->usbdev->bus->controller;
	dmabuf = ~0;
	dma = 0;
	opctrl->req.bRequestType = 0xa1;
	opctrl->req.bRequest = USB_DFU_UPLOAD;
	opctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	opctrl->req.wLength = cpu_to_le16(dfudev->xfersize);
	opctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	opctrl->len = dfudev->xfersize;
	if (dfudev->dma) {
		dmabuf = dma_map_single(ctrler, opctrl->datbuf,
				dfudev->xfersize, DMA_FROM_DEVICE);
		if (!dma_mapping_error(ctrler, dmabuf)) {
			dma = 1;
			opctrl->dfurb->transfer_flags |=
						URB_NO_TRANSFER_DMA_MAP;
			opctrl->dfurb->transfer_dma = dmabuf;
		} else
			dev_warn(&dfudev->intf->dev,
					"Cannot map DMA address\n");
	}

	blknum = *f_pos / BLKSIZE;
	numb = 0;
	do {
		opctrl->req.wValue = cpu_to_le16(blknum);
		if (dfu_submit_urb(opctrl, urb_timeout) ||
				dfu_get_status(stctrl))
			break;
		dfust = stctrl->dfuStatus.bState;
		if (dfust != dfuUPLOAD_IDLE && dfust != dfuIDLE) {
			dev_err(&dfudev->intf->dev,
				"Uploading failed. DFU State: %d\n", dfust);
			break;
		}
		len = READ_ONCE(opctrl->nxfer);
		if (len == 0)
			break;
		*f_pos += len;
		blknum = *f_pos / BLKSIZE;
		dma_sync_single_for_cpu(ctrler, dmabuf, len, DMA_FROM_DEVICE);
		if (copy_to_user(buff+numb, opctrl->datbuf, len)) {
			dev_err(&dfudev->intf->dev,
				"Failed to copy data into user space!\n");
			break;
		}
		numb += len;
	} while (numb < count && dfust == dfuUPLOAD_IDLE);

	if (dma)
		dma_unmap_single(ctrler, dmabuf, dfudev->xfersize,
				DMA_FROM_DEVICE);

	return numb;
}

static ssize_t dfu_dnload(struct file *filp, const char __user *buff,
			size_t count, loff_t *f_pos)
{
	struct dfu_device *dfudev;
	int blknum, numb, fpos;
	struct dfu_control *opctrl, *stctrl;
	int dfust, dma, len, lenrem, tmout;
	dma_addr_t dmabuf;
	struct device *ctrler;

	if (count == 0)
		return 0;
	dfudev = filp->private_data;
	dfust = dfu_get_state(dfudev);
	if (dfust != dfuIDLE && dfust != dfuDNLOAD_IDLE) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		return -EINVAL;
	}
	if (!access_ok(buff, count))
		return -EFAULT;

	ctrler = dfudev->usbdev->bus->controller;
	dmabuf = ~0;
	dma = 0;
	dfudev->req.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->req.bRequest = USB_DFU_DNLOAD;
	dfudev->req.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	if (dfudev->dma) {
		dmabuf = dma_map_single(ctrler, opctrl->datbuf, dfudev->xfersize,
					DMA_TO_DEVICE);
		if (!dma_mapping_error(ctrler, dmabuf)) {
			dma = 1;
			opctrl->dfurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
			opctrl->dfurb->transfer_dma = dmabuf;
		} else
			dev_warn(&dfudev->intf->dev,
					"Cannot map DMA address\n");
	}

	fpos = *f_pos;
	blknum = fpos / BLKSIZE;
	lenrem = count;
	numb = 0;
	do {
		opctrl->len = dfudev->xfersize > lenrem ? lenrem :
							dfudev->xfersize;
		if (copy_from_user(dfudev->datbuf, buff+numb, opctrl->len))
			break;
		opctrl->req.wValue = cpu_to_le16(blknum);
		opctrl->req.wLength = cpu_to_le16(opctrl->len);
		dma_sync_single_for_device(ctrler, dmabuf, opctrl->len,
						DMA_TO_DEVICE);
		if (dfu_submit_urb(opctrl, urb_timeout) ||
				dfu_get_status(stctrl))
			break;
		len = READ_ONCE(opctrl->nxfer);
		if (len == 0)
			break;
		numb += len;
		lenrem -= len;
		fpos += len;
		blknum = fpos / BLKSIZE;
		while (stctrl->dfuStatus.bState == dfuDNLOAD_BUSY) {
			tmout = stctrl->dfuStatus.wmsec[0] |
					(stctrl->dfuStatus.wmsec[1] << 8) |
					(stctrl->dfuStatus.wmsec[2] << 16);
			msleep(tmout);
			if (dfu_get_status(stctrl))
				break;
		}
		if (stctrl->dfuStatus.bState != dfuDNLOAD_IDLE &&
		    stctrl->dfuStatus.bState != dfuIDLE) {
			dev_err(&dfudev->intf->dev,
				"Downloading failed. DFU State: %d\n", dfust);
			break;
		}
	} while (lenrem > 0);
	if (*f_pos != 0 || numb > 32)
		*f_pos += numb;

	if (dma)
		dma_unmap_single(ctrler, dmabuf, dfudev->xfersize,
				DMA_FROM_DEVICE);

	return numb;
}

static const struct file_operations dfu_fops = {
	.owner		= THIS_MODULE,
	.open		= dfu_open,
	.release	= dfu_release,
	.read		= dfu_upload,
	.write		= dfu_dnload
};

static ssize_t dfu_sndcmd(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu1_device *dfudev;
	struct dfu_control *ctrl;

	if (count < 1 || count > 32)
		return count;

	dfudev = container_of(attr, struct dfu1_device, tachattr);
	if (!mutex_trylock(&dfudev->lock)) {
		dev_err(&dfudev->intf->dev,
				"Cannot send command, device busy\n");
		return count;
	}
	ctrl = kmalloc(sizeof(struct dfu_control) + count, GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ctrl->dfurb) {
		kfree(ctrl);
		return -ENOMEM;
	}

	ctrl->usbdev = dfudev->usbdev;
	ctrl->intf = dfudev->intf;
	ctrl->intfnum = dfudev->intfnum;
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wValue = 0;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wLength = cpu_to_le16(count);
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->datbuf = ctrl + 1;
	memcpy(ctrl->datbuf, buf, count);
	ctrl->len = count;
	if (dfu_submit_urb(ctrl, urb_timeout) ||
			dfu_get_status(ctrl))
		dev_err(&dfudev->intf->dev,
				"DFU command failed: %d, State: %d\n",
				(int)ctrl->dfuStatus.bStatus,
				(int)ctrl->dfuStatus.bState);
	else
		dev_info(&dfudev->intf->dev,
				"DFU commmand status: %d, State: %d\n",
				(int)ctrl->dfuStatus.bStatus,
				(int)ctrl->dfuStatus.bState);
	usb_free_urb(ctrl->dfurb);
	kfree(ctrl);
	mutex_unlock(&dfudev->lock);
	return count;
}

static ssize_t dfu_attr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu1_device *dfudev;

	dfudev = container_of(attr, struct dfu1_device, attrattr);
	return sprintf(buf, "Download:%d Upload:%d Manifest:%d Detach:%d\n",
		dfudev->download, dfudev->upload, dfudev->manifest,
		dfudev->detach);
}

static ssize_t dfu_timeout_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dfu1_device *dfudev;

	dfudev = container_of(attr, struct dfu1_device, tmoutattr);
	return sprintf(buf, "%d\n", dfudev->dettmout);
}

static ssize_t dfu_xfersize_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dfu1_device *dfudev;

	dfudev = container_of(attr, struct dfu1_device, xsizeattr);
	return sprintf(buf, "%d\n", dfudev->xfersize);
}

static ssize_t dfu_state_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu1_device *dfudev;
	struct dfu_control *ctrl;
	int dfstat;

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	dfudev = container_of(attr, struct dfu1_device, statattr);
	ctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ctrl->dfurb) {
		kfree(ctrl);
		return -ENOMEM;
	}
	ctrl->usbdev = dfudev->usbdev;
	ctrl->intf = dfudev->intf;
	ctrl->intfnum = dfudev->intfnum;
	dfstat = dfu_get_state(ctrl);
	usb_free_urb(ctrl->dfurb);
	kfree(ctrl);
	return sprintf(buf, "%d\n", dfstat);
}

static ssize_t stellaris_show(struct dfu1_device *dfudev,
			struct dfu_control *ctrl, char *buf)
{
	struct tidfu {
		unsigned short usMarker;
		unsigned short usVersion;
	} *stellaris;
	unsigned short usMarker, usVersion;
	int num = 0;

	ctrl->req.bRequest = 0x42;
	ctrl->req.bRequestType = 0xa1;
	ctrl->req.wValue = 0x23;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wLength = 4;
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->datbuf = ctrl->ocupy;
	ctrl->len = sizeof(struct tidfu);
	stellaris = ctrl->datbuf;
	if (dfu_submit_urb(ctrl, urb_timeout) == 0) {
		usMarker = le16_to_cpu(stellaris->usMarker);
		usVersion = le16_to_cpu(stellaris->usVersion);
		num = sprintf(buf, "Stellaris Marker: %4.4X, Version: %4.4X\n",
			usMarker, usVersion);
	}
	return num;
}

static ssize_t dfu_query_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu1_device *dfudev;
	struct dfu_control *ctrl;
	unsigned short idVendor, idProduct;
	ssize_t numbytes;

	dfudev = container_of(attr, struct dfu1_device, queryattr);
	idVendor = le16_to_cpu(dfudev->usbdev->descriptor.idVendor);
	idProduct = le16_to_cpu(dfudev->usbdev->descriptor.idProduct);
	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ctrl->dfurb) {
		kfree(ctrl);
		return -ENOMEM;
	}
	ctrl->usbdev = dfudev->usbdev;
	ctrl->intf = dfudev->intf;
	ctrl->intfnum = dfudev->intfnum;
	numbytes = 0;
	if (idVendor == USB_VENDOR_LUMINARY &&
	    idProduct == USB_PRODUCT_STELLARIS_DFU)
		numbytes = stellaris_show(dfudev, ctrl, buf);

	usb_free_urb(ctrl->dfurb);
	kfree(ctrl);
	return numbytes;
}

static ssize_t dfu_clear_cmd(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu1_device *dfudev;
	struct dfu_control *ctrl;
	int dfust;

	dfudev = container_of(attr, struct dfu1_device, abortattr);
	if (count < 1 || *buf != '1') {
		dev_warn(&dfudev->intf->dev, "Invalid command: %c\n", *buf);
		return count;
	}

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->dfurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ctrl->dfurb) {
		kfree(ctrl);
		return -ENOMEM;
	}

	ctrl->usbdev = dfudev->usbdev;
	ctrl->intf = dfudev->intf;
	ctrl->intfnum = dfudev->intfnum;
	dfust = dfu_get_state(ctrl);
	switch (dfust) {
	case dfuDNLOAD_IDLE:
	case dfuUPLOAD_IDLE:
		dfu_abort(ctrl);
		break;
	case dfuERROR:
		dfu_clr_status(ctrl);
		break;
	default:
		dev_warn(&dfudev->intf->dev, "Cannot clear, in state: %d\n",
				dfust);
		break;
	}
	usb_free_urb(ctrl->dfurb);
	kfree(ctrl);
	return count;
} */

static ssize_t detach_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct usb_interface *interface;
	int resp;
	char *strbuf;

	
	if (count != 3 || memcmp(buf, "---", 3) != 0) {
		strbuf = kmalloc(count+1, GFP_KERNEL);
		if (!strbuf) {
			dev_err(dev, "Out of Memory\n");
			return count;
		}
		memcpy(strbuf, buf, count);
		strbuf[count] = 0;
		dev_err(dev, "Invalid Detach Token: %s\n", strbuf);
		kfree(strbuf);
		return count;
	}

	interface = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(interface);
	mutex_lock(&dfudev->lock);
	resp = dfu_detach(dfudev);
	if (resp && resp != -EPROTO) {
		dev_err(dev, "Cannot detach the DFU device: %d\n", resp);
		goto exit_10;
	}
	if ((dfudev->cap & CAN_DETACH) == 0)
		usb_reset_device(dfudev->usbdev);
exit_10:
	mutex_unlock(&dfudev->lock);
	return count;
}

static ssize_t capbility_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dfu_device *dfudev;
	struct usb_interface *interface;
	int cap, download, upload, manifest, detach;

	interface = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(interface);
	cap = dfudev->cap;
	download = cap & 1;
	cap = cap >> 1;
	upload = cap & 1;
	cap = cap >> 1;
	manifest = cap & 1;
	cap = cap >> 1;
	detach = cap & 1;
	return sprintf(buf, "Download:%d Upload:%d Manifest:%d Detach:%d\n",
			download, upload, manifest, detach);
}

static ssize_t status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dfu_device *dfudev;
	struct usb_interface *interface;
	int resp, mwait, retv;

	interface = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(interface);
	mutex_lock(&dfudev->lock);
	resp = dfu_get_status(dfudev);
	mutex_unlock(&dfudev->lock);
	if (resp == 0) {
		mwait = dfudev->status.wmsec[2]<<16|
			dfudev->status.wmsec[1]<<8|
			dfudev->status.wmsec[0];
		retv = sprintf(buf, "Status: %hhd State: %hhd Wait: %d\n",
				dfudev->status.bStatus,
				dfudev->status.bState,
				mwait);
	} else {
		dev_err(dev, "Get DFU Status failed: %d\n", resp);
		retv = 0;
	}
	return retv;
}

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr, 
		char *buf, loff_t offset, size_t size)
{
	struct device *dev;
	struct usb_interface *intf;
	struct dfu_device *dfudev;
	int state, pos, blknum, usb_resp, mwait, count, remlen;
	char *curbuf;

	dev = container_of(kobj, struct device, kobj);
	intf = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(intf);
	if ((dfudev->cap & CAN_UPLOAD) == 0) {
		dev_warn(dev, "has no upload capbility\n");
		return 0;
	}
	pos = 0;
	curbuf = buf;
	remlen = size;
	blknum = offset / dfudev->xfersize;
	dfudev->prireq.bRequestType = USB_DFU_FUNC_UP;
	dfudev->prireq.bRequest = USB_DFU_UPLOAD;
	dfudev->prireq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->prireq.wLength = cpu_to_le16(dfudev->xfersize);

	mutex_lock(&dfudev->lock);
	state = dfu_get_state(dfudev);
	if (offset > 0 && state == dfuIDLE)
		goto exit_10;
	if ((offset == 0 && state != dfuIDLE) ||
			(offset > 0 && state != dfuUPLOAD_IDLE)) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", state);
		goto exit_10;
	}
	if (offset % dfudev->xfersize != 0)
		dev_warn(dev, "Offset: %llu not a multiple of transfer size: " \
				"%d\n", offset, dfudev->xfersize);
	while (remlen > dfudev->xfersize) {
		dfudev->prireq.wValue = cpu_to_le16(blknum);
		usb_resp = dfu_submit_urb(dfudev, 1, urb_timeout,
				curbuf, remlen);
		if (usb_resp) {
			dev_err(dev, "DFU upload error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		if (dfudev->nxfer == 0)
			goto exit_10;
		pos += dfudev->nxfer;
		curbuf += dfudev->nxfer;
		remlen -= dfudev->nxfer;
		blknum += 1;
		count = 0;
		do {
			usb_resp = dfu_get_status(dfudev);
			if (usb_resp) {
				dev_err(dev, "Cannot get DFU upload status: %d\n",
						usb_resp);
				goto exit_10;
			}
			mwait = dfudev->status.wmsec[2]<<16|
				dfudev->status.wmsec[1]<<8|
				dfudev->status.wmsec[0];
			if (dfudev->status.bState == dfuIDLE)
				goto exit_10;
			msleep(mwait);
			count += 1;
		} while (dfudev->status.bState != dfuUPLOAD_IDLE && count < 3);
		if (count == 3) {
			dev_err(dev, "DFU Stalled\n");
			goto exit_10;
		}
	}
	dfudev->prireq.wValue = cpu_to_le16(blknum);
	dfudev->prireq.wLength = cpu_to_le16(remlen);
	usb_resp = dfu_submit_urb(dfudev, 1, urb_timeout, curbuf, remlen);
	if (usb_resp) {
		dev_err(dev, "DFU upload error: %d\n", usb_resp);
		pos = usb_resp;
	} else
		pos += dfudev->nxfer;

exit_10:
	mutex_unlock(&dfudev->lock);
	return pos;
}

ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t pos, size_t size)
{
	return size;
}

static DEVICE_ATTR_WO(detach);
static DEVICE_ATTR_RO(capbility);
static DEVICE_ATTR_RO(status);
static BIN_ATTR_RW(firmware, 0);

static int dfu_create_attrs(struct dfu_device *dfudev)
{
	int retv;

	if (dfudev->proto == USB_DFU_PROTO_RUNTIME) {
		retv = device_create_file(&dfudev->intf->dev, &dev_attr_detach);
		if (retv != 0)
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
	} else {
		retv = sysfs_create_bin_file(&dfudev->intf->dev.kobj,
				&bin_attr_firmware);
		if (retv != 0)
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
	}
	retv = device_create_file(&dfudev->intf->dev, &dev_attr_capbility);
	if (retv != 0)
		dev_warn(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
	retv = device_create_file(&dfudev->intf->dev, &dev_attr_status);
	if (retv != 0)
		dev_warn(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
/*
	dfudev->attrattr.attr.name = "attr";
	dfudev->attrattr.attr.mode = 0444;
	dfudev->attrattr.show = dfu_attr_show;
	dfudev->attrattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->attrattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_10;
	}
	dfudev->tmoutattr.attr.name = "detach_timeout";
	dfudev->tmoutattr.attr.mode = 0444;
	dfudev->tmoutattr.show = dfu_timeout_show;
	dfudev->tmoutattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->tmoutattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_20;
	}
	dfudev->xsizeattr.attr.name = "xfersize";
	dfudev->xsizeattr.attr.mode = 0444;
	dfudev->xsizeattr.show = dfu_xfersize_show;
	dfudev->xsizeattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->xsizeattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_30;
	}
	dfudev->statattr.attr.name = "state";
	dfudev->statattr.attr.mode = 0444;
	dfudev->statattr.show = dfu_state_show;
	dfudev->statattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->statattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_40;
	}
	dfudev->abortattr.attr.name = "clear";
	dfudev->abortattr.attr.mode = 0200;
	dfudev->abortattr.store = dfu_clear_cmd;
	dfudev->abortattr.show = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->abortattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_50;
	}
	dfudev->queryattr.attr.name = "query";
	dfudev->queryattr.attr.mode =  0444;
	dfudev->queryattr.show = dfu_query_show;
	dfudev->queryattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->queryattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
		goto err_60;
	} */

	return retv;

/*err_60:
	device_remove_file(&dfudev->intf->dev, &dfudev->abortattr);
err_50:
	device_remove_file(&dfudev->intf->dev, &dfudev->statattr);
err_40:
	device_remove_file(&dfudev->intf->dev, &dfudev->xsizeattr);
err_30:
	device_remove_file(&dfudev->intf->dev, &dfudev->tmoutattr);
err_20:
	device_remove_file(&dfudev->intf->dev, &dfudev->attrattr); */
}

static void dfu_remove_attrs(struct dfu_device *dfudev)
{
	if (dfudev->proto == USB_DFU_PROTO_RUNTIME) {
		device_remove_file(&dfudev->intf->dev, &dev_attr_detach);
	} else {
		sysfs_remove_bin_file(&dfudev->intf->dev.kobj, &bin_attr_firmware);
	}
	device_remove_file(&dfudev->intf->dev, &dev_attr_capbility);
	device_remove_file(&dfudev->intf->dev, &dev_attr_status);
/*	device_remove_file(&dfudev->intf->dev, &dfudev->abortattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->statattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->xsizeattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->tmoutattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->attrattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->tachattr); */
}

/*static atomic_t dfu_index = ATOMIC_INIT(-1);
static atomic_t *dev_minors; */

static int dfu_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv, dfufdsc_len;
	struct dfu_device *dfudev;
	struct dfufdsc *dfufdsc;

	dev_info(&intf->dev, "DFU Probing called\n");
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

	dfudev->cap = dfufdsc->attr;
	dfudev->xfersize = le16_to_cpu(dfufdsc->xfersize);
	dfudev->dettmout = dfufdsc->tmout;
	dfudev->intf = intf;
	dfudev->usbdev = interface_to_usbdev(intf);
	dfudev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	dfudev->proto = intf->cur_altsetting->desc.bInterfaceProtocol;
	if (dfudev->usbdev->bus->controller->dma_mask)
		dfudev->dma = 1;
	else
		dfudev->dma = 0;
	dfudev->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->urb) {
		retv = -ENOMEM;
		goto err_10;
	}
	mutex_init(&dfudev->lock);

        usb_set_intfdata(intf, dfudev);
	dfu_create_attrs(dfudev);

	dev_info(&dfudev->intf->dev, "USB DFU inserted, CAN: %02x PROTO: %d\n",
			(int)dfudev->cap, dfudev->proto);
	return retv;

err_10:
	kfree(dfudev);
	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dfudev;

	dfudev = usb_get_intfdata(intf);
	mutex_lock(&dfudev->lock);
	usb_set_intfdata(intf, NULL);
	dfu_remove_attrs(dfudev);
	usb_free_urb(dfudev->urb);
	mutex_unlock(&dfudev->lock);
	kfree(dfudev);
	dev_info(&dfudev->intf->dev,  "USB DFU removed\n");
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

        return 0;
}

static void __exit usbdfu_exit(void)
{
	usb_deregister(&dfu_driver);
}

module_init(usbdfu_init);
module_exit(usbdfu_exit);
