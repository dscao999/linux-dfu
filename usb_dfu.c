/*
 * usbdfu.c
 *
 * Copyright (c) 2017 Dashi Cao        <dscao999@hotmail.com, caods1@lenovo.com>
 *
 * USB Abstract Control Model driver for USB Device Firmware Upgrade
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>

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

#define MAX_FMSIZE	(0x7ful << 56)

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
	union {
		unsigned char attrs;
		struct {
			unsigned int detach_attr:1;
			unsigned int capbility_attr:1;
			unsigned int abort_attr:1;
			unsigned int firmware_attr:1;
			unsigned int fmsize_attr:1;
			unsigned int status_attr:1;
		};
	};
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

int dfu_submit_urb(struct dfu_device *dfudev, struct usb_ctrlrequest *req,
	       	int tmout, void *datbuf, int buflen)
{
	int retusb, retv, pipe;
	unsigned long jiff_wait;

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
	if (retv)
		dev_err(&dfudev->intf->dev,
			"URB type: %2.2x, req: %2.2x request failed: %d\n",
			(int)req->bRequestType, (int)req->bRequest, retv);

	return retv;
}

#define DFUDEV_NAME "dfu"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("USB DFU Driver");


static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, 0644);
MODULE_PARM_DESC(urb_timeout, "USB urb completion timeout. "
	"Default: 200 milliseconds.");

static const struct usb_device_id dfu_ids[] = {
	{	.match_flags = USB_DEVICE_ID_MATCH_VENDOR|
			USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor = 0x1cbe,
		.bInterfaceClass = USB_CLASS_APP_SPEC,
		.bInterfaceSubClass = USB_DFU_SUBCLASS,
		.bInterfaceProtocol = USB_DFU_PROTO_RUNTIME
	},
	{	.match_flags = USB_DEVICE_ID_MATCH_VENDOR|
			USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor = 0x1cbe,
		.bInterfaceClass = USB_CLASS_APP_SPEC,
		.bInterfaceSubClass = USB_DFU_SUBCLASS,
		.bInterfaceProtocol = USB_DFU_PROTO_DFUMODE
	},
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
	return dfu_submit_urb(dfudev, &dfudev->auxreq, urb_timeout, NULL, 0);
}

static inline int dfu_detach(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->auxreq.bRequest = USB_DFU_DETACH;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = dfudev->dettmout > 5000? 5000 : dfudev->dettmout;
	dfudev->auxreq.wLength = 0;
	return dfu_submit_urb(dfudev, &dfudev->auxreq, urb_timeout, NULL, 0);
}

static inline int dfu_get_status(struct dfu_device *dfudev)
{
	dfudev->auxreq.bRequestType = USB_DFU_FUNC_UP;
	dfudev->auxreq.bRequest = USB_DFU_GETSTATUS;
	dfudev->auxreq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->auxreq.wValue = 0;
	dfudev->auxreq.wLength = cpu_to_le16(6);
	return dfu_submit_urb(dfudev, &dfudev->auxreq, urb_timeout,
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
	retv = dfu_submit_urb(dfudev, &dfudev->auxreq, urb_timeout,
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
	return dfu_submit_urb(dfudev, &dfudev->auxreq, urb_timeout, NULL, 0);
}

static inline int dfu_finish_dnload(struct dfu_device *dfudev)
{
	dfudev->prireq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->prireq.bRequest = USB_DFU_DNLOAD;
	dfudev->prireq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->prireq.wValue = 0;
	dfudev->prireq.wLength = 0;
	return dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout, NULL, 0);
}

static inline int wmsec2int(unsigned char *wmsec)
{
	return (wmsec[2] << 16)|(wmsec[1] << 8) | wmsec[0];
}

static int dfu_wait_state(struct dfu_device *dfudev, int state_mask)
{
	int count = 0, usb_resp, mwait;

	state_mask |= (1<<dfuERROR);
	do {
		usb_resp = dfu_get_status(dfudev);
		if (usb_resp) {
			dev_err(&dfudev->intf->dev, "Cannot get DFU status: " \
					"%d\n", usb_resp);
			return usb_resp;
		}
		if (state_mask & (1 << dfudev->status.bState))
			break;
		mwait = wmsec2int(dfudev->status.wmsec);
		msleep(mwait);
		count += 1;
	} while (count < 5);
	if (count == 5)
		dev_err(&dfudev->intf->dev, "DFU Stalled\n");
	return dfudev->status.bState;
}
static ssize_t abort_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct usb_interface *intf;
	char *strbuf;

	if (count != 3 || memcmp(buf, "xxx", 3) != 0) {
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

	intf = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(intf);
	dfu_abort(dfudev);
	return count;
}

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
	if ((dfudev->cap & CAN_DETACH) == 0) {
		resp = dfu_get_state(dfudev);
		if (resp != appDETACH) {
			dev_err(dev, "DFU device is not in appDETACH state: %d\n",
					resp);
			goto exit_10;
		}
		usb_reset_device(dfudev->usbdev);
	}
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
		char *buf, loff_t offset, size_t bufsz)
{
	struct device *dev;
	struct usb_interface *intf;
	struct dfu_device *dfudev;
	int pos, usb_resp, remlen, dfu_state, blknum, state_mask;
	unsigned long fm_size;
	char *curbuf;

	if (unlikely(bufsz == 0))
		return bufsz;
	fm_size = binattr->size;
	if (fm_size == 0)
		fm_size = MAX_FMSIZE;

	dev = container_of(kobj, struct device, kobj);
	intf = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(intf);
	if ((dfudev->cap & CAN_UPLOAD) == 0) {
		dev_warn(dev, "has no upload capbility\n");
		return 0;
	}
	if ((bufsz % dfudev->xfersize) != 0) {
		dev_err(&dfudev->intf->dev, "Buffer size: %lu is not a " \
				"mutiple of DFU transfer size: %d\n",
				bufsz, dfudev->xfersize);
		return -EINVAL;
	}
	pos = 0;
	curbuf = buf;
	remlen = offset + bufsz <= fm_size? bufsz : fm_size - offset;
	dfudev->prireq.bRequestType = USB_DFU_FUNC_UP;
	dfudev->prireq.bRequest = USB_DFU_UPLOAD;
	dfudev->prireq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->prireq.wLength = cpu_to_le16(dfudev->xfersize);
	blknum = offset / dfudev->xfersize;

	mutex_lock(&dfudev->lock);
	dfu_state = dfu_get_state(dfudev);
	if (offset > 0 && dfu_state == dfuIDLE)
		goto exit_10;
	if ((offset == 0 && dfu_state != dfuIDLE) ||
			(offset > 0 && dfu_state != dfuUPLOAD_IDLE)) {
		dev_err(&dfudev->intf->dev, "Incompatible State for " \
				"uploading: %d, Offset: %lld\n",
				dfu_state, offset);
		pos = -EPROTO;
		goto exit_10;
	}
	dfu_state = dfuUPLOAD_IDLE;
	state_mask = (1<<dfuUPLOAD_IDLE|1<<dfuIDLE);
	while (remlen > dfudev->xfersize && offset + pos < fm_size &&
			dfu_state == dfuUPLOAD_IDLE) {
		dfudev->prireq.wValue = cpu_to_le16(blknum);
		usb_resp = dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout,
				curbuf, dfudev->xfersize);
		if (usb_resp) {
			dev_err(dev, "DFU upload error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(dfudev->nxfer == 0);
		pos += dfudev->nxfer;
		curbuf += dfudev->nxfer;
		remlen -= dfudev->nxfer;
		blknum += 1;
		dfu_state = dfu_wait_state(dfudev, state_mask);
	}
	if (dfu_state == dfuIDLE)
		goto exit_10;
	if (unlikely(dfu_state != dfuUPLOAD_IDLE)) {
		dev_err(&dfudev->intf->dev, "Cannot continue uploading, " \
				"inconsistent state: %d\n", dfu_state);
		goto exit_10;
	}
	if (offset + pos == fm_size) {
		dfu_abort(dfudev);
		goto exit_10;
	}
	BUG_ON(remlen == 0);
	dfudev->prireq.wValue = cpu_to_le16(blknum);
	dfudev->prireq.wLength = cpu_to_le16(remlen);
	usb_resp = dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout,
			curbuf, remlen);
	if (usb_resp) {
		dev_err(dev, "DFU upload error: %d\n", usb_resp);
		pos = usb_resp;
		goto exit_10;
	}
	WARN_ON(dfudev->nxfer == 0);
	pos += dfudev->nxfer;
	dfu_state = dfu_wait_state(dfudev, state_mask);
	if (offset + pos == fm_size) {
		if (dfu_state == dfuUPLOAD_IDLE)
			dfu_abort(dfudev);
	}

exit_10:
	mutex_unlock(&dfudev->lock);
	return pos;
}

ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize)
{
	struct device *dev;
	struct usb_interface *intf;
	struct dfu_device *dfudev;
	int dfu_state, pos, usb_resp, remlen, blknum, state_mask;
	unsigned long fm_size;
	char *curbuf;

	if (unlikely(bufsize == 0))
		return 0;
	fm_size = binattr->size;
	if (fm_size == 0) {
		dev_err(dev, "The image size of DFU Device is unspecified. " \
				"Cannot program the device\n");
		return -EINVAL;
	}

	dev = container_of(kobj, struct device, kobj);
	intf = container_of(dev, struct usb_interface, dev);
	dfudev = usb_get_intfdata(intf);
	if ((dfudev->cap & CAN_DOWNLOAD) == 0) {
		dev_err(dev, "DFU Device has no download capbility\n");
		return -EINVAL;
	}
	if (offset == fm_size && bufsize > 0) {
		dev_err(dev, "Cannot program past the end of image. " \
				"Current offset: %lld\n", offset);
		return -EINVAL;
	}
	pos = 0;
	remlen = offset + bufsize <= fm_size? bufsize : fm_size - offset;
	curbuf = buf;
	dfudev->prireq.bRequestType = USB_DFU_FUNC_DOWN;
	dfudev->prireq.bRequest = USB_DFU_DNLOAD;
	dfudev->prireq.wIndex = cpu_to_le16(dfudev->intfnum);
	dfudev->prireq.wLength = cpu_to_le16(dfudev->xfersize);
	blknum = offset / dfudev->xfersize;
	mutex_lock(&dfudev->lock);
	dfu_state = dfu_get_state(dfudev);
	if ((offset == 0 && dfu_state != dfuIDLE) ||
			(offset > 0 && dfu_state != dfuDNLOAD_IDLE)) {
		dev_err(dev, "Inconsistent DFU State, offset: %lld " \
				"State: %d\n", offset, dfu_state);
		pos = -EPROTO;
		goto exit_10;
	}
	dfu_state = dfuDNLOAD_IDLE;
	state_mask = (1 << dfuDNLOAD_IDLE);
	while (remlen > dfudev->xfersize && offset + pos < fm_size &&
			dfu_state == dfuDNLOAD_IDLE) {
		dfudev->prireq.wValue = cpu_to_le16(blknum);
		usb_resp = dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout,
				curbuf, dfudev->xfersize);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(dfudev->nxfer == 0);
		pos += dfudev->nxfer;
		curbuf += dfudev->nxfer;
		remlen -= dfudev->nxfer;
		blknum += 1;
		dfu_state = dfu_wait_state(dfudev, state_mask);
	}
	if (unlikely(dfu_state != dfuDNLOAD_IDLE)) {
		dev_err(&dfudev->intf->dev, "Cannot continue downloading. " \
				"Invalid state: %d\n", dfu_state);
		goto exit_10;
	}
	if (offset + pos < fm_size) {
		BUG_ON(remlen == 0);
		dfudev->prireq.wValue = cpu_to_le16(blknum);
		dfudev->prireq.wLength = cpu_to_le16(remlen);
		usb_resp = dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout,
				curbuf, remlen);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(dfudev->nxfer == 0);
		pos += dfudev->nxfer;
		dfu_state = dfu_wait_state(dfudev, state_mask);
	}
	if (offset + pos == fm_size) {
		dfudev->prireq.wValue = cpu_to_le16(blknum+1);
		dfudev->prireq.wLength = 0;
		usb_resp = dfu_submit_urb(dfudev, &dfudev->prireq, urb_timeout,
				NULL, 0);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		state_mask = (1<<dfuMANIFEST)|(1<<dfuIDLE);
		dfu_state = dfu_wait_state(dfudev, state_mask);
		if (dfu_state == dfuIDLE)
			goto exit_10;
		msleep(wmsec2int(dfudev->status.wmsec)+10);
		if (dfudev->cap & CAN_MANIFEST)
			dfu_state = dfu_wait_state(dfudev, dfuIDLE);
		else {
			dfu_state = dfu_wait_state(dfudev,
					(1<<dfuMANIFEST_WAIT_RESET));
			if (dfu_state != dfuMANIFEST_WAIT_RESET)
				dev_err(&dfudev->intf->dev, "Inconsistent " \
						"state after downloading: %d\n",
						dfu_state);
			else
				usb_reset_device(dfudev->usbdev);
		}
	}

exit_10:
	mutex_unlock(&dfudev->lock);
	return pos;
}

static DEVICE_ATTR_WO(detach);
static DEVICE_ATTR_WO(abort);
static DEVICE_ATTR_RO(capbility);
static DEVICE_ATTR_RO(status);
static BIN_ATTR_RW(firmware, 0);

static ssize_t fmsize_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu", bin_attr_firmware.size);
}

static ssize_t fmsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t buflen)
{
	char *tmpbuf, *endchr;

	tmpbuf = kmalloc(buflen+1, GFP_KERNEL);
	if (!tmpbuf) {
		dev_err(dev, "Out of Memory\n");
		return -ENOMEM;
	}
	memcpy(tmpbuf, buf, buflen);
	tmpbuf[buflen] = 0;
	bin_attr_firmware.size = simple_strtoul(tmpbuf, &endchr, 10);
	kfree(tmpbuf);
	return buflen;
}

static DEVICE_ATTR_RW(fmsize);

static int dfu_create_attrs(struct dfu_device *dfudev)
{
	int retv;

	dfudev->attrs = 0;
	if (dfudev->proto == USB_DFU_PROTO_RUNTIME) {
		retv = device_create_file(&dfudev->intf->dev, &dev_attr_detach);
		if (unlikely(retv != 0))
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			dfudev->detach_attr = 1;
	} else {
		retv = device_create_file(&dfudev->intf->dev, &dev_attr_status);
		if (unlikely(retv != 0))
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			dfudev->status_attr = 1;
		retv = device_create_file(&dfudev->intf->dev, &dev_attr_fmsize);
		if (unlikely(retv != 0))
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			dfudev->fmsize_attr = 1;
		retv = device_create_file(&dfudev->intf->dev, &dev_attr_abort);
		if (unlikely(retv != 0))
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			dfudev->abort_attr = 1;
		retv = sysfs_create_bin_file(&dfudev->intf->dev.kobj,
				&bin_attr_firmware);
		if (unlikely(retv != 0))
			dev_warn(&dfudev->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			dfudev->firmware_attr = 1;
	}
	retv = device_create_file(&dfudev->intf->dev, &dev_attr_capbility);
	if (unlikely(retv != 0))
		dev_warn(&dfudev->intf->dev, "Cannot create sysfs file %d\n",
				retv);
	else
		dfudev->capbility_attr = 1;
	return retv;
}

static void dfu_remove_attrs(struct dfu_device *dfudev)
{
	if (dfudev->detach_attr)
		device_remove_file(&dfudev->intf->dev, &dev_attr_detach);
	if (dfudev->firmware_attr)
		sysfs_remove_bin_file(&dfudev->intf->dev.kobj, &bin_attr_firmware);
	if (dfudev->abort_attr)
		device_remove_file(&dfudev->intf->dev, &dev_attr_abort);
	if (dfudev->fmsize_attr)
		device_remove_file(&dfudev->intf->dev, &dev_attr_fmsize);
	if (dfudev->status_attr)
		device_remove_file(&dfudev->intf->dev, &dev_attr_status);
	if (dfudev->capbility_attr)
		device_remove_file(&dfudev->intf->dev, &dev_attr_capbility);
}

static int dfu_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv, dfufdsc_len, resp;
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
	if (dfudev->proto == USB_DFU_PROTO_DFUMODE) {
		resp = dfu_get_status(dfudev);
		if (dfudev->status.bState != dfuIDLE)
			dev_warn(&dfudev->intf->dev, "Not in idle state: %d\n",
					dfudev->status.bState);
	}
	dev_info(&dfudev->intf->dev, "USB DFU inserted, CAN: %02x PROTO: %d, " \
			"Poll Time Out: %d\n", (int)dfudev->cap, dfudev->proto,
			wmsec2int(dfudev->status.wmsec));
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
