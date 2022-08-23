/*
 * usb_icdi.c
 *
 * Copyright (c) 2017 Dashi Cao        <dscao999@hotmail.com, caods1@lenovo.com>
 *
 * USB ICDI Interface for TI development boards
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define MODULE_NAME	"usb_icdi"

#define ICDI_VID	0x1cbe
#define ICDI_PID	0x00fd

#define ICDI_START	"$"
#define ICDI_END	"#"

struct icdi_device {
	struct mutex lock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct completion urbdone;
	struct urb *urb;
	unsigned char *buf;
	int buflen;
	int nxfer;
	int intfnum;
	int xfersize;
	int pipe_in, pipe_out;
	volatile int resp;
	union {
		unsigned char attrs;
		struct {
			unsigned int firmware_attr:1;
			unsigned int fmsize_attr:1;
			unsigned int version_attr:1;
		};
	};
};

/*static int icdi_send(struct icdi_device *icdi, int *xfer)
{
}
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

struct icdi_status {
	__u8 bStatus;
	__u8 wmsec[3];
	__u8 bState;
	__u8 istr;
} __packed;

enum icdi_state {
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

static void icdi_urb_done(struct urb *urb)
{
	struct icdi_device *icdi;

	icdi = urb->context;
	icdi->resp = urb->status;
	icdi->nxfer = urb->actual_length;
	complete(&icdi->urbdone);
}

static void icdi_urb_timeout(struct icdi_device *icdi)
{
	usb_unlink_urb(icdi->urb);
	wait_for_completion(&icdi->urbdone);
}

int icdi_submit_urb(struct icdi_device *icdi, struct usb_ctrlrequest *req,
	       	int tmout, void *datbuf, int buflen)
{
	int retusb, retv, pipe;
	unsigned long jiff_wait;

	pipe = -1;
	if (req->bRequestType == USB_DFU_FUNC_DOWN)
		pipe = usb_sndctrlpipe(icdi->usbdev, 0);
	else if (req->bRequestType == USB_DFU_FUNC_UP)
		pipe = usb_rcvctrlpipe(icdi->usbdev, 0);
	usb_fill_control_urb(icdi->urb, icdi->usbdev, pipe,
			(unsigned char *)req,
			datbuf, buflen, icdi_urb_done, icdi);
	init_completion(&icdi->urbdone);
	icdi->resp = USB_DFU_ERROR_CODE;
	icdi->nxfer = 0;
	retusb = usb_submit_urb(icdi->urb, GFP_KERNEL);
	if (retusb != 0) {
		dev_err(&icdi->intf->dev,
			"URB type: %2.2x, req: %2.2x submit failed: %d\n",
			(int)req->bRequestType, (int)req->bRequest, retusb);
		return retusb;
	}

	jiff_wait = msecs_to_jiffies(tmout);
	if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
		icdi_urb_timeout(icdi);
		dev_err(&icdi->intf->dev,
			"URB req type: %2.2x, req: %2.2x timeout\n",
			(int)req->bRequestType, (int)req->bRequest);
		return icdi->resp;
	}
	retv = READ_ONCE(icdi->resp);
	if (retv)
		dev_err(&icdi->intf->dev,
			"URB type: %2.2x, req: %2.2x request failed: %d\n",
			(int)req->bRequestType, (int)req->bRequest, retv);

	return retv;
}*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("TI USB ICDI Driver");


static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, 0644);
MODULE_PARM_DESC(urb_timeout, "USB urb completion timeout. "
	"Default: 200 milliseconds.");

static const struct usb_device_id icdi_ids[] = {
	{	.match_flags = USB_DEVICE_ID_MATCH_DEVICE|
			USB_DEVICE_ID_MATCH_INT_CLASS,
		.idVendor = ICDI_VID,
		.idProduct = ICDI_PID,
		.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	},
	{}
};

MODULE_DEVICE_TABLE(usb, icdi_ids);

/*static inline int icdi_abort(struct icdi_device *icdi)
{
	icdi->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	icdi->auxreq.bRequest = USB_DFU_ABORT;
	icdi->auxreq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->auxreq.wValue = 0;
	icdi->auxreq.wLength = 0;
	return icdi_submit_urb(icdi, &icdi->auxreq, urb_timeout, NULL, 0);
}

static inline int icdi_detach(struct icdi_device *icdi)
{
	icdi->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	icdi->auxreq.bRequest = USB_DFU_DETACH;
	icdi->auxreq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->auxreq.wValue = icdi->dettmout > 5000? 5000 : icdi->dettmout;
	icdi->auxreq.wLength = 0;
	return icdi_submit_urb(icdi, &icdi->auxreq, urb_timeout, NULL, 0);
}

static inline int icdi_get_status(struct icdi_device *icdi)
{
	icdi->auxreq.bRequestType = USB_DFU_FUNC_UP;
	icdi->auxreq.bRequest = USB_DFU_GETSTATUS;
	icdi->auxreq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->auxreq.wValue = 0;
	icdi->auxreq.wLength = cpu_to_le16(6);
	return icdi_submit_urb(icdi, &icdi->auxreq, urb_timeout,
			&icdi->status, sizeof(icdi->status));
}

static inline int icdi_get_state(struct icdi_device *icdi)
{
	int retv;

	icdi->auxreq.bRequestType = USB_DFU_FUNC_UP;
	icdi->auxreq.bRequest = USB_DFU_GETSTATE;
	icdi->auxreq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->auxreq.wValue = 0;
	icdi->auxreq.wLength = cpu_to_le16(1);
	retv = icdi_submit_urb(icdi, &icdi->auxreq, urb_timeout,
			&icdi->state, sizeof(icdi->state));
	if (retv == 0)
		retv = icdi->state;
	return retv;
}

static inline int icdi_clear_status(struct icdi_device *icdi)
{
	icdi->auxreq.bRequestType = USB_DFU_FUNC_DOWN;
	icdi->auxreq.bRequest = USB_DFU_CLRSTATUS;
	icdi->auxreq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->auxreq.wValue = 0;
	icdi->auxreq.wLength = 0;
	return icdi_submit_urb(icdi, &icdi->auxreq, urb_timeout, NULL, 0);
}

static inline int icdi_finish_dnload(struct icdi_device *icdi)
{
	icdi->prireq.bRequestType = USB_DFU_FUNC_DOWN;
	icdi->prireq.bRequest = USB_DFU_DNLOAD;
	icdi->prireq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->prireq.wValue = 0;
	icdi->prireq.wLength = 0;
	return icdi_submit_urb(icdi, &icdi->prireq, urb_timeout, NULL, 0);
}

static inline int wmsec2int(unsigned char *wmsec)
{
	return (wmsec[2] << 16)|(wmsec[1] << 8) | wmsec[0];
}

static int icdi_wait_state(struct icdi_device *icdi, int state_mask)
{
	int count = 0, usb_resp, mwait;

	state_mask |= (1<<dfuERROR);
	do {
		usb_resp = icdi_get_status(icdi);
		if (usb_resp) {
			dev_err(&icdi->intf->dev, "Cannot get DFU status: " \
					"%d\n", usb_resp);
			return usb_resp;
		}
		if (state_mask & (1 << icdi->status.bState))
			break;
		mwait = wmsec2int(icdi->status.wmsec);
		msleep(mwait);
		count += 1;
	} while (count < 5);
	if (count == 5)
		dev_err(&icdi->intf->dev, "DFU Stalled\n");
	return icdi->status.bState;
}
static ssize_t abort_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct icdi_device *icdi;
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
	icdi = usb_get_intfdata(intf);
	icdi_abort(icdi);
	return count;
}

static ssize_t detach_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct icdi_device *icdi;
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
	icdi = usb_get_intfdata(interface);
	mutex_lock(&icdi->lock);
	resp = icdi_detach(icdi);
	if (resp && resp != -EPROTO) {
		dev_err(dev, "Cannot detach the DFU device: %d\n", resp);
		goto exit_10;
	}
	if ((icdi->cap & CAN_DETACH) == 0) {
		resp = icdi_get_state(icdi);
		if (resp != appDETACH) {
			dev_err(dev, "DFU device is not in appDETACH state: %d\n",
					resp);
			goto exit_10;
		}
		usb_reset_device(icdi->usbdev);
	}
exit_10:
	mutex_unlock(&icdi->lock);
	return count;
}

static ssize_t capbility_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *interface;
	int cap, download, upload, manifest, detach;

	interface = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(interface);
	cap = icdi->cap;
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
	struct icdi_device *icdi;
	struct usb_interface *interface;
	int resp, mwait, retv;

	interface = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(interface);
	mutex_lock(&icdi->lock);
	resp = icdi_get_status(icdi);
	mutex_unlock(&icdi->lock);
	if (resp == 0) {
		mwait = icdi->status.wmsec[2]<<16|
			icdi->status.wmsec[1]<<8|
			icdi->status.wmsec[0];
		retv = sprintf(buf, "Status: %hhd State: %hhd Wait: %d\n",
				icdi->status.bStatus,
				icdi->status.bState,
				mwait);
	} else {
		dev_err(dev, "Get DFU Status failed: %d\n", resp);
		retv = 0;
	}
	return retv;
}

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsz);
ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize);

static DEVICE_ATTR_WO(detach);
static DEVICE_ATTR_WO(abort);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(status);
static BIN_ATTR_RW(firmware, 0);

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr, 
		char *buf, loff_t offset, size_t bufsz)
{
	struct device *dev;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	int pos, usb_resp, remlen, icdi_state, blknum, state_mask;
	unsigned long fm_size;
	char *curbuf;

	if (unlikely(bufsz == 0))
		return bufsz;
	fm_size = binattr->size;
	if (fm_size == 0)
		fm_size = MAX_FMSIZE;

	dev = container_of(kobj, struct device, kobj);
	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if ((icdi->cap & CAN_UPLOAD) == 0) {
		dev_warn(dev, "has no upload capbility\n");
		return 0;
	}
	if ((bufsz % icdi->xfersize) != 0) {
		dev_err(&icdi->intf->dev, "Buffer size: %lu is not a " \
				"mutiple of DFU transfer size: %d\n",
				bufsz, icdi->xfersize);
		return -EINVAL;
	}
	pos = 0;
	curbuf = buf;
	remlen = offset + bufsz <= fm_size? bufsz : fm_size - offset;
	icdi->prireq.bRequestType = USB_DFU_FUNC_UP;
	icdi->prireq.bRequest = USB_DFU_UPLOAD;
	icdi->prireq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->prireq.wLength = cpu_to_le16(icdi->xfersize);
	blknum = offset / icdi->xfersize;

	mutex_lock(&icdi->lock);
	icdi_state = icdi_get_state(icdi);
	if (offset > 0 && icdi_state == dfuIDLE)
		goto exit_10;
	if ((offset == 0 && icdi_state != dfuIDLE) ||
			(offset > 0 && icdi_state != dfuUPLOAD_IDLE)) {
		dev_err(&icdi->intf->dev, "Incompatible State for " \
				"uploading: %d, Offset: %lld\n",
				icdi_state, offset);
		pos = -EPROTO;
		goto exit_10;
	}
	icdi_state = dfuUPLOAD_IDLE;
	state_mask = (1<<dfuUPLOAD_IDLE|1<<dfuIDLE);
	while (remlen > icdi->xfersize && offset + pos < fm_size &&
			icdi_state == dfuUPLOAD_IDLE) {
		icdi->prireq.wValue = cpu_to_le16(blknum);
		usb_resp = icdi_submit_urb(icdi, &icdi->prireq, urb_timeout,
				curbuf, icdi->xfersize);
		if (usb_resp) {
			dev_err(dev, "DFU upload error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(icdi->nxfer == 0);
		pos += icdi->nxfer;
		curbuf += icdi->nxfer;
		remlen -= icdi->nxfer;
		blknum += 1;
		icdi_state = icdi_wait_state(icdi, state_mask);
	}
	if (icdi_state == dfuIDLE) {
		bin_attr_firmware.size = offset + pos;
		goto exit_10;
	}
	if (unlikely(icdi_state != dfuUPLOAD_IDLE)) {
		dev_err(&icdi->intf->dev, "Cannot continue uploading, " \
				"inconsistent state: %d\n", icdi_state);
		goto exit_10;
	}
	if (offset + pos == fm_size) {
		icdi_abort(icdi);
		goto exit_10;
	}
	BUG_ON(remlen == 0);
	icdi->prireq.wValue = cpu_to_le16(blknum);
	icdi->prireq.wLength = cpu_to_le16(remlen);
	usb_resp = icdi_submit_urb(icdi, &icdi->prireq, urb_timeout,
			curbuf, remlen);
	if (usb_resp) {
		dev_err(dev, "DFU upload error: %d\n", usb_resp);
		pos = usb_resp;
		goto exit_10;
	}
	WARN_ON(icdi->nxfer == 0);
	pos += icdi->nxfer;
	icdi_state = icdi_wait_state(icdi, state_mask);
	if (offset + pos == fm_size && icdi_state == dfuUPLOAD_IDLE)
			icdi_abort(icdi);
	if (icdi_state == dfuIDLE)
		bin_attr_firmware.size = offset + pos;

exit_10:
	mutex_unlock(&icdi->lock);
	return pos;
}

ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize)
{
	struct device *dev;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	int icdi_state, pos, usb_resp, remlen, blknum, state_mask;
	unsigned long fm_size;
	char *curbuf;

	if (unlikely(bufsize == 0))
		return 0;
	dev = container_of(kobj, struct device, kobj);
	fm_size = binattr->size;
	if (fm_size == 0) {
		dev_err(dev, "The image size of DFU Device is unspecified. " \
				"Cannot program the device\n");
		return -EINVAL;
	}

	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if ((icdi->cap & CAN_DOWNLOAD) == 0) {
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
	icdi->prireq.bRequestType = USB_DFU_FUNC_DOWN;
	icdi->prireq.bRequest = USB_DFU_DNLOAD;
	icdi->prireq.wIndex = cpu_to_le16(icdi->intfnum);
	icdi->prireq.wLength = cpu_to_le16(icdi->xfersize);
	blknum = offset / icdi->xfersize;
	mutex_lock(&icdi->lock);
	icdi_state = icdi_get_state(icdi);
	if ((offset == 0 && icdi_state != dfuIDLE) ||
			(offset > 0 && icdi_state != dfuDNLOAD_IDLE)) {
		dev_err(dev, "Inconsistent DFU State, offset: %lld " \
				"State: %d\n", offset, icdi_state);
		pos = -EPROTO;
		goto exit_10;
	}
	icdi_state = dfuDNLOAD_IDLE;
	state_mask = (1 << dfuDNLOAD_IDLE);
	while (remlen > icdi->xfersize && offset + pos < fm_size &&
			icdi_state == dfuDNLOAD_IDLE) {
		icdi->prireq.wValue = cpu_to_le16(blknum);
		usb_resp = icdi_submit_urb(icdi, &icdi->prireq, urb_timeout,
				curbuf, icdi->xfersize);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(icdi->nxfer == 0);
		pos += icdi->nxfer;
		curbuf += icdi->nxfer;
		remlen -= icdi->nxfer;
		blknum += 1;
		icdi_state = icdi_wait_state(icdi, state_mask);
	}
	if (unlikely(icdi_state != dfuDNLOAD_IDLE)) {
		dev_err(&icdi->intf->dev, "Cannot continue downloading. " \
				"Invalid state: %d\n", icdi_state);
		goto exit_10;
	}
	if (offset + pos < fm_size) {
		BUG_ON(remlen == 0);
		icdi->prireq.wValue = cpu_to_le16(blknum);
		icdi->prireq.wLength = cpu_to_le16(remlen);
		usb_resp = icdi_submit_urb(icdi, &icdi->prireq, urb_timeout,
				curbuf, remlen);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		WARN_ON(icdi->nxfer == 0);
		pos += icdi->nxfer;
		icdi_state = icdi_wait_state(icdi, state_mask);
	}
	if (offset + pos == fm_size) {
		icdi->prireq.wValue = cpu_to_le16(blknum+1);
		icdi->prireq.wLength = 0;
		usb_resp = icdi_submit_urb(icdi, &icdi->prireq, urb_timeout,
				NULL, 0);
		if (usb_resp) {
			dev_err(dev, "DFU download error: %d\n", usb_resp);
			pos = usb_resp;
			goto exit_10;
		}
		state_mask = (1<<dfuIDLE)|(1<<dfuMANIFEST)|
			(1<<dfuMANIFEST_WAIT_RESET);
		icdi_state = icdi_wait_state(icdi, state_mask);
		if (icdi_state == dfuIDLE)
			goto exit_10;
		msleep(wmsec2int(icdi->status.wmsec)+1);
		icdi_state = icdi_wait_state(icdi, state_mask);
		if (icdi_state == dfuIDLE)
			goto exit_10;
		if (icdi_state == dfuMANIFEST_WAIT_RESET) {
			usb_reset_device(icdi->usbdev);
			goto exit_10;
		}
		if (icdi_state == dfuERROR) {
			icdi_clear_status(icdi);
			dev_warn(&icdi->intf->dev, "State changed to " \
					"dfuERROR. Error cleared\n");
		} else
			dev_warn(&icdi->intf->dev, "Unexpected State after" \
				       " firmware downloading\n");
	}

exit_10:
	mutex_unlock(&icdi->lock);
	return pos;
}

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

static DEVICE_ATTR_RW(fmsize); */
#define FLASH_BLOCK_SIZE 512
#define FLASH_ERASE_SIZE 1024
#define BUFSIZE (64 + 2*FLASH_BLOCK_SIZE)

static void icdi_urb_done(struct urb *urb)
{
	struct icdi_device *icdi;

	icdi = urb->context;
	icdi->resp = urb->status;
	icdi->nxfer = urb->actual_length;
	complete(&icdi->urbdone);
}

static void icdi_urb_timeout(struct icdi_device *icdi)
{
	usb_unlink_urb(icdi->urb);
	wait_for_completion(&icdi->urbdone);
}

static const char command_prefix[] = "$qRcmd,";

static ssize_t version_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *interface;
	int idx, i, retv, pos;
	uint8_t sum = 0;
	unsigned long jiff_wait;
	static const char command[] = "version";

	retv = 0;
	interface = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(interface);
	mutex_lock(&icdi->lock);
	icdi->buf = kmalloc(BUFSIZE, GFP_KERNEL);
	if (!icdi->buf) {
		retv = -ENOMEM;
		goto exit_10;
	}
	icdi->buflen = BUFSIZE;
	idx = sizeof(command_prefix) - 1;
	memcpy(icdi->buf, command_prefix, idx);
	for (i = 0; i < sizeof(command) - 1; i++)
		idx += sprintf(icdi->buf + idx, "%02x", command[i]);
	icdi->buf[idx] = 0;
	for (i = 1; i < idx; i++)
		sum += icdi->buf[i];
	idx += sprintf(icdi->buf + idx, "#%02x", sum);

	usb_fill_bulk_urb(icdi->urb, icdi->usbdev, icdi->pipe_out,
			icdi->buf, idx, icdi_urb_done, icdi);
	init_completion(&icdi->urbdone);
	icdi->resp = -255;
	icdi->nxfer = 0;
	retv = usb_submit_urb(icdi->urb, GFP_KERNEL);
	if (retv != 0) {
		dev_err(&icdi->intf->dev, "URB bulk write submit failed: %d\n", retv);
		goto exit_20;
	}
	jiff_wait = msecs_to_jiffies(urb_timeout);
	if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
		icdi_urb_timeout(icdi);
		dev_warn(&icdi->intf->dev, "URB bulk write operation timeout\n");
		retv = 0;
		goto exit_20;
	}
	retv = icdi->resp;
	if (unlikely(retv != 0))
		goto exit_20;

	pos = 0;
	icdi->buf[0] = '+';
	do {
		usb_fill_bulk_urb(icdi->urb, icdi->usbdev, icdi->pipe_in,
				icdi->buf + pos, icdi->buflen - pos,
				icdi_urb_done, icdi);
		init_completion(&icdi->urbdone);
		icdi->resp = -255;
		icdi->nxfer = 0;
		retv = usb_submit_urb(icdi->urb, GFP_KERNEL);
		if (retv != 0) {
			dev_err(&icdi->intf->dev, "URB bulk read submit failed: %d\n", retv);
			goto exit_20;
		}
		jiff_wait = msecs_to_jiffies(urb_timeout);
		if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
			icdi_urb_timeout(icdi);
			dev_warn(&icdi->intf->dev, "URB bulk read operation timeout\n");
			retv = 0;
			goto exit_20;
		}
		pos += icdi->nxfer;
	} while ((pos < 5 || icdi->buf[pos-3] != '#') && icdi->buf[0] == '+');
	if (icdi->buf[0] != '+') {
		dev_err(&icdi->intf->dev, "No response from command: %s\n", command);
		retv = 0;
	} else {
		retv = pos;
		memcpy(buf, icdi->buf, pos);
	}

exit_20:
	kfree(icdi->buf);
	icdi->buf = NULL;
exit_10:
	mutex_unlock(&icdi->lock);
	return retv;
}

static DEVICE_ATTR_RO(version);

static int icdi_create_attrs(struct icdi_device *icdi)
{
	int retv;

	icdi->attrs = 0;
	retv = device_create_file(&icdi->intf->dev, &dev_attr_version);
	if (unlikely(retv != 0))
		dev_warn(&icdi->intf->dev,
				"Cannot create sysfs file 'version': %d\n", retv);
	else
		icdi->version_attr = 1;
/*		retv = device_create_file(&icdi->intf->dev, &dev_attr_fmsize);
		if (unlikely(retv != 0))
			dev_warn(&icdi->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			icdi->fmsize_attr = 1;
		retv = device_create_file(&icdi->intf->dev, &dev_attr_abort);
		if (unlikely(retv != 0))
			dev_warn(&icdi->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			icdi->abort_attr = 1;
		retv = sysfs_create_bin_file(&icdi->intf->dev.kobj,
				&bin_attr_firmware);
		if (unlikely(retv != 0))
			dev_warn(&icdi->intf->dev,
					"Cannot create sysfs file %d\n", retv);
		else
			icdi->firmware_attr = 1;
	}
	retv = device_create_file(&icdi->intf->dev, &dev_attr_capbility);
	if (unlikely(retv != 0))
		dev_warn(&icdi->intf->dev, "Cannot create sysfs file %d\n",
				retv);
	else
		icdi->capbility_attr = 1; */
	return retv;
}

static void icdi_remove_attrs(struct icdi_device *icdi)
{
	if (icdi->version_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_version);
/*	if (icdi->firmware_attr)
		sysfs_remove_bin_file(&icdi->intf->dev.kobj, &bin_attr_firmware);
	if (icdi->abort_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_abort);
	if (icdi->fmsize_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_fmsize);
	if (icdi->status_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_status);
	if (icdi->capbility_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_capbility); */
}

static int icdi_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv, numpoints, i;
	unsigned int pntadr, pntattr, maxpkt_len;
	struct icdi_device *icdi;
	struct usb_host_endpoint *ep;
	struct usb_host_interface *host_intf;

	icdi = kmalloc(sizeof(struct icdi_device), GFP_KERNEL);
	if (!icdi)
		return -ENOMEM;
	retv = 0;
	icdi->usbdev = interface_to_usbdev(intf);
	icdi->intf = intf;
	host_intf = intf->cur_altsetting;
	numpoints = host_intf->desc.bNumEndpoints;
	icdi->pipe_in = -1;
	icdi->pipe_out = -1;
	for (ep = host_intf->endpoint, i = 0; i < numpoints; i++, ep++) {
		pntadr = ep->desc.bEndpointAddress;
		pntattr = ep->desc.bmAttributes;
		maxpkt_len = le16_to_cpu(ep->desc.wMaxPacketSize);
		if ((pntattr & USB_ENDPOINT_XFERTYPE_MASK) != 2)
			continue;
		if (pntadr & USB_DIR_IN)
			icdi->pipe_in = usb_rcvbulkpipe(icdi->usbdev, pntadr);
		else
			icdi->pipe_out = usb_sndbulkpipe(icdi->usbdev, pntadr);
	}
	if (icdi->pipe_in == -1 || icdi->pipe_out == -1) {
		dev_err(&intf->dev, "No bulk transfer endpoints\n");
		retv = -ENODEV;
		goto err_10;
	}

	icdi->intf = intf;
/*	icdi->cap = dfufdsc->attr;
	icdi->xfersize = le16_to_cpu(dfufdsc->xfersize);
	icdi->dettmout = dfufdsc->tmout;
	icdi->usbdev = interface_to_usbdev(intf);
	icdi->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	icdi->proto = intf->cur_altsetting->desc.bInterfaceProtocol;
	if (icdi->usbdev->bus->controller->dma_mask)
		icdi->dma = 1;
	else
		icdi->dma = 0;*/
	icdi->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!icdi->urb) {
		retv = -ENOMEM;
		goto err_10;
	}
	mutex_init(&icdi->lock);

        usb_set_intfdata(intf, icdi);
	icdi_create_attrs(icdi);
	dev_info(&icdi->intf->dev, "TI USB ICDI inserted\n");
	return retv;

err_10:
	kfree(icdi);
	return retv;
}

static void icdi_disconnect(struct usb_interface *intf)
{
	struct icdi_device *icdi;

	icdi = usb_get_intfdata(intf);
	mutex_lock(&icdi->lock);
	usb_set_intfdata(intf, NULL);
	icdi_remove_attrs(icdi);
	usb_free_urb(icdi->urb);
	mutex_unlock(&icdi->lock);
	kfree(icdi);
}

static struct usb_driver icdi_driver = {
	.name = MODULE_NAME,
	.probe = icdi_probe,
	.disconnect = icdi_disconnect,
	.id_table = icdi_ids,
};

static int __init usbicdi_init(void)
{
	int retv;

        retv = usb_register(&icdi_driver);
	if (retv) {
		pr_err("Cannot register USB DFU driver: %d\n", retv);
		return retv;
	}

        return 0;
}

static void __exit usbicdi_exit(void)
{
	usb_deregister(&icdi_driver);
}

module_init(usbicdi_init);
module_exit(usbicdi_exit);
