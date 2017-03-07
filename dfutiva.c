/*
 * usbtiva.c
 *
 * Copyright (c) 2017 Dashi Cao        <dscao999@hotmail.com, caods1@lenovo.com>
 *
 * USB Device Firmware Upgrade support for TI Tiva C series
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
#include "usbdfu.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("Tiva C USB DFU Driver");

#define USB_VENDOR_LUMINARY 0x1cbe
#define USB_PRODUCT_STELLARIS_DFU 0x0ff

static const struct usb_device_id dfu_ids[] = {
	{ USB_DFU_INTERFACE_INFO(USB_VENDOR_LUMINARY,
	  	USB_CLASS_APP_SPEC, USB_DFU_SUBCLASS, USB_DFU_PROTO_DFUMODE) },
	{ }
};
MODULE_DEVICE_TABLE(usb, dfu_ids);

static inline int dfu_abort(struct dfu_device *dfudev, struct dfu_control *ctrl)
{
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_ABORT;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	return dfu_submit_urb(dfudev, ctrl);
}

static inline int dfu_get_status(const struct dfu_device *dfudev,
					struct dfu_control *ctrl)
{
	ctrl->req.bRequestType = 0xa1;
	ctrl->req.bRequest = USB_DFU_GETSTATUS;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = cpu_to_le16(6);
	ctrl->pipe = usb_rcvctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = &ctrl->dfuStatus;
	ctrl->len = 6;
	return dfu_submit_urb(dfudev, ctrl);
}

static inline int dfu_get_state(const struct dfu_device *dfudev,
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
	retv = dfu_submit_urb(dfudev, ctrl);
	if (retv == 0)
		retv = ctrl->dfuState;
	return retv;
}

static inline int dfu_clr_status(const struct dfu_device *dfudev,
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
	return dfu_submit_urb(dfudev, ctrl);
}

static inline int dfu_finish_dnload(const struct dfu_device *dfudev,
					struct dfu_control *ctrl)
{
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wValue = 0;
	ctrl->req.wLength = 0;
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = NULL;
	ctrl->len = 0;
	return dfu_submit_urb(dfudev, ctrl);
}

static int dfu_open(struct inode *inode, struct file *filp)
{
	struct dfu_device *dfudev;
	int state, retv;
	struct dfu_control *ctrl;

	retv = 0;
	dfudev = container_of(inode->i_cdev, struct dfu_device, dfu_cdev);
	filp->private_data = dfudev;
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
	ctrl->urb = NULL;
	state = dfu_get_state(dfudev, ctrl);
	if (state != dfuIDLE) {
		dev_err(&dfudev->intf->dev, "Bad Initial State: %d\n", state);
		retv =  -EBUSY;
		goto err_20;
	}

	return retv;

err_20:
	kfree(dfudev->databuf);
err_10:
	mutex_unlock(&dfudev->dfulock);
	return retv;
}

static int dfu_release(struct inode *inode, struct file *filp)
{
	struct dfu_device *dfudev;
	struct dfu_control *stctrl;
	int retv;

	dfudev = filp->private_data;
	stctrl = dfudev->stctrl;
	stctrl->urb = NULL;
	retv = dfu_get_state(dfudev, stctrl);
	if (retv == dfuDNLOAD_IDLE)
		dfu_finish_dnload(dfudev, stctrl);
	else if (retv == dfuERROR)
		dfu_clr_status(dfudev, stctrl);
	else if (retv != dfuIDLE)
		dfu_abort(dfudev, stctrl);
	msleep(100);
	retv = dfu_get_state(dfudev, stctrl);
	if (retv != dfuIDLE)
		dev_err(&dfudev->intf->dev, "Need Reset! Stuck in State: %d\n",
				retv);
	kfree(dfudev->databuf);
	mutex_unlock(&dfudev->dfulock);
	return 0;
}

static ssize_t dfu_upload(struct file *filp, char __user *buff, size_t count,
			loff_t *f_pos)
{
	struct dfu_device *dfudev;
	int blknum, numb;
	struct dfu_control *opctrl, *stctrl;
	int dfust, dma, len;
	dma_addr_t dmabuf;
	struct device *ctrler;

	if (count <= 0)
		return 0;

	dfudev = filp->private_data;
	opctrl = dfudev->opctrl;
	stctrl = dfudev->stctrl;
	stctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!stctrl->urb)
		return -ENOMEM;
	numb = 0;
	dfust = dfu_get_state(dfudev, stctrl);
	if (dfust != dfuIDLE && dfust != dfuUPLOAD_IDLE) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		numb = -EINVAL;
		goto exit_10;
	}
	if (*f_pos != 0 && dfust == dfuIDLE)
		return 0;

	if (!access_ok(VERIFY_WRITE, buff, count)) {
		numb = -EFAULT;
		goto exit_10;
	}
	opctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->opctrl->urb) {
		numb = -ENOMEM;
		goto exit_10;
	}

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
		dmabuf = dma_map_single(ctrler, opctrl->buff, dfudev->xfersize,
					DMA_FROM_DEVICE);
		if (!dma_mapping_error(ctrler, dmabuf)) {
			dma = 1;
			opctrl->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
			opctrl->urb->transfer_dma = dmabuf;
		} else
			dev_warn(&dfudev->intf->dev,
					"Cannot map DMA address\n");
	}

	blknum = *f_pos / dfudev->xfersize;
	do {
		opctrl->req.wValue = cpu_to_le16(blknum);
		if (dfu_submit_urb(dfudev, opctrl) ||
				dfu_get_status(dfudev, stctrl))
			break;
		dfust = stctrl->dfuStatus.bState;
		if (dfust != dfuUPLOAD_IDLE && dfust != dfuIDLE) {
			dev_err(&dfudev->intf->dev,
				"Uploading failed. DFU State: %d\n", dfust);
			break;
		}
		len = ACCESS_ONCE(opctrl->nxfer);
		dma_sync_single_for_cpu(ctrler, dmabuf, len, DMA_FROM_DEVICE);
		if (copy_to_user(buff+numb, dfudev->databuf, len))
			break;
		numb += len;
		blknum++;
	} while (len == opctrl->len && numb < count);
	if (*f_pos != 0 || numb > 32)
		*f_pos += numb;

	if (dma)
		dma_unmap_single(ctrler, dmabuf, dfudev->xfersize,
				DMA_FROM_DEVICE);
	usb_free_urb(opctrl->urb);

exit_10:
	usb_free_urb(stctrl->urb);
	return numb;
}

static ssize_t dfu_dnload(struct file *filp, const char __user *buff,
			size_t count, loff_t *f_pos)
{
	struct dfu_device *dfudev;
	int blknum, numb;
	struct dfu_control *opctrl, *stctrl;
	int dfust, dma, len, lenrem, tmout;
	dma_addr_t dmabuf;
	struct device *ctrler;

	if (count <= 0)
		return 0;

	dfudev = filp->private_data;
	opctrl = dfudev->opctrl;
	stctrl = dfudev->stctrl;
	stctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!stctrl->urb)
		return -ENOMEM;
	numb = 0;
	dfust = dfu_get_state(dfudev, stctrl);
	if (dfust != dfuIDLE && dfust != dfuDNLOAD_IDLE) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		numb = -EINVAL;
		goto exit_10;
	}
	if (!access_ok(VERIFY_READ, buff, count)) {
		numb = -EFAULT;
		goto exit_10;
	}

	opctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dfudev->opctrl->urb) {
		numb = -ENOMEM;
		goto exit_10;
	}

	ctrler = dfudev->usbdev->bus->controller;
	dmabuf = ~0;
	dma = 0;
	opctrl->req.bRequestType = 0x21;
	opctrl->req.bRequest = USB_DFU_DNLOAD;
	opctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	opctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	opctrl->buff = dfudev->databuf;
	if (dfudev->dma) {
		dmabuf = dma_map_single(ctrler, opctrl->buff, dfudev->xfersize,
					DMA_TO_DEVICE);
		if (!dma_mapping_error(ctrler, dmabuf)) {
			dma = 1;
			opctrl->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
			opctrl->urb->transfer_dma = dmabuf;
		} else
			dev_warn(&dfudev->intf->dev,
					"Cannot map DMA address\n");
	}

	blknum = *f_pos / dfudev->xfersize;
	lenrem = count;
	do {
		opctrl->len = dfudev->xfersize > lenrem ? lenrem :
							dfudev->xfersize;
		if (copy_from_user(dfudev->databuf, buff+numb, opctrl->len))
			break;
		opctrl->req.wValue = cpu_to_le16(blknum);
		opctrl->req.wLength = cpu_to_le16(opctrl->len);
		dma_sync_single_for_device(ctrler, dmabuf, opctrl->len,
						DMA_TO_DEVICE);
		if (dfu_submit_urb(dfudev, opctrl) ||
				dfu_get_status(dfudev, stctrl))
			break;
		len = ACCESS_ONCE(opctrl->nxfer);
		numb += len;
		lenrem -= len;
		blknum++;
		while (stctrl->dfuStatus.bState == dfuDNLOAD_BUSY) {
			tmout = stctrl->dfuStatus.wmsec[0] |
					(stctrl->dfuStatus.wmsec[1] << 8) |
					(stctrl->dfuStatus.wmsec[2] << 16);
			msleep(tmout);
			if (dfu_get_status(dfudev, stctrl))
				break;
		}
		if (stctrl->dfuStatus.bState != dfuDNLOAD_IDLE &&
		    stctrl->dfuStatus.bState != dfuIDLE) {
			dev_err(&dfudev->intf->dev,
				"Downloading failed. DFU State: %d\n", dfust);
			break;
		}
	} while (len == opctrl->len && lenrem > 0);
	if (*f_pos != 0 || numb > 32)
 		*f_pos += numb;

	if (dma)
		dma_unmap_single(ctrler, dmabuf, dfudev->xfersize,
				DMA_FROM_DEVICE);
	usb_free_urb(opctrl->urb);

exit_10:
	usb_free_urb(stctrl->urb);
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
	struct dfu_device *dfudev;
	struct dfu_control *ctrl;
	void *bounce;
	int align16;

	if (count < 4 || count > 32)
		return count;

	dfudev = container_of(attr, struct dfu_device, tachattr);
	if (!mutex_trylock(&dfudev->dfulock)) {
		dev_err(&dfudev->intf->dev,
				"Cannot send command, device busy\n");
		return count;
	}
	align16 = (((count-1) >> 4) + 1) << 4;
	bounce = kmalloc(sizeof(struct dfu_control)+align16, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;
	memcpy(bounce, buf, count);
	
	ctrl = bounce + align16;
	ctrl->req.bRequestType = 0x21;
	ctrl->req.bRequest = USB_DFU_DNLOAD;
	ctrl->req.wValue = 0;
	ctrl->req.wIndex = cpu_to_le16(dfudev->intfnum);
	ctrl->req.wLength = cpu_to_le16(count);
	ctrl->pipe = usb_sndctrlpipe(dfudev->usbdev, 0);
	ctrl->buff = bounce;
	ctrl->len = count;
	ctrl->urb = NULL;
	if (dfu_submit_urb(dfudev, ctrl) || dfu_get_status(dfudev, ctrl))
		dev_err(&dfudev->intf->dev,
				"DFU command failed: %d, State: %d\n",
				(int)ctrl->dfuStatus.bStatus,
				(int)ctrl->dfuStatus.bState);
	else
		dev_info(&dfudev->intf->dev,
				"DFU commmand status: %d, State: %d\n",
				(int)ctrl->dfuStatus.bStatus,
				(int)ctrl->dfuStatus.bState);
	kfree(bounce);
	mutex_unlock(&dfudev->dfulock);
	return count;
}

static ssize_t dfu_attr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;

	dfudev = container_of(attr, struct dfu_device, attrattr);
	return sprintf(buf, "Download:%d Upload:%d Manifest:%d Detach:%d\n",
		dfudev->download, dfudev->upload, dfudev->manifest,
		dfudev->detach);
}

static ssize_t dfu_timeout_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;

	dfudev = container_of(attr, struct dfu_device, tmoutattr);
	return sprintf(buf, "%d\n", dfudev->dettmout);
}

static ssize_t dfu_xfersize_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;

	dfudev = container_of(attr, struct dfu_device, xsizeattr);
	return sprintf(buf, "%d\n", dfudev->xfersize);
}

static ssize_t dfu_state_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct dfu_device *dfudev;
	struct dfu_control *ctrl;
	int dfstat;

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return 0;
	dfudev = container_of(attr, struct dfu_device, statattr);
	ctrl->urb = NULL;
	dfstat = dfu_get_state(dfudev, ctrl);
	kfree(ctrl);
	return sprintf(buf, "%d\n", dfstat);
}

static ssize_t dfu_clear_cmd(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct dfu_control *ctrl;
	int dfust;

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return 0;
	dfudev = container_of(attr, struct dfu_device, abortattr);
	ctrl->urb = NULL;
	dfust = dfu_get_state(dfudev, ctrl);
	switch (dfust) {
	case dfuDNLOAD_IDLE:
	case dfuUPLOAD_IDLE:
		dfu_abort(dfudev, ctrl);
		break;
	case dfuERROR:
		dfu_clr_status(dfudev, ctrl);
		break;
	default:
		dev_warn(&dfudev->intf->dev, "Cannot clear, in state: %d\n",
				dfust);
		break;
	}
	kfree(ctrl);
	return count;
}

static int dfu_create_attrs(struct dfu_device *dfudev)
{
	int retv = 0;

	dfudev->tachattr.attr.name = "dfucmd";
	dfudev->tachattr.attr.mode = S_IWUSR;
	dfudev->tachattr.show = NULL;
	dfudev->tachattr.store = dfu_sndcmd;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->tachattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		return retv;
	}
	dfudev->attrattr.attr.name = "attr";
	dfudev->attrattr.attr.mode = S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->attrattr.show = dfu_attr_show;
	dfudev->attrattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->attrattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_10;
	}
	dfudev->tmoutattr.attr.name = "timeout";
	dfudev->tmoutattr.attr.mode = S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->tmoutattr.show = dfu_timeout_show;
	dfudev->tmoutattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->tmoutattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_20;
	}
	dfudev->xsizeattr.attr.name = "xfersize";
	dfudev->xsizeattr.attr.mode = S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->xsizeattr.show = dfu_xfersize_show;
	dfudev->xsizeattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->xsizeattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_30;
	}
	dfudev->statattr.attr.name = "state";
	dfudev->statattr.attr.mode = S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->statattr.show = dfu_state_show;
	dfudev->statattr.store = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->statattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_40;
	}
	dfudev->abortattr.attr.name = "clear";
	dfudev->abortattr.attr.mode = S_IWUSR;
	dfudev->abortattr.store = dfu_clear_cmd;
	dfudev->abortattr.show = NULL;
	retv = device_create_file(&dfudev->intf->dev, &dfudev->abortattr);
	if (retv != 0) {
		dev_err(&dfudev->intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_50;
	}

	return retv;

err_50:
	device_remove_file(&dfudev->intf->dev, &dfudev->statattr);
err_40:
	device_remove_file(&dfudev->intf->dev, &dfudev->xsizeattr);
err_30:
	device_remove_file(&dfudev->intf->dev, &dfudev->tmoutattr);
err_20:
	device_remove_file(&dfudev->intf->dev, &dfudev->attrattr);
err_10:
	device_remove_file(&dfudev->intf->dev, &dfudev->tachattr);
	return retv;
}

static void dfu_remove_attrs(struct dfu_device *dfudev)
{
	device_remove_file(&dfudev->intf->dev, &dfudev->abortattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->statattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->xsizeattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->tmoutattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->attrattr);
	device_remove_file(&dfudev->intf->dev, &dfudev->tachattr);
}

static int dfu_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int retv;
	struct dfu_device *dfudev;
	
	retv = dfu_prepare(&dfudev, intf, id);
	if (retv)
		return retv;

	dfudev->proto = 2;
	retv = dfu_create_attrs(dfudev);
	if (retv)
		goto err_10;

	cdev_init(&dfudev->dfu_cdev, &dfu_fops);
	dfudev->dfu_cdev.owner = THIS_MODULE;
	retv = cdev_add(&dfudev->dfu_cdev, dfudev->devnum, 1);
	if (retv) {
		dev_err(&dfudev->intf->dev, "Cannot add device: %d\n", retv);
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

	return retv;

err_30:
	cdev_del(&dfudev->dfu_cdev);
err_20:
	dfu_remove_attrs(dfudev);
err_10:
	dfu_cleanup(dfudev);
	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dfudev;

	dfudev = usb_get_intfdata(intf);
	device_destroy(dfu_class, dfudev->devnum);
	cdev_del(&dfudev->dfu_cdev);
	dfu_remove_attrs(dfudev);
	dfu_cleanup(dfudev);
}

static struct usb_driver dfu_driver = {
	.name = "dfutiva",
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
