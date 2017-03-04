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
	if (retv == dfuERROR)
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
	if (count % dfudev->xfersize != 0)
		return -EINVAL;
	opctrl = dfudev->opctrl;
	stctrl = dfudev->stctrl;
	stctrl->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!stctrl->urb)
		return -ENOMEM;
	numb = 0;
	dfust = dfu_get_state(dfudev, stctrl);
	if (*f_pos != 0 && dfust == dfuIDLE)
		goto exit_10;
	if ((*f_pos == 0 && dfust != dfuIDLE) ||
		(*f_pos != 0 && dfust != dfuUPLOAD_IDLE)) {
		dev_err(&dfudev->intf->dev, "Inconsistent State: %d\n", dfust);
		numb = -EINVAL;
		goto exit_10;
	}
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
	if ((*f_pos == 0 && dfust != dfuIDLE) ||
		(*f_pos != 0 && dfust != dfuDNLOAD_IDLE)) {
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
		dma_sync_single_for_device(ctrler, dmabuf, opctrl->len,
						DMA_TO_DEVICE);
		opctrl->req.wValue = cpu_to_le16(blknum);
		opctrl->req.wLength = cpu_to_le16(opctrl->len);
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
		if (stctrl->dfuStatus.bState != dfuDNLOAD_IDLE) {
			dev_err(&dfudev->intf->dev,
				"Uploading failed. DFU State: %d\n", dfust);
			break;
		}
	} while (len == opctrl->len && lenrem > 0);
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

static ssize_t dfu_switch(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dfu_device *dfudev;
	struct dfu_control *ctrl;

	dfudev = container_of(attr, struct dfu_device, actattr);

	ctrl = kmalloc(sizeof(struct dfu_control), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	if (count <= 0 || *buf != '-' || (*(buf+1) != '\n' && *(buf+1) != 0)) {
		dev_err(dev, "Invalid Command: %c\n", *buf);
		goto exit_10;
	}
	if (!dfudev->ftus.reset)
		dev_err(&dfudev->intf->dev, "Reset operation unsupported\n");

exit_10:
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
	
	retv = sprintf(buf, fmt, dfudev->attr, dfudev->dettmout,
			dfudev->xfersize);
	bst = dfu_get_state(dfudev, ctrl);
	retv += sprintf(buf+retv, "Current State: %d\n", bst);

	kfree(ctrl);
	return retv;
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

	dfudev->actattr.attr.name = "attach";
	dfudev->actattr.attr.mode = S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH;
	dfudev->actattr.show = dfu_show;
	dfudev->actattr.store = dfu_switch;
	usb_set_intfdata(intf, dfudev);
	retv = device_create_file(&intf->dev, &dfudev->actattr);
	if (retv != 0) {
		dev_err(&intf->dev, "Cannot create sysfs file %d\n", retv);
		goto err_10;
	}

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

	dfudev->feat = 0;

	return retv;

err_30:
	cdev_del(&dfudev->dfu_cdev);
err_20:
	device_remove_file(&intf->dev, &dfudev->actattr);
	usb_set_intfdata(intf, NULL);
err_10:
	dfu_cleanup(intf, dfudev);
	return retv;
}

static void dfu_disconnect(struct usb_interface *intf)
{
	struct dfu_device *dfudev;

	dfudev = usb_get_intfdata(intf);
	device_destroy(dfu_class, dfudev->devnum);
	cdev_del(&dfudev->dfu_cdev);
	device_remove_file(&intf->dev, &dfudev->actattr);
	usb_set_intfdata(intf, NULL);
	dfu_cleanup(intf, dfudev);
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
