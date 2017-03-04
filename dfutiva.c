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

static ssize_t dfu_upload(struct file *filp, char __user *buff, size_t count,
			loff_t *f_pos)
{
	struct dfu_device *dfudev;
	int blknum, retv, numb, nbytes;
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
			dev_err(&dfudev->intf->dev,
				"Uploading failed. DFU State: %d\n", dfust);
			break;
		}
		dma_sync_single_for_cpu(ctrler, dmabuf, opctrl->len, DMA_FROM_DEVICE);
		len = ACCESS_ONCE(opctrl->nxfer);
		numb += len;
		nbytes = copy_to_user(buff+numb, dfudev->databuf, len);
		blknum++;
	} while (len == opctrl->len && numb < count);
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
