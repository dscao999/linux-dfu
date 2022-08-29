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

#define MAX_FMSIZE	(0x7ful << 56)

struct icdi_device {
	struct mutex lock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct completion urbdone;
	struct urb *urb;
	unsigned char *buf;
	int intfnum;
	int pipe_in, pipe_out;
	int buflen;
	int inflen;
	volatile int resp, nxfer;
	unsigned int erase_size;
	unsigned int erase_offset;
	int partno;
	int in_debug;
	union {
		unsigned char attrs;
		struct {
			unsigned int firmware_attr:1;
			unsigned int fmsize_attr:1;
			unsigned int version_attr:1;
			unsigned int debug_attr:1;
		};
	};
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dashi Cao");
MODULE_DESCRIPTION("TI USB ICDI Driver");


static int urb_timeout = 200; /* milliseconds */
module_param(urb_timeout, int, 0644);
MODULE_PARM_DESC(urb_timeout, "USB urb completion timeout. "
	"Default: 200 milliseconds.");

static const uint32_t FP_CTRL	= 0xe0002000;
static const uint32_t DID0	= 0x400fe000;
static const uint32_t DID1	= 0x400fe004;
static const uint32_t DHCSR	= 0xe000edf0;
static const uint32_t CPUID	= 0xe000ed00;
static const uint32_t ICTR	= 0xE000E004;
static const uint32_t FMA	= 0x400fd000;

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

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize);
ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize);
static ssize_t fmsize_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t fmsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t buflen);
static ssize_t version_show(struct device *dev,
                struct device_attribute *attr, char *buf);
static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t buflen);

static BIN_ATTR_RW(firmware, 0);
static DEVICE_ATTR_RW(fmsize);
static DEVICE_ATTR_RW(debug);
static DEVICE_ATTR_RO(version);

static ssize_t fmsize_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", bin_attr_firmware.size);
}

static ssize_t fmsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t buflen)
{
	char *tmpbuf, *endchr;

	if (bin_attr_firmware.size != 0) {
		dev_warn(dev, "Firmware Size already set: %lu. Unable to modify\n", bin_attr_firmware.size);
		return buflen;
	}
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

static inline int byte2hexstr(const unsigned char *bytes, int len,
		char *buf, int buflen)
{
	char *curchr;
	const unsigned char *curbyt;
	unsigned char nibh, nibl;

	if (buflen & 1)
		buflen -= 1;
	curchr = buf;
	for (curbyt = bytes; curbyt < bytes + len && curchr < buf + buflen; curbyt++) {
		nibh = (*curbyt) >> 4;
		nibl = (*curbyt) & 0x0f;
		*curchr++ = nibh > 9? 'a' + nibh - 10 : '0' + nibh;
		*curchr++ = nibl > 9? 'a' + nibl - 10 : '0' + nibl;
	}
	return curchr - buf;
}

static inline unsigned char val2hex(unsigned char val)
{
	unsigned char nib = val & 0x0f;
	return nib > 9? 'a' + nib - 10 : '0' + nib;
}

static inline unsigned char hex2val(char hex)
{
	unsigned char v = 0;

	if (hex >= 'a' && hex <= 'f')
		v = hex - 'a' + 10;
	else if (hex >= 'A' && hex <= 'F')
		v = hex - 'A' + 10;
	else if (hex >= '0' && hex <= '9')
		v = hex - '0';
	return v;
}

static inline int hexstr2byte(const char *hexstr, int len, unsigned char *buf, int buflen)
{
	const char *hex;
	unsigned char *curbyt, nibh, nibl;

	if (len & 1)
		len -= 1;
	curbyt = buf;
	for (hex = hexstr; hex < hexstr + len && curbyt < buf + buflen; curbyt++) {
		nibh = hex2val(*hex++);
		nibl = hex2val(*hex++);
		*curbyt = (nibh << 4) | nibl;
	}
	return curbyt - buf;
}

static inline int append_check_sum(unsigned char *buf, int len, int buflen)
{
	unsigned char sum = 0;
	int i, pos;

	for (i = 1; i < len; i++)
		sum += buf[i];
	pos = len;
	buf[pos++] = '#';
	pos += byte2hexstr(&sum, 1, buf+pos, buflen - pos);
	return pos;
}

static void dump_response(struct icdi_device *icdi, int reslen)
{
	unsigned char *buf;
	int len;

	if (reslen <= 0)
		return;

	icdi->buf[reslen] = 0;
	dev_info(&icdi->intf->dev, "Response length %d: %s\n", reslen, icdi->buf);
	buf = kmalloc(2*reslen+1, GFP_KERNEL);
	if (!buf) {
		dev_warn(&icdi->intf->dev, "Out of Memory when dumping response");
		return;
	}
	len = byte2hexstr(icdi->buf, reslen, buf, 2*reslen);
	buf[len] = 0;
	dev_info(&icdi->intf->dev, "Raw Response: %s\n", buf);
	kfree(buf);
}

static int usb_sndrcv(struct icdi_device *icdi)
{
	int retv, pos, len;
	unsigned long jiff_wait;
	char command[32];
	unsigned char *curchr, sum = 0, check;

	pos = sizeof(command) - 1;
	len = icdi->inflen > pos? pos : icdi->inflen;
	memcpy(command, icdi->buf, len);
	command[len] = 0;
	usb_fill_bulk_urb(icdi->urb, icdi->usbdev, icdi->pipe_out,
			icdi->buf, icdi->inflen, icdi_urb_done, icdi);
	init_completion(&icdi->urbdone);
	icdi->resp = -255;
	icdi->nxfer = 0;
	retv = usb_submit_urb(icdi->urb, GFP_KERNEL);
	if (unlikely(retv != 0)) {
		dev_err(&icdi->intf->dev, "URB bulk write submit failed: %d, command: %s\n", retv, command);
		return retv;
	}
	jiff_wait = msecs_to_jiffies(urb_timeout);
	if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
		icdi_urb_timeout(icdi);
		dev_warn(&icdi->intf->dev, "URB bulk write operation timeout, command: %s\n", command);
	}
	retv = icdi->resp;
	if (unlikely(retv < 0))
		return retv;

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
		if (unlikely(retv < 0)) {
			dev_err(&icdi->intf->dev, "URB bulk read submit failed: %d, command: %s\n", retv, command);
			return retv;
		}
		jiff_wait = msecs_to_jiffies(urb_timeout);
		if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
			icdi_urb_timeout(icdi);
			dev_warn(&icdi->intf->dev, "URB bulk read operation timeout, command %s\n", command);
		}
		retv = icdi->resp;
		if (unlikely(retv < 0)) {
			dev_err(&icdi->intf->dev, "URB bulk read failed: %d, command: %s\n", retv, command);
			return retv;
		}
		pos += icdi->nxfer;
	} while ((pos < 3 || icdi->buf[pos-3] != '#') && icdi->buf[0] == '+');
	icdi->buf[pos] = 0;
	if (memcmp(icdi->buf, "+$OK:", 5) == 0)
		len = 5;
	else if (memcmp(icdi->buf, "+$E07:", 6) == 0)
		len = 6;
	else
		len = 2;
	for (curchr = icdi->buf+len; *curchr != '#' && curchr < icdi->buf + pos;
			curchr++)
		sum+= *curchr;
	check = (hex2val(icdi->buf[pos-2]) << 4) | hex2val(icdi->buf[pos-1]);
	if ((sum - check) != 0)
		dev_info(&icdi->intf->dev, "Raw response checksum error. " \
				"check sum: %02hhx, retv: %x\n", sum, retv);
	if (icdi->buf[0] != '+')
		dev_err(&icdi->intf->dev, "No response from command: %s\n",
				command);

	return pos;
}

static const char qRcmd[] = "$qRcmd,";
static const char qSupported[] = "$qSupported";
static const char qmark[] = "$?";

static int qRcmd_setup(unsigned char *buf, int buflen, const char *arg, int arglen)
{
	int len;

	len = sizeof(qRcmd) - 1;
	memcpy(buf, qRcmd, len);
	len += byte2hexstr(arg, arglen, buf+len, buflen - len);
	len = append_check_sum(buf, len, buflen);
	return len;
}

static inline void uint2hexstr(unsigned int val, char *hexs)
{
	unsigned char bytes[4];

	bytes[0] = (val >> 24) & 0x0ff;
	bytes[1] = (val >> 16) & 0x0ff;
	bytes[2] = (val >> 8) & 0x0ff;
	bytes[3] = val & 0x0ff;
	hexs[0] = val2hex(bytes[0]>>4);
	hexs[1] = val2hex(bytes[0] & 0x0f);
	hexs[2] = val2hex(bytes[1]>>4);
	hexs[3] = val2hex(bytes[1] & 0x0f);
	hexs[4] = val2hex(bytes[2]>>4);
	hexs[5] = val2hex(bytes[2] & 0x0f);
	hexs[6] = val2hex(bytes[3]>>4);
	hexs[7] = val2hex(bytes[3] & 0x0f);
}

static unsigned int mem_read(struct icdi_device *icdi, unsigned int addr)
{
	static const char memr[] = "$x,4";
	int len;
       	unsigned int val;

	val = 0xffffffff;
	memcpy(icdi->buf, memr, 2);
	uint2hexstr(addr, icdi->buf+2);
	memcpy(icdi->buf+10, memr+2, 2);
	icdi->inflen = append_check_sum(icdi->buf, 12, icdi->buflen);
	icdi->buf[icdi->inflen] = 0;
	len = usb_sndrcv(icdi);
	if (len == 12)
		val = icdi->buf[5]|(icdi->buf[6]<<8)|(icdi->buf[7]<<16)|
			(icdi->buf[8]<<24);
	return val;
}

static void mem_write(struct icdi_device *icdi, unsigned int addr, unsigned int val)
{
	static const char memw[] = "$X,4:";

	memcpy(icdi->buf, memw, 2);
	uint2hexstr(addr, icdi->buf+2);
	memcpy(icdi->buf+10, memw+2, 3);
	uint2hexstr(val, icdi->buf+13);
	icdi->inflen = append_check_sum(icdi->buf, 21, icdi->buflen);
	usb_sndrcv(icdi);
}

static int stop_debug(struct icdi_device *icdi, int firmware)
{
	int len;
	static const char debug_hreset[] = "debug hreset";
	static const char restore_vector[] = "set vectorcatch 0";
	static const char debug_disable[] = "debug disable";

	if (icdi->in_debug == 0)
		return 0;
	if (firmware) {
		icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen,
				restore_vector, sizeof(restore_vector) - 1);
		len = usb_sndrcv(icdi);
		if (unlikely(len < 0)) {
			dev_err(&icdi->intf->dev, "debug vectorcatch 0 failed\n");
			return len;
		}

		icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen,
				debug_hreset, sizeof(debug_hreset) - 1);
		len = usb_sndrcv(icdi);
		if (unlikely(len < 0)) {
			dev_err(&icdi->intf->dev, "debug hreset failed\n");
			return len;
		}
	}

	icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen,
			debug_disable, sizeof(debug_disable) - 1);
	len = usb_sndrcv(icdi);
	if (unlikely(len <= 0)) {
		dev_err(&icdi->intf->dev, "debug disable failed\n");
		return len;
	}
	icdi->in_debug = 0;
	return 0;
}

static int start_debug(struct icdi_device *icdi, int firmware)
{
	int len, retv;
	unsigned int val;
	static const char debug_clock[] = "debug clock \0";
	static const char debug_sreset[] = "debug sreset";

	if (icdi->in_debug)
		return 0;
	retv = -1;
	icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen,
			debug_clock, sizeof(debug_clock) - 1);
	len = usb_sndrcv(icdi);
	if (unlikely(len < 0)) {
		dev_err(&icdi->intf->dev, "debug clock failed\n");
		return retv;
	}
	len = sizeof(qSupported) - 1;
	memcpy(icdi->buf, qSupported, len);
	icdi->inflen = append_check_sum(icdi->buf, len, icdi->buflen);
	len = usb_sndrcv(icdi);
	if (unlikely(len < 0)) {
		dev_err(&icdi->intf->dev, "qSupported failed\n");
		return retv;
	}
	len = sizeof(qmark) - 1;
	memcpy(icdi->buf, qmark, len);
	icdi->inflen = append_check_sum(icdi->buf, len, icdi->buflen);
	len = usb_sndrcv(icdi);
	if (unlikely(len < 0)) {
		dev_err(&icdi->intf->dev, "question mark failed\n");
		return retv;
	}
	if (firmware) {
		icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen,
				debug_sreset, sizeof(debug_sreset) - 1);
		len = usb_sndrcv(icdi);
		if (unlikely(len < 0)) {
			dev_err(&icdi->intf->dev, "debug sreset failed\n");
			return retv;
		}
	}
	val = mem_read(icdi, DHCSR);
	if (val != 0x00030003)
		dev_warn(&icdi->intf->dev, "Maybe not in debug state\n");
	icdi->in_debug = 1;
	return 0;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *intf;
	int len;
	static const char in_debug[] = "in debug\n";
	static const char free_run[] = "free run\n";

	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if (icdi->in_debug) {
		len = sizeof(in_debug);
		memcpy(buf, in_debug, len);
	} else {
		len = sizeof(free_run);
		memcpy(buf, free_run, len);
	}
	return len;
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t buflen)
{
	int max_cmdlen, len, cmdlen, retv;
	char cmd[16];
	struct usb_interface *intf;
	struct icdi_device *icdi;
	static const char enter_debug[] = "-->debug<--";
	static const char leave_debug[] = "<--debug-->";

	cmdlen = sizeof(enter_debug) - 1;
	max_cmdlen = sizeof(cmd) - 1;
	len = max_cmdlen < buflen? max_cmdlen : buflen;
	memcpy(cmd, buf, len);
	cmd[len] = 0;
	if (buflen < cmdlen || buflen > max_cmdlen) {
		dev_info(dev, "Invalid Command length: %s\n", cmd);
		return -EINVAL;
	}

	retv = buflen;
	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	mutex_lock(&icdi->lock);
	icdi->buflen = 128;
	icdi->buf = kmalloc(icdi->buflen, GFP_KERNEL);
	if (unlikely(!icdi->buf)) {
		dev_err(dev, "Out of Memory\n");
		retv = -ENOMEM;
		goto exit_10;
	}
	if (memcmp(enter_debug, buf, cmdlen) == 0 && icdi->in_debug == 0) {
		retv = start_debug(icdi, 1);
		if (retv!= 0)
			dev_err(dev, "Cannot enter into debug state\n");
		else {
			mem_write(icdi, FMA, 0);
			retv = buflen;
		}
	} else if (memcmp(leave_debug, buf, cmdlen) == 0 && icdi->in_debug) {
		retv = stop_debug(icdi, 1);
		if (retv!= 0)
			dev_err(dev, "Cannot leave debug state\n");
		else
			retv = buflen;
	} else {
		dev_info(dev, "Invalid Command: %s\n", cmd);
		retv = -EINVAL;
	}
	kfree(icdi->buf);

exit_10:
	icdi->buflen = 0;
	mutex_unlock(&icdi->lock);
	return retv;
}

static int get_erase_size(struct icdi_device *icdi)
{
	int retv;
	unsigned int val;

	icdi->buflen = 1024;
	icdi->buf = kmalloc(1024, GFP_KERNEL);
	if (unlikely(!icdi->buf)) {
		icdi->erase_size = 1024;
		dev_warn(&icdi->intf->dev, "Out of Memory. Erase Block Size " \
				"set to %d\n", icdi->erase_size);
		icdi->buflen = 0;
		return -ENOMEM;
	}

	retv = start_debug(icdi, 0);
	if (unlikely(retv != 0)) {
		icdi->erase_size = 4096;
		dev_warn(&icdi->intf->dev, "Cannot enter into debug state: " \
				"%d\n", retv);
		goto exit_10;
	}
	val = mem_read(icdi, DID1);
	icdi->partno = (val >> 16) & 0x0ff;
	switch(icdi->partno) {
	case 0x2d:
		icdi->erase_size = 16384;
		bin_attr_firmware.size = 1048576;
		break;
	case 0xa1:
		icdi->erase_size = 1024;
		bin_attr_firmware.size = 262144;
		break;
	default:
		icdi->erase_size = 4096;
	}
	stop_debug(icdi, 0);

exit_10:
	kfree(icdi->buf);
	icdi->buf = NULL;
	icdi->buflen = 0;
	return icdi->erase_size;
}

#define FLASH_READ_SIZE	256

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr, 
		char *buf, loff_t offset, size_t bufsize)
{
	struct device *dev;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	int retv, len, xferlen, rdlen, remlen;
	unsigned long fm_size;
	unsigned char *curbuf, *dst, *src, c;

	if (unlikely(bufsize == 0))
		return bufsize;
	dev = container_of(kobj, struct device, kobj);
	if (unlikely(bufsize % FLASH_READ_SIZE) != 0) {
		dev_err(&icdi->intf->dev, "Read Buffer Size %lu is not a " \
				"multiple of %d\n", bufsize, FLASH_READ_SIZE);
		return -EINVAL;
	}
	fm_size = binattr->size;
	if (fm_size == 0)
		fm_size = MAX_FMSIZE;
	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if (icdi->in_debug == 0) {
		dev_err(dev, "Device not in debug state\n");
		return -ENODATA;
	}

	retv = 0;
	mutex_lock(&icdi->lock);
	remlen = offset + bufsize > fm_size? fm_size - offset : bufsize;
	icdi->buflen = 64 + 2 * icdi->erase_size;
	icdi->buf = kmalloc(icdi->buflen, GFP_KERNEL);
	if (unlikely(!icdi->buf)) {
		dev_err(&icdi->intf->dev, "Out of Memory\n");
		retv = -ENOMEM;
		goto exit_10;
	}
	if (offset >= fm_size)
		goto exit_20;

	do {
		rdlen = icdi->erase_size  < remlen? icdi->erase_size : remlen;
		curbuf = icdi->buf;
		*curbuf++ = '$';
		*curbuf++ = 'x';
		uint2hexstr(offset+retv, curbuf);
		curbuf += 8;
		*curbuf++ = ',';
		uint2hexstr(rdlen, curbuf);
		icdi->inflen = append_check_sum(icdi->buf, 19, icdi->buflen-19);
		len = usb_sndrcv(icdi);
		if (unlikely(len < 0) || memcmp(icdi->buf, "+$OK:", 5) != 0) {
			dev_err(&icdi->intf->dev, "Flash Dump failed at " \
					"%08llx, length: %d\n", offset + retv,
					rdlen);
			if (len > 0)
				dump_response(icdi, len);
			goto exit_20;
		}
		dst = buf + retv;
		src = icdi->buf+5;
		xferlen = 0;
		while (src - icdi->buf < len - 3 && xferlen < rdlen) {
			c = *src++;
			if (c == '}')
				c = (*src++) ^ 0x20;
			*dst++ = c;
			xferlen += 1;
		}
		if (xferlen != rdlen)
			dev_warn(dev, "Offset: %lld, read length: %d, actual " \
					"transfer: %d\n", offset + retv, rdlen,
					xferlen);
		retv += xferlen;
		remlen -= xferlen;
	} while (remlen > 0);

exit_20:
	kfree(icdi->buf);
	icdi->buf = NULL;
exit_10:
	icdi->buflen = 0;
	mutex_unlock(&icdi->lock);
	return retv;
}

ssize_t firmware_write(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr,
		char *buf, loff_t offset, size_t bufsize)
{
	struct device *dev;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	unsigned long fm_size;
	int remlen, wtlen, len, retv, i;
	unsigned char *curchr, c;
	static const char flash_erase[] = "$vFlashErase:";
	static const char flash_write[] = "$vFlashWrite:";

	dev = container_of(kobj, struct device, kobj);
	if (bufsize % FLASH_READ_SIZE) {
		dev_err(dev, "The flash programming size %lu is not a mutiple" \
			       " of %d\n", bufsize, FLASH_READ_SIZE);
		return -EINVAL;
	}
	fm_size = binattr->size;
	if (fm_size == 0) {
		fm_size = MAX_FMSIZE;
		dev_warn(dev, "The flash size of the Device is unspecified");
	}
	if (unlikely(bufsize == 0))
		return 0;
	if (offset >= fm_size) {
		dev_err(dev, "Cannot program past the flash size. " \
				"offset: %lld, size: %lu\n", offset, fm_size);
		return -ENXIO;
	}

	retv = 0;
	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if (icdi->in_debug == 0) {
		dev_err(dev, "Device not in debug state\n");
		return -EREMOTEIO;
	}
	remlen = offset + bufsize < fm_size? bufsize : fm_size - offset;
	icdi->buflen = 64 + 2 * icdi->erase_size;
	icdi->buf = kmalloc(icdi->buflen, GFP_KERNEL);
	if (unlikely(!icdi->buf)) {
		dev_err(dev, "Out of Memory\n");
		icdi->buflen = 0;
		return -ENOMEM;
	}
	icdi->erase_offset = 0;
	mutex_lock(&icdi->lock);
	while (remlen > icdi->erase_offset) {
		len = sizeof(flash_erase) - 1;
		memcpy(icdi->buf, flash_erase, len);
		uint2hexstr(offset+icdi->erase_offset, icdi->buf + len);
		len += 8;
		icdi->buf[len++] = ',';
		uint2hexstr(icdi->erase_size, icdi->buf + len);
		len += 8;
		icdi->inflen = append_check_sum(icdi->buf, len, icdi->buflen);
		len = usb_sndrcv(icdi);
		if (len < 0 || memcmp(icdi->buf, "+$OK", 4) != 0) {
			dev_err(dev, "Unable to erase. Offset: %u, size: %u\n", icdi->erase_offset, icdi->erase_size);
			if (len > 0)
				dump_response(icdi, len);
			goto exit_20;
		}
		icdi->erase_offset += icdi->erase_size;
	}

	do {
		wtlen = icdi->erase_size > remlen? remlen : icdi->erase_size;
		len = sizeof(flash_write) - 1;
		memcpy(icdi->buf, flash_write, len);
		uint2hexstr(offset+retv, icdi->buf + len);
		len += 8;
		icdi->buf[len++] = ':';
		curchr = icdi->buf + len;
		for (i = retv; i < retv+wtlen; i++) {
			c = buf[i];
			if (c == '#' || c == '$' || c == '}') {
				*curchr++ = '}';
				c ^= 0x20;
			}
			*curchr++ = c;
		}
		len = (curchr - icdi->buf);
		icdi->inflen = append_check_sum(icdi->buf, len, icdi->buflen);
		len = usb_sndrcv(icdi);
		if (len < 0 || memcmp(icdi->buf, "+$OK", 4) != 0) {
			dev_err(dev, "Cannot program the flash. Offset: %lld, size: %d\n", offset+retv, wtlen);
			if (len > 0)
				dump_response(icdi, len);
			goto exit_20;
		}
		retv += wtlen;
		remlen -= wtlen;
	} while (remlen > 0);


exit_20:
	mutex_unlock(&icdi->lock);
	kfree(icdi->buf);
	icdi->buflen = 0;
	return retv;
}

static ssize_t version_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *interface;
	int len, retv;
	static const char version[] = "version";

	retv = 0;
	interface = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(interface);
	mutex_lock(&icdi->lock);
	icdi->buflen = icdi->erase_size*2 + 64;
	icdi->buf = kmalloc(icdi->buflen, GFP_KERNEL);
	if (!icdi->buf) {
		retv = -ENOMEM;
		goto exit_10;
	}

	icdi->inflen = qRcmd_setup(icdi->buf, icdi->buflen, version, sizeof(version) - 1);
	len = usb_sndrcv(icdi);
	if (len > 5)
		retv = hexstr2byte(icdi->buf+2, len - 5, buf, 4096);
	else {
		dev_err(&icdi->intf->dev, "Command 'version' failed\n");
		retv = len;
		if (retv > 0)
			memcpy(buf, icdi->buf, retv);
	}
	kfree(icdi->buf);
	icdi->buf = NULL;

exit_10:
	icdi->buflen = 0;
	mutex_unlock(&icdi->lock);
	return retv;
}

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
	retv = device_create_file(&icdi->intf->dev, &dev_attr_fmsize);
	if (unlikely(retv != 0))
		dev_warn(&icdi->intf->dev,
				"Cannot create sysfs file 'fmsize' %d\n", retv);
	else
		icdi->fmsize_attr = 1;
	retv = device_create_file(&icdi->intf->dev, &dev_attr_debug);
	if (unlikely(retv != 0))
		dev_warn(&icdi->intf->dev,
				"Cannot create sysfs file 'debug' %d\n", retv);
	else
		icdi->debug_attr = 1;
	retv = sysfs_create_bin_file(&icdi->intf->dev.kobj,
			&bin_attr_firmware);
	if (unlikely(retv != 0))
		dev_warn(&icdi->intf->dev,
				"Cannot create sysfs file %d\n", retv);
	else
		icdi->firmware_attr = 1;
/*	}
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
	if (icdi->fmsize_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_fmsize);
	if (icdi->debug_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_debug);
	if (icdi->firmware_attr)
		sysfs_remove_bin_file(&icdi->intf->dev.kobj, &bin_attr_firmware);
/*	if (icdi->abort_attr)
		device_remove_file(&icdi->intf->dev, &dev_attr_abort);
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
		if ((pntattr & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_BULK)
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
	icdi->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!icdi->urb) {
		retv = -ENOMEM;
		goto err_10;
	}
	mutex_init(&icdi->lock);
        usb_set_intfdata(intf, icdi);
	get_erase_size(icdi);
	icdi->in_debug = 0;
	icdi_create_attrs(icdi);
	dev_info(&icdi->intf->dev, "TI USB ICDI board '%02X' inserted. Erase Size: %d\n", icdi->partno, icdi->erase_size);
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
