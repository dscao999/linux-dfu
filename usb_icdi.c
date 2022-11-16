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

struct flash_block {
	unsigned int offset;
	unsigned int nxtpos;
	unsigned char *block;
};

struct icdi_device {
	struct mutex lock;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct completion urbdone;
	struct urb *urb;
	int intfnum;
	int pipe_in, pipe_out;
	volatile int resp, nxfer;
	unsigned int erase_size;
	int partno;
	struct flash_block flash;
	union {
		unsigned char attrs;
		struct {
			unsigned int firmware_attr:1;
			unsigned int fmsize_attr:1;
			unsigned int version_attr:1;
			unsigned int debug_attr:1;
			unsigned int in_debug:1;
			unsigned int stalled:1;
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
static const uint32_t SYSPROP	= 0x400fe14c;
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

static inline unsigned int byte2word(unsigned char *byt)
{
	return (*byt)|((*(byt+1))<<8)|(*((byt+2))<<16)| ((*(byt+3))<<24);
}

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

static void dump_response(struct device *dev, char *urbuf, int reslen)
{
	unsigned char *buf;
	int len;

	if (reslen <= 0)
		return;

	urbuf[reslen] = 0;
	dev_info(dev, "Response length %d: '%s'\n", reslen, urbuf);
	buf = kmalloc(2*reslen+1, GFP_KERNEL);
	if (!buf) {
		dev_warn(dev, "Out of Memory when dumping response");
		return;
	}
	len = byte2hexstr(urbuf, reslen, buf, 2*reslen);
	buf[len] = 0;
	dev_info(dev, "Raw Response: %s\n", buf);
	kfree(buf);
}

static int usb_send(struct icdi_device *icdi, char *urbuf, int inflen)
{
	int retv = 0;
	unsigned long jiff_wait;

	usb_fill_bulk_urb(icdi->urb, icdi->usbdev, icdi->pipe_out,
			urbuf, inflen, icdi_urb_done, icdi);
	init_completion(&icdi->urbdone);
	icdi->resp = -255;
	icdi->nxfer = 0;
	retv = usb_submit_urb(icdi->urb, GFP_KERNEL);
	if (unlikely(retv != 0)) {
		dev_err(&icdi->intf->dev, "URB bulk write submit failed: %d\n", retv);
		return retv;
	}
	jiff_wait = msecs_to_jiffies(urb_timeout);
	if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
		icdi_urb_timeout(icdi);
		dev_warn(&icdi->intf->dev, "URB bulk write operation timeout\n");
	}
	retv = icdi->resp;
	if (unlikely(retv < 0))
		dev_err(&icdi->intf->dev, "URB bulk write operation failed: %d\n", retv);
	return retv;
}

static int usb_recv(struct icdi_device *icdi, char *urbuf, int buflen)
{
	int retv = 0;
	unsigned long jiff_wait;

	usb_fill_bulk_urb(icdi->urb, icdi->usbdev, icdi->pipe_in,
			urbuf, buflen, icdi_urb_done, icdi);
	init_completion(&icdi->urbdone);
	icdi->resp = -255;
	icdi->nxfer = 0;
	retv = usb_submit_urb(icdi->urb, GFP_KERNEL);
	if (unlikely(retv < 0)) {
		dev_err(&icdi->intf->dev, "URB bulk read submit failed: %d\n", retv);
		return retv;
	}
	jiff_wait = msecs_to_jiffies(urb_timeout);
	if (!wait_for_completion_timeout(&icdi->urbdone, jiff_wait)) {
		icdi_urb_timeout(icdi);
		dev_warn(&icdi->intf->dev, "URB bulk read operation timeout\n");
	}
	retv = icdi->resp;
	if (unlikely(retv < 0))
		dev_err(&icdi->intf->dev, "URB bulk read failed: %d\n", retv);
	return retv;
}

static int do_usb_sndrcv(struct icdi_device *icdi, char *urbuf, int inflen,
		int buflen)
{
	int retv, pos;

	retv = usb_send(icdi, urbuf, inflen);
	if (unlikely(retv < 0))
		return retv;
	pos = 0;
	urbuf[0] = '+';
	do {
		retv = usb_recv(icdi, urbuf+pos, buflen-pos);
		if (retv < 0)
			break;
		pos += icdi->nxfer;
	} while ((pos < 3 || urbuf[pos-3] != '#') && (urbuf[0] == '+' ||
				urbuf[0] == '-'));
	if (retv == 0)
		retv = pos;
	return retv;
}


static int usb_sndrcv(struct icdi_device *icdi, char *urbuf, int inflen,
		int buflen)
{
	int retv, len, resend;
	char *curchr, sum, check;
	char *cmd;

	cmd = kmalloc(inflen+1, GFP_KERNEL);
	if (unlikely(!cmd)) {
		dev_err(&icdi->intf->dev, "Out of Memory\n");
		return -ENOMEM;
	}
	memcpy(cmd, urbuf, inflen);

	do {
		resend = 0;
		retv = do_usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (retv < 0) {
			cmd[len] = 0;
			dev_err(&icdi->intf->dev, "command %s failed: %d\n",
					cmd ,retv);
			break;
		} else if (urbuf[0] == '-') {
			resend = 1;
			memcpy(urbuf, cmd, inflen);
		}
	} while (resend == 1);
	if (retv < 0)
		goto exit_10;

	len = retv;
	if (memcmp(urbuf, "+$OK:", 5) == 0 && urbuf[len-3] == '#') {
		sum = 0;
		for (curchr = urbuf+5; curchr < urbuf + len - 3; curchr++)
			sum += *curchr;
		check = (hex2val(urbuf[len-2]) << 4) | hex2val(urbuf[len-1]);
		if ((sum - check) != 0) {
			dev_warn(&icdi->intf->dev, "response checksum error. " \
					"computed: %02hhx, in packet: %02hhx\n",
					sum, check);
			cmd[inflen] = 0;
			dev_info(&icdi->intf->dev, "Command is: %s\n", cmd);
			urbuf[len] = 0;
			dev_info(&icdi->intf->dev, "Response is: %s\n", urbuf);
		}
	}

exit_10:
	kfree(cmd);
	return retv;
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

static unsigned int mem_read_word(struct icdi_device *icdi, unsigned int addr,
		char *urbuf, int buflen)
{
	static const char memr[] = "$x,4";
	int len, inflen;
       	unsigned int val;

	val = 0x0;
	memcpy(urbuf, memr, 2);
	uint2hexstr(addr, urbuf+2);
	memcpy(urbuf+10, memr+2, 2);
	inflen = append_check_sum(urbuf, 12, buflen);
	urbuf[inflen] = 0;
	len = usb_sndrcv(icdi, urbuf, inflen, buflen);
	if (len == 12 && memcmp(urbuf, "+$OK:", 5) == 0)
		val = byte2word(urbuf+5);
	else
		dev_err(&icdi->intf->dev, "Memory Read Failed: %08x\n", addr);
	return val;
}

static void mem_write_word(struct icdi_device *icdi, unsigned int addr,
		unsigned int val, char *urbuf, int buflen)
{
	int inflen, len;
	static const char memw[] = "$X,4:";

	memcpy(urbuf, memw, 2);
	uint2hexstr(addr, urbuf+2);
	memcpy(urbuf+10, memw+2, 3);
	uint2hexstr(val, urbuf+13);
	inflen = append_check_sum(urbuf, 21, buflen);
	len = usb_sndrcv(icdi, urbuf, inflen, buflen);
	if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
		dev_err(&icdi->intf->dev, "Memory Write failed. Address: " \
				"%08x\n", addr);
		if (len > 0)
			dump_response(&icdi->intf->dev, urbuf, len);
	}
}

static int stop_debug(struct icdi_device *icdi, char *urbuf, int buflen)
{
	int len, inflen, retv = 0;
	static const char debug_hreset[] = "debug hreset";
	static const char restore_vector[] = "set vectorcatch 0";
	static const char debug_disable[] = "debug disable";

	if (icdi->in_debug == 0)
		return retv;
	if (icdi->stalled) {
		inflen = qRcmd_setup(urbuf, buflen,
				restore_vector, sizeof(restore_vector) - 1);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(&icdi->intf->dev, "debug vectorcatch 0 failed\n");
			if (len > 0)
				dump_response(&icdi->intf->dev, urbuf, len);
			return len;
		}

		inflen = qRcmd_setup(urbuf, buflen,
				debug_hreset, sizeof(debug_hreset) - 1);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(&icdi->intf->dev, "debug hreset failed\n");
			if (len > 0)
				dump_response(&icdi->intf->dev, urbuf, len);
			return len;
		}
		icdi->stalled = 0;
	}

	inflen = qRcmd_setup(urbuf, buflen,
			debug_disable, sizeof(debug_disable) - 1);
	len = usb_sndrcv(icdi, urbuf, inflen, buflen);
	if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
		dev_err(&icdi->intf->dev, "debug disable failed\n");
		if (len > 0)
			dump_response(&icdi->intf->dev, urbuf, len);
		return len;
	}
	icdi->in_debug = 0;
	return retv;
}

static int start_debug(struct icdi_device *icdi, int firmware,
		char *urbuf, int buflen)
{
	int len, retv, inflen;
	unsigned int val;
	struct device *dev = &icdi->intf->dev;
	static const char debug_clock[] = "debug clock \0";
	static const char debug_sreset[] = "debug sreset";

	retv = -1;
	if (!icdi->in_debug) {
		inflen = qRcmd_setup(urbuf, buflen,
				debug_clock, sizeof(debug_clock) - 1);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(&icdi->intf->dev, "debug clock failed\n");
			if (len > 0)
				dump_response(dev, urbuf, len);
			return retv;
		}
		len = sizeof(qSupported) - 1;
		memcpy(urbuf, qSupported, len);
		inflen = append_check_sum(urbuf, len, buflen);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) ||
				memcmp(urbuf, "+$PacketSize=", 13) != 0) {
			dev_err(&icdi->intf->dev, "qSupported failed\n");
			if (len > 0)
				dump_response(dev, urbuf, len);
			return retv;
		}
		len = sizeof(qmark) - 1;
		memcpy(urbuf, qmark, len);
		inflen = append_check_sum(urbuf, len, buflen);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$S00", 5) != 0) {
			dev_err(&icdi->intf->dev, "question mark failed\n");
			if (len > 0)
				dump_response(dev, urbuf, len);
			return retv;
		}
		icdi->in_debug = 1;
	}
	if (firmware && !icdi->stalled) {
		inflen = qRcmd_setup(urbuf, buflen,
				debug_sreset, sizeof(debug_sreset) - 1);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(&icdi->intf->dev, "debug sreset failed\n");
			if (len > 0)
				dump_response(dev, urbuf, len);
			return retv;
		}
		icdi->stalled = 1;
	}
	val = mem_read_word(icdi, DHCSR, urbuf, buflen);
	if (val != 0x00030003)
		dev_warn(&icdi->intf->dev, "Maybe not in debug state\n");
	return 0;
}

#define PROG_SIZE	1024

static int write_block(struct icdi_device *icdi, int finish)
{
	int buflen, len, inflen, retv = 0, i, proged, remlen, plen;
	char *urbuf, *dst, *src, c;
	struct device *dev = &icdi->intf->dev;
	static const char flash_erase[] = "$vFlashErase:";
	static const char flash_write[] = "$vFlashWrite:";
	static const char flash_done[] = "$vFlashDone";

	urbuf = NULL;
	if (icdi->flash.nxtpos == 0)
		goto flash_done;

	buflen = 64 + 2 * PROG_SIZE;
	urbuf = kmalloc(buflen, GFP_KERNEL);
	if (unlikely(!urbuf)) {
		dev_err(dev, "Out of Memory\n");
		return -ENOMEM;
	}
	len = sizeof(flash_erase) - 1;
	memcpy(urbuf, flash_erase, len);
	uint2hexstr(icdi->flash.offset, urbuf + len);
	len += 8;
	urbuf[len++] = ',';
	uint2hexstr(icdi->erase_size, urbuf + len);
	len += 8;
	inflen = append_check_sum(urbuf, len, buflen);
	len = usb_sndrcv(icdi, urbuf, inflen, buflen);
	if (len < 0 || memcmp(urbuf, "+$OK", 4) != 0) {
		dev_err(dev, "Unable to erase. Offset: %d, size: %u\n",
				icdi->flash.offset, icdi->erase_size);
		if (len > 0)
			dump_response(dev, urbuf, len);
		retv = -1;
		goto exit_10;
	}

	remlen = icdi->flash.nxtpos;
	proged = 0;
	while (remlen > 0) {
		len = sizeof(flash_write) - 1;
		memcpy(urbuf, flash_write, len);
		uint2hexstr(icdi->flash.offset+proged, urbuf + len);
		len += 8;
		urbuf[len++] = ':';
		dst = urbuf + len;
		src = icdi->flash.block + proged;
		plen = remlen > PROG_SIZE? PROG_SIZE : remlen;
		for (i = 0; i < plen; i++) {
			c = *src++;
			if (c == '#' || c == '$' || c == '}') {
				*dst++ = '}';
				c ^= 0x20;
			}
			*dst++ = c;
		}
		len = dst - urbuf;
		inflen = append_check_sum(urbuf, len, buflen);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (len < 0 || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(dev, "Cannot program the flash. Offset: %d, size: %d\n",
				       icdi->flash.offset, icdi->flash.nxtpos);
			if (len > 0)
				dump_response(dev, urbuf, len);
			retv = -1;
		}
		proged += plen;
		remlen -= plen;
	}

flash_done:
	if (finish) {
		if (urbuf == NULL) {
			buflen = 64;
			urbuf = kmalloc(buflen, GFP_KERNEL);
			if (unlikely(!urbuf)) {
				dev_err(dev, "Out of Memory\n");
				return -ENOMEM;
			}
		}
		len = sizeof(flash_done) - 1;
		memcpy(urbuf, flash_done, len);
		inflen = append_check_sum(urbuf, len, buflen);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (len < 0 || memcmp(urbuf, "+$OK", 4) != 0) {
			dev_err(dev, "flush done sent failed.\n");
			if (len > 0)
				dump_response(dev, urbuf, len);
			retv = -1;
		}
	}

exit_10:
	if (urbuf)
		kfree(urbuf);
	return retv;
}

static int program_block(struct icdi_device *icdi, const char *buf, int buflen,
		unsigned int offset)
{
	const char *src;
	char *dst;
	int onemove, remlen, datlen, retv, nxfer;

	if (icdi->flash.offset + icdi->flash.nxtpos != offset) {
		dev_err(&icdi->intf->dev, "Not Continuous in one flash " \
				"operation. prev offset: %u, current offset: " \
				"%u\n", icdi->flash.offset+icdi->flash.nxtpos,
				offset);
		return -1;
	}
	src = buf;
	datlen = buflen;
	nxfer = 0;
	while (datlen > 0) {
		dst = icdi->flash.block + icdi->flash.nxtpos;
		remlen = icdi->erase_size - icdi->flash.nxtpos;
		onemove = datlen < remlen? datlen : remlen;
		memcpy(dst, src, onemove);
		src += onemove;
		icdi->flash.nxtpos += onemove;
		datlen -= onemove;
		remlen -= onemove;
		if (remlen == 0) {
			retv = write_block(icdi, 0);
			if (retv != 0) {
				dev_err(&icdi->intf->dev,
						"Flash Programming Failed\n");
				return nxfer;
			}
			icdi->flash.nxtpos = 0;
			icdi->flash.offset += icdi->erase_size;
		}
		nxfer += onemove;
	}
	WARN_ON(nxfer != buflen);
	return nxfer;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *intf;
	int len;
	static const char in_debug[] = "in debug\n";
	static const char free_run[] = "free run\n";
	static const char stalled[] = ", reset and stalled\n";

	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	if (icdi->in_debug) {
		len = sizeof(in_debug);
		memcpy(buf, in_debug, len);
		if (icdi->stalled) {
			memcpy(buf+len-2, stalled, sizeof(stalled));
			len += sizeof(stalled) - 2;
		}
	} else {
		len = sizeof(free_run);
		memcpy(buf, free_run, len);
	}
	return len;
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t stlen)
{
	int max_cmdlen, len, cmdlen, retv, buflen;
	char cmd[16], *urbuf;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	static const char enter_debug[] = "-->debug<--";
	static const char leave_debug[] = "<--debug-->";

	cmdlen = sizeof(enter_debug) - 1;
	max_cmdlen = sizeof(cmd) - 1;
	len = max_cmdlen < stlen? max_cmdlen : stlen;
	memcpy(cmd, buf, len);
	cmd[len] = 0;
	if (stlen < cmdlen || stlen > max_cmdlen) {
		dev_info(dev, "Invalid Command length: %s\n", cmd);
		return -EINVAL;
	}
	retv = stlen;

	intf = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(intf);
	mutex_lock(&icdi->lock);
	buflen = 128;
	urbuf = kmalloc(buflen, GFP_KERNEL);
	if (unlikely(!urbuf)) {
		dev_err(dev, "Out of Memory\n");
		retv = -ENOMEM;
		goto exit_10;
	}
	if (memcmp(enter_debug, buf, cmdlen) == 0) {
		if (icdi->in_debug && icdi->stalled)
			goto exit_20;
		retv = start_debug(icdi, 1, urbuf, buflen);
		if (retv!= 0)
			dev_err(dev, "Cannot enter into debug state\n");
		else {
			mem_write_word(icdi, FMA, 0, urbuf, buflen);
			icdi->flash.offset = 0;
			icdi->flash.nxtpos = 0;
			icdi->flash.block = kmalloc(icdi->erase_size,
					GFP_KERNEL);
			if (!icdi->flash.block) {
				dev_err(dev, "Out Of Memory\n");
				retv = -ENOMEM;
			} else
				retv = stlen;
		}
	} else if (memcmp(leave_debug, buf, cmdlen) == 0) {
		if (icdi->in_debug == 0)
			goto exit_20;
		if (icdi->stalled) {
			retv = write_block(icdi, 1);
			kfree(icdi->flash.block);
			icdi->flash.block = NULL;
		}
		if (retv != 0)
			dev_err(dev, "Cannot program the last block\n");
		retv = stop_debug(icdi, urbuf, buflen);
		if (retv != 0)
			dev_err(dev, "Cannot leave debug state\n");
		else
			retv = stlen;
	} else {
		dev_info(dev, "Invalid Command: %s\n", cmd);
		retv = -EINVAL;
	}
exit_20:
	kfree(urbuf);

exit_10:
	mutex_unlock(&icdi->lock);
	return retv;
}

static int get_erase_size(struct icdi_device *icdi)
{
	int retv, buflen;
	unsigned int val;
	char *urbuf;

	buflen = 8192;
	urbuf = kmalloc(buflen, GFP_KERNEL);
	if (unlikely(!urbuf)) {
		dev_err(&icdi->intf->dev, "Out of Memory\n");
		return -ENOMEM;
	}
	retv = start_debug(icdi, 0, urbuf, buflen);
	if (unlikely(retv != 0)) {
		icdi->erase_size = 4096;
		dev_warn(&icdi->intf->dev, "Cannot enter into debug state: " \
				"%d\n", retv);
		goto exit_10;
	}
	mem_write_word(icdi, FP_CTRL, 0x3000000, urbuf, buflen);
	val = mem_read_word(icdi, DID1, urbuf, buflen);
	dev_info(&icdi->intf->dev, "DID1: %08X\n", val);
	retv = stop_debug(icdi, urbuf, buflen);
	if (unlikely(retv != 0)) {
		dev_warn(&icdi->intf->dev, "Cannot get out of debug state: " \
				"%d\n", retv);
		goto exit_10;
	}
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
		retv = 1;
	}

exit_10:
	kfree(urbuf);
	return retv;
}

#define FLASH_READ_SIZE	256

ssize_t firmware_read(struct file *filep, struct kobject *kobj,
		struct bin_attribute *binattr, 
		char *buf, loff_t offset, size_t bufsize)
{
	struct device *dev;
	struct usb_interface *intf;
	struct icdi_device *icdi;
	int retv, len, xferlen, rdlen, remlen, buflen, inflen;
	unsigned long fm_size;
	char *curbuf, *dst, *src, c;
	char *urbuf;

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
	buflen = 64 + 2 * PROG_SIZE;
	urbuf = kmalloc(buflen, GFP_KERNEL);
	if (unlikely(!urbuf)) {
		dev_err(&icdi->intf->dev, "Out of Memory\n");
		retv = -ENOMEM;
		goto exit_10;
	}
	if (unlikely(offset >= fm_size))
		goto exit_20;

	do {
		rdlen = PROG_SIZE  < remlen? PROG_SIZE : remlen;
		curbuf = urbuf;
		*curbuf++ = '$';
		*curbuf++ = 'x';
		uint2hexstr(offset+retv, curbuf);
		curbuf += 8;
		*curbuf++ = ',';
		uint2hexstr(rdlen, curbuf);
		inflen = append_check_sum(urbuf, 19, buflen-19);
		len = usb_sndrcv(icdi, urbuf, inflen, buflen);
		if (unlikely(len < 0) || memcmp(urbuf, "+$OK:", 5) != 0) {
			dev_err(&icdi->intf->dev, "Flash Dump failed at " \
					"%08llx, length: %d\n", offset + retv,
					rdlen);
			if (len > 0)
				dump_response(dev, urbuf, len);
			goto exit_20;
		}
		dst = buf + retv;
		src = urbuf+5;
		xferlen = 0;
		while (src - urbuf < len - 3 && xferlen < rdlen) {
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
	kfree(urbuf);
exit_10:
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
	int retv;

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
	if (!icdi->in_debug || !icdi->stalled) {
		dev_err(dev, "Device not in debug and flash programming state\n");
		return -EREMOTEIO;
	}
	mutex_lock(&icdi->lock);
	retv = program_block(icdi, buf, bufsize, offset);
	mutex_unlock(&icdi->lock);
	return retv;
}

static ssize_t version_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct icdi_device *icdi;
	struct usb_interface *interface;
	int len, retv, inflen, buflen;
	char *urbuf;
	static const char version[] = "version";

	retv = 0;
	interface = container_of(dev, struct usb_interface, dev);
	icdi = usb_get_intfdata(interface);
	mutex_lock(&icdi->lock);
	buflen = 128;
	urbuf = kmalloc(buflen, GFP_KERNEL);
	if (!buf) {
		retv = -ENOMEM;
		goto exit_10;
	}

	inflen = qRcmd_setup(urbuf, buflen, version, sizeof(version) - 1);
	len = usb_sndrcv(icdi, urbuf, inflen, buflen);
	if (len > 5)
		retv = hexstr2byte(urbuf+2, len - 5, buf, 4096);
	else {
		dev_err(&icdi->intf->dev, "Command 'version' failed\n");
		retv = len;
		if (retv > 0)
			memcpy(buf, urbuf, retv);
	}
	kfree(urbuf);

exit_10:
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
	icdi->flash.block = NULL;
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
	if (icdi->stalled)
		kfree(icdi->flash.block);
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
