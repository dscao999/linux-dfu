ifneq ($(KERNELRELEASE),)
	obj-m += usbdfu.o
	usbdfu-objs := usb_dfu.o

	obj-m += usb_icdi.o
else
	KERNVER ?= $(shell uname -r)
	KERNELDIR ?= /lib/modules/$(KERNVER)/build
	WDIR := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(WDIR) modules

clean:
	@rm -f modules.order Module.symvers
	@rm -rf .tmp_versions
	@rm -f *.o *.ko *.mod.c
	@rm -f .*.ko.cmd .*.o.cmd
	@rm -f core
endif
