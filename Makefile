ifneq ($(KERNELRELEASE),)
	obj-m += usbdfu.o
	obj-m += usbprobe.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	WDIR := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(WDIR) modules

clean:
	rm -f modules.order Module.symvers
	rm -f *.o *.ko *.mod.c
	rm -f core
endif
