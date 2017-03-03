ifneq ($(KERNELRELEASE),)
	obj-m += usbdfu.o

usbdfu.o: usbdfu.h

else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
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
