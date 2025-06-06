CROSS_COMPILE = /home/bach/buildroot-labs/buildroot/output/host/usr/bin/arm-buildroot-linux-gnueabihf-
ARCH = arm
ifneq ($(KERNELRELEASE),)
    obj-m := led_driver.o
else
    KDIR := /home/bach/buildroot-labs/buildroot/output/build/linux-6.6.32
    PWD := $(shell pwd)
all:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
endif

