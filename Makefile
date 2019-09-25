obj-m += spi-bcm283x-rtdm.o
spi-bcm283x-rtdm-objs += src/spi-bcm283x-rtdm.o src/bcm2835.o 
SRC := $(shell pwd)

all:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) clean
	@rm -f *.o
	@rm -f *.o.*
