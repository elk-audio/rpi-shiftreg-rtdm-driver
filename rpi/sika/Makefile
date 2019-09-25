obj-m += rpi_shiftreg_rtdm.o
rpi_shiftreg_rtdm-objs += src/rpi_shiftreg_rtdm.o src/bcm2835.o 
SRC := $(shell pwd)

all:
	$(MAKE) ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) clean
	@rm -f *.o
	@rm -f *.o.*
