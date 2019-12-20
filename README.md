# RTDM shiftreg driver

Xenomai real-time driver to control shift registers on Elk Pi boards using GPIOs and SPI.

Based on Nicolas Schurando's spi-bcm283x-rtdm driver:
https://github.com/nicolas-schurando/spi-bcm283x-rtdm.git.

Credits to Mike McCauley's user space library to access bcm2835 peripherals:
http://www.airspayce.com/mikem/bcm2835/.

## Building
Navigate to `boards/\<board-name>`

and then build with:
```
$ export KERNEL_PATH=<path to kernel source>
$ export CROSS_COMPILE=<arm compiler prefix>
$ make
```

## Usage Example
To load the driver :
`insmod shiftreg_rtdm.ko adc_chans_per_tick=4`

where `adc_chans_per_tick` specifies how many adc channels are sampled per iteration.

---
Copyright 2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm