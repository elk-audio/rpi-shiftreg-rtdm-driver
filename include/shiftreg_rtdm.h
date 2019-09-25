 /*
 * Copyright (C) 2018 MIND Music Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SHIFTREG_RTDM_H
#define SHIFTREG_RTDM_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define SHIFTREG_RTDM_IOC_MAGIC		's'

// Config IOCTLS
#define SHIFTREG_SET_ADDR_CONFIG  				_IOW(SHIFTREG_RTDM_IOC_MAGIC, 1, int)

// Run time IOCTLS
//#define SHIFTREG_GET_INPUT_SHIFTREG_DATA		_IOW(RPI_SHIFTREG_RTDM_IOC_MAGIC, 2, int)
//#define SHIFTREG_SET_OUTPUT_SHIFTREG_DATA		_IOW(RPI_SHIFTREG_RTDM_IOC_MAGIC, 3, int)
#define SHIFTREG_XFER							_IOWR(SHIFTREG_RTDM_IOC_MAGIC, 2, int)

/**
 * User addresses to store shiftreg data. each address is an array
 * which is equal to the num of pins of the particular type.
 */
struct shiftreg_addr_config_t
{
	uint32_t __user *user_output_pin_data;
	uint32_t __user *user_input_pin_data;
	uint32_t __user *user_adc_pin_data;
};

#endif //SHIFTREG_RTDM_H