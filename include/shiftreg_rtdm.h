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

// Run time IOCTLS
#define SHIFTREG_RTDM_START_RT_TASK					_IO(SHIFTREG_RTDM_IOC_MAGIC, 1)
#define SHIFTREG_RTDM_STOP_RT_TASK					_IO(SHIFTREG_RTDM_IOC_MAGIC, 2)
#define SHIFTREG_RTDM_WAIT_ON_RT_TASK				_IO(SHIFTREG_RTDM_IOC_MAGIC, 3)

#endif //SHIFTREG_RTDM_H