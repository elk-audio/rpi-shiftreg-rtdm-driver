// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (C) 2019 Modern Ancient Instruments Networked AB, dba Elk Stockholm
 * Author(s): Sharan Yagneswar <sharan@elk.audio>
 *            Nitin Kulkarni <nitin@elk.audio>
 *
 * IOCTL interface for shofreg_rtdm module
 *
 */

#ifndef SHIFTREG_RTDM_H
#define SHIFTREG_RTDM_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define SHIFTREG_RTDM_IOC_MAGIC		's'

// Run time IOCTLS
#define SHIFTREG_RTDM_START_RT_TASK	_IO(SHIFTREG_RTDM_IOC_MAGIC, 1)
#define SHIFTREG_RTDM_STOP_RT_TASK	_IO(SHIFTREG_RTDM_IOC_MAGIC, 2)
#define SHIFTREG_RTDM_WAIT_ON_RT_TASK	_IO(SHIFTREG_RTDM_IOC_MAGIC, 3)
#define SHIFTREG_RTDM_SET_TICK_PERIOD	_IOW(SHIFTREG_RTDM_IOC_MAGIC, 4, int)

#endif //SHIFTREG_RTDM_H
