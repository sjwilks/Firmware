/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Simple UART driver interface.
 */

#ifndef _DRV_SENSORY_IO_H
#define _DRV_SENSORY_IO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define SENSORY_IO_DEFAULT_UART_PORT "/dev/ttyS0"

#define SENSORY_IO_DEVICE_PATH	"/dev/simple_uart"

typedef enum {
	DRIVER_TYPE_NONE = 0,
	DRIVER_TYPE_EMIC2,
	DRIVER_TYPE_LCD_2X60
} _driver_mode_t;

enum {
	SENSORY_STOP_MSG = 0,
	SENSORY_STARTUP_MSG,
	SENSORY_ERROR_MSG,
	SENSORY_NOTIFY_POSITIVE_MSG,
	SENSORY_NOTIFY_NEUTRAL_MSG,
	SENSORY_NOTIFY_NEGATIVE_MSG,
	SENSORY_ARE_WE_THERE_YET_MSG,
	SENSORY_ARMING_WARNING_MSG,
	SENSORY_BATTERY_WARNING_MSG,
	SENSORY_BATTERY_CRITICAL_MSG,
	SENSORY_NUMBER_OF_MSGS
};

/*
 * ObjDev tag for SENSORY_IO data.
 */
ORB_DECLARE(sensory_io);

/*
 * ioctl() definitions
 */
#define _SENSORY_IO_SET_BASE			0x3000
#define SENSORY_IO_SET_MSG			_IOC(_SENSORY_IO_SET_BASE, 1)

#endif /* _DRV_SENSORY_IO_H */
