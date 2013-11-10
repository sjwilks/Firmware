/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file ets_airspeed.cpp
 * @author Simon Wilks
 *
 * Driver for a text based notifier such as speech synthesis device or and LCD
 * that accepts ASCII messages.
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <drivers/device/device.h> 
#include <drivers/drv_sensory_io.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "helper.h"
#include "emic2.h"
#include "lcd_2x60.h"

#define TIMEOUT_5HZ 500


//namespace
//{

//SensoryIO	*g_dev;

//}


SensoryIO::SensoryIO(const char* uart_path) : CDev("SensoryIO", SENSORY_IO_DEVICE_PATH),
	_task_should_exit(false),
	_mode_changed(false),
	_mode(DRIVER_TYPE_NONE)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
//	g_dev = this;

	_debug_enabled = true;
}

SensoryIO::~SensoryIO()
{
	warnx("exiting");

	::close(_serial_fd);

	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
//	g_dev = nullptr;

}

int
SensoryIO::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* open the serial port */
	//_serial_fd = ::open(_port, O_RDWR);
	//if (_serial_fd < 0) {
	//	log("failed to open serial port: %s err: %d", _port, errno);
	//	/* tell the dtor that we are exiting, set error code */
	//	_task = -1;
	//	_exit(1);
	//}

	// TODO: Test it worked.
	//set_baudrate(9600);

	ret = OK;
out:
	return ret;
}

int
SensoryIO::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	//case SENSORY_IO_RESET:
	//	// send the message
	//	reset();
	//	break;	
	case SENSORY_IO_SET_MSG:
		// send the message
		send_msg_id(arg);
		break;
	default:
		break;
	}

	unlock();

	return OK;
}

int
SensoryIO::send_data(const char* buffer, size_t size)
{
	int c = 0;
	for (size_t i = 0; i < size; i++) {
		//printf("%d:%c\n", i, buffer[i]);
		c += ::write(_serial_fd, &buffer[i], sizeof(buffer[i]));
	}
	return OK;
}

int
SensoryIO::set_baudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	warnx("try baudrate: %d\n", speed);

	default:
		warnx("ERROR: Unsupported baudrate: %d\n", baud);
		return -EINVAL;
	}
	struct termios uart_config;
	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	//uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERROR setting config: %d (cfsetispeed)\n", termios_state);
		return -1;
	}
	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERROR setting config: %d (cfsetospeed)\n", termios_state);
		return -1;
	}
	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERROR setting baudrate (tcsetattr)\n");
		return -1;
	}

	_baudrate = baud;
	/* XXX if resetting the parser here, ensure it does exist (check for null pointer) */
	return 0;
}


//void
//SensoryIO::task_main()
//{
//}

void
SensoryIO::cmd_reset()
{
	//XXX add reset?
}


void
SensoryIO::cmd_interrupt()
{
	//_Helper->interrupt();
}


void
SensoryIO::print_info()
{
	warnx("port: %s, baudrate: %d", _port, _baudrate);

	usleep(100000);
}
