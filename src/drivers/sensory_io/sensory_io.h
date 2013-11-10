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
 * @file airspeed.h
 * @author Simon Wilks
 *
 * Generic driver for airspeed sensors connected via I2C.
 */

#ifndef SENSORY_IO_H_
#define SENSORY_IO_H_

#include <nuttx/config.h>

#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>


#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_sensory_io.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

class __EXPORT SensoryIO : public device::CDev
{
public:
	SensoryIO(const char* uart_path);
	virtual ~SensoryIO();

	virtual int	init();

	//virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	virtual void	print_info();

protected:
	virtual	int 		send_msg_id(unsigned long id) = 0;

	work_s			_work;
	unsigned		_num_reports;
	volatile unsigned	_next_report;
	volatile unsigned	_oldest_report;
	bool			_sensor_ok;
	int			_measure_ticks;
	bool			_collect_phase;

	orb_advert_t		_airspeed_pub;

	unsigned		_conversion_interval;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to SensoryIO
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						//< worker task
	bool 				_baudrate_changed;				///< flag to signal that the baudrate with the SensoryIO has changed
	bool				_mode_changed;					///< flag that the SensoryIO mode has changed
	_driver_mode_t			_mode;						///< current mode

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	//void		start();

	/**
	* Stop the automatic measurement state machine.
	*/
	//void		stop();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	//static void	cycle_trampoline(void *arg);

	/**
	 * Try to configure the SensoryIO, handle outgoing communication to the SensoryIO
	 */
	void		config();

	/**
	 * Trampoline to the worker task
	 */
	//static void	task_main_trampoline(void *arg);


	/**
	 * Worker task: main SensoryIO thread that configures the SensoryIO and parses incoming data, always running
	 */
	//void		task_main(void);

	/**
	 * Set the baudrate of the UART to the SensoryIO
	 */
	int		set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the SensoryIO
	 */
	void		cmd_reset();

	/**
	 * Send an interrupt command to stop outputing the current message
	 */
	void		cmd_interrupt();

	int		send_data(const char* buffer, size_t size);

};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

#endif /* SENSORY_IO_H */

