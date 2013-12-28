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
 * @file emic2.cpp
 *
 * The EMIC2 speech module.
 */
#include <poll.h>
#include <stdio.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/systemlib.h>

#include "emic2.h"

#define EMIC2_BAUD_RATE			9600		// fixed and non-configurable
#define EMIC2_READ_TIMEOUT		3000		// ms, if now data during this delay assume that full update received
#define EMIC2_MAX_MSG_LEN		80

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int emic2_main(int argc, char *argv[]);

namespace
{

EMIC2	*g_dev;

}

EMIC2::EMIC2(const char *uart_path) :
	SensoryIO(uart_path),
	_talking(false)
{
	_messages[SENSORY_STARTUP_MSG] = "Ready.";
	_messages[SENSORY_STOP_MSG] = "Stop.";
	_messages[SENSORY_ERROR_MSG] = "Error.";
	_messages[SENSORY_NOTIFY_POSITIVE_MSG] = "Done.";
	_messages[SENSORY_NOTIFY_NEUTRAL_MSG] = "OK.";
	_messages[SENSORY_NOTIFY_NEGATIVE_MSG] = "Press safety switch.";
	_messages[SENSORY_ARMING_WARNING_MSG] = "Arming. Stand clear.";
	_messages[SENSORY_BATTERY_WARNING_MSG] = "Low battery voltage.";
	_messages[SENSORY_BATTERY_CRITICAL_MSG] = "Critical battery. Panic.";
	_messages[SENSORY_ARE_WE_THERE_YET_MSG] = "Are we there yet.";

	g_dev = this;	
}

EMIC2::~EMIC2()
{
	warnx("EMIC2: Shutting down");
	_task_should_exit = true;
}

int
EMIC2::init()
{
	int ret = ERROR;

	if (OK != g_dev->SensoryIO::init())
		goto out;

	/* start the driver worker task */
	_task = task_spawn_cmd("emic2",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_SLOW_DRIVER,
			       2048,
			       (main_t)&EMIC2::task_main_trampoline,
			       nullptr);
	
	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:

//	set_baudrate(EMIC2_BAUD_RATE);
	return ret;
}

void
EMIC2::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
EMIC2::task_main()
{
	log("EMIC2::task_main starting");
	sleep(5); // TEMPORARY - REMOVE

	_serial_fd = ::open(_port, O_RDWR);
	set_baudrate(EMIC2_BAUD_RATE);

	while (!_task_should_exit) {
		if (strlen(_msg) > 0) {
			warnx("Saying: %s", _msg);
			send(_msg, strlen(_msg));
			_msg[0] = '\0';
		}
		usleep(500000);
	}
	log("exiting task_main");
	
	::close(_serial_fd);
	
	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
EMIC2::reset()
{
	warnx("RESET");
	_msg[0] = '\n';
	return OK;
}

int
EMIC2::set_volume(uint8_t value)
{
	strcpy(_msg, "V18\n");
	return OK;
}

int
EMIC2::interrupt()
{
	warnx("SHOULD NOT BE HERE");
	return OK;
	//send("<", 1);
	//char *msg;
	//return receive(msg);	// wait for the ready signal	
}

/**
 * The only thing you will ever receive from the Emic 2 is the
 * ":" character indicating that the device is ready for input.
 */
int
EMIC2::receive(char *msg)
{	
	struct pollfd fds;
	fds.fd = _serial_fd;
	fds.events = POLLIN;

	if (::poll(&fds, 1, EMIC2_READ_TIMEOUT) > 0) {
		uint8_t buf;
		::read(_serial_fd, &msg, sizeof(msg));
		warnx("RECV: %s", msg);
		if (msg[0] == ':') {
			_talking = false;
			return OK;
		}
	}
	return ERROR;
}

int
EMIC2::send_msg_id(unsigned long id)
{
	//warnx("Saying: %s", _messages[id]);
	// Filter out the messages we want to have spoken here.

	sprintf(_msg, "S%s.\n", _messages[id]);

	return OK;
	//return send(final_msg, strlen(final_msg));
}

/**
 * Send a message to the speech module. If it is currently speaking
 * interrupt and send the new message.
 * TODO(sjwilks): We may want to interrupt only on high priority messages.
 */
int
EMIC2::send(const char *msg, size_t size)
{	
//	warnx("SENDING: %s", msg);
	// Try and interrupt the current message
	if (_talking) {
		//interrupt();
	}
	_talking = true;
	send_data(msg, size);
	_talking = false;
	return OK;
}

/**
 * Local functions in support of the shell command.
 */
namespace emic2
{

#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

EMIC2	*g_dev;

void	start(const char *uart_path);
void	stop();
void	test();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new EMIC2(uart_path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	g_dev->reset();
	sleep(4);
	g_dev->set_volume(18);
	sleep(4);
	g_dev->send_msg_id(SENSORY_STARTUP_MSG);
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	warnx("gdev: %d", g_dev);
	int fd = open(SENSORY_IO_DEVICE_PATH, O_RDWR);

	if (fd < 0)
		err(1, "failed ");

	for (int i = 0; i < SENSORY_NUMBER_OF_MSGS; i++) {
		if (ioctl(fd, SENSORY_IO_SET_MSG, i) < 0) {
			err(1, "message send failed");
		}
		sleep(3);
	}

	close(fd);
	errx(0, "PASS");
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
emic2_main(int argc, char *argv[])
{
	/* set to default */
	const char* device_name = SENSORY_IO_DEFAULT_UART_PORT;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				device_name = argv[i + 1];
			}
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		emic2::start(device_name);

	if (!strcmp(argv[1], "stop"))
		emic2::stop();
	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		emic2::test();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		emic2::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', or 'status' [-d /dev/ttyS0-n]");
}
