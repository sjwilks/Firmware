/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file speech.cpp
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * Speech implementation.
 *
 * This is a prototype only. Turn this into a device class and use ioctl etc.
 *
 */

#include <drivers/drv_rc_input.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>
#include <unistd.h>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

#define MAX_TEXT_LEN 	80

static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const char daemon_name[] = "speech";
static const char commandline_usage[] = "usage: speech start|status|stop [-d <device>]";


class Speech
{
public:
	Speech(const char* device);
	virtual ~Speech();

	int 	init();

	void 	say(const char* message);
	void 	reset();

private:
	const char* _device;
	int _uart;

	int open_uart();
	int send_data(char buffer[], size_t size);
};

/**
 * Deamon management function.
 */
extern "C" __EXPORT int speech_main(int argc, char *argv[]);

Speech::Speech(const char *device)
	: _device(device),
	  _uart(-1) {
}

Speech::~Speech() {
	if (_uart != -1) {
		close(_uart);
	}
}

int
Speech::init()
{
	int ret;
	ret = open_uart();

	if (ret != OK) {
		warnx("UART init failed");
		return ret;
	}

	reset();

	return OK;
}

int
Speech::open_uart()
{
	/* baud rate */
	static const speed_t speed = B9600;

	/* open uart */
	_uart = open(_device, O_RDWR | O_NOCTTY);

	if (_uart < 0) {
		err(1, "Error opening port: %s", _device);
	}
	
	/* Back up the original uart configuration to restore it after exit */	
	int termios_state;
	struct termios uart_config_original;
	if ((termios_state = tcgetattr(_uart, &uart_config_original)) < 0) {
		close(_uart);
		err(1, "Error getting baudrate / termios config for %s: %d", _device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(_uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(_uart);
		err(1, "Error setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)",
		    _device, termios_state);
	}

	if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
		close(_uart);
		err(1, "Error setting baudrate / termios config for %s (tcsetattr)", _device);
	}

	return OK;
}

int
Speech::send_data(char* buffer, size_t size)
{
	for (size_t i = 0; i < size; i++) {
		write(_uart, &buffer[i], sizeof(buffer[i]));
	}

	return OK;
}

void
Speech::reset() {
	send_data("\n", 1);
}

void
Speech::say(const char* message)
{
	if (strlen(message) > MAX_TEXT_LEN) {
		warnx("Message too long: %s", message);
		message = "error. string too long.";
	}

	char final_msg[MAX_TEXT_LEN];
	sprintf(final_msg, "S%s.\n", message);
	warnx("MSG: %s", final_msg);
	send_data(final_msg, strlen(final_msg)+1);

	// TODO(sjwilks): Probably should add a blocking read to await the completed response: ":".
}

/**
 * Mainloop of daemon.
 */
int speech_thread_main(int argc, char *argv[]);


int
speech_thread_main(int argc, char *argv[])
{
	warnx("starting");

	thread_running = true;

	const char *device = "/dev/ttyS0";		/**< Default telemetry port: USART2 */

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -d\n%s", commandline_usage);
			}
		}
	}

	Speech* speech = new Speech(device);

	if (speech->init() < 0) {
		errx(1, "Failed to initialise, exiting.");
		thread_running = false;
	}

	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	int vehicle_sub = orb_subscribe(ORB_ID(vehicle_status));

	int last_gps_fix = -1;
	int last_gps_sat_count = -1;
	float last_battery_voltage = -1.0f;
	int last_vehicle_status_arming = -1;
	
	bool updated = false;

	while (!thread_should_exit) {
		warnx("Speak!");

		char msg[80];
		// GPS messages
		orb_check(gps_sub, &updated);
		if (updated) {
			struct vehicle_gps_position_s gps;
			memset(&gps, 0, sizeof(gps));
			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
			if (last_gps_fix != gps.fix_type || last_gps_sat_count != gps.satellites_visible) {
				if (gps.fix_type == 3) {
					sprintf(msg, "gee pee es 3D fix. Number of satellites %d", gps.satellites_visible);
				} else {
					sprintf(msg, "gee pee es no fix. Number of satellites %d", gps.satellites_visible);
				}
				speech->say(msg);

				last_gps_fix = gps.fix_type;
				last_gps_sat_count = gps.satellites_visible;
			}
		}
		sleep(3);

		// Battery messages
		orb_check(battery_sub, &updated);
		if (true/*updated*/) {
			struct battery_status_s battery;
			memset(&battery, 0, sizeof(battery));
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);
			
			if (last_battery_voltage != battery.voltage_v) {
				sprintf(msg, "Battery voltage %2.1f volts", battery.voltage_v);
				speech->say(msg);
			}
			last_battery_voltage = battery.voltage_v;
		}
		sleep(3);

		// Vehicle status messages
		orb_check(vehicle_sub, &updated);
		if (updated) {
			struct vehicle_status_s vehicle;
			memset(&vehicle, 0, sizeof(vehicle));
			orb_copy(ORB_ID(vehicle_status), vehicle_sub, &vehicle);
			if (last_vehicle_status_arming != vehicle.arming_state) {
				if (vehicle.arming_state == ARMING_STATE_ARMED) {
					speech->say("System is armed. Stand clear.");
				}
				if (vehicle.arming_state == ARMING_STATE_ARMED) {
					speech->say("System is in standby.");
				}
			}
			last_vehicle_status_arming = vehicle.arming_state;
		}
		sleep(3);
	}

	warnx("exiting");

	thread_running = false;

	return 0;
}

/**
 * Process command line arguments and start the daemon.
 */
int
speech_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "missing command\n%s", commandline_usage);
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("deamon already running");
			exit(0);
		}

		thread_should_exit = false;
		warnx("START");
		deamon_task = task_spawn_cmd(daemon_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 40,
					     2048,
					     speech_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("daemon is running");

		} else {
			warnx("daemon not started");
		}

		exit(0);
	}

	errx(1, "unrecognized command\n%s", commandline_usage);
}