/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file lcd.cpp
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * LCD implementation.
 *
 * What it does.......
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
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_gps_position.h>


static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const char daemon_name[] = "lcd";
static const char commandline_usage[] = "usage: lcd start|status|stop [-d <device>]";


class LCD_4DS
{
public:
	LCD_4DS(const char* device);
	virtual ~LCD_4DS();

	int init();

	void write_user_led(uint8_t id, uint16_t value);
	void write_led_gauge(uint8_t id, uint16_t value);
	void write_custom_digits(uint8_t id, uint16_t value);

private:
	struct write_obj {
		uint8_t code;				
		uint8_t object_id;
		uint8_t object_index;
		uint8_t value_L;
		uint8_t value_H;
		uint8_t checksum;
	};

	enum CommandCode {
		COMMAND_READ_OBJ = 0x00,
		COMMAND_WRITE_OBJ = 0x01,
		COMMAND_WRITE_STR = 0x02,
		COMMAND_WRITE_USTR = 0x03,
		COMMAND_WRITE_CONTRAST = 0x04,
		COMMAND_REPORT_OBJ = 0x05,
		COMMAND_REPORT_EVENT = 0x06
	};

	enum ObjectID {
		OBJECT_CUSTOM_DIGITS = 0x09,
		OBJECT_LED_GAUGE = 0x0b,
		OBJECT_USER_LED = 0x13
	};

	const char* _device;
	int _uart;

	int open_uart();
	int send_data(uint8_t *buffer, size_t size);
	void compute_msb_lsb(uint16_t value, uint8_t* msb, uint8_t* lsb);
	void write_obj(uint8_t id, uint8_t index, uint16_t value);
};

/**
 * Deamon management function.
 */
extern "C" __EXPORT int lcd_main(int argc, char *argv[]);

LCD_4DS::LCD_4DS(const char *device)
	: _device(device),
	  _uart(-1) {
}

LCD_4DS::~LCD_4DS() {
	if (_uart != -1) {
		close(_uart);
	}
}

int
LCD_4DS::init()
{
	int ret;
	ret = open_uart();

	if (ret != OK) {
		warnx("UART init failed");
		return ret;
	}

	return OK;
}

int
LCD_4DS::open_uart()
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
LCD_4DS::send_data(uint8_t *buffer, size_t size)
{
	uint16_t checksum = 0;

	for (size_t i = 0; i < size; i++) {
		if (i == size - 1) {
			/* Set the checksum: the first uint8_t is taken as the checksum. */
			buffer[i] = checksum;
		} else {
			checksum ^= buffer[i];
		}
		write(_uart, &buffer[i], sizeof(buffer[i]));
	}

	return OK;
}

void
LCD_4DS::compute_msb_lsb(uint16_t value, uint8_t* msb, uint8_t* lsb)
{
	*msb = (uint8_t)value & 0xff;
	*lsb = (uint8_t)(value >> 8) & 0xff;
}

void
LCD_4DS::write_obj(uint8_t id, uint8_t index, uint16_t value)
{
	struct write_obj obj;
	obj.code = COMMAND_WRITE_OBJ;
	obj.object_id = id;
	obj.object_index = index;

	compute_msb_lsb(value, &(obj.value_H), &(obj.value_L));

	uint8_t buffer[6];
	memcpy(buffer, &obj, 6);
	send_data(buffer, 6);
}

void
LCD_4DS::write_user_led(uint8_t index, uint16_t value)
{
	write_obj(OBJECT_USER_LED, index, value);
}

void
LCD_4DS::write_led_gauge(uint8_t index, uint16_t value)
{
	write_obj(OBJECT_LED_GAUGE, index, value);
}

void
LCD_4DS::write_custom_digits(uint8_t index, uint16_t value) {
	write_obj(OBJECT_CUSTOM_DIGITS, index, value);
}


/**
 * Mainloop of daemon.
 */
int lcd_thread_main(int argc, char *argv[]);


int
lcd_thread_main(int argc, char *argv[])
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

	LCD_4DS* lcd = new LCD_4DS(device);

	if (lcd->init() < 0) {
		errx(1, "Failed to initialise, exiting.");
		thread_running = false;
	}

	// If this app is running then the FMU must be running too.
	lcd->write_user_led(0, 1);

	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int telemetry_status_sub = orb_subscribe(ORB_ID(telemetry_status));
	int input_rc_sub = orb_subscribe(ORB_ID(input_rc));

	bool last_telemetry_status_off = true;
	bool last_gps_state_off = true;
	bool last_ppm_state_off = true;
	bool updated = false;
	int value = 0;

	while (!thread_should_exit) {	
		// Check for mavlink radio
		orb_check(vehicle_gps_position_sub, &updated);
		value = 0;
		if (updated && last_telemetry_status_off) {
			value = 1;	
		}
		last_telemetry_status_off = !last_telemetry_status_off;
		lcd->write_user_led(4, value);

		// Check for GPS
		bool new_data_vehicle_gps_position;
		orb_check(vehicle_gps_position_sub, &updated);
		value = 0;
		if (updated && last_gps_state_off) {
			value = 1;	
		}
		last_gps_state_off = !last_gps_state_off;
		lcd->write_user_led(3, value);

		// Check for RC input.
		value = 0;
		orb_check(input_rc_sub, &updated);
		if (updated) {
			/* get a local copy of the current sensor values */
			struct rc_input_values input_rc_raw;
			memset(&input_rc_raw, 0, sizeof(input_rc_raw));
			orb_copy(ORB_ID(input_rc), input_rc_sub, &input_rc_raw);
			
			value = 1;			

			// Read the current rc PWM values for the first 5 channels only.
			uint8_t ch[5] = {5, 6, 0, 1, 2};
			for (int i = 0; i < 5; ++i) {
				lcd->write_led_gauge(ch[i], input_rc_raw.values[i]);
				lcd->write_custom_digits(i, input_rc_raw.values[i]);
				}
		}
		lcd->write_user_led(1, value);
		lcd->write_user_led(3, value);

		usleep(25000);
	}

	warnx("exiting");

	thread_running = false;

	return 0;
}

/**
 * Process command line arguments and tart the daemon.
 */
int
lcd_main(int argc, char *argv[])
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
					     1024,
					     lcd_thread_main,
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
