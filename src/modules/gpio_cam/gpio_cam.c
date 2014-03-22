/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file gpio_cam.c
 *
 * Camera controller via GPIO driver.
 *
 * @author Simon Wilks <sjwilks@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <modules/px4iofirmware/protocol.h>


typedef enum CAM_STATUS {
	CAM_STATUS_STANDBY = 1,
	CAM_STATUS_OFF,
	CAM_STATUS_FILMING,
	CAM_STATUS_SHOOT
} cam_status_t;

struct gpio_cam_s {
	struct work_s work;
	int gpio_fd;
	bool use_io;
	int pin;
	struct actuator_armed_s armed;
	int actuator_armed_sub;
	bool relay_on;
	cam_status_t cam_status;
	int counter;
};

static struct gpio_cam_s gpio_cam_data;
static bool gpio_cam_started = false;

__EXPORT int gpio_cam_main(int argc, char *argv[]);

void gpio_cam_start(FAR void *arg);

void gpio_cam_cycle(FAR void *arg);

int gpio_cam_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: gpio_led {start|stop} [-p <1|2|a1|a2|r1|r2>]\n"
		     "\t-p\tUse pin:\n"
		     "\t\t1\tPX4IO RELAY1\n"
		     "\t\t2\tPX4IO RELAY2\n"
		     "\t\ta1\tPX4FMU GPIO_AUX1 (default)\n"
		     "\t\ta2\tPX4FMU GPIO_AUX2");

	} else {

		if (!strcmp(argv[1], "start")) {
			if (gpio_cam_started) {
				errx(1, "already running");
			}

			bool use_io = true;
			int pin = PX4IO_P_SETUP_RELAYS_POWER1;

			if (argc > 2) {
				if (!strcmp(argv[2], "-p")) {
					if (!strcmp(argv[3], "1")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_POWER1;

					} else if (!strcmp(argv[3], "2")) {
						use_io = true;
						pin = PX4IO_P_SETUP_RELAYS_POWER2;

					} else if (!strcmp(argv[3], "a1")) {
						use_io = false;
						//pin = GPIO_AUX_1;

					} else if (!strcmp(argv[3], "a2")) {
						use_io = false;
						//pin = GPIO_AUX_2;

					} else  {
						errx(1, "unsupported pin: %s", argv[3]);
					}
				}
			}

			memset(&gpio_cam_data, 0, sizeof(gpio_cam_data));
			gpio_cam_data.use_io = use_io;
			gpio_cam_data.pin = pin;
			int ret = work_queue(LPWORK, &gpio_cam_data.work, gpio_cam_start, &gpio_cam_data, 0);

			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				gpio_cam_started = true;
				char pin_name[24];

				if (use_io) {
					sprintf(pin_name, "PX4IO RELAY%i", pin);

				} else {
					sprintf(pin_name, "PX4FMU GPIO_AUX%i", pin);

				}

				warnx("start, using pin: %s", pin_name);
			}

			exit(0);


		} else if (!strcmp(argv[1], "stop")) {
			if (gpio_cam_started) {
				gpio_cam_started = false;
				warnx("stop");

			} else {
				errx(1, "not running");
			}

		} else {
			errx(1, "unrecognized command '%s', only supporting 'start' or 'stop'", argv[1]);
		}
	}
	// TODO(sjwilks): Missing return statement.
}

void gpio_cam_start(FAR void *arg)
{
	FAR struct gpio_cam_s *priv = (FAR struct gpio_cam_s *)arg;

	char *gpio_dev;

	if (priv->use_io) {
		gpio_dev = PX4IO_DEVICE_PATH;
	} else {
		gpio_dev = PX4FMU_DEVICE_PATH;
	}

	/* open GPIO device */
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		//printf("gpio_led: GPIO device \"%s\" open fail\n", gpio_dev);
		gpio_cam_started = false;
		return;
	}

	/* configure GPIO pin */
	/* px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	priv->counter = 0;
	priv->cam_status = CAM_STATUS_OFF;
	priv->relay_on = false;

	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, gpio_cam_cycle, priv, 0);

	if (ret != 0) {
		// TODO find way to print errors
		//printf("gpio_led: failed to queue work: %d\n", ret);
		gpio_cam_started = false;
		return;
	}
}

void gpio_cam_cycle(FAR void *arg)
{
	FAR struct gpio_cam_s *priv = (FAR struct gpio_cam_s *)arg;

	/* check for status updates*/
	bool status_updated;
	orb_check(priv->actuator_armed_sub, &status_updated);

	if (status_updated)
		orb_copy(ORB_ID(actuator_armed), priv->actuator_armed_sub, &priv->armed);

	priv->relay_on = false;

	if (priv->cam_status == CAM_STATUS_OFF) {
		/* always put the cam in standby whatever the armed state */
		// TODO(sjwilks): We may want to warn if we armed and it was
		// in standby yet (delayed start of filming)
		if (priv->counter < 40) {
			priv->relay_on = true;
		} else {
			priv->cam_status = CAM_STATUS_STANDBY;
		}
	} else if (priv->armed.armed) {
		if (priv->cam_status == CAM_STATUS_STANDBY) {
			/* start filming: switch on for 1 sec. */
			if (priv->counter < 5) {
				priv->relay_on = true;
			} else {
				priv->cam_status = CAM_STATUS_FILMING;
			}
		}
	} else if (priv->armed.ready_to_arm) {
		if (priv->cam_status == CAM_STATUS_FILMING) {
			/* we have probably just disarmed, stop filming: switch on for 0.5 secs. */
			if (priv->counter < 5) {
				priv->relay_on = true;
			} else {
				priv->cam_status = CAM_STATUS_STANDBY;
			}
		}	
	}

	if (priv->relay_on) {
		ioctl(priv->gpio_fd, GPIO_SET, priv->pin);
		priv->counter++;
	} else {
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
		priv->counter = 0;
	}

	/* repeat cycle at 10 Hz */
	if (gpio_cam_started) {
		work_queue(LPWORK, &priv->work, gpio_cam_cycle, priv, USEC2TICK(100000));

	} else {
		/* switch off LED on stop */
		ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
	}
}
