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
 * @file lcd_2x60.cpp
 *
 * An LCD device that provides 2 lines by 60 character display. Communication
 * with this LCD is simply straight text via a UART. It also supports inputs
 * though support for this will not be in the initial version.
 */
#include <poll.h>
#include <stdio.h>

#include "lcd_2x60.h"

#define LCD_2X60_BAUD_RATE		9600		// fixed and non-configurable
#define LCD_2X60_READ_TIMEOUT		1000		// ms, if now data during this delay assume that full update received

LCD_2X60::LCD_2X60(const int &fd) :
	_fd(fd)
{
	_messages[SENSORY_STARTUP_MSG] = "Ready.";
	_messages[SENSORY_BATTERY_WARNING_MSG] = "Low battery voltage.";	
}

LCD_2X60::~LCD_2X60()
{
}

int
LCD_2X60::configure(unsigned &baudrate)
{
	set_baudrate(_fd, LCD_2X60_BAUD_RATE);
	return 0;
}

int
LCD_2X60::reset()
{
	send("\n", 1);
	char *msg;
	return receive(msg);	// wait for the ready signal
}

int
LCD_2X60::interrupt()
{
	// Not supported.	
}

int
LCD_2X60::receive(void *msg)
{	
	// Not implemented.
	return -1;
}

int
LCD_2X60::send_msg_id(unsigned long id)
{
	// Filter out the messages we want to have spoken here.
	return send(_messages[id], sizeof(_messages[id]));
}

/**
 * Send a message to the LCD module.
 */
int
LCD_2X60::send(const void *msg, uint8_t size)
{
	return write(_fd, (const char *)msg, size);
}
