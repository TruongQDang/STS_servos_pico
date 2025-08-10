// This code will simply make the servo move in circle from (0, 180, 360)deg.
// Uart1 is used on Pico to communicate with driver board

#include <stdio.h>

#include "STSServoDriver/STSServoDriver.hpp"
#include "pico/stdlib.h"

#define BAUD_RATE 1000000

STSServoDriver servos;
const uint8_t SERVO_ID = 1;

int main()
{
	stdio_usb_init();

	gpio_set_function(8, GPIO_FUNC_UART);
	gpio_set_function(9, GPIO_FUNC_UART);
	uart_init(uart1, BAUD_RATE);

	gpio_init(25);
	gpio_set_dir(25, GPIO_OUT);
	gpio_put(25, 0);

	if (!servos.init(uart1)) {
		// Failed to get a ping reply, turn on the led.
		gpio_put(25, 1);
	}

	// Reset all servos to position mode: servos have three modes (position,
	// velocity, step position).
	// Position is the default mode so this shouldn't be needed but it's
	// here just to make sure (depending on what you've run before, the
	// servos could be in a different mode).
	// 0xFE is broadcast address and applies to all servos.
	servos.setMode(0xFE, STSMode::POSITION);

	while (true) {
		// Move servo to 0.
		servos.setTargetPosition(SERVO_ID, 0);
		// Wait for servo to start moving, then wait for end of motion
		sleep_ms(100);
		while (servos.isMoving(SERVO_ID)) {
			sleep_ms(50);
		}
		// Wait a bit more to see it stop
		sleep_ms(500);

		// Move to 180deg.
		servos.setTargetPosition(SERVO_ID, 2048);
		sleep_ms(100);
		while (servos.isMoving(SERVO_ID)) {
			sleep_ms(50);
		}
		sleep_ms(500);

		// Move to 360deg, at a slower speed (second argument is
		// steps/s).
		servos.setTargetPosition(SERVO_ID, 4095, 500);
		sleep_ms(100);
		while (servos.isMoving(SERVO_ID)) {
			sleep_ms(50);
		}
		sleep_ms(500);
	}
}
