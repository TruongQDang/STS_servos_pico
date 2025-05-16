#include <stdio.h>
#include "STSServoDriver/STSServoDriver.hpp"
#include "pico/stdlib.h"

#define BAUD_RATE 1000000

STSServoDriver servos;
const uint8_t SERVO_ID = 1;

int main()
{
        stdio_usb_init();

        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(1, GPIO_FUNC_UART);
        uart_init(uart0, BAUD_RATE);

        gpio_init(25);              // Initialize GPIO 25
        gpio_set_dir(25, GPIO_OUT); // Set GPIO 25 as output
        gpio_put(25, 1);
        sleep_ms(2000);
        gpio_put(25, 0);
        sleep_ms(500);

        servos.init(uart0);
        servos.setMode(0xFE, STSMode::POSITION);
        
        while (true) {
                if (!servos.ping(SERVO_ID)) {
                        gpio_put(25, 1);
                }

                // Move servo to 0.
                servos.setTargetPosition(SERVO_ID, 0);
                // Wait for servo to start moving, then wait for end of motion
                sleep_ms(100);
                while (servos.isMoving(SERVO_ID))
                {
                        sleep_ms(50);
                }
                // Wait a bit more to see it stop
                sleep_ms(500);

                // Move to 180deg.
                servos.setTargetPosition(SERVO_ID, 2048);
                sleep_ms(100);
                while (servos.isMoving(SERVO_ID))
                {
                        sleep_ms(50);
                }
                sleep_ms(500);

                // Move to 360deg, at a slower speed (second argument is steps/s).
                servos.setTargetPosition(SERVO_ID, 4095, 500);
                sleep_ms(100);
                while (servos.isMoving(SERVO_ID))
                {
                        sleep_ms(50);
                }
                sleep_ms(500);
        }
}
