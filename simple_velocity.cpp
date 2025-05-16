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
        for (int i = 0; i < 3; i++) {
                gpio_put(25, 1);
                sleep_ms(300);
                gpio_put(25, 0);
                sleep_ms(300);
        }

        sleep_ms(2000);

        servos.init(uart0);
        servos.setMode(0xFE, STSMode::VELOCITY);
        servos.setTargetVelocity(SERVO_ID, 3000);

        while (true)
        {
                if (servos.isMoving(SERVO_ID)) {
                        gpio_put(25, 1);
                } else {
                        gpio_put(25, 0);
                }
        }
}
