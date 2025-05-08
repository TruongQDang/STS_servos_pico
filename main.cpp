#include "STSServoDriver/STSServoDriver.hpp"
#include "pico/stdlib.h"

#define UART_ID uart0
#define BAUD_RATE 1000000
#define UART_TX_PIN 0
#define UART_RX_PIN 1
const uint8_t SERVO_ID = 1;

int main()
{
        STSServoDriver servos;
        gpio_init(25);
        gpio_set_dir(25, GPIO_OUT);
        // Set the GPIO pin mux to the UART - pin 0 is TX, 1 is RX; note use of UART_FUNCSEL_NUM for the general
        // case where the func sel used for UART depends on the pin number
        // Do this before calling uart_init to avoid losing data
        gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
        gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

        uart_init(UART_ID, BAUD_RATE);

        servos.init(UART_ID);

        while (true) {
                if (servos.ping(SERVO_ID)) {
                        gpio_put(25, 1);
                        sleep_ms(500);  
                } else {
                        gpio_put(25, 0);
                        sleep_ms(500);
                }
        }

        return 0;
}