#include <stdio.h>
#include "pico/stdlib.h"

#define UART_ID uart0
#define BAUD_RATE 1000000
#define UART_TX_PIN 0
#define UART_RX_PIN 1

int main()
{
        // Set the GPIO pin mux to the UART - pin 0 is TX, 1 is RX; note use of UART_FUNCSEL_NUM for the general
        // case where the func sel used for UART depends on the pin number
        // Do this before calling uart_init to avoid losing data
        gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
        gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

        uart_init(UART_ID, BAUD_RATE);

        while (true) {
                uart_puts(UART_ID, "Hello world!");
                sleep_ms(1000);
        }

        return 0;
}