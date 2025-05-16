#include <stdio.h>
#include "pico/stdlib.h"

#define BAUD_RATE 1000000

uint8_t result[12];
uint8_t result_index = 0;

int main()
{
        stdio_usb_init();

        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(5, GPIO_FUNC_UART);
        uart_init(uart0, BAUD_RATE);
        uart_init(uart1, BAUD_RATE);

        sleep_ms(100);

        uart_puts(uart0, "Hello I am new");

        sleep_ms(100);

        while(uart_is_readable(uart1)) {
                result[result_index++] = uart_getc(uart1);
        }
        //result[result_index] = '\0';        

        while (true) {
                printf("%s\n", result);
                sleep_ms(500);
        }
}