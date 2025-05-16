#include <stdio.h>
#include "pico/stdlib.h"

#define BAUD_RATE 1000000

void on_uart_rx() 
{
        while (uart_is_readable(uart1)) {
                char in = uart_getc(uart1);
                printf("Received on UART1: %c\n", in);
        }
}

int counter = 0;
int main()
{
        stdio_usb_init();

        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(5, GPIO_FUNC_UART);
        uart_init(uart0, BAUD_RATE);
        uart_init(uart1, BAUD_RATE);
        
        irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
        irq_set_enabled(UART1_IRQ, true);

        uart_set_irqs_enabled(uart1, true, false);

        while (true) {
                char out = 'A' + (counter % 26); // Cycle through 'A' to 'Z'
                printf("Sent: %c\n", out);       // Print sent data to USB serial
                uart_putc(uart0, out);           // Send character
                counter++;
                sleep_ms(500);
        }
}