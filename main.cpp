#include <stdio.h>
#include "STSServoDriver/STSServoDriver.hpp"
#include "pico/stdlib.h"

#define BAUD_RATE 1000000

void on_uart_rx()
{
        while (uart_is_readable(uart0))
        {
                char in = uart_getc(uart0);
                printf("Received on UART1: %c\n", in);
        }
}

STSServoDriver servos;

int main()
{
        stdio_usb_init();

        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(1, GPIO_FUNC_UART);
        uart_init(uart0, BAUD_RATE);

        irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
        irq_set_enabled(UART0_IRQ, true);

        // Fire interrupt for RX only
        uart_set_irqs_enabled(uart0, true, false);
        
        servos.init(uart0);
        uint8_t response[1] = {0xFF};
        while (true) {
                servos.sendMessage(1, 0x01, 0, response);
                sleep_ms(300);
        }
}

