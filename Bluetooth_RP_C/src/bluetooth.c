#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define BUFFER_SIZE 256

volatile uint8_t buffer_index = 0;
char buffer[BUFFER_SIZE];

void uart_rx_handler() {
    printf("hola");
    while (uart_is_readable(uart1)) {    
        char data = uart_getc(uart1);
        if (data != '\n')
        {
            buffer[buffer_index++] = data;
        } else {
            buffer[buffer_index] = '\0'; 
            buffer_index = 0;
            printf("Command: %s\n", buffer);
        }
    }

    irq_clear(UART1_IRQ);
}


int main() {
    
    stdio_init_all();
    uart_init(uart1, 9600);  
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);
    sleep_ms(100);

    uart_set_irq_enables(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    sleep_ms(100);
    

    while (true) {
    }
    
    return 0;

}