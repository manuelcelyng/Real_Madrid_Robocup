#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define BUFFER_SIZE 256

volatile uint8_t buffer_index = 0;
char buffer[BUFFER_SIZE];

void uart_rx_handler() {
    
    printf("Entro\n");
    irq_clear(UART1_IRQ);

    // uart_read_blocking(uart1, (uint8_t *)buffer, BUFFER_SIZE - 1);
    while (uart_is_readable(uart1)) {    
        char data = uart_getc(uart1);
        // printf("Command: %c\n", data);
        if (data != '-')
        {
            buffer[buffer_index++] = data;
        } else {
            buffer[buffer_index] = '\0'; 
            buffer_index = 0;
            printf("Command: %s\n", buffer);

            for (int i = 0; i < BUFFER_SIZE; i++) {
                buffer[i] = 0;
            }
        }
    }
    
    
}


int main() {
    
    stdio_init_all();
    uart_init(uart1, 9600);  
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);
    sleep_ms(100);

    uart_set_irq_enables(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    sleep_ms(100);
    

    while (true) {
        sleep_ms(1000);
    }
    
    return 0;

}