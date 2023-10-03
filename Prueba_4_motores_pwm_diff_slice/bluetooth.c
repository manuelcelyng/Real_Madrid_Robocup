#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdint.h>


//incluyo el .h
#include "bluetooth.h"
#include "sharedfunctions.h"



volatile uint8_t buffer_index = 0;
char buffer[BUFFER_SIZE]= {0};



void uart_rx_handler() {
    char *miCadena = (char *)malloc(20 * sizeof(char));
    irq_clear(UART1_IRQ);

    // uart_read_blocking(uart1, (uint8_t *)buffer, BUFFER_SIZE - 1);
    while (uart_is_readable(uart1)) {    
        char data = uart_getc(uart1);
        // printf("Command: %c\n", data);
        if (data != '\n')
        {
            buffer[buffer_index++] = data;
        } else {
            buffer[buffer_index] = '\0'; 
            //printf("Command: %s\n", buffer);
            strcpy(miCadena,  buffer);
            //printf("La cadena es: %s\n", miCadena);
            moverMotor(miCadena);
            // Puntero al buffer de nuevo al inicio
            buffer_index  = 0;
        }
    }
    
    free(miCadena);
}

void initUart(uint8_t gpio_tx ,uint8_t gpio_rx){
    uart_init(uart1, 9600);  
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);
    gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    sleep_ms(100);

    uart_set_irq_enables(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    sleep_ms(100);
}

