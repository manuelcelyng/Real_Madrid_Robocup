#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdint.h>


//incluyo el .h
#include "bluetooth.h"
#include "control.h"




volatile uint8_t buffer_index = 0;
char buffer[BUFFER_SIZE]= {0};

// QUITAR EL MALLOC Y FREE DE LA ISR

void uart_rx_handler() {
    irq_clear(UART1_IRQ);
    char data;
   
    while (uart_is_readable(uart1)) {    
        
        while ((data = uart_getc(uart1)) != '\n') {        
            buffer[buffer_index++] = data;
        }
        buffer[buffer_index] = '\0';
        
        // Estructura recibida move;value1;value2
        // La función strtok es un puntero que busca un delimitador en este caso ;, y almacena este todo el tamaño desde donde empieza hasta el delimitador
        char *move = strtok(buffer, ";");
        
        // La función strtok guardainternamente donde terminó en la anterior busqueda por lo que se manda el parametro nulo y el delimitador
        int value1 = atoi(strtok(NULL, ";"));       // Como se sabe que son números, se convierte a entero, si se quiere double >> double numero = strtod(strtok(buffer, ";"), NULL);
        int value2 = atoi(strtok(NULL, ";"));
        
        ejecutarMovimiento(move, value1, value2);
        buffer_index = 0;
        irq_set_enabled(UART1_IRQ, false);
    }
}

void initUart(uint8_t gpio_tx ,uint8_t gpio_rx){
    uart_init(uart1, 9600);  
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);
    gpio_set_function(gpio_tx, GPIO_FUNC_UART);
    gpio_set_function(gpio_rx, GPIO_FUNC_UART);
    sleep_ms(100);

    uart_set_irq_enables(uart1, true, false);
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_handler);
    irq_set_enabled(UART1_IRQ, true);
    sleep_ms(100);
}

