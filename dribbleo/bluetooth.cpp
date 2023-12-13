#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "bluetooth.hpp"
#include "dribbleo.hpp"


volatile uint8_t buffer_index = 0;
char buffer[BUFFER_SIZE]= {0};

void moverMotor(char* move, int value1, int value2){
    // strcmp(move, "T")        // Si se quiere comparar una cadena más larga en el if que no sea de un solo caracter
    // Movimiento GIRO, utilizar value1, es el valor del ÁNGULO
    if(move[0] == 'T') {
        printf("GIRO %d\n", value1);
    }

    // Movimiento DESPLAZAMIENTO, utilizar value1, es el valor de la DISTANCIA
    if(move[0] == 'D') {
        printf("DESPLAZAMIENTO %d\n", value1);
    }

    // Movimiento DESPLAZAMIENTO CIRCULAR, utilizar value1 es el RADIO, value2 es el ÁNGULO
    if(move[0] == 'C') {
        printf("DESPLAZAMIENTO CIRCULAR %d, %d\n", value1, value2);
    }

    // Movimiento de DRIBBLIGN, VALUES EN 0, NO USAR
    if(move[0] == 'A') {
        printf("A\n");
        dribbleo->activeDribbleo(true, false);
    }

    // Movimiento de KICK/PATEO, VALUES EN 0, NO USAR
    if(move[0] == 'K') {
        printf("k\n");
        dribbleo->activeDribbleo(false, true);
    }

    // Movimiento de STOP DRIBBLIGN, VALUES EN 0, NO USAR
    if(move[0] == 'G') {
         printf("G\n");
        dribbleo->activeDribbleo(false, false);

    }
}

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
        
        moverMotor(move, value1, value2);
        buffer_index = 0;
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


