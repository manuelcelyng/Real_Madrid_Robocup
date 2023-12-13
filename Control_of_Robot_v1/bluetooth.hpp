#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <stdint.h>

#define BUFFER_SIZE 160
#define GPIO_UART_TX 8
#define GPIO_UART_RX 9

// Varibles
extern volatile uint8_t buffer_index;
extern char buffer[BUFFER_SIZE];

// functions
void uart_rx_handler(void);
void initUart(uint8_t gpio_tx ,uint8_t gpio_rx);


#endif // UART_COMMUNICATION_H
