#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP 

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define GPIO_UART_TX 8
#define GPIO_UART_RX 9

#define BUFFER_SIZE 160

void moverMotor(char* move, int value1, int value2);

void uart_rx_handler();

void initUart(uint8_t gpio_tx ,uint8_t gpio_rx);

#endif