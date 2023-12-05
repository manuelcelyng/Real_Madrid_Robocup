#ifndef SHARED_FUNCTIONS_H
#define SHARED_FUNCTIONS_H
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
void ejecutarMovimiento(char* move, int value1, int value2);
extern QueueHandle_t xQueueISRBluetooth;
#endif // COMMUNICATION_H