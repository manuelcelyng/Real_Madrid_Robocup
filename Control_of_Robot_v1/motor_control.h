#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

/*
 // ORDEN RUEDAS
    1 |  2
    ---|---
    4  | 3
    */

#define PWM_GPIO_MOTOR_ONE 0    // PIN 1  - RUEDA 1
#define PWM_GPIO_MOTOR_TWO 2    // PIN 4  - RUEDA 2
#define PWM_GPIO_MOTOR_THREE 4  // PIN 6  - RUEDA 3
#define PWM_GPIO_MOTOR_FOUR  6  // PIN 9  - RUEDA 4
#define FRECUENCY_ALL_PWM_MOTORS 50

#define MAX_DUTY 800
#define MIN_DUTY 680

// vector for duty
typedef double DutyCycle[4];
extern DutyCycle duty;

// functions
void initPWM(uint8_t gpio, uint16_t frec);
void initMotor(uint8_t PWM_GPIO);
void initMotorControl();
void adjustPWM();
void moverMotor(char* move, int value1, int value2);

#endif // MOTOR_CONTROL_H
