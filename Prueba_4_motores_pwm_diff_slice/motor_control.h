#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#define PWM_GPIO_MOTOR_ONE 0 // PIN 1
#define PWM_GPIO_MOTOR_TWO 2 // PIN 4
#define PWM_GPIO_MOTOR_THREE 4 //PIN 6
#define PWM_GPIO_MOTOR_FOUR  6  // PIN 9
#define FRECUENCY_ALL_PWM_MOTORS 50


// functions
void initPWM(uint8_t gpio, uint16_t frec);
void initMotor(uint8_t PWM_GPIO);

#endif // MOTOR_CONTROL_H
