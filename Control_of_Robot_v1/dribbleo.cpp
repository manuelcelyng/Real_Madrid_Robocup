#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "dribbleo.hpp"

Dribbleo::Dribbleo() {
    // Inciailización de GPIO para motor
    gpio_init(MOTOR_CONTROL_1_PIN);
    gpio_init(MOTOR_CONTROL_2_PIN);
    gpio_init(MOTOR_ENABLE);                // Habilitar el control del puente H

    gpio_set_dir(MOTOR_CONTROL_1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_CONTROL_2_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_ENABLE, GPIO_OUT);

    // Se inicializa el motor apagado y se habilita el enable
    gpio_put(MOTOR_CONTROL_1_PIN, false);
    gpio_put(MOTOR_CONTROL_2_PIN, false);
    gpio_put(MOTOR_ENABLE, true);
}

void Dribbleo::activeDribbleo(bool value1, bool value2){
    gpio_put(MOTOR_CONTROL_1_PIN, value1);
    gpio_put(MOTOR_CONTROL_2_PIN, value2);
}