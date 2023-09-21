#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
 
#define PWM_PIN 15 
 const uint delay = 500; // 1s delay
 

 int main() {
     stdio_init_all();
     printf("ADC Example, measuring GPIO26\n");
 

     adc_init();
     adc_gpio_init(26);
     adc_select_input(0);
    
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Configurar el reloj PWM 50hz
    pwm_set_clkdiv(slice_num, 250.0f); // Establecer el divisor del reloj PWM
    pwm_set_wrap(slice_num, 10000);    // Establecer el valor máximo del contador PWM (10,000 para una frecuencia de 1 kHz)
    pwm_set_enabled(slice_num, true);  // Habilitar el PWM

    

    // CONFIGURACIÓN INICIAL DEL ESC, se comenta una vez que ya corre la primera vez.
    // Indica si quedó bien configurado el hecho de girar en ambos sentidos

    /*
    // punto neutro
    printf("set neutro");
    pwm_set_gpio_level(PWM_PIN, 750);

    sleep_ms(4000);

    printf("set forward Direction"); 

    // punto final de direccion 
    pwm_set_gpio_level(PWM_PIN, 1000);
    sleep_ms(4000);

    pwm_set_gpio_level(PWM_PIN, 500);

    sleep_ms(4000);
    */

    
    
    int duty = 750;
    sleep_ms(3000);
     while (1) {
    
    
        while(duty < 1000){
            duty +=10;
            pwm_set_gpio_level(PWM_PIN, duty);
            sleep_ms(1000);
        }

        while(duty>750){
            duty -=10;
            pwm_set_gpio_level(PWM_PIN, duty);
            sleep_ms(1000);
        }

        while(duty>500){
            duty-=10;
            pwm_set_gpio_level(PWM_PIN, duty);
            sleep_ms(1000);
        }

        while(duty<750){
            duty+=10; 
            pwm_set_gpio_level(PWM_PIN, duty);
            sleep_ms(1000);
        }

        sleep_ms(3000);
        
     }
 }