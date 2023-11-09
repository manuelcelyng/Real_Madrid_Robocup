#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"


// include mis propios .h
#include "motor_control.h" // .h definicion de variables y metodos para el control del motor
#include "wheel_control.h"
#include "sharedfunctions.h" // es el que comparte funciones con el codigo bluetooth.c

DutyCycle duty = {750,750,750,750};


void initPWM(uint8_t gpio, uint16_t frec){
    uint slice = pwm_gpio_to_slice_num(gpio);  // Get the value of the slice the gpio belongs to
    uint channel = pwm_gpio_to_channel(gpio);  // Get the value of the CHANNEL (A or B) to wich the gpio belongs in the specific slice.
    assert(frec>=8);                                             ///< PWM can manage interrupt periods greater than 262 milis - 3.5Hz-> see on sdk
    float prescaler = (float)SYS_CLK_KHZ/500;
    assert(prescaler<256); ///< the integer part of the clock divider can be greater than 255. 8bit prescaler
    uint32_t wrap = (500000)/(frec); // if mode phase_correct is actived,it means that the frecuency is divided by 2.
    assert(wrap<((1UL<<16)-1)); // validate wrap less than 16bits -> max is 65536
    pwm_config cfg =  pwm_get_default_config();
    //pwm_config_set_phase_correct(&cfg,true);   // the correct phase mode signal is defined. true
    pwm_config_set_clkdiv(&cfg,prescaler);     // split clock system running at 125MHz to run slower
    pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_FREE_RUNNING);  // mode PMW_DIV_FREE_RUNNING, counts always up to wrap value. no need input b
    pwm_config_set_wrap(&cfg,wrap);   // set the wrap value, calculated based on frecuency of reloj system and frecuency required.
    //pwm_set_irq_enabled(slice,true);  // Enable PWM instance interrupt, each slice need to enabled the pwm irq when is used
    //irq_set_enabled(PWM_IRQ_WRAP,true); //Enable or disable a specific interrupt on the executing core. The same Interrupt is generated by all slices    
    pwm_init(slice,&cfg,true);  // Enables the slice pwm. when the bool value is true.
}


void initMotor(uint8_t PWM_GPIO){
    gpio_set_function(PWM_GPIO, GPIO_FUNC_PWM);// enable GPIO like PMW function.
    pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_GPIO), pwm_gpio_to_channel( PWM_GPIO), 820); // set duty cycle of pwm signal, its choose based on Wrap config
    sleep_ms(3000); 
    pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_GPIO), pwm_gpio_to_channel(PWM_GPIO), 750); 
}

// la que llama el bluetooth
void moverMotor(char* buffer){
    if(buffer[0] =='z'){
        //printf("ENTRO %s", buffer[0]);
        
        // INICIAR EJECUCION 90 GRADOS
       
    }
    if(buffer[0] =='s'){
        //printf("ENTRO %c", buffer[0]);
        
        // INICIAR EJECUCION 180 GRADOS
    }
    if(buffer[0] =='a'){
        //printf("ENTRO %s", buffer[0]);
        
        // MOVIMIENTO LINEA RECTA - VER COMO IMPLEMENTAR (DISTANCIA QUE RECORRE ¿?)
    }
    if(buffer[0] =='b'){
        //printf("ENTRO %c", buffer[0]);
        
        // MOVIMIENTO DE ARCO, DADO EL RADIO Y LOS GRADOS.
    }
}

void setDutyxPID(int one_D, int two_D, int three_D, int four_D){
    pwm_set_gpio_level(PWM_GPIO_MOTOR_ONE, one_D); // RUEDA 1
    pwm_set_gpio_level(PWM_GPIO_MOTOR_TWO, two_D); // RUEDA 2
    pwm_set_gpio_level(PWM_GPIO_MOTOR_THREE, three_D); // RUEDA 3 
    pwm_set_gpio_level(PWM_GPIO_MOTOR_FOUR, four_D);   // RUEDA 4
}


void initMotorControl(){
    initPWM(PWM_GPIO_MOTOR_ONE , FRECUENCY_ALL_PWM_MOTORS);
    if(pwm_gpio_to_slice_num(PWM_GPIO_MOTOR_ONE)!= pwm_gpio_to_slice_num(PWM_GPIO_MOTOR_TWO)){
        initPWM(PWM_GPIO_MOTOR_TWO , FRECUENCY_ALL_PWM_MOTORS);
    }
    initPWM(PWM_GPIO_MOTOR_THREE , FRECUENCY_ALL_PWM_MOTORS);
    if(pwm_gpio_to_slice_num(PWM_GPIO_MOTOR_ONE)!= pwm_gpio_to_slice_num(PWM_GPIO_MOTOR_TWO)){
        initPWM(PWM_GPIO_MOTOR_FOUR , FRECUENCY_ALL_PWM_MOTORS);
    }

    //All motors are initialized one by one. Takes 2 seconds per motor
    initMotor(PWM_GPIO_MOTOR_ONE);
    initMotor(PWM_GPIO_MOTOR_TWO);
    initMotor(PWM_GPIO_MOTOR_THREE);
    initMotor(PWM_GPIO_MOTOR_FOUR);

    
}

void adjustPWM(){
    
    // pid[0] -> RUEDA 1
    // pid[1] -> RUEDA 2
    // pid[2] -> RUEDA 3  
    // pid[3] -> RUEDA 4 
    for(int i = 0 ; i<4; i++){
        duty[i] =  duty[i] + (int)pid[i];
        if (duty[i] > MAX_DUTY)
        {
            duty[i] = MAX_DUTY;
        }else if (duty[i]< MIN_DUTY)
        {
            duty[i] = MIN_DUTY;
        }        
    }

    DutyCycle duty_aux = {750,750,750,750};
    if (duty[0]> 770 || duty[0]<730)
    {
        duty_aux[0] = duty[0];
    }
    if (duty[1]> 770 || duty[1]<730)
    {
        duty_aux[1] = duty[1];
    }
    if (duty[2]> 770 || duty[2]<730)
    {
        duty_aux[2] = duty[2];
    }
    if (duty[3]> 770 || duty[3]<730)
    {
        duty_aux[3] = duty[3];
    }
    setDutyxPID(duty_aux[0], duty_aux[1], duty_aux[2], duty_aux[3]);

}
