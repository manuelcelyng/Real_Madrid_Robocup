#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

// .h propios.
#include "motor_control.h"
#include "control.h"
#include "wheel_control.h"
#include "bluetooth.h" // .h definicion de todo lo relacionado a la comunicación bluetooth
#include "imu.h"



// para habilitar el control del otro core
bool run_control_wheels = false;

// Mutex para proteger el acceso a la variable compartida
static mutex_t my_mutex ;
static mutex_t my_mutex2;

// gyro de la IMU y bandera
int16_t gyro = 0;
int16_t acceleration[3] = {0,0,0};
volatile bool timer_fired = false;
volatile bool timer_fired2 = false;
volatile bool timer_fired21 = false;

//Functions core 1 and timers callback
void main2();
bool repeating_timer_callback(struct repeating_timer *t);
bool repeating_timer_callback2(struct repeating_timer *t);
bool repeating_timer_callback21(struct repeating_timer *t);

bool repeating_timer_callback(struct repeating_timer *t) {
    mutex_enter_blocking(&my_mutex2);
    mpu6050_read_raw(&gyro, acceleration);
    mutex_exit(&my_mutex2);
    timer_fired =  true;

    return true;
}

bool repeating_timer_callback2(struct repeating_timer *t) {
    timer_fired2 =  true;
    return true;
}
bool repeating_timer_callback21(struct repeating_timer *t) {
    timer_fired21 =  true;

    return true;
}


int main(){
    stdio_init_all();
    // defino los mutexes necesarios para la correcta ejecución del programa.
    mutex_init(&my_mutex);
    mutex_init(&my_mutex2);
    // Initialize motor control _ init motors with pwm for stop and init bluetooth
    initMotorControl();
    sleep_ms(3000);
    // Initialize I2C and check magnetPresent in encoders
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state for all encoders, afte it , i2c encoders 2 and 4 are active
    // reset IMU  - OFFSET ANGULO 0
    mpu6050_reset();  // IMU IN I2C0
    // For necesario para las primeras lecturas erroneas de la IMU.
    for (size_t i = 0; i < 40; i++)
    {
        mpu6050_read_raw(&gyro, acceleration);
        sleep_ms(5);
    }
    // Inicializa el Bluetooth
    initUart(GPIO_UART_TX, GPIO_UART_RX);
    // inicializo el otro core 
    multicore_launch_core1(main2); 
    // CONTROL PID - ROBOT 
    //control space
    float delta = 0.001;
    float q[3][1] = {{0}, {0}, {0}};
    float T = 30;
    float b = 2 * 3.141592653589793 / T;
    float ek[3][1] = {{0}, {0}, {0}};
    float ek2[3][1] = {{0}, {0}, {0}};
    float U[3][1] = {{0}, {0}, {0}};
    float uk[3][1] = {{0}, {0}, {0}};
    float e[3][1] = {{0}, {0}, {0}};

    uint64_t offset_time = time_us_64();

    //Imu variables
    int32_t tiempo_prev = 0;
    int32_t dt;
    long double ang_z_prev = 0;
    long double ang_z = 0;
    long double vel = 0;
    long double dist = 0;
    // Calculate qd
    float qd[3][1];
    qd[0][0] = 0;//q[0][0];//-4*sinf(b * t_i)*sinf(b * t_i);
    qd[1][0] = 0;//q[1][0]+0.03 ;//2*sinf(b * t_i);
    qd[2][0] = 0;

    // DEFINE THE REPEATING TIMER FOR IMU
    struct repeating_timer timer;
    

    while (1)
    {
        if(run_command){
            while (1) {
                // Wait for alarm callback to set timer_fired
                while (!timer_fired)
                {
                    tight_loop_contents();
                } // while
                timer_fired =  false;

                //Calcular angulo de rotación con giroscopio
                if(gyro > 60 || gyro < -60){ 
                    ang_z = gyro*IMU_INTERVAL_TIMER_US;
                    //ang_z /= 16400000.0;
                    ang_z /= 939650784.0;
                    ang_z += ang_z_prev;
                    ang_z_prev=ang_z;
                } // if
                // if (acceleration[0] > 60 || acceleration[0] < -60)                {
                //     vel += acceleration[0] * (9.81/4096.0)*0.005;
                //     dist += vel*0.005;
                //     printf("acce: %d\n", acceleration[0]);
                //     printf("vel: %f\n", vel);
                //     printf("dist: %f\n", dist);
                // }
                
                
                //float ay_m_s2 = acceleration[1];// * (9.81/16384.0);
                
                //printf("ay: %f\n", ay_m_s2);            

                // CONTROL GENERAL DEL CARRO

                float t_i = (time_us_64()-offset_time)*1e-6;

                if(select_movement == 1){
                    qd[2][0] += TO_RAD(value1,0);
                    select_movement = 0;
                }else if(select_movement == 2){
                    qd[0][0] += value2;
                    select_movement = 0;
                }

                // Update ek
                for (int j = 0; j < 3; j++) {
                    ek2[j][0] = ek[j][0];
                    ek[j][0] = e[j][0];
                }

                // ang_Z es la variable que calcula la IMU  TOMA MUTEX
                //printf("ang: %f\n", ang_z);
                q[2][0] = ang_z;
                

                // Calculate e
                for (int j = 0; j < 3; j++) {
                    e[j][0] = qd[j][0] - q[j][0];
                } // for

                // CAMBIAMOS LAS CONSTANTES DEL PID
                // printf("%f \n" , q[2][0]);
                for(int i = 0 ; i<4 ; i++){
                    constansP[i] = 10*exp(-1*e[2][0]*e[2][0]) + constansP_C[i];
                    //constansI[i] = 0.05*exp(-1*e[2][0]*e[2][0]) + constansI_C[i];
                    //constansD[i] = 0.5*exp(-1*e[2][0]*e[2][0]) + constansD_C[i];
                } // for

                // Call the control function to update U
                uk[0][0] = U[0][0];
                uk[1][0] = U[1][0];
                uk[2][0] = U[2][0];
                control(e, ek, ek2, q, uk, U);

                // Call the planta function to calculate dq and dteta
                float dq[3][1];
                float dteta[4][1];
                planta(U, q, dq, dteta);

                // Update q
                for (int j = 0; j < 2; j++) {
                    q[j][0] += dq[j][0] * delta;
                } // for

                
                mutex_enter_blocking(&my_mutex);
                if(!run_command){
                    cancel_repeating_timer(&timer);
                    mutex_exit(&my_mutex);
                    
                    break;
                }
                desiredSpeed[0] = dteta[0][0];  // RUEDA 1
                desiredSpeed[1] = dteta[1][0];  // RUEDA 2
                desiredSpeed[2] = dteta[2][0];  // RUEDA 3
                desiredSpeed[3] = dteta[3][0];  // RUEDA 4

                //printf("%f, %f, %f, %f \n", desiredSpeed[0], desiredSpeed[1], desiredSpeed[2], desiredSpeed[3]);
                if(e[0][0] > -0.1 && e[0][0] < 0.1 && e[1][0] > -0.1 && e[1][0] < 0.1 && e[2][0] > -0.1 && e[2][0] < 0.1){
                    run_control_wheels = false;
                    // desiredSpeed[0] = 0;  // RUEDA 1
                    // desiredSpeed[1] = 0;  // RUEDA 2
                    // desiredSpeed[2] = 0;  // RUEDA 3
                    // desiredSpeed[3] = 0;  // RUEDA 4
                    
                    // printf("%f, %f, %f \n", e[0][0], e[1][0], e[2][0]);
                }
                // printf("q0 : %f  - q1: %f \n", q[0][0], q[1][0]);
               
                
                mutex_exit(&my_mutex);
            } // while
        } // if
        else{
            irq_set_enabled(UART1_IRQ, true);
            //printf("Habilita interrupcion \n");
            __wfi();
            if(run_command){
                // qd[0][0] = 0;
                // qd[1][0] = 0;
                // qd[2][0] = 0;
                // q[0][0] = 0;
                // q[1][0] = 0;
                // q[2][0] = 0;
                // ang_z = 0;
                irq_set_enabled(UART1_IRQ, false);
                run_control_wheels = true;
                add_repeating_timer_ms(IMU_INTERVAL_TIMER_MS, repeating_timer_callback, NULL, &timer);
                __sev(); // Lanza el evento
            }
           
             
        }
        
    }
    
    

    return 0;
}

// wait for init from main or core 0
// CORE 1
void main2() {

    // desiredSpeed[0] = -100;
    // desiredSpeed[1] = -100;
    // desiredSpeed[2] = -100;
    // desiredSpeed[3] = -100;

    AngleData startAngle; // struct with all start angles
    // to control when the wheel is stopped
    double previousAngle = 0; // task sample
    int valid_quadrant = 0;
    int corrected_encoder_error = 0; // task sample
    bool flag_sample = false; // task sample
    // for calculate start angle offset in the first sample
    bool startAngle_bool =  false; // task sample

    // DEFINE THE REPEATING TIMER FOR CALCULATE THE SPEED ANGULAR
    struct repeating_timer timer;
    struct repeating_timer timer2;
    alarm_pool_t *pool11=  alarm_pool_create(1,2);
    //add_repeating_timer_us(SAMPLING_TIME, repeating_timer_callback2, NULL, &timer);
    //add_repeating_timer_us(ENCODER_TIMER_TOTAL_VALUE_US, repeating_timer_callback21, NULL, &timer2);

    // for(int f= 0 ; f<4 ; f++){
    //     duty[f] = 740;
    // }

    while(true){
        
        if(run_control_wheels){

            while (true)
            {
                for(int i = 0 ; i<4; i++){

                    switch(i)
                    {
                    case 0:
                        switchI2c(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3);
                        break;
                    case 1:
                        switchI2c(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0);
                        break;
                    case 2:
                        switchI2c(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1);
                        break;
                    case 3: 
                        switchI2c(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2);
                        break;
                    }  

                    // Reinicio variables 
                    numberTurns = 0; // cvaptura el numero de vueltas de la llanta
                    flagHelp  = 0; // ayuda para el correcto calculo de las vueltas
                    previousQuadrantNumber = 0; //  ayuda para el correcto calculo de las vueltas
                    correctedAngle = 0; // es donde se guarda el angulo en cada muestreo
                    startAngle_bool =  true; // para indicar que la primera muestra es un offset
                    corrected_encoder_error = 0; // para validar que las muestras sean coherentes con el giro de la rueda
                    flag_sample = false; // para definir que despues del offset se inicia con el calculo del corrected_encoder_error

                    //Inicializo el timer de muestreo 
                    //add_repeating_timer_us(SAMPLING_TIME, repeating_timer_callback2, NULL, &timer);
                    alarm_pool_add_repeating_timer_us(pool11,SAMPLING_TIME, repeating_timer_callback2, NULL, &timer);
                
                    while (true)
                    {
                        // if el cual toma las muestras y otros calculos que validan la dirección de la rueda
                        if(timer_fired2){
                            previousAngle = correctedAngle;
                            // MUTEX FOR I2C
                            mutex_enter_blocking(&my_mutex2);
                            obtainAngle(startAngle[i],startAngle_bool);
                            mutex_exit(&my_mutex2);  
                            if(startAngle_bool){
                                startAngle[i] = correctedAngle;
                                alarm_pool_add_repeating_timer_us(pool11,TIME_WINDOW_US, repeating_timer_callback21, NULL, &timer2);
                                startAngle_bool = false;
                            }else{
                                if(!flag_sample){
                                    flag_sample = true;
                                }else{
                                    if((correctedAngle - previousAngle)<0){
                                        corrected_encoder_error-=1;  // disminuye el angulo - pwm >750 , antihorario
                                    }else{ 
                                        corrected_encoder_error+=1; //  aumenta el angulo - pwm <750 , horario
                                    }
                                }
                            }
                            timer_fired2 = false;
                            
                            // printf(" R %d: %f \n",i+1, correctedAngle);
                            
                        }
                        
                        // if de la ventana de tiempo durante la cual se tomarán muestras.
                        if(timer_fired21){
                            cancel_repeating_timer(&timer);
                            cancel_repeating_timer(&timer2);
                            
                            valid_quadrant = ((int)(correctedAngle/90) + 1) -  ((int)(previousAngle/90) + 1);
                            previousAngle =  correctedAngle-previousAngle;
                            //previosAngle_direction = previousAngle;

                            // Garantizar que cuando no se está moviendo el robot, el angulo calculado sea exactamente 0
                            if(previousAngle<0){
                                previousAngle= -1*previousAngle;
                            }
                            if( previousAngle<=0.2   ||  (previousAngle)>359.8 ){
                                speedData[i] = 0;
                            }else{
                                // ELIGE DEPENDIENDO LA DIRECCION DE LA VELOCIDAD
                                //if( valid_quadrant==3 || corrected_encoder_error<0)
                                if( valid_quadrant==3 || corrected_encoder_error<0){
                                    correctedAngle =  360-correctedAngle;
                                    speedData[i] = -1*((double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                                }else if(correctedAngle<340){
                                    speedData[i] = (double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S);
                                }else{
                                    correctedAngle =  360-correctedAngle;
                                    speedData[i] = -1*((double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                                }
                            }
                        
                            // printf("angulo R %d: %f \n",i+1, correctedAngle);
                            // printf("speedData RUEDA %d : %f \n", i+1 ,  speedData[i] );

                            
                            timer_fired21 =  false;

                            // Ajusto la velocidad de una vez;
                            mutex_enter_blocking(&my_mutex);
                            calcularControlPID(i);  
                            mutex_exit(&my_mutex);
                            adjustPWM(i);
                            // out of while and continue with for with another encoder

                            break;
                        } // if
                    }// end While _ calcula velocidad angular
                
                }// end For de 0 a 3, para calcular la velocidad de cada una de las ruedas
        
                mutex_enter_blocking(&my_mutex);
                if(!run_control_wheels){
                    for(int i = 0 ; i<4; i++){
                        // printf("error_previo R %d : %f\n",i+1,pidPreviousError[i]);
                        if(desiredSpeed[i] < 1 && desiredSpeed[i] > -1){
                            if(i==3){
                                run_control_wheels = false;
                                run_command =  false;
                            }
                        }
                        else{
                            run_control_wheels = true;
                            break;
                        }
                    }
                    if(!run_control_wheels){
                        for(int i = 0; i<4 ; i++){
                            pid[i] = 0;
                            duty[i] = 750;
                            adjustPWM(i);
                        }
                        mutex_exit(&my_mutex);
                        break;
                    }
                }  
                mutex_exit(&my_mutex);

               

             

            }// end while_control_wheel
            


        }// end if

        else{
            __wfe();
            
        }  

    } // end of while(1)
}

