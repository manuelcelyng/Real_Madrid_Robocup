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
#include "sharedfunctions.h"
#include "wheel_control.h"
#include "bluetooth.h" // .h definicion de todo lo relacionado a la comunicación bluetooth
#include "imu.h"




semaphore_t sem1;
semaphore_t sem2;

// Mutex para proteger el acceso a la variable compartida
static mutex_t my_mutex ;
static mutex_t my_mutex2;

// gyro de la IMU y bandera
int16_t gyro = 0;
volatile bool timer_fired = false;
volatile bool timer_fired2 = false;
volatile bool timer_fired21 = false;

//Functions core 1 and timers callback
void main2();

bool repeating_timer_callback(struct repeating_timer *t) {
    mutex_enter_blocking(&my_mutex2);
    mpu6050_read_raw(&gyro);
    mutex_exit(&my_mutex2);
    timer_fired =  true;

    return true;
}

bool repeating_timer_callback2(struct repeating_timer *t) {
    timer_fired2 =  true;
    printf("entra timer\n");
    return true;
}
bool repeating_timer_callback21(struct repeating_timer *t) {
    timer_fired21 =  true;

    return true;
}


int main(){
    stdio_init_all();

      // Inicializar los semáforos
    // sem_init(&sem1, 0, 1); // Inicializa sem1 con 1 permiso
    // sem_init(&sem2, 1, 1); // Inicializa sem2 con cero permisos
    mutex_init(&my_mutex);
    mutex_init(&my_mutex2);

    // Initialize motor control _ init motors with pwm for stop and init bluetooth
    initMotorControl();
    sleep_ms(3000);
    // Initialize I2C and check magnetPresent in encoders
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state for all encoders, afte it , i2c encoders 2 and 4 are active
    // reset IMU 
    mpu6050_reset();  // IMU IN I2C0
   
    for (size_t i = 0; i < 40; i++)
    {
        mpu6050_read_raw(&gyro);
        sleep_ms(5);
    }

    //initUart(GPIO_UART_TX, GPIO_UART_RX); // initialize BLUETOoth

    // inicializo el otro core 
    multicore_launch_core1(main2); 

    // CONTROL PID - ROBOT 
      //control space
    float delta = 0.001;
    float q[3][1] = {{0}, {0}, {0}};
    float T = 30;
    float b = 2 * 3.141592653589793 / T;
    float ek[3][1] = {{0}, {0}, {0}};
    float U[3][1] = {{0}, {0}, {0}};
    float uk[3][1] = {{0}, {0}, {0}};
    float e[3][1] = {{0}, {0}, {0}};
    uint64_t offset_time = time_us_64();
    
   
    //Imu variables
    int32_t tiempo_prev = 0;
    int32_t dt;
    long double ang_z_prev = 0;
    long double ang_z = 0;

    // DEFINE THE REPEATING TIMER FOR IMU
    struct repeating_timer timer;
    add_repeating_timer_ms(IMU_INTERVAL_TIMER_MS, repeating_timer_callback, NULL, &timer);

    while (1) {
        
      

        // dt = (time_us_32()-tiempo_prev);
        // tiempo_prev = time_us_32();

       

        // Wait for alarm callback to set timer_fired
        while (!timer_fired)
        {
            tight_loop_contents();
        }
        timer_fired =  false;

        //Calcular angulo de rotación con giroscopio
        if(gyro > 60 || gyro < -60){ 
            ang_z = gyro*IMU_INTERVAL_TIMER_US;
            //ang_z /= 16400000.0;
            ang_z /= 939650784.0;
            ang_z += ang_z_prev;
            ang_z_prev=ang_z;
        }
       

        // CONTROL GENERAL DEL CARRO

        float t_i = (time_us_64()-offset_time)*1e-6;

        // Calculate qd
        float qd[3][1];
        qd[0][0] = q[0][0];//-4*sinf(b * t_i)*sinf(b * t_i);
        qd[1][0] = q[1][0]+0.03 ;//2*sinf(b * t_i);
        qd[2][0] = 0;

        // Update ek
        for (int j = 0; j < 3; j++) {
            ek[j][0] = e[j][0];
        }

        // ang_Z es la variable que calcula la IMU  TOMA MUTEX
        //printf("ang: %f\n", ang_z);
        q[2][0] = ang_z;
        

        // Calculate e
        for (int j = 0; j < 3; j++) {
            e[j][0] = qd[j][0] - q[j][0];
        }

        // CAMBIAMOS LAS CONSTANTES DEL PID
        for(int i = 0 ; i<4 ; i++){
            if(e[2][0]<0) {
                constansP[i] = ((2*PI + e[2][0])/5*PI) + constansP_C[i];
                //constansI[i] = ((2*PI + e[2][0])/15*PI) + constansI_C[i];
                //constansD[i] = ((2*PI + e[2][0])/15*PI) + constansD_C[i];
            }else {
                constansP[i] = ((2*PI - e[2][0])/5*PI) + constansP_C[i];
                //constansI[i] = ((2*PI - e[2][0])/15*PI) + constansI_C[i];
                //constansD[i] = ((2*PI - e[2][0])/15*PI) + constansD_C[i];
            }
        }

        // Call the control function to update U
        uk[0][0] = U[0][0];
        uk[1][0] = U[1][0];
        uk[2][0] = U[2][0];
        control(e, ek, q, uk, U);

        // Call the planta function to calculate dq and dteta
        float dq[3][1];
        float dteta[4][1];
        planta(U, q, dq, dteta);

        // Update q
        for (int j = 0; j < 2; j++) {
            q[j][0] += dq[j][0] * delta;
        }

        //printf("%f,%f,%f,%f\n",dteta[0][0], dteta[1][0], dteta[2][0], dteta[3][0]);
        /*for (int j = 0; j < 4; j++) {
            if (-100< dteta[j][0] && dteta[j][0] < 100)
            {
                dteta[j][0] = 0;
            }
        }*/
        //printf("%f,%f,%f,%f\n",dteta[0][0], dteta[1][0], dteta[2][0], dteta[3][0]);
        

     
        
        mutex_enter_blocking(&my_mutex);
        desiredSpeed[0] = dteta[0][0];  // RUEDA 1
        desiredSpeed[1] = dteta[3][0];  // RUEDA 2
        desiredSpeed[2] = dteta[2][0];  // RUEDA 3
        desiredSpeed[3] = dteta[1][0];  // RUEDA 4
        mutex_exit(&my_mutex);
        
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
    double previousAngle = 0;
    // for calculate start angle offset in the first sample
    bool startAngle_bool =  false;

    // DEFINE THE REPEATING TIMER FOR CALCULATE THE SPEED ANGULAR
    struct repeating_timer timer;
    struct repeating_timer timer2;
    //add_repeating_timer_us(SAMPLING_TIME, repeating_timer_callback2, NULL, &timer);
    //add_repeating_timer_us(ENCODER_TIMER_TOTAL_VALUE_US, repeating_timer_callback21, NULL, &timer2);

    while(true){

        
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

            // Reinicio variables globales necesarias
            numberTurns = 0;
            flagHelp  = 0;
            previousQuadrantNumber = 0;
            correctedAngle = 0;
            startAngle_bool =  true;
            add_repeating_timer_us(SAMPLING_TIME, repeating_timer_callback2, NULL, &timer);
            // printf("RUEDA :%d \n", i+1);
            while (true)
            {
                
                if(timer_fired2){
                    previousAngle = correctedAngle;
                    // MUTEX FOR I2C
                    mutex_enter_blocking(&my_mutex2);
                    obtainAngle(startAngle[i],startAngle_bool);
                    mutex_exit(&my_mutex2);  
                    if(startAngle_bool){
                        startAngle[i] = correctedAngle;
                        add_repeating_timer_us(TIME_WINDOW_US, repeating_timer_callback21, NULL, &timer2);
                        startAngle_bool = false;
                    }
                    timer_fired2 = false;
                }
                
                if(timer_fired21){
                    cancel_repeating_timer(&timer);
                    // Garantizar que cuando no se está moviendo el robot, el angulo calculado sea exactamente 0
                    previousAngle =  correctedAngle-previousAngle;
                    if(previousAngle<0){
                        previousAngle= -1*previousAngle;
                    }
                    if( previousAngle<=0.1   ||  (previousAngle)>359.9 ){
                        speedData[i] = 0;
                       
                    }else{
                        
                        if(duty[i]>750){
                            correctedAngle =  360-correctedAngle;
                            speedData[i] = -1*((double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                        }else{
                            speedData[i] = (double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S);
                        }
                    }

                    //printf("speedData RUEDA %d : %f \n", i+1 ,  speedData[i] );

                    cancel_repeating_timer(&timer2);
                    timer_fired21 =  false;

                    // Ajusto la velocidad de una vez;
                    mutex_enter_blocking(&my_mutex);
                    calcularControlPID(i);  
                    mutex_exit(&my_mutex);
                    adjustPWM(i);
                    // out of while and continue with for with another encoder

                    break;
                }
            }
            
            
        }
        // mutex_enter_blocking(&my_mutex);
        // calcularControlPID();  
        // mutex_exit(&my_mutex);
        

        sleep_ms(10);
        

    } // end of while(1)
}

