#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/sync.h"

// .h propios.
#include "motor_control.h"
#include "control.h"
#include "sharedfunctions.h"
#include "wheel_control.h"
#include "bluetooth.h" // .h definicion de todo lo relacionado a la comunicación bluetooth
#include "imu.h"


semaphore_t sem1;
semaphore_t sem2;

void main2();

int main(){
    stdio_init_all();

      // Inicializar los semáforos
    sem_init(&sem1, 0, 1); // Inicializa sem1 con 1 permiso
    sem_init(&sem2, 1, 1); // Inicializa sem2 con cero permisos
    // Initialize motor control _ init motors with pwm for stop and init bluetooth
    initMotorControl();
    sleep_ms(3000);


    // Initialize I2C and check magnetPresent in encoders
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state for all encoders, afte it , i2c encoders 2 and 4 are active

    mpu6050_reset();  // IMU IN I2C0

    initUart(GPIO_UART_TX, GPIO_UART_RX); // initialize BLUETOoth
    
    // inicializo el otro core 
    multicore_launch_core1(main2); 

    //Imu variables
    int16_t acceleration[3];
    int16_t gyro[3] = {0,0,0};
    int32_t tiempo_prev = 0;
    int32_t dt;
    long double ang_z = 0;
    long double ang_z_prev = 0;


    //control space
    float delta = 0.01;
    float q[3][1] = {{0}, {0}, {0}};
    float T = 30;
    float b = 2 * 3.141592653589793 / T;
    float ek[3][1] = {{0}, {0}, {0}};
    float U[3][1] = {{0}, {0}, {0}};
    float uk[3][1] = {{0}, {0}, {0}};
    float e[3][1] = {{0}, {0}, {0}};
    uint64_t offset_time = time_us_64();

    while (1) {
        tight_loop_contents();
        

        dt = (time_us_32()-tiempo_prev);
        tiempo_prev=time_us_32();

        //Calcular angulo de rotación con giroscopio  
        ang_z = gyro[2]*dt;
        ang_z /= 131072000.0;
        ang_z += ang_z_prev;
        ang_z_prev=ang_z;
        printf("Rotacion en Z:  ");
        printf("%f  \n", ang_z);

        float t_i = (time_us_64()-offset_time)*1e-6;

        // Calculate qd
        float qd[3][1];
        qd[0][0] = 0; //2*sinf(b * t_i);
        qd[1][0] = 4*sinf(b * t_i);
        qd[2][0] = 0;

        // Update ek
        for (int j = 0; j < 3; j++) {
            ek[j][0] = e[j][0];
        }

        q[2][0] = ang_z;

        // Calculate e
        for (int j = 0; j < 3; j++) {
            e[j][0] = qd[j][0] - q[j][0];
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

        printf("%f,%f,%f,%f\n",dteta[0][0], dteta[1][0], dteta[2][0], dteta[3][0]);
        for (int j = 0; j < 4; j++) {
            if (-150< dteta[j][0] && dteta[j][0] < 150)
            {
                dteta[j][0] = 0;
            }
        }
        printf("%f,%f,%f,%f\n",dteta[0][0], dteta[1][0], dteta[2][0], dteta[3][0]);
        
        mpu6050_read_raw(acceleration, gyro);
        sem_acquire_blocking(&sem1); // Adquirir el semáforo sem1

        desiredSpeed[0] = dteta[0][0];  // RUEDA 1
        desiredSpeed[1] = dteta[2][0];  // RUEDA 2
        desiredSpeed[2] = dteta[3][0];  // RUEDA 3
        desiredSpeed[3] = dteta[1][0];  // RUEDA  4

        sem_release(&sem2);
     }

    return 0;
}

// wait for init from main or core 0
// CORE 1
void main2() {

    // desiredSpeed[0] = 200;  // RUEDA 1
    // desiredSpeed[1] = -200;  // RUEDA 3
    // desiredSpeed[2] = 0;  // RUEDA 2 
    // desiredSpeed[3] = 0; // RUEDA 4

    AngleData startAngle; // struct with all start angles
    // measure the time in microseconds with function time_us_64()
    uint64_t current_micros = 0;
    uint64_t previous_micros = 0;
    uint64_t ayudamedios = 0;

    // to control when the wheel is stopped
    double previousAngle = 0;

    

    while(true){
        
        sem_acquire_blocking(&sem2); // Adquirir el semáforo sem2
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
            // Inicializo los tiempos
            previous_micros =  time_us_64();
            ayudamedios = time_us_64();
             // start angle offset
            obtainAngle(0);
            startAngle[i] = correctedAngle;
           
            while (true)
            {
                current_micros  = time_us_64();
                
                if(((current_micros - ayudamedios)) >= SAMPLING_TIME){
                    ayudamedios = current_micros;
                    previousAngle = correctedAngle;
                    obtainAngle(startAngle[i]);   
                }
                
                if((current_micros - previous_micros) >= TIME_WINDOW_US){
                    // Garantizar que cuando no se está moviendo el robot, el angulo calculado sea exactamente 0
                    previousAngle =  correctedAngle-previousAngle;
                    if(previousAngle<0){
                        previousAngle= -1*previousAngle;
                    }
                    if( previousAngle<=1   ||  (previousAngle)>=359 && numberTurns==0 ){
                        speedData[i] = 0;
                    }else{
                        
                        if(duty[i]>750){
                            correctedAngle =  360-correctedAngle;
                            speedData[i] = -1*((double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                        }else{
                            speedData[i] = (double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S);
                        }
                    }
                    // out of while and continue with for with another encoder
                    break;
                }
            }
            
            
        }

        
        calcularControlPID();  
        adjustPWM();
        sem_release(&sem1); // Liberar el semáforo sem1
        
        

    } // end of while(1)
}
