#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

// include libraries
#include "imu.h"

int main() {
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    init_i2c_imu();
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    int32_t tiempo_prev = 0;
    int32_t dt;
    long double ang_z = 0;
    long double ang_z_prev = 0;


    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        dt = (time_us_32()-tiempo_prev);
        tiempo_prev=time_us_32();

        //Calcular angulo de rotación con giroscopio  
        ang_z = gyro[2]*dt;
        ang_z /= 131072000.0;
        ang_z += ang_z_prev;
        ang_z_prev=ang_z;

        printf("Rotacion en Z:  ");
        printf("%d  ", dt);
        printf("%d  ", gyro[2]);
        printf("%f  \n", ang_z);
        sleep_ms(1);
    }
}