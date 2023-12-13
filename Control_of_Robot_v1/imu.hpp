
#ifndef IMU_HW
#define IMU_HW

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"

static int addr = 0x68;
#define XAccel 0x06
#define YAccel 0x08
#define ZAccel 0x0A
#define XGyro 0x13
#define YGyro 0x15
#define ZGyro 0x17
#define XOffsetAccel -1252
#define YOffsetAccel -615
#define ZOffsetAccel 1800
#define XOffsetGyro 19
#define YOffsetGyro -47
#define ZOffsetGyro 5
// #define GPIO_FUNC_I2C 3
#define IMU_I2C_SDA_PIN 20
#define IMU_I2C_SCL_PIN 21

#define IMU_INTERVAL_TIMER_MS 25
#define IMU_INTERVAL_TIMER_US 25000

/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope
   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

void mpu6050_reset();
void init_i2c_imu();
void setOffset(int16_t valor, uint8_t dir);
int16_t getOffset(uint8_t dir);
void mpu6050_read_raw(int16_t *gyro, int16_t accel[3]);
#endif