#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

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

// By default these devices  are on bus address 0x68
static int addr = 0x68;
#define XAccel 0x06
#define YAccel 0x08
#define ZAccel 0x0A
#define XGyro 0x13
#define YGyro 0x15
#define ZGyro 0x17

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

void setOffset(int16_t valor, uint8_t dir){
    uint8_t buf[3];
    buf[0] = dir;
    buf[1] = (valor >> 8) & 0xFF;
    buf[2] = valor & 0xFF;
    i2c_write_blocking(i2c_default, addr, buf, 3, false);
}

int16_t getOffset(uint8_t dir){
    int16_t valor;
    uint8_t buffer[2];
    uint8_t val = dir;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);
    valor = buffer[0] << 8 | buffer[1];
    return valor;
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
#endif

int main() {
    stdio_init_all();
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    int16_t acceleration0[3], gyro0[3];
    long f_ax,f_ay, f_az;
    int p_ax, p_ay, p_az;
    long f_gx,f_gy, f_gz;
    int p_gx, p_gy, p_gz;
    int counter=0;
    acceleration0[0] = getOffset(XAccel);
    acceleration0[1] = getOffset(YAccel);
    acceleration0[2] = getOffset(ZAccel);
    gyro0[0] = getOffset(XGyro);
    gyro0[1] = getOffset(YGyro);
    gyro0[2] = getOffset(ZGyro);

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Filtrar las lecturas
        f_ax = f_ax-(f_ax>>5)+acceleration[0];
        p_ax = f_ax>>5;

        f_ay = f_ay-(f_ay>>5)+acceleration[1];
        p_ay = f_ay>>5;

        f_az = f_az-(f_az>>5)+acceleration[2];
        p_az = f_az>>5;

        f_gx = f_gx-(f_gx>>3)+gyro[0];
        p_gx = f_gx>>3;

        f_gy = f_gy-(f_gy>>3)+gyro[1];
        p_gy = f_gy>>3;

        f_gz = f_gz-(f_gz>>3)+gyro[2];
        p_gz = f_gz>>3;

        if (counter==100){
            //Mostrar las lecturas separadas por un [tab]
            printf("promedio:");
            printf("%d  ", p_ax); 
            printf("%d  ", p_ay); 
            printf("%d  ", p_az); 
            printf("%d  ", p_gx); 
            printf("%d  ", p_gy); 
            printf("%d  \n", p_gz);

            //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
            if (p_ax>0){acceleration0[0]--;}
            else {acceleration0[0]++;}
            if (p_ay>0){acceleration0[1]--;}
            else {acceleration0[1]++;}
            if (p_az-16384>0){acceleration0[2]--;}
            else {acceleration0[2]++;}

            setOffset(acceleration0[0], XAccel);
            setOffset(acceleration0[1], YAccel);
            setOffset(acceleration0[2], ZAccel);

            //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
            if (p_gx>0) {gyro0[0]--;}
            else {gyro0[0]++;}
            if (p_gy>0) gyro0[1]--;
            else {gyro0[1]++;}
            if (p_gz>0) gyro0[2]--;
            else {gyro0[2]++;}

            setOffset(gyro0[0], XGyro);
            setOffset(gyro0[1], YGyro);
            setOffset(gyro0[2], ZGyro);

            counter = 0;
        }
        counter++;
    }
}