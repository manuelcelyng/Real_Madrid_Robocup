#include "imu.h"

#ifdef i2c0

void init_i2c_imu(){
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c0, addr, buf, 2, false);
    setOffset(XOffsetAccel, XAccel);
    setOffset(YOffsetAccel, YAccel);
    setOffset(ZOffsetAccel, ZAccel);
    setOffset(XOffsetGyro, XGyro);
    setOffset(YOffsetGyro, YGyro);
    setOffset(ZOffsetGyro, ZGyro);
}

void setOffset(int16_t valor, uint8_t dir){
    uint8_t buf[3];
    buf[0] = dir;
    buf[1] = (valor >> 8) & 0xFF;
    buf[2] = valor & 0xFF;
    i2c_write_blocking(i2c0, addr, buf, 3, false);
}

int16_t getOffset(uint8_t dir){
    int16_t valor;
    uint8_t buffer[2];
    uint8_t val = dir;
    i2c_write_blocking(i2c0, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c0, addr, buffer, 2, false);
    valor = buffer[0] << 8 | buffer[1];
    return valor;
}

void mpu6050_read_raw(int16_t *gyro) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    //uint8_t val = 0x3B;
    // i2c_write_blocking(i2c0, addr, &val, 1, true); // true to keep master control of bus
    // i2c_read_blocking(i2c0, addr, buffer, 6, false);

    // for (int i = 0; i < 3; i++) {
    //     accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    // }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    uint8_t val = 0x47;
    i2c_write_blocking(i2c0, addr, &val, 1, true);
    i2c_read_blocking(i2c0, addr, buffer, 2, false);  // False - finished with bus

    
    *gyro = (buffer[0] << 8 | buffer[1]);
    

    // // Now temperature from reg 0x41 for 2 bytes
    // // The register is auto incrementing on each read
    // val = 0x41;
    // i2c_write_blocking(i2c_default, addr, &val, 1, true);
    // i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    //*temp = buffer[0] << 8 | buffer[1];
}
#endif