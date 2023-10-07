#include "pico/stdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// include my own .h
#include "encoder.h"

float startAngle;
float degAngle;
float correctedAngle;
const uint8_t STATUS = ENCODER_STATUS;
const uint8_t RAWANGLE_H = ENCODER_RAWANGLE_H;
const uint8_t RAWANGLE_L = ENCODER_RAWANGLE_L;


bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void initI2C(){
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
   
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(ENCODER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
 
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
 
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
 
        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.
 
        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
 
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}



void obtainInfo(){

    // buffer[0] 0000000000001111  -> encoder send us bits 8:11
    // 0000000011111111[1]  -> encoder send us bits 0:7   // TOTAL 12 BITS INFORMATION
    uint8_t buffer[2];  // buffer[0] guarda los más significativos, y buffer[1] guarda los menos significativos.
    uint16_t rawAngle;

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(i2c_default, ENCODER_ADDR,&RAWANGLE_L,1, true);
    i2c_read_blocking(i2c_default, ENCODER_ADDR, &buffer[1],1,false);

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(i2c_default, ENCODER_ADDR,&RAWANGLE_H,1, true);
    i2c_read_blocking(i2c_default, ENCODER_ADDR, &buffer[0],1,false);

    // obtain the angle and print
   
    rawAngle = (buffer[0]<<8) | buffer[1];   // example, higbye = 00000000|00001111, le plicamos <<8  resulta -> 00001111|00000000
    // finalmente : 00000111100000000 or 0000000011111111 =  00001111|11111111 
    degAngle = rawAngle * 0.087890625;

    printf("Valor del angulo es:  %f\n" , degAngle);

}

void checkMagnetPresent(){
    uint8_t magnedStatus =0;
    const uint8_t statuscoso  = 0x0b;
    while ((magnedStatus&32)!= 32) // while the magnet is not adjusted to the proper distante - 32 : MD=1
    {
        i2c_write_blocking(i2c_default, ENCODER_ADDR,&STATUS,1, true); // send for read data in Register Status
        i2c_read_blocking(i2c_default, ENCODER_ADDR, &magnedStatus,1,false); // Read data sended from encoder status register

        printf("value is : %d", magnedStatus);
        sleep_ms(1000);
    }

    printf("Magnet Found!");
    sleep_ms(3000);
    

}

void correctAngle(){
    correctedAngle = degAngle - startAngle;

    if(correctAngle<0){
        correctedAngle = correctedAngle +360;
    }

    printf("Corrected angle: %f",  correctedAngle);
}

int main() {
    stdio_init_all();
    
    

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    sleep_ms(7000);
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state
    obtainInfo();
    startAngle = degAngle; // angle for starting

    while(true){

        tight_loop_contents();
        sleep_ms(1000);
        obtainInfo();
        correctAngle();
    }

     return 0;  
#endif


}