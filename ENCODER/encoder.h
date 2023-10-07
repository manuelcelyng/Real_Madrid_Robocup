#ifndef ENCONDER_HW
#define ENCONDER_HW

// CONSTANT PINS GPIO
#define ENCODER_I2C_SDA_PIN 4  // PIN 6 
#define ENCODER_I2C_SCL_PIN 5  // PIN 7

// CONSTATS DIRECTION OF SLAVE IN ENCODER AND REGISTERS FOR READ OR WRITE
#define ENCODER_ADDR 0X36
#define ENCODER_STATUS 0x0B
#define ENCODER_RAWANGLE_H 0X0C // define constans for 11:8 bits information
#define ENCODER_RAWANGLE_L 0x0D  // define constants for 7:0 bits information

// VARIABLES WITH TYPE NECCESARY FOR USE IN I2C HARDWARE API
const uint8_t STATUS;
const uint8_t RAWANGLE_H;
const uint8_t RAWANGLE_L;



// methods
void initI2C(); // The I2C is initialized for the GPIOS and TESTS are run to find out which SLAVES are enabled for communication.
void checkMagnetPresent(); //checks register 0x0b in the encoder, to know the state of the magnet and if it is correct, enable it.
void obtainInfo(); // make a reading so the degAngle gets updated
void correctAngle(); 



#endif