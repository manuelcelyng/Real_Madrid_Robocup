#ifndef ENCONDER_HW
#define ENCONDER_HW

// CONSTANT PINS GPIO
#define ENCODER_I2C_SDA_PIN 4  // PIN 6 
#define ENCODER_I2C_SCL_PIN 5  // PIN 7
#define ENCODER_I2C_SDA_PIN_2 8 // PIN 11
#define ENCODER_I2C_SCL_PIN_2 9 // PIN 12
#define ENCODER_I2C_SDA_PIN_3 6 // PIN 9
#define ENCODER_I2C_SCL_PIN_3 7 // PIN 10

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
double obtainInfo(i2c_inst_t *a); // make a reading so the degAngle gets updated
void correctAngle(); 
void checkQuadrant();



//CONSTANTS FOR CONTROL P or PI or PID 
#define Kp  3     // usually has a number greater than ki and Kd
#define Ki  0.005 // usually has a much smaller number than Kp and Kd
#define Kd  2.05  // usually has a number greater than Ki and less than Kp

// def constants for initial w of the motor.
float pid_p;  // proportional
float pid_i;  // integral 
float pid_d;  // derivativo

// var for PID control of w
float error;      // for the current error
float integral;   // for representation of the sum of erros
float prev_error; // for the last error
float velo_deseada;
float velo_medida;


#endif 