#ifndef ENCONDER_HW
#define ENCONDER_HW

// CONSTANT PINS GPIO
// FOR I2C 0
#define ENCODER_I2C_SDA_PIN_1 12  // PIN 16
#define ENCODER_I2C_SCL_PIN_1 13  // PIN 17
#define ENCODER_I2C_SDA_PIN_2 20  // PIN 26
#define ENCODER_I2C_SCL_PIN_2 21  // PIN 27
// FOR I2C 1
#define ENCODER_I2C_SDA_PIN_3 14  // PIN 19
#define ENCODER_I2C_SCL_PIN_3 15  // PIN 20
#define ENCODER_I2C_SDA_PIN_4 18  // PIN 24 - pin 10 counting from USB
#define ENCODER_I2C_SCL_PIN_4 19  // PIN 25 - pin 9  counting from USB

// CONSTATS DIRECTION OF SLAVE IN ENCODER AND REGISTERS FOR READ OR WRITE
#define ENCODER_ADDR 0X36       // address of i2c slave in the encoder.
#define ENCODER_STATUS 0x0B     // address to read the state of the magnetic field and see if it is correct
#define ENCODER_RAWANGLE_H 0X0C // address. define constans for 11:8 bits information
#define ENCODER_RAWANGLE_L 0x0D  //address.  define constants for 7:0 bits information

//Limites of PID -  Angular speed and some params
#define maxAngularSpeed 400


// TO CALCULATE FROM DEGREES TO RADIANS AND TIME WINDOWS TO CALCULATE ANGULAR VELOCITY
#define total_time 10
#define sampling_time 5000  // Time in microseconds. to sample the encoder angle.
#define timeWindow_u 25000 // Time in microseconds. Time window to calculate the angular velocity of an encoder.
#define inv_timeWindow_s 40  // [s^-1]The inverse of timeWindow_u. Pass timeWindow_u to seconds and do 1/timeWindow_u.
#define toRad(angle,turns) ((turns*2  + angle/180)*3.141592) // Calculate the radians taking into account the turns and the last angle given by the encoder.

// VARIABLES WITH TYPE NECCESARY FOR USE IN I2C HARDWARE API
const uint8_t STATUS;
const uint8_t RAWANGLE_H;
const uint8_t RAWANGLE_L;
// for calculate angle
 typedef union{
        uint16_t rawAngle;
        uint16_t quadrantNumber;
}_uint_16_t;

// for angles
typedef struct 
{
    double startAngle_1;
    double startAngle_2;
    double startAngle_3;
    double startAngle_4;
}__angle;

// for angular speed
typedef struct 
{
    double angular_1;
    double angular_2;
    double angular_3;
    double angular_4;
}__speed;
// for  desired angular speed
typedef struct 
{
    double desired_1;
    double desired_2;
    double desired_3;
    double desired_4;

}__speed_desir;

// for PID
typedef struct
{
    double PID_1;
    double PID_2;
    double PID_3;
    double PID_4;
}__pid;
// for integral 
typedef struct 
{
    float i_1;
    float i_2;
    float i_3;
    float i_4;
}__pid_i;

// previous error for PID
typedef struct 
{
    double error_1;
    double error_2;
    double error_3;
    double error_4;
}__error_p;




// methods
void initI2C(); // The I2C is initialized for the GPIOS and TESTS are run to find out which SLAVES are enabled for communication.
void checkMagnetPresent(); //checks register 0x0b in the encoder, to know the state of the magnet and if it is correct, enable it.
void obtainAngle(i2c_inst_t *a, double startAngle); // make a reading so the degAngle gets updated



//CONSTANTS FOR CONTROL P or PI or PID 
#define Kp  3     // usually has a number greater than ki and Kd
#define Ki  0.005 // usually has a much smaller number than Kp and Kd
#define Kd  2.05  // usually has a number greater than Ki and less than Kp




#endif 