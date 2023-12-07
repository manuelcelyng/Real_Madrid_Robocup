#ifndef ENCODER_HW
#define ENCODER_HW
#include "hardware/i2c.h"
#include <inttypes.h> // for pring uint64_t or another type

#define PI 3.141592
// Constantes para control P, PI o PID
//RUEDA 1
#define KP_0 0.8
#define KI_0 0.1
#define KD_0 0.02
//RUEDA 2
#define KP_1 0.8
#define KI_1 0.1
#define KD_1 0.02
//RUEDA 3
#define KP_2 0.8
#define KI_2 0.1
#define KD_2 0.02
// RUEDA 4
#define KP_3 0.8
#define KI_3 0.1
#define KD_3 0.02
// defines 3 vectors that contain all constants of pid for each wheel
typedef double ConstantsP[4];
typedef double ConstantsI[4];
typedef double ConstantsD[4];

extern ConstantsP constansP_C;
extern ConstantsI constansI_C;
extern ConstantsD constansD_C;
extern ConstantsP constansP;
extern ConstantsI constansI;
extern ConstantsD constansD;


// CONSTANT PINS GPIO FOR I2C
/*
 // ORDEN RUEDAS
     1 | 2     <---- I2C_1
    ---|---
     4 | 3     <---- I2C_1
*/
// FOR I2C 1   -  RUEDA 1
#define ENCODER_I2C_SDA_PIN_0 10  // PIN 16 RUEDA 1 I2C1  CAMBIO CAMBIO IMPORTANTE
#define ENCODER_I2C_SCL_PIN_0 11  // PIN 17 RUEDA 1 I2C1
// FOR I2C 1  -  RUEDA 2 
#define ENCODER_I2C_SDA_PIN_1 26  // PIN 26 RUEDA 2 I2C1
#define ENCODER_I2C_SCL_PIN_1 27  // PIN 27 RUEDA 2 I2C1
// FOR I2C 1  - RUEDA 3
#define ENCODER_I2C_SDA_PIN_2 14  // PIN 19 RUEDA 3 I2C1  CAMBIO CAMBIO IMPORTANTE
#define ENCODER_I2C_SCL_PIN_2 15  // PIN 20 RUEDA 3 I2C1
// FOR I2C 1  - RUEDA 4
#define ENCODER_I2C_SDA_PIN_3 18  // PIN 24 RUEDA 4 I2C1
#define ENCODER_I2C_SCL_PIN_3 19  // PIN 25 RUEDA 4 I2C1


// GPIO LED CONDICION DE FINALIZACIÓN
#define GPIO_LED 5


// Constantes de dirección del esclavo en el encoder y registros para leer o escribir
#define ENCODER_ADDR 0X36        // Direcciones de esclavo I2C
#define ENCODER_STATUS 0x0B      // Dirección para leer el estado del campo magnético
#define ENCODER_RAWANGLE_H 0x0C  // Dirección para los bits 11:8 de información
#define ENCODER_RAWANGLE_L 0x0D  // Dirección para los bits 7:0 de información
// Variables con tipos necesarios para el uso en la API de hardware I2C
extern const uint8_t STATUS;
extern const uint8_t RAWANGLE_H;
extern const uint8_t RAWANGLE_L;

// Límites del PID, velocidad angular máxima y algunos parámetros
#define MAX_ANGULAR_SPEED 200
#define TOTAL_TIME 25 // Para el PID 1/T  donde T es el tiempo total entre errores calculados-> T = TIME_WINDOW_US*4
// Conversión de grados a radianes y ventana de tiempo para calcular la velocidad angular
#define SAMPLING_TIME 1200 // Time in microseconds to sample encoder angle
#define TIME_WINDOW_US 6250//12500  // Time window in microseconds for calculating the angular velocity of a single encoder
#define INV_TIME_WINDOW_S 160  // [s^-1] Inverso de TIME_WINDOW_US, convertido a segundos y calculado como 1 / TIME_WINDOW_US
#define TO_RAD(angle, turns) (((turns * 2.0) + (angle / 180.0)) * 3.141592) // convert degrees to radians

// for calculate angle
 typedef union{
        uint16_t rawAngle;
        uint16_t quadrantNumber;
}_uint_16_t;

extern uint8_t previousQuadrantNumber;
extern int numberTurns;
extern double correctedAngle;
extern int flagHelp;

/* INFORMACION IMPORTANTE
  // En todos los arreglos a continuación hay un orden por llanta .
  // posicion [0] -> RUEDA 1
  // posicion [1] -> RUEDA 3
  // posicion [2] -> RUEDA 2  
  // posicion [3] -> RUEDA 4 
*/

// Para ángulos iniciales
typedef double AngleData[4];
// Para velocidades angulares
typedef double SpeedData[4];
// Para velocidades angulares deseadas
typedef double DesiredSpeedData[4];
// Para control PID - Se comparte con el core 0 para actualizar el pwm en el.
typedef double PIDData[4];
// Para términos integrales de PID;
typedef double PIDIntegralData[4];
// Para errores previos de PID
typedef double PIDErrorData[4];

// Declaro las variables como externas para verlas desde el main
extern SpeedData speedData;
extern DesiredSpeedData desiredSpeed;
extern PIDData pid;
extern PIDIntegralData pidIntegral;
extern PIDErrorData pidPreviousError;

// Métodos
void initI2C();
void checkMagnetPresent();
void switchI2c(uint sda_enable, uint scl_enable, uint sda_disable , uint scl_disable);
void obtainAngle(double startAngle, bool start);
void calcularControlPID(int i);



#endif