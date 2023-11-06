#ifndef ENCODER_HW
#define ENCODER_HW

extern uint8_t previousQuadrantNumber;
extern uint64_t numberTurns;

// Constantes de pines GPIO
// #define _ENCODER_I2C_SDA_PINS {12, 14, 20, 18}  // Pines SDA de I2C
// #define _ENCODER_I2C_SCL_PINS {13, 15, 21, 19}  // Pines SCL de I2C

// CONSTANT PINS GPIO
/*
 // ORDEN RUEDAS
    1 |  2
    ---|---
    4  | 3
    */
// FOR I2C 0   - RUEDA 1
#define ENCODER_I2C_SDA_PIN_0 12  // PIN 16 RUEDA 1
#define ENCODER_I2C_SCL_PIN_0 13  // PIN 17 RUEDA 1
// FOR I2C 1  - RUEDA 3
#define ENCODER_I2C_SDA_PIN_1 14  // PIN 26 RUEDA 3
#define ENCODER_I2C_SCL_PIN_1 15  // PIN 27 RUEDA 3
// FOR I2C 0  - RUEDA 2
#define ENCODER_I2C_SDA_PIN_2 20  // PIN 19 RUEDA 2
#define ENCODER_I2C_SCL_PIN_2 21  // PIN 20 RUEDA 2
// FOR I2C 1  - RUEDA 4
#define ENCODER_I2C_SDA_PIN_3 18  // PIN 24 RUEDA 4
#define ENCODER_I2C_SCL_PIN_3 19  // PIN 25 RUEDA 4


// Constantes de dirección del esclavo en el encoder y registros para leer o escribir
#define ENCODER_ADDR 0X36        // Direcciones de esclavo I2C
#define ENCODER_STATUS 0x0B      // Dirección para leer el estado del campo magnético
#define ENCODER_RAWANGLE_H 0x0C  // Dirección para los bits 11:8 de información
#define ENCODER_RAWANGLE_L 0x0D  // Dirección para los bits 7:0 de información

// Límites del PID, velocidad angular máxima y algunos parámetros
#define MAX_ANGULAR_SPEED 400
#define TOTAL_TIME 10 // Para el PID 1/T  donde T es el tiempo total entre errores calculados-> T = TIME_WINDOW_US*4

// Conversión de grados a radianes y ventana de tiempo para calcular la velocidad angular
#define SAMPLING_TIME 5000  // Tiempo en microsegundos para muestrear el ángulo del encoder
#define TIME_WINDOW_US 25000  // Ventana de tiempo en microsegundos para calcular la velocidad angular de un solo encoder
#define INV_TIME_WINDOW_S 40  // [s^-1] Inverso de TIME_WINDOW_US, convertido a segundos y calculado como 1 / TIME_WINDOW_US
#define TO_RAD(angle, turns) ((turns * 2 + angle / 180) * 3.141592)

// Variables con tipos necesarios para el uso en la API de hardware I2C
extern const uint8_t STATUS;
extern const uint8_t RAWANGLE_H;
extern const uint8_t RAWANGLE_L;

// for calculate angle
 typedef union{
        uint16_t rawAngle;
        uint16_t quadrantNumber;
}_uint_16_t;

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

// Métodos
void initI2C();
void checkMagnetPresent();
void obtainAngle(i2c_inst_t *a, double startAngle);

// Constantes para control P, PI o PID
#define KP 3
#define KI 0.005
#define KD 2.05

#endif