#ifndef ENCODER_HW
#define ENCODER_HW

// Constantes de pines GPIO
#define ENCODER_I2C_SDA_PINS {4, 8, 6, 26}  // Pines SDA de I2C
#define ENCODER_I2C_SCL_PINS {5, 9, 7, 27}  // Pines SCL de I2C

// Constantes de dirección del esclavo en el encoder y registros para leer o escribir
#define ENCODER_ADDR 0X36        // Direcciones de esclavo I2C
#define ENCODER_STATUS 0x0B      // Dirección para leer el estado del campo magnético
#define ENCODER_RAWANGLE_H 0x0C  // Dirección para los bits 11:8 de información
#define ENCODER_RAWANGLE_L 0x0D  // Dirección para los bits 7:0 de información

// Límites del PID, velocidad angular máxima y algunos parámetros
#define MAX_ANGULAR_SPEED 400
#define TOTAL_TIME 10 // Para el PID 1/T  donde T es el tiempo total entre errores calculados

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

// Estructura para ángulos iniciales
typedef struct {
    double startAngle[4];
} AngleData;

// Estructura para velocidades angulares
typedef struct {
    double angular[4];
} SpeedData;

// Estructura para velocidades angulares deseadas
typedef struct {
    double desired[4];
} DesiredSpeedData;

// Estructura para control PID
typedef struct {
    double PID[4];
} PIDData;

// Estructura para términos integrales de PID
typedef struct {
    float i[4];
} PIDIntegralData;

// Estructura para errores previos de PID
typedef struct {
    double previous[4];
} PIDErrorData;

// Métodos
void initI2C();
void checkMagnetPresent();
void obtainAngle(i2c_inst_t *a, double startAngle);

// Constantes para control P, PI o PID
#define KP 3
#define KI 0.005
#define KD 2.05

#endif