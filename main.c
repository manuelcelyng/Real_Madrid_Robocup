#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>


#include "pico/sync.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

// .h propios.
#include "sharedfunctions.h"
#include "motor_control.h"
#include "control.h"
#include "wheel_control.h"
#include "bluetooth.h" // .h definicion de todo lo relacionado a la comunicación bluetooth
#include "imu.h"

// SECCIÓN DE DEFINICIÓN DE VARIABLES GLOBALES Y FUNCIONES DE ATENCIÓN A INTERRUPCIÓN COMO LOS TIMERS.
// Timers, los cuales son interrupciónes que dan paso a la ejecución de determinada tarea.
bool repeating_timer_callback_imu(struct repeating_timer *t);
bool repeating_timer_callback_encoder(struct repeating_timer *t);
// Definir semáforos, mutex y cola necesarios para el correcto funcionamiento de las tareas.
SemaphoreHandle_t xSemaphoreCore0; // Semaforo entre la IMU y el Main control
SemaphoreHandle_t xSemaphoreCore1; // Semafoto entre la tarea de la muestra del ENCODER y la Ventana
// SemaphoreHandle_t xSemaphoreBluetooth;
SemaphoreHandle_t xSemaphoreInitControl; // Semáforo para indicarle inicio a la tarea del control de la llante
SemaphoreHandle_t xMutexSharedResources; // Mutex para la variable compartida desiredSpeed 
SemaphoreHandle_t xMutexSharedI2C; // Mutex Para el uso del I2C 
QueueHandle_t xQueueBluetoothCommands; // Cola, el bluetooth comparte datos con las tareas a través de está para evitar problemas en la sincronización
QueueHandle_t xQueueAnguloImu;
QueueHandle_t xQueueAngleEncoder;
QueueHandle_t xQueueISRBluetooth;

/////////////////////////////////////////////// VARIABLES Y ESTRUCTURAS GLOBALES ////////////////////////////////////////////////////
// para habilitar el control del otro core
bool run_control_wheels = false;
// gyro de la IMU y bandera
int16_t gyro = 0;
int16_t acceleration[3] = {0,0,0};

// Estructura que contiene lo que el bluetooth va a mandar y compartir.
typedef struct {
    int value1;
    int value2;
    int movement;
} DataGroup;
// Estructura para comunicar entre las tareas de sample encoder y ventana de tiempo encoder.
typedef struct {
    double previousAngle;
    double correctedAngle;
    double corrected_encoder_error;
} DataAngle;
//Estructuras para la creación de los Timers

struct repeating_timer timer2;
alarm_pool_t *pool11; // Variable para poder definir el timer en el core 1.


// SECCIÓN IMPLEMENTACIÓN DE LAS FUNCIONES
bool repeating_timer_callback_imu(struct repeating_timer *t) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreCore0, &xHigherPriorityTaskWoken);
    // Suelta el semaforo, para ejecutar una muestra en la tarea da la IMU
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true;
}

bool repeating_timer_callback_encoder(struct repeating_timer *t){
    // Suelta el semaforo, para ejecutar una muestra en la tarea del encoder.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphoreCore1, &xHigherPriorityTaskWoken);
    // Suelta el semaforo, para ejecutar una muestra en la tarea da la IMU
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return true;
}




void TaskvBluetooth(void *pvParameters) {
    DataGroup dataCommands;
    while (1)
    {
        char datosRecibidos[20]; 
         if (xQueueReceive(xQueueISRBluetooth, &datosRecibidos, portMAX_DELAY)) {
            // Procesar los datos
            char *move = strtok(datosRecibidos, ";");

            // La función strtok guardainternamente donde terminó en la anterior busqueda por lo que se manda el parametro nulo y el delimitador
            int value_1 = atoi(strtok(NULL, ";"));       // Como se sabe que son números, se convierte a entero, si se quiere double >> double numero = strtod(strtok(buffer, ";"), NULL);
            int value_2 = atoi(strtok(NULL, ";"));

            // Hacer algo con los datos (por ejemplo, imprimirlos)
            ejecutarMovimiento(move, value_1, value_2);

            dataCommands.value1 = value1;
            dataCommands.value2 = value2;
            dataCommands.movement = select_movement;
        // printf("%d,%d,%d \n" , dataCommands.value1, dataCommands.value2, dataCommands.movement);
        xQueueSend(xQueueBluetoothCommands, &dataCommands, portMAX_DELAY);

        }
        
    }
    
}



// Funciones de tarea
void TaskvImuSample(void *pvParameters) {
    int16_t angulo_calculado = 0;
    while (1) {
        xSemaphoreTake(xSemaphoreCore0, portMAX_DELAY);
        
        xSemaphoreTake(xMutexSharedI2C, portMAX_DELAY);
        mpu6050_read_raw(&gyro, acceleration);
        xSemaphoreGive(xMutexSharedI2C);
        angulo_calculado =  gyro;

        xQueueSend(xQueueAnguloImu, &angulo_calculado, portMAX_DELAY); // queue que sincroniza con taskvMainControl
    }
}

void TaskvMainControl(void *pvParameters) {

    struct repeating_timer timer;
    // CONTROL PID - ROBOT 
    //control space
    float delta = 0.001;
    float q[3][1] = {{0}, {0}, {0}};
    float T = 30;
    float b = 2 * 3.141592653589793 / T;
    float ek[3][1] = {{0}, {0}, {0}};
    float ek2[3][1] = {{0}, {0}, {0}};
    float U[3][1] = {{0}, {0}, {0}};
    float uk[3][1] = {{0}, {0}, {0}};
    float e[3][1] = {{0}, {0}, {0}};

    uint64_t offset_time = time_us_64();

    //Imu variables
    int32_t tiempo_prev = 0;
    int32_t dt;
    long double ang_z_prev = 0;
    long double ang_z = 0;
    long double vel = 0;
    long double dist = 0;
    // Calculate qd
    float qd[3][1];
    qd[0][0] = 0;//q[0][0];//-4*sinf(b * t_i)*sinf(b * t_i);
    qd[1][0] = 0;//q[1][0]+0.03 ;//2*sinf(b * t_i);
    qd[2][0] = 0;

    
    bool taskState =  false;
    DataGroup bluetoothCommands;
    int angulo=0;
    run_command = true;
    while (1) {
        if(taskState){
            xQueueReceive(xQueueAnguloImu, &angulo, portMAX_DELAY);
            if(gyro > 60 || gyro < -60){ 
                    ang_z = gyro*IMU_INTERVAL_TIMER_US;
                    //ang_z /= 16400000.0;
                    ang_z /= 939650784.0;
                    ang_z += ang_z_prev;
                    ang_z_prev=ang_z;
                }
            float t_i = (time_us_64()-offset_time)*1e-6;

            if(bluetoothCommands.movement == 1){
                qd[2][0] += TO_RAD(bluetoothCommands.value1,0);
                bluetoothCommands.movement = 0;
            }else if(bluetoothCommands.movement == 2){
                qd[0][0] += value2*cosf(TO_RAD(bluetoothCommands.value1,0));
                qd[1][0] += value2*sinf(TO_RAD(bluetoothCommands.value1,0));
                bluetoothCommands.movement = 0;
            }

            // Update ek
            for (int j = 0; j < 3; j++) {
                ek2[j][0] = ek[j][0];
                ek[j][0] = e[j][0];
            }

            // ang_Z es la variable que calcula la IMU  TOMA MUTEX
            //printf("ang: %f\n", ang_z);
            q[2][0] = ang_z;
            

            // Calculate e
            for (int j = 0; j < 3; j++) {
                e[j][0] = qd[j][0] - q[j][0];
            } // for

            // CAMBIAMOS LAS CONSTANTES DEL PID
            // printf("%f \n" , q[2][0]);
            for(int i = 0 ; i<4 ; i++){
                constansP[i] = 10*exp(-1*e[2][0]*e[2][0]) + constansP_C[i];
                //constansI[i] = 0.05*exp(-1*e[2][0]*e[2][0]) + constansI_C[i];
                //constansD[i] = 0.5*exp(-1*e[2][0]*e[2][0]) + constansD_C[i];
            } // for

            // Call the control function to update U
            uk[0][0] = U[0][0];
            uk[1][0] = U[1][0];
            uk[2][0] = U[2][0];
            control(e, ek, ek2, q, uk, U);

            // Call the planta function to calculate dq and dteta
            float dq[3][1];
            float dteta[4][1];
            planta(U, q, dq, dteta);

            // Update q
            for (int j = 0; j < 2; j++) {
                q[j][0] += dq[j][0] * delta;
            } // for

            
            xSemaphoreTake(xMutexSharedResources, portMAX_DELAY);
            if(!run_command){
                cancel_repeating_timer(&timer);
                taskState = false;
                
            }else{
                //printf("%f, %f, %f, %f \n", desiredSpeed[0], desiredSpeed[1], desiredSpeed[2], desiredSpeed[3]);
                if(e[0][0] > -0.1 && e[0][0] < 0.1 && e[1][0] > -0.1 && e[1][0] < 0.1 && e[2][0] > -0.10726 && e[2][0] < 0.10726){
                    run_control_wheels = false;
                    desiredSpeed[0] = 0;  // RUEDA 1
                    desiredSpeed[1] = 0;  // RUEDA 2
                    desiredSpeed[2] = 0;  // RUEDA 3
                    desiredSpeed[3] = 0;  // RUEDA 4
                    
                    // printf("%f, %f, %f \n", e[0][0], e[1][0], e[2][0]);
                }else{
                    // printf("q0 : %f  - q1: %f \n", q[0][0], q[1][0]);

                    desiredSpeed[0] = dteta[0][0];  // RUEDA 1
                    desiredSpeed[1] = dteta[1][0];  // RUEDA 2
                    desiredSpeed[2] = dteta[2][0];  // RUEDA 3
                    desiredSpeed[3] = dteta[3][0];  // RUEDA 4
                }
                
            }
            xSemaphoreGive(xMutexSharedResources);
        }else{
            xQueueReceive(xQueueBluetoothCommands, &bluetoothCommands, portMAX_DELAY);
            add_repeating_timer_ms(5, repeating_timer_callback_imu, NULL, &timer); // activa el timer cada 5 segundos 
            xSemaphoreTake(xMutexSharedResources, portMAX_DELAY);
            run_command= true;
            taskState = true;
            run_control_wheels = true;
            xSemaphoreGive(xMutexSharedResources);
            xSemaphoreGive(xSemaphoreInitControl); // Inicializa la ejecución del control de las Ruedas
            
          
        }    
    }
}

// Esta tarea toma muestras del enconder en cada una de las ruedas.
void TaskvEncoderSample(void *pvParameters){
    // pool11 =  alarm_pool_create(1,2);
    DataAngle dataAngle;
    dataAngle.corrected_encoder_error = 0;
    dataAngle.correctedAngle = 0;
    dataAngle.previousAngle = 0;
    AngleData startAngle = {0.0, 0.0, 0.0, 0.0}; // struct with all start angles
    bool flag_sample = false;
    bool startAngle_bool =  true;
    int cantidad_muestras = 0;
    correctedAngle = 0;
    int i = 0;
    // add_repeating_timer_ms(7000, repeating_timer_callback_encoder, NULL, &timer2);
    while (1) {
        //Tómo el semaforo que se libera desde la interrupción Timer.
        xSemaphoreTake(xSemaphoreCore1, portMAX_DELAY);
        dataAngle.previousAngle = correctedAngle;
        // MUTEX FOR I2C
        // xSemaphoreTake(xMutexSharedI2C, portMAX_DELAY);
        // obtainAngle(startAngle[i],startAngle_bool);
        // xSemaphoreGive(xMutexSharedI2C);  

        if(startAngle_bool){
            startAngle[i] = correctedAngle;
            startAngle_bool = false;
        }else{
            if(!flag_sample){
                flag_sample = true;
            }else{
                if((correctedAngle - dataAngle.previousAngle)<0){
                    dataAngle.corrected_encoder_error-=1;  // disminuye el angulo - pwm >750 , antihorario
                }else{ 
                    dataAngle.corrected_encoder_error+=1; //  aumenta el angulo - pwm <750 , horario
                }
            }
        }

        cantidad_muestras+=1;

        // Tomo 5 Muestras
        if(cantidad_muestras == 5){
            cancel_repeating_timer(&timer2); //Cancela la alarma para que no muestree mientras se calcula el PID 
            dataAngle.correctedAngle =  correctedAngle;
            xQueueSend(xQueueAngleEncoder, &dataAngle, portMAX_DELAY);
            xSemaphoreTake(xSemaphoreCore1, portMAX_DELAY); // Tómo de vuelta el semaforo para proseguir con las nuevas muestras.
            //Reinicio las variables
            i+=1;
            numberTurns = 0;
            cantidad_muestras = 0;
            previousQuadrantNumber = 0;
            flagHelp  = 0;
            startAngle_bool =  true;
            flag_sample = false;
            correctedAngle = 0;
            if(i == 4){
                i=0;
            }

        }
    }
}

void TaskvPIDWheels(void *pvParameters) {
    pool11 =  alarm_pool_create(1,2);
    DataAngle dataAngle;
    int valid_quadrant = 0 ;
    bool taskState = false;
    while (1) {
        if(taskState){
            for(int i = 0 ; i<4; i++){

                switch(i)
                {
                case 0:
                    switchI2c(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3);
                    break;
                case 1:
                    switchI2c(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0);
                    break;
                case 2:
                    switchI2c(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1);
                    break;
                case 3: 
                    switchI2c(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2);
                    break;
                }  

                alarm_pool_add_repeating_timer_us(pool11,SAMPLING_TIME, repeating_timer_callback_encoder, NULL, &timer2); 

                 // cuando se terminó un muestreo Entra a le ejecución de esta cola.
                xQueueReceive(xQueueAngleEncoder, &dataAngle, portMAX_DELAY);
                // Validamos el cuadrante, esto con el fin de una correcta lectura de la velocidad.
                valid_quadrant = ((int)(dataAngle.correctedAngle/90) + 1) -  ((int)(dataAngle.previousAngle/90) + 1);
                // Calculamos esto para saber si la velocidad de la llanta es 0.
                dataAngle.previousAngle =  dataAngle.correctedAngle-dataAngle.previousAngle;

                if(dataAngle.previousAngle<0){
                    dataAngle.previousAngle= -1*dataAngle.previousAngle;
                }
                if( dataAngle.previousAngle<=0.2   ||  (dataAngle.previousAngle)>359.8 ){
                        speedData[i] = 0;
                }else{
                    // ELIGE DEPENDIENDO LA DIRECCION DE LA VELOCIDAD
                    //if( valid_quadrant==3 || corrected_encoder_error<0)
                    if( valid_quadrant==3 || dataAngle.corrected_encoder_error<0){
                        dataAngle.correctedAngle =  360-dataAngle.correctedAngle;
                        speedData[i] = -1*((double)((TO_RAD(dataAngle.correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                    }else if(correctedAngle<340){
                        speedData[i] = (double)((TO_RAD(dataAngle.correctedAngle, numberTurns))*INV_TIME_WINDOW_S);
                    }else{
                        dataAngle.correctedAngle =  360-dataAngle.correctedAngle;
                        speedData[i] = -1*((double)((TO_RAD(correctedAngle, numberTurns))*INV_TIME_WINDOW_S));
                    }
                }


                // Ajusto la velocidad de una vez;
                xSemaphoreTake(xMutexSharedResources, portMAX_DELAY);
                calcularControlPID(i);  
                xSemaphoreGive(xMutexSharedResources);
                adjustPWM(i);
                // out of while and continue with for with another encoder
               
            }

            xSemaphoreTake(xMutexSharedResources, portMAX_DELAY);
            taskState =  run_control_wheels;
            xSemaphoreGive(xMutexSharedResources);
            if(!taskState){
                for(int i = 0 ; i<4; i++){
                    // printf("error_previo R %d : %f\n",i+1,pidPreviousError[i]);
                    if(desiredSpeed[i] < 1 && desiredSpeed[i] > -1){
                        if(i==3){
                            taskState = false;
                            xSemaphoreTake(xMutexSharedResources, portMAX_DELAY);
                            run_command =  false;
                            xSemaphoreGive(xMutexSharedResources);
                        }
                    }
                    else{
                        taskState = true;
                        break;
                    }
                }
                if(!taskState){
                    for(int i = 0; i<4 ; i++){
                        pid[i] = 0;
                        duty[i] = 750;
                        adjustPWM(i);
                    }
                }
            }  
               


        }else{
            xSemaphoreTake(xSemaphoreInitControl, portMAX_DELAY);
            // printf("InitControl\n");
            // Inicializa el Timer del muestreo del encoder.
            taskState =  true;

        }
       
    }
}

void SIMULAR_ISR_BLUETOOTH(){
    char datosRecibidos[] = "D;20;14";

    // Enviar los datos a la cola
    xQueueSendFromISR(xQueueISRBluetooth, &datosRecibidos, NULL);
}



int main() {
    stdio_init_all();

    // Initialize motor control _ init motors with pwm for stop and init bluetooth
    initMotorControl();
    sleep_ms(3000);

    // Initialize I2C and check magnetPresent in encoders
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state for all encoders, afte it , i2c encoders 2 and 4 are active
    // reset IMU  - OFFSET ANGULO 0
    mpu6050_reset();  // IMU IN I2C0
    // // For necesario para las primeras lecturas erroneas de la IMU.
    for (size_t i = 0; i < 40; i++)
    {
        mpu6050_read_raw(&gyro, acceleration);
        sleep_ms(5);
    }

     // Inicializa el Bluetooth
    initUart(GPIO_UART_TX, GPIO_UART_RX);

    // Define the task handles
    
    TaskHandle_t handle1;
    TaskHandle_t handle2;
    TaskHandle_t handle3;
    TaskHandle_t handle4;
    TaskHandle_t handle5;

    // Crear semáforos, mutex y cola
    // Semaforo para sincronizar la tarea de la IMU con el Main_CONTROL
    xSemaphoreCore0 = xSemaphoreCreateBinary();
    xSemaphoreCore1 = xSemaphoreCreateBinary();
    //xSemaphoreBluetooth = xSemaphoreCreateBinary();
    xSemaphoreInitControl = xSemaphoreCreateBinary();
    xMutexSharedResources = xSemaphoreCreateMutex(); // Entre ambos core
    xMutexSharedI2C = xSemaphoreCreateMutex(); // Entre ambos core

    // Creo una queue , el cual será la data que se manda por bluetooth
    xQueueBluetoothCommands = xQueueCreate(1, sizeof(DataGroup) );
    xQueueAnguloImu =  xQueueCreate(1, sizeof(int) );
    xQueueAngleEncoder =  xQueueCreate(1, sizeof(DataAngle));
    xQueueISRBluetooth = xQueueCreate(1, sizeof(char[20]));

    //TIMER EJECUTANDOSE EN EL CORE 1



    // Crear y lanzar tareas
    xTaskCreate(TaskvImuSample, "TaskImu", configMINIMAL_STACK_SIZE, NULL, 1, &handle1);
    xTaskCreate(TaskvImuSample, "TaskImu", configMINIMAL_STACK_SIZE, NULL, 1, &handle1);
    xTaskCreate(TaskvMainControl, "TaskMainControl", configMINIMAL_STACK_SIZE, NULL, 1, &handle2);
    xTaskCreate(TaskvEncoderSample, "TaskEncoder", configMINIMAL_STACK_SIZE, NULL, 1, &handle3);
    xTaskCreate(TaskvPIDWheels, "TaskWheelsControl", configMINIMAL_STACK_SIZE, NULL, 1 , &handle4);
    xTaskCreate(TaskvBluetooth, "TaskBluetooth", configMINIMAL_STACK_SIZE, NULL, 1 , &handle5);

    // Pin Tasks
    vTaskCoreAffinitySet(handle1, (1 << 0)); // Core 0
    vTaskCoreAffinitySet(handle2, (1 << 0)); // Core 0
    vTaskCoreAffinitySet(handle3, (1 << 1)); // Core 1
    vTaskCoreAffinitySet(handle4, (1 << 1)); // Core 1
    vTaskCoreAffinitySet(handle5, (1 << 0)); // Core 0

    // Configurar e inicializar UART e interrupción UART
    // ...

    // Iniciar el planificador de tareas
    SIMULAR_ISR_BLUETOOTH();
    vTaskStartScheduler();
    


    while (1) {
        // Bucle principal
    }

    return 0;
}
