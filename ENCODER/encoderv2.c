#include "pico/stdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <inttypes.h>
// include my own .h
#include "encoderv2.h"


// All struct for PID control
SpeedData speedData = {0.0, 0.0, 0.0, 0.0};
DesiredSpeedData desiredSpeed = {0.0, 0.0, 0.0, 0.0};
PIDData pid = {0.0, 0.0, 0.0, 0.0};
PIDIntegralData pidIntegral = {0.0, 0.0, 0.0, 0.0};
PIDErrorData pidPreviousError = {0.0, 0.0, 0.0, 0.0};



// for quadrant
uint8_t previousQuadrantNumber = 0;
uint64_t numberTurns = 0;
// for total angle calculated
//double totalAngle = 0; // for obtain total angle
double correctedAngle= 0;

const uint8_t STATUS = ENCODER_STATUS;
const uint8_t RAWANGLE_H = ENCODER_RAWANGLE_H;
const uint8_t RAWANGLE_L = ENCODER_RAWANGLE_L;


bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


void initI2C(){
    // configure with speed mode plus -> 1000kbps for i2c0 and i2c1
    i2c_init(i2c_default, 1000 * 1000);
    i2c_init(i2c1, 1000 * 1000);
    // configura 1 encoder y lo deja listo para el inicio de lectura.
    gpio_set_function(ENCODER_I2C_SDA_PIN_0, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN_0, GPIO_FUNC_I2C);
     // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_0);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_0);

    // configura 2 encoder 
    gpio_set_function(ENCODER_I2C_SDA_PIN_1, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN_1, GPIO_FUNC_I2C);
    // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_1);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_1);

     // configura 3 encoder
    gpio_set_function(ENCODER_I2C_SDA_PIN_2,  GPIO_FUNC_NULL);
    gpio_set_function(ENCODER_I2C_SCL_PIN_2,  GPIO_FUNC_NULL);
     // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_2);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_2);

    // configura 4 encoder
    gpio_set_function(ENCODER_I2C_SDA_PIN_3, GPIO_FUNC_NULL);
    gpio_set_function(ENCODER_I2C_SCL_PIN_3, GPIO_FUNC_NULL);
     // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_3);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_3);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, GPIO_FUNC_I2C));
    

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

// this function only is a verification of state of magnet in the all Encoders
void checkMagnetPresent(){
    uint8_t magnedStatus = 0;
    // const uint8_t statuscoso  = 0x0b;

    for(int equisde = 0 ;  equisde<4 ;  equisde++){
        magnedStatus = 0;
        switch (equisde)
        {
        case 1:
            switchI2c(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0);
            break;
        case 2:
             switchI2c(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1);
            break;
        }

        while ((magnedStatus&32)!= 32) // while the magnet is not adjusted to the proper distante - 32 : MD=1
        {
            if(equisde == 0 || equisde ==1){
                i2c_write_blocking(i2c0, ENCODER_ADDR,&STATUS,1, true); // send for read data in Register Status
                i2c_read_blocking(i2c0, ENCODER_ADDR, &magnedStatus,1,false); // Read data sended from encoder status register

            }else{
                i2c_write_blocking(i2c1, ENCODER_ADDR,&STATUS,1, true); // send for read data in Register Status
                i2c_read_blocking(i2c1, ENCODER_ADDR, &magnedStatus,1,false); // Read data sended from encoder status register
            }
           
            printf("value is for encoder %d : %d\n", equisde+1 ,magnedStatus);
            sleep_ms(1000);
        }

        printf("Magnet Found! Encoder: %d", equisde+1);
        
        sleep_ms(3000);

        
    }
    
    

}

void obtainAngle(i2c_inst_t * a, double startAngle){

    // buffer[0] 0000000000001111  -> encoder send us bits 8:11
    // 0000000011111111[1]  -> encoder send us bits 0:7   // TOTAL 12 BITS INFORMATION
    uint8_t buffer[2];  // buffer[0] guarda los más significativos, y buffer[1] guarda los menos significativos.
    _uint_16_t aux;

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(a, ENCODER_ADDR,&RAWANGLE_L,1, true);
    i2c_read_blocking(a, ENCODER_ADDR, &buffer[1],1,false);

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(a, ENCODER_ADDR,&RAWANGLE_H,1, true);
    i2c_read_blocking(a, ENCODER_ADDR, &buffer[0],1,false);

    // obtain the angle and print
   
    aux.rawAngle = (buffer[0]<<8) | buffer[1];   // example, higbye = 00000000|00001111, le plicamos <<8  resulta -> 00001111|00000000
    // finalmente : 00000111100000000 or 0000000011111111 =  00001111|11111111 
    correctedAngle = aux.rawAngle * 0.087890625;
   

    // CORRECTED ANGLE 
    
    correctedAngle = correctedAngle - startAngle;
    // printf("corrected angle %f\n" , correctedAngle);
    
    if(correctedAngle<0){
        correctedAngle = correctedAngle + 360;
    }  


    // quadrant for turns 
    /*
    //quadrants
    4  |  1
    ---|---
    3  |  2
    */

   //Quadrant 1
   if(correctedAngle>=0 && correctedAngle <= 90){
        aux.quadrantNumber = 1;
   }
   if(correctedAngle>90 && correctedAngle <= 180){
        aux.quadrantNumber = 2;
   }
   if(correctedAngle>180 && correctedAngle <= 270){
        aux.quadrantNumber = 3;
   }
   if(correctedAngle>270 && correctedAngle < 360){
        aux.quadrantNumber = 4;
   }

   if(aux.quadrantNumber != previousQuadrantNumber){
        if(aux.quadrantNumber == 1 &&  previousQuadrantNumber == 4){
            numberTurns++;
        }
        if(aux.quadrantNumber == 4 &&  previousQuadrantNumber == 1){
            numberTurns--;
        }

        previousQuadrantNumber = aux.quadrantNumber;

   }

    // NO USO TOTAL ANGLE, DADO QUE PUEDO CALCULARLO DE UNA MANERA MAS EFECTIVA
   //totalAngle = ((double)(numberTurns)*360) + correctedAngle;

   //totalAngle =  (totalAngle*3.141592)/180; // radianes

    // printf("total angle %f\n", totalAngle);

}

void switchI2c(uint sda_enable, uint scl_enable, uint sda_disable , uint scl_disable){
    // enable 2 pin gpio for use I2C
    gpio_set_function(sda_enable, GPIO_FUNC_I2C);
    gpio_set_function(scl_enable, GPIO_FUNC_I2C);
    // disable 2 pin gpio for dont use I2C
    gpio_set_function(sda_disable, GPIO_FUNC_NULL);
    gpio_set_function(scl_disable, GPIO_FUNC_NULL);
}


void calcularControlPID(){
    double error = 0; 
    // for all motors
    for(int i=0 ; i<4 ; i++){
        error =  speedData[i] - desiredSpeed[i]; 
        pidIntegral[i] = pidIntegral[i] + KI*error;
        pid[i] = (KP*error) + pidIntegral[i] +  KD*((error-(pidPreviousError[i]))*TOTAL_TIME); // se multiplica por 2 porque el tiempo es de 500milis entre dos errores calculados
        pidPreviousError[i] = error;

        if(pid[i] > MAX_ANGULAR_SPEED ){
            pid[i] = MAX_ANGULAR_SPEED;
        }else if (pid[i]<-MAX_ANGULAR_SPEED)
        {
            pid[i]  = -MAX_ANGULAR_SPEED;
        }
        pid[i] = (pid[i]/4); // considerando que la velocidad deseada es maximo 400 rad/s y lo mapeamos entre 0 y 100
    }
}


int main() {
    stdio_init_all();
    AngleData startAngle; // struct with all start angles
    uint8_t encoder = 0;
    // measure the time in microseconds with function time_us_64()
    uint64_t current_micros = 0;
    uint64_t previous_micros = 0;
    uint64_t ayudamedios = 0;
    
    sleep_ms(5000);
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state for all encoders, afte it , i2c encoders 2 and 4 are active
    
    // Initialize start angle for each one encoder.
    // First encoder after checkmagnegPresent is ENCODER_2 that is conected to i2c0
    obtainAngle(i2c0,0); // angle for starting - RUEDA 2
    startAngle[2]  =  correctedAngle; //(totalAngle*180)/3.141592; // save the angle in grades, its necessary in function obtainAngle
    switchI2c(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2); // Enable encoder 1 i2c and disable encoder 2 i2c
    obtainAngle(i2c0,0); // angle for starting - RUEDA 1
    startAngle[0]  = correctedAngle; // (totalAngle*180)/3.141592;

    // Four encoder after checkmagnaegPreset is ENCODER_4 that is conected to i2c1
    obtainAngle(i2c1,0); // angle for starting - RUEDA 4
    startAngle[3]  = correctedAngle; // (totalAngle*180)/3.141592;
    switchI2c(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3); // Enable encoder 3 i2c and disable encoder 4 i2c
    obtainAngle(i2c1,0); // angle for starting - RUEDA 3
    startAngle[1]  = correctedAngle; // (totalAngle*180)/3.141592;

    sleep_ms(5000); // sleep 5 ms previo a calcular las velocidades angulares.

    while(true){

        

        for(int i = 0 ; i<4; i++){

            switch(i)
            {
            case 1:
                switchI2c(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0);
                break;
            case 2:
                switchI2c(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1);
            }  

            numberTurns = 0;
            previousQuadrantNumber = 0;
            current_micros = time_us_64();
            previous_micros =  current_micros;
            ayudamedios = current_micros;

            while (true)
            {
                current_micros  = time_us_64();

                if(((current_micros - ayudamedios)) >= SAMPLING_TIME){
                    ayudamedios = current_micros;

                    if(i==0 || i == 2)
                        obtainAngle(i2c0, startAngle[i]);
                    else 
                        obtainAngle(i2c1, startAngle[i]);
                }

                if((current_micros - previous_micros) >= TIME_WINDOW_US){
                    // printf("velocidad angular: %f\n", angulo_inicial);
                    printf("v: %" PRId64 "\n", numberTurns);
                    printf("vel: %f \n", TO_RAD(correctedAngle, numberTurns)*INV_TIME_WINDOW_S); 
                    speedData[i] = TO_RAD(correctedAngle, numberTurns)*INV_TIME_WINDOW_S; // angular speed  
                    break;
                } 

            }
            
        }

        calcularControlPID();  
        switchI2c(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2);
        switchI2c(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3);
        //__wfi(); // a mimir, y esperar un evento
    } // end of while(1)


        
     

    return 0;
}

