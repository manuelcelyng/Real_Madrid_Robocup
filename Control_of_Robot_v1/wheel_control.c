#include "pico/stdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <inttypes.h>
// include my own .h
#include "wheel_control.h"
#include "imu.h"


// All struct for PID control
SpeedData speedData = {0.0, 0.0, 0.0, 0.0};
DesiredSpeedData desiredSpeed = {0.0, 0.0, 0.0, 0.0};
PIDData pid = {0.0, 0.0, 0.0, 0.0};
PIDIntegralData pidIntegral = {0.0, 0.0, 0.0, 0.0};
PIDErrorData pidPreviousError = {0.0, 0.0, 0.0, 0.0};

// constants of each one wheel 
/*   Positions
0 -> Rueda 1
1 -> Rueda 3
2 -> Rueda 2
3 -> Rueda 4
*/
// CONSTANTES
ConstantsP constansP_C = {KP_0, KP_1, KP_2, KP_3};
ConstantsI constansI_C = {KI_0, KI_1, KI_2, KI_3};
ConstantsD constansD_C = {KD_0, KD_1, KD_2, KD_3};
// CONSTANTES ADAPTATIVAS
ConstantsP constansP = {KP_0, KP_1, KP_2, KP_3};
ConstantsI constansI = {KI_0, KI_1, KI_2, KI_3};
ConstantsD constansD = {KD_0, KD_1, KD_2, KD_3};

// for quadrant
uint8_t previousQuadrantNumber = 0;
int numberTurns = 0;
int flagHelp = 0;

// for total angle calculated
double correctedAngle= 0;

// addres of registers in the encoder
const uint8_t STATUS = ENCODER_STATUS;
const uint8_t RAWANGLE_H = ENCODER_RAWANGLE_H;
const uint8_t RAWANGLE_L = ENCODER_RAWANGLE_L;


bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


void initI2C(){
    // configure with speed mode plus -> 1000kbps for i2c0 and i2c1
    i2c_init(i2c_default, 400 * 1000);
    i2c_init(i2c1, 1000 * 1000);
    // configura 1 encoder y lo deja listo para el inicio de lectura.
    gpio_set_function(ENCODER_I2C_SDA_PIN_0, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN_0, GPIO_FUNC_I2C);
     // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_0);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_0);

    // configura 2 encoder 
    gpio_set_function(ENCODER_I2C_SDA_PIN_1, GPIO_FUNC_NULL);
    gpio_set_function(ENCODER_I2C_SCL_PIN_1, GPIO_FUNC_NULL);
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

     // configura la imu
    gpio_set_function(IMU_I2C_SDA_PIN,  GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN,  GPIO_FUNC_I2C);

    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_0,ENCODER_I2C_SCL_PIN_0, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_1,ENCODER_I2C_SCL_PIN_1, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_2,ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(ENCODER_I2C_SDA_PIN_3,ENCODER_I2C_SCL_PIN_3, GPIO_FUNC_I2C));
    bi_decl(bi_2pins_with_func(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, GPIO_FUNC_I2C));
    

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

        while ((magnedStatus&32)!= 32) // while the magnet is not adjusted to the proper distante - 32 : MD=1
        {
            
            i2c_write_blocking(i2c1, ENCODER_ADDR,&STATUS,1, true); // send for read data in Register Status
            i2c_read_blocking(i2c1, ENCODER_ADDR, &magnedStatus,1,false); // Read data sended from encoder status register
           
            printf("value is for encoder %d : %d\n", equisde+1 ,magnedStatus);
            sleep_ms(10);
        }

        printf("Magnet Found! Encoder: %d \n ", equisde+1);
        
        sleep_ms(30);

        
    }
    

}


// The main function for calculate angule in the wheel
void obtainAngle( double startAngle, bool start){
    
    // buffer[0] 0000000000001111  -> encoder send us bits 8:11
    // 0000000011111111[1]  -> encoder send us bits 0:7   // TOTAL 12 BITS INFORMATION
    uint8_t buffer[2];  // buffer[0] guarda los más significativos, y buffer[1] guarda los menos significativos.
    _uint_16_t aux;

    
    // info de los registros y los estados del enconder. 
    i2c_write_blocking(i2c1, ENCODER_ADDR,&RAWANGLE_L,1, true);
    i2c_read_blocking(i2c1, ENCODER_ADDR, &buffer[1],1,true);

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(i2c1, ENCODER_ADDR,&RAWANGLE_H,1, true);
    i2c_read_blocking(i2c1, ENCODER_ADDR, &buffer[0],1,false);

    // obtain the angle and print
   
    aux.rawAngle = (buffer[0]<<8) | buffer[1];   // example, higbye = 00000000|00001111, le plicamos <<8  resulta -> 00001111|00000000
    // finalmente : 00000111100000000 or 0000000011111111 =  00001111|11111111

    correctedAngle = (double)aux.rawAngle * 0.087890625;

    //printf("CorrectedAngle: %f \n" , correctedAngle);
    // quadrant for turns 
    /*
    //quadrants
    4  |  1
    ---|---
    3  |  2
    */
    if(!start){

            // CORRECTED ANGLE  
        correctedAngle = correctedAngle - startAngle;
        if(correctedAngle<0){ 
            correctedAngle = correctedAngle + 360;
        }  
        // define the quadrant
        aux.quadrantNumber = (int) (correctedAngle/90) + 1;
        //printf("Quadrante:  %d  \n", aux.quadrantNumber);

        // verify for count the turn and not have false counts
        

        // count the turn
        if(aux.quadrantNumber != previousQuadrantNumber){

            if(aux.quadrantNumber ==2 && previousQuadrantNumber == 1){
             flagHelp++;
            }

            if(aux.quadrantNumber ==3 && previousQuadrantNumber == 2){
             flagHelp++;
            }

            if(flagHelp==2){
                if ((aux.quadrantNumber == 1 && previousQuadrantNumber == 4  ) ||
                (aux.quadrantNumber == 4 && previousQuadrantNumber == 1  )) {

                    numberTurns++;
                }
            }
            

            previousQuadrantNumber = aux.quadrantNumber;
        }
    }
    
}

void switchI2c(uint sda_enable, uint scl_enable, uint sda_disable , uint scl_disable){
    // enable 2 pin gpio for use I2C
    gpio_set_function(sda_enable, GPIO_FUNC_I2C);
    gpio_set_function(scl_enable, GPIO_FUNC_I2C);
    // disable 2 pin gpio for dont use I2C
    gpio_set_function(sda_disable, GPIO_FUNC_NULL);
    gpio_set_function(scl_disable, GPIO_FUNC_NULL);
}

//calculates the control of each wheel, and save their values on the structure
void calcularControlPID(int i){
    double error = 0; 
    // for all motors
    // for(int i=0 ; i<4 ; i++){
        
    error =  speedData[i] - desiredSpeed[i]; 
    // printf("Speed: %f, %d\n", speedData[i], i);
    pidIntegral[i] = (pidIntegral[i] + constansI[i]*error)/TOTAL_TIME;
    pid[i] = (constansP[i]*error) + pidIntegral[i] +  constansD[i]*((error-(pidPreviousError[i]))*TOTAL_TIME);
    pidPreviousError[i] = error;

    // if for contro max speed angular in each wheel
    if(pid[i] > 50){
        pid[i] = 50;
    }else if (pid[i]< -50)
    {
        pid[i]  = -50;
    }
    
    // divided by 4 taking into account the max speed -> only use the 25% of PID calculated
    pid[i] = (pid[i]/4); // considerando que la velocidad deseada es maximo 400 rad/s y lo mapeamos entre 0 y 100
    // }
    
}


