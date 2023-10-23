#include "pico/stdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <inttypes.h>
// include my own .h
#include "encoder.h"

double startAngle=0; // angle calculated with encoder

//for PID
/*
float pid_p  = 0;       // proportional
float pid_i  = 0;       // integral 
float pid_d  = 0;       // derivativo
float error  = 0;       // for the current error
float prev_error = 0;   // for the last error
float velo_deseada = 0; // for desired w
float velo_medida = 0;  // for measured w
*/

// measure the time in microseconds with function time_us_64()
uint64_t current_micros = 0;
uint64_t previous_micros = 0;

// for quadrant
uint8_t previousQuadrantNumber = 0;
uint64_t numberTurns = 0;

#ifdef deBUG
1int numberTurns;
int quadrantNumber;
int previousQuadrantNumber = 0;
#endif

const uint8_t STATUS = ENCODER_STATUS;
const uint8_t RAWANGLE_H = ENCODER_RAWANGLE_H;
const uint8_t RAWANGLE_L = ENCODER_RAWANGLE_L;


bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}


void initI2C(){
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
   
    i2c_init(i2c_default, 100 * 1000);
    i2c_init(i2c1, 100 * 1000);
    // configura 1 encoder
    gpio_set_function(ENCODER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN, GPIO_FUNC_I2C);
     // coloca en pull up 
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // configura 2 encoder 
    gpio_set_function(ENCODER_I2C_SDA_PIN_2, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_I2C);
    // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_2);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_2);

     // configura 3 encoder
    gpio_set_function(ENCODER_I2C_SDA_PIN_3, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_I2C_SCL_PIN_3, GPIO_FUNC_I2C);
     // coloca en pull up 
    gpio_pull_up(ENCODER_I2C_SDA_PIN_3);
    gpio_pull_up(ENCODER_I2C_SCL_PIN_3);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
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

// this function only is a verification of state of magnet in the encoder 
void checkMagnetPresent(){
    uint8_t magnedStatus =0;
    const uint8_t statuscoso  = 0x0b;
    while ((magnedStatus&32)!= 32) // while the magnet is not adjusted to the proper distante - 32 : MD=1
    {
        i2c_write_blocking(i2c_default, ENCODER_ADDR,&STATUS,1, true); // send for read data in Register Status
        i2c_read_blocking(i2c_default, ENCODER_ADDR, &magnedStatus,1,false); // Read data sended from encoder status register

        printf("value is : %d\n", magnedStatus);
        sleep_ms(1000);
    }

    printf("Magnet Found!");
    sleep_ms(3000);
    

}


double obtainInfo(i2c_inst_t * a){

    // buffer[0] 0000000000001111  -> encoder send us bits 8:11
    // 0000000011111111[1]  -> encoder send us bits 0:7   // TOTAL 12 BITS INFORMATION
    uint8_t buffer[2];  // buffer[0] guarda los más significativos, y buffer[1] guarda los menos significativos.
    uint16_t rawAngle;
    double degAngle;
    double correctedAngle;   
    uint8_t quadrantNumber;

    
    // info de los registros y los estados del enconder. 
    i2c_write_blocking(a, ENCODER_ADDR,&RAWANGLE_L,1, true);
    i2c_read_blocking(a, ENCODER_ADDR, &buffer[1],1,false);

    // info de los registros y los estados del enconder. 
    i2c_write_blocking(a, ENCODER_ADDR,&RAWANGLE_H,1, true);
    i2c_read_blocking(a, ENCODER_ADDR, &buffer[0],1,false);

    // obtain the angle and print
   
    rawAngle = (buffer[0]<<8) | buffer[1];   // example, higbye = 00000000|00001111, le plicamos <<8  resulta -> 00001111|00000000
    // finalmente : 00000111100000000 or 0000000011111111 =  00001111|11111111 
    degAngle = rawAngle * 0.087890625;
   

    // CORRECTED ANGLE 
    
    correctedAngle = degAngle - startAngle;
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
        quadrantNumber = 1;
   }
   if(correctedAngle>90 && correctedAngle <= 180){
        quadrantNumber = 2;
   }
   if(correctedAngle>180 && correctedAngle <= 270){
        quadrantNumber = 3;
   }
   if(correctedAngle>270 && correctedAngle < 360){
        quadrantNumber = 4;
   }

   if(quadrantNumber != previousQuadrantNumber){
        if(quadrantNumber == 1 &&  previousQuadrantNumber == 4){
            numberTurns++;
        }
        if(quadrantNumber == 4 &&  previousQuadrantNumber == 1){
            numberTurns--;
        }

        previousQuadrantNumber = quadrantNumber;

   }

   double totalAngle = ((double)(numberTurns)*360) + correctedAngle;

   totalAngle =  (totalAngle*3.141592)/180; // radianes

    // printf("total angle %f\n", totalAngle);

    return totalAngle;


}

#ifdef BUG
void correctAngle(){
    correctedAngle = degAngle - startAngle;

    if(correctedAngle<0){
        correctedAngle = correctedAngle + 360;
    }

    //printf("Corrected angle: %f\n",  correctedAngle);
}

void checkQuadrant(){
    /*
    //quadrants
    4  |  1
    ---|---
    3  |  2
    */

   //Quadrant 1
   if(correctedAngle>=0 && correctedAngle <= 90){
        quadrantNumber = 1;
   }
   if(correctedAngle>=90 && correctedAngle <= 180){
        quadrantNumber = 2;
   }
   if(correctedAngle>=180 && correctedAngle <= 270){
        quadrantNumber = 3;
   }
   if(correctedAngle>=270 && correctedAngle <= 360){
        quadrantNumber = 4;
   }

   if(quadrantNumber != previousQuadrantNumber){
        if(quadrantNumber == 1 &&  previousQuadrantNumber == 4){
            numberTurns++;
        }
        if(quadrantNumber == 4 &&  previousQuadrantNumber == 1){
            numberTurns--;
        }

        previousQuadrantNumber = quadrantNumber;

   }

   float totalAngle = ((float)(numberTurns/5.0)*360) + correctedAngle;

    if((numberTurns/5.0)>=1.0){
         printf("absolute position of the motor in degree angle  %f\n", totalAngle);
         printf("Number Turns  %f\n", numberTurns/5.0);
    }
  
  
}
#endif

int main() {
    stdio_init_all();
    double angulo_inicial = 0;
    double angulo_final = 0;
    float pid_p  = 0;       // proportional
    float pid_i  = 0;       // integral 
    float pid_d  = 0;       // derivativo
    float error  = 0;       // for the current error
    float prev_error = 0;   // for the last error
    double velo_deseada = 0; // for desired w
    double velo_medida = 0;  // for measured w
    uint64_t ayudamedios = 0;
    previous_micros = 0; // initial previous
    int cual = 0;
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    sleep_ms(5000);
    initI2C(); // initialize I2C, pins are defined in encoder.h
    checkMagnetPresent(); // block until magnet is found in required state
    startAngle = obtainInfo(i2c0); // angle for starting , for offset 
    startAngle =  (startAngle*180)/3.141592;
    gpio_set_function(ENCODER_I2C_SDA_PIN_2, GPIO_FUNC_NULL);
    gpio_set_function(ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_NULL);
    while(true){
        current_micros  = time_us_64();
         if(((current_micros - ayudamedios)) >= 1500){
            ayudamedios = current_micros;
            if(cual==0){
                angulo_inicial =  obtainInfo(i2c0);
              
                } // actualizar el angulo maximo para el motor girando a 7200rpm -> 1.5milis
            else{
                angulo_inicial =  obtainInfo(i2c1);
              
                }
         }
        
        if(((current_micros - previous_micros)) >= 1000000){
            // printf("velocidad angular: %f\n", angulo_inicial);
            printf("v: %" PRId64 "\n", numberTurns);
            // reinicio todo el conteo
            // startAngle  = 0 ;
            // startAngle  = obtainInfo();
            // startAngle =  (startAngle*180)/3.141592;
            numberTurns = 0;
            previousQuadrantNumber = 0;
            
            if(pid_d==0){
                gpio_set_function(ENCODER_I2C_SDA_PIN_2, GPIO_FUNC_I2C);
                gpio_set_function(ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_I2C);
                gpio_set_function(ENCODER_I2C_SDA_PIN, GPIO_FUNC_NULL);
                gpio_set_function(ENCODER_I2C_SCL_PIN, GPIO_FUNC_NULL);
                pid_d =1;
                cual=0;
              
            }
            else if (pid_d == 1)
            {
                gpio_set_function(ENCODER_I2C_SDA_PIN, GPIO_FUNC_I2C);
                gpio_set_function(ENCODER_I2C_SCL_PIN, GPIO_FUNC_I2C);
                gpio_set_function(ENCODER_I2C_SDA_PIN_2, GPIO_FUNC_NULL);
                gpio_set_function(ENCODER_I2C_SCL_PIN_2, GPIO_FUNC_NULL);
                pid_d=2;
                
            }else{
                cual = 1;
                pid_d=0;
            }

            current_micros = time_us_64();
            previous_micros =  current_micros;
            ayudamedios = current_micros;
            
        }
        
    }


        //printf("velocidad angular :  %f   %llu  %llu  %f %f  \n ", velo_medida,tiempo_final,tiempo_inicial,angulo_final,angulo_inicial);
        
     







        /*
        // error
        error = velo_deseada-velo_medida;
        // Codigo del control PID
        pid_p =  Kp*error;
        pid_i = pid_i + Ki*error; 
        pid_d = (error-prev_error)/((tiempo_final-tiempo_inicial)/1e6);

        prev_error =  error;
        */
    
#endif
    return 0;
}

