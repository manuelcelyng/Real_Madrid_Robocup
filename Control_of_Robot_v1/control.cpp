#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "dribbleo.hpp"

bool run_command = false;
int select_movement = 0; // 0 , no hace nada.
int value1 = 0;
int value2 = 0;

void planta(float U[3][1], float q[3][1], float dq[3][1], float dteta[4][1]) {
    float phi = q[2][0];
    float r = 0.0325;
    float L = 0.09;
    
    // Calculate the elements of the rotation matrix R
    float R[3][3];
    R[0][0] = cosf(phi);
    R[0][1] = -sinf(phi);
    R[0][2] = 0;
    R[1][0] = sinf(phi);
    R[1][1] = cosf(phi);
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;  

    // Calculate the elements of the rotation matrix A r=3.25cm o 0.0325  y L=9cm o 0.09
    // float A[4][3];
    // A[0][0] = -0.7071067/r;
    // A[0][1] = 0.7071067/r;
    // A[0][2] = L/r;
    // A[1][0] = -0.7071067/r;
    // A[1][1] = -0.7071067/r;
    // A[1][2] = L/r;
    // A[2][0] = 0.7071067/r;
    // A[2][1] = -0.7071067/r;
    // A[2][2] = L/r;
    // A[3][0] = 0.7071067/r;
    // A[3][1] = 0.7071067/r;
    // A[3][2] = L/r;

    float A[4][3];
    A[0][0] = -0.8660254/r;
    A[0][1] = 0.5/r;
    A[0][2] = L/r;
    A[1][0] = 0.8660254/r;
    A[1][1] = 0.5/r;
    A[1][2] = L/r;
    A[2][0] = 0.7071067/r;
    A[2][1] = -0.7071067/r;
    A[2][2] = L/r;
    A[3][0] = -0.7071067/r;
    A[3][1] = -0.7071067/r;
    A[3][2] = L/r;

    dteta[0][0] = A[0][0]*U[0][0] + A[0][1]*U[1][0] + A[0][2]*U[2][0];
    dteta[1][0] = A[1][0]*U[0][0] + A[1][1]*U[1][0] + A[1][2]*U[2][0];
    dteta[2][0] = A[2][0]*U[0][0] + A[2][1]*U[1][0] + A[2][2]*U[2][0];
    dteta[3][0] = A[3][0]*U[0][0] + A[3][1]*U[1][0] + A[3][2]*U[2][0];

    dteta[0][0] *= 5;
    dteta[1][0] *= 5;
    dteta[2][0] *= 5;
    dteta[3][0] *= 5;
    // Limitar dteta al rango de -400 a 400
    for (int i = 0; i < 4; i++) {
        if (dteta[i][0] > 150.0) {
            dteta[i][0] = 150.0;
        } else if (dteta[i][0] < -150.0) {
            dteta[i][0] = -150.0;
        }
    }

    // Perform matrix-vector multiplication to get dq
    dq[0][0] = R[0][0] * U[0][0] + R[0][1] * U[1][0] + R[0][2] * U[2][0];
    dq[1][0] = R[1][0] * U[0][0] + R[1][1] * U[1][0] + R[1][2] * U[2][0];
    dq[2][0] = R[2][0] * U[0][0] + R[2][1] * U[1][0] + R[2][2] * U[2][0];
}

void control(float e[3][1], float ek[3][1], float ek2[3][1], float q[3][1], float uk[3][1], float U[3][1]) {
    float kc = 100;
    float v_max = 10.0;
    float w_max = 8;
    float ti = 0.01;
    float ts = 0.06;
    float td = 0.001;
    float phi = q[2][0];
    float q0 = kc * (1 + ts / (2 * ti) + td / ts);
    float q1 = -kc * (1 - ts / (2 * ti) + 2*td / ts);
    float q2 = -kc *td/ts;
    float k[3][1] = {{1}, {1}, {0.06}};
    
    // Calculate the rotation matrix R
    float R[3][3];
    R[0][0] = cosf(phi);
    R[0][1] = -sinf(phi);
    R[0][2] = 0;
    R[1][0] = sinf(phi);
    R[1][1] = cosf(phi);
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;

    // Calculate u
    float u[3][1];
    for (int i = 0; i < 3; i++) {
        //u[i][0] = (q0 * e[i][0] + q1 * ek[i][0] + q2 * ek2[i][0])*k[i][0] + uk[i][0];//uk[i][0] + q0 * e[i][0] + q1 * ek[i][0];
        if (i == 2 && e[i][0]<0.1 && e[i][0]>-0.1){
            u[i][0] = 0;
        }else {
            u[i][0] = kc*(e[i][0])*k[i][0];//uk[i][0] + q0 * e[i][0] + q1 * ek[i][0];
        }
        uk[i][0] = u[i][0];
    }
    //

    // Calculate the inverse of R
    float Rinv[3][3];
    Rinv[0][0] = cosf(phi);
    Rinv[0][1] = sinf(phi);
    Rinv[0][2] = 0;
    Rinv[1][0] = -sinf(phi);
    Rinv[1][1] = cosf(phi);
    Rinv[1][2] = 0;
    Rinv[2][0] = 0;
    Rinv[2][1] = 0;
    Rinv[2][2] = 1;
    // Calculate the inverse matrix Rinv from R (you may use a library or implement matrix inversion)

    // Calculate U by multiplying Rinv and u
    float Utemp[3][1];
    Utemp[0][0] = R[0][0] * u[0][0] + R[0][1] * u[1][0] + R[0][2] * u[2][0];
    Utemp[1][0] = R[1][0] * u[0][0] + R[1][1] * u[1][0] + R[1][2] * u[2][0];
    Utemp[2][0] = R[2][0] * u[0][0] + R[2][1] * u[1][0] + R[2][2] * u[2][0];
    // Apply the velocity limits
    for (int i = 0; i < 2; i++) {
        if (Utemp[i][0] > v_max) {
            U[i][0] = v_max;
        } else if (Utemp[i][0] < -v_max) {
            U[i][0] = -v_max;
        } else {
            U[i][0] = Utemp[i][0];
        }
    }
    if (Utemp[2][0] > w_max) {
        U[2][0] = w_max;
    } else if (Utemp[2][0] < -w_max) {
        U[2][0] = -w_max;
    } else {
        U[2][0] = Utemp[2][0];
    }
}


// Funcion que llama el bluetooth
void ejecutarMovimiento(char* move, int value_1, int value_2){
    // strcmp(move, "T")        // Si se quiere comparar una cadena más larga en el if que no sea de un solo caracter
    // Movimiento GIRO, utilizar value1, es el valor del ÁNGULO
    if(move[0] == 'T') {
        // Giro sobre su propio eje un ángulo determinado ( Parámetros : ángulo)
        value1 = value_1;
        //printf("GIRO %d\n", value_1);
        select_movement = 1;
        run_command =  true;
    }

    // Movimiento DESPLAZAMIENTO, utilizar value1, es el valor de la ángulo
    if(move[0] == 'D') {
        // Desplazamiento en linea recta para diferentes direcciones con relación al frente del robot y durante una distancia determinada ( Parámetros: ángulo y distancia.)
        //printf("DESPLAZAMIENTO %d\n", value_1);
        value1 = value_1;
        value2 = value_2;  
        select_movement = 2;
        run_command =  true;
    }

    // Movimiento DESPLAZAMIENTO CIRCULAR, utilizar value1 es el RADIO, value2 es el ÁNGULO
    if(move[0] == 'C') {
        value1 = value_1;
        value2 = value_2; 
        //  Desplazamiento circular con un radio determinado ( parámetros :  radio ,  ángulo)
        //printf("DESPLAZAMIENTO CIRCULAR %d, %d\n", value_1, value_2);
        select_movement = 3;
        run_command =  true;
    }

    // Movimiento de DRIBBLIGN
    if(move[0] == 'A') {
        dribbleo->activeDribbleo(true, false);
        run_command =  true;
    }

     // Movimiento de KICK/PATEO
    if(move[0] == 'K') {
        dribbleo->activeDribbleo(true, false);
        run_command =  true;
    }

     // Movimiento de STOP DRIBBLIGN
    if(move[0] == 'G') {
        dribbleo->activeDribbleo(false, false);
        run_command =  true;
    }
}