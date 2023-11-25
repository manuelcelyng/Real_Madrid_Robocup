#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

void planta(float U[3][1], float q[3][1], float dq[3][1], float dteta[4][1]) {
    float phi = q[2][0];
    float r = 0.1;
    float L = 1;
    
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

    // Calculate the elements of the rotation matrix A
    float A[4][3];
    A[0][0] = -0.7071067/r;
    A[0][1] = 0.7071067/r;
    A[0][2] = L/r;
    A[1][0] = -0.7071067/r;
    A[1][1] = -0.7071067/r;
    A[1][2] = L/r;
    A[2][0] = 0.7071067/r;
    A[2][1] = -0.7071067/r;
    A[2][2] = L/r;
    A[3][0] = 0.7071067/r;
    A[3][1] = 0.7071067/r;
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
        if (dteta[i][0] > 200.0) {
            dteta[i][0] = 200.0;
        } else if (dteta[i][0] < -200.0) {
            dteta[i][0] = -200.0;
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
    float w_max = 2;
    float ti = 0.01;
    float ts = 0.06;
    float td = 0.001;
    float phi = q[2][0];
    float q0 = kc * (1 + ts / (2 * ti) + td / ts);
    float q1 = -kc * (1 - ts / (2 * ti) + 2*td / ts);
    float q2 = -kc *td/ts;
    float k[3][1] = {{1}, {1}, {0.01}};
    
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
<<<<<<< HEAD
        u[i][0] = (q0 * e[i][0] + q1 * ek[i][0] + q2 * ek2[i][0])*k[i][0] + uk[i][0];//uk[i][0] + q0 * e[i][0] + q1 * ek[i][0];
=======
        if (i == 2 && e[i][0]<0.1745 && e[i][0]>-0.1745)
        {
            u[i][0] = 0;
        }else {
            u[i][0] = kc* e[i][0]*k[i][0];//uk[i][0] + q0 * e[i][0] + q1 * ek[i][0];
        }
>>>>>>> b88f72f97c76e27e45daede1ad7e941b4e8bdff6
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