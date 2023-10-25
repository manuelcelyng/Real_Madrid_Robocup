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

    // Perform matrix-vector multiplication to get dq
    dq[0][0] = R[0][0] * U[0][0] + R[0][1] * U[1][0] + R[0][2] * U[2][0];
    dq[1][0] = R[1][0] * U[0][0] + R[1][1] * U[1][0] + R[1][2] * U[2][0];
    dq[2][0] = R[2][0] * U[0][0] + R[2][1] * U[1][0] + R[2][2] * U[2][0];
}

void control(float e[3][1], float ek[3][1], float q[3][1], float uk[3][1], float U[3][1]) {
    float kc = 100;
    float v_max = 4.54;
    float w_max = 1.57;
    float ti = 0.01;
    float ts = 0.01;
    float phi = q[2][0];
    float q0 = kc * (1 + ts / (2 * ti));
    float q1 = -kc * (1 - ts / (2 * ti));
    
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
        u[i][0] = uk[i][0] + q0 * e[i][0] + q1 * ek[i][0];
    }

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

int main() {
    stdio_init_all();
    float delta = 0.01;
    float q[3][1] = {{-0.2}, {1}, {0}};
    float T = 2;
    float b = 2 * 3.141592653589793 / T;
    float ek[3][1] = {{0}, {0}, {0}};
    float U[3][1] = {{0}, {0}, {0}};
    float uk[3][1] = {{0}, {0}, {0}};
    float e[3][1] = {{0}, {0}, {0}};
    for (int i = 0; i < 4000; i++) { // Assuming 4000 iterations, matching the Python code
        float t_i = i * delta;

        // Calculate qd
        float qd[3][1];
        qd[0][0] = sinf(b * t_i);
        qd[1][0] = sinf(b * t_i);
        qd[2][0] = 0;

        // Update ek
        float ek_temp[3][1];
        for (int j = 0; j < 3; j++) {
            ek[j][0] = e[j][0];
        }

        // Calculate e
        for (int j = 0; j < 3; j++) {
            e[j][0] = qd[j][0] - q[j][0];
        }

        // Call the control function to update U
        uk[0][0] = U[0][0];
        uk[1][0] = U[1][0];
        uk[2][0] = U[2][0];
        control(e, ek, q, uk, U);

        // Call the planta function to calculate dq and dteta
        float dq[3][1];
        float dteta[4][1];
        planta(U, q, dq, dteta);

        // Update q
        for (int j = 0; j < 3; j++) {
            q[j][0] += dq[j][0] * delta;
        }

        printf("%f,%f,%f\n",q[0][0], qd[0][0], dteta[1][0]);
        sleep_ms(10);
    }
    while (true) {
        //printf("Prendido\n");
        sleep_ms(250);
    }
}
