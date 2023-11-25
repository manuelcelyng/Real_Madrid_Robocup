#ifndef CONTROL_CAR_H
#define CONTROL_CAR_H

// Función - Representación del carro
void planta(float U[3][1], float q[3][1], float dq[3][1], float dteta[4][1]);

// Función - Calcula el control PI
void control(float e[3][1], float ek[3][1], float q[3][1], float uk[3][1], float U[3][1]);

void ejecutarMovimiento(char* move, int value1, int value2);

extern bool run_command;
extern int select_movement;

#endif