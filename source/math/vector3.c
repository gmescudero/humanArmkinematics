/**
 * @file vector3.c
 * @author German Moreno Escudero
 * @brief Tools to manage 3 dimension vectors
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "vector3.h"


void vector3_add(double a[3], double b[3], double output[3])
{
    output[0] = a[0] + b[0];
    output[1] = a[1] + b[1];
    output[2] = a[2] + b[2];
}

void vector3_substract(double a[3], double b[3], double output[3])
{
    output[0] = a[0] - b[0];
    output[1] = a[1] - b[1];
    output[2] = a[2] - b[2];
}
