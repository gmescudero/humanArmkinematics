#include "vector3.h"

/**
 * Adds two vectors of length 3
 */
void vector3_add(double a[3], double b[3], double output[3])
{
    output[0] = a[0] + b[0];
    output[1] = a[1] + b[1];
    output[2] = a[2] + b[2];
}

/**
 * Substracts two vectors of length 3
 */
void vector3_substract(double a[3], double b[3], double output[3])
{
    output[0] = a[0] - b[0];
    output[1] = a[1] - b[1];
    output[2] = a[2] - b[2];
}
