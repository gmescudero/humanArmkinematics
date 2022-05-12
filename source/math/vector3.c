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
#include "math.h"
#include "general.h"
#include "constants.h"
#include <stddef.h>


ERROR_CODE vector3_copy(double a[3], double output[3]) {
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    output[0]=a[0]; output[1]=a[1]; output[2]=a[2];

    return RET_OK;
}

ERROR_CODE vector3_add(double a[3], double b[3], double output[3]) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == b)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] = a[0] + b[0];
    result[1] = a[1] + b[1];
    result[2] = a[2] + b[2];

    return vector3_copy(result,output);
}

ERROR_CODE vector3_substract(double a[3], double b[3], double output[3]) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == b)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];

    return vector3_copy(result,output);
}

ERROR_CODE vector3_rotate90y(double a[3], double output[3]) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] = -a[2];
    result[1] =  a[1];
    result[2] =  a[0];

    return vector3_copy(result,output);
}

ERROR_CODE vector3_rotateMinus90y(double a[3], double output[3]) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] =  a[2];
    result[1] =  a[1];
    result[2] = -a[0];

    return vector3_copy(result,output);
}

ERROR_CODE vector3_dot(double a[3], double b[3], double *output) {
    double result;
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == b)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];

    *output = result;

    return RET_OK;
}

ERROR_CODE vector3_norm(double a[3], double *output) {
    double result;
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

    *output = result;

    return RET_OK;
}

ERROR_CODE vector3_normalize(double a[3], double output[3]) {
    ERROR_CODE status;
    double result[3];
    double norm;
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    status = vector3_norm(a, &norm);
    if (RET_OK == status) {
        if (EPSI > norm) {
            // A vector of zeros cannot be normalized
            status = RET_ERROR;
        }
        else {
            result[0] = a[0]/norm;
            result[1] = a[1]/norm;
            result[2] = a[2]/norm;
        }
    }
    if (RET_OK == status) {
        status = vector3_copy(result,output);
    }

    return status;
}