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

ERROR_CODE vector3_scale(double a[3], double scale, double *output) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] = a[0]*scale;
    result[1] = a[1]*scale;
    result[2] = a[2]*scale;

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

ERROR_CODE vector3_cross(double a[3], double b[3], double output[3]) {
    double result[3];
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == b)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];

    return vector3_copy(result,output);
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

ERROR_CODE vector3_angleBetweenTwoVectorsCalculate(double a[3], double b[3], double *output) {
    ERROR_CODE status;
    double aux[3];
    double sinAngle;
    double cosAngle;
    // Check arguments
    if (NULL == a)      return RET_ARG_ERROR;
    if (NULL == b)      return RET_ARG_ERROR;
    if (NULL == output) return RET_ARG_ERROR;

    status = vector3_dot(a,b, &cosAngle);
    if (RET_OK == status) {
        status = vector3_cross(a,b,aux);
    }
    if (RET_OK == status) {
        status = vector3_norm(aux, &sinAngle);
    }
    if (RET_OK == status) {
        *output = atan2(sinAngle, cosAngle);
    }

    return status;
}

void vector3_to_spherical_coordinates_convert(double vector[3], double *theta, double *rho, int *shperical_convention) {
    *theta = atan2(sqrt(vector[0]*vector[0]+vector[1]*vector[1]) , vector[2]);
    if (fabs(sin(*theta)) < 0.5) {
        // Avoid singularity
        *shperical_convention = 1;
        *theta = atan2(sqrt(vector[2]*vector[2]+vector[1]*vector[1]) , vector[0]);
        *rho   = atan2(vector[1],vector[2]);
    }
    else {
        *shperical_convention = 0;
        *rho   = atan2(vector[1],vector[0]);
    }
}

/**
 * @brief Convert a set of spherical coordinates to its 3D vector
 * 
 * @param theta (input) Theta angle of spherical coordinates
 * @param rho (input) Rho angle of spherical coordinates
 * @param shperical_convention (input) Convention used to get shperical coordinates 
 * @param vector (output) Converted vector
 */
void vector3_from_spherical_coordinates_convert(double theta, double rho, int shperical_convention, double vector[3]) {
    double ct = cos(theta);
    double st = sin(theta);
    double cr = cos(rho); 
    double sr = sin(rho);

    if (0 == shperical_convention) {
        vector[0] = st*cr; vector[1] = st*sr; vector[2] = ct;
    }
    else {
        vector[0] = ct; vector[1] = st*sr; vector[2] = st*cr;
    }
}

/**
 * @brief Compute the derivatives of the spherical representation of 3D vectors
 * 
 * @param theta (input) Theta angle of spherical coordinates
 * @param rho (input) Rho angle of spherical coordinates
 * @param shperical_convention (input) Convention used to get shperical coordinates 
 * @param dtheta (output) Derivative with respect to theta
 * @param drho (output) Derivative with respect to rho
 */
void vector3_spherical_coordinates_derivatives_compute(double theta, double rho, int shperical_convention, double dtheta[3],double drho[3]) {
    double ct = cos(theta);
    double st = sin(theta);
    double cr = cos(rho); 
    double sr = sin(rho);

    if (0 == shperical_convention) {
        dtheta[0] =  ct*cr; dtheta[1] = ct*sr; dtheta[2] = -st;
          drho[0] = -st*sr;   drho[1] = st*cr;   drho[2] = 0.0;
    }
    else { // Alternative
        dtheta[0] = -st; dtheta[1] = ct*sr; dtheta[2] =  ct*cr;  
          drho[0] = 0.0;   drho[1] = st*cr;   drho[2] = -st*sr;
    }
}