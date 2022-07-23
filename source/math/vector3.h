/**
 * @file vector3.h
 * @author German Moreno Escudero
 * @brief Tools to manage 3 dimension vectors
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __vector_3_h__
#define __vector_3_h__

#include "errors.h"

/**
 * @brief Copy a 3x1 vector
 * 
 * @param a (input) Vector to copy
 * @param output (output) Copied vector
 * @return ERROR_CODE 
 */
ERROR_CODE vector3_copy(double a[3], double output[3]);
/**
 * @brief Add two 3x1 vectors. a+b
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) The result vector of a+b
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_add(double a[3], double b[3], double output[3]);
/**
 * @brief Substract two 3x1 vectors. a-b
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) The result vector of a-b
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_substract(double a[3], double b[3], double output[3]);
/**
 * @brief Rotate a 3x1 vector 90 degrees in Y axis
 * 
 * @param a (input) Vector to rotate
 * @param output (output) The rotated vector
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_rotate90y(double a[3], double output[3]);
/**
 * @brief Rotate a 3x1 vector -90 degrees in Y axis
 * 
 * @param a (input) Vector to rotate
 * @param output (output) The rotated vector
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_rotateMinus90y(double a[3], double output[3]);
/**
 * @brief Scale a given vector by a scaling factor
 * 
 * @param a (input) Vector to scale
 * @param scale (input) Scaling factor
 * @param output (output) Scaled vector
 * @return ERROR_CODE 
 */
ERROR_CODE vector3_scale(double a[3], double scale, double *output);
/**
 * @brief Compute the dot product between two vectors. a*b
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) Result value
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_dot(double a[3], double b[3], double *output);
/**
 * @brief Compute the cross product between two vectors. a x b
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) Result vector
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_cross(double a[3], double b[3], double output[3]);
/**
 * @brief Compute the norm value of a given vector
 * 
 * @param a (input) Vector to obtain norm
 * @param output (output) Result value
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_norm(double a[3], double *output);
/**
 * @brief Normalize a given vector to make its length be 1
 * 
 * @param a (input) Vector to normalize
 * @param output (output) Normalized vector
 * @return ERROR_CODE: RET_OK on succes
 */
ERROR_CODE vector3_normalize(double a[3], double output[3]);
/**
 * @brief Compute the angle between two vectors
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) The angle between both given vectors
 * @return ERROR_CODE 
 */
ERROR_CODE vector3_angleBetweenTwoVectorsCalculate(double a[3], double b[3], double *output);

#endif