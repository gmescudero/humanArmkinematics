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

/**
 * @brief Add two 3x1 vectors
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output (output) The result vector of a+b
 */
void vector3_add(double a[3], double b[3], double output[3]);
/**
 * @brief Substract two 3x1 vectors
 * 
 * @param a (input) First vector
 * @param b (input) Second vector
 * @param output The result vector of a-b
 */
void vector3_substract(double a[3], double b[3], double output[3]);

#endif