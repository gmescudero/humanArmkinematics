/**
 * @file matrix.h
 * @author Germán Moreno Escudero
 * @brief Tools to manage matrixes
 * @version 0.1
 * @date 2022-07-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __matrix_h__
#define __matrix_h__

#include "errors.h"

typedef struct MATRIX_STRUCT
{
    unsigned rows;
    unsigned cols;
    double **data;
}MATRIX;

/**
 * @brief Allocate memory for a matrix
 * 
 * @param rows (input) Number of rows
 * @param cols (input) Number of columns
 * @return MATRIX with allocated memory 
 */
MATRIX matrix_allocate(unsigned rows, unsigned cols);
/**
 * @brief Allocate memory for a square matrix and initialize it with an identity matrix
 * 
 * @param size (input) Range of the matrix
 * @return MATRIX 
 */
MATRIX matrix_identity_allocate(unsigned size);
/**
 * @brief Free the Matrix allocated memory
 * 
 * @param a (input) Matrix to be freed
 */
void matrix_free(MATRIX a);

/**
 * @brief Copy a given matrix into another one 
 * 
 * @param a (input) Matrix to copy
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_copy(MATRIX a, MATRIX *output);
/**
 * @brief Transpose a given matrix
 * 
 * @param a (input) Matrix to transpose
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_transpose(MATRIX a, MATRIX *output);
/**
 * @brief Add two given matrixes
 * 
 * @param a (input) First matrix
 * @param b (input) Second matrix
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_add(MATRIX a, MATRIX b, MATRIX *output);
/**
 * @brief Multiply two given matrixes
 * 
 * @param a (input) First matrix
 * @param b (input) Second matrix
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_multiply(MATRIX a, MATRIX b, MATRIX *output);
/**
 * @brief Compute the inverse of a matrix
 * 
 * @param a (input) Matrix to invert
 * @param output (output) Inverted matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_inverse(MATRIX a, MATRIX *output);
/**
 * @brief Compute the Moore-Penrose pseudoinverse of a matrix
 * 
 * @param a (input) Matrix to invert
 * @param output (output) Inverted matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_pseudoinverse(MATRIX a, MATRIX *output);
/**
 * @brief Show a given Matrix
 * 
 * @param a (input) Matrix to print
 * @param name (input) Specific name of the matrix. Can be NULL
 */
void matrix_print(MATRIX a, const char *name);
#endif /* __matrix_h__ */