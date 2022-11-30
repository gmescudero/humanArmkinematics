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
#include <stdbool.h>

typedef struct MATRIX_STRUCT
{
    bool allocated;
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
 * @brief Allocate memory for a matrix based on a given one
 * 
 * @param a (input) Base Matrix
 * @return MATRIX 
 */
MATRIX matrix_from_matrix_allocate(MATRIX a);
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
 * @brief Multiply a matrix by a given value
 * 
 * @param a (input) Matrix to scale
 * @param scale (input) Scale value
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_scale(MATRIX a, double scale, MATRIX *output);
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
 * @brief Substract two given matrixes
 * 
 * @param a (input) First matrix
 * @param b (input) Second matrix
 * @param output (output) Resulting matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_substract(MATRIX a, MATRIX b, MATRIX *output);
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
 * @brief Compute the matrix trace
 * 
 * @param a (input) Matrix to compute trace from
 * @param output (output) Value of the trace
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_trace(MATRIX a, double *output);
/**
 * @brief Compute a minor of a matrix for a given element
 * 
 * @param a (input) Matrix to get minor from
 * @param row (input) Row of the matrix element 
 * @param col (input) Column of the matrix element
 * @param output (output) The resulting minor
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_minor(MATRIX a, int row, int col, MATRIX *output);
/**
 * @brief Compute the determinant of a matrix
 * 
 * @param a (input) Matrix to calculate determinant from 
 * @param output (output) The determinant value
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_determinant(MATRIX a, double *output);
/**
 * @brief Compute the adjoint matrix
 * 
 * @param a (input) Matrix to compute adjoint from
 * @param output (output) Resulting Adjoint matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_adjoint(MATRIX a, MATRIX *output);
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
/**
 * @brief Solve a system of linear equations in the form Ax - b = 0 with the Jacobi method
 * 
 * @param A (input) Coeficients matrix
 * @param b (input) Offset vector
 * @param tolerance (input) Precission value to reach
 * @param guess (input/output) Solution guess. Serves as initial guess and is set with the final solution
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_linear_system_solve(MATRIX A, MATRIX b, double tolerance, MATRIX *guess);


#endif /* __matrix_h__ */