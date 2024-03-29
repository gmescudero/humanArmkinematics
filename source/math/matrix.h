/**
 * @file matrix.h
 * @author German Moreno Escudero
 * @brief Library to operate with dynamically allocaded matrixes
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
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

typedef struct MATRIX_SVD_STRUCT {
    MATRIX U;
    MATRIX V;
    MATRIX Sigma;
    bool set;
} MATRIX_SVD;

/**
 * @brief Allocate memory for a matrix
 * 
 * @param rows (input) Number of rows
 * @param cols (input) Number of columns
 * @return MATRIX with allocated memory 
 */
MATRIX matrix_allocate(unsigned rows, unsigned cols);
/**
 * @brief Allocate memory for a matrix of zeros
 * 
 * @param rows (input) Number of rows
 * @param cols (input) Number of columns
 * @return MATRIX with allocated memory 
 */
MATRIX matrix_zeros_allocate(unsigned rows, unsigned cols);
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
 * @brief Set a given square matrix with an identity
 * 
 * @param a (outpt) Identity matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_identity_set(MATRIX *a);
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
/**
 * @brief Compute the eigenvalues and eigenvectors of a real symmetric matrix
 * 
 * @param A (input) Matrix to get eigen values of
 * @param eigenvalues (output) Vector with all the eigenvalues in decreasing order
 * @param eigenvectors (output) Matrix with the eigenvectors asocianted to each eigenvalue as columns
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_eigen(MATRIX A, double eigenvalues[], MATRIX *eigenvectors);
/**
 * @brief Compute the Singular Value Decomposition of a matrix and allocate memory for it
 * 
 * @param A (input) Matrix to get SVD from
 * @return MATRIX_SVD: A structure with the singular values decomposition
 */
MATRIX_SVD matrix_svd_allocate_and_set(MATRIX A);
/**
 * @brief Free the allocated memory of a SVD
 * 
 * @param svd (input) SVD structure to free
 */
void matrix_svd_free(MATRIX_SVD svd);
/**
 * @brief Compute the Moore-Penrose pseudoinverse of a matrix using SVD
 * 
 * @param a (input) Matrix to invert
 * @param output (output) Inverted matrix
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_pseudoinverse_svd(MATRIX A, MATRIX *output);
/**
 * @brief Obtain the Householders reduction to upper triangular matrix
 * 
 * @param A (input) Matrix to make upper triangular
 * @param output (output) The resulting upper triangular matrix
 * @param transform (output) The transformation multiplied to the original A matrix from the left. Can be NULL.
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_upper_triangular(MATRIX A, MATRIX *output, MATRIX *transform);
/**
 * @brief Obtain the Householders reduction to upper bidiagonal matrix
 * 
 * @param A (input) Matrix to make bidiagonal
 * @param output (output) The resulting upper bidiagonal matrix
 * @param left_transform (output) The transformation multiplied to the original A matrix from the left. Can be NULL.
 * @param right_transform (output) The transformation multiplied to the original A matrix from the right. Can be NULL.
 * @return ERROR_CODE 
 */
ERROR_CODE matrix_upper_bidiagonal(MATRIX A, MATRIX *output, MATRIX *left_transform, MATRIX *right_transform);

#endif /* __matrix_h__ */