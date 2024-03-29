/**
 * @file matrix.c
 * @author German Moreno Escudero
 * @brief Library to operate with dynamically allocaded matrixes
 * @version 0.1
 * @date 2023-01-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "constants.h"
#include "matrix.h"
#include "general.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define INVERSE_ALGORITHM (1) // 0 for standard algorithm and 1 for gauss jordan approach
// Gauss Jordan algorithm is way faster than the standard method

#if (0 == INVERSE_ALGORITHM)
static ERROR_CODE smatrix_inverse_standard(MATRIX a, MATRIX *output);
#else
static ERROR_CODE smatrix_inverse_gauss_jordan(MATRIX a, MATRIX *output);
#endif


MATRIX matrix_allocate(unsigned rows, unsigned cols) {
    MATRIX result;
    result.allocated = true;
    result.rows = rows;
    result.cols = cols;

    result.data = NULL;
    result.data = (double **) malloc(rows*sizeof(double));
    if (NULL == result.data) result.allocated = false;

    for (int i = 0; i < rows; i++) {
        result.data[i] = NULL;
        result.data[i] = (double *) malloc(cols*sizeof(double));
        if (NULL == result.data[i]) result.allocated = false;
    }

    if (false == result.allocated || 0 >= rows || 0>= cols) {
        err_str("Failed to allocate matrix %ux%u",rows,cols);
    }
    return result;
}

MATRIX matrix_zeros_allocate(unsigned rows, unsigned cols) {
    MATRIX result = matrix_allocate(rows,cols);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            result.data[r][c] = 0.0;
        }
    }
    return result;
}

MATRIX matrix_identity_allocate(unsigned size) {
    MATRIX result = matrix_allocate(size,size);
    if (RET_OK != matrix_identity_set(&result)) {
        matrix_free(result);
        err_str("Failed to set identity matrix");
    }
    return result;
}

MATRIX matrix_from_matrix_allocate(MATRIX a) {
    MATRIX result = matrix_allocate(a.rows,a.cols);
    if (RET_OK != matrix_copy(a,&result)) {
        err_str("Failed to initialize matrix");
    }
    return result;
}

void matrix_free(MATRIX a) {
    for (int i = 0; i < a.rows; i++) {
        free(a.data[i]);
    }
    free(a.data);
    a.allocated = false;
    a.rows = 0;
    a.cols = 0;
}

ERROR_CODE matrix_copy(MATRIX a, MATRIX *output) {
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (output->rows != a.rows)     return RET_ARG_ERROR;
    if (output->cols != a.cols)     return RET_ARG_ERROR;

    for (int r = 0; r<a.rows; r++) {
        for (int c = 0; c<a.cols; c++) {
            output->data[r][c] = a.data[r][c];
        }
    }

    return RET_OK;
}

ERROR_CODE matrix_identity_set(MATRIX *a) {
    int size = a->rows;
    // Check arguments
    if (false == a->allocated) return RET_ARG_ERROR;
    if (a->rows != a-> cols)   return RET_ARG_ERROR;

    for (int r = 0; r<size; r++) {
        for (int c = 0; c<size; c++) {
            a->data[r][c] = (r==c) ? 1.0 : 0.0;
        }
    }
    return RET_OK;
}

ERROR_CODE matrix_transpose(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;

    result = matrix_allocate(a.cols, a.rows);
    for (int r = 0; r < a.rows; r++) {
        for (int c = 0; c < a.cols; c++) {
            result.data[c][r] = a.data[r][c];
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}


ERROR_CODE matrix_scale(MATRIX a, double scale, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;

    result = matrix_allocate(a.rows, a.cols);
    for (int r = 0; r < a.rows; r++) {
        for (int c = 0; c < a.cols; c++) {
            result.data[r][c] = a.data[r][c] * scale;
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_add(MATRIX a, MATRIX b, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == b.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != b.rows)           return RET_ARG_ERROR;
    if (a.cols != b.cols)           return RET_ARG_ERROR;

    result = matrix_allocate(a.rows, a.cols);
    for (int r = 0; r < a.rows; r++) {
        for (int c = 0; c < a.cols; c++) {
            result.data[r][c] = a.data[r][c] + b.data[r][c];
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_substract(MATRIX a, MATRIX b, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == b.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != b.rows)           return RET_ARG_ERROR;
    if (a.cols != b.cols)           return RET_ARG_ERROR;

    result = matrix_allocate(a.rows, a.cols);
    for (int r = 0; r < a.rows; r++) {
        for (int c = 0; c < a.cols; c++) {
            result.data[r][c] = a.data[r][c] - b.data[r][c];
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_multiply(MATRIX a, MATRIX b, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == b.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.cols != b.rows)           return RET_ARG_ERROR;

    result = matrix_allocate(a.rows, b.cols);
    for (int i = 0; i < a.rows; ++i) {
        for (int j = 0; j < b.cols; ++j) {
            result.data[i][j] = 0.0;
            for (int k = 0; k < a.cols; ++k) {
                result.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_trace(MATRIX a, double *output) {
    ERROR_CODE status = RET_OK;
    int size = a.rows;
    // Check arguments
    if (NULL == output)         return RET_ARG_ERROR;
    if (false == a.allocated)   return RET_ARG_ERROR;
    if (a.rows != a.cols)       return RET_ARG_ERROR;

    *output = 0.0;
    for (int i = 0; i < size; i++) {
        *output += a.data[i][i];
    }

    return status;
}

ERROR_CODE matrix_minor(MATRIX a, int row, int col, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != a.cols)           return RET_ARG_ERROR;
    if (0 > row || a.rows <= row)   return RET_ARG_ERROR;
    if (0 > col || a.cols <= col)   return RET_ARG_ERROR;

    int rp, cp;

    result = matrix_allocate(a.rows-1, a.cols-1);
    for(int r = 0; r < result.rows; r++) {
        for (int c = 0; c < result.cols; c++) {
            if (r >= row) rp = r+1; else rp = r;
            if (c >= col) cp = c+1; else cp = c;
            result.data[r][c] = a.data[rp][cp];
        }   
    }
    
    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_determinant(MATRIX a, double *output) {
    ERROR_CODE status = RET_OK;
    int size = a.rows;
    // Check arguments
    if (NULL == output)         return RET_ARG_ERROR;
    if (false == a.allocated)   return RET_ARG_ERROR;
    if (a.rows != a.cols)       return RET_ARG_ERROR;

    if      (1 == size) {
        *output = a.data[0][0];
    }
    else if (2 == size) {
        *output = a.data[0][0]*a.data[1][1] - a.data[0][1]*a.data[1][0];
    }
    else if (3 == size) {
        *output = a.data[0][0]*a.data[1][1]*a.data[2][2]
                + a.data[0][1]*a.data[1][2]*a.data[2][0]
                + a.data[0][2]*a.data[1][0]*a.data[2][1]
                - a.data[0][2]*a.data[1][1]*a.data[2][0]
                - a.data[0][0]*a.data[1][2]*a.data[2][1]
                - a.data[0][1]*a.data[1][0]*a.data[2][2];
    }
    else if (4 == size) {
        *output = a.data[0][0]*a.data[1][1]*a.data[2][2]*a.data[3][3] 
                - a.data[0][0]*a.data[1][1]*a.data[2][3]*a.data[3][2] 
                - a.data[0][0]*a.data[1][2]*a.data[2][1]*a.data[3][3] 
                + a.data[0][0]*a.data[1][2]*a.data[2][3]*a.data[3][1] 
                + a.data[0][0]*a.data[1][3]*a.data[2][1]*a.data[3][2] 
                - a.data[0][0]*a.data[1][3]*a.data[2][2]*a.data[3][1] 
                - a.data[0][1]*a.data[1][0]*a.data[2][2]*a.data[3][3] 
                + a.data[0][1]*a.data[1][0]*a.data[2][3]*a.data[3][2] 
                + a.data[0][1]*a.data[1][2]*a.data[2][0]*a.data[3][3] 
                - a.data[0][1]*a.data[1][2]*a.data[2][3]*a.data[3][0] 
                - a.data[0][1]*a.data[1][3]*a.data[2][0]*a.data[3][2] 
                + a.data[0][1]*a.data[1][3]*a.data[2][2]*a.data[3][0] 
                + a.data[0][2]*a.data[1][0]*a.data[2][1]*a.data[3][3] 
                - a.data[0][2]*a.data[1][0]*a.data[2][3]*a.data[3][1] 
                - a.data[0][2]*a.data[1][1]*a.data[2][0]*a.data[3][3] 
                + a.data[0][2]*a.data[1][1]*a.data[2][3]*a.data[3][0] 
                + a.data[0][2]*a.data[1][3]*a.data[2][0]*a.data[3][1] 
                - a.data[0][2]*a.data[1][3]*a.data[2][1]*a.data[3][0] 
                - a.data[0][3]*a.data[1][0]*a.data[2][1]*a.data[3][2] 
                + a.data[0][3]*a.data[1][0]*a.data[2][2]*a.data[3][1] 
                + a.data[0][3]*a.data[1][1]*a.data[2][0]*a.data[3][2] 
                - a.data[0][3]*a.data[1][1]*a.data[2][2]*a.data[3][0] 
                - a.data[0][3]*a.data[1][2]*a.data[2][0]*a.data[3][1] 
                + a.data[0][3]*a.data[1][2]*a.data[2][1]*a.data[3][0];
    }
    else {
        double partialDet = 0;
        MATRIX minor = matrix_allocate(size-1,size-1);
        *output = 0.0;
        for (int i = 0; RET_OK == status && i < size; i++) {
            if (EPSI < fabs(a.data[0][i])) {
                status = matrix_minor(a, 0, i, &minor);
                if (RET_OK == status) {
                    status = matrix_determinant(minor, &partialDet);
                }
                if (RET_OK == status) {
                    if (0 == i%2) {
                        *output += a.data[0][i]*partialDet;
                    }
                    else {
                        *output -= a.data[0][i]*partialDet;
                    }
                }
            }
        }
        matrix_free(minor);
    }

    return status;
}

ERROR_CODE matrix_adjoint(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    MATRIX minor;
    int size = a.rows;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != a.cols)           return RET_ARG_ERROR;

    if (1 == size) {
        // Determinant of a 0x0 matrix is 1
        output->data[0][0] = 1;
    }
    else {
        result = matrix_allocate(size, size);
        minor  = matrix_allocate(size-1,size-1);
        for (int r = 0; RET_OK == status && r < size; r++) {
            for (int c = 0; RET_OK == status && c < size; c++) {
                status = matrix_minor(a,r,c,&minor);
                if (RET_OK == status) {
                    status = matrix_determinant(minor, &result.data[r][c]);
                    if (RET_OK == status && r%2 != c%2) {
                        result.data[r][c] *= -1.0;
                    }
                }
            }
        }
        if (RET_OK == status) {
            status = matrix_transpose(result, output);
        }
        matrix_free(minor);
        matrix_free(result);
    }
    return status;
}

#if (0 == INVERSE_ALGORITHM)
static ERROR_CODE smatrix_inverse_standard(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    double determinant = 0.0;
    int size = a.rows;

    result = matrix_allocate(size,size);

    if (RET_OK == status) {
        status = matrix_adjoint(a, &result);
    }
    for (int i = 0; RET_OK == status && i < size; i++) {
        determinant += a.data[0][i]*result.data[i][0];
    }
    if (RET_OK == status && EPSI > fabs(determinant)) {
        status = RET_ERROR;
        err_str("Failed to invert, matrix is singular");
        matrix_print(a, "Singular matrix");
    }
    if (RET_OK == status) {
        status = matrix_scale(result, 1.0/determinant, &result);
    }

    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(result);
    return status;
}
#endif

#if (1 == INVERSE_ALGORITHM)
static ERROR_CODE smatrix_inverse_gauss_jordan(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX aux;
    MATRIX result;
    double temp;
    int size = a.rows;
    
    result = matrix_identity_allocate(size);
    aux = matrix_from_matrix_allocate(a);

    for(int k=0; RET_OK == status && k<size; k++) {
        temp = aux.data[k][k]; 
        if (EPSI > fabs(temp)) {
            status = RET_ERROR;
            err_str("Failed to invert, matrix is singular");
            matrix_print(a, "Singular matrix");
        }
        else {
            for(int j=0; j<size; j++) {
                aux.data[k][j] /= temp;
                result.data[k][j] /= temp;
            }
            for(int i=0; i<size; i++) {
                temp = aux.data[i][k];
                for(int j=0; j<size;j++) { 
                    if(i!=k) {
                        aux.data[i][j]    -= aux.data[k][j]*temp; 
                        result.data[i][j] -= result.data[k][j]*temp;
                    }
                }
            }
        }
    }
    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(aux);
    matrix_free(result);
    return status;
}
#endif

ERROR_CODE matrix_inverse(MATRIX a, MATRIX *output) {
    // Check arguments
    if (NULL == output)                 return RET_ARG_ERROR;
    if (false == a.allocated)           return RET_ARG_ERROR;
    if (false == output->allocated)     return RET_ARG_ERROR;
    if (a.rows != a.cols)               return RET_ARG_ERROR;
    if (a.rows != output->rows)         return RET_ARG_ERROR;
    if (output->rows != output->cols)   return RET_ARG_ERROR;

#if (0 == INVERSE_ALGORITHM)
    return smatrix_inverse_standard(a, output);
#else
    return smatrix_inverse_gauss_jordan(a, output);
#endif
}

void matrix_print(MATRIX a, const char *name) {
    char string[2048] = "";
    char part_string[32] = "";  

    if (false == a.allocated) {
        wrn_str("Failed to print matrix. Not allocated!");
        return;
    }

    for (int r = 0; r < MIN(a.rows,10); r ++) {
        strcat(string, "\n\t\t");
        for (int c = 0; c < MIN(a.cols,10); c ++) {
            sprintf(part_string,"%f\t",a.data[r][c]);
            strcat(string, part_string);
        }
        if (10 < a.cols) {
            // Print a maximum of 10 columns
            sprintf(part_string,"... +%d", a.cols-10);
            strcat(string, part_string);
        }
    }
    if (10 < a.rows) {
        // Print a maximum of 10 rows
        sprintf(part_string,"\n\t\t... +%d\n",a.rows-10);
        strcat(string, part_string);
    }
    log_str("%dx%d Matrix (%s): %s",a.rows, a.cols, (NULL!=name)?name:"-", string);
}

ERROR_CODE matrix_linear_system_solve(MATRIX A, MATRIX b, double tolerance, MATRIX *guess) {
    ERROR_CODE status = RET_OK;
    int size = A.cols;
    // Check arguments
    if (false == A.allocated)      return RET_ARG_ERROR;
    if (false == b.allocated)      return RET_ARG_ERROR;
    if (false == guess->allocated) return RET_ARG_ERROR;
    if (A.rows != A.cols)          return RET_ARG_ERROR;
    if (b.rows != size)            return RET_ARG_ERROR;
    if (b.cols != 1)               return RET_ARG_ERROR;
    if (guess->rows != size)       return RET_ARG_ERROR;
    if (guess->cols != 1)          return RET_ARG_ERROR;
    if (0 >= tolerance)            return RET_ARG_ERROR;

    double convergence = 1e300;
    double sigma;
    MATRIX guess_new = matrix_from_matrix_allocate(*guess);
    
    // Solve A*guess = b
    for (int iteration = 0; RET_OK == status && convergence > tolerance &&  iteration < 1000; iteration++) {
        for (int i = 0; RET_OK == status && i < size; i++) {
            sigma = 0.0;
            for (int j = 0; j < size; j++) {
                if (i!=j) {
                    sigma += A.data[i][j] * guess->data[j][0];
                }
            }
            if (fabs(A.data[i][i]) > EPSI) {
                guess_new.data[i][0] = (b.data[i][0] - sigma) / A.data[i][i];
            }
            else {
                err_str("Element %d,%d of the matrix is zero",i,i);
                status = RET_ERROR;
            }
        }
        if (RET_OK == status) {
            // Evaluate convergence 
            convergence = 0.0;
            for (int element = 0; RET_OK == status && element < size; element++) {
                convergence += fabs(guess_new.data[element][0] - guess->data[element][0]);
            }
            // Set new guess
            status = matrix_copy(guess_new, guess);

            dbg_str("%s -> Iteration %d with convergence value %f",__FUNCTION__,iteration,convergence);
        }
    }

    // Check result
    if (convergence > tolerance) {
        err_str("Did not find solution to system Ax = b");
        status = RET_ERROR;
    }

    matrix_free(guess_new);

    return status;
}

/**
 * @brief Finds the index of the largest off-diagonal element
 * 
 * @param A (input) Matrix in which to look
 * @param row (output) Row of the maximum index
 * @param col (output) Column of the maixum index
 */
void smatrix_maxind(MATRIX A, int *row, int *col) {
    *row = 0;
    *col = 1;
    for (int r = 0; r < A.rows-1; r++) { 
        for (int c = r+1; c < A.cols; c++) {
            if (fabs(A.data[r][c]) > fabs(A.data[*row][*col])){
                *row = r;
                *col = c;
            }
        }
    }
}

/**
 * @brief Rotate a matrix around a pivot element with a given angle with the form A := P' A P
 * 
 * @param row (input) Row of the pivot element
 * @param col (input) Column of the pivot element
 * @param angle (inptu) Angle to rotate
 * @param A (input/output) Matrix to rotate
 * @return ERROR_CODE 
 */
ERROR_CODE smatrix_pivot(int row, int col, double angle, MATRIX *A) {
    ERROR_CODE status = RET_OK;
    MATRIX P = matrix_identity_allocate(A->rows);

    P.data[row][row] = cos(angle);  P.data[row][col] =-sin(angle);
    P.data[col][row] = sin(angle);  P.data[col][col] = cos(angle);  

    // A := Pt A P
    status = matrix_multiply(*A,P, A);
    if (RET_OK == status) status = matrix_transpose(P,&P);
    if (RET_OK == status) status = matrix_multiply(P,*A, A);

    matrix_free(P);
    return status;
}

/**
 * @brief Rotate a matrix around a pivot element with a given angle with the form A := A P
 * 
 * @param row (input) Row of the pivot element
 * @param col (input) Column of the pivot element
 * @param angle (inptu) Angle to rotate
 * @param A (input/output) Matrix to rotate
 * @return ERROR_CODE 
 */
ERROR_CODE smatrix_rotate(int row, int col, double angle, MATRIX *A) {
    ERROR_CODE status = RET_OK;
    MATRIX P = matrix_identity_allocate(A->rows);

    P.data[row][row] = cos(angle);  P.data[row][col] =-sin(angle);
    P.data[col][row] = sin(angle);  P.data[col][col] = cos(angle);  

    // A := A P
    status = matrix_multiply(*A,P, A);

    matrix_free(P);
    return status;
}

/**
 * @brief Check if a given matrix is diagonal
 * 
 * @param A (input) Matrix to check
 * @param check_no_zero_in_diagonal (input) If true, checks that the diagonal is non-zero
 * @return true if matrix is diagonal
 * @return false if matrix is not diagonal
 */
bool smatrix_diagonal_check(MATRIX A, bool check_no_zero_in_diagonal) {
    double acum = 0.0;
    if (A.rows != A.cols) return false;
    for (int r = 0; r < A.rows; r++) {
        if (true == check_no_zero_in_diagonal && fabs(A.data[r][r]) < EPSI) {
            return false;
        }
        for (int c = r+1; c < A.cols; c++) {
            acum += A.data[r][c]*A.data[r][c];
            acum += A.data[c][r]*A.data[c][r];
        }
    }
    if (sqrt(acum) > EPSI) {
        return false;
    }
    return true;
}

/**
 * @brief Check if a given matrix is symmetric
 * 
 * @param A (input) Matrix to check
 * @return true if matrix is symmetric
 * @return false if matrix is not symmetric
 */
bool smatrix_symmetryc_check(MATRIX A) {
    if (A.rows != A.cols) return false;
    for (int r = 0; r < A.rows; r++) {
        for (int c = r+1; c < A.cols; c++) {
            if (A.data[r][c] != A.data[c][r]) {
                return false;
            }
        }
    }
    return true;
}

ERROR_CODE matrix_eigen(MATRIX A, double eigenvalues[], MATRIX *eigenvectors) {
    ERROR_CODE status = RET_OK;
    int size = A.rows;

    if (false == A.allocated)             return RET_ARG_ERROR;
    if (false == eigenvectors->allocated) return RET_ARG_ERROR;
    if (NULL == eigenvalues)              return RET_ARG_ERROR;

    if (!smatrix_symmetryc_check(A)) return RET_ARG_ERROR;
    if (eigenvectors->rows != size)  return RET_ARG_ERROR;
    if (eigenvectors->cols != size)  return RET_ARG_ERROR;

    // Initialize
    status = matrix_identity_set(eigenvectors);

    // Iterate rotations
    int iterations = 0;
    while (iterations++ < 100 && RET_OK == status && !smatrix_diagonal_check(A, false)) {        
        // Find pivot index
        int pivot_row = 0;
        int pivot_col = 1;
        smatrix_maxind(A,&pivot_row,&pivot_col);
        double pivot_value = A.data[pivot_row][pivot_col];
        // Compute sine and cosine of rotation
        double angle = 0.5*atan2(2.0*pivot_value, A.data[pivot_row][pivot_row] - A.data[pivot_col][pivot_col]);
        // Rotate A around pivot
        status = smatrix_pivot(pivot_row, pivot_col, angle, &A);
        // Rotate Eigen vectors
        if (RET_OK == status) status = smatrix_rotate(pivot_row, pivot_col, angle, eigenvectors);

        // dbg_str("%s -> Eigen IT [%d]: k:<%d>, l:<%d>, p:<%f>, angle:<%f>",
        //     __FUNCTION__,iterations,pivot_row,pivot_col,pivot_value,angle);
    }

    // Get eigenvalues
    for (int k = 0; RET_OK == status && k < size; k++) {
        eigenvalues[k] = A.data[k][k];
    }

    // Order in decreasing value
    for (int k = 0; k < size-1; k++) {
        int maxind = k;
        for (int l = k+1; l < size; l++) {
            if (eigenvalues[l] > eigenvalues[k]) {
                maxind = l;
            }
        }
        if (k != maxind) {
            double aux = eigenvalues[k];
            eigenvalues[k] = eigenvalues[maxind];
            eigenvalues[maxind] = aux;
            for(int i = 0; i < size; i++) {
                aux = eigenvectors->data[i][k];
                eigenvectors->data[i][k] = eigenvectors->data[i][maxind];
                eigenvectors->data[i][maxind] = aux;
            }
        }
    }

    if (!smatrix_diagonal_check(A, false)) {
        err_str("Failed to compute eigen values of matrix A");
        matrix_print(A,"A");
        return RET_ERROR;
    }

    dbg_str("%s -> Found eigen values of matrix in %d iterations. Largest eigen value: %f",__FUNCTION__,iterations, eigenvalues[0]);

    return status;
}


MATRIX_SVD matrix_svd_allocate_and_set_old(MATRIX A) {
    ERROR_CODE status = RET_OK;
    MATRIX_SVD svd;
    int n = A.cols;
    int m = A.rows;
    double eigenvalues[MAX(n,m)];

    // Initialize SVD
    for (int i = 0; i < MAX(n,m); i++) {
        eigenvalues[i] = 0.0;
    }
    svd.U = matrix_allocate(m,m);
    svd.V = matrix_allocate(n,n);
    svd.Sigma = matrix_zeros_allocate(m,n);
    svd.set = false;

    // Allocate auxiliary matrixes
    MATRIX At = matrix_allocate(n,m);
    MATRIX A_At = matrix_allocate(m,m);
    MATRIX At_A = matrix_allocate(n,n);

    // Set A transpose
    status = matrix_transpose(A,&At);

    // Get A * A transpose and A transpose * A
    if (RET_OK == status) status = matrix_multiply(A,At, &A_At);
    if (RET_OK == status) status = matrix_multiply(At,A, &At_A);

    // Compute squared eigenvalues and eigenvectors matrixes 
    if (RET_OK == status) status = matrix_eigen(A_At, eigenvalues, &svd.U);
    if (RET_OK == status) status = matrix_eigen(At_A, eigenvalues, &svd.V);

    // Set Sigma diagonal matrix
    for (int i = 0; RET_OK == status && i < MIN(n,m); i++) {
        svd.Sigma.data[i][i] = sqrt(eigenvalues[i]);
    }

    if (RET_OK == status) {
        svd.set = true;
    }
    else {
        err_str("Failed to compute SVD");
        matrix_svd_free(svd);
    }

    // Free auxiliary matrixes
    matrix_free(At);
    matrix_free(A_At);
    matrix_free(At_A);

    return svd;
}

void matrix_svd_free(MATRIX_SVD svd) {
    matrix_free(svd.Sigma);
    matrix_free(svd.U);
    matrix_free(svd.V);
}

ERROR_CODE matrix_pseudoinverse_wip(MATRIX A, MATRIX *output) {
    ERROR_CODE status = RET_OK;

    if (false == A.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (A.rows != output->cols)     return RET_ARG_ERROR;
    if (A.cols != output->rows)     return RET_ARG_ERROR;

    MATRIX_SVD svd = matrix_svd_allocate_and_set(A);
    if (false == svd.set) {
        return RET_ERROR;
    }

    // Compute the pseudo-inverse of Sigma
    MATRIX S_inv  = matrix_zeros_allocate(svd.Sigma.cols, svd.Sigma.rows);
    for (int i = 0; i < MIN(S_inv.rows,S_inv.cols); i++) {
        if (fabs(svd.Sigma.data[i][i]) > EPSI) {
            S_inv.data[i][i] = 1.0/svd.Sigma.data[i][i];
        }
    }

    // pinv(A) = V*pinv(S)*Ut
    MATRIX Ut        = matrix_allocate(svd.U.cols, svd.U.rows);
    MATRIX Sinv_Ut   = matrix_allocate(S_inv.rows, Ut.cols);
    MATRIX V_Sinv_Ut = matrix_allocate(svd.V.rows, Sinv_Ut.cols);

    status = matrix_transpose(svd.U,&Ut);
    if (RET_OK == status) status = matrix_multiply(S_inv,Ut, &Sinv_Ut);
    if (RET_OK == status) status = matrix_multiply(svd.V,Sinv_Ut, &V_Sinv_Ut);

    if (RET_OK == status) status = matrix_copy(V_Sinv_Ut, output);

    matrix_free(S_inv);
    matrix_free(Ut);
    matrix_free(Sinv_Ut);
    matrix_free(V_Sinv_Ut);
    matrix_svd_free(svd);

    return status;
}

/**
 * @brief Compute a Householder transformation
 * 
 * @param A (input) Matrix to transform
 * @param h_row (input) Row to start from
 * @param h_col (input) Column to start from
 * @param row_vector (input) Treat a column or a row
 * @param Householder (output) Resulting householder
 */
static void smatrix_householder(MATRIX A, int h_row, int h_col, bool row_vector, MATRIX *Householder) {
    int size;
    int start_index;
    double v[MAX(A.rows,A.cols)];
    MATRIX result;

    // Set the v vector with the A column/row treated
    if (false == row_vector) {
        size        = A.rows;
        start_index = h_row;
        result = matrix_identity_allocate(size);
        for (int i = start_index; i < size; i++) {
            v[i] = A.data[i][h_col];
        }
    }
    else {
        size        = A.cols;
        start_index = h_col;
        result = matrix_identity_allocate(size);
        for (int i = start_index; i < size; i++) {
            v[i] = A.data[h_row][i];
        }
    }

    // Compute the non zero value alpha of the working column
    double alpha = 0;
    for (int i = start_index; i < size; i++) {
        alpha +=  v[i] * v[i];
    }
    alpha = sqrt(alpha);
    // Compute the v vector as v = x - [alpha, 0, .. 0]^T and then v = v/|v|
    double norm = 0;
    v[start_index] -= alpha;
    for (int i = start_index; i < size; i++) {
        norm += v[i]*v[i];
    }
    if (EPSI < norm) {
        norm = sqrt(norm);
        for (int i = start_index; i < size; i++) {
            v[i] *= 1/norm; 
        }
    }
    // Build the Householder matrix
    for (int i = start_index; i < size; i++) {
        for (int j = start_index; j < size; j++) {
            result.data[i][j] -= 2*v[i]*v[j]; 
        }
    }
    
    matrix_copy(result, Householder);
    matrix_free(result);
}

/**
 * @brief Compute a Givens rotation matrix
 * 
 * @param A (input) Matrix to transform
 * @param row (input) Row of the rotation element
 * @param col (input) Column of the rotation element
 * @param Givens (output) Resulting Givens rotation matrix
 */
static void smatrix_givens_rotation(MATRIX A, int row, int col, MATRIX *Givens) {

    double a = A.data[col][col];
    double b = A.data[row][col];
    double norm = sqrt(a*a + b*b);
    double cosine; 
    double sine;

    matrix_identity_set(Givens);
    
    if ((EPSI > norm) || (EPSI > fabs(b))) {
        // Givens as identity
        return;
    }
    if (EPSI > fabs(a)) {
        cosine = 0.0;
        sine   = -copysign(1.0,b);
    }
    else {
        cosine =  a/norm;
        sine   = -b/norm;
    }

    Givens->data[row][row] = cosine;
    Givens->data[col][col] = cosine;
    Givens->data[row][col] = sine;
    Givens->data[col][row] = sine;

    if (row < col) {
        Givens->data[row][col] *= -1.0;
    }
    else {
        Givens->data[col][row] *= -1.0;
    }
}

ERROR_CODE matrix_upper_triangular(MATRIX A, MATRIX *output, MATRIX *transform) {
    ERROR_CODE status = RET_OK;

    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == A.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (A.rows != output->rows)     return RET_ARG_ERROR;
    if (A.cols != output->cols)     return RET_ARG_ERROR;

    int small_size = MIN(A.rows, A.cols);
    MATRIX result = matrix_from_matrix_allocate(A);
    bool transform_acum = false;
    if (NULL != transform && transform->allocated && transform->rows == A.rows && transform->cols == A.rows) {
        transform_acum = true;
        status = matrix_identity_set(transform);
    }

#if 0 // Householder
    MATRIX Householder = matrix_identity_allocate(A.rows);

    for (int col = 0; RET_OK == status && col < small_size && col < A.rows-1; col++) {
        // Compute the Householder transform
        smatrix_householder(result,col,col,false,&Householder);
        // Comute the new A 
        status = matrix_multiply(Householder, result, &result);
        // matrix_print(Householder,"Householder");
        // matrix_print(result,"HA");
        if (RET_OK == status && true == transform_acum) {
            status = matrix_multiply(Householder, *transform, transform);
        }
    }

    if (RET_OK == status) status = matrix_copy(result, output);
    matrix_free(Householder);
#else // Givens
    MATRIX Givens = matrix_identity_allocate(A.rows);

    for (int col = 0; RET_OK == status && col < small_size && col < A.rows-1; col++) {
        for (int row = col+1; row < A.rows; row++) {
            // Check if element is already 0
            if (EPSI > fabs(result.data[row][col])) continue;
            // Compute Givens transform
            smatrix_givens_rotation(result,row,col, &Givens);
            // Comute the new A 
            status = matrix_multiply(Givens, result, &result);
            // matrix_print(Givens,"Givens");
            // matrix_print(result,"GA");
            if (RET_OK == status && true == transform_acum) {
                status = matrix_multiply(Givens, *transform, transform);
            }
        }
    }

    if (RET_OK == status) status = matrix_copy(result, output);
    matrix_free(Givens);
#endif 
    matrix_free(result);

    return status;
}

ERROR_CODE matrix_upper_bidiagonal(MATRIX A, MATRIX *output, MATRIX *left_transform, MATRIX *right_transform) {
    ERROR_CODE status = RET_OK;

    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == A.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (A.rows != output->rows)     return RET_ARG_ERROR;
    if (A.cols != output->cols)     return RET_ARG_ERROR;

    int small_size = MIN(A.rows, A.cols);
    MATRIX result = matrix_from_matrix_allocate(A);
    MATRIX Householder_col = matrix_identity_allocate(A.rows);
    MATRIX Householder_row = matrix_identity_allocate(A.cols);

    bool left_acum  = false;
    bool right_acum = false;
    if (NULL != left_transform && left_transform->allocated && left_transform->rows == A.rows && left_transform->cols == A.rows) {
        left_acum = true;
        status = matrix_identity_set(left_transform);
    }
    if (NULL != right_transform && right_transform->allocated && right_transform->rows == A.cols && right_transform->cols == A.cols) {
        right_acum = true;
        status = matrix_identity_set(right_transform);
    }

    for (int ind = 0; RET_OK == status && ind < small_size && ind < A.rows-1; ind++) {
        // Compute the Householder transform for column
        smatrix_householder(result,ind,ind,false,&Householder_col);
        // Comute the new A 
        if (RET_OK == status) status = matrix_multiply(Householder_col, result, &result);
        if (RET_OK == status && true == left_acum) {
            status = matrix_multiply(Householder_col, *left_transform, left_transform);
        }
        // matrix_print(Householder_col,"Householder_col");
        // matrix_print(result,"HA");

        // Compute the Householder transform for row
        smatrix_householder(result,ind,ind+1,true,&Householder_row);
        // Comute the new A 
        if (RET_OK == status) status = matrix_multiply(result, Householder_row, &result);
        if (RET_OK == status && true == right_acum) {
            status = matrix_multiply(*right_transform, Householder_row, right_transform);
        }
        // matrix_print(Householder_row,"Householder_row");
        // matrix_print(result,"HAH");
    }

    if (RET_OK == status) status = matrix_copy(result, output);
    matrix_free(result);
    matrix_free(Householder_col);
    matrix_free(Householder_row);

    return status;
}

MATRIX_SVD matrix_svd_allocate_and_set(MATRIX A) {
    ERROR_CODE status = RET_OK;
    MATRIX_SVD svd;

    // Initialize SVD
    svd.U = matrix_allocate(A.rows,A.rows);
    svd.V = matrix_allocate(A.cols,A.cols);
    svd.Sigma = matrix_zeros_allocate(A.rows,A.cols);
    svd.set = false;

    MATRIX A_transformed = matrix_from_matrix_allocate(A);
    MATRIX left_transform = matrix_from_matrix_allocate(svd.U);
    MATRIX right_transform = matrix_from_matrix_allocate(svd.V);
    status = matrix_upper_bidiagonal(A, &A_transformed, &left_transform, &right_transform);
    matrix_print(A_transformed,"Bidiagonal");

    MATRIX At  = matrix_allocate(A.cols,A.rows);
    MATRIX AtA = matrix_allocate(A.cols,A.cols);
    MATRIX Givens_right = matrix_allocate(A.cols,A.cols);
    MATRIX Givens_left = matrix_allocate(A.rows,A.rows);

    for (int ind = 0; RET_OK == status && ind < A.cols-1; ind++) {
        dbg_str("Iteration %d",ind);
        // Compute AtA
        status = matrix_transpose(A_transformed, &At);
        if (RET_OK == status) status = matrix_multiply(At,A_transformed, &AtA);
        // Compute Givens rotation for AtA
        if (RET_OK == status) smatrix_givens_rotation(AtA, ind+1, ind, &Givens_right);
        // Apply Givens transform to the right
        if (RET_OK == status) status = matrix_multiply(A_transformed,Givens_right, &A_transformed);
        matrix_print(AtA, "AtA");
        matrix_print(Givens_right,"Givens right");
        matrix_print(A_transformed,"A after Givens right");
        matrix_multiply(Givens_right,AtA,&AtA);
        matrix_print(AtA,"Givens to AtA");
        // Acumulate right transform
        if (RET_OK == status) status = matrix_multiply(right_transform,Givens_right, &right_transform);
        if (ind < A.rows-1) {
            // Compute Givens rotation for A
            if (RET_OK == status) smatrix_givens_rotation(A_transformed,ind+1,ind, &Givens_left);
            // Apply transposed Givens transform to the left
            // if (RET_OK == status) status = matrix_transpose(Givens_left,&Givens_left);
            if (RET_OK == status) status = matrix_multiply(Givens_left, A_transformed, &A_transformed);
            matrix_print(Givens_left,"Givens left");
            matrix_print(A_transformed,"A after Givens left");
            // Acumulate left transform
            if (RET_OK == status) status = matrix_multiply(Givens_left,left_transform, &left_transform);
        }
    }

    if (RET_OK == status) status = matrix_transpose(left_transform,&svd.U);
    if (RET_OK == status) status = matrix_copy(right_transform, &svd.V);
    if (RET_OK == status) status = matrix_copy(A_transformed, &svd.Sigma);

    matrix_free(Givens_left);
    matrix_free(Givens_right);
    matrix_free(AtA);
    matrix_free(A_transformed);
    matrix_free(left_transform);
    matrix_free(right_transform);

    if (RET_OK == status) {
        svd.set = true;
    }
    else {
        err_str("Failed to compute SVD");
        matrix_svd_free(svd);
    }

    return svd;
}