
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
 * @brief Finds the index of the largest off-diagonal element in a row
 * 
 * @param A (input) Matrix in which to look
 * @param row (input) Row to look
 * @return int: the index of the largest off-diagonal element
 */
int smatrix_maxind(MATRIX A, int row) {
    int maxind = row+1;

    for (int col = row+2; col < A.cols; col++) {
        if (A.data[row][col] > A.data[row][maxind]){
            maxind = col;
        }
    }

    return maxind;
}

/**
 * @brief Update the process of the eigen search procedure
 * 
 * @param increment (input) Value to increment the eigen value with
 * @param eigen (input/output) The current eigen value to be updated
 * @param changed (input/output) Indicates whether the eigen values has changed or not
 * @param state (input/output) Indicates how many eigen vectors are left to compute
 */
void smatrix_eigen_iteration_update(double increment, double *eigen, bool *changed, int *state) {
    double old_eig = *eigen; 
    *eigen += increment;
    if (true == *changed && fabs(old_eig - *eigen) < EPSI) {
        *changed = false; 
        state--;
    }
    else if (false == *changed && fabs(old_eig - *eigen) > EPSI) {
        *changed = true;
        state++;
    }
}

/**
 * @brief Rotate two elements of a matrix
 * 
 * @param row1 (input) Row of the first element to rotate
 * @param col1 (input) Column of the first element to rotate
 * @param row2 (input) Row of the second element to rotate
 * @param col2 (input) Column of the second element to rotate
 * @param sine (input) Sine of the angle Phi that will be rotated
 * @param cosine (input) Cosine of the angle Phi that will be rotated
 * @param A (input/output) Matrix whose index will be rotated
 */
void smatrix_two_elements_rotate(int row1, int col1, int row2, int col2, double sine, double cosine, MATRIX *A) {
    double a_r1c1 = cosine*A->data[row1][col1] - sine*A->data[row2][col2];
    double a_r2c2 = sine*A->data[row1][col1] + cosine*A->data[row2][col2];
    A->data[row1][col1] = a_r1c1;
    A->data[row2][col2] = a_r2c2;
}

ERROR_CODE matrix_eigen(MATRIX A, double eigenvalues[], MATRIX *eigenvectors) {
    ERROR_CODE status = RET_OK;
    int size = A.rows;

    if (false == A.allocated)             return RET_ARG_ERROR;
    if (false == eigenvectors->allocated) return RET_ARG_ERROR;
    if (NULL == eigenvalues)              return RET_ARG_ERROR;

    if (A.rows !=size)              return RET_ARG_ERROR;
    if (A.cols !=size)              return RET_ARG_ERROR;
    if (eigenvectors->rows != size) return RET_ARG_ERROR;
    if (eigenvectors->cols != size) return RET_ARG_ERROR;

    int largest_row_index[size];
    bool changed[size];
    int state = size;

    // Initialize
    status = matrix_identity_set(eigenvectors);
    for (int k = 0; RET_OK == status && k < size; k++) {
        largest_row_index[k] = smatrix_maxind(A,k);
        eigenvalues[k] = A.data[k][k];
        changed[k] = true;
    }

    // Iterate
    int iterations = 10000;
    while (iterations-- > 0 && RET_OK == status && state > 0) {
        int pivot_row = 0;
        int pivot_col = largest_row_index[pivot_row];
        // Find pivot index
        for (int k = 1; k < size-1; k++) {
            if (A.data[k][largest_row_index[k]] > A.data[pivot_row][pivot_col]) {
                pivot_row = k;
                pivot_col = largest_row_index[pivot_row];
            }
        }
        double pivot_value = A.data[pivot_row][pivot_col];
        // Compute sine and cosine of rotation
        double y = (eigenvalues[pivot_col]-eigenvalues[pivot_row])*0.5;
        double d = fabs(y) + sqrt(pivot_value*pivot_value + y*y);
        double r = sqrt(pivot_value*pivot_value + d*d);
        double cosine = d/r;
        double sine = pivot_value/r;
        double increment = pivot_value*pivot_value/d;
        if (y < 0) {
            sine = -sine;
            increment = -increment;
        }
        // Update algorithm status
        smatrix_eigen_iteration_update(-increment, &eigenvalues[pivot_row], &changed[pivot_row], &state);
        smatrix_eigen_iteration_update( increment, &eigenvalues[pivot_col], &changed[pivot_col], &state);
        // Rotate rows and columns around pivot
        A.data[pivot_row][pivot_col] = 0.0;
        for (int i = 0; i < pivot_row-1; i++) {
            smatrix_two_elements_rotate(i,pivot_row,i,pivot_col,sine,cosine,&A);
        }
        for (int i = pivot_row+1; i < pivot_col-1; i++) {
            smatrix_two_elements_rotate(pivot_row,i,i,pivot_col,sine,cosine,&A);
        }
        for (int i = pivot_col+1; i < size; i++) {
            smatrix_two_elements_rotate(pivot_row,i,pivot_col,i,sine,cosine,&A);
        }
        // Rotate eigen vectors
        for (int i = 0; i < size; i++) {
            smatrix_two_elements_rotate(i,pivot_row,i,pivot_col,sine,cosine,eigenvectors);
        }
        // Update largest indexes
        for (int k = 0; RET_OK == status && k < size; k++) {
            largest_row_index[k] = smatrix_maxind(A,k);
        }
    }

    if (0 >= iterations) {
        err_str("Failed to compute eigen values of matrix A with %d out of %d remaining",state,size);
        matrix_print(A,"A");
        return RET_ERROR;
    }

    dbg_str("%s -> Found eigen values of matrix in %d iterations. Largest eigen value: %f",__FUNCTION__,10000-iterations, eigenvalues[0]);

    return status;
}


typedef struct MATRIX_SVD_STRUCT {
    MATRIX U;
    MATRIX V;
    MATRIX Sigma;
    bool set;
} MATRIX_SVD;

// TODO: Work In Progress, still a lot to do
MATRIX_SVD matrix_svd_allocate_and_set(MATRIX A, double tolerance, int *iterations) {
    ERROR_CODE status = RET_OK;
    MATRIX_SVD svd;
    double error = tolerance + 10.0;
    int n = A.cols;
    int m = A.rows;

    // Initialize SVD
    svd.U = matrix_allocate(n,n);
    svd.V = matrix_identity_allocate(m);
    svd.Sigma = matrix_allocate(m,n);
    svd.set = false;

    // status = matrix_copy()

    // Set the U matrix as 
    for (int it = 0; RET_OK == status && error > tolerance && it < *iterations; it++) {
        for (int i = 0; i < n-1; i++) {
            error = 0.0;
            for (int j = i+1; j < n; j++) {
                double norm_i, norm_j;
                
            }
        }
    }

    return svd;
}