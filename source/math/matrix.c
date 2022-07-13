
#include "constants.h"
#include "matrix.h"
#include "general.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


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

    if (false == result.allocated) {
        err_str("Failed to allocate matrix");
    }
    return result;
}

MATRIX matrix_identity_allocate(unsigned size) {
    MATRIX result = matrix_allocate(size,size);
    for (int r = 0; r<size; r++) {
        for (int c = 0; c<size; c++) {
            result.data[r][c] = (r==c) ? 1.0 : 0.0;
        }
    }
    return result;
}


void matrix_free(MATRIX a) {
    for (int i = 0; i < a.rows; i++) {
        free(a.data[i]);
    }
    free(a.data);
    a.allocated = false;
}

ERROR_CODE matrix_copy(MATRIX a, MATRIX *output) {
    // Check arguments
    if (NULL == output) return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (output->rows != a.rows) return RET_ARG_ERROR;
    if (output->cols != a.cols) return RET_ARG_ERROR;

    for (int r = 0; r<a.rows; r++) {
        for (int c = 0; c<a.cols; c++) {
            output->data[r][c] = a.data[r][c];
        }
    }

    return RET_OK;
}

ERROR_CODE matrix_transpose(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output) return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
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
    if (NULL == output) return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
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
    if (NULL == output)   return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == b.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != b.rows) return RET_ARG_ERROR;
    if (a.cols != b.cols) return RET_ARG_ERROR;

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
    if (NULL == output)   return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == b.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != b.rows) return RET_ARG_ERROR;
    if (a.cols != b.cols) return RET_ARG_ERROR;

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
    if (NULL == output)   return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == b.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.cols != b.rows) return RET_ARG_ERROR;

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

ERROR_CODE matrix_inverse(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    double temp;
    int size = a.rows;
    // Check arguments
    if (NULL == output)   return RET_ARG_ERROR;
    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != a.cols) return RET_ARG_ERROR;
    
    result = matrix_identity_allocate(size);

    for(int k=0; RET_OK == status && k<size; k++) {
        temp = a.data[k][k]; 
        if (EPSI > fabs(temp)) {
            status = RET_ERROR;
            err_str("Failed to invert, matrix is singular");
        }
        else {
            for(int j=0; j<size; j++) {
                a.data[k][j] /= temp;
                result.data[k][j] /= temp;
            }
            for(int i=0; i<size; i++) {
                temp = a.data[i][k];
                for(int j=0; j<size;j++) { 
                    if(i!=k) {
                        a.data[i][j]     -= a.data[k][j]*temp; 
                        result.data[i][j]-= result.data[k][j]*temp;
                    }
                }
            }
        }
    }
    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(result);
    return status;
}


ERROR_CODE matrix_pseudoinverse(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    MATRIX at;
    MATRIX aat;
    MATRIX aatinv;

    if (false == a.allocated) return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;

    // (Jt*J)^-1
    at      = matrix_allocate(a.cols, a.rows);
    aat     = matrix_allocate(a.rows, at.cols);
    aatinv  = matrix_allocate(aat.rows, aat.cols);
    result  = matrix_allocate(at.rows, aatinv.cols);

    status = matrix_transpose(a, &at);
    if (RET_OK == status) {
        status = matrix_multiply(a, at, &aat);
    }
    if (RET_OK == status) {
        status = matrix_inverse(aat, &aatinv);
    }
    if (RET_OK == status) {
        status = matrix_multiply(at, aatinv, &result);
    }
    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(result);
    matrix_free(aatinv);
    matrix_free(aat);
    matrix_free(at);
    return status;
}

void matrix_print(MATRIX a, const char *name) {
    char string[250] = "";
    char part_string[25] = "";  

    if (false == a.allocated) wrn_str("Failed to print matrix. Not allocated!");

    for (int r = 0; r < a.rows; r ++) {
        strcat(string, "\n\t\t");
        for (int c = 0; c < a.cols; c ++) {
            sprintf(part_string,"%f\t",a.data[r][c]);
            strcat(string, part_string);
        }
    }
    log_str("%dx%d Matrix (%s): %s",a.rows, a.cols, (NULL!=name)?name:"-", string);
}