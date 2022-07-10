
#include "matrix.h"
#include "general.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


MATRIX matrix_allocate(unsigned rows, unsigned cols) {
    MATRIX result;
    result.rows = rows;
    result.cols = cols;
    result.data = (double **) malloc(rows*sizeof(double));
    for (int i = 0; i < rows; i++) {
        result.data[i] = (double *) malloc(cols*sizeof(double));
    }
    return result;
}

MATRIX matrix_from_buffer_allocate(unsigned rows, unsigned cols, double *buff[]) {
    MATRIX result;

    result = matrix_allocate(rows, cols);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c<cols; c++) {
            result.data[r][c] = buff[r][c];
        }    
    }
    return result;
}

void matrix_free(MATRIX a) {
    for (int i = 0; i < a.rows; i++) {
        free(a.data[i]);
    }
    free(a.data);
}

ERROR_CODE matrix_copy(MATRIX a, MATRIX *output) {
    // Check arguments
    if (NULL == output) return RET_ARG_ERROR;
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

ERROR_CODE matrix_add(MATRIX a, MATRIX b, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)   return RET_ARG_ERROR;
    if (a.rows != b.rows) return RET_ARG_ERROR;
    if (a.cols != b.cols) return RET_ARG_ERROR;

    result = matrix_allocate(a.cols, a.rows);

    for (int r = 0; r < a.rows; r++) {
        for (int c = 0; c < a.cols; c++) {
            result.data[r][c] = a.data[r][c] + b.data[r][c];
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}

ERROR_CODE matrix_multiply(MATRIX a, MATRIX b, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    // Check arguments
    if (NULL == output)   return RET_ARG_ERROR;
    if (a.rows != b.cols) return RET_ARG_ERROR;
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
    ERROR_CODE status;
    MATRIX result;
    double temp;
    int size = a.rows;
    // Check arguments
    if (NULL == output)   return RET_ARG_ERROR;
    if (a.rows != a.cols) return RET_ARG_ERROR;
    
    result = matrix_allocate(size, size);
    for(int i=0; i<size; i++)
        for(int j=0; j<size; j++)
            if(i==j)
                result.data[i][j]=1.0;
            else
                result.data[i][j]=0.0;

    for(int k=0; k<size; k++) {
        temp = a.data[k][k]; 
        for(int j=0; j<size; j++) {
            a.data[k][j] /= temp;
            result.data[k][j] /= temp;
        }
        for(int i=0; i<size; i++) {
            temp = a.data[i][k];
            for(int j=0; j<size;j++) { 
                if(i==k)
                    break;
                a.data[i][j]     -= a.data[k][j]*temp; 
                result.data[i][j]-= result.data[k][j]*temp;
            }
        }
    }

    status = matrix_copy(result, output);
    matrix_free(result);
    return status;
}


ERROR_CODE matrix_pseudoinverse(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    MATRIX at;
    MATRIX aat;

    // (Jt*J)^-1
    at      = matrix_allocate(a.cols, a.rows);
    aat     = matrix_allocate(at.rows, a.cols);
    result  = matrix_allocate(aat.rows, aat.cols);

    status = matrix_transpose(a, &at);
    if (RET_OK == status) {
        status = matrix_multiply(at, a, &aat);
    }
    if (RET_OK == status) {
        status = matrix_inverse(aat, &result);
    }
    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(result);
    matrix_free(aat);
    matrix_free(at);
    return status;
}