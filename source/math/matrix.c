
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
    return status;
}

ERROR_CODE matrix_inverse_standard(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX result;
    double determinant;

    int size = a.rows;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != a.cols)           return RET_ARG_ERROR;

    result = matrix_identity_allocate(size);

    status = matrix_determinant(a, &determinant);
    if (RET_OK == status && EPSI > fabs(determinant)) {
        status = RET_ERROR;
        err_str("Failed to invert, matrix is singular");
        matrix_print(a, "Singular matrix");
    }
    if (RET_OK == status) {
        status = matrix_adjoint(a, &result);
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

ERROR_CODE matrix_inverse_gauss_jordan(MATRIX a, MATRIX *output) {
    ERROR_CODE status = RET_OK;
    MATRIX aux;
    MATRIX result;
    double temp;
    int size = a.rows;
    // Check arguments
    if (NULL == output)             return RET_ARG_ERROR;
    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;
    if (a.rows != a.cols)           return RET_ARG_ERROR;
    
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

ERROR_CODE matrix_inverse(MATRIX a, MATRIX *output) {
#if 1
    return matrix_inverse_gauss_jordan(a, output);
#else
    return matrix_inverse_standard(a, output);
#endif
}

ERROR_CODE matrix_pseudoinverse_alt1(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    MATRIX at;
    MATRIX aat;
    MATRIX aatinv;

    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;

    // Jt*(J*Jt)^-1
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

ERROR_CODE matrix_pseudoinverse_alt2(MATRIX a, MATRIX *output) {
    ERROR_CODE status;
    MATRIX result;
    MATRIX at;
    MATRIX ata;
    MATRIX atainv;

    if (false == a.allocated)       return RET_ARG_ERROR;
    if (false == output->allocated) return RET_ARG_ERROR;

    // (Jt*J)^-1*Jt
    at      = matrix_allocate(a.cols, a.rows);
    ata     = matrix_allocate(at.rows, a.cols);
    atainv  = matrix_allocate(ata.rows, ata.cols);
    result  = matrix_allocate(atainv.rows, at.cols);

    status = matrix_transpose(a, &at);
    if (RET_OK == status) {
        status = matrix_multiply(at, a, &ata);
    }
    if (RET_OK == status) {
        status = matrix_inverse(ata, &atainv);
    }
    if (RET_OK == status) {
        status = matrix_multiply(atainv, at, &result);
    }
    if (RET_OK == status) {
        status = matrix_copy(result, output);
    }
    matrix_free(result);
    matrix_free(atainv);
    matrix_free(ata);
    matrix_free(at);
    return status;
}

ERROR_CODE matrix_pseudoinverse(MATRIX a, MATRIX *output) {
    ERROR_CODE status;

    if (a.rows <= a.cols) {
        status = matrix_pseudoinverse_alt1(a,output);
        if (RET_OK != status) {
            status = matrix_pseudoinverse_alt2(a,output);
        }
    }
    else {
        status = matrix_pseudoinverse_alt2(a,output);
        if (RET_OK != status) {
            status = matrix_pseudoinverse_alt1(a,output);
        }
    }

    return status;
}

void matrix_print(MATRIX a, const char *name) {
    char string[2048] = "";
    char part_string[16] = "";  

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