#include <math.h>
#include <stdio.h>

typedef struct ImuDataStruct {
    double a[3]; // accelerometer
    double g[3]; // gyroscope
    double b[3]; // magnetometer
    double linAcc[3]; // linear acceleration
    double timeStamp; // timestamp
} ImuData;

#define EPSI (1e-9)
#define IMU_MAX_NUMBER (1)
#define QUEST_VECTORS_NUM (2)

/**
 * @brief Gyroscope noise value
 * @details The manufacturer indicates 0.007 dps/sqrt(Hz), which makes it 7.071 dps for 50 Hz
 */
#define GYR_NOISE_50HZ (7.071)
/**
 * @brief Q matrix for EKF. Process noise.
 */
double Q[36] = {
    GYR_NOISE_50HZ,0,0,0,0,0,
    0,GYR_NOISE_50HZ,0,0,0,0,
    0,0,GYR_NOISE_50HZ,0,0,0,
    0,0,0,GYR_NOISE_50HZ,0,0,
    0,0,0,0,GYR_NOISE_50HZ,0,
    0,0,0,0,0,GYR_NOISE_50HZ
};
/**
 * @brief R matrix for EKF. Sensors nosie. To be determined exactly
 */
double R[36] = {
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1
};
/**
 * @brief State vector for each sensor. gx, gy, gz, mx, my, mz
 * @details The state vector is composed by the gravity and magnetic north in cartesian coordinates
 */
static double x[IMU_MAX_NUMBER][6]  = {{0.0}};
/**
 * @brief Covariance matrix for each sensor
 */
static double P[IMU_MAX_NUMBER][36] = {{0.0}};
/**
 * @brief Time period between samples for each sensor. 1/frequency
 */
static double T[IMU_MAX_NUMBER];


double vec_norm(double a[3]) {
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

double vec_dot(double a[3], double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vec_cross(double a[3], double b[3], double result[3]) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void vec_print(double v[3], const char *name) {
    printf("Vector %s: %f,%f,%f\n",(NULL != name)?name:"", v[0],v[1],v[2]);
}

double mat_trace(double a[3][3]) {
    double result = 0.0;
    for (int i=0; i<3; i++) result += a[i][i];
    return result;
}

double mat_det(double a[3][3]) {
    return    a[0][0]*a[1][1]*a[2][2]
            + a[0][1]*a[1][2]*a[2][0]
            + a[0][2]*a[1][0]*a[2][1]
            - a[0][2]*a[1][1]*a[2][0]
            - a[0][0]*a[1][2]*a[2][1]
            - a[0][1]*a[1][0]*a[2][2];
}

void mat_vec_mul(double a[3][3], double v[3], double result[3]) {
    result[0] = a[0][0]*v[0] + a[0][1]*v[1] + a[0][2]*v[2];
    result[1] = a[1][0]*v[0] + a[1][1]*v[1] + a[1][2]*v[2];
    result[2] = a[2][0]*v[0] + a[2][1]*v[1] + a[2][2]*v[2];
}

void mat_adjoint(double M[3][3], double result[3][3]) {
   result[0][0] = M[1][1]*M[2][2] - M[1][2]*M[2][1];
   result[0][1] = -M[1][0]*M[2][2] + M[1][2]*M[2][0];
   result[0][2] = M[1][0]*M[2][1] - M[1][1]*M[2][0];
   result[1][0] = -M[0][1]*M[2][2] + M[0][2]*M[2][1];
   result[1][1] = M[0][0]*M[2][2] - M[0][2]*M[2][0];
   result[1][2] = -M[0][0]*M[2][1] + M[0][1]*M[2][0];
   result[2][0] = M[0][1]*M[1][2] - M[0][2]*M[1][1];
   result[2][1] = -M[0][0]*M[1][2] + M[0][2]*M[1][0];
   result[2][2] = M[0][0]*M[1][1] - M[0][1]*M[1][0];
}

int mat_inv(double *a, int size, double *output) {
    int status = 0;
    double aux[size][size];
    double result[size][size];
    double temp;
    
    // Initialize aux with a and result with identity matrix
    for (int i=0; i<size; i++) {
        for (int j=0; j<size; j++) {
            aux[i][j] = a[i*size +j];
            result[i][j] = (i==j)?1.0:0.0;
        }
    }

    for(int k=0; 0 == status && k<size; k++) {
        temp = aux[k][k]; 
        if (EPSI > fabs(temp)) {
            status = 1;
            printf("ERROR: Failed to invert, matrix is singular\n");
            break;
        }
        for(int j=0; j<size; j++) {
            aux[k][j] /= temp;
            result[k][j] /= temp;
        }
        for(int i=0; i<size; i++) {
            temp = aux[i][k];
            for(int j=0; j<size;j++) { 
                if(i!=k) {
                    aux[i][j]    -= aux[k][j]*temp; 
                    result[i][j] -= result[k][j]*temp;
                }
            }
        }
    }

    // Copy result to output matrix
    for (int i=0; i<size; i++) {
        for (int j=0; j<size; j++) {
            output[i*size +j] = result[i][j];
        }
    }
    return status;
}

void mat_print(double M[3][3], const char *name) {
    printf("Matrix %s:\n",(NULL != name)?name:"");
    for (int i=0;i<3;i++){
        for(int j=0;j<3;j++) {
            printf("\t%f",M[i][j]);
        }
        printf("\n");
    }
}

void kf_state_estimate(double T, double g_x, double g_y, double g_z, double h_x, double h_y, double h_z, double omega_x, double omega_y, double omega_z, double *x_kp1_k) {
   x_kp1_k[0] = -T*g_y*omega_z + T*g_z*omega_y + g_x;
   x_kp1_k[1] = T*g_x*omega_z - T*g_z*omega_x + g_y;
   x_kp1_k[2] = -T*g_x*omega_y + T*g_y*omega_x + g_z;
   x_kp1_k[3] = -T*h_y*omega_z + T*h_z*omega_y + h_x;
   x_kp1_k[4] = T*h_x*omega_z - T*h_z*omega_x + h_y;
   x_kp1_k[5] = -T*h_x*omega_y + T*h_y*omega_x + h_z;
}

void kf_covariance_estimate(double T, double *P_k_k, double *Q, double omega_x, double omega_y, double omega_z, double *P_kp1_k) {
   P_kp1_k[0] = T*omega_y*(T*omega_y*P_k_k[14] - T*omega_z*P_k_k[8]) - T*omega_z*(T*omega_y*P_k_k[13] - T*omega_z*P_k_k[7]) + Q[0];
   P_kp1_k[1] = -T*omega_x*(T*omega_y*P_k_k[14] - T*omega_z*P_k_k[8]) + T*omega_z*(T*omega_y*P_k_k[12] - T*omega_z*P_k_k[6]) + Q[1];
   P_kp1_k[2] = T*omega_x*(T*omega_y*P_k_k[13] - T*omega_z*P_k_k[7]) - T*omega_y*(T*omega_y*P_k_k[12] - T*omega_z*P_k_k[6]) + Q[2];
   P_kp1_k[3] = T*omega_y*(T*omega_y*P_k_k[17] - T*omega_z*P_k_k[11]) - T*omega_z*(T*omega_y*P_k_k[16] - T*omega_z*P_k_k[10]) + Q[3];
   P_kp1_k[4] = -T*omega_x*(T*omega_y*P_k_k[17] - T*omega_z*P_k_k[11]) + T*omega_z*(T*omega_y*P_k_k[15] - T*omega_z*P_k_k[9]) + Q[4];
   P_kp1_k[5] = T*omega_x*(T*omega_y*P_k_k[16] - T*omega_z*P_k_k[10]) - T*omega_y*(T*omega_y*P_k_k[15] - T*omega_z*P_k_k[9]) + Q[5];
   P_kp1_k[6] = T*omega_y*(-T*omega_x*P_k_k[14] + T*omega_z*P_k_k[2]) - T*omega_z*(-T*omega_x*P_k_k[13] + T*omega_z*P_k_k[1]) + Q[6];
   P_kp1_k[7] = -T*omega_x*(-T*omega_x*P_k_k[14] + T*omega_z*P_k_k[2]) + T*omega_z*(-T*omega_x*P_k_k[12] + T*omega_z*P_k_k[0]) + Q[7];
   P_kp1_k[8] = T*omega_x*(-T*omega_x*P_k_k[13] + T*omega_z*P_k_k[1]) - T*omega_y*(-T*omega_x*P_k_k[12] + T*omega_z*P_k_k[0]) + Q[8];
   P_kp1_k[9] = T*omega_y*(-T*omega_x*P_k_k[17] + T*omega_z*P_k_k[5]) - T*omega_z*(-T*omega_x*P_k_k[16] + T*omega_z*P_k_k[4]) + Q[9];
   P_kp1_k[10] = -T*omega_x*(-T*omega_x*P_k_k[17] + T*omega_z*P_k_k[5]) + T*omega_z*(-T*omega_x*P_k_k[15] + T*omega_z*P_k_k[3]) + Q[10];
   P_kp1_k[11] = T*omega_x*(-T*omega_x*P_k_k[16] + T*omega_z*P_k_k[4]) - T*omega_y*(-T*omega_x*P_k_k[15] + T*omega_z*P_k_k[3]) + Q[11];
   P_kp1_k[12] = T*omega_y*(T*omega_x*P_k_k[8] - T*omega_y*P_k_k[2]) - T*omega_z*(T*omega_x*P_k_k[7] - T*omega_y*P_k_k[1]) + Q[12];
   P_kp1_k[13] = -T*omega_x*(T*omega_x*P_k_k[8] - T*omega_y*P_k_k[2]) + T*omega_z*(T*omega_x*P_k_k[6] - T*omega_y*P_k_k[0]) + Q[13];
   P_kp1_k[14] = T*omega_x*(T*omega_x*P_k_k[7] - T*omega_y*P_k_k[1]) - T*omega_y*(T*omega_x*P_k_k[6] - T*omega_y*P_k_k[0]) + Q[14];
   P_kp1_k[15] = T*omega_y*(T*omega_x*P_k_k[11] - T*omega_y*P_k_k[5]) - T*omega_z*(T*omega_x*P_k_k[10] - T*omega_y*P_k_k[4]) + Q[15];
   P_kp1_k[16] = -T*omega_x*(T*omega_x*P_k_k[11] - T*omega_y*P_k_k[5]) + T*omega_z*(T*omega_x*P_k_k[9] - T*omega_y*P_k_k[3]) + Q[16];
   P_kp1_k[17] = T*omega_x*(T*omega_x*P_k_k[10] - T*omega_y*P_k_k[4]) - T*omega_y*(T*omega_x*P_k_k[9] - T*omega_y*P_k_k[3]) + Q[17];
   P_kp1_k[18] = T*omega_y*(T*omega_y*P_k_k[32] - T*omega_z*P_k_k[26]) - T*omega_z*(T*omega_y*P_k_k[31] - T*omega_z*P_k_k[25]) + Q[18];
   P_kp1_k[19] = -T*omega_x*(T*omega_y*P_k_k[32] - T*omega_z*P_k_k[26]) + T*omega_z*(T*omega_y*P_k_k[30] - T*omega_z*P_k_k[24]) + Q[19];
   P_kp1_k[20] = T*omega_x*(T*omega_y*P_k_k[31] - T*omega_z*P_k_k[25]) - T*omega_y*(T*omega_y*P_k_k[30] - T*omega_z*P_k_k[24]) + Q[20];
   P_kp1_k[21] = T*omega_y*(T*omega_y*P_k_k[35] - T*omega_z*P_k_k[29]) - T*omega_z*(T*omega_y*P_k_k[34] - T*omega_z*P_k_k[28]) + Q[21];
   P_kp1_k[22] = -T*omega_x*(T*omega_y*P_k_k[35] - T*omega_z*P_k_k[29]) + T*omega_z*(T*omega_y*P_k_k[33] - T*omega_z*P_k_k[27]) + Q[22];
   P_kp1_k[23] = T*omega_x*(T*omega_y*P_k_k[34] - T*omega_z*P_k_k[28]) - T*omega_y*(T*omega_y*P_k_k[33] - T*omega_z*P_k_k[27]) + Q[23];
   P_kp1_k[24] = T*omega_y*(-T*omega_x*P_k_k[32] + T*omega_z*P_k_k[20]) - T*omega_z*(-T*omega_x*P_k_k[31] + T*omega_z*P_k_k[19]) + Q[24];
   P_kp1_k[25] = -T*omega_x*(-T*omega_x*P_k_k[32] + T*omega_z*P_k_k[20]) + T*omega_z*(-T*omega_x*P_k_k[30] + T*omega_z*P_k_k[18]) + Q[25];
   P_kp1_k[26] = T*omega_x*(-T*omega_x*P_k_k[31] + T*omega_z*P_k_k[19]) - T*omega_y*(-T*omega_x*P_k_k[30] + T*omega_z*P_k_k[18]) + Q[26];
   P_kp1_k[27] = T*omega_y*(-T*omega_x*P_k_k[35] + T*omega_z*P_k_k[23]) - T*omega_z*(-T*omega_x*P_k_k[34] + T*omega_z*P_k_k[22]) + Q[27];
   P_kp1_k[28] = -T*omega_x*(-T*omega_x*P_k_k[35] + T*omega_z*P_k_k[23]) + T*omega_z*(-T*omega_x*P_k_k[33] + T*omega_z*P_k_k[21]) + Q[28];
   P_kp1_k[29] = T*omega_x*(-T*omega_x*P_k_k[34] + T*omega_z*P_k_k[22]) - T*omega_y*(-T*omega_x*P_k_k[33] + T*omega_z*P_k_k[21]) + Q[29];
   P_kp1_k[30] = T*omega_y*(T*omega_x*P_k_k[26] - T*omega_y*P_k_k[20]) - T*omega_z*(T*omega_x*P_k_k[25] - T*omega_y*P_k_k[19]) + Q[30];
   P_kp1_k[31] = -T*omega_x*(T*omega_x*P_k_k[26] - T*omega_y*P_k_k[20]) + T*omega_z*(T*omega_x*P_k_k[24] - T*omega_y*P_k_k[18]) + Q[31];
   P_kp1_k[32] = T*omega_x*(T*omega_x*P_k_k[25] - T*omega_y*P_k_k[19]) - T*omega_y*(T*omega_x*P_k_k[24] - T*omega_y*P_k_k[18]) + Q[32];
   P_kp1_k[33] = T*omega_y*(T*omega_x*P_k_k[29] - T*omega_y*P_k_k[23]) - T*omega_z*(T*omega_x*P_k_k[28] - T*omega_y*P_k_k[22]) + Q[33];
   P_kp1_k[34] = -T*omega_x*(T*omega_x*P_k_k[29] - T*omega_y*P_k_k[23]) + T*omega_z*(T*omega_x*P_k_k[27] - T*omega_y*P_k_k[21]) + Q[34];
   P_kp1_k[35] = T*omega_x*(T*omega_x*P_k_k[28] - T*omega_y*P_k_k[22]) - T*omega_y*(T*omega_x*P_k_k[27] - T*omega_y*P_k_k[21]) + Q[35];
}

void kf_state_corrected(double *P_kp1_k, double *S_inv, double *x_kp1_k, double *z_k, double *x_kp1_kp1) {
   x_kp1_kp1[0] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[0]*S_inv[0] + P_kp1_k[1]*S_inv[6] + P_kp1_k[2]*S_inv[12] + P_kp1_k[3]*S_inv[18] + P_kp1_k[4]*S_inv[24] + P_kp1_k[5]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[0]*S_inv[1] + P_kp1_k[1]*S_inv[7] + P_kp1_k[2]*S_inv[13] + P_kp1_k[3]*S_inv[19] + P_kp1_k[4]*S_inv[25] + P_kp1_k[5]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[0]*S_inv[2] + P_kp1_k[1]*S_inv[8] + P_kp1_k[2]*S_inv[14] + P_kp1_k[3]*S_inv[20] + P_kp1_k[4]*S_inv[26] + P_kp1_k[5]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[0]*S_inv[3] + P_kp1_k[1]*S_inv[9] + P_kp1_k[2]*S_inv[15] + P_kp1_k[3]*S_inv[21] + P_kp1_k[4]*S_inv[27] + P_kp1_k[5]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[0]*S_inv[4] + P_kp1_k[1]*S_inv[10] + P_kp1_k[2]*S_inv[16] + P_kp1_k[3]*S_inv[22] + P_kp1_k[4]*S_inv[28] + P_kp1_k[5]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[0]*S_inv[5] + P_kp1_k[1]*S_inv[11] + P_kp1_k[2]*S_inv[17] + P_kp1_k[3]*S_inv[23] + P_kp1_k[4]*S_inv[29] + P_kp1_k[5]*S_inv[35]) + x_kp1_k[0];
   x_kp1_kp1[1] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[6]*S_inv[0] + P_kp1_k[7]*S_inv[6] + P_kp1_k[8]*S_inv[12] + P_kp1_k[9]*S_inv[18] + P_kp1_k[10]*S_inv[24] + P_kp1_k[11]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[6]*S_inv[1] + P_kp1_k[7]*S_inv[7] + P_kp1_k[8]*S_inv[13] + P_kp1_k[9]*S_inv[19] + P_kp1_k[10]*S_inv[25] + P_kp1_k[11]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[6]*S_inv[2] + P_kp1_k[7]*S_inv[8] + P_kp1_k[8]*S_inv[14] + P_kp1_k[9]*S_inv[20] + P_kp1_k[10]*S_inv[26] + P_kp1_k[11]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[6]*S_inv[3] + P_kp1_k[7]*S_inv[9] + P_kp1_k[8]*S_inv[15] + P_kp1_k[9]*S_inv[21] + P_kp1_k[10]*S_inv[27] + P_kp1_k[11]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[6]*S_inv[4] + P_kp1_k[7]*S_inv[10] + P_kp1_k[8]*S_inv[16] + P_kp1_k[9]*S_inv[22] + P_kp1_k[10]*S_inv[28] + P_kp1_k[11]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[6]*S_inv[5] + P_kp1_k[7]*S_inv[11] + P_kp1_k[8]*S_inv[17] + P_kp1_k[9]*S_inv[23] + P_kp1_k[10]*S_inv[29] + P_kp1_k[11]*S_inv[35]) + x_kp1_k[1];
   x_kp1_kp1[2] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[12]*S_inv[0] + P_kp1_k[13]*S_inv[6] + P_kp1_k[14]*S_inv[12] + P_kp1_k[15]*S_inv[18] + P_kp1_k[16]*S_inv[24] + P_kp1_k[17]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[12]*S_inv[1] + P_kp1_k[13]*S_inv[7] + P_kp1_k[14]*S_inv[13] + P_kp1_k[15]*S_inv[19] + P_kp1_k[16]*S_inv[25] + P_kp1_k[17]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[12]*S_inv[2] + P_kp1_k[13]*S_inv[8] + P_kp1_k[14]*S_inv[14] + P_kp1_k[15]*S_inv[20] + P_kp1_k[16]*S_inv[26] + P_kp1_k[17]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[12]*S_inv[3] + P_kp1_k[13]*S_inv[9] + P_kp1_k[14]*S_inv[15] + P_kp1_k[15]*S_inv[21] + P_kp1_k[16]*S_inv[27] + P_kp1_k[17]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[12]*S_inv[4] + P_kp1_k[13]*S_inv[10] + P_kp1_k[14]*S_inv[16] + P_kp1_k[15]*S_inv[22] + P_kp1_k[16]*S_inv[28] + P_kp1_k[17]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[12]*S_inv[5] + P_kp1_k[13]*S_inv[11] + P_kp1_k[14]*S_inv[17] + P_kp1_k[15]*S_inv[23] + P_kp1_k[16]*S_inv[29] + P_kp1_k[17]*S_inv[35]) + x_kp1_k[2];
   x_kp1_kp1[3] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[18]*S_inv[0] + P_kp1_k[19]*S_inv[6] + P_kp1_k[20]*S_inv[12] + P_kp1_k[21]*S_inv[18] + P_kp1_k[22]*S_inv[24] + P_kp1_k[23]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[18]*S_inv[1] + P_kp1_k[19]*S_inv[7] + P_kp1_k[20]*S_inv[13] + P_kp1_k[21]*S_inv[19] + P_kp1_k[22]*S_inv[25] + P_kp1_k[23]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[18]*S_inv[2] + P_kp1_k[19]*S_inv[8] + P_kp1_k[20]*S_inv[14] + P_kp1_k[21]*S_inv[20] + P_kp1_k[22]*S_inv[26] + P_kp1_k[23]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[18]*S_inv[3] + P_kp1_k[19]*S_inv[9] + P_kp1_k[20]*S_inv[15] + P_kp1_k[21]*S_inv[21] + P_kp1_k[22]*S_inv[27] + P_kp1_k[23]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[18]*S_inv[4] + P_kp1_k[19]*S_inv[10] + P_kp1_k[20]*S_inv[16] + P_kp1_k[21]*S_inv[22] + P_kp1_k[22]*S_inv[28] + P_kp1_k[23]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[18]*S_inv[5] + P_kp1_k[19]*S_inv[11] + P_kp1_k[20]*S_inv[17] + P_kp1_k[21]*S_inv[23] + P_kp1_k[22]*S_inv[29] + P_kp1_k[23]*S_inv[35]) + x_kp1_k[3];
   x_kp1_kp1[4] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[24]*S_inv[0] + P_kp1_k[25]*S_inv[6] + P_kp1_k[26]*S_inv[12] + P_kp1_k[27]*S_inv[18] + P_kp1_k[28]*S_inv[24] + P_kp1_k[29]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[24]*S_inv[1] + P_kp1_k[25]*S_inv[7] + P_kp1_k[26]*S_inv[13] + P_kp1_k[27]*S_inv[19] + P_kp1_k[28]*S_inv[25] + P_kp1_k[29]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[24]*S_inv[2] + P_kp1_k[25]*S_inv[8] + P_kp1_k[26]*S_inv[14] + P_kp1_k[27]*S_inv[20] + P_kp1_k[28]*S_inv[26] + P_kp1_k[29]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[24]*S_inv[3] + P_kp1_k[25]*S_inv[9] + P_kp1_k[26]*S_inv[15] + P_kp1_k[27]*S_inv[21] + P_kp1_k[28]*S_inv[27] + P_kp1_k[29]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[24]*S_inv[4] + P_kp1_k[25]*S_inv[10] + P_kp1_k[26]*S_inv[16] + P_kp1_k[27]*S_inv[22] + P_kp1_k[28]*S_inv[28] + P_kp1_k[29]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[24]*S_inv[5] + P_kp1_k[25]*S_inv[11] + P_kp1_k[26]*S_inv[17] + P_kp1_k[27]*S_inv[23] + P_kp1_k[28]*S_inv[29] + P_kp1_k[29]*S_inv[35]) + x_kp1_k[4];
   x_kp1_kp1[5] = (-x_kp1_k[0] + z_k[0])*(P_kp1_k[30]*S_inv[0] + P_kp1_k[31]*S_inv[6] + P_kp1_k[32]*S_inv[12] + P_kp1_k[33]*S_inv[18] + P_kp1_k[34]*S_inv[24] + P_kp1_k[35]*S_inv[30]) + (-x_kp1_k[1] + z_k[1])*(P_kp1_k[30]*S_inv[1] + P_kp1_k[31]*S_inv[7] + P_kp1_k[32]*S_inv[13] + P_kp1_k[33]*S_inv[19] + P_kp1_k[34]*S_inv[25] + P_kp1_k[35]*S_inv[31]) + (-x_kp1_k[2] + z_k[2])*(P_kp1_k[30]*S_inv[2] + P_kp1_k[31]*S_inv[8] + P_kp1_k[32]*S_inv[14] + P_kp1_k[33]*S_inv[20] + P_kp1_k[34]*S_inv[26] + P_kp1_k[35]*S_inv[32]) + (-x_kp1_k[3] + z_k[3])*(P_kp1_k[30]*S_inv[3] + P_kp1_k[31]*S_inv[9] + P_kp1_k[32]*S_inv[15] + P_kp1_k[33]*S_inv[21] + P_kp1_k[34]*S_inv[27] + P_kp1_k[35]*S_inv[33]) + (-x_kp1_k[4] + z_k[4])*(P_kp1_k[30]*S_inv[4] + P_kp1_k[31]*S_inv[10] + P_kp1_k[32]*S_inv[16] + P_kp1_k[33]*S_inv[22] + P_kp1_k[34]*S_inv[28] + P_kp1_k[35]*S_inv[34]) + (-x_kp1_k[5] + z_k[5])*(P_kp1_k[30]*S_inv[5] + P_kp1_k[31]*S_inv[11] + P_kp1_k[32]*S_inv[17] + P_kp1_k[33]*S_inv[23] + P_kp1_k[34]*S_inv[29] + P_kp1_k[35]*S_inv[35]) + x_kp1_k[5];
}

void kf_covariance_corrected(double *P_kp1_k, double *S_inv, double *P_kp1_kp1) {
   P_kp1_kp1[0] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[6] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[12] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[18] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[24] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[30] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[0];
   P_kp1_kp1[1] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[7] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[13] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[19] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[25] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[31] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[1];
   P_kp1_kp1[2] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[8] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[14] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[20] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[26] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[32] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[2];
   P_kp1_kp1[3] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[9] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[15] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[21] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[27] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[33] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[3];
   P_kp1_kp1[4] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[10] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[16] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[22] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[28] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[34] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[4];
   P_kp1_kp1[5] = (-P_kp1_k[0]*S_inv[1] - P_kp1_k[1]*S_inv[7] - P_kp1_k[2]*S_inv[13] - P_kp1_k[3]*S_inv[19] - P_kp1_k[4]*S_inv[25] - P_kp1_k[5]*S_inv[31])*P_kp1_k[11] + (-P_kp1_k[0]*S_inv[2] - P_kp1_k[1]*S_inv[8] - P_kp1_k[2]*S_inv[14] - P_kp1_k[3]*S_inv[20] - P_kp1_k[4]*S_inv[26] - P_kp1_k[5]*S_inv[32])*P_kp1_k[17] + (-P_kp1_k[0]*S_inv[3] - P_kp1_k[1]*S_inv[9] - P_kp1_k[2]*S_inv[15] - P_kp1_k[3]*S_inv[21] - P_kp1_k[4]*S_inv[27] - P_kp1_k[5]*S_inv[33])*P_kp1_k[23] + (-P_kp1_k[0]*S_inv[4] - P_kp1_k[1]*S_inv[10] - P_kp1_k[2]*S_inv[16] - P_kp1_k[3]*S_inv[22] - P_kp1_k[4]*S_inv[28] - P_kp1_k[5]*S_inv[34])*P_kp1_k[29] + (-P_kp1_k[0]*S_inv[5] - P_kp1_k[1]*S_inv[11] - P_kp1_k[2]*S_inv[17] - P_kp1_k[3]*S_inv[23] - P_kp1_k[4]*S_inv[29] - P_kp1_k[5]*S_inv[35])*P_kp1_k[35] + (-P_kp1_k[0]*S_inv[0] - P_kp1_k[1]*S_inv[6] - P_kp1_k[2]*S_inv[12] - P_kp1_k[3]*S_inv[18] - P_kp1_k[4]*S_inv[24] - P_kp1_k[5]*S_inv[30] + 1)*P_kp1_k[5];
   P_kp1_kp1[6] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[0] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[12] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[18] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[24] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[30] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[6];
   P_kp1_kp1[7] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[1] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[13] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[19] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[25] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[31] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[7];
   P_kp1_kp1[8] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[2] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[14] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[20] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[26] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[32] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[8];
   P_kp1_kp1[9] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[3] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[15] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[21] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[27] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[33] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[9];
   P_kp1_kp1[10] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[4] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[16] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[22] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[28] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[34] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[10];
   P_kp1_kp1[11] = (-P_kp1_k[6]*S_inv[0] - P_kp1_k[7]*S_inv[6] - P_kp1_k[8]*S_inv[12] - P_kp1_k[9]*S_inv[18] - P_kp1_k[10]*S_inv[24] - P_kp1_k[11]*S_inv[30])*P_kp1_k[5] + (-P_kp1_k[6]*S_inv[2] - P_kp1_k[7]*S_inv[8] - P_kp1_k[8]*S_inv[14] - P_kp1_k[9]*S_inv[20] - P_kp1_k[10]*S_inv[26] - P_kp1_k[11]*S_inv[32])*P_kp1_k[17] + (-P_kp1_k[6]*S_inv[3] - P_kp1_k[7]*S_inv[9] - P_kp1_k[8]*S_inv[15] - P_kp1_k[9]*S_inv[21] - P_kp1_k[10]*S_inv[27] - P_kp1_k[11]*S_inv[33])*P_kp1_k[23] + (-P_kp1_k[6]*S_inv[4] - P_kp1_k[7]*S_inv[10] - P_kp1_k[8]*S_inv[16] - P_kp1_k[9]*S_inv[22] - P_kp1_k[10]*S_inv[28] - P_kp1_k[11]*S_inv[34])*P_kp1_k[29] + (-P_kp1_k[6]*S_inv[5] - P_kp1_k[7]*S_inv[11] - P_kp1_k[8]*S_inv[17] - P_kp1_k[9]*S_inv[23] - P_kp1_k[10]*S_inv[29] - P_kp1_k[11]*S_inv[35])*P_kp1_k[35] + (-P_kp1_k[6]*S_inv[1] - P_kp1_k[7]*S_inv[7] - P_kp1_k[8]*S_inv[13] - P_kp1_k[9]*S_inv[19] - P_kp1_k[10]*S_inv[25] - P_kp1_k[11]*S_inv[31] + 1)*P_kp1_k[11];
   P_kp1_kp1[12] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[0] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[6] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[18] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[24] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[30] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[12];
   P_kp1_kp1[13] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[1] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[7] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[19] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[25] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[31] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[13];
   P_kp1_kp1[14] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[2] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[8] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[20] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[26] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[32] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[14];
   P_kp1_kp1[15] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[3] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[9] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[21] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[27] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[33] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[15];
   P_kp1_kp1[16] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[4] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[10] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[22] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[28] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[34] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[16];
   P_kp1_kp1[17] = (-P_kp1_k[12]*S_inv[0] - P_kp1_k[13]*S_inv[6] - P_kp1_k[14]*S_inv[12] - P_kp1_k[15]*S_inv[18] - P_kp1_k[16]*S_inv[24] - P_kp1_k[17]*S_inv[30])*P_kp1_k[5] + (-P_kp1_k[12]*S_inv[1] - P_kp1_k[13]*S_inv[7] - P_kp1_k[14]*S_inv[13] - P_kp1_k[15]*S_inv[19] - P_kp1_k[16]*S_inv[25] - P_kp1_k[17]*S_inv[31])*P_kp1_k[11] + (-P_kp1_k[12]*S_inv[3] - P_kp1_k[13]*S_inv[9] - P_kp1_k[14]*S_inv[15] - P_kp1_k[15]*S_inv[21] - P_kp1_k[16]*S_inv[27] - P_kp1_k[17]*S_inv[33])*P_kp1_k[23] + (-P_kp1_k[12]*S_inv[4] - P_kp1_k[13]*S_inv[10] - P_kp1_k[14]*S_inv[16] - P_kp1_k[15]*S_inv[22] - P_kp1_k[16]*S_inv[28] - P_kp1_k[17]*S_inv[34])*P_kp1_k[29] + (-P_kp1_k[12]*S_inv[5] - P_kp1_k[13]*S_inv[11] - P_kp1_k[14]*S_inv[17] - P_kp1_k[15]*S_inv[23] - P_kp1_k[16]*S_inv[29] - P_kp1_k[17]*S_inv[35])*P_kp1_k[35] + (-P_kp1_k[12]*S_inv[2] - P_kp1_k[13]*S_inv[8] - P_kp1_k[14]*S_inv[14] - P_kp1_k[15]*S_inv[20] - P_kp1_k[16]*S_inv[26] - P_kp1_k[17]*S_inv[32] + 1)*P_kp1_k[17];
   P_kp1_kp1[18] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[0] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[6] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[12] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[24] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[30] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[18];
   P_kp1_kp1[19] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[1] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[7] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[13] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[25] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[31] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[19];
   P_kp1_kp1[20] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[2] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[8] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[14] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[26] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[32] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[20];
   P_kp1_kp1[21] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[3] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[9] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[15] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[27] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[33] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[21];
   P_kp1_kp1[22] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[4] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[10] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[16] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[28] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[34] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[22];
   P_kp1_kp1[23] = (-P_kp1_k[18]*S_inv[0] - P_kp1_k[19]*S_inv[6] - P_kp1_k[20]*S_inv[12] - P_kp1_k[21]*S_inv[18] - P_kp1_k[22]*S_inv[24] - P_kp1_k[23]*S_inv[30])*P_kp1_k[5] + (-P_kp1_k[18]*S_inv[1] - P_kp1_k[19]*S_inv[7] - P_kp1_k[20]*S_inv[13] - P_kp1_k[21]*S_inv[19] - P_kp1_k[22]*S_inv[25] - P_kp1_k[23]*S_inv[31])*P_kp1_k[11] + (-P_kp1_k[18]*S_inv[2] - P_kp1_k[19]*S_inv[8] - P_kp1_k[20]*S_inv[14] - P_kp1_k[21]*S_inv[20] - P_kp1_k[22]*S_inv[26] - P_kp1_k[23]*S_inv[32])*P_kp1_k[17] + (-P_kp1_k[18]*S_inv[4] - P_kp1_k[19]*S_inv[10] - P_kp1_k[20]*S_inv[16] - P_kp1_k[21]*S_inv[22] - P_kp1_k[22]*S_inv[28] - P_kp1_k[23]*S_inv[34])*P_kp1_k[29] + (-P_kp1_k[18]*S_inv[5] - P_kp1_k[19]*S_inv[11] - P_kp1_k[20]*S_inv[17] - P_kp1_k[21]*S_inv[23] - P_kp1_k[22]*S_inv[29] - P_kp1_k[23]*S_inv[35])*P_kp1_k[35] + (-P_kp1_k[18]*S_inv[3] - P_kp1_k[19]*S_inv[9] - P_kp1_k[20]*S_inv[15] - P_kp1_k[21]*S_inv[21] - P_kp1_k[22]*S_inv[27] - P_kp1_k[23]*S_inv[33] + 1)*P_kp1_k[23];
   P_kp1_kp1[24] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[0] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[6] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[12] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[18] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[30] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[24];
   P_kp1_kp1[25] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[1] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[7] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[13] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[19] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[31] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[25];
   P_kp1_kp1[26] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[2] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[8] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[14] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[20] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[32] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[26];
   P_kp1_kp1[27] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[3] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[9] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[15] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[21] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[33] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[27];
   P_kp1_kp1[28] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[4] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[10] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[16] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[22] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[34] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[28];
   P_kp1_kp1[29] = (-P_kp1_k[24]*S_inv[0] - P_kp1_k[25]*S_inv[6] - P_kp1_k[26]*S_inv[12] - P_kp1_k[27]*S_inv[18] - P_kp1_k[28]*S_inv[24] - P_kp1_k[29]*S_inv[30])*P_kp1_k[5] + (-P_kp1_k[24]*S_inv[1] - P_kp1_k[25]*S_inv[7] - P_kp1_k[26]*S_inv[13] - P_kp1_k[27]*S_inv[19] - P_kp1_k[28]*S_inv[25] - P_kp1_k[29]*S_inv[31])*P_kp1_k[11] + (-P_kp1_k[24]*S_inv[2] - P_kp1_k[25]*S_inv[8] - P_kp1_k[26]*S_inv[14] - P_kp1_k[27]*S_inv[20] - P_kp1_k[28]*S_inv[26] - P_kp1_k[29]*S_inv[32])*P_kp1_k[17] + (-P_kp1_k[24]*S_inv[3] - P_kp1_k[25]*S_inv[9] - P_kp1_k[26]*S_inv[15] - P_kp1_k[27]*S_inv[21] - P_kp1_k[28]*S_inv[27] - P_kp1_k[29]*S_inv[33])*P_kp1_k[23] + (-P_kp1_k[24]*S_inv[5] - P_kp1_k[25]*S_inv[11] - P_kp1_k[26]*S_inv[17] - P_kp1_k[27]*S_inv[23] - P_kp1_k[28]*S_inv[29] - P_kp1_k[29]*S_inv[35])*P_kp1_k[35] + (-P_kp1_k[24]*S_inv[4] - P_kp1_k[25]*S_inv[10] - P_kp1_k[26]*S_inv[16] - P_kp1_k[27]*S_inv[22] - P_kp1_k[28]*S_inv[28] - P_kp1_k[29]*S_inv[34] + 1)*P_kp1_k[29];
   P_kp1_kp1[30] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[0] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[6] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[12] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[18] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[24] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[30];
   P_kp1_kp1[31] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[1] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[7] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[13] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[19] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[25] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[31];
   P_kp1_kp1[32] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[2] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[8] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[14] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[20] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[26] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[32];
   P_kp1_kp1[33] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[3] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[9] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[15] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[21] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[27] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[33];
   P_kp1_kp1[34] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[4] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[10] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[16] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[22] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[28] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[34];
   P_kp1_kp1[35] = (-P_kp1_k[30]*S_inv[0] - P_kp1_k[31]*S_inv[6] - P_kp1_k[32]*S_inv[12] - P_kp1_k[33]*S_inv[18] - P_kp1_k[34]*S_inv[24] - P_kp1_k[35]*S_inv[30])*P_kp1_k[5] + (-P_kp1_k[30]*S_inv[1] - P_kp1_k[31]*S_inv[7] - P_kp1_k[32]*S_inv[13] - P_kp1_k[33]*S_inv[19] - P_kp1_k[34]*S_inv[25] - P_kp1_k[35]*S_inv[31])*P_kp1_k[11] + (-P_kp1_k[30]*S_inv[2] - P_kp1_k[31]*S_inv[8] - P_kp1_k[32]*S_inv[14] - P_kp1_k[33]*S_inv[20] - P_kp1_k[34]*S_inv[26] - P_kp1_k[35]*S_inv[32])*P_kp1_k[17] + (-P_kp1_k[30]*S_inv[3] - P_kp1_k[31]*S_inv[9] - P_kp1_k[32]*S_inv[15] - P_kp1_k[33]*S_inv[21] - P_kp1_k[34]*S_inv[27] - P_kp1_k[35]*S_inv[33])*P_kp1_k[23] + (-P_kp1_k[30]*S_inv[4] - P_kp1_k[31]*S_inv[10] - P_kp1_k[32]*S_inv[16] - P_kp1_k[33]*S_inv[22] - P_kp1_k[34]*S_inv[28] - P_kp1_k[35]*S_inv[34])*P_kp1_k[29] + (-P_kp1_k[30]*S_inv[5] - P_kp1_k[31]*S_inv[11] - P_kp1_k[32]*S_inv[17] - P_kp1_k[33]*S_inv[23] - P_kp1_k[34]*S_inv[29] - P_kp1_k[35]*S_inv[35] + 1)*P_kp1_k[35];
}


void kf_initialize(int imuInd, double grv[3], double mag[3], double period) {
    T[imuInd] = period;
    for (int i=0; i<3; i++) {
        x[imuInd][i]   = grv[2-i];
        x[imuInd][i+3] = mag[i];
    }
    for (int i=0; i<6; i++) {
        int ind = i*6+i;
        P[imuInd][ind] = 100.0*R[ind];
    }
}


void kf_gravity_and_north_filter(int imuInd, double gyr[3], double grv[3], double mag[3]) {
    double x_kp1_k[6]  = {0};
    double P_kp1_k[36] = {0};
    double z[6] = {grv[0], grv[1], grv[2], mag[0], mag[1], mag[2]};
    double y[6] = {0};
    double S[36] = {0};
    double S_inv[36] = {0};
    double K[36] = {0};
    double Ky[6] = {0};
    double I_K[36] = {0};

    // Predict
    kf_state_estimate(T[imuInd],
        x[imuInd][0],x[imuInd][1],x[imuInd][2],x[imuInd][3],x[imuInd][4],x[imuInd][5],
        gyr[0],gyr[1],gyr[2], 
        x_kp1_k);
    kf_covariance_estimate(T[imuInd],
        P[imuInd],Q,
        gyr[0],gyr[1],gyr[2], 
        P_kp1_k);
    // Correct
    for (int i=0;i<6;i++) {
        y[i] = z[i] - x_kp1_k[i];
    }
    for (int i=0;i<36;i++) {
        S[i] = P_kp1_k[i] + R[i];
    }
    mat_inv(S,6,S_inv);
    kf_state_corrected(P_kp1_k, S_inv, x_kp1_k, z, x[imuInd]);
    kf_covariance_corrected(P_kp1_k, S_inv, P[imuInd]);

    for (int i=0; i<3; i++) {
        grv[i] = x[imuInd][i];
        mag[i] = x[imuInd][i+3];
    }
}

int quest_2_vectors_compute(double r[QUEST_VECTORS_NUM][3], double b[QUEST_VECTORS_NUM][3], double weights[QUEST_VECTORS_NUM], double q[4]) {
    static int recursion = 0;
    double B[3][3] = {{0.0}};
    double z[3]    = {0.0};
    double a[QUEST_VECTORS_NUM];
    double norm_a   = 0.0;

    // Normalize weights
    for(int i=0; i<QUEST_VECTORS_NUM; i++) {
        norm_a += weights[i]*weights[i];
    }
    norm_a = sqrt(norm_a);
    for(int i=0; i<QUEST_VECTORS_NUM; i++) {
        a[i] = weights[i]/norm_a;
    }

    // Set B and z
    for (int k=0; k<QUEST_VECTORS_NUM; k++) {
        double br[3][3] = {
            b[k][0]*r[k][0], b[k][0]*r[k][1], b[k][0]*r[k][2],
            b[k][1]*r[k][0], b[k][1]*r[k][1], b[k][1]*r[k][2],
            b[k][2]*r[k][0], b[k][2]*r[k][1], b[k][2]*r[k][2]
        };
        double bxr[3];
        vec_cross(b[k],r[k], bxr);
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                B[i][j] += a[k]*br[i][j];
            }
            z[i] = z[i] + a[k]*bxr[i];
        }
    }

    // Lambda max
    double rxr[3];
    double bxb[3];
    vec_cross(r[0],r[1], rxr);
    vec_cross(b[0],b[1], bxb);
    double aux = vec_dot(r[0],r[1])*vec_dot(b[0],b[1]) + vec_norm(rxr)*vec_norm(bxb);
    double lambda = sqrt(a[0]*a[0] + a[1]*a[1] + 2.0*a[0]*a[1]*aux);
    // Gamma
    double P[3][3] = {{0.0}};
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            P[i][j] = ((i==j)?(lambda+mat_trace(B)):0.0) - B[i][j] - B[j][i];
        }
    }
    double gamma = mat_det(P);
    if (EPSI > gamma) {
        if (1 == recursion) {
            return 1;
        }

        // rotation angle is 180, perfoming sequential rotations method
        recursion = 1;
        double r_aux[QUEST_VECTORS_NUM][3];
        double q_aux[4];

        // Rotation of 180 around x
        printf("QUEST -> 180 x\n");
        for (int i=0; i<QUEST_VECTORS_NUM; i++) {
            r_aux[i][0] =  r[i][0]; r_aux[i][1] = -r[i][1]; r_aux[i][2] = -r[i][2];
        }
        recursion = quest_2_vectors_compute(r_aux,b,a, q_aux);
        if (0 == recursion) {
            q[0] =  q_aux[1]; q[1] = -q_aux[0]; q[2] = -q_aux[2]; q[3] =  q_aux[3]; 
            return 0;
        }
        // Rotation of 180 around y
        printf("QUEST -> 180 y\n");
        for (int i=0; i<QUEST_VECTORS_NUM; i++) {
            r_aux[i][0] = -r[i][0]; r_aux[i][1] =  r[i][1]; r_aux[i][2] = -r[i][2]; 
        }
        recursion = quest_2_vectors_compute(r_aux,b,a, q_aux);
        if (0 == recursion) {
            q[0] =  q_aux[2]; q[1] = -q_aux[1]; q[2] = -q_aux[0]; q[3] =  q_aux[3]; 
            return 0;
        }
        // Rotation of 180 around z
        printf("QUEST -> 180 z\n");
        for (int i=0; i<QUEST_VECTORS_NUM; i++) {
            r_aux[i][0] = -r[i][0]; r_aux[i][1] = -r[i][1]; r_aux[i][2] =  r[i][2];
        }
        recursion = quest_2_vectors_compute(r_aux,b,a, q_aux);
        if (0 == recursion) {
            q[0] =  q_aux[3]; q[1] = -q_aux[2]; q[2] = -q_aux[1]; q[3] = -q_aux[0]; 
            return 0;
        }
    }

    // build vector part of the quaternion (x)
    double adjP[3][3];
    double x[3];
    mat_adjoint(P, adjP);
    mat_vec_mul(adjP,z, x);

    // build quaternion
    double scale = 1.0/(sqrt(gamma*gamma + vec_norm(x)*vec_norm(x)));
    double norm_acumulate = 0.0;
    q[0] = scale* gamma;
    norm_acumulate = q[0]*q[0];
    for (int i=0; i<3; i++) {
        q[i+1] = scale*x[i];
        norm_acumulate += q[i+1]*q[i+1];
    }
    norm_acumulate = sqrt(norm_acumulate);
    for (int i=0; i<4; i++) {
        q[i] = q[i] / norm_acumulate;
    }

    return 0;
}


void quaternion_from_imu_data_compute(int imuInd, ImuData d, double q[4]) {
    double gyr[3] = {d.g[0],d.g[1],d.g[2]};
    double grv[3] = {d.a[0]-d.linAcc[0],d.a[1]-d.linAcc[1],d.a[2]-d.linAcc[2]};
    double mag[3] = {d.b[0],d.b[1],d.b[2]};

    // Filter gravity and Magnetic North from imu data
    kf_gravity_and_north_filter(imuInd,gyr, grv, mag);
    printf("Gravity: %f,%f,%f\n",grv[0],grv[1],grv[2]);
    printf("North:   %f,%f,%f\n",mag[0],mag[1],mag[2]);

    // Compute the quaternion from the gravity and magnetic references
    double reference[QUEST_VECTORS_NUM][3] = {
        0,0,-1,
        1,0,0
    };
    double norm_grv = vec_norm(grv);
    double norm_mag = vec_norm(mag);
    double actual[QUEST_VECTORS_NUM][3] = {
        grv[0]/norm_grv,grv[1]/norm_grv,grv[2]/norm_grv,
        mag[0]/norm_mag,mag[1]/norm_mag,mag[2]/norm_mag
    };
    double weights[QUEST_VECTORS_NUM] = {0.5,0.5};
    quest_2_vectors_compute(reference,actual,weights, q);
}

int main(int argc, char **argv) {
#if 1
    double period = 0.02;
    ImuData imu = {
        .a = {0.0, 0.0,-1.0},
        .g = {0.0, 0.0, 1.57},
        .b = {1.0, 0.0, 0.0},
        .linAcc = {0.0,0.0,0.0},
        .timeStamp = 0
    };
    double q[4] = {0.0};
    kf_initialize(0,imu.a,imu.b,period);
    for (int i = 0; i < 150; i++) {
        imu.b[0] = cos(period*imu.g[2]*i);
        imu.b[1] = sin(period*imu.g[2]*i);
        quaternion_from_imu_data_compute(0, imu, q);
        printf("[%d] Quaternion: %f, %f,%f,%f\n\n",i,q[0],q[1],q[2],q[3]);
    }
    imu.g[2] = 0.0;
    for (int i = 0; i < 50; i++) {
        quaternion_from_imu_data_compute(0, imu, q);
        printf("[%d] Quaternion: %f, %f,%f,%f\n\n",i+150,q[0],q[1],q[2],q[3]);
    }
#else
    double reference[QUEST_VECTORS_NUM][3] = {
        0,0,1,
        1,0,0
    };
    double actual1[QUEST_VECTORS_NUM][3] = {
        0,0,1,
        -1,0,0
    };
    double actual2[QUEST_VECTORS_NUM][3] = {
        0,0,-1,
        1,0,0
    };
    double weights[QUEST_VECTORS_NUM] = {0.7071,0.7071};
    double q[4] = {0.0};
    quest_2_vectors_compute(reference,actual1,weights,q);
    printf("Quaternion: %f, %f,%f,%f\n\n",q[0],q[1],q[2],q[3]);
    quest_2_vectors_compute(reference,actual2,weights,q);
    printf("Quaternion: %f, %f,%f,%f\n\n",q[0],q[1],q[2],q[3]);
#endif
}