/******************************************************************************/
/*
 * Project  : ExoFlex
 * Version  : 0.1
 * File:   imu.h
 * Author: Aldo Contreras
 * Description : GUI for read serial port
 * Created on June 23, 2019, 11:36
 */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#ifdef __GNUC__
#include "lpsensor/ImuData.h"
#endif


// #define M_PI           3.14159265358979323846  /* pi */

typedef enum{
    IMUS_NOT_CONNECTED = 0,
    IMUS_CONNECTED,
    IMUS_READING
}IMUS_CONNECTION_STATUS;

typedef struct _IMUS_POSE{
    // Translation vector
    float t[3];
    // Quaternion vector
    float q[4];
    // Euler angles vector
    float r[3];
}IMUS_POSE;

typedef struct _ARM_JOINTS{
    float elbow_position[3];
    float wrist_position[3];
}ARM_JOINTS;

typedef struct _ARM_MEASUREMENT{
    float arm;
    float forearm;
}ARM_MEASUREMENT;

/******************************************************************************/
/*                            Exported functions                              */
/******************************************************************************/
// extern "C" void initilize_imus(int);
// extern "C" void read_imus(ImuData);
// extern "C" void stop_imus();

bool initialize_imus(int n_imus, int init);

void read_imus(ImuData *imus);

void stop_imus();

unsigned char setOffsetIMUs(int v);

void imus_direct_kinematics(ImuData (*imus), ARM_JOINTS (*data_query), bool twoImus, ARM_MEASUREMENT arm_params, bool show);

float degree2radian(float degree);

void printMatrix(float (Mat)[4][4]);

void DH_A(float a, float alpha, float d, float theta, float (*Mat)[4]);

void MultiplyMatrix4x4(float (*Mat)[4], float Mat1[4][4], float Mat2[4][4]);

#endif /*IMU_H*/

#ifdef __cplusplus
}
#endif
