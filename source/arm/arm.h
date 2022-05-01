/**
 * @file arm.h
 * @author German Moreno Escudero
 * @brief Human Arm descriptor package
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __arm_h__
#define __arm_h__

#include "Quaternion.h"
#include "errors.h"

#define ROT_AXIS_CALIB_MAX_WINDOW (50)

typedef struct ARM_POSE_STRUCT{
    double shoulderPosition[3];
    double elbowPosition[3];
    double wristPosition[3];
} ARM_POSE;

typedef struct ARM_ROT_AXIS_CALIB_CONFIG_STRUCT
{
    int    window;
    double stepSize;
    double minVel;
} ARM_ROT_AXIS_CALIB_CONFIG;


/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

void initializeArm(
    ARM_POSE initialArmPose);

void terminateArm();

void printArmPose(
    const ARM_POSE pose);

ARM_POSE rotateArm(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation);

ARM_POSE getArmPositions();

/**
 * @brief Calibrate a rotation axis for a 1 DOF joint knowing its angular velocity
 * 
 * @param rotationV (input) Current rotation vector
 * @param omegaR (input) Current angular velocity
 * @param newRotV (output) New rotation vector
 * @return ERROR_CODE: RET_OK on success
 */
ERROR_CODE calibrateRotationAxis(
    double rotationV[3],
    double omegaR[3],
    double newRotV[3]);

#endif