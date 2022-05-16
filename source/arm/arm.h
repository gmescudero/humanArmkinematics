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

void arm_joint_positions_set(
    ARM_POSE initial_arm_pose);

void arm_pose_print(
    const ARM_POSE pose);

ARM_POSE arm_rotate(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation);

ARM_POSE arm_pose_get();

/**
 * @brief Compute the angular velocity from one frame
 * 
 * @param q1 (input) The orientation of the reference frame
 * @param q2 (input) The orientation of the second frame
 * @param angVel1 (input) The angular velocity of the reference frame
 * @param angVel2 (input) The angular velocity of the second frame
 * @param angVelR (output) The computed angular velocity
 * @return ERROR_CODE 
 */
ERROR_CODE arm_relative_angular_vel_compute(
    Quaternion q1, Quaternion q2, 
    double angVel1[3], double angVel2[3], double angVelR[3]);

/**
 * @brief Calibrate a rotation axis for a 1 DOF joint knowing its angular velocity
 * 
 * @param omegaR (input) Current angular velocity
 * @param rotationV (input/output) Rotation vector
 * @return ERROR_CODE:
 *  - RET_OK on success
 *  - RET_ERROR otherwise
 */
ERROR_CODE arm_calibrate_rotation_axis(
    double omegaR[3],
    double rotationV[3]);

#endif