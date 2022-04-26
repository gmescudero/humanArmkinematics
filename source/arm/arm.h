#ifndef __arm_h__
#define __arm_h__

#include "Quaternion.h"

typedef struct ARM_POSE_STRUCT{
    double shoulderPosition[3];
    double elbowPosition[3];
    double wristPosition[3];
} ARM_POSE;

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

#endif