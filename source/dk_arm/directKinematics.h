#ifndef __direct_kinematics_h__
#define __direct_kinematics_h__

#include "Quaternion.h"

typedef struct ARM_POSE_STRUCT{
    double shoulderPosition[3];
    double elbowPosition[3];
    double wristPosition[3];
} ARM_POSE;

void printArmPose(const ARM_POSE pose);

void getArmPositions(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation,
    ARM_POSE *arm);

#endif