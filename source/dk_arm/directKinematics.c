#include "directKinematics.h"
#include "Quaternion.h"
#include "vector3.h"
#include <stdio.h>


/**
 * Print the arm pose
 */
void printArmPose(const ARM_POSE pose)
{
    printf("\n Arm pose:\n");
    printf("\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.shoulderPosition[0],pose.shoulderPosition[1],pose.shoulderPosition[2]);
    printf("\t el: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.elbowPosition[0],pose.elbowPosition[1],pose.elbowPosition[2]);
    printf("\t wr: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.wristPosition[0],pose.wristPosition[1],pose.wristPosition[2]);
}

/**
 * Get the arm pose given the orientation of each segment
 */
void getArmPositions(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation,
    ARM_POSE *arm)
{
    ARM_POSE initialPose = {
        .shoulderPosition = {0.0, 0.0, 0.0},
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };
    double sh2el_vector[3];
    double sh2el_vector_rot[3];
    double el2wr_vector[3];
    double el2wr_vector_rot[3];

    vector3_substract(initialPose.elbowPosition, initialPose.shoulderPosition, sh2el_vector);
    vector3_substract(initialPose.wristPosition, initialPose.elbowPosition, el2wr_vector);

    Quaternion_rotate(&sh2el_orientation, sh2el_vector, sh2el_vector_rot);
    Quaternion_rotate(&el2wr_orientation, el2wr_vector, el2wr_vector_rot);

    vector3_add(arm->shoulderPosition, sh2el_vector_rot, arm->elbowPosition);
    vector3_add(arm->elbowPosition, el2wr_vector_rot, arm->wristPosition);
}