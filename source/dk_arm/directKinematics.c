#include "directKinematics.h"
#include "Quaternion.h"
#include <stdio.h>

/**
 * Adds two vectors of length 3
 */
void vector3_add(double a[3], double b[3], double output[3])
{
    output[0] = a[0] + b[0];
    output[1] = a[1] + b[1];
    output[2] = a[2] + b[2];
}

/**
 * Substracts two vectors of length 3
 */
void vector3_substract(double a[3], double b[3], double output[3])
{
    output[0] = a[0] - b[0];
    output[1] = a[1] - b[1];
    output[2] = a[2] - b[2];
}

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