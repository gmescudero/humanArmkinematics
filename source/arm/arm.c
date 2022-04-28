/**
 * @file arm.c
 * @author German Moreno Escudero
 * @brief Human Arm descriptor package
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "arm.h"
#include "Quaternion.h"
#include "vector3.h"
#include <string.h>
#include <stdio.h>

#define DEFAULT_SHOULDER_2_ELBOW_LENGTH (10.0)
#define DEFAULT_ELBOW_2_WRIST_LENGTH    (5.0)
#define DEFAULT_TOTAL_ARM_LENGTH (DEFAULT_SHOULDER_2_ELBOW_LENGTH+DEFAULT_ELBOW_2_WRIST_LENGTH)

static bool initialized = false;

static ARM_POSE currentPose = {
        .shoulderPosition = {0.0, 0.0, 0.0},
        .elbowPosition    = {0.0, 0.0, -DEFAULT_SHOULDER_2_ELBOW_LENGTH},
        .wristPosition    = {0.0, 0.0, -DEFAULT_TOTAL_ARM_LENGTH},
    };


void initializeArm(ARM_POSE initialArmPose)
{
    memcpy(&currentPose, &initialArmPose, sizeof(ARM_POSE));
    initialized = true;   
}

void terminateArm()
{
    bzero(&currentPose,sizeof(ARM_POSE));
    initialized = false;   
}

/**
 * Print the arm pose
 */
void printArmPose(const ARM_POSE pose)
{
    printf("Arm pose:\n");
    printf("\t sh: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.shoulderPosition[0],pose.shoulderPosition[1],pose.shoulderPosition[2]);
    printf("\t el: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.elbowPosition[0],pose.elbowPosition[1],pose.elbowPosition[2]);
    printf("\t wr: \t<%0.4f>\t<%0.4f>\t<%0.4f> \n",pose.wristPosition[0],pose.wristPosition[1],pose.wristPosition[2]);
}

/**
 * Rotate the arm segments by two given quaternions
 */
ARM_POSE rotateArm(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation)
{
    double sh2el_vector[3];
    double sh2el_vector_rot[3];
    double el2wr_vector[3];
    double el2wr_vector_rot[3];

    if (!initialized) return currentPose;
    // Compute arm vectors from joints positions positions
    vector3_substract(currentPose.elbowPosition, currentPose.shoulderPosition, sh2el_vector);
    vector3_substract(currentPose.wristPosition, currentPose.elbowPosition, el2wr_vector);

    // Rotate the arm vectors, applying the shoulder rotation to both shoulder and elbow
    Quaternion_rotate(&sh2el_orientation, sh2el_vector, sh2el_vector_rot);
    Quaternion_rotate(&sh2el_orientation, el2wr_vector, el2wr_vector);
    Quaternion_rotate(&el2wr_orientation, el2wr_vector, el2wr_vector_rot);

    // Compute the resulting elbow and wrist position
    vector3_add(currentPose.shoulderPosition, sh2el_vector_rot, currentPose.elbowPosition);
    vector3_add(currentPose.elbowPosition, el2wr_vector_rot, currentPose.wristPosition);

    return currentPose;
}

/**
 * Get the arm pose given the orientation of each segment
 */
ARM_POSE getArmPositions()
{
    return currentPose;
}

