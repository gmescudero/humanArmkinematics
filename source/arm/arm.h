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

#define ARM_POSE_STRING_MAX_LENGTH (256)

#define ARM_NUMBER_OF_JOINTS (2)

enum ARM_KINEMATIC_NODE_ENUM {
    SHOULDER,
    ELBOW,
    WRIST,
    NUMBER_OF_NODES
} ARM_KINEMATIC_NODE;


typedef struct ARM_FRAME_STRUCT {
    double position[3];
    Quaternion orientation;
} ARM_FRAME;


typedef struct ARM_POSE_STRUCT {
    ARM_FRAME shoulder;
    ARM_FRAME elbow;
    ARM_FRAME wrist;
} ARM_POSE;



/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

/**
 * @brief Set the length of upper arm and forearm
 * 
 * @param upper_arm (input) Length from shoulder to elbow
 * @param forearm (input) Length from elbow to wrist
 * @return ERROR_CODE 
 */
ERROR_CODE arm_segments_length_set(double upper_arm, double forearm);

void arm_joint_positions_set(
    ARM_POSE initial_arm_pose);

/**
 * @brief Convert arm pose to a string
 * 
 * @param pose (input) Given arm pose
 * @param pose_str (output) Arm pose converted to string
 */
void arm_pose_to_string(
    const ARM_POSE pose, 
    char pose_str[]);

/**
 * @brief Print arm pose as INFO
 * 
 * @param pose (input) Given arm pose
 */
void arm_pose_print(
    const ARM_POSE pose);

/**
 * @brief Apply homogeneous transformation
 * 
 * @param origin (input) Point to rotate
 * @param transform (input) Transformation to apply
 * @param output (output) Transformation point
 * @return ERROR_CODE 
 */
ERROR_CODE arm_homogeneous_transform(ARM_FRAME origin, ARM_FRAME transform, ARM_FRAME *output);
/**
 * @brief Direct kinematics of an arm
 * 
 * @param joints (input) Quaternion of each joints
 * @param output (output) Computed arm pose
 * @return ERROR_CODE 
 */
ERROR_CODE arm_direct_kinematics_compute(Quaternion joints[ARM_NUMBER_OF_JOINTS], ARM_POSE *output);

/**
 * @brief Apply an incremental rotation to current arm pose
 * 
 * @param sh2el_orientation (input) Rotation of upper arm segment
 * @param el2wr_orientation (input) Rotation of forearm segment
 * @return ARM_POSE: resulting position
 */
ARM_POSE arm_rotate(
    Quaternion sh2el_orientation,
    Quaternion el2wr_orientation);

/**
 * @brief Retrieve the current postion 
 * 
 * @return ARM_POSE: current position
 */
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


#endif