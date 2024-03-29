/**
 * @file arm_kin.h
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

/******************************************************************************/
/* Defines                                                                    */
/******************************************************************************/

#define ARM_POSE_STRING_MAX_LENGTH (256)

#define ARM_NUMBER_OF_JOINTS (2)

/******************************************************************************/
/* Enumerates                                                                 */
/******************************************************************************/

enum ARM_KINEMATIC_NODE_ENUM {
    SHOULDER,
    ELBOW,
    WRIST,
    NUMBER_OF_NODES
};

enum ARM_ELBOW_ANGLES_ENUM {
    GAMMA_PS = 0,
    BETA_CARRYING,
    ALPHA_FE,
    ARM_ELBOW_ANGLES_NUMBER
};

enum ARM_SHOULDER_ANGLES_ENUM {
    SH_ROTATION = 0,
    SH_FLEXION,
    SH_ABDUCTION,
    ARM_SHOULDER_ANGLES_NUMBER
};

/******************************************************************************/
/* Types Definitions                                                          */
/******************************************************************************/

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

/**
 * @brief Set an specific arm pose
 * 
 * @param initial_arm_pose (input) Arm pose of each joint and link 
 */
void arm_pose_set(
    ARM_POSE initial_arm_pose);

/**
 * @brief Set the orientation of the arm and forearm body segments
 * 
 * @param q_arm (input) Quaternion orientation of the arm segment w.r.t. the global frame
 * @param q_forearm (input) Quaternion orientation of the forearm segment w.r.t. the global frame
 * @param q_wrist(input) Quaternion orientation of the wrist w.r.t. the global frame
 * @return ERROR_CODE 
 */
ARM_POSE arm_orientations_set(Quaternion q_arm, Quaternion q_forearm, Quaternion q_wrist);

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
 * @brief Inverse kinematis of an arm
 * 
 * @param upper_arm (input) Orientation of the Upper Arm
 * @param forearm (input) Orientation of the Forearm
 * @param joints (output) The joints chain
 * @return ERROR_CODE 
 */
ERROR_CODE arm_inverse_kinematics_compute(Quaternion upper_arm, Quaternion forearm, Quaternion joints[ARM_NUMBER_OF_JOINTS]);

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

/**
 * @brief Compute the quaternion to convert from 2 to 1
 * 
 * @param q1 (input) The orientation of the reference frame
 * @param q2 (input) The orientation of the second frame
 * @return Quaternion: The quaternion to convert from q2 to q1
 */
Quaternion arm_quaternion_between_two_get(Quaternion q1, Quaternion q2);

/**
 * @brief Set the zero point for the elbow angles
 * 
 * @param alphaR (input) Expected Flexion/Extension (FE) angle at current point
 * @param gammaR (input) Expected Pronation/Supination (PS) angle at current point
 * @param q_sensor1 (input) Orientation of the first imu
 * @param q_sensor2 (input) Orientation of the second imu
 * @param rotationV1 (input) Rotation vector of the FE axis
 * @param rotationV2 (input) Rotation vector of the PS axis
 * @return ERROR_CODE 
 */
ERROR_CODE arm_elbow_angles_zero(
    double alphaR, 
    double gammaR,     
    Quaternion q_sensor1, 
    Quaternion q_sensor2, 
    double rotationV1[3], 
    double rotationV2[3]);

/**
 * @brief Compute the elbow angles from quaternion orientations and rotation vecors
 * 
 * @param q_sensor1 (input) Orientation of the first imu
 * @param q_sensor2 (input) Orientation of the second imu
 * @param rotationV1 (input) Rotation vector of the FE axis
 * @param rotationV2 (input) Rotation vector of the PS axis
 * @param anglesFE_B_PS (output) Elbow angles in the next order: [Flexion/Extension, Carrying angle, Pronation/Supination]
 * @return ERROR_CODE 
 */
ERROR_CODE arm_elbow_angles_from_rotation_vectors_get(
    Quaternion q_sensor1, 
    Quaternion q_sensor2, 
    double rotationV1[3], 
    double rotationV2[3],
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER]);

/**
 * @brief Retrive shoulder angles from its quaternions representation
 * 
 * @param q_sensor1 (input) Orientation of the first imu
 * @param shoulderAngles (output) shoulder angles as [ROTATION, FLEXION/EXTENSION, ABDUCTION/ADDUCTION]
 */
ERROR_CODE arm_shoulder_angles_compute(
    Quaternion q_sensor1, double shoulderAngles[ARM_SHOULDER_ANGLES_NUMBER]);

#endif