/**
 * @file calib.h
 * @author German Moreno Escudero
 * @brief This module implements all the necessary functionality for calibrating IMUs in different ways
 * @version 0.1
 * @date 2022-11-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __calib_h__
#define __calib_h__

#include "errors.h"
#include "Quaternion.h"
#include "imu.h"

#define CALIB_MIN_VEL (5e1)

#define DEFAULT_ROT_AXIS_CALIB_WINDOW  (3)
#define DEFAULT_ROT_AXIS_CALIB_STEP_SZ (0.3)

#define CALIB_TWO_ROT_AXES_WINDOW (1000) // if sample time for IMUs is 50Hz this makes 10s window
#define CALIB_TWO_ROT_AXES_IMU_DATA_BUFF_SIZE (40)
#define CALIB_TWO_ROT_AXES_MAX_ITERATIONS (100)
#define CALIB_TWO_ROT_AXES_CORRECTION_ITERATIONS (5)

/**
 * @brief Adjust minimum velocity values to consider movement
 * 
 * @param min_vel (input) Velicity value to set 
 */
void cal_min_velocity_set(double min_vel);

/**
 * @brief Set the calibration data from a given set of IMU sensor readings
 * 
 * @param known_quat (input) Predefined known quaternion during calibration
 * @param im (input) Readings from IMU sensors
 * @param number_of_imus (input) Number of IMU sensors to calibrate
 */
void cal_static_imu_quat_calibration_set(
    Quaternion known_quat[IMU_MAX_NUMBER],
    Quaternion imus_quat[IMU_MAX_NUMBER]);

/**
 * @brief Apply calibration to raw quaternion data from IMUs
 * 
 * @param imus_quat (input) IMU raw quaternion data
 * @param index (input) IMU index
 * @param calibrated_data (output) Calibrated quaternion
 * @return ERROR_CODE 
 */
ERROR_CODE cal_static_imu_quat_calibration_apply(
    Quaternion imus_quat, int index, Quaternion *calibrated_data);
/**
 * @brief Retrieve the IMUs quaternion calibrated with the static value
 * 
 * @param calib_quat (output) resulting quaternion data
 * @return ERROR_CODE 
 */
ERROR_CODE cal_static_imu_quat_calibrated_data_get(Quaternion calib_quat[IMU_MAX_NUMBER]);
/**
 * @brief Get the calibrated flag
 * 
 * @param index (input) The imu index
 * @return true if calibrated
 * @return false if not calibrated
 */
bool cal_static_imu_quat_calibrated_flag_get(int index);
/**
 * @brief Set the current number of IMUs
 * 
 * @param imus_num (input) Number of imus
 */
void cal_static_imu_quat_number_of_imus_set(int imus_num);

/**
 * @brief Calibrate a rotation axis for a 1 DOF joint knowing its angular velocity
 * 
 * @param omega1_from1 (input) Current angular velocity in the first sensor
 * @param omega2_from2 (input) Current angular velocity in the second sensor
 * @param q_sensor1 (input) Quaternion orientation of the first sensor
 * @param q_sensor2 (input) Quaternion orientation of the second sensor
 * @param rotationV (input/output) Rotation vector
 * @return ERROR_CODE:
 *  - RET_OK on success
 *  - RET_ERROR otherwise
 */
ERROR_CODE cal_automatic_rotation_axis_calibrate(
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV[3]);



/**
 * @brief Initialize two rotation axes calibration resources. It requires to set up buffers for database field 
 *  including gyroscope and quaternion measures as well as relative angular velocity.
 * 
 * @param imu_data_buff_size (input) Size of the imu measures buffer
 * @param obs_data_buff_size (input) Size of the observations buffer
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_initialize(int imu_data_buff_size, int obs_data_buff_size);
/**
 * @brief Terminate the calibration package and destroy all used resources
 */
void cal_gn2_terminate();
/**
 * @brief Update database observations.
 * 
 * @param omega1_from1 (input) Angular velocity of the first IMU sensor in IMU ref. frame
 * @param omega2_from2 (input) Angular velocity of the second IMU sensor in IMU ref. frame
 * @param q_sensor1 (input) Orientation quaternnion of the first IMU sensor
 * @param q_sensor2 (input) Orientation quaternnion of the second IMU sensor
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_observations_update(double omega1_from1[3], double omega2_from2[3], Quaternion q_sensor1, Quaternion q_sensor2);
/**
 * @brief Update database observations from database values.
 * 
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_observations_from_database_update();
/**
 * @brief Compute total Root Mean Square value for Gauss-Newton
 * 
 * @param rotationV1 (input/output) First rotation vector
 * @param rotationV2 (input/output) Second rotation vector
 * @param error (output) Mean error of the new solution
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_root_mean_square(double rotationV1[3], double rotationV2[3], double *error);
/**
 * @brief Compute two rotation axes automatic calibration from arbitrary motion
 * 
 * @param rotationV1 (output) First rotation vector 
 * @param rotationV2 (output) Second rotation vector 
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_two_rot_axes_calib(double rotationV1[3], double rotationV2[3]);
/**
 * @brief Calibrate the orientation zero of the body segments 
 * 
 * @param rotationV1 (input) First rotation vector
 * @param rotationV2 (input) Second rotation vector
 * @param q_sensor1 (input) Arm sensor quaternion reading 
 * @param q_sensor2 (input) Forearm sensor quaternion reading
 * @param q1gb_expected (input) Arm orientation at current position
 * @param q2gb_expected (input) Forearm orientation at current postion
 * @param q1_zeroAndBody (output) Quaternion conversion from raw sensor reading to arm body segment
 * @param q2_zeroAndBody (output) Quaternion conversion from raw sensor reading to forearm body segment
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_zero_pose_calibrate(
    double rotationV1[3],
    double rotationV2[3], 
    Quaternion q_sensor1, 
    Quaternion q_sensor2,
    Quaternion q1gb_expected,
    Quaternion q2gb_expected,
    Quaternion *q1_zeroAndBody,
    Quaternion *q2_zeroAndBody);

/**
 * @brief Retrieve the IMU quaternions from database and apply calibration
 * 
 * @param q1 (output) Arm quaternion
 * @param q2 (output) Forearm quaternion
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_calibrated_orientations_from_database_get(Quaternion *q1, Quaternion *q2);

/**
 * @brief Retrieve the calibrated relative elbow quaternion and/or the elbow angles
 * 
 * @param q (output) Relative elbow quaternion
 * @param angles (output) Elbow angles as [PS,CARRYING,FE]
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_calibrated_relative_orientation_get(Quaternion *q, double angles[3]);

/**
 * @brief Perform the two rotation axes calibration from an already calibrated set of rotation vectors
 * 
 * @param rotationV1 (output) First rotation vector 
 * @param rotationV2 (output) Second rotation vector 
 * @return ERROR_CODE 
 */
ERROR_CODE cal_gn2_two_rot_axes_calib_correct(double rotationV1[3], double rotationV2[3]);

#endif /* __calib_h__ */