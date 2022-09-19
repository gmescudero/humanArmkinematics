
#ifndef __calib_h__
#define __calib_h__

#include "errors.h"
#include "Quaternion.h"
#include "imu.h"

#define CALIB_MIN_VEL (5e1)

#define DEFAULT_ROT_AXIS_CALIB_WINDOW  (3)
#define DEFAULT_ROT_AXIS_CALIB_STEP_SZ (0.3)

#define CALIB_ONE_ROT_AXIS_WINDOW (10)
#define CALIB_ONE_ROT_AXIS_MAX_ITERATIONS (20)
#define CALIB_ONE_ROT_AXIS_MAX_ERROR (1e-1)
#define CALIB_ONE_ROT_AXIS_STEP_SZ (1)

#define CALIB_TWO_ROT_AXES_WINDOW (1000)
#define CALIB_TWO_ROT_AXES_IMU_DATA_BUFF_SIZE (40)
#define CALIB_TWO_ROT_AXES_MAX_ITERATIONS (200)

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
 * @param imus_quat (input) Readings from IMU sensors
 * @param number_of_imus (input) Number of IMU sensors to calibrate
 */
void cal_static_imu_quat_calibration_set(
    Quaternion known_quat[IMU_MAX_NUMBER],
    Quaternion imus_quat[IMU_MAX_NUMBER]);

/**
 * @brief Retrieve the IMUs quaternion calibrated with the static value
 * 
 * @param calib_quat (output) resulting quaternion data
 * @return ERROR_CODE 
 */
ERROR_CODE cal_static_imu_quat_calibrated_data_get(Quaternion calib_quat[IMU_MAX_NUMBER]);

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
 * @brief Calibrate two rotation axis for a 2 DOF joint knowing its angular velocities
 * 
 * @param omega1_from1 (input) Current angular velocity in the first sensor
 * @param omega2_from2 (input) Current angular velocity in the second sensor
 * @param q_sensor1 (input) Quaternion orientation of the first sensor
 * @param q_sensor2 (input) Quaternion orientation of the second sensor
 * @param rotationV1 (input/output) First rotation vector
 * @param rotationV2 (input/output) Second rotation vector
 * @return ERROR_CODE 
 */
ERROR_CODE cal_automatic_two_rotation_axes_calibrate(
    double omega1_from1[3],
    double omega2_from2[3],
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV1[3],
    double rotationV2[3]);




/**
 * @brief Calibrate two rotation axis for a 2 DOF joint knowing its angular velocities by using genetic algorithms
 * 
 * @param q_sensor1 (input) Quaternion orientation of the first sensor
 * @param q_sensor2 (input) Quaternion orientation of the second sensor
 * @param rotationV1 (input/output) First rotation vector
 * @param rotationV2 (input/output) Second rotation vector
 * @return ERROR_CODE 
 */
ERROR_CODE cal_automatic_two_rotation_axes_ga_calibrate(
    Quaternion q_sensor1,
    Quaternion q_sensor2,
    double rotationV1[3],
    double rotationV2[3]);




/**
 * @brief TODO: make descriptions and clear everything
 */
ERROR_CODE cal_two_rot_axes_calib_initialize(int imu_data_buff_size, int obs_data_buff_size);
void cal_two_rot_axes_calib_terminate();
ERROR_CODE cal_two_rot_axes_calib_observations_update(double omega1_from1[3], double omega2_from2[3], Quaternion q_sensor1, Quaternion q_sensor2);
ERROR_CODE cal_two_rot_axes_calib_observations_from_database_update();
ERROR_CODE cal_two_rot_axes_calib_compute(double rotationV1[3], double rotationV2[3]);



#endif /* __calib_h__ */