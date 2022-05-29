
#ifndef __calib_h__
#define __calib_h__

#include "errors.h"
#include "Quaternion.h"
#include "imu.h"


#define DEFAULT_ROT_AXIS_CALIB_WINDOW  (3)
#define DEFAULT_ROT_AXIS_CALIB_STEP_SZ (0.3)
#define DEFAULT_ROT_AXIS_CALIB_MIN_VEL (3e-1)


/**
 * @brief Set the calibration data from a given set of IMU sensor readings
 * 
 * @param known_quat (input) Predefined known quaternion during calibration
 * @param imus_quat (input) Readings from IMU sensors
 * @param number_of_imus (input) Number of IMU sensors to calibrate
 */
void cal_static_imu_quat_calibration_set(
    Quaternion known_quat[IMU_MAX_NUMBER],
    Quaternion imus_quat[IMU_MAX_NUMBER],
    int number_of_imus);
/**
 * @brief Apply the calibration to a new IMU sensors reading
 * 
 * @param imus_quat (input) Readings from IMU sensors
 * @param number_of_imus (input) Number of IMU sensors to calibrate
 * @param calibrated_data (output) resulting quaternion data
 * @return ERROR_CODE 
 */
ERROR_CODE cal_static_imu_quat_calibration_apply(
    Quaternion imus_quat[IMU_MAX_NUMBER],
    int number_of_imus,
    Quaternion calibrated_data[IMU_MAX_NUMBER]);

/**
 * @brief Calibrate a rotation axis for a 1 DOF joint knowing its angular velocity
 * 
 * @param omegaR (input) Current angular velocity
 * @param rotationV (input/output) Rotation vector
 * @return ERROR_CODE:
 *  - RET_OK on success
 *  - RET_ERROR otherwise
 */
ERROR_CODE cal_automatic_rotation_axis_calibrate(
    double omegaR[3],
    double rotationV[3]);

#endif /* __calib_h__ */