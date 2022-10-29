/**
 * @file imu_config.hpp
 * @author German Moreno Escudero
 * @brief LPMS IMU configuration
 * @version 0.1
 * @date 2022-10-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <thread>
#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
// #define IMU_CONNECTION_PORT "COM0"
#endif

#ifdef __GNUC__
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
// #define IMU_CONNECTION_PORT "/dev/ttyUSB0";
#endif

using namespace std;
#define WAIT(ms) (this_thread::sleep_for(chrono::milliseconds(ms)))


/**
 * @brief Set sensor ID
 * 
 * @details The available choices are:
 *      - id >= 0
 * 
 * @param id (input) IMU id to be set. The available choices are:
 *      - id >= 0
 * @param lpms (input) IMU sensor handler
 */
void imu_set_SensorID(int id, LpmsSensorI *lpms);
/**
 * @brief Set sensor filter mode
 * 
 * @param mode (input) IMU filter mode. The available choices are:
 *     - SELECT_FM_GYRO_ONLY         0
 *     - SELECT_FM_GYRO_ACC          1
 *     - SELECT_FM_GYRO_ACC_MAG      2
 *     - SELECT_FM_MADGWICK_GYRO_ACC 3
 *     - SELECT_FM_MADGWICK_GYRO_ACC_MAG 4
 * @param lpms (input) IMU sensor handler
 */
void imu_set_FilterMode(int mode, LpmsSensorI *lpms);
/**
 * @brief Set the range of the gyroscope
 * 
 * @param range (input) IMU gyroscope range. The available choices are:
 *     - SELECT_GYR_RANGE_125DPS     125
 *     - SELECT_GYR_RANGE_245DPS     245
 *     - SELECT_GYR_RANGE_250DPS     250
 *     - SELECT_GYR_RANGE_500DPS     500 
 *     - SELECT_GYR_RANGE_1000DPS    1000 
 *     - SELECT_GYR_RANGE_2000DPS    2000
 * @param lpms (input) IMU sensor handler
 */
void imu_set_GyroRange(int range, LpmsSensorI *lpms);
/**
 * @brief Set the range of the accelerometer
 * 
 * @param range (input) IMU accelerometer range. The available choices are:
 *     - SELECT_ACC_RANGE_2G     2
 *     - SELECT_ACC_RANGE_4G     4
 *     - SELECT_ACC_RANGE_8G     8
 *     - SELECT_ACC_RANGE_16G    16
 * @param lpms (input) IMU sensor handler
 */
void imu_set_AccRange(int range, LpmsSensorI *lpms);
/**
 * @brief Set the range of the accelerometer
 * 
 * @param range (input) IMU accelerometer range. The available choices are:
 *     - SELECT_MAG_RANGE_4GAUSS     4
 *     - SELECT_MAG_RANGE_8GAUSS     8
 *     - SELECT_MAG_RANGE_12GAUSS    12
 *     - SELECT_MAG_RANGE_16GAUSS    16
 * @param lpms (input) IMU sensor handler
 */
void imu_set_MagRange(int range, LpmsSensorI *lpms);
/**
 * @brief Set the sensor sampling rate in Hz
 * 
 * @param rate (input) IMU polling rate. The available choices are:
 *     - SELECT_STREAM_FREQ_5HZ          5
 *     - SELECT_STREAM_FREQ_10HZ         10
 *     - SELECT_STREAM_FREQ_25HZ         25
 *     - SELECT_STREAM_FREQ_50HZ         50
 *     - SELECT_STREAM_FREQ_100HZ        100
 *     - SELECT_STREAM_FREQ_200HZ        200
 *     - SELECT_STREAM_FREQ_400HZ        400
 *     - SELECT_STREAM_FREQ_800HZ        800
 * @param lpms (input) IMU sensor handler
 */
void imu_set_SamplingRate(int rate, LpmsSensorI *lpms);
/**
 * @brief Set the sensor outputs to enable
 * 
 * @param data (input) Enable outputs bit word. The available choices are:
 *     - SELECT_LPMS_QUAT_OUTPUT_ENABLED                 0x1
 *     - SELECT_LPMS_EULER_OUTPUT_ENABLED                (0x1 << 1)
 *     - SELECT_LPMS_LINACC_OUTPUT_ENABLED               (0x1 << 2)
 *     - SELECT_LPMS_PRESSURE_OUTPUT_ENABLED             (0x1 << 3)
 *     - SELECT_LPMS_GYRO_OUTPUT_ENABLED                 (0x1 << 4)
 *     - SELECT_LPMS_ACC_OUTPUT_ENABLED                  (0x1 << 5)
 *     - SELECT_LPMS_MAG_OUTPUT_ENABLED                  (0x1 << 6)
 *     - SELECT_LPMS_GYRO_TEMP_OUTPUT_ENABLED            (0x1 << 7)
 *     - SELECT_LPMS_TEMPERATURE_OUTPUT_ENABLED          (0x1 << 8)
 *     - SELECT_LPMS_ALTITUDE_OUTPUT_ENABLED             (0x1 << 9)
 *     - SELECT_LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED     (0x1 << 10)
 * @param lpms (input) IMU sensor handler
 */
void imu_set_OutputData(int data, LpmsSensorI *lpms);
/**
 * @brief Set the magnetic correction mode
 * 
 * @param mode (input) IMU magnetometer correction mode. The available choices are:
 *     - SELECT_IMU_SLOW     0
 *     - SELECT_IMU_MEDIUM   1
 *     - SELECT_IMU_FAST     2
 *     - SELECT_IMU_DYNAMIC  3   
 * @param lpms (input) IMU sensor handler
 */
void imu_set_MagneticCorrection(int mode, LpmsSensorI *lpms);
/**
 * @brief Set the linear acceleration compensation mode
 * 
 * @param mode (input) IMU linear acceleration compensation mode. The available choices are:
 *     - SELECT_LPMS_LIN_ACC_COMP_MODE_OFF       0
 *     - SELECT_LPMS_LIN_ACC_COMP_MODE_WEAK      1
 *     - SELECT_LPMS_LIN_ACC_COMP_MODE_MEDIUM    2
 *     - SELECT_LPMS_LIN_ACC_COMP_MODE_STRONG    3
 *     - SELECT_LPMS_LIN_ACC_COMP_MODE_ULTRA     4
 * @param lpms (input) IMU sensor handler
 */
void imu_set_LinAccCompensationMode(int mode, LpmsSensorI *lpms);
/**
 * @brief Set the rotational acceleration compensation mode
 * 
 * @param mode (input) IMU rotational acceleration compensation mode. The available choices are:
 *     - SELECT_LPMS_CENTRI_COMP_MODE_OFF        0
 *     - SELECT_LPMS_CENTRI_COMP_MODE_ON         1
 * @param lpms (input) IMU sensor handler
 */
void imu_set_RotationalAccCompensation(int mode, LpmsSensorI *lpms);
/**
 * @brief Select sensor bus mode
 * 
 * @param mode (input) IMU bus mode. The available choices are:
 *     - SELECT_LPMS_LPBUS_DATA_MODE_32          0
 *     - SELECT_LPMS_LPBUS_DATA_MODE_16          1
 * @param lpms (input) IMU sensor handler
 */
void imu_set_LpBusMode(int mode, LpmsSensorI *lpms);