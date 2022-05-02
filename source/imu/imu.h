/**
 * @file imu.h
 * @author German Moreno Escudero
 * @brief LPMS IMU managing pack
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "errors.h"
#include "general.h"

#ifdef __GNUC__
#include "lpsensor/ImuData.h"
#endif

/**
 * @brief Conversion from Degrees to Radians
 */
#define DEG_2_RAD(x) (x * M_PI/180)

/**
 * @brief Maximum number of IMU sensors
 */
#define IMU_MAX_NUMBER (COM_PORTS_MAX_NUM)
/**
 * @brief Maximum timeout to wait for IMU connection in seconds
 */
#define IMU_CONNECTION_TIMEOUT (4)

// #define M_PI           3.14159265358979323846  /* pi */

typedef enum{
    IMUS_NOT_CONNECTED = 0,
    IMUS_CONNECTED,
    IMUS_READING
}IMUS_CONNECTION_STATUS;

typedef struct _IMUS_POSE{
    // Translation vector
    float t[3];
    // Quaternion vector
    float q[4];
    // Euler angles vector
    float r[3];
}IMUS_POSE;

typedef struct _ARM_JOINTS{
    float elbow_position[3];
    float wrist_position[3];
}ARM_JOINTS;

typedef struct _ARM_MEASUREMENT{
    float arm;
    float forearm;
}ARM_MEASUREMENT;

/******************************************************************************/
/*                            Declare functions                               */
/******************************************************************************/

/**
 * @brief Initialize an IMU sensor connected to a given COM port
 * 
 * @param com_port (input) The COM port where the IMU is connected
 * @return ERROR_CODE: RET_OK on succes, RET_ERROR on failure and RET_ARG_ERROR if the given arguments are invalid
 */
ERROR_CODE imu_initialize(const char *com_port);

/**
 * @brief Initialize a set of IMU sensors from a given set of COM ports
 * 
 * @param com_ports (input) The list of COM ports 
 * @param imus_num (input) How many IMU sensors to initialize
 * @return ERROR_CODE: RET_OK on succes, RET_ERROR on failure and RET_ARG_ERROR if the given arguments are invalid
 */
ERROR_CODE imu_batch_initialize(COM_PORTS com_ports, unsigned int imus_num);

/**
 * @brief Terminate all imu connections and handlers
 */
void imu_batch_terminate();

/**
 * @brief Set CSV logging file with IMU related headers
 */
void imu_csv_headers_set(void);

/**
 * @brief Log a given set of IMU data to a CSV file
 * 
 * @param d (input) Given IMU data
 */
void imu_csv_log(ImuData d);

/**
 * @brief Read data from an IMU sensor with a given index
 * 
 * @param index (input) Index of the IMU sensor to be read
 * @param imus (output) IMU data retrieved
 * @return ERROR_CODE: RET_OK on succes, RET_ERROR on failure and RET_ARG_ERROR if the given arguments are invalid
 */
ERROR_CODE imu_read(unsigned int index, ImuData *imus) ;

/**
 * @brief Read data from a set of IMU sensors
 * 
 * @param imus_num (input) The number of IMU sensors to be read starting from index 0
 * @param imus (output) IMU data retrieved
 * @return ERROR_CODE: RET_OK on succes, RET_ERROR on failure and RET_ARG_ERROR if the given arguments are invalid
 */
ERROR_CODE imu_batch_read(unsigned int imus_num, ImuData imus[]);

/**
 * @brief Print the IMU data through console
 * 
 * @param imu IMU data to be printed
 */
void imu_data_print (ImuData imu);






unsigned char setOffsetIMUs(int v);

void imus_direct_kinematics(ImuData (*imus), ARM_JOINTS (*data_query), bool twoImus, ARM_MEASUREMENT arm_params, bool show);

void printMatrix(float (Mat)[4][4]);

void DH_A(float a, float alpha, float d, float theta, float (*Mat)[4]);

void MultiplyMatrix4x4(float (*Mat)[4], float Mat1[4][4], float Mat2[4][4]);

#endif /*IMU_H*/

#ifdef __cplusplus
}
#endif
