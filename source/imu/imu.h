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
#include "database.h"

#ifdef __GNUC__
#include "lpsensor/ImuData.h"
#endif

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

typedef struct _IMU_NOISE_DATA{
    double accMean[3];
    double accVar[3];
    double gyrMean[3];
    double gyrVar[3];
    double magMean[3];
    double magVar[3];
} IMU_NOISE_DATA;

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

typedef void(*IMU_READ_CALLBACK)(ImuData d, const char* id);

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
 * @brief Search COM ports and Initialize a set of IMU sensors from a given set of COM ports
 * 
 * @param imus_num (input) How many IMU sensors to initialize
 * @return ERROR_CODE: RET_OK on succes, RET_ERROR on failure and RET_ARG_ERROR if the given arguments are invalid
 */
ERROR_CODE imu_batch_search_and_initialize(unsigned int imus_num);

/**
 * @brief Remove initialized sensors from the handler
 */
void imu_all_sensors_remove();
/**
 * @brief Terminate all imu connections and handlers
 */
void imu_terminate();

/**
 * @brief Return total number of initialized IMU sensors
 * 
 * @return int: number of IMU sensors
 */
int imu_number_get();

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
 * @brief Attach the read callback to an IMU sensor
 * 
 * @param index (input) IMU sensor index to attach callback to
 * @param csv_dump (input) Set the read callback to dump the csv or not
 * @return ERROR_CODE 
 */
ERROR_CODE imu_read_callback_attach(unsigned int index, bool csv_dump);
/**
 * @brief Detach a read callback from an IMU sensor
 * 
 * @param index (input) IMU sensor index to detach callback from
 * @return ERROR_CODE 
 */
ERROR_CODE imu_read_callback_detach(unsigned int index);
/**
 * @brief Print the IMU data through console
 * 
 * @param imu IMU data to be printed
 */
void imu_data_print (ImuData imu);

/**
 * @brief Take measures from the IMU in a static position to measure the mean and variance of the IMU data
 * 
 * @param index (input) IMU index to read
 * @param iterations (input) Number of iterations to use for noise calculations
 * @param noise 
 * @return ERROR_CODE 
 */
ERROR_CODE imu_static_errors_measure(unsigned int index, int iterations, IMU_NOISE_DATA *noise);




ERROR_CODE imu_orientation_offset_set(int v);
ERROR_CODE imu_orientation_offset_reset();

#endif /*IMU_H*/

#ifdef __cplusplus
}
#endif
