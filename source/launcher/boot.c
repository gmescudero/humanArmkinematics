/**
 * @file boot.c
 * @author Germán Moreno Escudero
 * @brief This file is used to manage the initialization and cleaning of all pack resources
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "launch.h"
#include "general.h"
#include "calib.h"

ERROR_CODE hak_initialize() {
    ERROR_CODE status = RET_OK;
    
    log_str("Initializing all required packages for Human Arm Kinematics");
    
    if (RET_OK == status) status = log_file_initalize(); 
    if (RET_OK == status) status = db_initialize();
    if (RET_OK == status) status = cal_two_rot_axes_calib_initialize(CALIB_TWO_ROT_AXES_IMU_DATA_BUFF_SIZE, CALIB_TWO_ROT_AXES_WINDOW);

    return status;
}

ERROR_CODE hak_terminate() {
    ERROR_CODE status = RET_OK;

    log_str("Cleaning all resources used for Human Arm Kinematics");

    imu_terminate();
    status = db_terminate();
    if (RET_OK != status) wrn_str("Failed to clean all resources");

    return (RET_OK == status)? RET_OK:RET_ERROR;
}