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
#include <signal.h>
#include <stdlib.h>

static void shak_interrupt_handler(int sig);

/**
 * @brief Interrupt handler used to correctly get out of a CTRL-C situation
 * 
 * @param sig (input) The signal that triggered the handler
 */
static void shak_interrupt_handler(int sig) {
    // log_str("Killed with CTRL-C (signal %d)",sig);
    imu_terminate();
    cal_gn2_terminate();
    db_terminate();
    exit(0);
}

ERROR_CODE hak_initialize() {
    ERROR_CODE status = RET_OK;
    
    log_str("Initializing all required packages for Human Arm Kinematics");
    
    struct sigaction act;
    act.sa_handler = shak_interrupt_handler;
    sigaction(SIGINT, &act, NULL);

    if (RET_OK == status) status = log_file_initalize(); 
    if (RET_OK == status) status = db_initialize();
    if (RET_OK == status) status = cal_gn2_initialize(CALIB_TWO_ROT_AXES_IMU_DATA_BUFF_SIZE, CALIB_TWO_ROT_AXES_WINDOW);

    return status;
}

ERROR_CODE hak_terminate() {
    ERROR_CODE status = RET_OK;

    log_str("Cleaning all resources used for Human Arm Kinematics");

    imu_terminate();
    cal_gn2_terminate();
    status = db_terminate();
    if (RET_OK != status) wrn_str("Failed to clean all resources");

    return (RET_OK == status)? RET_OK:RET_ERROR;
}