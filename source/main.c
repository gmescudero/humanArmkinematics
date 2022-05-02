
/**
 * @file main.c
 * @author German Moreno Escudero
 * @brief Main execution program 
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Quaternion.h"
#include "constants.h"
#include "arm.h"
#include "imu.h"
#include "errors.h"
#include "general.h"

#define STATUS_EVAL(code) {if (RET_OK != code) err_str("[%d] Failed: %d \n",__LINE__, code);}

static ERROR_CODE initialize() {
    ERROR_CODE status = RET_OK;
    ARM_POSE initialArmPose = {
        .shoulderPosition = {0.0, 0.0, 0.0},
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };
    
    // Initialize logging files
    status = log_file_initalize();

    if (RET_OK == status) {
        // Initialize Arm
        initializeArm(initialArmPose);
    }

    return status;
}

int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;
    COM_PORTS discoveredPorts;
    ImuData data;
    double startTime = -1.0;
    double currentTime = -1.0;
    // double csvBuff[CSV_FILE_VALUES_NUMBER] = {0.0};

    /* Initialize all packages */
    log_str("Initialize");
    status = initialize();
    STATUS_EVAL(status);

    /* Configure traces and csv */
    if (RET_OK == status) {
        log_str("Set CSV headers");
        imu_csv_headers_set();
        log_str("Set trace level");
        status = trace_level_set(INFO,DEBUG);
    }

    /* Look for com ports avalilable in the system */
    if (RET_OK == status) {
        log_str("Retrieve available COM ports");
        status = com_ports_list(&discoveredPorts);
        STATUS_EVAL(status);
    }

    /* Initialize IMU in a given COM port */
    if (RET_OK == status) {
        log_str("Initialize the IMU sensor in the first COM port");
        status = imu_initialize(discoveredPorts.ports_names[0]);
        STATUS_EVAL(status);
    }

    if (RET_OK == status) {
        log_str("Loop reading IMU data and logging it to the CSV file");
        do {
            millis_sleep(10);
            /* Read IMU data */
            if (RET_OK == status) {
                status = imu_read(0, &data);
            }
            if (RET_OK == status && -1.0 == startTime){
                startTime   = data.timeStamp;
            }
            if (RET_OK == status) {
                currentTime = data.timeStamp - startTime;
                imu_csv_log(data);
                dbg_str("Time: %f",currentTime);
            }
        } while ((RET_OK == status || RET_NO_EXEC == status) && 10 > currentTime);
        STATUS_EVAL(status);
    } 

    /* Terminate all imus */
    log_str("Terminate all IMU connections");
    if (RET_OK == status) {
        imu_batch_terminate();
    }
    return status;
}
