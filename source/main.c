
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
#include "database.h"
#include <string.h>

#define STATUS_EVAL(code) {if (RET_OK != code && RET_NO_EXEC != status) err_str("[%d] Failed: %d ",__LINE__, code);}

static ERROR_CODE initialize() {
    ERROR_CODE status = RET_OK;
    
    status = log_file_initalize();
    STATUS_EVAL(status);

    if (RET_OK == status) {
        status = db_initialize();
        STATUS_EVAL(status);
    }

    return status;
}

int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;
    DB_FIELD_IDENTIFIER fields_monitored[] = {
        DB_IMU_TIMESTAMP,
        DB_CALIB_OMEGA,
        DB_CALIB_OMEGA_NORM,
        DB_CALIB_ERROR,
        DB_CALIB_ROT_VECTOR,
        DB_CALIB_SPHERICAL_COORDS,
        DB_CALIB_COST_DERIVATIVE
    };
    int num_fields = sizeof(fields_monitored)/sizeof(DB_FIELD_IDENTIFIER);
    COM_PORTS discoveredPorts;
    ImuData data;
    double startTime   = -1.0;
    double currentTime = -1.0;
    double rotVector[3] = {0.5,0.5,0.5};

    /* Initialize all packages */
    log_str("Initialize");
    status = initialize();
    STATUS_EVAL(status);

    /* Set the csv logging from the database */
    if (RET_OK == status) {
        log_str("Set the database fields to track into the csv");
        for (int i = 0; i<num_fields && RET_OK==status; i++) {
            status = db_csv_field_add(fields_monitored[i],0);
        }
        STATUS_EVAL(status);
    }

    /* Look for COM ports avalilable in the system */
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
        log_str("Loop reading IMU data to calibrate ");
        do {
            millis_sleep(20);
            if (RET_OK == status) {
                // Read IMU data
                status = imu_read(0, &data);
                STATUS_EVAL(status);
            }
            if (RET_OK == status && -1.0 == startTime){
                // Initialize start time if required
                startTime   = data.timeStamp;
            }
            if (RET_OK == status) {
                // Update current time and log to the CSV
                currentTime = data.timeStamp - startTime;
            }
            if (RET_OK == status) {
                // Execute the calibration 
                double gyr[] = {(float)data.g[0], (float)data.g[1], (float)data.g[2]};
                status = arm_calibrate_rotation_axis(gyr,rotVector);
                STATUS_EVAL(status);
            }
            if (RET_OK == status) {
                // Write csv file
                status = db_csv_dump();
                STATUS_EVAL(status);
            }

        } while ((RET_OK == status || RET_NO_EXEC == status) && 20 > currentTime);
    } 

    log_str("Rotation vector obtained: [%f, %f, %f]", rotVector[0], rotVector[1], rotVector[2]);

    log_str("Terminate all IMU connections");
    imu_batch_terminate();

    log_str("Clean memory and environment");
    status += db_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
