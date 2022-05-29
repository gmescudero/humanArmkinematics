
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
#include "calib.h"
#include <string.h>

#define IMUS_NUM (2) // Set the expected minimum imu sensors for the program to work

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
    COM_PORTS discoveredPorts;
    ImuData data[IMUS_NUM];
    Quaternion predefined_quats[IMUS_NUM];
    Quaternion calibrated_quats[IMUS_NUM];
    Quaternion joints[ARM_NUMBER_OF_JOINTS];
    double startTime   = -1.0;
    double currentTime = -1.0;
    double lastTime    = -1.0;
    // double rotVector[3] = {0.5,0.5,0.5};

    /* Initialize all packages */
    log_str("Initialize");
    status = initialize();
    STATUS_EVAL(status);

    /* Set the csv logging from the database */
    if (RET_OK == status) {
        log_str("Set the database fields to track into the csv");
        status += db_csv_field_add(DB_IMU_TIMESTAMP,0);
        // status += db_csv_field_add(DB_IMU_QUATERNION,0);
        // status += db_csv_field_add(DB_IMU_TIMESTAMP,1);
        // status += db_csv_field_add(DB_IMU_QUATERNION,1);
        status += db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
        status += db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
        status += db_csv_field_add(DB_ARM_WRIST_POSITION,0);
        status += db_csv_field_add(DB_ARM_WRIST_ORIENTATION,0);
        STATUS_EVAL(status);
    }

    /* Look for COM ports avalilable in the system */
    if (RET_OK == status) {
        log_str("Retrieve available COM ports");
        status = com_ports_list(&discoveredPorts);
        STATUS_EVAL(status);
        if (RET_OK == status && discoveredPorts.ports_number < IMUS_NUM) {
            err_str("Failed to initialize the 2 required IMU sensors");
            status = RET_ERROR;
        }
    }

    /* Initialize IMU in a given COM port */
    if (RET_OK == status) {
        log_str("Initialize the IMU sensors");
        // status = imu_initialize(discoveredPorts.ports_names[0]);
        status = imu_batch_initialize(discoveredPorts, discoveredPorts.ports_number);
        STATUS_EVAL(status);
    }

    /* Calibrate IMU quaternions */
    if (RET_OK == status) {
        log_str("Calibrate IMU sensors");
        log_str("STAND IN T-POSE TO CALIBRATE");
        sleep_ms(2000);
        if (RET_OK == status) {
            status = imu_batch_read(IMUS_NUM, data);
            STATUS_EVAL(status);
        }
        if (RET_OK == status) {
            for (int i = 0; i < IMUS_NUM; i++) {
                Quaternion_set(1.0, 0.0, 0.0, 0.0, &predefined_quats[i]);
            }
            cal_static_imu_quat_calibration_set(predefined_quats, data, IMUS_NUM);
        }
        log_str("CALIBRATION_DONE");
    }

    if (RET_OK == status) {
        log_str("Loop reading IMU data to calibrate ");
        do {
            sleep_ms(50);
            if (RET_OK == status) {
                // Read IMU data
                // status = imu_read(0, &data);
                status = imu_batch_read(IMUS_NUM, data);
                STATUS_EVAL(status);
            }
            if (RET_OK == status && -1.0 == startTime){
                // Initialize start time if required
                startTime   = data[0].timeStamp;
            }
            if (RET_OK == status) {
                // Update current time
                lastTime    = currentTime;
                currentTime = data[0].timeStamp - startTime;
            }
            if (RET_OK == status) {
                // Get the calibrated quaternions from IMU data
                status = cal_static_imu_quat_calibration_apply(data, IMUS_NUM, calibrated_quats);
                STATUS_EVAL(status);
            }
            if (RET_OK == status) {
                // Compute each joint value
                Quaternion sh_joint_conj;
                Quaternion_copy(&calibrated_quats[0], &joints[SHOULDER]);
                Quaternion_conjugate(&joints[SHOULDER], &sh_joint_conj);
                Quaternion_multiply(&sh_joint_conj, &calibrated_quats[1], &joints[ELBOW]);
            }
            if (RET_OK == status) {
                // Compute the positions
                status = arm_direct_kinematics_compute(joints, NULL);
                STATUS_EVAL(status);
            }

            if (RET_OK == status) {
                // Print the wrist position through console
                status = db_field_print(DB_ARM_WRIST_POSITION, 0);
                STATUS_EVAL(status);
            }
            if (RET_OK == status) {
                // Write csv file
                status = db_csv_dump();
                STATUS_EVAL(status);
            }
            // dbg_str("time: %f (oldTime: %f)",currentTime,lastTime);
        } while ((RET_OK == status || RET_NO_EXEC == status) && 50 > currentTime && currentTime - lastTime > EPSI);
    } 

    // log_str("Rotation vector obtained: [%f, %f, %f]", rotVector[0], rotVector[1], rotVector[2]);

    log_str("Terminate all IMU connections");
    imu_terminate();

    log_str("Clean memory and environment");
    status += db_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
