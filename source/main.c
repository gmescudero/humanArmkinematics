
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

/******************************/
/** CONFIG ********************/
/******************************/
#define USE_IMU_CALIB (0)           // Set whether to use the IMUs offset calibration or the integrated one (1 use imu, 0 use app)
#define USE_AUTO_CALIB (2)          // Set whether or not to use autocalibration of rotation axis (0 no autocalib, 1 one axis, 2 two axes)
#define IMUS_NUM (2)                // Set the expected minimum imu sensors for the program to work
#define UPPER_ARM_LENGTH (10.0)     // Set the Upper-arm length
#define FOREARM_LENGTH (5.0)        // Set the Forearm length
/******************************/

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
    Quaternion joints[ARM_NUMBER_OF_JOINTS];
    Quaternion imus_quat[IMUS_NUM];
    double buffTime;
    double startTime   = -1.0;
    double currentTime = -1.0;
    int iteration_count = 3000;

#if   1 <= USE_AUTO_CALIB
    double rotVector1[3] = {1.0,0.0,0.0};
#endif
#if   2 <= USE_AUTO_CALIB
    double rotVector2[3] = {0.0,0.0,1.0};
#endif

    /* Initialize all packages */
    log_str("Initialize");
    status = initialize();
    STATUS_EVAL(status);

    /* Set the current arm measures */
    if (RET_OK == status) {
        status = arm_segments_length_set(UPPER_ARM_LENGTH, FOREARM_LENGTH);
        STATUS_EVAL(status);
    }

    /* Set the csv logging from the database */
    if (RET_OK == status) {
        log_str("Set the database fields to track into the csv");
        status += db_csv_field_add(DB_IMU_TIMESTAMP,0);
        // status += db_csv_field_add(DB_IMU_TIMESTAMP,1);
        status += db_csv_field_add(DB_IMU_QUATERNION,0);
        status += db_csv_field_add(DB_IMU_QUATERNION,1);
        // status += db_csv_field_add(DB_IMU_ANGULAR_VELOCITY,0);
        // status += db_csv_field_add(DB_IMU_ANGULAR_VELOCITY,1);
        // status += db_csv_field_add(DB_IMU_GYROSCOPE,0);
        // status += db_csv_field_add(DB_IMU_GYROSCOPE,1);
        // status += db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
        // status += db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
        // status += db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
        // status += db_csv_field_add(DB_ARM_WRIST_POSITION,0);
        status += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
        status += db_csv_field_add(DB_CALIB_ROT_VECTOR,1);
        status += db_csv_field_add(DB_CALIB_ERROR,0);
        status += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
        status += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,1);
        status += db_csv_field_add(DB_CALIB_OMEGA,0);
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

    /* Set starting time */
    if (RET_OK == status) {
        status = imu_batch_read(IMUS_NUM, data);
        STATUS_EVAL(status);
        if (RET_OK == status) {
            startTime = data[0].timeStamp;
        }
    }

    /* Calibrate IMU quaternions */
    if (RET_OK == status) {
        log_str("Calibrate IMU sensors");
        log_str("STAND IN T-POSE TO CALIBRATE");
        sleep_ms(2000);
        if (RET_OK == status) {
#if USE_IMU_CALIB
            status = imu_orientation_offset_set(1);
            STATUS_EVAL(status);
#else
            Quaternion known_quats[IMUS_NUM] = {
                {.w = 1.0, .v={0.0, 0.0, 0.0}},
                {.w = 1.0, .v={0.0, 0.0, 0.0}},
            };
            Quaternion read_quats[IMUS_NUM];
            for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
                quaternion_from_float_buffer_build(data[i].q, &read_quats[i]);
            }
            cal_static_imu_quat_calibration_set(known_quats, read_quats);
#endif
        }
        if (RET_OK == status) {
            log_str("CALIBRATION_DONE");
        }
    }

    /* Start IMU reading callbacks */
    for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
        status = imu_read_callback_attach(i, 0==i);
        STATUS_EVAL(status);
    }

    if (RET_OK == status) {
        log_str("Start loop procedure");
        do {
            if (RET_NO_EXEC == status) {
                status = RET_OK;
            }
            if (RET_OK == status) {
                int hasNewData;
                sleep_ms(100);
                // Check there is new data from the IMU sensors and reset heartbeat
                dbg_str("Checking for imu data in database");
                for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
                    status = db_read(DB_IMU_NEW_DATA, i, &hasNewData);
                    STATUS_EVAL(status);
                    if (RET_OK == status && (int)false == hasNewData) {
                        wrn_str("Imu sensor %d data retrieving timeout", i);
                        status = RET_NO_EXEC;
                    }
                    if (RET_OK == status) {
                        hasNewData = (int)false;
                        status = db_write(DB_IMU_NEW_DATA, i, &hasNewData);
                        STATUS_EVAL(status);
                    }
                }
            }

            if (RET_OK == status) {
                // Update current time
                status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
                STATUS_EVAL(status);
                if (RET_OK == status) {
                    currentTime = buffTime - startTime;
                }
            }
#if USE_IMU_CALIB // Using imus simple calibration
            if (RET_OK == status) {
                // Get the calibrated quaternions from IMU data
                double quat_buff[4] = {0.0};
                for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
                    status = db_read(DB_IMU_QUATERNION, i, quat_buff);
                    STATUS_EVAL(status);
                    quaternion_from_buffer_build(quat_buff, &imus_quat[i]);
                }
            }
#else
            if (RET_OK == status) {
                // Get the calibrated quaternions from IMU data
                status = cal_static_imu_quat_calibrated_data_get(imus_quat);
                STATUS_EVAL(status);
            }
#endif
            if (RET_OK == status) {
                // Compute each joint value
                status = arm_inverse_kinematics_compute(imus_quat[0], imus_quat[1], joints); 
                STATUS_EVAL(status);
            }
            if (RET_OK == status) {
                // Compute the positions
                status = arm_direct_kinematics_compute(joints, NULL);
                STATUS_EVAL(status);
            }
#if 0 != USE_AUTO_CALIB // Using auto-calibration
            if (RET_OK == status) {
                // Calibration of rotation axes
                double omega1[3], omega2[3];
                status = db_read(DB_IMU_GYROSCOPE,0,omega1);
                STATUS_EVAL(status);
                if (RET_OK == status) {
                    status = db_read(DB_IMU_GYROSCOPE,1,omega2);
                    STATUS_EVAL(status);
                }
                if (RET_OK == status) {
    #if   1 == USE_AUTO_CALIB
                    status = cal_automatic_rotation_axis_calibrate(omega1, omega2, imus_quat[0], imus_quat[1], rotVector1);
    #elif 2 == USE_AUTO_CALIB
                    status = cal_automatic_two_rotation_axis_calibrate(omega1, omega2, imus_quat[0], imus_quat[1], rotVector1, rotVector2);
    #endif
                    STATUS_EVAL(status);
                }
            }
#endif
            if (RET_OK == status) {
                // Print the wrist position through console
                status = db_field_print(DB_ARM_WRIST_POSITION, 0);
                STATUS_EVAL(status);
            }
            dbg_str("time: %f",currentTime);
            iteration_count--;
        } while ((RET_OK == status || RET_NO_EXEC == status) && 30 > currentTime && 0 < iteration_count);
    } 

#if   1 <= USE_AUTO_CALIB
    log_str("Rotation vector 1:[%f,%f,%f]", rotVector1[0],rotVector1[1], rotVector1[2]);
#endif
#if   2 <= USE_AUTO_CALIB
    log_str("Rotation vector 2:[%f,%f,%f]", rotVector2[0],rotVector2[1], rotVector2[2]);
#endif

    log_str("Terminate all IMU connections");
    imu_terminate();

    log_str("Clean memory and environment");
    status += db_terminate();

    log_str("Final return code: %d", status);
    return (RET_OK == status)? RET_OK:RET_ERROR;
}
