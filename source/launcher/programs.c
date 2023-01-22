/**
 * @file programs.c
 * @author German Moreno Escudero
 * @brief This file implements some high level programs using the HAK functionality
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "launch.h"
#include "database.h"
#include "imu.h"
#include "constants.h"
#include "calib.h"
#include "arm_kin.h"
#include "comms.h"
#include <string.h>

#define NO_DATA_MAX_ITERATIONS (10)


static char shak_ip[32]   = COM_DEFAULT_IP;
static unsigned shak_port = COM_DEFAULT_PORT;


void hak_network_setup(char *ip, unsigned port) {
    strcpy(shak_ip,ip);
    shak_port = port;
}


ERROR_CODE hak_record_imus_data(int imus_num, double time, int measureNoiseIterations) {
    ERROR_CODE status = RET_OK;
    IMU_NOISE_DATA noise;
    ImuData data[imus_num];
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    int hasNewData;
    int noDataIterations = NO_DATA_MAX_ITERATIONS;

    // Check arguments
    if (imus_num > IMU_MAX_NUMBER || imus_num <= 0) return RET_ARG_ERROR;
    if (time <= EPSI)                               return RET_ARG_ERROR;
    if (measureNoiseIterations < 0)                 return RET_ARG_ERROR;

    /* Set the csv logging from the database */
    log_str("Set the database fields to track into the csv");
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_TIMESTAMP,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_GYROSCOPE,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_ACCELEROMETER,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_MAGNETOMETER,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_ANGULAR_VELOCITY,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_LINEAR_ACCELERATION,imu);
        if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,imu);
    }
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (imus_num > imu_number_get()) {
        if (RET_OK == status) status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Measure noise values */
    if (0 < measureNoiseIterations) {
        log_str("Measuring IMUs in static position to gather noise data");
        log_str(" -> [USER]: Keep IMU sensors in a steady position");
        if (RET_OK == status) sleep_s(2);
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            if (RET_OK == status) status = imu_static_errors_measure(imu, measureNoiseIterations, &noise);
            if (RET_OK == status) {
                log_str("IMU %d noise data: \n"
                    "\t accelerometer: mean <%f,%f,%f> var <%f,%f,%f> \n"
                    "\t gyroscope:     mean <%f,%f,%f> var <%f,%f,%f> \n"
                    "\t magnetometer:  mean <%f,%f,%f> var <%f,%f,%f> \n",
                        imu, 
                        noise.accMean[0],noise.accMean[1],noise.accMean[2], noise.accVar[0], noise.accVar[1], noise.accVar[2],
                        noise.gyrMean[0],noise.gyrMean[1],noise.gyrMean[2], noise.gyrVar[0], noise.gyrVar[1], noise.gyrVar[2],
                        noise.magMean[0],noise.magMean[1],noise.magMean[2], noise.magVar[0], noise.magVar[1], noise.magVar[2]
                );
            }
        }
        log_str("Finished IMUs noise measuring");
    }

    /* Read IMUs data for the given amount of time */
    log_str("Starting loop gathering IMU sensors data");
    log_str(" -> [USER]: Starting recording for %f seconds",time);
    if (RET_OK == status) sleep_s(2);
    /* Set starting time */
    if (RET_OK == status) status = imu_batch_read(imus_num, data);
    if (RET_OK == status) startTime = data[0].timeStamp;
    if (RET_OK != status) err_str("Failed to read IMU sensors: ret: %d",status);

    /* Start IMU reading callbacks */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_attach(imu, 0==imu);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        else sleep_s(2);
    }

    do {
        /* Reset no execution buffer or trigger error if timeout is reached */
        if (RET_NO_EXEC == status) {
            if (0 <= noDataIterations--) {
                status = RET_ERROR;
                err_str("Unable to gather IMUs data. Reading timeout");
            }
            else {
                sleep_ms(100);
                status = RET_OK;
            }
        }
        else {
            noDataIterations = NO_DATA_MAX_ITERATIONS;
        }

        /* Check for available data in IMU sensors */
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            if (RET_OK == status) status = db_read(DB_IMU_NEW_DATA, imu, &hasNewData);
            if (RET_OK == status && (int)false == hasNewData) {
                wrn_str("Imu sensor %d data retrieving timeout", imu);
                status = RET_NO_EXEC;
            }
            /* Reset new data flag */
            if (RET_OK == status && (int)true == hasNewData) {
                hasNewData = (int)false;
                status = db_write(DB_IMU_NEW_DATA, imu, &hasNewData);
            }
        }

        /* Retrieve current timestamp from database */
        if (RET_OK == status) {
            status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime); 
            if (RET_OK != status) err_str("Failed to read IMU sensors timestamp from database");
        }
        if (RET_OK == status) currentTime = buffTime - startTime;

        dbg_str("Current time %f seconds out of %f seconds",currentTime, time);

        /* Wait for next iteration */
        if (RET_OK == status) sleep_s(1);

    } while ((RET_OK == status || RET_NO_EXEC == status) && time > currentTime);
    log_str(" -> [USER]: Finished recording data");

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}

ERROR_CODE hak_two_axes_auto_calib_and_kinematics(double time, bool computeShoulderAngles, bool computeElbowAngles) {
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    double rotationV1[3] = {0,0,1};
    double rotationV2[3] = {1,0,0};
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    double q_buff[4];
    Quaternion q1,q2;
    Quaternion q_zero = {.w=1,.v={0,0,0}};
    ARM_POSE pose;
    double anglesPS_B_FE[ARM_ELBOW_ANGLES_NUMBER];

    /* Set the csv logging from the database */
    log_str("Set the database fields to track into the csv");
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_TIMESTAMP,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_GYROSCOPE,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_GYROSCOPE,1);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,1);
    if (RET_OK == status) status = db_csv_field_add(DB_CALIB_OMEGA,0);
    if (RET_OK == status) status = db_csv_field_add(DB_CALIB_OMEGA_NORM,0);
    if (RET_OK == status) status = db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
    if (RET_OK == status) status = db_csv_field_add(DB_CALIB_ROT_VECTOR,1);
    if (RET_OK == status) status = db_csv_field_add(DB_CALIB_ERROR,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_WRIST_POSITION,0);
    if (RET_OK == status && computeShoulderAngles) status = db_csv_field_add(DB_ARM_SHOULDER_ANGLES,0);
    if (RET_OK == status && computeElbowAngles)    status = db_csv_field_add(DB_ARM_ELBOW_ANGLES,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Reset IMUs offset */
    if (RET_OK == status) {
        status = imu_orientation_offset_reset();
        if (RET_OK != status) err_str("Failed to reset IMU sensors orientation");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
        log_str("Starting loop gathering IMU sensors data to calibrate rotation axes");
        log_str(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination");

        /* Start IMU reading callbacks */
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            status = imu_read_callback_attach(imu, 0==imu);
            if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        }
        sleep_s(2);
    }

    /* Loop while gathering data for two axes calibration */
    if (RET_OK == status) {
        do {
            if (RET_OK == status) sleep_s(1);
            if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);
            log_str("Current observations count: %d/%d",db_field_buffer_current_size_get(DB_CALIB_OMEGA,0),CALIB_TWO_ROT_AXES_WINDOW);
        } while (RET_OK == status && CALIB_TWO_ROT_AXES_WINDOW > db_field_buffer_current_size_get(DB_CALIB_OMEGA,0));
        log_str(" -> [USER]: Finished recording calibration data");
    }

    /* Perform calibration algorithm */
    if (RET_OK == status) {
        log_str("Calibrating rotation two axes");

        status = cal_gn2_two_rot_axes_calib(rotationV1,rotationV2);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensors");
        
        log_str("Finished two axes calibration: ");
        log_str("\tRotation vector 1:[%f,%f,%f]", rotationV1[0],rotationV1[1], rotationV1[2]);
        log_str("\tRotation vector 2:[%f,%f,%f]", rotationV2[0],rotationV2[1], rotationV2[2]);
    }

    /* Set zero value for elbow angles */
    if (RET_OK == status) {
        log_str("Set the zero point");
        log_str(" -> [USER]: Stand in a pose to be considered as zero in 5");
        for (int i = 4; RET_OK == status && i >= 0; i--) {
            sleep_s(1);
            log_str(" -> [USER]: %d",i);
        }
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff,&q1);
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff,&q2);
        if (RET_OK == status) status = cal_gn2_zero_pose_calibrate(
            rotationV1,rotationV2, q1, q2, q_zero, q_zero, NULL, NULL);
        if (RET_OK == status) {
            log_str(" -> [USER]: Zero position set ");
            status = db_field_print(DB_ARM_WRIST_POSITION,0);
        }
        if (RET_OK != status) err_str("Failed to perform zero procedure");
    }

    /* Apply calibration functions to imu readings */
    if (RET_OK == status) {
        log_str("Starting loop printing arm angles");
        log_str(" -> [USER]: Starting recording for %f seconds",time);
        sleep_s(2);

        /* Set starting time */
        status = db_read(DB_IMU_TIMESTAMP,0,&startTime);

        /* Set inital error value */
        double calibration_error, current_error;
        double calibration_timer = 0.0;
        if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &calibration_error);

        do {
            /* Update observations buffer */
            if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);

            /* Update calibration error */
            if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &current_error);

            /* Correct calibration if the error is 10% worse or 5 senconds have passed */
            if (RET_OK == status && ((current_error > calibration_error*1.1) || (5 < calibration_timer))) {
                status = cal_gn2_two_rot_axes_calib_correct(rotationV1,rotationV2);
                if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &calibration_error);
                if (RET_OK == status) {
                    dbg_str("Online calibration correction performed: \n"
                        "\tTime since last calibration: %fs\n"
                        "\tError went from %f to %f",calibration_timer,current_error,calibration_error);
                    calibration_timer = 0.0;
                }
                else if (RET_NO_EXEC == status) {
                    status = RET_OK;
                }
                else {
                    err_str("Failed to perform online calibration");
                }
            }

            /* Compute the calibrated segment orientations */
            if (RET_OK == status) status = cal_gn2_calibrated_orientations_from_database_get(&q1,&q2);

            /* Compute arm positions */
            if (RET_OK == status) pose = arm_orientations_set(q1,q2,q2);

            /* Compute current shoulder angles */
            if (RET_OK == status && computeShoulderAngles) status = arm_shoulder_angles_compute(q1,NULL);

            /* Compute current elbow angles */
            // if (RET_OK == status && computeElbowAngles) status = arm_elbow_angles_from_rotation_vectors_get(
            //     q1, q2, rotationV1, rotationV2, anglesPS_B_FE);
            if (RET_OK == status) cal_gn2_calibrated_relative_orientation_get(NULL, anglesPS_B_FE);


            /* Retrieve current timestamp from database */
            if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
            if (RET_OK == status) {
                double previous_time = currentTime;
                currentTime = buffTime - startTime;
                calibration_timer += currentTime - previous_time;
            }
            
            /* Log data through terminal and log file */
            dbg_str("Current time %f seconds out of %f seconds",currentTime, time);
            if (RET_OK == status && computeShoulderAngles) status = db_field_print(DB_ARM_SHOULDER_ANGLES,0);
            if (RET_OK == status && computeElbowAngles)    status = db_field_print(DB_ARM_ELBOW_ANGLES,0);
            if (RET_OK == status) arm_pose_print(pose);

            /* Wait for next iteration */
            if (RET_OK == status) sleep_ms(500);

        } while (RET_OK == status && time > currentTime);
        if (RET_OK != status) err_str("Failed to apply calibration");
    }

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}

ERROR_CODE hak_static_calib_kinematics(double time, bool computeShoulderAngles)
{
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    ImuData data[2];
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    Quaternion known_quats[2] = { /* Quats for T-pose */
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
    };
    Quaternion read_quats[2];
    ARM_POSE pose;

    /* Set the csv logging from the database */
    log_str("Set the database fields to track into the csv");
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_TIMESTAMP,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,1);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_WRIST_POSITION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
    if (RET_OK == status && computeShoulderAngles) status = db_csv_field_add(DB_ARM_SHOULDER_ANGLES,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        if (RET_OK == status) status = imu_batch_search_and_initialize(imus_num);
        if (RET_RESOURCE_UNAVAILABLE == status) {
            imus_num = 1;
            wrn_str("Only found one IMU sensor");
            status = imu_batch_search_and_initialize(imus_num);
        }
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Reset IMUs offset */
    if (RET_OK == status) {
        status = imu_orientation_offset_reset();
        if (RET_OK != status) err_str("Failed to reset IMU sensors orientation");
    }

    /* Retrieve current IMU data */
    if (RET_OK == status) {
        log_str("Calibrate IMU sensors");
        log_str("USER -> STAND IN T-POSE TO CALIBRATE");
        log_str(" -> [USER]: Stand in T-pose to calibrate IMU orientations");
        if (RET_OK == status) sleep_s(4);
        if (RET_OK == status) status = imu_batch_read(imus_num, data);
        if (RET_OK != status) err_str("Failed to read IMUs data");
    }

    /* Set calibration data */
    if (RET_OK == status) {
        for (int imu = 0; imu < imus_num; imu++) {
            quaternion_from_float_buffer_build(data[imu].q, &read_quats[imu]);
        }
        if (1 == imus_num) 
            Quaternion_copy(&read_quats[0],&read_quats[1]);
        cal_static_imu_quat_calibration_set(known_quats, read_quats);
    }

    /* Start IMU reading callbacks */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_attach(imu, 0==imu);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        else sleep_s(2);
    }

    /* Apply calibration functions to imu readings */
    if (RET_OK == status) {
        log_str("Starting loop printing arm wrist position and recording kinematic data");
        log_str(" -> [USER]: Starting recording for %f seconds",time);
        sleep_s(2);

        /* Set starting time */
        if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP,0,&startTime);

        do {
            /* Get the current quaternions */
            if (RET_OK == status) status = cal_static_imu_quat_calibrated_data_get(read_quats);

            /* Mock the second quaternion if only one IMU is used */
            if (RET_OK == status && 1 == imus_num) 
                Quaternion_copy(&read_quats[0],&read_quats[1]);
            
            /* Set calibrated orientations */
            if (RET_OK == status) pose = arm_orientations_set(read_quats[0],read_quats[1],read_quats[1]);

            /* Get the shoudler angles */
            if (RET_OK == status && computeShoulderAngles) status = arm_shoulder_angles_compute(read_quats[0],NULL);

            /* Retrieve current timestamp from database */
            if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
            if (RET_OK == status) currentTime = buffTime - startTime;

            /* Log relevant data */
            dbg_str("Current time %f seconds out of %f seconds",currentTime, time);
            if (RET_OK == status && computeShoulderAngles) status = db_field_print(DB_ARM_SHOULDER_ANGLES,0);
            // if (RET_OK == status) status = db_field_print(DB_IMU_QUATERNION,0);
            if (RET_OK == status) arm_pose_print(pose);

            /* Wait for next iteration */
            if (RET_OK == status) sleep_ms(100);

        } while (RET_OK == status && time > currentTime);
        if (RET_OK != status) err_str("Failed to apply calibration");
    }

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}


ERROR_CODE hak_two_axes_auto_calib_and_kinematics_forever(bool shoulder, bool elbow, bool net) {
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    int observations = 0;
    double rotationV1[3] = {0,0,1};
    double rotationV2[3] = {1,0,0};
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    double q_buff[4];
    Quaternion q1,q2;
    Quaternion q_zero = {.w=1,.v={0,0,0}};
    ARM_POSE pose;
    double anglesPS_B_FE[ARM_ELBOW_ANGLES_NUMBER];

    /* Reset the csv logging from the database */
    db_csv_reset();

    /* Reduce log file tracing for long-term operations */
    // status = trace_level_set(INFO,INFO);

    /* Start socket client */
    if (RET_OK == status && true == net) status = com_client_initialize(shak_ip,shak_port,0);

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Reset IMUs offset */
    if (RET_OK == status) {
        status = imu_orientation_offset_reset();
        if (RET_OK != status) err_str("Failed to reset IMU sensors orientation");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
        log_str("Starting loop gathering IMU sensors data to calibrate rotation axes");
        log_str(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination");
        if (true == net)
            status = com_string_build_send(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination");

        /* Start IMU reading callbacks */
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            status = imu_read_callback_attach(imu, 0==imu);
            if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        }
        sleep_s(2);
    }

    /* Loop while gathering data for two axes calibration */
    if (RET_OK == status) {
        do {
            if (RET_OK == status) sleep_s(1);
            if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);
            observations = db_field_buffer_current_size_get(DB_CALIB_OMEGA,0);
            log_str("Current observations count: %d/%d",observations,CALIB_TWO_ROT_AXES_WINDOW);
        } while (RET_OK == status && CALIB_TWO_ROT_AXES_WINDOW > observations);
        
        log_str(" -> [USER]: Finished recording calibration data");
        if (true == net)
            status = com_string_build_send(" -> [USER]: Finished recording calibration data");
    }

    /* Perform calibration algorithm */
    if (RET_OK == status) {
        log_str("Calibrating rotation two axes");

        status = cal_gn2_two_rot_axes_calib(rotationV1,rotationV2);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensors");
        
        if (true == net)
            status = com_string_build_send("Finished two axes calibration: \n"
                "\tRotation vector 1:[%f,%f,%f]\n"
                "\tRotation vector 2:[%f,%f,%f]", 
                rotationV1[0],rotationV1[1], rotationV1[2],
                rotationV2[0],rotationV2[1], rotationV2[2]);
        else {
            log_str("Finished two axes calibration: ");
            log_str("\tRotation vector 1:[%f,%f,%f]", rotationV1[0],rotationV1[1], rotationV1[2]);
            log_str("\tRotation vector 2:[%f,%f,%f]", rotationV2[0],rotationV2[1], rotationV2[2]);
        }
    }

    /* Set zero value for elbow angles */
    if (RET_OK == status) {
        log_str("Set the zero point");
        log_str(" -> [USER]: Stand in a pose to be considered as zero in 5");
        if (true == net)
            status = com_string_build_send(" -> [USER]: Stand in a pose to be considered as zero in 5");
        for (int i = 4; RET_OK == status && i >= 0; i--) {
            sleep_s(1);
            log_str(" -> [USER]: %d",i);
        }
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff,&q1);
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff,&q2);
        if (RET_OK == status) status = cal_gn2_zero_pose_calibrate(
            rotationV1,rotationV2, q1, q2, q_zero, q_zero, NULL, NULL);
        if (RET_OK == status) {
            log_str(" -> [USER]: Zero position set ");        
            if (true == net)
                status = com_string_build_send(" -> [USER]: Zero position set ");
            status = db_field_print(DB_ARM_WRIST_POSITION,0);
        }
        if (RET_OK != status) err_str("Failed to perform zero procedure");
    }

    /* Apply calibration functions to imu readings */
    if (RET_OK == status) {
        log_str("Starting loop");
        sleep_s(2);

        /* Set starting time */
        status = db_read(DB_IMU_TIMESTAMP,0,&startTime);

        /* Set inital error value */
        double best_error, calibration_error, current_error;
        if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &calibration_error);
        best_error = calibration_error;

        do {
            /* Update observations buffer */
            if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);

            /* Update calibration error */
            if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &current_error);

            /* Correct calibration if the error is 5% worse */
            if (RET_OK == status && (current_error > calibration_error*1.05 || current_error > best_error*1.20)) {
                status = cal_gn2_two_rot_axes_calib_correct(rotationV1,rotationV2);
                if (RET_OK == status) status = cal_gn2_root_mean_square(rotationV1,rotationV2, &calibration_error);
                if (RET_OK == status) {
                    dbg_str("Online calibration correction performed: \n"
                        "\tError went from %f to %f",current_error,calibration_error);
                        current_error = calibration_error;
                }
                else if (RET_NO_EXEC == status) {
                    status = RET_OK;
                }
                else {
                    err_str("Failed to perform online calibration");
                }
            }

            /* Update the best error */
            if (RET_OK == status && current_error < best_error) best_error = current_error;

            /* Compute the calibrated segment orientations */
            if (RET_OK == status) status = cal_gn2_calibrated_orientations_from_database_get(&q1,&q2);

            /* Compute arm positions */
            if (RET_OK == status) pose = arm_orientations_set(q1,q2,q2);

            /* Compute current shoulder angles */
            if (RET_OK == status && shoulder) status = arm_shoulder_angles_compute(q1,NULL);

            /* Compute current elbow angles */
            if (RET_OK == status && elbow) cal_gn2_calibrated_relative_orientation_get(NULL, anglesPS_B_FE);

            /* Retrieve current timestamp from database */
            if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
            if (RET_OK == status) currentTime = buffTime - startTime;
            
            /* Log data through terminal, log file and network*/
            if (RET_OK == status) {
                if (true == net) {
                    ARM_POSE pose = arm_pose_get();
                    status = com_string_build_send(
                        "Current time:      %f\n"
                        "Wrist position:    %f,%f,%f\n"
                        "Wrist orientation: %f, %f,%f,%f",
                        currentTime,
                        pose.wrist.position[0],pose.wrist.position[1],pose.wrist.position[2],
                        pose.wrist.orientation.w,pose.wrist.orientation.v[0],pose.wrist.orientation.v[1],pose.wrist.orientation.v[2]);
                }
                else {
                    dbg_str("Current time %f",currentTime);
                    if (RET_OK == status && shoulder) status = db_field_print(DB_ARM_SHOULDER_ANGLES,0);
                    if (RET_OK == status && elbow)    status = db_field_print(DB_ARM_ELBOW_ANGLES,0);
                    if (RET_OK == status) arm_pose_print(pose);
                }
            }             

            /* Wait for next iteration */
            if (RET_OK == status) sleep_ms(100);

        } while (RET_OK == status);
        if (RET_OK != status) err_str("Failed to apply calibration");
    }

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}


ERROR_CODE hak_laidig2017() {
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    int observations = 0;
    double rotationV1[3] = {0,0,1};
    double rotationV2[3] = {1,0,0};
    Quaternion q1, q2;

    /* Reset the csv logging from the database */
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_TIMESTAMP,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ANGLES,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ANGLES,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_QUATERNION,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
        log_str("Starting loop gathering IMU sensors data to calibrate rotation axes");
        log_str(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination");
        
        /* Start IMU reading callbacks */
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            status = imu_read_callback_attach(imu, 0==imu);
            if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        }
        sleep_s(2);
    }

    /* Loop while gathering data for two axes calibration */
    if (RET_OK == status) {
        do {
            if (RET_OK == status) sleep_s(1);
            if (RET_OK == status) status = cal_gn2_observations_from_database_update(0.0);
            observations = db_field_buffer_current_size_get(DB_CALIB_OMEGA,0);
            log_str("Current observations count: %d/%d",observations,CALIB_TWO_ROT_AXES_WINDOW);
        } while (RET_OK == status && CALIB_TWO_ROT_AXES_WINDOW > observations);
        
        log_str(" -> [USER]: Finished recording calibration data");
    }
    
    /* Perform calibration algorithm */
    if (RET_OK == status) {
        log_str("Calibrating rotation two axes");

        status = cal_gn2_two_rot_axes_calib(rotationV1,rotationV2);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensors");
        else {
            log_str("Finished two axes calibration: ");
            log_str("\tRotation vector 1:[%f,%f,%f]", rotationV1[0],rotationV1[1], rotationV1[2]);
            log_str("\tRotation vector 2:[%f,%f,%f]", rotationV2[0],rotationV2[1], rotationV2[2]);
        }
    }

    /* Set the angles to zero in the current pose */
    if (RET_OK == status) {
        log_str("Set the zero point");
        log_str(" -> [USER]: Stand in a pose to be considered as zero in 5");
        for (int i = 4; RET_OK == status && i >= 0; i--) {
            sleep_s(1);
            log_str(" -> [USER]: %d",i);
            if (1 == i) status = imu_orientation_offset_set(1);
        }
        if (RET_OK == status) {
            // Get the current quaternions 
            double q_buff[4];
            if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
            if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q1);
            if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
            if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q2);   
        }
        if (RET_OK == status) status = arm_elbow_angles_zero(0.0,0.0,q1,q2,rotationV1,rotationV2);
        if (RET_OK == status) {
            log_str(" -> [USER]: Zero position set ");
            status = db_field_print(DB_ARM_ELBOW_ANGLES,0);
        }
        if (RET_OK != status) err_str("Failed to perform zero procedure");
    }

    /* Set starting time */
    double startTime    = sys_timestamp_get();
    double currentTime  = startTime;
    double previousTime = startTime;
    
    /* LOOP */
    while (RET_OK == status) {

        /* Get current sensor data from database */
        if (RET_OK == status) {
            // Get the current quaternions 
            double q_buff[4];
            if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
            if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q1);
            if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
            if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q2);   
            if (RET_OK != status) err_str("Failed to retrieve current sensor quaternions");
        }

        /* Compute current shoulder angles */
        if (RET_OK == status) {
            status = arm_shoulder_angles_compute(q1,NULL);
            if (RET_OK != status) err_str("Failed to compute shoulder angles");
        }

        /* Compute current elbow angles */
        if (RET_OK == status) {
            status = arm_elbow_angles_from_rotation_vectors_get(q1,q2,rotationV1,rotationV2,NULL);
            if (RET_OK != status) err_str("Failed to compute elbow angles");
        }

        /* Log data through console */
        if (RET_OK == status) log_str("Current time %.3f",1e-3*(currentTime-startTime));
        if (RET_OK == status) status = db_field_print(DB_ARM_SHOULDER_ANGLES,0);
        if (RET_OK == status) status = db_field_print(DB_ARM_ELBOW_ANGLES,0);
        if (RET_OK == status) status = db_field_print(DB_IMU_QUATERNION,0);
        if (RET_OK == status) status = db_field_print(DB_IMU_QUATERNION,1);

        /* If iteration not executed reset error code */
        if (RET_NO_EXEC == status) status = RET_OK;

        /* Wait for next iteration */
        if (RET_OK == status) {
            currentTime = sys_timestamp_get();
            sleep_ms((int)floor(100 - (currentTime - previousTime)));
            previousTime = sys_timestamp_get();
        }
    }

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}