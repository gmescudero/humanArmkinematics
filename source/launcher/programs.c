
#include "launch.h"
#include "database.h"
#include "imu.h"
#include "constants.h"

#define NO_DATA_MAX_ITERATIONS (10)

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

    /* Look for IMU sensors and initialize 2 of them */
    if (RET_OK == status) status = imu_batch_search_and_initialize(imus_num);
    if (RET_OK != status) err_str("Failed to setup IMU sensors");

    /* Measure noise values */
    if (0 < measureNoiseIterations) {
        log_str("Measuring IMUs in static position to gather noise data");
        log_str(" -> [USER]: Keep IMU sensors in a steady position");
        sleep_s(2);
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
    sleep_s(2);
    /* Set starting time */
    if (RET_OK == status) status = imu_batch_read(imus_num, data);
    if (RET_OK == status) startTime = data[0].timeStamp;
    if (RET_OK != status) err_str("Failed to read IMU sensors");

    /* Start IMU reading callbacks */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_attach(imu, 0==imu);
    }
    if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensors");

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
        if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
        if (RET_OK == status) currentTime = buffTime - startTime;
        if (RET_OK != status) err_str("Failed to read IMU sensors timestamp from database");

        dbg_str("Current time %f seconds out  of %f seconds",currentTime, time);

        /* Wait for next iteration */
        if (RET_OK == status) sleep_s(1);

    } while ((RET_OK == status || RET_NO_EXEC == status) && time > currentTime);
    log_str(" -> [USER]: Finished recording data");

    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}

#define IMUS_NUM (2)

ERROR_CODE hak_two_axes_auto_calib_and_kinematics(double time) {
    ERROR_CODE status = RET_OK;

    // TODO: make this function

    return status;
}


#if 0 // TODO: Create programs for other utilities
{
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
    status = hak_initialize();
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
        status += db_csv_field_add(DB_IMU_GYROSCOPE,0);
        status += db_csv_field_add(DB_IMU_GYROSCOPE,1);
        status += db_csv_field_add(DB_IMU_ACCELEROMETER,0);
        status += db_csv_field_add(DB_IMU_ACCELEROMETER,1);
        status += db_csv_field_add(DB_IMU_MAGNETOMETER,0);
        status += db_csv_field_add(DB_IMU_MAGNETOMETER,1);
        status += db_csv_field_add(DB_IMU_ANGULAR_VELOCITY,0);
        status += db_csv_field_add(DB_IMU_ANGULAR_VELOCITY,1);
        status += db_csv_field_add(DB_IMU_LINEAR_ACCELERATION,0);
        status += db_csv_field_add(DB_IMU_LINEAR_ACCELERATION,1);
        status += db_csv_field_add(DB_IMU_QUATERNION,0);
        status += db_csv_field_add(DB_IMU_QUATERNION,1);
        // status += db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
        // status += db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
        // status += db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
        // status += db_csv_field_add(DB_ARM_WRIST_POSITION,0);
        // status += db_csv_field_add(DB_CALIB_ROT_VECTOR,0);
        // status += db_csv_field_add(DB_CALIB_ROT_VECTOR,1);
        // status += db_csv_field_add(DB_CALIB_ERROR,0);
        // status += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,0);
        // status += db_csv_field_add(DB_CALIB_COST_DERIVATIVE,1);
        // status += db_csv_field_add(DB_CALIB_OMEGA,0);
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
        log_str("USER -> STAND IN T-POSE TO CALIBRATE");
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
        double min_vel = 0;
        for (int i = 0; RET_OK == status && i < IMUS_NUM; i++) {
            log_str("USER -> STAY STILL FOR A WHILE");
            IMU_NOISE_DATA imu_noise;
            status = imu_static_errors_measure(i,30000,&imu_noise);
            if (RET_OK == status) {
                double norm_var;
                status = vector3_norm(imu_noise.gyrVar, &norm_var);
                min_vel = MAX(min_vel, norm_var*norm_var);
            }
        }
        if (RET_OK == status) {
            min_vel *= 1.05; // Add a 5% threshold
            cal_min_velocity_set(min_vel); 
        }
        if (RET_OK == status) {
            log_str("USER -> CALIBRATION_DONE");
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
            // Reset status
            status = RET_OK;

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
                    status = cal_automatic_two_rotation_axes_calibrate(omega1, omega2, imus_quat[0], imus_quat[1], rotVector1, rotVector2);
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

    status = hak_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
#endif