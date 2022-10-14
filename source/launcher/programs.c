
#include "launch.h"
#include "database.h"
#include "imu.h"
#include "constants.h"
#include "calib.h"
#include "arm.h"

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

ERROR_CODE hak_two_axes_auto_calib_and_elbow_angles(double time) {
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    double rotationV1[3] = {0,0,1};
    double rotationV2[3] = {1,0,0};
    double rotationV2_2[3]; // rotationV2 in coordinates of sensor 2
    double q_buff[4];
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    Quaternion q1,q2;
    Quaternion q12;
    double anglesFE_B_PS[ARM_ELBOW_ANGLES_NUMBER];

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
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ANGLES,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_QUATERNION,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Reset IMUs offset */
    if (RET_OK == status) {
        status = imu_orientation_offset_reset();
        if (RET_OK == status) status = imu_orientation_offset_set(1);
        if (RET_OK != status) err_str("Failed to reset IMU sensors orientation");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
        log_str("Starting loop gathering IMU sensors data to calibrate rotation axes");
        log_str(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination for %f seconds",time);

        /* Start IMU reading callbacks */
        for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
            status = imu_read_callback_attach(imu, 0==imu);
            if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        }
        sleep_s(2);
    }

    /* Loop while data keeps beeing gathered */
    do {
        if (RET_OK == status) sleep_s(1);
        if (RET_OK == status) status = cal_gn2_observations_from_database_update();
        log_str("Current observations count: %d/%d",db_field_buffer_current_size_get(DB_CALIB_OMEGA,0),CALIB_TWO_ROT_AXES_WINDOW);
    } while (RET_OK == status && CALIB_TWO_ROT_AXES_WINDOW > db_field_buffer_current_size_get(DB_CALIB_OMEGA,0));
    log_str(" -> [USER]: Finished recording calibration data");

    /* Perform calibration algorithm */
    log_str("Calibrating rotation two axes");
    if (RET_OK == status) {
        status = cal_gn2_two_rot_axes_calib(rotationV1,rotationV2);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensors");
    }
    log_str("Finished two axes calibration: ");
    log_str("\tRotation vector 1:[%f,%f,%f]", rotationV1[0],rotationV1[1], rotationV1[2]);
    log_str("\tRotation vector 2:[%f,%f,%f]", rotationV2[0],rotationV2[1], rotationV2[2]);

    /* Set zero value for elbow angles */
    if (RET_OK == status) {
        log_str("Set the zero point");
        log_str(" -> [USER]: Stand in a pose to be considered as zero for the elbow angles in 5");
        for (int i = 4; RET_OK == status && i >= 0; i--) {
            sleep_s(1);
            log_str(" -> [USER]: %d",i);
        }
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q1);
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q2);
        if (RET_OK == status) q12 = arm_quaternion_between_two_get(q2,q1);
        if (RET_OK == status) Quaternion_rotate(&q12,rotationV2,rotationV2_2);
        if (RET_OK == status) status = arm_elbow_angles_zero(0.0, 0.0, q1, q12, rotationV1, rotationV2_2);
        if (RET_OK == status) {
            log_str(" -> [USER]: Elbow angles zero position set ");
            status = db_field_print(DB_ARM_ELBOW_ANGLES,0);
        }
        else {
            err_str("Failed to perform zero procedure");
        }
    }

    /* Apply calibration functions to imu readings */
    log_str("Starting loop printing arm angles");
    log_str(" -> [USER]: Starting recording for %f seconds",time);
    if (RET_OK == status) sleep_s(2);

    /* Set starting time */
    if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP,0,&startTime);

    do {
        /* Get the current quaternions */
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,0,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q1);
        if (RET_OK == status) status = db_read(DB_IMU_QUATERNION,1,q_buff);
        if (RET_OK == status) quaternion_from_buffer_build(q_buff, &q2);

        /* Get the second rotation vector in second sensor coordinate system */
        if (RET_OK == status) q12 = arm_quaternion_between_two_get(q2,q1);
        if (RET_OK == status) Quaternion_rotate(&q12,rotationV2,rotationV2_2);

        /* Compute current elbow angles */
        if (RET_OK == status) status = arm_elbow_angles_from_rotation_vectors_get(
            q1, q12, rotationV1, rotationV2_2, anglesFE_B_PS);

        /* Retrieve current timestamp from database */
        if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
        if (RET_OK == status) currentTime = buffTime - startTime;

        dbg_str("Current time %f seconds out of %f seconds",currentTime, time);

        if (RET_OK == status) status = db_field_print(DB_ARM_ELBOW_ANGLES,0);

        /* Wait for next iteration */
        if (RET_OK == status) sleep_ms(100);

    } while (RET_OK == status && time > currentTime);
    if (RET_OK != status) err_str("Failed to apply calibration");

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}

ERROR_CODE hak_two_axes_auto_calib_and_kinematics(double time) {
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
    // if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
    // if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
    // if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_WRIST_POSITION,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (RET_OK == status && imus_num > imu_number_get()) {
        status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Reset IMUs offset */
    if (RET_OK == status) {
        status = imu_orientation_offset_reset();
        if (RET_OK == status) status = imu_orientation_offset_set(1);
        if (RET_OK != status) err_str("Failed to reset IMU sensors orientation");
    }

    /* Read IMUs data for the given amount of time */
    if (RET_OK == status) {
        log_str("Starting loop gathering IMU sensors data to calibrate rotation axes");
        log_str(" -> [USER]: Perform arbitrary motions of the elbow including flexion/extension and pronation/supination for %f seconds",time);

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
            if (RET_OK == status) status = cal_gn2_observations_from_database_update();
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
        log_str(" -> [USER]: Stand in a pose to be considered as zero for the elbow angles in 5");
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
            log_str(" -> [USER]: Elbow angles zero position set ");
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

        do {
            /* Compute the calibrated segment orientations */
            if (RET_OK == status) status = cal_gn2_orientations_from_database_calib_apply(&q1,&q2);

            /* Compute arm positions */
            if (RET_OK == status) arm_orientations_set(q1,q2,q2);

            /* Retrieve current timestamp from database */
            if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
            if (RET_OK == status) currentTime = buffTime - startTime;
            
            /* Log data throug terminal and log file */
            log_str("Quats: arm:<%f, %f,%f,%f> forearm:<%f, %f,%f,%f>", q1.w, q1.v[0], q1.v[1], q1.v[2], q2.w, q2.v[0], q2.v[1], q2.v[2]);
            dbg_str("Current time %f seconds out of %f seconds",currentTime, time);
            if (RET_OK == status) status = db_field_print(DB_ARM_WRIST_POSITION,0);

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

ERROR_CODE hak_static_calib_kinematics(double time)
{
    ERROR_CODE status = RET_OK;
    int imus_num = 2;
    double startTime   = -1.0;
    double currentTime = -1.0;
    double buffTime    = -1.0;
    Quaternion known_quats[2] = { /* Quats for T-pose */
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
        {.w = 1.0, .v={0.0, 0.0, 0.0}},
    };
    Quaternion read_quats[2];

    /* Set the csv logging from the database */
    log_str("Set the database fields to track into the csv");
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_TIMESTAMP,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_IMU_QUATERNION,1);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_POSITION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_WRIST_POSITION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ORIENTATION,0);
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_ELBOW_ORIENTATION,0);
    if (RET_OK != status) err_str("Failed to setup database CSV logging");

    /* Look for IMU sensors and initialize the required of them */
    if (imus_num > imu_number_get()) {
        if (RET_OK == status) status = imu_batch_search_and_initialize(imus_num);
        if (RET_OK != status) err_str("Failed to setup IMU sensors");
    }

    /* Retrieve current IMU data */
    ImuData data[2];
    log_str("Calibrate IMU sensors");
    log_str("USER -> STAND IN T-POSE TO CALIBRATE");
    log_str(" -> [USER]: Stand in T-pose to calibrate IMU orientations");
    if (RET_OK == status) sleep_s(2);
    if (RET_OK == status) status = imu_batch_read(imus_num, data);

    /* Set calibration data */    
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        quaternion_from_float_buffer_build(data[imu].q, &read_quats[imu]);
    }
    if (RET_OK == status) cal_static_imu_quat_calibration_set(known_quats, read_quats);

    /* Start IMU reading callbacks */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_attach(imu, 0==imu);
        if (RET_OK != status) err_str("Failed to initialize reading callback for IMU sensor %d",imu);
        else sleep_s(2);
    }

    /* Apply calibration functions to imu readings */
    log_str("Starting loop printing arm wrist position and recording kinematic data");
    log_str(" -> [USER]: Starting recording for %f seconds",time);
    if (RET_OK == status) sleep_s(2);

    /* Set starting time */
    if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP,0,&startTime);

    do {
        /* Get the current quaternions */
        if (RET_OK == status) status = cal_static_imu_quat_calibrated_data_get(read_quats);

        /* Compute joint values */
        Quaternion joints[ARM_NUMBER_OF_JOINTS];
        if (RET_OK == status) status = arm_inverse_kinematics_compute(read_quats[0], read_quats[1], joints); 

        /* Compute arm positions */
        if (RET_OK == status) status = arm_direct_kinematics_compute(joints, NULL);

        /* Retrieve current timestamp from database */
        if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
        if (RET_OK == status) currentTime = buffTime - startTime;

        dbg_str("Current time %f seconds out of %f seconds",currentTime, time);

        if (RET_OK == status) status = db_field_print(DB_ARM_WRIST_POSITION,0);

        /* Wait for next iteration */
        if (RET_OK == status) sleep_ms(100);

    } while (RET_OK == status && time > currentTime);
    if (RET_OK != status) err_str("Failed to apply calibration");

    /* Remove used resources */
    for (int imu = 0; RET_OK == status && imu < imus_num; imu++) {
        status = imu_read_callback_detach(imu);
    }

    return status;
}

ERROR_CODE hak_static_calib_shoulder_angles(double time)
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
    if (RET_OK == status) status = db_csv_field_add(DB_ARM_SHOULDER_ANGLES,0);
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

    /* Retrieve current IMU data */
    if (RET_OK == status) {
        log_str("Calibrate IMU sensors");
        log_str("USER -> STAND IN T-POSE TO CALIBRATE");
        log_str(" -> [USER]: Stand in T-pose to calibrate IMU orientations");
        if (RET_OK == status) sleep_s(2);
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
            if (RET_OK == status) arm_shoulder_angles_compute(NULL);

            /* Retrieve current timestamp from database */
            if (RET_OK == status) status = db_read(DB_IMU_TIMESTAMP, 0, &buffTime);
            if (RET_OK == status) currentTime = buffTime - startTime;

            /* Log relevant data */
            dbg_str("Current time %f seconds out of %f seconds",currentTime, time);
            if (RET_OK == status) status = db_field_print(DB_ARM_SHOULDER_ANGLES,0);
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
