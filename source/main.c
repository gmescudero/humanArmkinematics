
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
    ARM_POSE initialArmPose = {
        .shoulderPosition = {0.0, 0.0, 0.0},
        .elbowPosition    = {0.0, 0.0, -10.0},
        .wristPosition    = {0.0, 0.0, -15.0},
    };
    
    // Initialize logging files
    status = log_file_initalize();
    if (RET_OK == status) {
        status = db_initialize();
    }
    if (RET_OK == status) {
        // Initialize Arm
        initializeArm(initialArmPose);
    }

    return status;
}

int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;
    COM_PORTS discoveredPorts;
    // IMU_NOISE_DATA noiseData;
    ImuData data;
    double startTime = -1.0;
    double currentTime = -1.0;
    double rotVector[3]    = {0.5,0.5,0.5};
    double newRotVector[3] = {0.5,0.5,0.5};
    double error = 0.0;

    double csvBuff[CSV_FILE_VALUES_NUMBER] = {0.0};

    /* Initialize all packages */
    log_str("Initialize");
    status = initialize();
    STATUS_EVAL(status);

    /* Configure traces*/
    if (RET_OK == status) {
        log_str("Set trace level");
        status = trace_level_set(INFO,DEBUG);
    }

    /* Configure CSV file*/
    if (RET_OK == status) {
        log_str("Set CSV headers");
        const char *headers[CSV_FILE_VALUES_NUMBER] = {
            "timestamp",
            "omegaR_X","omegaR_Y","omegaR_Z",
            "rotVector_X","rotVector_Y","rotVector_Z",
            "error"
        };
        csv_headers_set(headers,8);
        // imu_csv_headers_set();
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

    /* Measure Noise from IMU *//* 
    if (RET_OK == status) {
        log_str("Measure IMUs noise. Keep the sensor as steady as possible");
        status = imu_static_errors_measure(0, 1000, &noiseData);
        log_str("Measured mean of gyroscope: [%f, %f, %f]",
            noiseData.gyrMean[0],noiseData.gyrMean[1],noiseData.gyrMean[2]);
        log_str("Measured variance of gyroscope: [%f, %f, %f]",
            noiseData.gyrVar[0],noiseData.gyrVar[1],noiseData.gyrVar[2]);
        STATUS_EVAL(status);
    } */

    millis_sleep(1000);
    
    if (RET_OK == status) {
        log_str("Loop reading IMU data and logging it to the CSV file");
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
                memcpy(rotVector,newRotVector,sizeof(newRotVector));
                double gyr[] = {(float)data.gRaw[0], (float)data.gRaw[1], (float)data.gRaw[2]};
                status = calibrateRotationAxis(rotVector,gyr,newRotVector, &error);
                STATUS_EVAL(status);
            }
            if (RET_OK == status) {
                // imu_csv_log(data);
                int i = 0;
                csvBuff[i++] = currentTime;
                csvBuff[i++] = data.g[0]; csvBuff[i++] = data.g[1]; csvBuff[i++] = data.g[2]; 
                csvBuff[i++] = newRotVector[0]; csvBuff[i++] = newRotVector[1]; csvBuff[i++] = newRotVector[2]; 
                csvBuff[i++] = error;
                csv_log(csvBuff);
                dbg_str("Time: %f | omegaR: [%f, %f, %f] | newRotVector: [%f, %f, %f] | error: %f",
                    currentTime,
                    data.gRaw[0],data.gRaw[1],data.gRaw[2],
                    newRotVector[0],newRotVector[1],newRotVector[2],
                    error);
            }
            
        } while ((RET_OK == status || RET_NO_EXEC == status) && -20 > currentTime);
    } 
    if (RET_OK == status) {
        log_str("The obtained rotation axis: [%f, %f, %f]",newRotVector[0],newRotVector[1],newRotVector[2]);
    }

    log_str("Write Timestamp %f to the database",data.timeStamp);
    if (RET_OK == status) {
        status = db_write(DB_TIMESTAMP, (void*)&(data.timeStamp));
    }

    log_str("Read Timestamp from the database");
    if (RET_OK == status) {
        double time = 0.0;
        status = db_read(DB_TIMESTAMP, (void*)&(time));
        if (RET_OK == status) log_str("\t -> retrieved timestamp: %f",time);
    }

    /* Terminate all imus */
    log_str("Terminate all IMU connections");
    if (RET_OK == status) {
        imu_batch_terminate();
    }

    log_str("Clean memory and environment");
    status += db_terminate();
    
    return status;
}
