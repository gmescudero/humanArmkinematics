/**
 * @file imu.cpp
 * @author German Moreno Escudero
 * @brief LPMS IMU managing pack
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
// #define IMU_CONNECTION_PORT "COM0"
#endif

#ifdef __GNUC__
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
// #define IMU_CONNECTION_PORT "/dev/ttyUSB0";
#endif

#include "imu.h"
#include <unistd.h>

// char imu_conn[15] = IMU_CONNECTION_PORT;
static unsigned char num_imus = 0;
static LpmsSensorManagerI* manager = NULL; /* Gets a LpmsSensorManager instance */
static LpmsSensorI* lpms[IMU_MAX_NUMBER] = {NULL};

static ERROR_CODE simu_database_update(ImuData d, int index);
static void simu_callback_new_data_update(ImuData d, const char *id);
static void simu_callback_new_data_update_and_dump(ImuData d, const char *id);

ERROR_CODE imu_initialize(const char *com_port){
    ERROR_CODE status;
    int   connection_status = SENSOR_CONNECTION_CONNECTED;
    short timeout_counter   = IMU_CONNECTION_TIMEOUT; 
    unsigned int index      = (unsigned int) num_imus;

    dbg_str("%s -> Connect IMU %d through COM port \"%s\" ",__FUNCTION__,index, com_port);

    // Check given arguments
    if (NULL == com_port) return RET_ARG_ERROR;
    if (IMU_MAX_NUMBER <= index) return RET_ARG_ERROR;

    // Initialize LPMS manager if not done already
    if (NULL == manager){
        dbg_str("%s -> Initializing LPMS sensor manager factory",__FUNCTION__);
        manager = LpmsSensorManagerFactory();   
    }

    // Add a new sensor to the list
    lpms[index] = manager->addSensor(DEVICE_LPMS_U2, com_port);
    lpms[index]->setVerbose(true);
    if (NULL == lpms[index]) return RET_ERROR;

    // Retrieve cthe connection status
    connection_status = lpms[index]->getConnectionStatus();
    // Wait for the connection status to be CONNECTED
    while ((connection_status != SENSOR_CONNECTION_CONNECTED) && (timeout_counter > 0)){
        sleep(1);
        timeout_counter--;
        connection_status = lpms[index]->getConnectionStatus();
    }
    if (connection_status != SENSOR_CONNECTION_CONNECTED) {
        err_str("IMU Sensor %d failed to connect through %s", index, com_port);
        return RET_ERROR;
    }

    // Update number of imus and database
    num_imus++;
    status = db_write(DB_IMU_NUMBER, 0, (int*)&(num_imus));

    return status;
}

ERROR_CODE imu_batch_initialize(COM_PORTS com_ports, unsigned int imus_num){
    ERROR_CODE status = RET_OK;
    unsigned int i;

    dbg_str("%s -> Connect %d IMUs out of %d",__FUNCTION__, imus_num, com_ports.ports_number);

    // Check arguments
    if (imus_num > com_ports.ports_number || imus_num <= 0) return RET_ARG_ERROR;

    for (i = 0; i < imus_num && RET_OK == status; i++) {
        status = imu_initialize(com_ports.ports_names[i]);
    }

    return status;
}

ERROR_CODE imu_batch_search_and_initialize(unsigned int imus_num) {
    ERROR_CODE status = RET_OK;
    COM_PORTS discoveredPorts;

    dbg_str("%s -> Search and initialize %d IMUs",__FUNCTION__, imus_num);

    // Check arguments
    if (imus_num > IMU_MAX_NUMBER || imus_num <= 0) return RET_ARG_ERROR;

    /* Look for COM ports avalilable in the system */
    if (RET_OK == status) {
        log_str("Retrieve available COM ports");
        status = com_ports_list(&discoveredPorts);
        if (RET_OK == status && discoveredPorts.ports_number < imus_num) {
            err_str("Found only %d when %d IMU sensors required");
            status = RET_ERROR;
        }
    }

    /* Initialize required IMU sensors */
    status  = imu_batch_initialize(discoveredPorts, imus_num);

    return status;
}

void imu_all_sensors_remove(){
    // Removes the initialized sensor
    for (int i = 0; i < num_imus; i++) {
        manager->removeSensor(lpms[i]);
    }
    // Set total number of IMUs to 0
    num_imus = 0;
    manager = NULL;
    // Update database
    db_write(DB_IMU_NUMBER, 0, (int*)&(num_imus));
}

void imu_terminate(){
    log_str("Terminate all IMU connections");
    // Removes the initialized sensors
    imu_all_sensors_remove();
    // Deletes LpmsSensorManager object
    delete manager;
}

int imu_number_get() {
    return (int)num_imus;
}

/**
 * @brief Callback for IMU data update
 * 
 * @param d (input) Last IMU data retrieved from sensor
 * @param id (input) COM port
 */
static void simu_callback_new_data_update(ImuData d, const char *id) {
    // Update database fields
    ERROR_CODE status;
    int comparison = -1;
    int imu;
    int index;
    char imu_id_str[COM_PORTS_LENGTH];

    // Retrieve IMU index
    for (imu = 0; 0 != comparison && imu < num_imus; imu++) {
        lpms[imu]->getDeviceId(imu_id_str);
        comparison = strcmp(id, imu_id_str);
    }
    if (0 != comparison) {
        err_str("Error identifying imu index");
        return;
    }
    else {
        index = imu-1;
    }

    dbg_str("%s -> Updating IMU %d data in port %s",__FUNCTION__, index, id);
    // Update database
    status = simu_database_update(d, index);
    if (RET_OK != status) {
        err_str("Error updating database IMU related fields from callback");
        return;
    }
}

/**
 * @brief Callback for IMU data update and update
 * 
 * @param d (input) Last IMU data retrieved from sensor
 * @param id (input) COM port
 */
static void simu_callback_new_data_update_and_dump(ImuData d, const char *id) {
    // Update database fields
    ERROR_CODE status;
    int comparison = -1;
    int imu;
    int index;
    char imu_id_str[COM_PORTS_LENGTH];

    // Retrieve IMU index
    for (imu = 0; 0 != comparison && imu < num_imus; imu++) {
        lpms[imu]->getDeviceId(imu_id_str);
        comparison = strcmp(id,imu_id_str);
    }
    if (0 != comparison) {
        err_str("Error identifying imu index");
        return;
    }
    else {
        index = imu-1;
    }

    dbg_str("%s -> Updating and dumping IMU %d data in port %s",__FUNCTION__, index, id);
    // Update database
    status = simu_database_update(d, index);
    if (RET_OK != status) {
        err_str("Error updating database IMU related fields from callback");
        return;
    }
    // Dump csv
    status = db_csv_dump();
    if (RET_OK != status) {
        err_str("Error updating csv from database");
        return;
    }
}

/**
 * @brief Callback used as placeholder for detached callback
 * 
 * @param d N/A
 * @param id N/A
 */
static void simu_callback_no_operation(ImuData d, const char *id) {
    /* No operation */
}


ERROR_CODE imu_read_callback_attach(unsigned int index, bool csv_dump) {
    dbg_str("%s -> Attaching data retrieving callback to IMU%d",__FUNCTION__, index);
    // Check arguments
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;

    if (true == csv_dump) {
        // Read data, update database and dump data into the csv
        lpms[index]->setCallback(&simu_callback_new_data_update_and_dump);
    }
    else {
        // Read data and update database
        lpms[index]->setCallback(&simu_callback_new_data_update);
    }

    return RET_OK;
}

ERROR_CODE imu_read_callback_detach(unsigned int index) {
    dbg_str("%s -> Detaching data retrieving callback from IMU%d",__FUNCTION__, index);
    // Check arguments
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;
    
    lpms[index]->setCallback(&simu_callback_no_operation);

    return RET_OK;
}

ERROR_CODE imu_read(unsigned int index, ImuData *imu) {
    // dbg_str("%s -> Read IMU %d out of %d",__FUNCTION__, index, num_imus);
    ERROR_CODE status;

    // Check arguments
    if (NULL == imu) return RET_ARG_ERROR;
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;
    if (lpms[index]->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED) return RET_ERROR;
    if (0 >= lpms[index]->hasImuData()) return RET_NO_EXEC;

    // Retrieve IMU data
    *imu = lpms[index]->getCurrentData();
    // Update database fields
    status = simu_database_update(*imu, index);

    return status;
}

ERROR_CODE imu_batch_read(unsigned int imus_num, ImuData imus[]) {
    dbg_str("%s -> Read %d IMUs out of %d",__FUNCTION__, imus_num, num_imus);
    ERROR_CODE status = RET_OK;
    // Check arguments
    if (imus_num > num_imus || imus_num <= 0) return RET_ARG_ERROR;
    if (NULL == imus) return RET_ARG_ERROR;

    for (unsigned int i = 0; i < imus_num && RET_OK == status; i++) {
        status = imu_read(i, &imus[i]);
    }

    return status;
}


void imu_data_print (ImuData imu){
    log_str(" =========== IMU DATA =========== ");

    log_str("\topenMatId:           %d",imu.openMatId);
    log_str("\ttimeStamp:           %g",imu.timeStamp);

    log_str("\taccelerometer_raw:   [%g,%g,%g]",imu.aRaw[0],imu.aRaw[1],imu.aRaw[2]);
    log_str("\tgyroscope_raw:       [%g,%g,%g]",imu.gRaw[0],imu.gRaw[1],imu.gRaw[2]);
    log_str("\tmagnetometer_raw:    [%g,%g,%g]",imu.bRaw[0],imu.bRaw[1],imu.bRaw[2]);

    log_str("\taccelerometer_calib: [%g,%g,%g]",imu.a[0],imu.a[1],imu.a[2]);
    log_str("\tgyroscope_calib:     [%g,%g,%g]",imu.g[0],imu.g[1],imu.g[2]);
    log_str("\tmagnetometer_calib:  [%g,%g,%g]",imu.b[0],imu.b[1],imu.b[2]);

    log_str("\tangular_vel:         [%g,%g,%g]",imu.w[0],imu.w[1],imu.w[2]);
    log_str("\teuler_angles:        [%g,%g,%g]",imu.r[0],imu.r[1],imu.r[2]);
    log_str("\tquaternion:          [%g,%g,%g,%g]",imu.q[0],imu.q[1],imu.q[2],imu.q[3]);

    log_str("\trotation_matrix: \n"
        "\t\t[%g,%g,%g]\n\t\t[%g,%g,%g]\n\t\t[%g,%g,%g]",
        imu.rotationM[0],imu.rotationM[1],imu.rotationM[2],
        imu.rotationM[3],imu.rotationM[4],imu.rotationM[5],
        imu.rotationM[6],imu.rotationM[7],imu.rotationM[8]);
    log_str("\trotation_matrix (zeroed): \n"
        "\t\t[%g,%g,%g]\n\t\t[%g,%g,%g]\n\t\t[%g,%g,%g]",
        imu.rotOffsetM[0],imu.rotOffsetM[1],imu.rotOffsetM[2],
        imu.rotOffsetM[3],imu.rotOffsetM[4],imu.rotOffsetM[5],
        imu.rotOffsetM[6],imu.rotOffsetM[7],imu.rotOffsetM[8]);
    log_str("\tlinear_accel:        [%g,%g,%g]",imu.linAcc[0],imu.linAcc[1],imu.linAcc[2]);

    log_str("\tpressure:            %g",imu.pressure);
    log_str("\tgyro_temp:           %g",imu.pressure);
    log_str("\ttemperature:         %g",imu.temperature);

    log_str("\tdata_frame_index:    %d",imu.frameCount);

    // HeaveMotionModule hm;
    // GaitTrackingModule gm;

    log_str(" ================================ ");
}


void imu_csv_headers_set(void) {
    const char headers[CSV_FILE_VALUES_NUMBER][CSV_HEADER_MAX_LENGTH] = {
        "timestamp",
        "rawAcc0","rawAcc1","rawAcc2",
        "rawGyr0","rawGyr1","rawGyr2",
        "rawMag0","rawMag1","rawMag2",
        "acc0","acc1","acc2",
        "gyr0","gyr1","gyr2",
        "mag0","mag1","mag2",
        "linAcc0","linAcc1","linAcc2",
        "angVel0","angVel1","angVel2",
        "quatW","quatX","quatY","quatZ"
    };

    csv_headers_set(headers, 29);
}

void imu_csv_log(ImuData d) {
    const double dat[CSV_FILE_VALUES_NUMBER] = {
        // Timestamp
        d.timeStamp,
        // Raw accelerometer
        d.aRaw[0], d.aRaw[1], d.aRaw[2],
        // Raw gyroscope
        d.gRaw[0], d.gRaw[1], d.gRaw[2],
        // Raw magnetometer
        d.bRaw[0], d.bRaw[1], d.bRaw[2],
        // Calibrated accelerometer
        d.a[0], d.a[1], d.a[2],
        // Calibrated gyroscope
        d.g[0], d.g[1], d.g[2],
        // Calibrated magnetometer
        d.b[0], d.b[1], d.b[2],
        // Linear acceleration
        d.linAcc[0], d.linAcc[1], d.linAcc[2],
        // Angular velocity
        d.w[0], d.w[1], d.w[2],
        // Quaternion 
        d.q[0], d.q[1], d.q[2], d.q[3],

        0.0};
        
    csv_log(dat);
}

ERROR_CODE imu_static_errors_measure(unsigned int index, int iterations, IMU_NOISE_DATA *noise) {
    ERROR_CODE status = RET_OK;
    ImuData data;

    double sumAcc[3]  = {0.0};
    double sumAcc2[3] = {0.0};
    double sumGyr[3]  = {0.0};
    double sumGyr2[3] = {0.0};
    double sumMag[3]  = {0.0};
    double sumMag2[3] = {0.0};
    int noDataTimeout = 20;

        
    // Check arguments
    if (1 > iterations || 100000 <= iterations) return RET_ARG_ERROR; // TODO make this a macro
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;
    if (NULL == noise) return RET_ARG_ERROR;
    // Check availability
    if (lpms[index]->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED) return RET_ERROR;

    for (int i = 0; i < iterations && RET_OK == status; i++) {
        // Pause
        while (0 >= lpms[index]->hasImuData()) {
            sleep_ms(10);
            if (0 >= noDataTimeout--) {
                err_str("Unable to retrieve IMU sensor %d data",index);
                return RET_ERROR;
            }
        }
        noDataTimeout = 20;

        // Read imu
        status = imu_read(index,&data);
        // Aggregate values
        for (int j = 0; j < 3 && RET_OK == status; j++) {
            sumAcc[j]  += data.a[j];
            sumAcc2[j] += data.a[j]*data.a[j];
            sumGyr[j]  += data.g[j];
            sumGyr2[j] += data.g[j]*data.g[j];
            sumMag[j]  += data.b[j];
            sumMag2[j] += data.b[j]*data.b[j];
        }
    }

    for (int j = 0; j < 3 && RET_OK == status; j++) {
        noise->accMean[j] = sumAcc[j]/iterations;
        noise->accVar[j]  = (sumAcc2[j] - (noise->accMean[j])*(noise->accMean[j]))/(iterations-1);
        noise->gyrMean[j] = sumGyr[j]/iterations;
        noise->gyrVar[j]  = (sumGyr2[j] - (noise->gyrMean[j])*(noise->gyrMean[j]))/(iterations-1);
        noise->magMean[j] = sumMag[j]/iterations;
        noise->magVar[j]  = (sumMag2[j] - (noise->magMean[j])*(noise->magMean[j]))/(iterations-1);
    }

    return status;
}

/**
 * @brief Update IMU values into database fields
 * 
 * @param d (input) Given IMU data
 * @return ERROR_CODE 
 */
static ERROR_CODE simu_database_update(ImuData d, int index) {
    ERROR_CODE status;
    double timestamp = d.timeStamp;
    double acc[3]    = {(double)d.a[0],(double)d.a[1],(double)d.a[2]};
    double gyr[3]    = {(double)d.g[0],(double)d.g[1],(double)d.g[2]};
    double mag[3]    = {(double)d.b[0],(double)d.b[1],(double)d.b[2]};
    double linAcc[3] = {(double)d.linAcc[0],(double)d.linAcc[1],(double)d.linAcc[2]};
    double angVel[3] = {(double)d.w[0],(double)d.w[1],(double)d.w[2]};
    double quat[4]   = {(double)d.q[0],(double)d.q[1],(double)d.q[2],(double)d.q[3]};
    int newData      = (int)true;

    status = db_write(DB_IMU_TIMESTAMP, index, &timestamp);

    if (RET_OK == status) {
        status = db_write(DB_IMU_ACCELEROMETER, index, &acc);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_GYROSCOPE, index, &gyr);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_MAGNETOMETER, index, &mag);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_LINEAR_ACCELERATION, index, &linAcc);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_ANGULAR_VELOCITY, index, &angVel);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_QUATERNION, index, &quat);
    }
    if (RET_OK == status) {
        status = db_write(DB_IMU_NEW_DATA, index, &newData);
    }
    return status;
}




/* TODO: this has to be reviewed to see in which way it affects */
ERROR_CODE imu_orientation_offset_set(int v){
    // Check status
    if (num_imus <= 0) return RET_ERROR;

    dbg_str("%s -> set IMU offset to %d",__FUNCTION__,v);

    for (int i = 0; i < num_imus; i++) {
        if (lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) { 
            lpms[i]->setOrientationOffset(v);
        }
        else {
            return RET_ERROR;
        }
    }
    return RET_OK;
}

ERROR_CODE imu_orientation_offset_reset(){
    // Check status
    if (num_imus <= 0) return RET_ERROR;

    dbg_str("%s -> reset IMU offset", __FUNCTION__);

    for (int i = 0; i < num_imus; i++) {
        if (lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED) { 
            lpms[i]->resetOrientationOffset();
        }
        else {
            return RET_ERROR;
        }
    }
    return RET_OK;
}