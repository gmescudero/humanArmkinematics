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


ERROR_CODE imu_initialize(const char *com_port){
    int   connection_status = SENSOR_CONNECTION_CONNECTED;
    short timeout_counter   = IMU_CONNECTION_TIMEOUT; 
    unsigned int index      = (unsigned int) num_imus;

    dbg_str("%s -> Connect IMU %d through COM port \"%s\" ",__FUNCTION__,index, com_port);

    // Check given arguments
    if (NULL == com_port) return RET_ARG_ERROR;
    if (IMU_MAX_NUMBER <= index) return RET_ARG_ERROR;

    // Initialize LPMS manager if not done already
    if (NULL == manager){
        manager = LpmsSensorManagerFactory();   
    }

    // Add a new sensor to the list
    lpms[index] = manager->addSensor(DEVICE_LPMS_U2, com_port);
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

    num_imus++;
    return RET_OK;
}

ERROR_CODE imu_batch_initialize(COM_PORTS com_ports, unsigned int imus_num){
    ERROR_CODE status;

    dbg_str("%s -> Connect %d IMUs out of %d",__FUNCTION__, imus_num, com_ports.ports_number);

    // Check arguments
    if (imus_num > com_ports.ports_number || imus_num <= 0) return RET_ARG_ERROR;

    for (unsigned int i = 0; i < imus_num && RET_OK == status; i++) {
        status = imu_initialize(com_ports.ports_names[i]);
    }

    return status;
}

void imu_batch_terminate(){
    // Removes the initialized sensor
    for (int i = 0; i < num_imus; i++) {
        manager->removeSensor(lpms[i]);
    }
    // Deletes LpmsSensorManager object
    delete manager;
    // Set total number of IMUs to 0
    num_imus = 0;
}

ERROR_CODE imu_read(unsigned int index, ImuData *imu) {
    // dbg_str("%s -> Read IMU %d out of %d",__FUNCTION__, index, num_imus);

    // Check arguments
    if (NULL == imu) return RET_ARG_ERROR;
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;
    if (lpms[index]->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED) return RET_ERROR;
    if (0 >= lpms[index]->hasImuData()) return RET_NO_EXEC;

    // Retrieve IMU data
    *imu = lpms[index]->getCurrentData();

    return RET_OK;
}

ERROR_CODE imu_batch_read(unsigned int imus_num, ImuData imus[]) {
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
    const char *headers[CSV_FILE_VALUES_NUMBER] = {
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
        
    // Check arguments
    if (1 > iterations || 100000 <= iterations) return RET_ARG_ERROR; // TODO make this a macro
    if (0 > index || num_imus <= index) return RET_ARG_ERROR;
    if (NULL == noise) return RET_ARG_ERROR;
    // Check availability
    if (lpms[index]->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED) return RET_ERROR;

    for (int i = 0; i < iterations && RET_OK == status; i++) {
        // Pause
        while (0 >= lpms[index]->hasImuData()) {
            millis_sleep(1);
        }
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




unsigned char setOffsetIMUs(int v){
    // Check status
    if (num_imus == 0) return RET_ERROR;

    for (int i = 0; i < num_imus; i++) {
        if (lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData() ) { 
            lpms[i]->setOrientationOffset(v);
        }
        else {
            return RET_ERROR;
        }
    }
    return RET_OK;
}

void imus_direct_kinematics(ImuData (*imus), ARM_JOINTS (*data_query), bool twoImus, ARM_MEASUREMENT arm_params, bool show) {
    // Parameters initialization
    float arm  = arm_params.arm;
    float fore = arm_params.forearm;
    float a = 0.0;

    // Get IMU Euler angles
    float r[3];
    r[0] = imus -> r[0];
    r[1] = imus -> r[1];
    r[2] = imus -> r[2];

    // Degree to Radian
    float theta[3];
    theta[0] = DEG_2_RAD(r[0]);
    theta[1] = DEG_2_RAD(r[1]);
    theta[2] = DEG_2_RAD(r[2]);

    // Calculos cinematica directa
    float A1[4][4];
    float A2[4][4];
    float A3[4][4];

    // Denavit Hartenberg
    DH_A(a,  M_PI/2,   0, theta[2] + M_PI/2, A1);
    DH_A(a,  M_PI/2,   0, theta[1] + (3*M_PI)/2, A2);
    DH_A(a, -M_PI/2, arm, theta[0] + 0, A3);

    // Get the Elbow Transform Matrix
    float A_1_2[4][4];
    MultiplyMatrix4x4(A_1_2, A1, A2);
    float A_elbow[4][4];
    MultiplyMatrix4x4(A_elbow, A_1_2, A3);

    float elbow_t[3];

    elbow_t[0] = A_elbow[0][3];
    elbow_t[1] = A_elbow[1][3];
    elbow_t[2] = A_elbow[2][3];

    ARM_JOINTS pos;

    pos.elbow_position[0] = elbow_t[0]; pos.elbow_position[1] = elbow_t[1]; pos.elbow_position[2] = elbow_t[2];

    // Write position data to data_query pointer
    for (int i = 0; i < 3; i++){
        data_query -> elbow_position[i] = pos.elbow_position[i];
    }

    // If we want to use two IMUs to determine the wrist position
    float r_Imu_2[3];
    float theta_Imu_2[3];
    float wrist_t[3];
    if (twoImus){
        // Get IMU Euler angles

        r_Imu_2[0] = imus[1].r[0];
        r_Imu_2[1] = imus[1].r[1];
        r_Imu_2[2] = imus[1].r[2];

        // Degree to Radian

        theta_Imu_2[0] = DEG_2_RAD(r[0]) - theta[0];
        theta_Imu_2[1] = DEG_2_RAD(r[1]) - theta[1];
        theta_Imu_2[2] = DEG_2_RAD(r[2]);


        float A4[4][4];
        float A5[4][4];
        DH_A(a,  M_PI/2,    0, theta_Imu_2[1] + 0, A4);
        DH_A(a,       0, fore, theta_Imu_2[0] + 0, A5);

        // Get the Wrist Transform Matrix
        float A_elbow_4[4][4];
        MultiplyMatrix4x4(A_elbow_4, A_elbow, A4);
        float A_wrist[4][4];
        MultiplyMatrix4x4(A_wrist, A_elbow_4, A5);

        wrist_t[0] = A_wrist[0][3];
        wrist_t[1] = A_wrist[1][3];
        wrist_t[2] = A_wrist[2][3];

        pos.wrist_position[0] = wrist_t[0]; pos.wrist_position[1] = wrist_t[1]; pos.wrist_position[2] = wrist_t[2];

        // Write position data to data_query pointer
        for (int i = 0; i < 3; i++){
            data_query -> wrist_position[i] = pos.wrist_position[i];
        }
    }


    // Print data
    if (show) {
        printf("Euler IMU 1: x: %f, y: %f, z: %f\nEuler IMU 1 (radians): x: %f, y: %f, z: %f\nElbow position: x: %f, y: %f, z: %f\n", r[0], r[1], r[2], theta[0], theta[1], theta[2], elbow_t[0], elbow_t[1], elbow_t[2]);
        printf("Euler IMU 2: x: %f, y: %f, z: %f\nEuler IMU 2 (radians): x: %f, y: %f, z: %f\nWrist position: x: %f, y: %f, z: %f\n\n", r_Imu_2[0], r_Imu_2[1], r_Imu_2[2], theta_Imu_2[0], theta_Imu_2[1], theta_Imu_2[2] ,wrist_t[0], wrist_t[1], wrist_t[2]);
        } 

}

void printMatrix(float (Mat)[4][4]) {
    for (int h = 0; h < 4; h++) {
        for (int w = 0; w < 4; w++) {
                printf("%f,", Mat[h][w]);
        }
        printf("\n\n");
    }
}

void DH_A(float a, float alpha, float d, float theta, float (*Mat)[4]){
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            Mat[i][j] = 0;
        }
    }
    Mat[0][0] = cos(theta); Mat[0][1] = -sin(theta)*cos(alpha); Mat[0][2] = sin(theta)*sin(alpha);  Mat[0][3] = a*cos(theta);
    Mat[1][0] = sin(theta); Mat[1][1] =  cos(theta)*cos(alpha); Mat[1][2] = -cos(theta)*sin(alpha); Mat[1][3] = a*sin(theta);
    Mat[2][0] = 0;          Mat[2][1] = sin(alpha);             Mat[2][2] = cos(alpha);             Mat[2][3] = d;
    Mat[3][0] = 0;          Mat[3][1] = 0;                      Mat[3][2] = 0;                      Mat[3][3] = 1;
}

void MultiplyMatrix4x4(float (*Mat)[4], float Mat1[4][4], float Mat2[4][4]){
    float M[4][4];
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            Mat[i][j] = 0;
            M[i][j] = 0;
        }
    }
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
                M[i][j] += (Mat1[i][k] * Mat2[k][j]);
            }
        }
    }
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            Mat[i][j] = M[i][j];
        }
    }
}
