/******************************************************************************/
/*
 * Project  : ExoFlex
 * Version  : 0.1
 * File:   imu.cpp
 * Author: Aldo Contreras
 * Description : GUI for read serial port
 * Created on June 23, 2019, 11:37
 */
/******************************************************************************/

#ifdef _WIN32
#include "LpmsSensorI.h"
#include "LpmsSensorManagerI.h"
char imu_conn[15] = "COM0";
#endif

#ifdef __GNUC__
#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
char imu_conn[15] = "/dev/ttyUSB0";
#endif

#include "imu.h"
extern "C" unsigned int sleep(unsigned int __seconds);
// extern "C" void initilize_imus(int n_imus, int init);
// extern "C" void read_imus(ImuData *imus);
// extern "C" void stop_imus();


unsigned char num_imus = 0;
int initial_COM = -1;
LpmsSensorManagerI* manager; /* Gets a LpmsSensorManager instance */
LpmsSensorI* lpms[7];

/* 
* Initialize imu sensors and wait for them to connect.
* Return true if all correctly connected and false otherwise
*/
bool initialize_imus(int n_imus, int init){
    bool    ret = true;
    int     connection_status = SENSOR_CONNECTION_CONNECTED;
    short   timeout_counter;

    num_imus    = n_imus;
    initial_COM = init;
    manager = LpmsSensorManagerFactory();
    for (int i = 0; i < num_imus; i++) {
        imu_conn[strlen(imu_conn)-1] = init + '0';
        lpms[i] = manager->addSensor(DEVICE_LPMS_U2, imu_conn);
        init++; 
    }

    // Wait for connection
    for (int i = 0; (i < num_imus) && (SENSOR_CONNECTION_CONNECTED == connection_status); i++) {
        connection_status = lpms[i]->getConnectionStatus();
        timeout_counter = 4; // wait for 4 seconds maximum until each imu is connected
        while ((connection_status != SENSOR_CONNECTION_CONNECTED) && (timeout_counter > 0)){
            sleep(1);
            timeout_counter--;
            connection_status = lpms[i]->getConnectionStatus();
        }
        if (connection_status != SENSOR_CONNECTION_CONNECTED) {
            printf("IMU Sensor %d failed to connect\n",i);
            ret = false;
        }
    }
    return ret;
}

void read_imus(ImuData *imus){
    for (int i = 0; i < num_imus; i++) {
        if (lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData() ){
            imus[i] = lpms[i]->getCurrentData();
        }
    }
}

void stop_imus(){
    // Removes the initialized sensor
    for (int i = 0; i < num_imus; i++) {
        manager->removeSensor(lpms[i]);
    }
    // Deletes LpmsSensorManager object
    delete manager;
}

unsigned char setOffsetIMUs(int v){
    if (num_imus == 0) return 0;
    for (int i = 0; i < num_imus; i++) {
        if (lpms[i]->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms[i]->hasImuData() ) { lpms[i]->setOrientationOffset(v);}
        else {return 0;}
    }
    return 1;
}

void imus_direct_kinematics(ImuData (*imus), ARM_JOINTS (*data_query), bool twoImus, ARM_MEASUREMENT arm_params, bool show) {
    // Parameters initialization
    float arm = arm_params.arm;
    float fore = arm_params.forearm;
    float a = 0.0;

    // Get IMU Euler angles
    float r[3];
    r[0] = imus -> r[0];
    r[1] = imus -> r[1];
    r[2] = imus -> r[2];

    // Degree to Radian
    float theta[3];
    theta[0] = degree2radian(r[0]);
    theta[1] = degree2radian(r[1]);
    theta[2] = degree2radian(r[2]);

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

        theta_Imu_2[0] = degree2radian(r[0]) - theta[0];
        theta_Imu_2[1] = degree2radian(r[1]) - theta[1];
        theta_Imu_2[2] = degree2radian(r[2]);


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

float degree2radian(float degree) {
    float radian;
    return radian = degree * M_PI/180;
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
