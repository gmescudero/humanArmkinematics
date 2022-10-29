/**
 * @file imu_config.cpp
 * @author German Moreno Escudero
 * @brief LPMS IMU configuration
 * @version 0.1
 * @date 2022-10-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "imu_config.hpp"


void imu_configuration_apply(LpmsSensorI *lpms) {
    // Save parameters to sensor
    lpms->saveCalibrationData();
    WAIT(500);
}


void imu_set_SensorID(int id, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_OPENMAT_ID, id);
}

void imu_set_FilterMode(int mode, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_FILTER_MODE, mode);
}

void imu_set_GyroRange(int range, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_GYR_RANGE, range);
}

void imu_set_AccRange(int range, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_ACC_RANGE, range);
}

void imu_set_MagRange(int range, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_MAG_RANGE, range);
}

void imu_set_SamplingRate(int rate, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_SAMPLING_RATE, rate);
}

void imu_set_OutputData(int data, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_SELECT_DATA, data);
}

void imu_set_MagneticCorrection(int mode, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_PARAMETER_SET, SELECT_IMU_SLOW);
}

void imu_set_LinAccCompensationMode(int mode, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_LIN_ACC_COMP_MODE, mode);
}

void imu_set_RotationalAccCompensation(int mode, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_CENTRI_COMP_MODE, mode);
}

void imu_set_LpBusMode(int mode, LpmsSensorI *lpms) {
    WAIT(100);
    lpms->setConfigurationPrm(PRM_LPBUS_DATA_MODE, mode);
}