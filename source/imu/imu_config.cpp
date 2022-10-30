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
extern "C" {
#include "general.h"
}

void imu_configuration_apply(LpmsSensorI *lpms) {
    // Save parameters to sensor
    lpms->saveCalibrationData();
    WAIT(500);
}


void imu_configuration_set(LpmsSensorI *lpms, int param, int *val) {
    WAIT(100);
    if (false == lpms->setConfigurationPrm(param, val)) {
        wrn_str("Failed to configure parameter %d with value %d",param,*val);
    }
}

void imu_set_SensorID(int id, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_OPENMAT_ID,&id);
}

void imu_set_FilterMode(int mode, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_FILTER_MODE,&mode);
}

void imu_set_GyroRange(int range, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_GYR_RANGE,&range);
}

void imu_set_AccRange(int range, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_ACC_RANGE,&range);
}

void imu_set_MagRange(int range, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_MAG_RANGE,&range);
}

void imu_set_SamplingRate(int rate, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_SAMPLING_RATE,&rate);
}

void imu_set_OutputData(int data, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_SELECT_DATA,&data);
}

void imu_set_MagneticCorrection(int mode, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_SELECT_DATA,&mode);
}

void imu_set_LinAccCompensationMode(int mode, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_LIN_ACC_COMP_MODE,&mode);
}

void imu_set_RotationalAccCompensation(int mode, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_CENTRI_COMP_MODE,&mode);
}

void imu_set_LpBusMode(int mode, LpmsSensorI *lpms) {
    imu_configuration_set(lpms,PRM_LPBUS_DATA_MODE,&mode);
}