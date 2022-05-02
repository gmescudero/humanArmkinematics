
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

#define STATUS_EVAL(code) {if (RET_OK != code) err_str("[%d] DEP: %d \n",__LINE__, code);}

int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;
    COM_PORTS discoveredPorts;
    ImuData data;

    log_file_initalize();
    trace_level_set(DEBUG,DEBUG);
    err_str("pollo");
    wrn_str("pollo");
    log_str("pollo");
    dbg_str("pollo");
#if 0
    /* Look for com ports avalilable in the system */
    if (RET_OK == status) {
        status = com_ports_list(&discoveredPorts);
        STATUS_EVAL(status);
    }

    /* Initialize IMU in a given COM port */
    if (RET_OK == status) {
        status = imu_initialize(discoveredPorts.ports_names[0]);
        STATUS_EVAL(status);
    }

    /* Read IMU data */
    if (RET_OK == status) {
        status = imu_read(0, &data);
        STATUS_EVAL(status);
    }


    /* Show retrieved data */
    if (RET_OK == status) {
        imu_data_print(data);
    }

    /* Terminate all imus */
    if (RET_OK == status) {
        imu_batch_terminate();
    }
#endif
    return status;
}
