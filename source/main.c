
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
#include "functions.h"


int main(int argc, char **argv) {
    COM_PORTS discoveredPorts;

    /* Look for com ports avalilable in the system */
    if (RET_OK != com_ports_list(&discoveredPorts)){
        err_str("DEP: %d\n",__LINE__);
        return RET_ERROR;
    }

    /* Initialize IMU in a given COM port */
    if (RET_OK != imu_initialize("/dev/ttyS3")){
        err_str("DEP: %d\n",__LINE__);
        return RET_ERROR;
    }
}
