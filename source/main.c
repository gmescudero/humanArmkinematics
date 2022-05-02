
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

#define STATUS_EVAL(code) {if (RET_OK != code) err_str("[%d] Failed: %d \n",__LINE__, code);}

int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;
    COM_PORTS discoveredPorts;
    ImuData data;
    double csvTry[CSV_FILE_VALUES_NUMBER] = {
        0.0,0.1,0.2,0.3,0.4,
        1.0,2.0,3.0,4.0,5.0,
        10.0,20.0,30.0,40.0,50.0,
        123.4,567.8,901.2,345.6,789.0
    };
    char *csvHead[CSV_FILE_VALUES_NUMBER] = {
        "Hello","Csv","Values","","","","","","","","","","","","","","","","","" 
    };

    log_file_initalize();
    trace_level_set(DEBUG,DEBUG);
    // csv_set_headers(csvHead);
    csv_log(csvTry);
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
