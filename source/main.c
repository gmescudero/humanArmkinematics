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
#include "general.h"
#include "vector3.h"
#include "database.h"
#include "calib.h"
#include "launch.h"
#include <string.h>


int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;

    if (RET_OK == status) status = hak_initialize();

    if (RET_OK == status) status = hak_record_imus_data(2,30,1000);

    if (RET_OK == status) status = hak_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
