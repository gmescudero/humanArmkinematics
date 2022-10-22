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
#include "launch.h"


int main(int argc, char **argv) {
    ERROR_CODE status = RET_OK;

    if (RET_OK == status) status = hak_initialize();

    // if (RET_OK == status) status = hak_record_imus_data(2,60,0);
    if (RET_OK == status) status = hak_two_axes_auto_calib_and_kinematics(40, false, true);
    // if (RET_OK == status) status = hak_static_calib_kinematics(40, true);

    if (RET_OK == status) status = hak_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
