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
#include "comms.h"
#include <string.h>
#include <stdlib.h>


int main(int argc, char **argv) {
    ERROR_CODE status = hak_initialize();
    char ip[32]   = COM_DEFAULT_IP;
    unsigned port = COM_DEFAULT_PORT;

    if (1 < argc && 32 > strlen(argv[1])) {
        strcpy(ip,argv[1]);

        if (2 < argc && 8 > strlen(argv[2])) {
            port = (unsigned)atoi(argv[2]);
        }
        hak_network_setup(ip, port);
    }

    // if (RET_OK == status) status = hak_record_imus_data(2,60,0);
    // if (RET_OK == status) status = hak_two_axes_auto_calib_and_kinematics(60, false, true);
    // if (RET_OK == status) status = hak_static_calib_kinematics(40, true);
    if (RET_OK == status) status = hak_two_axes_auto_calib_and_kinematics_forever(false,false,true||argc>1);

    status += hak_terminate();

    return (RET_OK == status)? RET_OK:RET_ERROR;
}
