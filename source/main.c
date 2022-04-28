#include <stdio.h>
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
        printf("DEP: %d\n",__LINE__);
        return RET_ERROR;
    }

    /* Initialize imu in a given COM port */
    if (RET_OK != imu_initialize("/dev/ttyS3")){
        printf("DEP: %d\n",__LINE__);
        return RET_ERROR;
    }
    

}

#if 0
int show_ports_IMU(app_widgets* app_wgs){
    unsigned int i = 0; //number of ports found
    enum sp_return error = sp_list_ports(&ports);
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(app_wgs->g_combo_box_serialport_IMU));

    if(error == SP_OK){
        for(i = 0; ports[i]; i++){
            //printf("Found port: '%s'\n", sp_get_port_name(ports[i]));
            gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(app_wgs->g_combo_box_serialport_IMU), sp_get_port_name(ports[i]));
        }
        gtk_combo_box_set_active(GTK_COMBO_BOX(app_wgs->g_combo_box_serialport_IMU), 0);
        sp_free_port_list(ports);
        //printf("Count: %d\n",i);
    }else{
        log_str("-> Review device connections and drivers\n");
    }
    return(i);
}
#endif
/*
	 Operation completed successfully. 
	SP_OK = 0,

	 Invalid arguments were passed to the function. 
	SP_ERR_ARG = -1,

	 A system error occurred while executing the operation. 
	SP_ERR_FAIL = -2,

	 A memory allocation failed while executing the operation. 
	SP_ERR_MEM = -3,

	 The requested operation is not supported by this system or device. 
	SP_ERR_SUPP = -4
*/
