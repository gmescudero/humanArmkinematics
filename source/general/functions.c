
#include <stdio.h>
#include <string.h>
#include <libserialport.h>  /* READ COM ports before connect */
#include <time.h>
#include "functions.h"

struct sp_port **ports;

ERROR_CODE com_ports_list(COM_PORTS *discoveredPorts) {
    enum sp_return error;

    /* Check arguments */
    if (NULL == discoveredPorts) return RET_ERROR;

    /* Reset number of COM ports */
    discoveredPorts->ports_number = 0;

    /* Find the serial COM ports  */
    error = sp_list_ports(&ports);

    /* Check error */
    if (SP_OK == error) {
        printf("Ports found: \n");
        /* Show all COM ports */
        for(int i = 0; ports[i]; i++){
            /* Increment COM ports total number of ports */
            discoveredPorts->ports_number++;
            /* Copy the port name to the given structure */
            strcpy(discoveredPorts->ports_names[i], sp_get_port_name(ports[i]));

            printf("\tFound port: '%s'\n", discoveredPorts->ports_names[i]);
        }
    }
    else {
        printf("Error looking for COM ports (%d) \n",(int)error);
        return RET_ERROR;
    }

    return RET_OK;
}


#if 0
void log_str(char *text, ...){
    va_list args;
    time_t raw_time;
    struct tm * time_info;
    time ( &raw_time );
    time_info = localtime ( &raw_time );
    int mill3 = current_timestamp() % 1000;

    flog = fopen(log_name, "a+"); // Opens a file for reading and appending.
    if (flog != NULL) {
        // Print timestamp
        fprintf(flog,"[%02d%02d%02d_%03d] ", time_info->tm_hour, time_info->tm_min,
                time_info->tm_sec, mill3);
        // Print the trace with the va arguments
        va_start(args, text);
        vfprintf(flog, text, args);
        va_end(args);
        // Close file
        fclose(flog);
    }else{
        printf("-> Error opening log File [%s]\n", log_name);
    }

    va_start(args, text);
    vprintf(text, args);
    va_end(args);
}
#endif