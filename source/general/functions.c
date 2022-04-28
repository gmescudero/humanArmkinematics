
#include <stdio.h>
#include <stdarg.h>
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

void log_str(char *text, ...){
    va_list args;

    va_start(args, text);
    printf("[INFO   ] ");
    vprintf(text, args);
    printf("\n");
    va_end(args);
}

void wrn_str(char *text, ...){
    va_list args;

    va_start(args, text);
    printf("[WARNING] ");
    vprintf(text, args);
    printf("\n");
    va_end(args);
}

void err_str(char *text, ...){
    va_list args;

    va_start(args, text);
    printf("[ERROR  ] ");
    vprintf(text, args);
    printf("\n");
    va_end(args);
}