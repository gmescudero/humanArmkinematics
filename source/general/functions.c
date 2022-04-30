
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <libserialport.h>  /* READ COM ports before connect */
#include <time.h>
#include "functions.h"

struct sp_port **ports;
TRACE_LEVEL trace_level = INFO;

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
        log_str("Ports found: ");
        /* Show all COM ports */
        for(int i = 0; ports[i]; i++){
            /* Increment COM ports total number of ports */
            discoveredPorts->ports_number++;
            /* Copy the port name to the given structure */
            strcpy(discoveredPorts->ports_names[i], sp_get_port_name(ports[i]));

            log_str("\tFound port: '%s'", discoveredPorts->ports_names[i]);
        }
    }
    else {
        log_str("Error looking for COM ports (%d) ",(int)error);
        return RET_ERROR;
    }

    return RET_OK;
}


ERROR_CODE trace_level_set(TRACE_LEVEL lvl) {
    /* Check arguments */
    if (NONE > lvl || NUMBER_OF_LEVELS <= lvl) return RET_ARG_ERROR;

    trace_level = lvl;
    return RET_OK;
}

void dbg_str(const char *text, ...){
    va_list args;

    if (DEBUG <= trace_level) {
        va_start(args, text);
        printf("[DEBUG  ] ");
        vprintf(text, args);
        printf("\n");
        va_end(args);
    }
}

void log_str(const char *text, ...){
    va_list args;

    if (INFO <= trace_level) {
        va_start(args, text);
        printf("[INFO   ] ");
        vprintf(text, args);
        printf("\n");
        va_end(args);
    }
}

void wrn_str(const char *text, ...){
    va_list args;

    if (WARNING <= trace_level) {
        va_start(args, text);
        printf("[WARNING] ");
        vprintf(text, args);
        printf("\n");
        va_end(args);
    }
}

void err_str(const char *text, ...){
    va_list args;

    if (ERROR <= trace_level) {
        va_start(args, text);
        printf("[ERROR  ] ");
        vprintf(text, args);
        printf("\n");
        va_end(args);
    }
}