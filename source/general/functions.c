#include <unistd.h>
#include <string.h>
#include <libserialport.h>  /* READ COM ports before connect */
#include <time.h>
#include "general.h"


void sleep_s(int seconds) {
    sleep(seconds);
}

void sleep_ms(int millis) {
    struct timespec ts;
    struct timespec remaining;
    int res;

    ts.tv_sec = millis / 1000;
    ts.tv_nsec = (millis % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &remaining);
    } while (res);
}

ERROR_CODE com_ports_list(COM_PORTS *discoveredPorts) {
    struct sp_port **ports;
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
        if (0 == discoveredPorts->ports_number) {
            log_str("\tNone");
            return RET_NO_EXEC;
        }
    }
    else {
        err_str("Error looking for COM ports (%d) ",(int)error);
        return RET_ERROR;
    }

    return RET_OK;
}

