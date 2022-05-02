
#include <string.h>
#include <libserialport.h>  /* READ COM ports before connect */
#include <time.h>
#include "general.h"

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

