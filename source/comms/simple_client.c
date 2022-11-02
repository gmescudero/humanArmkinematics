/**
 * @file simple_client.c
 * @author German Moreno Escudero
 * @brief This file implements a simple client that receives form a server app
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "general.h"
#include "comms.h"
#include <signal.h>
#include <string.h>
#include <stdlib.h>

static volatile int keep_running = 1;

/**
 * @brief Interrupt handler used to correctly get out of a CTRL-C situation
 * 
 * @param sig (input) The signal that triggered the handler
 */
static void scom_interrupt_handler(int sig) {
    keep_running = 0;
}

/**
 * @brief Simple client setup. This client stays up until an error is triggered printing everything received
 * 
 * @param ip (input) IP to connect to
 * @param port (input) Port to connect to
 */
ERROR_CODE scom_simple_client(const char *ip, unsigned short port) {
    ERROR_CODE status;
    char buff[COM_BUFF_SIZE] = {'\0'};
    unsigned int bytes_num   = 0UL;

    log_str("Simple client initializing ...");

    struct sigaction act;
    act.sa_handler = scom_interrupt_handler;
    sigaction(SIGINT, &act, NULL);

    status = com_client_initialize(ip, port, 0);

    while (RET_OK == status && 1 == keep_running) {
        status = com_receive(buff,&bytes_num);
        if (RET_NO_EXEC == status) {
            status = RET_OK;
        }
        else {
            log_str("[%.0f] Received %u bytes: \n%s\n",sys_timestamp_get(),bytes_num,buff);
        }
        sleep_ms(100);
    }

    com_terminate();
    return status;
}

int main(int argc, char **argv) {
    ERROR_CODE status;
    char ip[32]   = COM_DEFAULT_IP;
    unsigned port = COM_DEFAULT_PORT;

    if (1 < argc && 32 > strlen(argv[1])) {
        strcpy(ip,argv[1]);
    }
    if (2 < argc && 8 > strlen(argv[2])) {
        port = (unsigned)atoi(argv[2]);
    }

    status = scom_simple_client(ip,port);

    return (RET_OK == status)? RET_OK:RET_ERROR;
}