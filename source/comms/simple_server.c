/**
 * @file simple_server.c
 * @author German Moreno Escudero
 * @brief This file implements a simple server that receives form a server app
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "general.h"
#include "comms.h"
#include <signal.h>

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
 * @brief Simple server setup. This server stays up until an error is triggered printing everything received
 * 
 * @param ip (input) IP to connect to
 * @param port (input) Port to connect to
 */
ERROR_CODE scom_simple_server(const char *ip, unsigned short port) {
    ERROR_CODE status;
    char buff[COM_BUFF_SIZE] = {'\0'};
    unsigned int bytes_num   = 0UL;

    log_str("Simple server initializing ...");

    struct sigaction act;
    act.sa_handler = scom_interrupt_handler;
    sigaction(SIGINT, &act, NULL);

    status = com_server_initialize(ip, port, 0);

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

    status = scom_simple_server("127.0.0.1",1234);

    return (RET_OK == status)? RET_OK:RET_ERROR;
}