/**
 * @file comms.c
 * @author German Moreno Escudero
 * @brief This file implements the communications through UDP
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "errors.h"


#define COM_BUFF_SIZE (1024)

/**
 * @brief Initialize UDP server
 * 
 * @param cli_ip (input) IP to connect to
 * @param port (input) Port in where to set server
 * @param timeout (input) Tiemout for receiving data. If 0, wait forever
 * @return ERROR_CODE 
 */
ERROR_CODE com_server_initialize(const char *cli_ip, unsigned short port, int timeout);
/**
 * @brief Initialize UDP client
 * 
 * @param svr_ip (input) IP to connect to
 * @param port (input) Port to connect to
 * @param timeout (input) Tiemout for receiving data. If 0, wait forever
 * @return ERROR_CODE 
 */
ERROR_CODE com_client_initialize(const char *svr_ip, unsigned short port, int timeout);
/**
 * @brief Data send
 * 
 * @param payload (input) Data buffer to send
 * @return ERROR_CODE 
 */
ERROR_CODE com_send(char payload[COM_BUFF_SIZE]);
/**
 * @brief String data build and send
 * 
 * @param text (input) Format string
 * @param ... (input) Variable args
 * @return ERROR_CODE 
 */
ERROR_CODE com_string_build_send(char *text, ...);
/**
 * @brief Data receive NON-BLOCKING
 * 
 * @param payload (output) Received data
 * @param slen (output) Number of bytes
 * @return ERROR_CODE 
 */
ERROR_CODE com_receive(char payload[COM_BUFF_SIZE], unsigned int *slen);
/**
 * @brief Data receive BLOCKING with timeout COM_READ_TIMEOUT_S seconds
 * 
 * @param payload (output) Received data
 * @param slen (output) Number of bytes
 * @return ERROR_CODE 
 */
ERROR_CODE com_receive_blocking(char payload[COM_BUFF_SIZE], unsigned int *slen);
/**
 * @brief Close connection
 * 
 */
void com_terminate();