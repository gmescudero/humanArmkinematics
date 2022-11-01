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

#include <string.h> //memset
#include <unistd.h> //close;
#include <arpa/inet.h>
#include <sys/socket.h>
#include "comms.h"
#include "general.h"

#define COM_SOCK_ERROR (-1)

int socket_d = COM_SOCK_ERROR;
struct sockaddr_in socket_me;
struct sockaddr_in socket_other;
struct timeval read_timeout = {
    .tv_sec  = 0,
    .tv_usec = 0
};

ERROR_CODE com_server_initialize(const char *cli_ip, unsigned short port, int timeout) {    
    // Check status
    if (COM_SOCK_ERROR != socket_d) {
        err_str("Socket already initialized");
        return RET_RESOURCE_UNAVAILABLE;
    }
    
    // Create socket
    socket_d = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (COM_SOCK_ERROR == socket_d) {
        err_str("Failed to create UDP socket for server");
        return RET_ERROR;
    }

	// Zero out the structure and configure socket
	memset((char *) &socket_me, 0, sizeof(socket_me));

	socket_me.sin_family = AF_INET;
	socket_me.sin_port = htons(port);
	socket_me.sin_addr.s_addr = htonl(INADDR_ANY);

    memset((char *) &socket_other, 0, sizeof(socket_other));
    socket_other.sin_family = AF_INET;
    socket_other.sin_port = htons(port);
    if (0 == inet_aton(cli_ip , &socket_other.sin_addr)) {
        err_str("Failed to convert ip %s into binary");
        return RET_ERROR;
    }

    if (0 < timeout) {
        read_timeout.tv_sec = timeout;
        setsockopt(socket_d, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
    }

	// Bind socket to port
	if (COM_SOCK_ERROR == bind(socket_d , (struct sockaddr*)&socket_me, sizeof(socket_me))) {
        err_str("Failed to bind UDP socket for server");
        return RET_ERROR;
    }

    log_str("Created UDP server on port %d for ip %s",port,cli_ip);

    return RET_OK;
}

ERROR_CODE com_client_initialize(const char *srv_ip, unsigned short port, int timeout) {
    // Check status
    if (COM_SOCK_ERROR != socket_d) {
        err_str("Socket already initialized");
        return RET_RESOURCE_UNAVAILABLE;
    }
    
    // Create socket
    socket_d = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (COM_SOCK_ERROR == socket_d) {
        err_str("Failed to create UDP socket for client");
        return RET_ERROR;
    }

	// Zero out the structure and configure socket
    memset((char *) &socket_other, 0, sizeof(socket_other));
    socket_other.sin_family = AF_INET;
    socket_other.sin_port = htons(port);
    if (0 < timeout) {
        read_timeout.tv_sec = timeout;
        setsockopt(socket_d, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
    }

    if (0 == inet_aton(srv_ip , &socket_other.sin_addr)) {
        err_str("Failed to convert ip %s into binary");
        return RET_ERROR;
    }

    log_str("Created UDP client connected to ip %s and port %d",srv_ip,port);

    return RET_OK;
}

ERROR_CODE com_send(char payload[COM_BUFF_SIZE]) {
    int slen = sizeof(socket_other);

    // Check status
    if (COM_SOCK_ERROR == socket_d) {
        err_str("Socket not initialized");
        return RET_RESOURCE_UNAVAILABLE;
    }

    // Send the message
    if (COM_SOCK_ERROR == sendto(socket_d, (void*)payload, strlen(payload) , 0 , (struct sockaddr *) &socket_other, slen)) {
        err_str("Failed to send data");
        return RET_ERROR;
    }

    dbg_str("%s -> Sent %d bytes through socket",__FUNCTION__,slen);

    return RET_OK;
}

ERROR_CODE com_receive(char payload[COM_BUFF_SIZE], unsigned int *slen) {

    if (COM_SOCK_ERROR == recvfrom(socket_d, payload, COM_BUFF_SIZE, MSG_DONTWAIT, (struct sockaddr *) &socket_other, slen)) {
        dbg_str("%s -> Received no data",__FUNCTION__);
        return RET_NO_EXEC;
    }

    dbg_str("%s -> Received %d bytes through socket",__FUNCTION__,slen);

    return RET_OK;
}

ERROR_CODE com_receive_blocking(char payload[COM_BUFF_SIZE], unsigned int *slen) {

    // Clean buffer
    memset(payload, '\0', COM_BUFF_SIZE);

    if (COM_SOCK_ERROR == recvfrom(socket_d, payload, COM_BUFF_SIZE, 0, (struct sockaddr *) &socket_other, slen)) {
        err_str("Failed to receive data or timeout expired (%f seconds)", 
            (double)read_timeout.tv_sec+(1e-6 * (double)read_timeout.tv_usec));
        return RET_ERROR;
    }

    dbg_str("%s -> Received %d bytes through socket",__FUNCTION__,slen);

    return RET_OK;
}

void com_terminate() {
    if (COM_SOCK_ERROR != socket_d) {
        log_str("Terminating UDP socket connection");
        shutdown(socket_d, SHUT_RDWR);
        close(socket_d);
        socket_d = COM_SOCK_ERROR;
    }
}