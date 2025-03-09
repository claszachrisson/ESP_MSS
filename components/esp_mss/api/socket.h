#ifndef SOCKET_H
#define SOCKET_H

#include <cstdint>

int multicast_send(const uint8_t *buf, uint32_t len);
//void create_multicast_socket_receiver();
void create_multicast_socket_sender();

void udp_server_register_data_callback(void (*callback)(const uint8_t *buf, uint32_t len));
void multicast_receive_task(void *pvParameters);

#endif