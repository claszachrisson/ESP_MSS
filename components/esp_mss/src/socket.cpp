#include "socket.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_attr.h"
#include "util.h"

#define UDP_PORT                3333
#define MULTICAST_TTL           64
#define MULTICAST_IPV4_ADDR     "232.10.0.1"
//#define MULTICAST_IPV4_ADDR     "232.10.11.12"
//#define MULTICAST_IPV4_ADDR     "192.168.86.88"

static int multicast_socket;
struct sockaddr_in dest_addr = {0};
struct sockaddr_in saddr = {0};

static const char *UDP_TAG = "socket";
constexpr uint16_t CHUNK_SIZE = 2048;
DMA_ATTR uint8_t recvbuf[CHUNK_SIZE];
//void (*udp_client_data_cb)(const uint8_t *buf, uint32_t len);

//#ifdef CONFIG_MSS_DEVICE_SLAVE
void (*udp_server_data_cb)(const uint8_t *buf, uint32_t len);
void udp_server_register_data_callback(void (*callback)(const uint8_t *buf, uint32_t len)) {
    udp_server_data_cb = callback;
}
//#endif

static int socket_add_ipv4_multicast_group()
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;

    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(UDP_TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
        goto err;
    }
    ESP_LOGI(UDP_TAG, "Configured IPV4 Multicast address %s", inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(UDP_TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }

    err = setsockopt(multicast_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        goto err;
    }

 err:
    return err;
}

static void create_multicast_ipv4_socket()
{
    int err = 0;

    multicast_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (multicast_socket < 0) {
        ESP_LOGE(UDP_TAG, "Failed to create socket. Error %d", errno);
    }
    //int err = 0;

    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(multicast_socket, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "Failed to bind socket. Error %d", errno);
    }

    // Get the ESP32's Wi-Fi interface address
    if (esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(netif, &ip_info);
        err = setsockopt(multicast_socket, IPPROTO_IP, IP_MULTICAST_IF, &ip_info.ip, sizeof(ip_info.ip));
        if (err < 0) {
            ESP_LOGE(UDP_TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
        }
    } else {
        ESP_LOGE(UDP_TAG, "Failed to get Wi-Fi interface");
    }
    // // Assign the IPv4 multicast source interface, via its IP
    // // (only necessary if this socket is IPV4 only)
    // err = setsockopt(multicast_socket, IPPROTO_IP, IP_MULTICAST_IF, &saddr.sin_addr,
    //                  sizeof(saddr.sin_addr));
    // if (err < 0) {
    //     ESP_LOGE(UDP_TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
    // }
}

void create_multicast_socket_sender() {
    create_multicast_ipv4_socket();
    socket_add_ipv4_multicast_group();

    int send_buf_size = 8192;  // Increase buffer size

    int err = 0;
    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = MULTICAST_TTL;
    err = setsockopt(multicast_socket, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        goto err;
    }

    // set destination multicast addresses for sending from these sockets
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    err = inet_aton(MULTICAST_IPV4_ADDR, &dest_addr.sin_addr.s_addr);
    if (err != 1) {
        ESP_LOGE(UDP_TAG, "Configured IPV4 multicast address '%s' is invalid.", MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        //err = -1;
        goto err;
    }
    ESP_LOGI(UDP_TAG, "Configured IPV4 Multicast address %s", inet_ntoa(dest_addr.sin_addr.s_addr));
    if (!IP_MULTICAST(ntohl(dest_addr.sin_addr.s_addr))) {
        ESP_LOGW(UDP_TAG, "Configured IPV4 multicast address '%s' is not a valid multicast address. This will probably not work.", MULTICAST_IPV4_ADDR);
    }
    err = setsockopt(multicast_socket, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));
    if (err != 1)
    {
        ESP_LOGW(UDP_TAG, "Failed to set SO_SNDBUF. Error %d", errno);
    }
    return;
err:
    close(multicast_socket);
}

void create_multicast_socket_receiver() {
    create_multicast_ipv4_socket();
    int err = 0;

    err = socket_add_ipv4_multicast_group();
    if (err < 0) {
        goto err;
    }
    return;
err:
    close(multicast_socket);
}

int multicast_send(const uint8_t *buf, const uint32_t len) {
    int err = 0;
    /*uint32_t offset = 0;

    while (offset < len) {
        err = sendto(multicast_socket, buf + offset, CHUNK_SIZE, 0,
                     (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(UDP_TAG, "IPV4 sendto failed. errno: %d", errno);
            return err;
        }
        offset += CHUNK_SIZE;
    }*/
    err = sendto(multicast_socket, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "IPV4 sendto failed. errno: %d", errno);
    }
    //ESP_LOGI(UDP_TAG, "IPV4 sendto succeeded with len: %d", len);
    return err;
}

void multicast_receive_task(void *pvParameters)
{
    while (true) {
        create_multicast_socket_receiver();
        if (multicast_socket < 0) {
            ESP_LOGE(UDP_TAG, "Failed to create IPv4 multicast socket");
            // Nothing to do!
            vTaskDelay(5 / portTICK_PERIOD_MS);
            continue;
        }

        // Loop waiting for UDP received, and sending UDP packets if we don't
        // see any.
        int err = 1;
        while (err > 0) {
            struct timeval tv = {
                .tv_sec = 2,
                .tv_usec = 0,
            };
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(multicast_socket, &rfds);

            int s = select(multicast_socket + 1, &rfds, NULL, NULL, &tv);
            if (s < 0) {
                ESP_LOGE(UDP_TAG, "Select failed: errno %d", errno);
                err = -1;
                break;
            }
            if (s > 0) {
                if (FD_ISSET(multicast_socket, &rfds)) {
                    // Incoming datagram received
                    //uint8_t recvbuf[4096];

                    struct sockaddr_storage raddr; // Large enough for both IPv4 or IPv6
                    socklen_t socklen = sizeof(raddr);
                    int len = recvfrom(multicast_socket, recvbuf, sizeof(recvbuf), 0,
                                       (struct sockaddr *)&raddr, &socklen);
                    if (len < 0) {
                        ESP_LOGE(UDP_TAG, "multicast recvfrom failed: errno %d", errno);
                        err = -1;
                        break;
                    }
                    //ESP_LOGI(UDP_TAG, "received %d bytes", len);
                    udp_server_data_cb(recvbuf, len);
                    toggle_led();
                }
            }
        }

        ESP_LOGE(UDP_TAG, "Shutting down socket and restarting...");
        shutdown(multicast_socket, 0);
        close(multicast_socket);
    }
}


/*
static void udp_socket_init() {
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    udp_socket = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket < 0) {
        ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(UDP_TAG, "Socket created");

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt (udp_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
}

static void udp_socket_deinit() {
    ESP_LOGE(UDP_TAG, "Shutting down socket");
    shutdown(udp_socket, 0);
    close(udp_socket);
}

void udp_socket_transmit_data(const uint8_t *buf, uint32_t len) {
    ESP_LOGI(UDP_TAG, "Sending %lu bytes", len);
    if (udp_socket < 0) {
        ESP_LOGE(UDP_TAG, "Socket not initialized!");
        return;
    }
    //const char *payload = "Message from ESP32";
    //int err = sendto(udp_socket, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    int err = sendto(udp_socket, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
        return;
    }
    ESP_LOGI(UDP_TAG, "Message sent");
}

static void udp_server_task(void *pvParameters) {
    uint8_t rx_buf[4096];


    while (1) {
        udp_socket_init();

        int err = bind(udp_socket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(UDP_TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        // Receive data
        while (1) {
            ESP_LOGI(UDP_TAG, "Waiting for data");

            int len = recvfrom(udp_socket, rx_buf, sizeof(rx_buf) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(UDP_TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer); * /

                udp_server_data_cb(rx_buf, len);
            }
        }

        if (udp_socket != -1) {
            ESP_LOGE(UDP_TAG, "Shutting down socket and restarting...");
            shutdown(udp_socket, 0);
            close(udp_socket);
        }
    }
    vTaskDelete(NULL);
}
*/