#include "esp_mss.h"
#include "amp.h"
#include "util.h"
#include "a2dp_sink.h"
#include "socket.h"
#include "wifi.h"

#ifdef CONFIG_MSS_DEVICE_MASTER
#define MSS_DEVICE_MASTER CONFIG_MSS_DEVICE_MASTER
#else
#define MSS_DEVICE_SLAVE CONFIG_MSS_DEVICE_SLAVE
#endif
//#undef MSS_DEVICE_MASTER
//#define MSS_DEVICE_SLAVE

void dummy_data_cb(const uint8_t *buf, uint32_t len)
{
    ESP_LOGI("main", "UDP server data cb called, length %d", len);
}

TAS5805M ESPMultiSpeakerSystem::amp;

ESPMultiSpeakerSystem::ESPMultiSpeakerSystem() {}

void ESPMultiSpeakerSystem::init() {
    init_nvs_flash();
    wifi_init_sta();
    init_led();
    toggle_led();

    amp.init();

#ifdef MSS_DEVICE_MASTER
    create_multicast_socket_sender();

    a2dp_sink_register_amp_data_cb(TAS5805M::data_cb);
    // Initialize A2DP sink
    a2dp_sink_init();
#else
    // Get audio from UDP to amp over I2S
    udp_server_register_data_callback(TAS5805M::data_cb);
    //udp_server_register_data_callback(dummy_data_cb);

    // Start listening on UDP multicast
    xTaskCreate(&multicast_receive_task, "mcast_rec_task", 4096, nullptr, 5, nullptr);

#endif
    amp.play();
}