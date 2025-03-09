#include "wifi.h"

#include <cstring>
#include <sys/types.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

//#define MSS_WIFI_SSID           CONFIG_MSS_WIFI_SSID
#define MSS_WIFI_PASS           CONFIG_MSS_WIFI_PASS
#define MSS_WIFI_MAXIMUM_RETRY  CONFIG_MSS_WIFI_MAXIMUM_RETRY

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

constexpr uint8_t WIFI_SSID_LEN = 32;
static uint8_t MSS_WIFI_SSID[WIFI_SSID_LEN] = CONFIG_MSS_WIFI_SSID;

static const char *STA_TAG = "wifi station";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

constexpr uint16_t AP_LIST_MAX_LENGTH = 8;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MSS_WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(STA_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(STA_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        ESP_LOGI(STA_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();


    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t max_ap_count = AP_LIST_MAX_LENGTH;
    wifi_ap_record_t ap_list[AP_LIST_MAX_LENGTH];
    uint16_t ap_count = 0;
    memset(ap_list, 0, sizeof(ap_list));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(STA_TAG, "wifi_init finished.");

    wifi_scan_config_t scan_config = {
        .ssid = &MSS_WIFI_SSID[0] // Scan only for this SSID
    };
    ESP_LOGI(STA_TAG, "wifi scan starting");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true)); // Blocking scan

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&max_ap_count, ap_list));
    ESP_LOGI(STA_TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, AP_LIST_MAX_LENGTH);

    // Find the best AP with the strongest signal
    int8_t best_rssi = -128;
    wifi_ap_record_t best_ap = {};

    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(STA_TAG, "Found AP: %s, RSSI: %d, Channel: %d",
                 ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);
        if (memcmp(ap_list[i].ssid, MSS_WIFI_SSID, WIFI_SSID_LEN) == 0) {  // Match SSID
            if (ap_list[i].rssi > best_rssi) {
                best_rssi = ap_list[i].rssi;
                best_ap = ap_list[i];
            }
        }
    }
    if (best_rssi == -128)
    {
        ESP_LOGW(STA_TAG, "No AP found with matching SSID: %s", MSS_WIFI_SSID);
        best_ap.primary = 0;
    }
    else ESP_LOGI(STA_TAG, "Best AP RSSI: %d, Channel: %d", best_ap.ssid, best_ap.rssi, best_ap.primary);


    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    wifi_config.sta = {};
    memcpy(wifi_config.sta.ssid, MSS_WIFI_SSID, WIFI_SSID_LEN);
    memcpy(wifi_config.sta.password, MSS_WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.channel = best_ap.primary;
    wifi_config.sta.threshold.rssi = -75;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    //ESP_LOGW(STA_TAG, "Wifi PS mode set to NONE");
    ESP_ERROR_CHECK(esp_wifi_connect() );


    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(STA_TAG, "connected to ap SSID:%s password:%s",
                 MSS_WIFI_SSID, MSS_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(STA_TAG, "Failed to connect to SSID:%s, password:%s",
                 MSS_WIFI_SSID, MSS_WIFI_PASS);
    } else {
        ESP_LOGE(STA_TAG, "UNEXPECTED EVENT");
    }
}