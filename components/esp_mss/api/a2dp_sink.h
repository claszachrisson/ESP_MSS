#ifndef A2DP_SINK
#define A2DP_SINK

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

// Must be called before a2dp_sink_init()
void a2dp_sink_register_amp_data_cb(void (*data_cb)(const uint8_t *buf, uint32_t len));
void a2dp_sink_init();
void a2dp_sink_register_abs_vol_cb(void (*abs_vol_callback)(uint8_t vol));



#endif