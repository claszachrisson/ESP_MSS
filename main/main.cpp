#include "sdkconfig.h"
#include <stdio.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_mss.h"
#include <string.h>

ESPMultiSpeakerSystem mss;

extern "C" void app_main(void)
{
    //init_nvs_flash();

    mss.init();

    //amp.play();
}