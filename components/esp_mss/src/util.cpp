#include "util.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include <portmacro.h>

constexpr gpio_num_t led_gpio = GPIO_NUM_12;
bool led_state;

// Initialize NVS (Non-Volatile Storage)
void init_nvs_flash() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_led()
{
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
    led_state = false;
}

void toggle_led()
{
    led_state = !led_state;
    gpio_set_level(led_gpio, led_state);
    ESP_LOGI("led", "led state toggled, %d", led_state);
}

// I2C

