//
// Created by Clas Zachrisson on 2025-03-03.
//

#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
//#include "driver/i2c_types.h"
#include "esp_log.h"
#include <string.h>
#include <cstring>
#include <span>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MSS_I2C_SCL_IO CONFIG_MSS_I2C_SCL_IO
#define MSS_I2C_SDA_IO CONFIG_MSS_I2C_SDA_IO
#define MSS_AMP_I2C_ADDRESS CONFIG_MSS_AMP_I2C_ADDRESS

struct i2c_pair {
    uint8_t reg_addr;
    uint8_t reg_val;
};

class I2CDevice {
public:
    I2CDevice();
protected:
    uint8_t device_address;
    gpio_num_t SCL_pin;
    gpio_num_t SDA_pin;

    void get_default_device_config()
    {
        this->device_address = static_cast<gpio_num_t>(MSS_AMP_I2C_ADDRESS);
        this->SCL_pin = static_cast<gpio_num_t>(MSS_I2C_SCL_IO);
        this->SDA_pin = static_cast<gpio_num_t>(MSS_I2C_SDA_IO);
    }
    esp_err_t i2c_init();
    esp_err_t probe_device(uint8_t device_address) const;
    uint8_t register_read_byte(uint8_t reg_addr);
    esp_err_t register_read_byte(uint8_t reg_addr, uint8_t *dest_data);
    esp_err_t register_read_bytes(uint8_t reg_addr, uint8_t *dest_data, size_t len);
    esp_err_t register_write_byte(uint8_t reg_addr, uint8_t value);

    esp_err_t register_write_bytes(uint8_t reg_addr, std::span<const uint8_t> bytes);
    esp_err_t register_write_bytes(uint8_t reg_addr, const uint8_t* bytes, size_t len);

    virtual void register_write_random_bytes(std::span<const i2c_pair> i2c_pairs);
private:
    static constexpr char device_name[] = "I2C";
};


#endif //I2C_DEVICE_H