//
// Created by Clas Zachrisson on 2025-03-03.
//

#include "i2c_device.h"

constexpr uint32_t I2C_DEFAULT_100KHZ = 100000;
constexpr int DEFAULT_TIMEOUT = -1;

i2c_master_bus_handle_t i2c_bus;
i2c_master_dev_handle_t i2c_handle;

I2CDevice::I2CDevice(){
    this->get_default_device_config();
}

esp_err_t I2CDevice::i2c_init(){

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = -1,
        .sda_io_num = this->SDA_pin,
        .scl_io_num = this->SCL_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));

    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = this->device_address,
        .scl_speed_hz = I2C_DEFAULT_100KHZ,
    };
    const esp_err_t err = i2c_master_bus_add_device(i2c_bus, &cfg, &i2c_handle);
    if (err != ESP_OK) {
        ESP_LOGE(this->device_name, "I2C bus add failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t I2CDevice::probe_device(const uint8_t device_address) const
{
    const esp_err_t status = i2c_master_probe(i2c_bus, device_address, 10);
    if (status != ESP_OK) {
        ESP_LOGE(this->device_name, "Failed probing device: %x", status);
    }
    return status;
}
uint8_t register_read_byte(const uint8_t reg_addr) {
    uint8_t data[1];
    i2c_master_transmit_receive(i2c_handle, (uint8_t[]){reg_addr}, 1, data, 1, DEFAULT_TIMEOUT);
    return data[0];
}
esp_err_t I2CDevice::register_read_byte(const uint8_t reg_addr, uint8_t *dest_data) {
    return i2c_master_transmit_receive(i2c_handle, (uint8_t[]){reg_addr}, 1, dest_data, 1, DEFAULT_TIMEOUT);
}
esp_err_t register_read_bytes(const uint8_t reg_addr, uint8_t *dest_data, const size_t len) {
    return i2c_master_transmit_receive(i2c_handle, (uint8_t[]){reg_addr}, 1, dest_data, len, DEFAULT_TIMEOUT);
}


esp_err_t I2CDevice::register_write_byte(const uint8_t reg_addr, const uint8_t value) {
   return i2c_master_transmit(i2c_handle, (uint8_t[]){reg_addr, value}, 2, DEFAULT_TIMEOUT);
}

esp_err_t I2CDevice::register_write_bytes(const uint8_t reg_addr, const std::span<const uint8_t> bytes) {
    const size_t len = bytes.size() + 1;
    uint8_t buffer[len];
    buffer[0] = reg_addr;
    std::memcpy(&buffer[1], bytes.data(), bytes.size());

    return i2c_master_transmit(i2c_handle, buffer, len, DEFAULT_TIMEOUT);
}
esp_err_t I2CDevice::register_write_bytes(const uint8_t reg_addr, const uint8_t* bytes, const size_t len) {
    const size_t write_len = len + 1;
    uint8_t buffer[write_len];
    buffer[0] = reg_addr;
    std::memcpy(&buffer[1], bytes, len);

    return i2c_master_transmit(i2c_handle, buffer, len, DEFAULT_TIMEOUT);
}

void I2CDevice::register_write_random_bytes(const std::span<const i2c_pair> i2c_pairs) {
    for (const auto&[reg_addr, reg_val] : i2c_pairs)
    {
        if (reg_addr == 0xFF && reg_val == 0xFF) { vTaskDelay(5 / portTICK_PERIOD_MS); continue; }
        register_write_byte(reg_addr, reg_val);
    }
}