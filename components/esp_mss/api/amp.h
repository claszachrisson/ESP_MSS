#ifndef TAS5805M_H
#define TAS5805M_H

#include <cstdint>
#include "driver/gpio.h"
#include "util.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
//#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "i2c_device.h"
#include <array>
#include "biquad.h"

using std::array;

#define MSS_AMP_PDN_PIN CONFIG_MSS_AMP_PDN_PIN
#define MSS_I2S_LRCK_PIN CONFIG_MSS_I2S_LRCK_PIN
#define MSS_I2S_SCLK_PIN CONFIG_MSS_I2S_SCLK_PIN
#define MSS_I2S_SDO_PIN CONFIG_MSS_I2S_SDO_PIN
#define MSS_I2S_SAMPLE_RATE CONFIG_MSS_I2S_SAMPLE_RATE

enum TAS5805M_err_t {
    TAS5805M_OK = 0,
    TAS5805M_FAIL = 1, // Generic TAS5805M_err_t code indicating unknown error
    TAS5805M_INVALID_ARGUMENT = 2,
    TAS5805M_BQ_INIT_ERROR = 3,
    TAS5805M_BQ_WRITE_ERROR = 4,
    TAS5805M_BOOK_ERROR = 5,
    TAS5805M_I2C_READ_ERROR = 6,
    TAS5805M_DEVICE_NOT_FOUND = 7,
};
enum TAS5805M_control_mask_t {
    TAS5805M_BQ_WRITE_FAIL_MASK = 0x40,
};

enum control_port_register {
    TAS_GOTO_PG = 0x00,
    TAS_RESET_CTRL = 0x01,
    TAS_DEVICE_CTRL_1 = 0x02,
    TAS_DEVICE_CTRL_2 = 0x03,
    TAS_SIG_CH_CTRL = 0x28,
    TAS_SAP_CTRL1 = 0x33,
    TAS_FS_MON = 0x37,
    TAS_CLKDET_STATUS = 0x39,
    TAS_DIG_VOL_CTRL = 0x4C,
    TAS_AUTO_MUTE_CTRL = 0x50,
    TAS_AUTO_MUTE_TIME = 0x51,
    TAS_DSP_MISC = 0x66,
    TAS_POWER_STATE = 0x68,
    TAS_CHAN_FAULT = 0x70,
    TAS_GLOBAL_FAULT_1 = 0x71,
    TAS_GLOBAL_FAULT_2 = 0x72,
    TAS_FAULT_CLEAR = 0x78,
    TAS_GOTO_BK = 0x7f,
    TAS_PROG_CMD = 0xFF
};

enum control_port_state
{
    TAS_ZERO = 0x00,
    TAS_BK_BQ = 0xAA,
    TAS_PG_BQ_LEFT_1 = 0x24,
    TAS_PG_BQ_RIGHT_1 = 0x26,
    TAS_POWER_STATE_PLAY = 0x03,
    TAS_POWER_STATE_HIZ = 0x02,
    TAS_POWER_STATE_SLEEP = 0x01,
    TAS_POWER_STATE_DEEP_SLEEP = 0x00,
    TAS_POWER_STATE_HIZ_DSP_DISABLED = 0x12,
    TAS_ANALOG_FAULT_CLEAR = 0x80,
    TAS_RESET_ALL = 0x11,
    TAS_DIG_VOL_NEG_24_DB = 0x48,
    TAS_DSP_BYPASS_EQ = 0x01,
    TAS_DSP_BYPASS_DRC = 0x02,
    TAS_DSP_BYPASS_FIR = 0x04,
    TAS_DSP_UNIQUE_EQ = 0x08,
    TAS_DSP_BQ_LEFT1 = 0x18,
    TAS_DSP_BQ_LEFT2 = 0x2C,
    TAS_DSP_BQ_LEFT3 = 0x40,
    TAS_DSP_BQ_RIGHT1 = 0x54,
    TAS_AUTO_MUTE_DISABLED = 0x80,
    TAS_AUTO_MUTE_ENABLED = 0x07,
    TAS_AUTO_MUTE_5S = 0x77,
    TAS_SIG_CH_FS_MODE_44_1 = 0x08,
    TAS_SIG_CH_BCK_32FS = 0x30,
    TAS_SIG_CH_BCK_64FS = 0x50,
    TAS_WORD_LENGTH_24_BIT = 0x02,
    TAS_WORD_LENGTH_32_BIT = 0x03,
    TAS_DATA_FORMAT_LTJ = 0x30,
    TAS_PROG_JUMP_LEFTBQ1 = 0xFA,
    TAS_PROG_JUMP_RIGHTBQ1 = 0xFB,
    TAS_PROG_DELAY = 0xFF,
};

class TAS5805M : public I2CDevice {
public:
    TAS5805M();

    TAS5805M_err_t init();
    TAS5805M_err_t play();
    TAS5805M_err_t error_check();
    //TAS5805M_err_t write_bq_coeffs();
    //TAS5805M_err_t set_digital_vol(uint32_t vol);
    static void data_cb(const uint8_t *data, uint32_t len);

private:
    static i2s_chan_handle_t tx_handle;
    static constexpr TickType_t xDelay = 20 / portTICK_PERIOD_MS;

    gpio_num_t PDN_pin;
    gpio_num_t LRCK_pin;
    gpio_num_t SCLK_pin;
    gpio_num_t SDO_pin;
    uint32_t sample_rate;
    uint32_t dsp_sample_rate = 96000;


    static const array<i2c_pair, 30> init_sequence;

    static const array<i2c_pair, 4> set_play_state_sequence;
    static const array<i2c_pair, 4> set_hiz_state_sequence;
    static const uint8_t pg0_bk0_sequence[][2];

    static array<uint8_t, 20> dsp_bq_left;
    static array<uint8_t, 40> dsp_bq_right;

    void get_default_config()
    {
        this->PDN_pin = static_cast<gpio_num_t>(MSS_AMP_PDN_PIN);
        this->LRCK_pin = static_cast<gpio_num_t>(MSS_I2S_LRCK_PIN);
        this->SDO_pin = static_cast<gpio_num_t>(MSS_I2S_SDO_PIN);
        this->sample_rate = static_cast<gpio_num_t>(MSS_I2S_SAMPLE_RATE);
    }
    TAS5805M_err_t goto_state(uint8_t state);
    TAS5805M_err_t random_write(const uint8_t data[][2], size_t length);
    void register_write_random_bytes(std::span<const i2c_pair> i2c_pairs) override;
    TAS5805M_err_t read_register(uint8_t reg, uint8_t *data);
    static void error_check_timer_cb(TimerHandle_t xTimer);
    static void error_check_task(void *pvParameter);
};
#endif