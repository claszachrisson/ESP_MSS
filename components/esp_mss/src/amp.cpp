#include "amp.h"

using std::array;

#define TIMER_PERIOD_MS    10000

i2s_chan_handle_t TAS5805M::tx_handle;

static const char *TAS_TAG = "TAS5805M";
uint8_t data_32bit[8192]{};

const array<i2c_pair, 30> TAS5805M::init_sequence = {{
	{TAS_PROG_CMD , TAS_PROG_DELAY},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO },
	{TAS_DEVICE_CTRL_2 , TAS_POWER_STATE_HIZ_DSP_DISABLED },
	{TAS_PROG_DELAY , TAS_PROG_DELAY},
	{TAS_RESET_CTRL, TAS_RESET_ALL },
	{TAS_PROG_CMD, TAS_PROG_DELAY},
	{TAS_DEVICE_CTRL_2 , TAS_POWER_STATE_HIZ },
	{TAS_PROG_CMD, TAS_PROG_DELAY},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO},
	{TAS_SAP_CTRL1, TAS_DATA_FORMAT_LTJ},
	{TAS_DIG_VOL_CTRL, 0x54}, // -18dB
	//{GOTO_BK , TAS_ZERO },
	{TAS_DEVICE_CTRL_2 , TAS_POWER_STATE_HIZ },
	{TAS_PROG_CMD , TAS_PROG_DELAY},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO },
	{TAS_DSP_MISC, ( TAS_DSP_UNIQUE_EQ | TAS_DSP_BYPASS_FIR | TAS_DSP_BYPASS_DRC ) },
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_BK_BQ},
	{TAS_GOTO_PG , TAS_PG_BQ_LEFT_1},
	{TAS_PROG_CMD , TAS_PROG_JUMP_LEFTBQ1}, // Program will jump at this point to write BQs and come back once that's done
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_BK_BQ},
	{TAS_GOTO_PG , TAS_PG_BQ_RIGHT_1},
	{TAS_PROG_CMD , TAS_PROG_JUMP_RIGHTBQ1},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO},
	{TAS_FAULT_CLEAR, TAS_ANALOG_FAULT_CLEAR },
	{TAS_PROG_CMD, TAS_PROG_DELAY},
}};

const array<i2c_pair, 4> TAS5805M::set_hiz_state_sequence= {{
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO},
	{TAS_DEVICE_CTRL_2 , TAS_POWER_STATE_HIZ},
	{TAS_PROG_DELAY , TAS_PROG_DELAY},
}};
const array<i2c_pair, 4> TAS5805M::set_play_state_sequence = {{
	{TAS_PROG_DELAY , TAS_PROG_DELAY},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO},
	{TAS_DEVICE_CTRL_2 , TAS_POWER_STATE_PLAY},
}};

const uint8_t TAS5805M::pg0_bk0_sequence[][2] = {
	{TAS_PROG_DELAY , TAS_PROG_DELAY},
	{TAS_GOTO_PG , TAS_ZERO},
	{TAS_GOTO_BK , TAS_ZERO},
	{TAS_GOTO_PG , TAS_ZERO},
};

/*array<uint8_t, 40> TAS5805M::dsp_bq_right = {{
	0x08, 0x05, 0x28, 0x6d, // BQ 1 B0
	0xf0, 0x05, 0xf2, 0x41, //		B1
	0x07, 0xf5, 0x2b, 0x44, //		B2
	0x0f, 0xfa, 0x0d, 0xbf, //		A1
	0xf8, 0x05, 0xac, 0x4f, //		A2
	0x06, 0xf5, 0xc8, 0x36, // BQ 2 B0
	0xf2, 0x14, 0x6f, 0x94, //		B1
	0x06, 0xf5, 0xc8, 0x36, //		B2
	0x0f, 0xd7, 0x1a, 0x59, //		A1
	0xf8, 0x28, 0x23, 0x13  //		A2
}};*/
array<uint8_t, 40> TAS5805M::dsp_bq_right = {{
	0x07, 0xf6, 0x8b, 0x33, // BQ 1 B0
	0xf0, 0x12, 0xe9, 0x9a, //		B1
	0x07, 0xf6, 0x8b, 0x33, //		B2
	0x0f, 0xed, 0x0b, 0x39, //		A1
	0xf8, 0x12, 0xde, 0x4f, //		A2
	0x08, 0x00, 0x00, 0x00, // BQ 2 B0
	0x00, 0x00, 0x00, 0x00, //		B1
	0x00, 0x00, 0x00, 0x00, //		B2
	0x00, 0x00, 0x00, 0x00, //		A1
	0x00, 0x00, 0x00, 0x00  //		A2
}};

array<uint8_t, 20> TAS5805M::dsp_bq_left = {{
	0x07, 0xa3, 0x60, 0x91, // BQ 1 B0
	0xf0, 0xb9, 0x3e, 0xde, //		B1
	0x07, 0xa3, 0x60, 0x91, //		B2
	0x0f, 0x42, 0x90, 0x36, //		A1
	0xf8, 0xb5, 0x0d, 0xf2, //		A2
}};



TAS5805M::TAS5805M()
{
	this->get_default_config();
}

TAS5805M_err_t TAS5805M::init() {
	// Power on the TAS5805M
	gpio_set_direction(this->PDN_pin, GPIO_MODE_OUTPUT);
	gpio_set_level(this->PDN_pin, 1);
	vTaskDelay(xDelay);

	Biquad BQ1 = Biquad(Biquad::BQ_TYPE_HPF, 1000.0, Biquad::Q707, 96000);
	this->i2c_init();

	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	i2s_new_channel(&chan_cfg, &tx_handle, nullptr);
	i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);

	i2s_std_config_t std_cfg = {
		.clk_cfg = { //I2S_STD_CLK_DEFAULT_CONFIG(TAS5805M::sample_rate),
			.sample_rate_hz = this->sample_rate,
			.clk_src = I2S_CLK_SRC_APLL,
			.mclk_multiple = I2S_MCLK_MULTIPLE_1152
		},
		.slot_cfg = slot_cfg,
		.gpio_cfg = {
			.mclk = I2S_GPIO_UNUSED,
			.bclk = GPIO_NUM_18,
			.ws = GPIO_NUM_23,
			.dout = GPIO_NUM_5,
			.din = I2S_GPIO_UNUSED,
			.invert_flags = {
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv = false,
			},
		},
	};
	i2s_channel_init_std_mode(tx_handle, &std_cfg);
	i2s_channel_enable(tx_handle);
	
	// Check I2C connection
	if (this->probe_device(this->device_address) != ESP_OK) {
		bool device_found = false;
		ESP_LOGW(TAS_TAG, "Device did not respond at 0x%02X", this->device_address);
		for(int add=this->device_address-2;add<this->device_address+2;add++) {
			if (this->probe_device(this->device_address) == ESP_OK) {
				ESP_LOGI(TAS_TAG, "Device found at address 0x%02X", add);
				this->device_address = add;
				device_found = true;
				break;
			}
		}
		if (!device_found)
		{
			ESP_LOGE(TAS_TAG, "Device not found, aborting initialisation");
			return TAS5805M_DEVICE_NOT_FOUND;
		}
	}
	
	this->register_write_random_bytes(init_sequence);

	TAS5805M_err_t err;
	if ((err = this->error_check()) != TAS5805M_OK) {
		ESP_LOGE(TAS_TAG, "Failed to initialize TAS5805M");
	} else {
		if (TimerHandle_t timer = xTimerCreate("TAS_err_check_timer", pdMS_TO_TICKS(TIMER_PERIOD_MS), pdTRUE, this, error_check_timer_cb); timer != nullptr) {
			// Start the timer
			xTimerStart(timer, 0);
		} else {
			ESP_LOGE(TAS_TAG, "Failed to create error checking timer");
		}
		ESP_LOGI(TAS_TAG, "Successfully initialized TAS5805M");
	}
	return err;
}

TAS5805M_err_t TAS5805M::error_check() {
	uint8_t ch_fault = 0xFF;
	uint8_t glb_fault1 = 0xFF;
	uint8_t glb_fault2 = 0xFF;
	if((register_read_byte(TAS_CHAN_FAULT, &ch_fault) |
			register_read_byte(TAS_GLOBAL_FAULT_1, &glb_fault1) |
			register_read_byte(TAS_GLOBAL_FAULT_2, &glb_fault2)) == TAS5805M_OK) {
		if((ch_fault | glb_fault1 | glb_fault2) == 0x00) { 
			ESP_LOGI(TAS_TAG, "Error check GOOD");
			return TAS5805M_OK;
		}
	}
	ESP_LOGE(TAS_TAG, "Error check CHAN_FAULT: %d ", ch_fault);
	ESP_LOGE(TAS_TAG, "Error check GLOBAL_FAULT_1: %d ", glb_fault1);
	ESP_LOGE(TAS_TAG, "Error check GLOBAL_FAULT_2: %d ", glb_fault2);
	return TAS5805M_FAIL;
}

TAS5805M_err_t TAS5805M::goto_state(const uint8_t state){
	uint8_t data;
	switch(state){
		case TAS_POWER_STATE_HIZ:
			this->register_write_random_bytes(set_hiz_state_sequence);
			break;
		case TAS_POWER_STATE_PLAY:
			this->register_write_random_bytes(set_play_state_sequence);
			break;
		default:
			return TAS5805M_INVALID_ARGUMENT;
	}
	if(this->register_read_byte(TAS_POWER_STATE, &data) != ESP_OK) return TAS5805M_I2C_READ_ERROR;
	if(data != state) return TAS5805M_FAIL;
	return TAS5805M_OK;
}

TAS5805M_err_t TAS5805M::play(){
	return this->goto_state(TAS_POWER_STATE_PLAY);
}

/*TAS5805M_err_t TAS5805M::set_digital_vol(uint32_t vol) {
	//TAS5805M::random_write(TAS5805M::pg0_bk0_sequence, sizeof(TAS5805M::pg0_bk0_sequence) / sizeof(TAS5805M::pg0_bk0_sequence[0]));
	uint8_t bytes[4];
	bytes[0] = (vol >> 24) & 0xFF; // MSB
	bytes[1] = (vol >> 16) & 0xFF;
	bytes[2] = (vol >> 8) & 0xFF;
	bytes[3] = vol & 0xFF; // LSB
	ESP_LOGI(TAS_TAG, "Setting digital volume to %d, %d, %d, %d", bytes[0], bytes[1], bytes[2], bytes[3]);
	register_write_byte(GOTO_PG, 0x00);
	register_write_byte(GOTO_BK, 0x8C);
	register_write_byte(GOTO_PG, 0x2A);
	register_write_bytes(0x24, bytes, (size_t)4);
	register_write_bytes(0x28, bytes, (size_t)4);
	register_write_random_bytes(pg0_bk0_sequence);
	return TAS5805M_OK;
}*/

void TAS5805M::data_cb(const uint8_t* data, const uint32_t len) {
	size_t bytes_written;
	i2s_channel_write(tx_handle, data, len, &bytes_written, 100);
}
void TAS5805M::register_write_random_bytes(const std::span<const i2c_pair> i2c_pairs) {
	for (const auto&[reg_addr, reg_val] : i2c_pairs)
	{
		if (reg_addr == TAS_PROG_CMD)
		{
			switch (reg_val)
			{
			case TAS_PROG_DELAY:
				vTaskDelay(5 / portTICK_PERIOD_MS);
				break;
			case TAS_PROG_JUMP_LEFTBQ1:
				this->register_write_bytes(TAS_DSP_BQ_LEFT1, dsp_bq_left);
				break;
			case TAS_PROG_JUMP_RIGHTBQ1:
				this->register_write_bytes(TAS_DSP_BQ_RIGHT1, dsp_bq_right);
				break;
			default:
				ESP_LOGW(TAS_TAG, "Unhandled PROG_CMD %02X", reg_val);
			}
		}
		else
		{
			register_write_byte(reg_addr, reg_val);
		}
	}
}

void TAS5805M::error_check_task(void *pvParameter) {
	// Add your code here for the heavy work
	auto *self = static_cast<TAS5805M*>(pvParameter);
	self->error_check();
	vTaskDelete(nullptr);  // Delete the task when done
}

void TAS5805M::error_check_timer_cb(TimerHandle_t xTimer) {
	auto *self = static_cast<TAS5805M*>(pvTimerGetTimerID(xTimer));
	ESP_LOGI("APP_MAIN", "Error checking TAS...");
	xTaskCreate(&TAS5805M::error_check_task, "TAS_err_check_task", 4096, self, 0, nullptr);
}