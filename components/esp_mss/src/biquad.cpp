//
// Created by Clas Zachrisson on 2025-03-09.
//
#include "biquad.h"
#include "esp_log.h"
#include <algorithm>

const char* TAG = "Biquad";

double Biquad::clamp_min = std::numeric_limits<int32_t>::min();
double Biquad::clamp_max = std::numeric_limits<int32_t>::max();
double Biquad::Q707 = 0.707106781186547;

Biquad::Biquad(int type, double char_freq, double Q_factor, int sample_freq=96000) {
    this->type = type;
    this->char_freq = char_freq;
    this->sample_freq = sample_freq;
    this->Q_factor = Q_factor;
    switch (this->type)
    {
      case BQ_TYPE_HPF:
        {
            const double w0 = 2.0*M_PI*this->char_freq/this->sample_freq;
            const double alpha=sin(w0)/(2.0*this->Q_factor);
            ESP_LOGD(TAG, "w0: %lf, alpha: %lf", w0, alpha);
            double a0 = 1 + alpha;
            this->coefficients[0] =   (1 + cos(w0))/2 / a0; // b0
            this->coefficients[1] =  -(1 + cos(w0))   / a0; // b1
            this->coefficients[2] =   (1 + cos(w0))/2 / a0; // b2
            this->coefficients[3] =  -(2 * cos(w0))   / a0; // a1
            this->coefficients[4] =   (1 - alpha)     / a0; // a2
            break;
        }
      default:
        ESP_LOGE(TAG, "Invalid Filter type");
    }
    ESP_LOGD(TAG, "Biquad coefficients: %.12f \t %.12f \t %.12f \t %.12f \t %.12f",
        this->coefficients[0],this->coefficients[1],this->coefficients[2],this->coefficients[3],this->coefficients[4]);
    this->coefficients[3] = -this->coefficients[3]; // a1 and a2 must be negated
    this->coefficients[4] = -this->coefficients[4];
    this->convert_to_dsp_format();
}

void Biquad::convert_to_dsp_format() {
    int32_t intermediates[5]{};
    int dsp_i=0;
    for (int i = 0; i < this->coefficients.size(); ++i) {
        intermediates[i] = static_cast<int32_t>( std::clamp(this->coefficients[i] * (1 << (this->decimals)), clamp_min, clamp_max) );
        for (int j=3; j>=0;j--)
        {
            this->i2c_values[dsp_i++] = (intermediates[i] >> 8*j) & 0xFF;
        }
    }
    ESP_LOGD(TAG, "DSP coefficients: %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
        this->i2c_values[0], this->i2c_values[1], this->i2c_values[2], this->i2c_values[3],
        this->i2c_values[4], this->i2c_values[5],this->i2c_values[6], this->i2c_values[7],
        this->i2c_values[8], this->i2c_values[9], this->i2c_values[10], this->i2c_values[11],
        this->i2c_values[12], this->i2c_values[13], this->i2c_values[14], this->i2c_values[15],
        this->i2c_values[16], this->i2c_values[17], this->i2c_values[18],this->i2c_values[19]);
}