//
// Created by Clas Zachrisson on 2025-03-09.
//

#ifndef BIQUAD_H
#define BIQUAD_H

#define _USE_MATH_DEFINES
#include <span>
#include "math.h"

class Biquad
{
public:
    Biquad();
    Biquad(int type, double char_freq, double Q_factor=Q707, double gain=1, int sample_freq=96000);
    static double Q707;
    enum filter_type
    {
        BQ_TYPE_FLAT,
        BQ_TYPE_HPF,
        BQ_TYPE_LPF,
        BQ_TYPE_BPF,
        BQ_TYPE_NOTCH,
        BQ_TYPE_SHELF,
    };

    uint8_t* data() { return i2c_values.data(); }
    size_t size() const { return i2c_values.size(); }

private:
    int type;
    int sample_freq;
    double char_freq;
    double Q_factor;
    double gain;
    int decimals = 27;
    static double clamp_min;
    static double clamp_max;
    static double signs[];

    std::array<double, 5> coefficients{};
    std::array<uint8_t, 20> i2c_values{};

    void init(int type, double char_freq, double Q_factor=Q707, int sample_freq=96000);
    void convert_to_dsp_format();
};

#endif // BIQUAD_H
