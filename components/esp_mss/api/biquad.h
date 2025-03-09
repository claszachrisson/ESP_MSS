//
// Created by Clas Zachrisson on 2025-03-09.
//

#ifndef BIQUAD_H
#define BIQUAD_H

#define _USE_MATH_DEFINES
#include "math.h"
#include <span>

class Biquad {
  public:
  Biquad(int type, double char_freq, double Q_factor, int sample_freq);
  static double Q707;
  enum filter_type {
    BQ_TYPE_HPF = 0x0,
    BQ_TYPE_LPF = 0x1,
    BQ_TYPE_BPF = 0x2,
    BQ_TYPE_NOTCH = 0x3,
    BQ_TYPE_SHELF = 0x4,
};
  private:
    int type;
    int sample_freq;
    double char_freq;
    double Q_factor;
    int decimals = 27;
    static double clamp_min;
    static double clamp_max;

    std::array<double, 5> coefficients{};
    std::array<uint8_t, 20> i2c_values{};

    void convert_to_dsp_format();
};

#endif //BIQUAD_H
