#ifndef ESP_MSS_H
#define ESP_MSS_H

#include "amp.h"

class ESPMultiSpeakerSystem {
public:
    ESPMultiSpeakerSystem();
    void init();

private:
    static TAS5805M amp;
    static void master_data_cb(const uint8_t *buf, uint32_t len);
};

#endif