/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Wire.h>
#include <time.h>
#include "def.h"

extern void receiveEvent(int count);
extern void requestEvent();

struct RGB {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct RegMem {
    union {
        struct  {
            uint8_t address;
            float voltage1;
            float voltage2;
            float voltage3;
            float current1;
            float current2;
            uint64_t energy;
            uint8_t tiltServoAngle;
            uint8_t fanPwm;
            uint16_t fanRpm;
            uint8_t ledBrightness;
            RGB ledColors[PIN_NEOPIXEL_NUM];
        };
        uint8_t raw[1];
    };

    RegMem();

    // setup
    void begin();

    void dump();
};

inline RegMem::RegMem() : raw{}
{
    address = I2C_SLAVE_ADDRESS;
    ledBrightness = 255;
}

static constexpr size_t kRegMemSize = sizeof(RegMem);
