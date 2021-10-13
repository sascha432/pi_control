/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

#define PIN_VOLTAGE1 A0
#define PIN_VOLTAGE2 A1
#define PIN_CURRENT1 A2
#define PIN_CURRENT2 A3

#define PIN_BUTTON1 -1
#define PIN_BUTTON2 -1

#define PIN_MOTION_SENSOR -1

#define PIN_NEOPIXEL 4
#define PIN_NEOPIXEL_NUM 3

#define PIN_TILT_SERVO 9

#define PIN_FAN_PWM 5
#define PIN_FAN_TACHO 2

namespace Timer1 {

    static constexpr uint8_t kPreScaler = 1;
    static constexpr uint8_t kPreScalerBV = _BV(CS10);
    static constexpr float kTicksPerMicrosecond = F_CPU / kPreScaler / 1000000.0;
    static constexpr uint32_t kTicksPerMillisecond = F_CPU / kPreScaler / 1000.0;
    static constexpr uint32_t kTicksPerMinute = F_CPU * 60.0 / kPreScaler;

}
