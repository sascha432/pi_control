/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

#ifndef I2C_SLAVE_ADDRESS
#define I2C_SLAVE_ADDRESS 0x78
#endif

#define PIN_VOLTAGE1 A0
#define PIN_VOLTAGE2 A1
#define PIN_VOLTAGE3 A2
#define PIN_CURRENT1 A3
#define PIN_CURRENT2 A6

#define PIN_BUTTON1 -1
#define PIN_BUTTON2 -1

#define PIN_MOTION_SENSOR -1

#define PIN_NEOPIXEL 4
#define PIN_NEOPIXEL_NUM 3

#define PIN_TILT_SERVO 9
#define SERVO_MIN 84
#define SERVO_MAX 160

#define PIN_FAN_PWM 5
#define PIN_FAN_TACHO 2
#define FAN_STARTUP_PWM 255
#define FAN_MIN_PWM 18
#define FAN_MAX_PWM 255

namespace Timer1 {

    static constexpr uint8_t kPreScaler = 1;
    static constexpr uint8_t kPreScalerBV = _BV(CS10);
    static constexpr float kTicksPerMicrosecond = F_CPU / kPreScaler / 1000000.0;
    static constexpr uint32_t kTicksPerMillisecond = F_CPU / kPreScaler / 1000.0;
    static constexpr uint32_t kTicksPerMinute = F_CPU * 60.0 / kPreScaler;

}

namespace Timer2 {

    static constexpr uint16_t kPreScaler = 256;
    static constexpr uint8_t kPreScalerBV = _BV(CS21)|_BV(CS22);
    static constexpr float kTicksPerMicrosecond = F_CPU / kPreScaler / 1000.0;
    static constexpr float kTicksPerSecond = F_CPU / kPreScaler / 1.0;
    static constexpr uint32_t kTicksPerMinute = F_CPU * 60.0 / kPreScaler;

}
