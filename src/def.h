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

// 5, 2, 9, 4, 8, 10, 11, 12, 13, 7
// 3

// Pi 5.35V rail/USB
#define PIN_VOLTAGE1 A0
// 9-20V input rail
#define PIN_VOLTAGE2 A1
// 4.25V lion battery
#define PIN_VOLTAGE3 A2

// Pi 5.35V rail/USB
#define PIN_CURRENT1 A3
// 9-20V input rail
#define PIN_CURRENT2 A6

// I2C
#define PIN_SDA A4
#define PIN_SCL A5

// power button
#define PIN_BUTTON1 8       // PB0
// reset button
#define PIN_BUTTON2 10      // PB2
// unused
#define PIN_BUTTON3 11      // PB3

enum PinChangeFlagsEnum : uint8_t {
    NONE = 0,
    BUTTON1 = _BV(0),
    BUTTON2 = _BV(2),
    BUTTON3 = _BV(3),
    ANY = BUTTON1|BUTTON2|BUTTON3
};

// power LED
#define PIN_LED1 12         // PB4
// reset indicator LED
#define PIN_LED2 13         // PB5
// unused
#define PIN_LED3 7          // PD7

// time required to press the power button to initial a shutdown in milliseconds
#define POWER_OFF_DELAY 3500
// blink power LED while the button is pressed
#define POWER_OFF_LED_BLINK_INTERVAL 500

// time required to force a hard reset in milliseconds
#define HARD_RESET_DELAY 5000
// time required to invoke a reboot in milliseconds
#define SOFT_RESET_DELAY 1000
// blink reset LED while the button is pressed
#define RESET_LED_BLINK_INTERVAL 500

#define PIN_MOTION_SENSOR 11    // PB3

#define PIN_NEOPIXEL 4          // PD4
#define PIN_NEOPIXEL_NUM 3

#define PIN_TILT_SERVO 9        // PB1
#define SERVO_MIN 84
#define SERVO_MAX 160

#define PIN_FAN_PWM 5           // PD5
#define PIN_FAN_TACHO 2         // PD2/INT0
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
