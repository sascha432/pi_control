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
    ANY = BUTTON1|BUTTON2|BUTTON3,
    MAX = 3
};

// power LED
#define PIN_LED1 7          // PD7
// reset indicator LED
#define PIN_LED2 12         // PB4
// unused
#define PIN_LED3 13         // PB5

// time required to press the power button to initial a shutdown in milliseconds
// the LED starts to blink while the button is pressed and becomes solid once the timeout
// has been reached and the halt signal is sent. the button is disabled after that
// the signal will be sent via I2C
#define POWER_OFF_DELAY 3500

// time required to invoke a reboot in milliseconds
// the LED starts to blink while the button is pressed. once the soft reset timeout has
// been reached, the LED starts to blink 4 times faster. the button is still active in case
// a hard reset is required
// the signal will be sent via I2C
#define SOFT_RESET_DELAY 2500
// time required to force a hard reset in milliseconds
// once the hard reset timeout has been reached, the LED becomes solid. the button is disabled
// once the hard reset is invoked
// the hard reset pulls the run pin low to force a reset
#define HARD_RESET_DELAY 7500

// if the WDT reset signal is not received for the timeout in milliseconds,
// a soft reset is sent
#define WATCH_DOG_SOFT_TIMEOUT 60000

// if the system does not respond to the to the soft reset, the run pin will be pulled
// low to force a reset
#define WATCH_DOG_HARD_TIMEOUT 120000

// motion sensor to enable/disable the screen or run other action if user presence is detected
#define PIN_MOTION_SENSOR 11    // PB3

// buzzer to signal errors or alarms
#define PIN_BUZZER 3            // PD3

// 3 WS21812 LEDs integrated into the stand
// used to display system temperature / fan speed or run animations during reboot etc...
#define PIN_NEOPIXEL 4          // PD4
#define PIN_NEOPIXEL_NUM 3

// server to tilt the screen
#define PIN_TILT_SERVO 9        // PB1
#define SERVO_MIN 84
#define SERVO_MAX 160

// PWM fan control and RPM tacho signal
#define PIN_FAN_PWM 5           // PD5
#define PIN_FAN_TACHO 2         // PD2/INT0
// start up speed if no temperure is available, for example during boot
#define FAN_STARTUP_PWM 255
// min. pwm level that keeps the fan running. to start the fan a higher value will be used
// TODO verify that the fan is really spinning and increase the min. level if it stalled
#define FAN_MIN_PWM 18
// max. fan speed
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
