/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include "main.h"
#include "i2c.h"

class Fan {
public:

    // ~1 second update
    static constexpr auto kRPMUpdateRate = static_cast<uint8_t>(1.0 * Timer2::kTicksPerSecond / 256.0);

public:
    Fan();

    void begin();

    static void fanSignalISR();

    void setPwm(uint8_t pwm);
    uint8_t getPwm() const;
    float getPwmPercent() const;

    uint16_t getRpm() const;

    bool isFanOn() const;

private:
    // void _updateRpm();
    // called from ISR
    void _updateRpmNonAtomic();
    void _resetTimer();

public:
    void _timerOverflow();

private:
    static volatile uint16_t _fanTachoSignal;
    uint16_t _fanTimer;
    uint8_t _fanPwm;
    uint8_t _rpmUpdateCounter;
};

inline Fan::Fan() :
    _fanTimer(0),
    _fanPwm(FAN_STARTUP_PWM),
    _rpmUpdateCounter(0)
{
}

inline void Fan::begin()
{
    pinMode(PIN_FAN_TACHO, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FAN_TACHO), Fan::fanSignalISR, RISING);
    setPwm(FAN_STARTUP_PWM);
    // enable ISR for updating RPM
    TIMSK2 |= _BV(TOIE2);
    _resetTimer();
}

inline bool Fan::isFanOn() const
{
    return _fanPwm != 0;
}

inline void Fan::setPwm(uint8_t pwm)
{
    _fanPwm = pwm;
    analogWrite(PIN_FAN_PWM, _fanPwm);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        regMem.fanPwm = _fanPwm;
    }
}

inline uint8_t Fan::getPwm() const
{
    return _fanPwm;
}

inline uint16_t Fan::getRpm() const
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return regMem.fanRpm;
    }
    __builtin_unreachable();
}

inline float Fan::getPwmPercent() const
{
    auto pwm = getPwm();
    if (pwm < FAN_MIN_PWM) {
        return 0;
    }
    return std::clamp<float>(((pwm - FAN_MIN_PWM) * 100) / static_cast<float>(FAN_MAX_PWM - FAN_MIN_PWM), 1.0, 100.0);
}

// inline void Fan::_updateRpm()
// {
//     uint16_t count;
//     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//         count = _fanTachoSignal;
//         _fanTachoSignal = 0;
//     }
//     uint16_t time = millis16();
//     uint16_t diff = time - _fanTimer;
//     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//         if (!diff || diff > 1000) {
//             regMem.fanRpm = 0;
//         }
//         else {
//             regMem.fanRpm = (30000UL * count) / diff;
//         }

//     }
//     _fanTimer = time;
// }

inline void Fan::_resetTimer()
{
    _rpmUpdateCounter = Fan::kRPMUpdateRate;
}

inline void Fan::_updateRpmNonAtomic()
{
    uint16_t count = _fanTachoSignal;
    _fanTachoSignal = 0;
    uint16_t time = millis16();
    uint16_t diff = time - _fanTimer;
    if (!diff || diff > 0x7fff) {
        regMem.fanRpm = 0;
    }
    else {
        regMem.fanRpm = (30000UL * count) / diff;
    }
    _fanTimer = time;
}

inline void Fan::_timerOverflow()
{
    if (++_rpmUpdateCounter > kRPMUpdateRate) {
        _rpmUpdateCounter = 0;
        fan._updateRpmNonAtomic();
    }
}
