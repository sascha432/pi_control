/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include "def.h"
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>

class Buttons {
public:
    using PinChangedType = volatile PinChangeFlagsEnum;

public:
    Buttons();

    void begin();
    void loop();

    void ISRHandler();

private:
    void _setupPinChangeInterrupt(uint8_t pin);
    // get button bits
    uint8_t _getPortBState() const;

public:
    // get changed bits and update state
    uint8_t _getPortBChangeSet();
    // button timer handling
    void press(uint8_t button);
    void repeat(uint8_t button);
    void release(uint8_t button);

private:
    volatile uint8_t _portBState;
    PinChangedType _changeFlags;
    uint8_t _portBMask;
    uint8_t _buttonsTimer[3];
};

inline Buttons::Buttons() :
    _portBState(0),
    _changeFlags(PinChangedType::NONE)
{
}

inline void Buttons::begin()
{
    // read current state
    _portBState = _getPortBState();
    // enable interrupts
    _setupPinChangeInterrupt(PIN_BUTTON1);
    _setupPinChangeInterrupt(PIN_BUTTON2);
    _setupPinChangeInterrupt(PIN_BUTTON3);
}

inline void Buttons::_setupPinChangeInterrupt(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
    *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
    _portBMask |= _BV(digitalPinToPCMSKbit(pin));
    PCIFR |= _BV(digitalPinToPCICRbit(pin));
    PCICR |= _BV(digitalPinToPCICRbit(pin));
}

inline uint8_t Buttons::_getPortBState() const
{
    return PINB & _portBMask;
}

inline uint8_t Buttons::_getPortBChangeSet()
{
    uint8_t value = _getPortBState();
    uint8_t changeSet = value ^ _portBState;
    _portBState = value;
    return changeSet;
}

inline void Buttons::press(uint8_t button)
{
    _buttonsTimer[button] = 0;
    #if DEBUG
        Serial.print(F("press "));
        Serial.println(button);
    #endif
}

inline void Buttons::repeat(uint8_t button)
{
    _buttonsTimer[button]++;
    #if DEBUG
        Serial.print(F("repeat "));
        Serial.print(button);
        Serial.print(' ');
        Serial.println(_buttonsTimer[button]);
    #endif
}

inline void Buttons::release(uint8_t button)
{
    #if DEBUG
        Serial.print(F("release "));
        Serial.print(button);
        Serial.print(' ');
        Serial.println(_buttonsTimer[button]);
    #endif
}

inline __attribute__((always_inline)) void Buttons::ISRHandler()
{
    // update flags for loop() method
    _changeFlags = static_cast<PinChangedType>(static_cast<uint8_t>(_changeFlags) | (_getPortBChangeSet() & PinChangedType::ANY));
}

extern Buttons buttons;
