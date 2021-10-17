/**
 * Author: sascha_lammers@gmx.de
 */

// pin change level interrupt handler for PORTB

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

#define LOOP_METHOD_ATOMIC_BLOCK ATOMIC_FORCEON
// #define LOOP_METHOD_ATOMIC_BLOCK ATOMIC_RESTORESTATE

class Buttons {
public:
    using PinChangedType = volatile PinChangeFlagsEnum;

    static constexpr uint8_t kBlinkInterval = 100;

public:
    Buttons();

    void begin();
    // called from ISR(TIMER2_OVF_vect)
    void loop();

    // both methods are called from the pin change level ISR
    void ISRHandler();
    void handleButtons();

private:
    // setup a pin
    void _setupPinChangeInterrupt(uint8_t pin);

public:
    // disable interrupt
    void stopPinChangeInterrupt(uint8_t pin);
    // get button bits
    uint8_t getPortBState() const;
    // get changed bits and update state
    uint8_t getPortBChangeSet();

private:
    // stores the last button state
    volatile uint8_t _portBState;
    //
    PinChangedType _changeFlags;
    // mask for enable pins
    uint8_t _portBMask;
};

inline Buttons::Buttons()
{
}

inline void Buttons::begin()
{
    // enable interrupts
    _setupPinChangeInterrupt(PIN_BUTTON1);
    _setupPinChangeInterrupt(PIN_BUTTON2);
    _setupPinChangeInterrupt(PIN_BUTTON3);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // read current state
        _portBState = getPortBState();
        _changeFlags = PinChangedType::NONE;
    }
}

inline void Buttons::_setupPinChangeInterrupt(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
    *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
    _portBMask |= _BV(digitalPinToPCMSKbit(pin));
    PCIFR |= _BV(digitalPinToPCICRbit(pin));
    PCICR |= _BV(digitalPinToPCICRbit(pin));
}

inline void Buttons::stopPinChangeInterrupt(uint8_t pin)
{
    *digitalPinToPCMSK(pin) &= ~_BV(digitalPinToPCMSKbit(pin));
    _portBState &= ~_BV(digitalPinToPCMSKbit(pin));
}

inline uint8_t Buttons::getPortBState() const
{
    return PINB & _portBMask;
}

inline uint8_t Buttons::getPortBChangeSet()
{
    uint8_t value = getPortBState();
    uint8_t changeSet = value ^ _portBState;
    _portBState = value;
    return changeSet;
}

inline __attribute__((always_inline)) void Buttons::ISRHandler()
{
    // update flags for loop() method
    _changeFlags = static_cast<PinChangedType>(static_cast<uint8_t>(_changeFlags) | (getPortBChangeSet() & PinChangedType::ANY));
}

extern Buttons buttons;
