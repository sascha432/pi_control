/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <PushButton.h>
#include "buttons.h"

template<uint8_t _BitMask>
class InterruptPushButton : public PushButton {
public:
    InterruptPushButton(uint8_t button, uint8_t options) :
        PushButton(button, options)
    {
    }

protected:
    virtual boolean _update_button_state() override
    {
        return !(buttons.getPortBState() & _BitMask);
    }
};
