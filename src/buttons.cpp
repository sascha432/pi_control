/**
 * Author: sascha_lammers@gmx.de
 */

#include "buttons.h"
#include "main.h"
#include "interrupt_push_button.h"

extern InterruptPushButton<PinChangeFlagsEnum::BUTTON1> button1;
extern InterruptPushButton<PinChangeFlagsEnum::BUTTON2> button2;
extern InterruptPushButton<PinChangeFlagsEnum::BUTTON3> button3;

void Buttons::loop()
{
    // process events
    ATOMIC_BLOCK(LOOP_METHOD_ATOMIC_BLOCK) {
        if (button1.isPressed()) {
            button1.update();
        }
    }
    ATOMIC_BLOCK(LOOP_METHOD_ATOMIC_BLOCK) {
        if (button2.isPressed()) {
            button2.update();
        }
    }
    ATOMIC_BLOCK(LOOP_METHOD_ATOMIC_BLOCK) {
        if (button3.isPressed()) {
            button3.update();
        }
    }
}

inline __attribute__((always_inline)) void Buttons::handleButtons()
{
    PinChangeFlagsEnum flags = _changeFlags;
    _changeFlags = PinChangedType::NONE;
    // process events
    if (flags & PinChangedType::BUTTON1) {
        button1.update();
    }
    if (flags & PinChangedType::BUTTON2) {
        button2.update();
    }
    if (flags & PinChangedType::BUTTON3) {
        button3.update();
    }
}

ISR(PCINT0_vect)
{
    buttons.ISRHandler();
    buttons.handleButtons();
}
