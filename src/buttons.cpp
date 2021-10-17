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
    PinChangeFlagsEnum flags;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // clear flags
        flags = _changeFlags;
        _changeFlags = PinChangedType::NONE;
    }
    // process events
    if (flags & PinChangedType::BUTTON1) {
        button1.update();
    }
    if (flags & PinChangedType::BUTTON2) {
        button2.update();
    }
    if (flags & PinChangedType::BUTTON2) {
        button3.update();
    }
}

ISR(PCINT0_vect)
{
    buttons.ISRHandler();
}
