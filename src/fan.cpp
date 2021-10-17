/**
 * Author: sascha_lammers@gmx.de
 */

#include "fan.h"
#include "main.h"
#include "buttons.h"

volatile uint16_t Fan::_fanTachoSignal = 0;

void Fan::fanSignalISR()
{
    // count singals from the fan for RPM calculation
    _fanTachoSignal++;
}

volatile bool buttonLoopLock = false;

ISR(TIMER2_OVF_vect)
{
    fan._timerOverflow();

    // call button loop with interrupts enabled
    // needs a flag to avoid being called multiple times depending in the overflow time
    if (!buttonLoopLock) {
        buttonLoopLock = true;
        sei();
        buttons.loop();
        cli();
        buttonLoopLock = false;
    }
}
