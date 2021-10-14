/**
 * Author: sascha_lammers@gmx.de
 */

#include "fan.h"
#include "main.h"

volatile uint16_t Fan::_fanTachoSignal = 0;

void Fan::fanSignalISR()
{
    _fanTachoSignal++;
}

ISR(TIMER2_OVF_vect)
{
    fan._timerOverflow();
}
