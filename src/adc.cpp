/**
 * Author: sascha_lammers@gmx.de
 */

#include <Arduino.h>
#include "adc.h"

ADCInterrupt adc;

ISR(ADC_vect)
{
    // if the analog pin has been changed during last ISR, the next result could be from previous pin
    if (adc._counter != adc.kSkipNextReading) {
        adc._addValue(ADC);
    }

    // rotate pins
    if (++adc._counter >= adc.kAverageSampleCount) {
        adc._selectNextSourcePin();
    }
    adc._resetTimer();
}

ISR(TIMER1_COMPB_vect)
{
}
