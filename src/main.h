/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "def.h"
#include "helpers.h"

class RegMem;
class TwiBuffer;
class Fan;
class Buttons;

extern RegMem regMem;
extern TwiBuffer twiBuffer;
extern Fan fan;
extern Buttons buttons;