/**
  Author: sascha_lammers@gmx.de
*/

#include <util/atomic.h>
#include <wiring_private.h>
#include "i2c.h"

void receiveEvent(int count)
{
    // if (!count) {
    //     DBG1_PRINTF("no data received");
    //     return;
    // }

    // // all data is read from the buffer while being processed
    // auto cmd = regMem.readCommand(count);
    // if (cmd.command == TinyPwm::Commands::INVALID) {
    //     DBG1_PRINTF("Wire.available() = 0");
    //     return;
    // }

    // DBG1_PRINTF("receive event count=%u command=%u data=%02x %02x %02x %02x", count, cmd.command, cmd.raw[1], cmd.raw[2], cmd.raw[3], cmd.raw[4]); // sizeof(TinyPwm::Command);

    // twiBuffer.clear();

    // while(cmd.command != TinyPwm::Commands::END) {
    //     switch(cmd.command) {
    //         case TinyPwm::Commands::ANALOG_READ:
    //             if (CMD_CHECK_LENGTH(ANALOG_READ)) {
    //                 if (cmd.ANALOG_READ.pin < sizeof(TinyPwm::kPinCount)) {
    //                     twiBuffer.send(regMem.analog[cmd.ANALOG_READ.pin]);
    //                 }
    //                 else {
    //                     DBG1_PRINTF("ANALOG_READ pin=%u out of range (0-%u)", cmd.ANALOG_READ.pinNo, sizeof(kAnalogPins) - 1);
    //                 }
    //                 removeCommand(cmd, sizeof(cmd.ANALOG_READ));
    //             }
    //             break;
    //         case TinyPwm::Commands::ANALOG_WRITE:
    //             if (CMD_CHECK_LENGTH(ANALOG_WRITE)) {
    //                 regMem.analogWrite(cmd.ANALOG_WRITE.pin, cmd.ANALOG_WRITE.pwmValue);
    //                 removeCommand(cmd, sizeof(cmd.ANALOG_WRITE));
    //             }
    //             break;
    //         case TinyPwm::Commands::DIGITAL_READ:
    //             if (CMD_CHECK_LENGTH(DIGITAL_READ)) {
    //                 uint8_t state = regMem.digitalRead(cmd.DIGITAL_READ.pin);
    //                 twiBuffer.send(state);
    //                 removeCommand(cmd, sizeof(cmd.DIGITAL_READ));
    //             }
    //             break;
    //         case TinyPwm::Commands::DIGITAL_WRITE:
    //             if (CMD_CHECK_LENGTH(DIGITAL_WRITE)) {
    //                 regMem.digitalWrite(cmd.DIGITAL_WRITE.pin, cmd.DIGITAL_WRITE.value);
    //                 removeCommand(cmd, sizeof(cmd.DIGITAL_WRITE));
    //             }
    //             break;
    //         case TinyPwm::Commands::PIN_MODE:
    //             if (CMD_CHECK_LENGTH(PIN_MODE)) {
    //                 regMem.pinMode(cmd.PIN_MODE.pin, cmd.PIN_MODE.mode);
    //                 removeCommand(cmd, sizeof(cmd.PIN_MODE));
    //             }
    //             break;
    //         case TinyPwm::Commands::WRITE_EEPROM: {
    //                 uint16_t position = 0;
    //                 Config::Config config;
    //                 if (!config.read(position)) {
    //                     config.clear();
    //                 }
    //                 regMem.updateConfig(config.data());
    //                 position = 0;
    //                 config.write(position);
    //                 // backup copy in case the MCU resets during write
    //                 config.write(position);
    //                 removeCommand(cmd, 1);
    //             }
    //             break;
    //         case TinyPwm::Commands::RESET_EEPROM: {
    //                 // write default settings to EEPROM
    //                 Config::Config config;
    //                 config.clear();
    //                 uint16_t position = 0;
    //                 config.write(position);
    //                 config.write(position);

    //                 position = 0;
    //                 if (!config.read(position)) {
    //                     config.clear();
    //                 }
    //                 // reset device
    //                 regMem = RegMem(config.data());
    //                 regMem.begin();
    //                 return;
    //             }
    //             break;

    //         case TinyPwm::Commands::SET_PWM_PARAMS:
    //             if (CMD_CHECK_LENGTH(SET_PWM_PARAMS)) {
    //                 regMem.setPwmParams(cmd.SET_PWM_PARAMS.prescaler, cmd.SET_PWM_PARAMS.overflow);
    //                 removeCommand(cmd, sizeof(cmd.SET_PWM_PARAMS));
    //             }
    //             break;
    //         case TinyPwm::Commands::SET_PWM_FREQUENCY:
    //             if (CMD_CHECK_LENGTH(SET_PWM_FREQUENCY)) {
    //                 regMem.setPwmFrequency(cmd.SET_PWM_FREQUENCY.frequency);
    //                 removeCommand(cmd, sizeof(cmd.SET_PWM_FREQUENCY));
    //             }
    //             break;
    //         case TinyPwm::Commands::ADC_SET_AREF:
    //             if (CMD_CHECK_LENGTH(ADC_SET_AREF)) {
    //                 regMem.setAnalogReference(cmd.ADC_SET_AREF.mode);
    //                 removeCommand(cmd, sizeof(cmd.ADC_SET_AREF));
    //             }
    //             break;
    //         case TinyPwm::Commands::ADC_SET_READ_CYCLES:
    //             if (CMD_CHECK_LENGTH(ADC_SET_READ_CYCLES)) {
    //                 regMem.setAnalogCycles(cmd.ADC_SET_READ_CYCLES.pin, cmd.ADC_SET_READ_CYCLES.cycles);
    //                 removeCommand(cmd, sizeof(cmd.ADC_SET_READ_CYCLES));
    //             }
    //             break;
    //         case TinyPwm::Commands::READ_SFR:
    //             if (CMD_CHECK_LENGTH(READ_SFR)) {
    //                 twiBuffer.send(_SFR_IO8(cmd.READ_SFR.address));
    //                 removeCommand(cmd, sizeof(cmd.READ_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::WRITE_SFR:
    //             if (CMD_CHECK_LENGTH(WRITE_SFR)) {
    //                 _SFR_IO8(cmd.WRITE_SFR.address) = cmd.WRITE_SFR.data;
    //                 removeCommand(cmd, sizeof(cmd.WRITE_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::MASK_SFR:
    //             if (CMD_CHECK_LENGTH(MASK_SFR)) {
    //                 _SFR_IO8(cmd.MASK_SFR.address) = (_SFR_IO8(cmd.MASK_SFR.address) & cmd.MASK_SFR.mask) | cmd.MASK_SFR.data;
    //                 removeCommand(cmd, sizeof(cmd.MASK_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::OR_SFR:
    //             if (CMD_CHECK_LENGTH(OR_SFR)) {
    //                 _SFR_IO8(cmd.OR_SFR.address) |= cmd.OR_SFR.data;
    //                 removeCommand(cmd, sizeof(cmd.OR_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::AND_SFR:
    //             if (cmd.length >= sizeof(cmd.AND_SFR)) {
    //                 _SFR_IO8(cmd.AND_SFR.address) &= cmd.AND_SFR.data;
    //                 removeCommand(cmd, sizeof(cmd.AND_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::XOR_SFR:
    //             if (cmd.length >= sizeof(cmd.XOR_SFR)) {
    //                 _SFR_IO8(cmd.XOR_SFR.address) ^= cmd.XOR_SFR.data;
    //                 removeCommand(cmd, sizeof(cmd.XOR_SFR));
    //             }
    //             break;
    //         case TinyPwm::Commands::CLI:
    //             cli();
    //             removeCommand(cmd, 0);
    //             break;
    //         case TinyPwm::Commands::SEI:
    //             sei();
    //             removeCommand(cmd, 0);
    //             break;
    //         case TinyPwm::Commands::INFO:
    //             twiBuffer.send(TinyPwm::Info());
    //             break;
    //         default:
    //             DBG1_PRINTF("invalid command=%u", cmd.command);
    //             return;
    //     }
    // }
}

void RegMem::begin()
{
}

void RegMem::dump()
{
    Serial.print(F("--- runtime "));
    Serial.println(millis() / 1000);
    Serial.print(F("I2C address 0x"));
    Serial.println(address, HEX);
    Serial.print(F("U1 "));
    Serial.print(voltage1, 3);
    Serial.print(F(" I1 "));
    Serial.println(current1, 3);
    Serial.print(F("U2 "));
    Serial.print(voltage2, 3);
    Serial.print(F(" I2 "));
    Serial.println(current2, 3);
    Serial.print(F("U3 "));
    Serial.println(voltage3, 3);
    Serial.print(F("Fan "));
    Serial.print((fanPwm * 100.0) / 255.0);
    Serial.print(F("% RPM "));
    Serial.println(fanRpm);
    Serial.print(F("Tilt "));
    Serial.print(((SERVO_MAX - tiltServoAngle) / 2.8) + 5.0, 1);
    Serial.println(F("°"));
    Serial.print(F("LED Brightness "));
    Serial.print((ledBrightness * 100) / 255.0, 1);
    Serial.println('%');
}

// TinyPwm::Command RegMem::readCommand(uint8_t count)
// {
//     TinyPwm::Command cmd;
//     DBG_PRINTF("wire available=%u count=%u", Wire.available(), count);

//     if (Wire.available()) {
//         auto dst = reinterpret_cast<uint8_t *>(&cmd.command);
//         auto dataEnd = dst + min(count, sizeof(cmd) - (dst - reinterpret_cast<uint8_t *>(&cmd)));
//         auto end = cmd.raw + sizeof(cmd);
//         // read data
//         while(dst < dataEnd && Wire.available()) {
//             *dst++ = Wire.read();
//         }
//         cmd.length = dst - reinterpret_cast<const uint8_t *>(&cmd.command);
//         // discard extra data
//         while(Wire.available()) {
//             Wire.read();
//         }
//         // fill missing data with 0xff
//         while(dst < end) {
//             *dst++ = 0xff;
//         }
// #if DEBUG
//         Serial.printf(F("cmd length %u data="), (unsigned)cmd.length);
//         dst = reinterpret_cast<uint8_t *>(&cmd);
//         end = dst + sizeof(cmd);
//         while(dst < end) {
//             Serial.printf("%02x ", (unsigned)*dst);
//         }
//         Serial.println();
// #endif
//     }
//     else {
//         cmd.command = TinyPwm::Commands::INVALID;
//         DBG_PRINTF("Wire.available=0 count=%u", count);
//     }
//     return cmd;
// }

// void RegMem::setPwmParams(TinyPwm::ClockPrescaler clockPrescaler, uint8_t overflow)
// {
//     // defaults
//     // TCCR0A 00000111 (???/WGM01/WGM00)
//     // TCCR0B 00000111 (CS02/CS01/CS00) timer prescaler 64
//     // TCCR1 11000111 (CTC1/PWM1A/CS12/CS11/CS10) clock prescaler 64
//     // OCR1C 255

//     // DBG2_PRINTF("setTimerParams before TCCR0A=%u TCCR0B=%u TCCR1=%u OCR1C=%u FCLK=%u PCK=%u", TCCR0A, TCCR0B, TCCR1, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1));
//     // testDelay(10000);

//     ATOMIC_BLOCK(ATOMIC_FORCEON) {
//         // TCCR0B = static_cast<uint8_t>(timerPrescaler);

//         if (overflow && overflow != 255) {
//             OCR1C = overflow; // set overflow counter
//             TCCR0A = (TCCR0A & 0b11111100) | _BV(WGM01); // CTC mode
//         }
//         else {
//             OCR1C = 255;
//             TCCR0A = (TCCR0A & 0b11111100) | (_BV(WGM01)|_BV(WGM00)); // Fast PWM
//         }
//         TCCR1 = (TCCR1 & static_cast<uint8_t>(TinyPwm::ClockPrescaler::MASK)) | static_cast<uint8_t>(clockPrescaler); // set prescaler
//     }

//     DBG2_PRINTF("TCCR0A=%u TCCR0B=%u TCCR1=%u OCR1C=%u FCLK=%u PCK=%u", TCCR0A, TCCR0B, TCCR1, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1));
//     // testDelay(10000);
// }

// uint16_t RegMem::getPwmFrequency() const
// {
//     DBG2_PRINTF("F_CPU=%lu OCR1C=%u pre=%u clk=%u mode=%s", F_CPU, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1), isPwmModeFastPwm() ? "FastPWM" : (isPwmModeCTC() ? "CTC" : (isPwmPhaseCorrect() ? "PhaseCorrect" : "N/A")));

//     if (isPwmModeFastPwm()) {
//         return (F_CPU / 256UL) / TinyPwm::getClockPrescaler(TCCR1);
//     }
//     return F_CPU / (TinyPwm::getClockPrescaler(TCCR1) * (1UL + OCR1C));
// }

// void RegMem::selectAnalogPin(uint8_t pin)
// {
//     // translate to internal number
//     pin = analogPins[pin];
//     if ((arrays.pinModes[pin] & ANALOG_INPUT) == 0) {
//         DBG2_PRINTF("selectAnalogPin pin #%u not in analog input mode", pin);
//         return;
//     }
//     analogPinSelected = pin;
//     cycleCounter = -1; // skip first reading after selecting a new pin
//     // get digital pin
//     auto digitalPin = TinyPwm::_kPins[pin];

//     DBG3_PRINTF("select_analog_pin %u %u", (unsigned)pin, (unsigned)analogInputToDigitalPin(digitalPin));
// #if defined(ADMUX)
//     ADMUX = ((analog_reference & ADMUX_REFS_MASK) << REFS0) | ((digitalPin & ADMUX_MUX_MASK) << MUX0); // select the channel and reference
// #if defined(REFS2)
//     ADMUX |= (((analog_reference & 0x04) >> 2) << REFS2); // some have an extra reference bit in a weird position.
// #endif
// #endif
//     ADCHelper::startConversion();
// }

// void RegMem::selectNextAnalogPin()
// {
//     if (analogPinCount == 0) {
//         return;
//     }
//     if (++analogPinSelected >= analogPinCount) {
//         analogPinSelected = 0;
//     }
//     if (analogPinSelected == 0) {
//         // start new conversion, only one analog pin is available
//         cycleCounter = 0;
//         ADCHelper::startConversion();
//     }
//     else {
//         // select next pin and start conversion
//         selectAnalogPin(analogPinSelected);
//     }
// }

// void RegMem::analogWrite(uint8_t pin, uint8_t value)
// {
//     if (pin >= TinyPwm::kPinCount) {
//         DBG2_PRINTF("analogWrite pin #%u not valid", pin);
//         return;
//     }
// #if DEBUG
//     if (arrays.pinModes[pin] == 0xff) {
//         DBG2_PRINTF("analogWrite pin #%u used for debugging", pin);
//         return;
//     }
// #endif
//     if (arrays.pinModes[pin] != OUTPUT) {
//         DBG2_PRINTF("analogWrite pin #%u not in OUTPUT mode", pin);
//         return;
//     }
//     DBG2_PRINTF("analogWrite pin #%u=%u (scaled %u)", pin, value, value * OCR1C / 255);
//     ::analogWrite(pin, value * OCR1C / 255);
//     arrays.values[pin] = value;
// }

// void RegMem::digitalWrite(uint8_t pin, bool state)
// {
//     if (pin >= TinyPwm::kPinCount) {
//         DBG2_PRINTF("digitalWrite pin #%u not valid", pin);
//         return;
//     }
// #if DEBUG
//     if (arrays.pinModes[pin] == 0xff) {
//         DBG2_PRINTF("digitalWrite pin #%u used for debugging", pin);
//         return;
//     }
// #endif
//     if (arrays.pinModes[pin] != OUTPUT) {
//         DBG2_PRINTF("digitalWrite pin #%u not in OUTPUT mode", pin);
//         return;
//     }
//     DBG2_PRINTF("analogWrite pin #%u=%u (scaled %u)", pin, value, value * OCR1C / 255);
//     ::digitalWrite(pin, state);
//     arrays.values[pin] = state;
// }

// bool RegMem::digitalRead(uint8_t pin)
// {
//     if (pin >= TinyPwm::kPinCount) {
//         DBG2_PRINTF("digitalRead pin #%u not valid", pin);
//         return false;
//     }
//     return ::digitalRead(pin);
// }

// void RegMem::_resetAnalogPins()
// {
//     analogPinCount = 0;
//     for(uint8_t i = 0; i < TinyPwm::kPinCount; i++) {
//         if (arrays.pinModes[i] == ANALOG_INPUT) {
//             analogPins[analogPinCount++] = i;
//         }
//     }
//     // start new cycle
//     selectNextAnalogPin();
//     // we don't know if resetting selected the same pin or a new one
//     // skip first reading
//     cycleCounter = -1;
//     DBG2_PRINTF("analogPinCount %u", analogPinCount);
// }

// void RegMem::pinMode(uint8_t pin, uint8_t mode)
// {
//     if (pin >= TinyPwm::kPinCount) {
//         DBG2_PRINTF("pinMode pin #%u not valid", pin);
//         return;
//     }
// #if DEBUG
//     if (arrays.pinModes[pin] == 0xff) {
//         DBG2_PRINTF("pinMode pin #%u used for debugging", pin);
//         return;
//     }
// #endif
//     bool isAnalog = arrays.pinModes[pin] & ANALOG_INPUT;
//     auto digitalPin = TinyPwm::_kPins[pin];

//     if (mode & ANALOG_INPUT) {
//         arrays.pinModes[pin] = ANALOG_INPUT;
//         mode = INPUT;
//         isAnalog = true;
//     }
//     else {
//         arrays.pinModes[pin] = mode;
//     }
//     ::pinMode(digitalPin, mode);

//     // number of analog pins has changed
//     if (isAnalog) {
//         _resetAnalogPins();
//     }
// }

// void RegMem::readAnalogValue()
// {
//     if (cycleCounter <= 0) {
//         // reset average for new pin or new cycle
//         analogValue = ADC;
//     }
//     else {
//         analogValue += ADC;
//     }

//     // DBG3_PRINTF("adc_read=%u ADC=%u cycles=%u/%u", (unsigned)analogPinNo, (unsigned)ADC, (unsigned)cycleCounter, (unsigned)cycles[analogPinNo]);

//     // translate to internal pin number
//     uint8_t analogPinNo = analogPins[analogPinSelected];

//     if (++cycleCounter >= arrays.cycles[analogPinNo]) {
//         analog[analogPinNo].value = analogValue / cycleCounter;
//         analog[analogPinNo].reading++;

// #if DEBUG
//         if (analog[analogPinNo].reading % 8192 == 0) {
//             unsigned long mul = (analog_reference == INTERNAL2V56_NO_CAP ? 2560UL : analog_reference == INTERNAL1V1 ? 1100UL : 0);
//             DBG3_PRINTF("adc_value pin_no=%u value=%u(%u) %lumV", (unsigned)analogPinNo, analog[analogPinNo].value, analog[analogPinNo].reading, (unsigned long)((analog[analogPinNo].value * mul) >> 10));
//         }
// #endif

//         selectNextAnalogPin();
//     }
// }


// /**
//   Author: sascha_lammers@gmx.de
// */

// #include <Arduino.h>
// #include <wiring_private.h>
// #include <time.h>
// #include "twi_buffer.h"
// #include "i2c.h""

void requestEvent()
{
//     twiBuffer.sendTo(Wire);
}

// static bool twiLengthError(const char *name, int length, int expected)
// {
//     return false;
// }

// #define DBG_TWI_LENGTH_ERROR(command) twiLengthError(_STRINGIFY(command), cmd.length, sizeof(cmd.command))
// #define CMD_CHECK_LENGTH(command) ((cmd.length < sizeof(cmd.command)) ? DBG_TWI_LENGTH_ERROR(command) : true)

// //TODO review function
// void removeCommand(TinyPwm::Command &cmd, uint8_t cmdStructLength)
// {
//     cmdStructLength += sizeof(cmd.command);
//     // move to next command
//     memmove(cmd.rawCommand, cmd.rawCommand + cmdStructLength, sizeof(cmd.rawCommand) - cmdStructLength);
//     // fill the rest with 0xff
//     memset(cmd.rawCommand + sizeof(cmd.rawCommand) - cmdStructLength, 0xff, cmdStructLength);
//     // update length
//     cmd.length -= cmdStructLength;
// }

// void receiveEvent(int count)
// {
//     if (!count) {
//         DBG1_PRINTF("no data received");
//         return;
//     }

//     // all data is read from the buffer while being processed
//     auto cmd = regMem.readCommand(count);
//     if (cmd.command == TinyPwm::Commands::INVALID) {
//         DBG1_PRINTF("Wire.available() = 0");
//         return;
//     }

//     DBG1_PRINTF("receive event count=%u command=%u data=%02x %02x %02x %02x", count, cmd.command, cmd.raw[1], cmd.raw[2], cmd.raw[3], cmd.raw[4]); // sizeof(TinyPwm::Command);

//     twiBuffer.clear();

//     while(cmd.command != TinyPwm::Commands::END) {
//         switch(cmd.command) {
//             case TinyPwm::Commands::ANALOG_READ:
//                 if (CMD_CHECK_LENGTH(ANALOG_READ)) {
//                     if (cmd.ANALOG_READ.pin < sizeof(TinyPwm::kPinCount)) {
//                         twiBuffer.send(regMem.analog[cmd.ANALOG_READ.pin]);
//                     }
//                     else {
//                         DBG1_PRINTF("ANALOG_READ pin=%u out of range (0-%u)", cmd.ANALOG_READ.pinNo, sizeof(kAnalogPins) - 1);
//                     }
//                     removeCommand(cmd, sizeof(cmd.ANALOG_READ));
//                 }
//                 break;
//             case TinyPwm::Commands::ANALOG_WRITE:
//                 if (CMD_CHECK_LENGTH(ANALOG_WRITE)) {
//                     regMem.analogWrite(cmd.ANALOG_WRITE.pin, cmd.ANALOG_WRITE.pwmValue);
//                     removeCommand(cmd, sizeof(cmd.ANALOG_WRITE));
//                 }
//                 break;
//             case TinyPwm::Commands::DIGITAL_READ:
//                 if (CMD_CHECK_LENGTH(DIGITAL_READ)) {
//                     uint8_t state = regMem.digitalRead(cmd.DIGITAL_READ.pin);
//                     twiBuffer.send(state);
//                     removeCommand(cmd, sizeof(cmd.DIGITAL_READ));
//                 }
//                 break;
//             case TinyPwm::Commands::DIGITAL_WRITE:
//                 if (CMD_CHECK_LENGTH(DIGITAL_WRITE)) {
//                     regMem.digitalWrite(cmd.DIGITAL_WRITE.pin, cmd.DIGITAL_WRITE.value);
//                     removeCommand(cmd, sizeof(cmd.DIGITAL_WRITE));
//                 }
//                 break;
//             case TinyPwm::Commands::PIN_MODE:
//                 if (CMD_CHECK_LENGTH(PIN_MODE)) {
//                     regMem.pinMode(cmd.PIN_MODE.pin, cmd.PIN_MODE.mode);
//                     removeCommand(cmd, sizeof(cmd.PIN_MODE));
//                 }
//                 break;
//             case TinyPwm::Commands::WRITE_EEPROM: {
//                     uint16_t position = 0;
//                     Config::Config config;
//                     if (!config.read(position)) {
//                         config.clear();
//                     }
//                     regMem.updateConfig(config.data());
//                     position = 0;
//                     config.write(position);
//                     // backup copy in case the MCU resets during write
//                     config.write(position);
//                     removeCommand(cmd, 1);
//                 }
//                 break;
//             case TinyPwm::Commands::RESET_EEPROM: {
//                     // write default settings to EEPROM
//                     Config::Config config;
//                     config.clear();
//                     uint16_t position = 0;
//                     config.write(position);
//                     config.write(position);

//                     position = 0;
//                     if (!config.read(position)) {
//                         config.clear();
//                     }
//                     // reset device
//                     regMem = RegMem(config.data());
//                     regMem.begin();
//                     return;
//                 }
//                 break;

//             case TinyPwm::Commands::SET_PWM_PARAMS:
//                 if (CMD_CHECK_LENGTH(SET_PWM_PARAMS)) {
//                     regMem.setPwmParams(cmd.SET_PWM_PARAMS.prescaler, cmd.SET_PWM_PARAMS.overflow);
//                     removeCommand(cmd, sizeof(cmd.SET_PWM_PARAMS));
//                 }
//                 break;
//             case TinyPwm::Commands::SET_PWM_FREQUENCY:
//                 if (CMD_CHECK_LENGTH(SET_PWM_FREQUENCY)) {
//                     regMem.setPwmFrequency(cmd.SET_PWM_FREQUENCY.frequency);
//                     removeCommand(cmd, sizeof(cmd.SET_PWM_FREQUENCY));
//                 }
//                 break;
//             case TinyPwm::Commands::ADC_SET_AREF:
//                 if (CMD_CHECK_LENGTH(ADC_SET_AREF)) {
//                     regMem.setAnalogReference(cmd.ADC_SET_AREF.mode);
//                     removeCommand(cmd, sizeof(cmd.ADC_SET_AREF));
//                 }
//                 break;
//             case TinyPwm::Commands::ADC_SET_READ_CYCLES:
//                 if (CMD_CHECK_LENGTH(ADC_SET_READ_CYCLES)) {
//                     regMem.setAnalogCycles(cmd.ADC_SET_READ_CYCLES.pin, cmd.ADC_SET_READ_CYCLES.cycles);
//                     removeCommand(cmd, sizeof(cmd.ADC_SET_READ_CYCLES));
//                 }
//                 break;
//             case TinyPwm::Commands::READ_SFR:
//                 if (CMD_CHECK_LENGTH(READ_SFR)) {
//                     twiBuffer.send(_SFR_IO8(cmd.READ_SFR.address));
//                     removeCommand(cmd, sizeof(cmd.READ_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::WRITE_SFR:
//                 if (CMD_CHECK_LENGTH(WRITE_SFR)) {
//                     _SFR_IO8(cmd.WRITE_SFR.address) = cmd.WRITE_SFR.data;
//                     removeCommand(cmd, sizeof(cmd.WRITE_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::MASK_SFR:
//                 if (CMD_CHECK_LENGTH(MASK_SFR)) {
//                     _SFR_IO8(cmd.MASK_SFR.address) = (_SFR_IO8(cmd.MASK_SFR.address) & cmd.MASK_SFR.mask) | cmd.MASK_SFR.data;
//                     removeCommand(cmd, sizeof(cmd.MASK_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::OR_SFR:
//                 if (CMD_CHECK_LENGTH(OR_SFR)) {
//                     _SFR_IO8(cmd.OR_SFR.address) |= cmd.OR_SFR.data;
//                     removeCommand(cmd, sizeof(cmd.OR_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::AND_SFR:
//                 if (cmd.length >= sizeof(cmd.AND_SFR)) {
//                     _SFR_IO8(cmd.AND_SFR.address) &= cmd.AND_SFR.data;
//                     removeCommand(cmd, sizeof(cmd.AND_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::XOR_SFR:
//                 if (cmd.length >= sizeof(cmd.XOR_SFR)) {
//                     _SFR_IO8(cmd.XOR_SFR.address) ^= cmd.XOR_SFR.data;
//                     removeCommand(cmd, sizeof(cmd.XOR_SFR));
//                 }
//                 break;
//             case TinyPwm::Commands::CLI:
//                 cli();
//                 removeCommand(cmd, 0);
//                 break;
//             case TinyPwm::Commands::SEI:
//                 sei();
//                 removeCommand(cmd, 0);
//                 break;
//             case TinyPwm::Commands::INFO:
//                 twiBuffer.send(TinyPwm::Info());
//                 break;
//             default:
//                 DBG1_PRINTF("invalid command=%u", cmd.command);
//                 return;
//         }
//     }
// }

