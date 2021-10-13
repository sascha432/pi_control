/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include "def.h"

class ADCInterrupt {
public:
    // internal 1.1V reference
    static constexpr uint8_t kAnalogSource = INTERNAL;
    // ADC analog voltage reference
    static constexpr float kReferenceVoltage = 1.1;
    // values of the ADC 0-1023
    static constexpr uint16_t kAdcValues = 1024;
    // amount of readings before switching to the next pin
    // the first result after switching is discarded
    static constexpr uint8_t kAverageSampleCount = 4;

    static_assert(kAverageSampleCount >= 1, "kAverageSampleCount must be 1 or greater");
    static_assert(kAverageSampleCount < ((1UL << (sizeof(uint16_t) << 3)) / kAdcValues), "reduce kAverageSampleCount to fit into uint16_t (kAdcValues * kAverageSampleCount <= 0xffff)");

    // timer 1 compare match b is the the trigger source for the ADC
    static constexpr float kTriggerFrequencyHz = 2500.0;
    static constexpr uint32_t kReadIntervalInTicks = ((1000000 / kTriggerFrequencyHz) * Timer1::kTicksPerMicrosecond);
    static_assert(kReadIntervalInTicks > 0x077f, "use free running mode for max. read rate");
    static_assert(kReadIntervalInTicks < 0xffff, "overflow, the timer 1 compare match b is 16 bit and limited to ~244Hz");

    // value of the counter to indicate that the analog pin has changed
    static constexpr int8_t kSkipNextReading = -1;

    // voltage calibration
    static constexpr float kU1Cal = 1.0;
    static constexpr float kU2Cal = 1.0;

    // up to 6V
    static constexpr float kU1R1 = 2200.0;
    static constexpr float kU1R2 = 10000.0;
    static constexpr float kU1Divider = (((kU1R2 + kU1R1) / kU1R1) * kU1Cal);

    // up to 22V
    static constexpr float kU2R1 = 2400.0;
    static constexpr float kU2R2 = 47000.0;
    static constexpr float kU2Divider = (((kU2R2 + kU2R1) / kU2R1) * kU2Cal);

    static constexpr float kVoltage1Multiplier = (kU1Cal * (kU1Divider * kReferenceVoltage / (kAdcValues * static_cast<float>(kAverageSampleCount))));
    static constexpr float kVoltage2Multiplier = (kU2Cal * (kU2Divider * kReferenceVoltage / (kAdcValues * static_cast<float>(kAverageSampleCount))));

    // current calibration
    static constexpr float kI1Cal = 0.934066364;
    static constexpr float kI2Cal = 0.927767444;

    // max. current
    static constexpr float kI1MaxA = 3.23 * kI1Cal;
    static constexpr float kI2MaxA = 2.93 * kI2Cal;

    // MAX471

    // R = U / (I * 500µA/V)
    // 0.680 = U / (I * 0.500)
    // 0.680 = (ADC / 1024) / (I * 0.500)
    // max. 3.23A * kI1Cal = kI1MaxA
    inline float ADC2Ampere1(uint16_t adc) const
    {
        return (adc * 25.0) / (kAverageSampleCount * 8704.0 * kI1Cal);
    }

    inline float ADC2Watt1(uint16_t Uadc, uint16_t Iadc) const
    {
        return (Uadc * Iadc * (25.0 * kVoltage1Multiplier)) / (kAverageSampleCount * kAverageSampleCount * 8704.0 * kI1Cal);
    }

    // R = U / (I * 500µA/V)
    // 0.750 = U / (I * 0.500)
    // 0.750 = (ADC / 1024) / (I * 0.500)
    // max. 2.93A * kI2Cal = kI2MaxA
    inline float ADC2Ampere2(uint16_t adc) const
    {
        return (adc) / (kAverageSampleCount * 384.0 * kI2Cal);
    }

    inline float ADC2Watt2(uint16_t Uadc, uint16_t Iadc) const
    {
        return (Uadc * Iadc * kVoltage2Multiplier) / (kAverageSampleCount * kAverageSampleCount * 384.0 * kI2Cal);
    }


    // read order

    enum class AnalogPinType : uint8_t {
        VOLTAGE1,
        CURRENT1,
        VOLTAGE2,
        CURRENT2,
        MAX
    };

    static constexpr auto kNumChannels = static_cast<uint8_t>(AnalogPinType::MAX);

public:
    ADCInterrupt();

    void begin();

    float getVoltage1_V() const;
    float getCurrent1_A() const;
    float getPower1_W() const;

    float getVoltage2_V() const;
    float getCurrent2_A() const;
    float getPower2_W() const;

    uint16_t getADCSum(uint8_t channel) const;
    uint16_t getADCSum(AnalogPinType channel) const;
    uint16_t getADCAvg(uint8_t channel) const;
    uint16_t getADCAvg(AnalogPinType channel) const;

public:
    // public for the ISR
    void _selectNextSourcePin();
    void _addValue(uint16_t value);
    void _resetTimer();

    volatile int8_t _counter;
    volatile uint16_t _sum;
    volatile uint8_t _analogSource;

private:
    uint16_t _results[kNumChannels];
};

extern ADCInterrupt adc;

inline ADCInterrupt::ADCInterrupt() :
    _analogSource(0),
    _results{}
{
}

inline void ADCInterrupt::_selectNextSourcePin()
{
    // store collected values
    auto tmp = _sum;
    _results[_analogSource] = tmp;
    _sum = 0;

    // reset counter
    _counter = kSkipNextReading;
    _analogSource = ((_analogSource + 1) % kNumChannels);
    switch (static_cast<AnalogPinType>(_analogSource)) {
        case AnalogPinType::VOLTAGE1:
            ADMUX = (kAnalogSource << 6) | (PIN_VOLTAGE1 - 14);
            break;
        case AnalogPinType::CURRENT1:
            ADMUX = (kAnalogSource << 6) | (PIN_CURRENT1 - 14);
            break;
        case AnalogPinType::VOLTAGE2:
            ADMUX = (kAnalogSource << 6) | (PIN_VOLTAGE2 - 14);
            break;
        case AnalogPinType::CURRENT2:
            ADMUX = (kAnalogSource << 6) | (PIN_CURRENT2 - 14);
            break;
        case AnalogPinType::MAX:
            break;
    }
}

inline void ADCInterrupt::_addValue(uint16_t value)
{
    _sum += value;
}

inline void ADCInterrupt::_resetTimer()
{
    OCR1B = TCNT1;
    OCR1B += kReadIntervalInTicks;
}

inline void ADCInterrupt::begin()
{
    _sum = 0;
    _analogSource = kNumChannels - 1;
    _selectNextSourcePin();

    // Set ADEN in ADCSRA to enable the ADC.
    // Set ADATE in ADCSRA to enable auto-triggering.
    // Set the Prescaler to 128 (16MHz/128 = 125KHz) (ADPSn)
    // Set ADIE in ADCSRA to enable the ADC interrupt.
    // Set ADSC in ADCSRA to start the ADC conversion
    ADCSRA |= _BV(ADEN) | _BV(ADATE) | (_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)) | _BV(ADIE) | _BV(ADSC);

    // set ADC trigger source - Timer/Counter1 Compare Match B
    ADCSRB |= _BV(ADTS2) | _BV(ADTS0);
    TIMSK1 |= _BV(OCIE1B);

    _resetTimer();
}

inline float ADCInterrupt::getVoltage1_V() const
{
    uint16_t voltage;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        voltage = getADCSum(AnalogPinType::VOLTAGE1);
    }
    return voltage * kVoltage1Multiplier;
}

inline float ADCInterrupt::getVoltage2_V() const
{
    uint16_t voltage;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        voltage = getADCSum(AnalogPinType::VOLTAGE2);
    }
    return voltage * kVoltage2Multiplier;
}

inline float ADCInterrupt::getCurrent1_A() const
{
    uint16_t current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = getADCSum(AnalogPinType::CURRENT1);
    }
    return ADC2Ampere1(current);
}

inline float ADCInterrupt::getCurrent2_A() const
{
    uint16_t current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = getADCSum(AnalogPinType::CURRENT2);
    }
    return ADC2Ampere2(current);
}

inline float ADCInterrupt::getPower1_W() const
{
    uint16_t voltage;
    uint16_t current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        voltage = getADCSum(AnalogPinType::VOLTAGE1);
        current = getADCSum(AnalogPinType::CURRENT1);
    }
    return ADC2Watt1(voltage, current);
}

inline float ADCInterrupt::getPower2_W() const
{
    uint16_t voltage;
    uint16_t current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        voltage = getADCSum(AnalogPinType::VOLTAGE2);
        current = getADCSum(AnalogPinType::CURRENT2);
    }
    return ADC2Watt2(voltage, current);
}

inline uint16_t ADCInterrupt::getADCSum(uint8_t channel) const
{
    return _results[channel];
}

inline uint16_t ADCInterrupt::getADCSum(AnalogPinType channel) const
{
    return getADCSum(static_cast<uint8_t>(channel));
}

inline uint16_t ADCInterrupt::getADCAvg(uint8_t channel) const
{
    return getADCSum(channel) / kAverageSampleCount;
}

inline uint16_t ADCInterrupt::getADCAvg(AnalogPinType channel) const
{
    return getADCAvg(static_cast<uint8_t>(channel));
}
