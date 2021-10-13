/**
 * Author: sascha_lammers@gmx.de
 */

#include <Arduino.h>
#include <pins_arduino.h>
#include "main.h"
#include "adc.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
// #include <EEPROM.h>
// #include <Button.h>
// #include <ButtonEventCallback.h>
// #include <PushButton.h>
// #include <Bounce2.h>

const char __compile_date__[] PROGMEM = { __DATE__ " " __TIME__ };

void set_version(char *buffer)
{
    sprintf_P(buffer, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

Servo tiltServo;
uint8_t tiltServoAngle = 160;

uint8_t fanPwm = 0;
volatile uint16_t fanTacho = 0;
uint32_t fanTimer = 0;
uint8_t fanSpeedCounter = 0;
bool fanOn = false;

void fan_signal()
{
    fanTacho++;
}

#if 1

Adafruit_NeoPixel pixels(PIN_NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


void setAll(byte red, byte green, byte blue)
{
    for (uint8_t i = 0; i < PIN_NEOPIXEL_NUM; i++) {
        pixels.setPixelColor(i, pixels.Color(red, green, blue));
    }
    pixels.show();
}


void setPixel(int Pixel, byte red, byte green, byte blue)
{
    pixels.setPixelColor(Pixel, pixels.Color(red, green, blue));
}

byte *Wheel(byte WheelPos)
{
    static byte c[3];

    if (WheelPos < 85)
    {
        c[0] = WheelPos * 3;
        c[1] = 255 - WheelPos * 3;
        c[2] = 0;
    }
    else if (WheelPos < 170)
    {
        WheelPos -= 85;
        c[0] = 255 - WheelPos * 3;
        c[1] = 0;
        c[2] = WheelPos * 3;
    }
    else
    {
        WheelPos -= 170;
        c[0] = 0;
        c[1] = WheelPos * 3;
        c[2] = 255 - WheelPos * 3;
    }

    return c;
}

void rainbowCycle(int SpeedDelay)
{
    byte *c;
    uint16_t i, j;

    for (j = 0; j < 256 * 5; j++)
    { // 5 cycles of all colors on wheel
        for (i = 0; i < PIN_NEOPIXEL_NUM; i++)
        {
            c = Wheel(((i * 256 / PIN_NEOPIXEL_NUM) + j) & 255);
            setPixel(i, *c, *(c + 1), *(c + 2));
        }
        pixels.show();
        for(uint8_t i = 0; i < 10; i++) {
            if (Serial.available()) {
                break;
            }
            delayMicroseconds(SpeedDelay * 100UL);
        }
    }
}

#endif

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_FAN_TACHO, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FAN_TACHO), fan_signal, RISING);

    analogWrite(PIN_FAN_PWM, fanPwm);

    pixels.begin();
    pixels.show();

    tiltServo.attach(PIN_TILT_SERVO);
    tiltServo.write(tiltServoAngle);

    char message[32];
    set_version(message);
    Serial.println(message);
    Serial.print(F("Compiled on "));
    Serial.println(FPSTR(__compile_date__));

}

#if 0
// calibratiuon for current and voltage
// 'r' to reset average

#define CAL_METHOD getCurrent1_A
// #define CAL_METHOD getCurrent2_A

static float avg = NAN;
static int16_t num = -20;

void calibrate()
{
    float value = adc.CAL_METHOD();
    if (isnan(avg) || (Serial.available() && Serial.read() == 'r')) {
        avg = value;
        num = -20;
    }
    else {
        avg = ((avg * 4) + value) / 5.0;
    }
    Serial.print(F(_STRINGIFY(CAL_METHOD) " "));
    Serial.print(num++);
    Serial.print(' ');
    Serial.print(value, 5);
    Serial.print(' ');
    Serial.println(avg, 5);
}

void loop()
{
    calibrate();
    delay(500);
}

#else

void moveServo(uint8_t angle)
{
    tiltServo.write(tiltServoAngle);
    Serial.print(F("Servo "));
    Serial.println(angle);
}

void loop()
{
    if (Serial.available()) {
        switch(Serial.read()) {
            case '0':
                analogWrite(PIN_FAN_PWM, 0);
                fanOn = false;
                break;
            case '1':
                fanPwm = 16;
                analogWrite(PIN_FAN_PWM, fanPwm);
                fanOn = true;
                break;
            case '+':
                fanPwm = std::clamp<int16_t>(fanPwm + 4, 16, 255);
                analogWrite(PIN_FAN_PWM, fanPwm);
                fanOn = true;
                break;
            case '-':
                fanPwm = std::clamp<int16_t>(fanPwm - 4, 16, 255);
                analogWrite(PIN_FAN_PWM, fanPwm);
                fanOn = true;
                break;
            case '*':
                tiltServoAngle = std::clamp<int16_t>(tiltServoAngle - 2, 40, 160);
                moveServo(tiltServoAngle);
                break;
            case '/':
                tiltServoAngle = std::clamp<int16_t>(tiltServoAngle + 2, 40, 160);
                moveServo(tiltServoAngle);
                break;
            case '7':
                tiltServoAngle = 40;
                moveServo(tiltServoAngle);
                break;
            case '9':
                tiltServoAngle = 160;
                moveServo(tiltServoAngle);
                break;
        }
        while (Serial.available()) {
            Serial.read();
        }
    }
    delay(10);
    if (++fanSpeedCounter > 200 && fanOn) {
        fanSpeedCounter = 0;
        cli();
        auto count = fanTacho;
        fanTacho = 0;
        sei();
        auto time = millis();
        auto diff = time - fanTimer;
        auto rpm = (30000UL * count) / diff;
        fanTimer = time;
        Serial.print(F("FAN "));
        Serial.print((fanPwm * 100) / 255.0, 2);
        Serial.print(F("% RPM "));
        Serial.println(rpm);
        setAll(fanPwm, 255 - fanPwm, 0);
    }
    else if (!fanOn) {
        rainbowCycle(15);
    }
}

#endif
