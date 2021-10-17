/**
 * Author: sascha_lammers@gmx.de
 */

#include <Arduino.h>
#include <pins_arduino.h>
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "fan.h"
#include "buttons.h"
#include "twi_buffer.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <interrupt_push_button.h>
// #include <EEPROM.h>

const char __compile_date__[] PROGMEM = { __DATE__ " " __TIME__ };

void enableLEDs();
void disableLEDs();
void pixelsShow();

void setVersion(char *buffer)
{
    sprintf_P(buffer, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

RegMem regMem;
TwiBuffer twiBuffer;
Servo tiltServo;
Fan fan;
Adafruit_NeoPixel pixels(PIN_NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Buttons buttons;
InterruptPushButton<PinChangeFlagsEnum::BUTTON1> button1(PIN_BUTTON1, ENABLE_INTERNAL_PULLUP|PRESSED_WHEN_LOW);
InterruptPushButton<PinChangeFlagsEnum::BUTTON2> button2(PIN_BUTTON2, ENABLE_INTERNAL_PULLUP|PRESSED_WHEN_LOW);
InterruptPushButton<PinChangeFlagsEnum::BUTTON3> button3(PIN_BUTTON3, ENABLE_INTERNAL_PULLUP|PRESSED_WHEN_LOW);

#if 1

void setAll(byte red, byte green, byte blue)
{
    for (uint8_t i = 0; i < PIN_NEOPIXEL_NUM; i++) {
        pixels.setPixelColor(i, pixels.Color(red, green, blue));
    }
    pixelsShow();
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
        pixelsShow();
        for(uint8_t i = 0; i < 10; i++) {
            if (Serial.available()) {
                return;
            }
            delayMicroseconds(SpeedDelay * 100UL);
        }
    }
}

#endif

// LED helpers to disable output during servo movements
// blocking interrupts causes the servo to twitch

bool ledsActive = true;

void enableLEDs()
{
    ledsActive = true;
    pixels.show();
}

void disableLEDs()
{
    ledsActive = false;
    pixels.show();
}

void pixelsShow()
{
    if (ledsActive) {
        pixels.show();
    }
}

uint32_t detatchServoTimer = 0;
uint16_t detatchServoTimerTimeout;

// timeout in millis to enable LED output after moving the servo
void moveServo(int16_t tiltServoAngle, uint16_t timeout = 350)
{
    if (!detatchServoTimer) {
        // disable LEDs while moving servo
        disableLEDs();
        tiltServo.attach(PIN_TILT_SERVO, SERVO_MIN, SERVO_MAX);
    }
    tiltServoAngle = std::clamp<int16_t>(tiltServoAngle, SERVO_MIN, SERVO_MAX);
    tiltServo.write(tiltServoAngle);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        regMem.tiltServoAngle = tiltServoAngle;
    }
    detatchServoTimer = millis();
    detatchServoTimerTimeout = timeout;
}

#if DEBUG

void clearScreen()
{
    Serial.flush();
    Serial.print(F("\033[H"));
    Serial.print(F("\033[2J"));
    Serial.print(F("\033[1;1H"));
    Serial.flush();
}

#endif

void buttonsPressHandler(Button &btn)
{
    // reset timer when the button is pressed
    if (&btn == &button1) {
        digitalWrite(PIN_LED1, LOW);
    }
    else if (&btn == &button2) {
        digitalWrite(PIN_LED2, LOW);
    }
    else if (&btn == &button3) {
        digitalWrite(PIN_LED3, LOW);
    }
}

void buttonsReleaseHandler(Button &btn, uint16_t duration)
{
    #if DEBUG
        Serial.print(F("release "));
        Serial.print((uint16_t)&btn, 16);
        Serial.print(' ');
        Serial.println(duration);
    #endif
    if (&btn == &button1) {
        if (duration > POWER_OFF_DELAY) {
            // disable button
            buttons.stopPinChangeInterrupt(PIN_BUTTON1);
            // send halt signal
        }
        digitalWrite(PIN_LED1, (duration > POWER_OFF_DELAY));
    }
    else if (&btn == &button2) {
        if (duration > HARD_RESET_DELAY) {
            // disable button
            buttons.stopPinChangeInterrupt(PIN_BUTTON2);
            // force hard reset
        }
        else if (duration > SOFT_RESET_DELAY) {
            // leave button active to be able to force hard reset
            // send reboot signal
        }
        digitalWrite(PIN_LED2, (duration > SOFT_RESET_DELAY));
    }
    else if (&btn == &button3) {
        digitalWrite(PIN_LED3, LOW);
    }
}

void buttonsHoldRepeatHandler(Button &btn, uint16_t duration, uint16_t repeat)
{
    #if DEBUG
        Serial.print(F("hold "));
        Serial.print((uint16_t)&btn, 16);
        Serial.print(' ');
        Serial.print(duration);
        Serial.print(' ');
        Serial.println(repeat);
    #endif
    // blink LEDs while the button is held down
    if (&btn == &button1) {
        if (duration > POWER_OFF_DELAY) {
            buttonsReleaseHandler(btn, duration);
        }
        else {
            digitalWrite(PIN_LED1, ((repeat / 4) & 1) == 0);
        }
    }
    else if (&btn == &button2) {
        if (duration > HARD_RESET_DELAY) {
            buttonsReleaseHandler(btn, duration);
        }
        else if (duration > SOFT_RESET_DELAY) {
            digitalWrite(PIN_LED2, (repeat & 1) == 0);
        }
        else {
            digitalWrite(PIN_LED2, ((repeat / 4) & 1) == 0);
        }
    }
    else if (&btn == &button3) {
        // unused
    }
}

void setup()
{
    Serial.begin(115200);

    // I2C client
    regMem = RegMem();
    regMem.begin();

    Wire.begin(regMem.address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // timer setup
        TCCR1A = 0;
        TCCR1B = Timer1::kPreScalerBV;
        TCCR2B = (TCCR2B & 0b11111000) | Timer2::kPreScalerBV;
        // ADC interrupt
        adc.begin();
        // FAN setup
        fan.begin();
    }

    // setup LEDs
    digitalWrite(PIN_LED1, LOW);
    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED2, LOW);
    pinMode(PIN_LED2, OUTPUT);
    digitalWrite(PIN_LED3, LOW);
    pinMode(PIN_LED3, OUTPUT);

    // change pin level interrupts for buttons
    buttons.begin();

    button1.onPress(buttonsPressHandler);
    button1.onHoldRepeat(Buttons::kBlinkInterval, Buttons::kBlinkInterval, buttonsHoldRepeatHandler);
    button1.onRelease(buttonsReleaseHandler);
    button2.onPress(buttonsPressHandler);
    button2.onHoldRepeat(Buttons::kBlinkInterval, Buttons::kBlinkInterval, buttonsHoldRepeatHandler);
    button2.onRelease(buttonsReleaseHandler);
    #if 0
    button3.onPress(buttonsPressHandler);
    button3.onHoldRepeat(..., ..., buttonsHoldRepeatHandler);
    button3.onRelease(buttonsReleaseHandler);
    #endif

    // WS2812 LEDs
    pixels.begin();
    pixels.clear();
    pixels.show();

    char message[32];
    setVersion(message);
    Serial.println(message);
    Serial.print(F("Compiled on "));
    Serial.println(FPSTR(__compile_date__));

    #if DEBUG
        delay(1000);
        clearScreen();
    #endif

    // setup servo and move into default position
    moveServo(SERVO_MAX);

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

void loop()
{
    if (detatchServoTimer && millis() - detatchServoTimer > detatchServoTimerTimeout) {
        // disable servo and enable LEDs after timeout
        tiltServo.detach();
        enableLEDs();
        detatchServoTimer = 0;
    }

    #if DEBUG && 0

        if (Serial.available()) {
            switch(Serial.read()) {
                case '0':
                    fan.setPwm(0);
                    break;
                case '1':
                    fan.setPwm(FAN_MIN_PWM);
                    break;
                case '+':
                    fan.setPwm(std::clamp<int16_t>(fan.getPwm() + 8, FAN_MIN_PWM, FAN_MAX_PWM));
                    break;
                case '-':
                    fan.setPwm(std::clamp<int16_t>(fan.getPwm() - 8, FAN_MIN_PWM, FAN_MAX_PWM));
                    break;
                case '*':
                    moveServo(tiltServo.read() - 3, 150);
                    break;
                case '/':
                    moveServo(tiltServo.read() + 3, 150);
                    break;
                case '7':
                    moveServo(SERVO_MIN);
                    break;
                case '9':
                    moveServo(SERVO_MAX);
                    break;
            }
            while (Serial.available()) {
                Serial.read();
            }
        }
        delay(10);

        if (fan.isFanOn()) {
            auto fanPwm = fan.getPwm();
            setAll(fanPwm, 255 - fanPwm, 0);
        }
        else if (ledsActive) {
            rainbowCycle(15);
        }

        static uint32_t dumpTimer = 0;
        uint32_t time = millis();
        if (time - dumpTimer > 1000) {
            dumpTimer = time;
            clearScreen();
            regMem.dump();
        }

    #endif
}

#endif
