# ATMega328P co-processor for Raspberry Pi

This is for a board that controls some LEDs, power buttons and a tilt server. It also adds current measurement and handling the UPS.

This is more or less a private project, but maybe someone is interested in looking at the source code.
I have added a few comments to get a better understanding and described issues I ran into.

*** this is work in progress, the feature list is not completely implemented yet. probably finished in a week or two ***

## I2C Slave

- Running as I2C slave
- PWM fan control with RPM monitoring
- WS2812 LEDs
- Servo for tilt control of the TFT
- Motion sensor to turn off the screen
- Voltage, current and energy monitoring for input and 5V output (MAX471)
- UPS battery monitoring and shutdown when the battery state is critical
- Status LEDs displaying FAN speed or animations
- Interrupt driven power, reboot and hard reset button with timeouts and LED indicators
- Watchdog to reboot the system if it does not respond anymore

## Python Script

 - Read voltage, current and energy
 - UPS low battery shutdown
 - Control fan or set to auto mode
 - LED animations
 - Control tilt servo
 - MQTT support for recording metrics
