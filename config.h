// config.h file for gyroMPU6050
#ifndef config_h
#define config_h
#include <Arduino.h>

// Unless you know what you're doing, only edit things between the two rows of ====== comments ======= below
#define         RATE_MODE 1
#define HEADING_HOLD_MODE 2
#define            NORMAL 1
#define           REVERSE 2
#define            AXIS_X 0
#define            AXIS_Y 1
#define            AXIS_Z 2

// for the ATtiny85 version:
//   the MPU6050 module SDA line connects to D0 and the SCL line to D2
//   assuming you're using a USBASP for programming:
//     MOSI D0, MISO D1, SCK D2, RST D5.  Burn bootloader for "Internal 16 MHz" clock

// ================ edit the #defines within this section, to configure custom settings ================
// don't add comments on the same line after any #define - this will confuse the compiler
#if defined(__AVR_ATmega328P__)
  // for the 328P, which input and output pins to use (choose numbers from 0 to 7):
  // avoid using pin 1 (serial transmit) if you want to use DEBUG
  // input pins connect to receiver outputs.  the output pin drives your rudder servo
  // if you don't want to use the AUX_INPUT, still define the pin, but leave it disconnected
  #define  RUDDER_INPUT_PIN 2
  #define     AUX_INPUT_PIN 3
  #define RUDDER_OUTPUT_PIN 4
#elif defined(__AVR_ATtiny85__)
  // for the ATtiny85 version only pins 3, 1, 4 are available
  // input pins connect to receiver outputs.  the output pin drives your rudder servo
  // if you don't want to use the AUX_INPUT, leave it disconnected
  #define  RUDDER_INPUT_PIN 3
  #define     AUX_INPUT_PIN 1
  #define RUDDER_OUTPUT_PIN 4
#endif
// servo type selection
// most servos can work at 50Hz - digital servos can often work at higher frequencies which
// results in better performance - but beware that too high a setting may damage your servo
// and possibly result in loss of tail control during flight.  Check the documentation for 
// your servo.
#define SERVO_FREQUENCY 50
// nominal servo centre position (microseconds).  Standard servos use 1500 to 1520 but some
// faster digial servos use 760 or 960 microseconds for their centre position
#define SERVO_CENTRE 1500
// output (rudder servo) travel limits (microseconds)
#define MIN_RUDDER_OUTPUT 1000
#define MAX_RUDDER_OUTPUT 2000


// which axis of gyro to use (depends which way up you mount it): AXIS_Z, AXIS_Y, AXIS_X
#define GYRO_AXIS AXIS_Z

// operating sense NORMAL or REVERSE
#define DIRECTION NORMAL

// default operation mode (if AUX_INPUT_PIN not connected)
// choices are RATE_MODE or HEADING_HOLD_MODE
#define DEFAULT_MODE RATE_MODE

// default gain (if AUX_INPUT_PIN not connected)
// range is 0 (gyro off) to 100 (ridiculously high gain)
#define DEFAULT_GAIN 30

// number of milliseconds to look ahead when operating in heading hold mode
// shorter times can be used for faster servos, and will give a faster response
// but don't set it less than your servo response time, as this will result in 
// unstable (and uncontrollable) operation! 
#define HH_LOOK_AHEAD_MS 230

// target yaw rate in degrees per second at maximum rudder channel deflection (500us from centre) in heading hold mode
// actual yaw rate will always be less than this - and is affected by HH_LOOK_AHEAD_MS - higher LOOK_AHEAD values limit the maximum acheivable yaw rate
#define HH_TARGET_MAX_YAW_RATE 540

// enable debug mode to send serial debug/info 9600 baud data
// on the 328P version this is on the normal Serial output
// on the ATtiny85 version the AUX_INPUT_PIN (normally used as AUX mode/gain input) is used - D1 if you've not altered the #define above
// #define DEBUG
// ====== end of custom settings defines - only edit the other code if you know what you're doing! ======
#endif
