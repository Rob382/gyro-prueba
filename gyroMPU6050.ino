// Simple single-axis gyro (like a helicopter tail gyro)
// Using MPU6050 sensor.
// Takes one or two input channels from receiver:
//   First channel is the 'rudder' or 'tail rotor' channel
//   Second (optional) channel controls gyro gain and operating mode:
//     One side of zero for rate mode and the other side for heading hold.  Distance from zero sets the gain
//
// This version compiles for either the ATmega328P (Nano, Uno, etc) or the ATtiny85 (e.g. Digispark boards)
// ceptimus  July 2020
//
// 2020-07-15 swapped to 2000 degree per second gyro range and halved the available gain in both modes
// 2020-07-25 look-ahead damping for hh mode, variable servo output rate and gyro sample rate, other small changes

#include "config.h"
#include "MPU6050.h"

#if defined(__AVR_ATmega328P__)
  #define PIN PIND
  #define PORT PORTD
#elif defined(__AVR_ATtiny85__)
  #define PIN PINB
  #define PORT PORTB
#else
  #error sketch supports either ATmega328P (Nano, Uno, etc.) or ATtiny85
#endif

// (fairly) fast MACROS to wait or test for transitions and to set output levels on I/O pins
#define WAIT_RUDDER_INPUT_HIGH(); while (!(PIN & _BV(RUDDER_INPUT_PIN))) {}
#define WAIT_RUDDER_INPUT_LOW(); while (PIN & _BV(RUDDER_INPUT_PIN)) {}
#define RUDDER_INPUT() (PIN & _BV(RUDDER_INPUT_PIN))
#define AUX_INPUT() (PIN & _BV(AUX_INPUT_PIN))
#define RUDDER_OUTPUT_HIGH(); PORT |= _BV(RUDDER_OUTPUT_PIN);
#define RUDDER_OUTPUT_LOW(); PORT &= ~_BV(RUDDER_OUTPUT_PIN);

#ifdef DEBUG
  char printBuff[16];  
  #if defined(__AVR_ATtiny85__)
    // for debugging on ATtiny85, software serial using AUX_INPUT_PIN for transmit.  Receive isn't used so D5 (RST) is specified.
    // AUX input must be disconnected when debugging - ATtiny85s just don't have enough pins.
    #include <SoftwareSerial.h>
    SoftwareSerial ss(5, AUX_INPUT_PIN);
  #elif defined(__AVR_ATmega328P__)
    #define ss Serial
  #endif
#endif

volatile uint32_t ipPulse; // rudder input pulse width in microseconds.  measured by pinchange interrupts.
volatile uint32_t auxInputPulseWidth; // in microseconds.  measured by pinchange interrupts.

void setup() {
  pinMode(RUDDER_INPUT_PIN, INPUT_PULLUP);
  #if defined(__AVR_ATmega328P__)
    // always enable AUX input on ATmega328P - even in debug mode
    pinMode(AUX_INPUT_PIN, INPUT_PULLUP);
    cli();
    PCICR |= 0x04; // enable port D pin change interrupts
    PCMSK2 = _BV(RUDDER_INPUT_PIN) | _BV(AUX_INPUT_PIN); // for the RUDDER_INPUT and AUX_INPUT pins
    sei();
  #elif defined(__AVR_ATtiny85__) and not defined(DEBUG)
    // pin change interrupts on the ATtiny85 are incompatible with the SoftwareSerial library
    // so in debug mode on the ATtiny85 we have to measure the rudder input pulse width outside interrupts
    pinMode(AUX_INPUT_PIN, INPUT_PULLUP);
    cli();    
    GIMSK |= 0x20; // enable pin change interrupts
    PCMSK = _BV(RUDDER_INPUT_PIN) | _BV(AUX_INPUT_PIN); // for the RUDDER_INPUT and AUX_INPUT pins
    sei();    
  #endif

  #ifdef DEBUG
    ss.begin(9600);
  #endif

  delay(200);
  while (MPU6050::start()) {} // keep retrying while not receiving a good (zero) result
  MPU6050::calibrateGyro(-1);
  RUDDER_OUTPUT_LOW();
  pinMode(RUDDER_OUTPUT_PIN, OUTPUT);  
    
  #ifdef DEBUG
    ss.println(F("You should see gyro and rudder data scrolling below."));
    ss.println(F("Make sure your rudder channel is connected to the gyro and your transmitter is on."));
  #endif
  
  // now set default value for AUX_INPUT, in case it's not connected (or we're in DEBUG)
  // (it could be affected by a startup glitch on the pinchange interrupt, even when not connected)
  #if DEFAULT_MODE == RATE_MODE
    auxInputPulseWidth = 1500 - (DEFAULT_GAIN * 5);
  #elif DEFAULT_MODE == HEADING_HOLD_MODE
    auxInputPulseWidth = 1500 + (DEFAULT_GAIN * 5);
  #else
    #error DEFAULT_MODE must be RATE_MODE or HEADING_HOLD_MODE
  #endif

  ipPulse = 1500UL;
}

uint32_t auxPositiveEdgeMicros = 0UL;
uint32_t rudPositiveEdgeMicros = 0UL;
uint8_t previousPin = 0;
// pinchange interrupt routine
#if defined(__AVR_ATmega328P__)
  ISR(PCINT2_vect) { // pinchange interrupt
#elif defined(__AVR_ATtiny85__)
 #if not defined(DEBUG)
  ISR(PCINT0_vect) { // pinchange interrupt
 #endif
#else
  #error sketch supports either ATmega328P (Nano, Uno, etc.) or ATtiny85
#endif
#if defined(__AVR_ATmega328P__) or not defined(DEBUG)
    uint32_t us = micros();
    uint8_t pin = PIN;
    if (previousPin & _BV(RUDDER_INPUT_PIN)) { // expecting a falling edge on rudder pin
      if (!(pin & _BV(RUDDER_INPUT_PIN))) {
        ipPulse = us - rudPositiveEdgeMicros;
      }
    } else { // expecting a rising edge on rudder pin
      if (pin & _BV(RUDDER_INPUT_PIN)) {
        rudPositiveEdgeMicros = us;
      }      
    }
    if (previousPin & _BV(AUX_INPUT_PIN)) { // expecting a falling edge on auxilliary pin
      if (!(pin & _BV(AUX_INPUT_PIN))) {
        auxInputPulseWidth = us - auxPositiveEdgeMicros;
      }
    } else { // expecting a rising edge on auxilliary pin
      if (pin & _BV(AUX_INPUT_PIN)) {
        auxPositiveEdgeMicros = us;
      }      
    }

    previousPin = pin;
  }
#endif

void loop() {
  static int32_t hhTarget = 0L; // target heading when in heading hold mode
  int32_t opPulse;
  int32_t expectedHeading;
  
  #if defined(__AVR_ATtiny85__) and defined(DEBUG)
    // pin change interrupts on the ATtiny85 are incompatible with the SoftwareSerial library
    // so in debug mode on the ATtiny85 we have to measure the rudder input pulse width outside interrupts
    // wait for positive pulse on rudder input pin, and time that pulse
    WAIT_RUDDER_INPUT_LOW(); // guard check - the input should always be low at this point
    WAIT_RUDDER_INPUT_HIGH();
    uint32_t us = micros();
    WAIT_RUDDER_INPUT_LOW();
    ipPulse = micros() - us;
    if (ipPulse < 800U) {
      ipPulse = 800U;
    } else if (ipPulse > 2200U) {
      ipPulse = 2200U;
    }
    MPU6050::poll();
  #else
    // except when debugging on the ATtiny85 (when the output pulse rate is synchronized to the rudder input pulse rate)
    // wait for the next gyro sample to arrive - the gyro sample rate is configured to provide samples at 
    // SERVO_FREQUENCY Hz, where SERVO_FREQUENCY is defined in config.h to suit your servo
    while (!MPU6050::poll()) {}
  #endif
  // take atomic copies of rudder and auxilliary input pulse widths - so that interrupts occurring mid-read don't give erroneous results
  cli();
  int32_t rud = (int32_t)ipPulse;
  int32_t aux = (int32_t)auxInputPulseWidth;
  sei();
  
  if (auxInputPulseWidth < 1500) { // rate mode
    #if DIRECTION == NORMAL
        opPulse = rud + (((int32_t)MPU6050::gyroRate * (1500L - aux)) >> 10);
    #elif DIRECTION == REVERSE
        opPulse = rud - (((int32_t)MPU6050::gyroRate * (1500L - aux)) >> 10);
    #else
      #error DIRECTION must be NORMAL or REVERSE
    #endif
    // when switching from rate to heading hold mode, the last heading in 'rate' becomes the initial target for heading hold
    expectedHeading = hhTarget = MPU6050::gyroPosn; 
  } else { // heading hold mode
    // adjust hhTarget (bearing) from rudder-stick input
    #if DIRECTION == NORMAL
      hhTarget += HH_ADJUST_TARGET(1500L - rud); 
    #elif DIRECTION == REVERSE
      hhTarget -= HH_ADJUST_TARGET(1500L - rud);;
    #endif
    // limit hhTarget to +/- 180 degree range
    if (hhTarget >= DEG_180) {
      hhTarget -= DEG_360;
    } else if (hhTarget < -DEG_180) {
      hhTarget += DEG_360;
    }
    // calculate expected heading in HH_LOOK_AHEAD_MS milliseconds from now, based on current heading and current rotation rate
    expectedHeading = MPU6050::gyroPosn + ((int32_t)MPU6050::gyroRate) * HH_LOOK_AHEAD_MULTIPLIER;
    int32_t hhError = hhTarget - expectedHeading;
    
    // normalize hhError in case of +/- 180 degree crossing
    if (hhError >= DEG_180) {
      hhError -= DEG_360;
    } else if (hhError < -DEG_180) {
      hhError += DEG_360;
    }

    // limit hhError to +/- 60 degrees
    if (hhError > DEG_60) {
      hhError = DEG_60;
    } else if (hhError < -DEG_60) {
      hhError = -DEG_60;
    }

    // clip hhTarget to within +/- 60 degrees of current heading, and normalize to the +/- 180 degree range
    
    hhTarget = expectedHeading + hhError;
    if (hhTarget >= DEG_180) {
      hhTarget -= DEG_360;
    } else if (hhTarget < -DEG_180) {
      hhTarget += DEG_360;
    }

    opPulse = HH_SERVO_OUTPUT(hhError, aux - 1500L);
  }

  opPulse += SERVO_CENTRE - 1500; // convert from internal 1500 servo centre to value specifed in config file
  
  // limit output throw (servo travel limits specified in user defines)
  if (opPulse > MAX_RUDDER_OUTPUT) {
    opPulse = MAX_RUDDER_OUTPUT;
  } else if (opPulse < MIN_RUDDER_OUTPUT) {
    opPulse = MIN_RUDDER_OUTPUT;
  }

  // send the pulse to the servo.
  if (micros() < 4294964296UL) { // micros() wraps back to zero approximately every 71.6 minutes - skip one output frame if that's about to happen 
    uint32_t us = micros() + opPulse;
    RUDDER_OUTPUT_HIGH();
    while (micros() < us) {}
    RUDDER_OUTPUT_LOW();
  }  

  #ifdef DEBUG
    static uint32_t nextPrint = millis();
    if (millis() > nextPrint) {
      nextPrint += 500UL;  // print twice per second
      ss.print(HH_LOOK_AHEAD_MULTIPLIER);
      ss.print(F("  expectedHeading: "));
      sprintf(printBuff, "%05d", (expectedHeading * 100L) / DEG_10);
      if (printBuff[0] == '0') {
        printBuff[0] = ' ';  // replace leading zero on +ve numbers with space
      }
      // units in printBuff are tenths of degrees, so squeeze in a decimal point.
      printBuff[5] = printBuff[4];
      printBuff[4] = '.';
      printBuff[6] = '\0'; // terminate string
      ss.print(printBuff);
      ss.print("  ");
      ss.print(F("  hhTarget: "));
      sprintf(printBuff, "%05d", (hhTarget * 100L) / DEG_10);
      if (printBuff[0] == '0') {
        printBuff[0] = ' ';  // replace leading zero on +ve numbers with space
      }
      // units in printBuff are tenths of degrees, so squeeze in a decimal point.
      printBuff[5] = printBuff[4];
      printBuff[4] = '.';
      printBuff[6] = '\0'; // terminate string
      ss.print(printBuff);
      ss.print("  ");
      ss.print(aux < 1500L ? F("Rate mode: ") : F("Heading hold mode: "));
      ss.print(abs(1500L - aux) / 5);
      ss.print("%  ");
      #if GYRO_AXIS == AXIS_X
        ss.print(F("Chosen axis(X) rate: "));
      #elif GYRO_AXIS == AXIS_Y
        ss.print(F("Chosen axis(Y) rate: "));
      #elif GYRO_AXIS == AXIS_Z
        ss.print(F("Chosen axis(Z) rate: "));
      #else
        #error GYRO_AXIS must be AXIS_X, AXIS_Y, or AXIS_Z
      #endif
      sprintf(printBuff, "%06d", MPU6050::gyroRate);
      if (printBuff[0] == '0') {
        printBuff[0] = ' ';  // replace leading zero on +ve numbers with space
      }
      ss.print(printBuff);
      ss.print(F("  posn: "));
      sprintf(printBuff, "%05d", (MPU6050::gyroPosn * 100L) / DEG_10);
      if (printBuff[0] == '0') {
        printBuff[0] = ' ';  // replace leading zero on +ve numbers with space
      }
      // units in printBuff are tenths of degrees, so squeeze in a decimal point.
      printBuff[5] = printBuff[4];
      printBuff[4] = '.';
      printBuff[6] = '\0'; // terminate string
      ss.print(printBuff);
      ss.print(F("  Rudder channel:"));
      ss.print(rud);
      ss.print(F("  Servo output:"));
      ss.println(opPulse);
    }    
  #endif
}
