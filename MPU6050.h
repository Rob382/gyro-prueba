#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>

#include "config.h"
#include "utils.h"

#define MPU6050_TWI_ADDRESS 0x68
// sample rate divider: when using a gyro as a clock, the gyroscope output rate is 1kHz.
// the rate at which readings are written to the FIFO is this 1kHz rate divided by (1 + SMPRT_DIV)
#define SMPRT_DIV ((1000 - SERVO_FREQUENCY/2)/SERVO_FREQUENCY)
// degree constants for the 2000 degree per second range, depends on the SMPRT_DIV
#define DEG_60 ((32768L * (1000L /(SMPRT_DIV + 1)) * 60L) / 2000L)
#define DEG_360 (DEG_60 * 6)
#define DEG_180 (DEG_60 * 3)
#define DEG_10 ((DEG_60 + 3) / 6)
// multiplier used to adjust the heading hold current error to the expected error after the look-ahead time
#define HH_LOOK_AHEAD_MULTIPLIER ((HH_LOOK_AHEAD_MS + ((SMPRT_DIV + 1) / 2)) / (SMPRT_DIV + 1))
// extracted from my 'piagma' project (Feb 2014) but simplified for simple gyro operation
// ceptimus  July 2020

// convert from degrees per second at 500 microsecond input to gyro units per update per microsecond of input
#define HH_ADJUST_TARGET(X) (((X) * HH_TARGET_MAX_YAW_RATE * 512L) / 15625L)

// convert ERR (heading hold error in gyro units) and GAIN (nominally 0 to 500) to a servo output pulse width in microseconds, centred on 1500
#define HH_SERVO_OUTPUT(ERR, GAIN) (1500L - ((ERR) * (GAIN) * (SMPRT_DIV + 1)) / 200000L)
class MPU6050 {
  public: 
    static uint16_t start(void); // return zero on success or error code
    static bool poll(void); // read and process the data from the gyros FIFO.  Return true if readings have been updated
    // calibrate gyro (not accelerometer) over five seconds of reasonable stability.  If LED >= 0, flash that output during calibration.
    static void calibrateGyro(int LED);
    static int32_t gyroPosn; // integrated axis reading 819.2 counts per degree of rotation.
    static int16_t gyroRate; // returns average rate between the two most recent update() calls that processed new readings
  private:
    static int16_t gyro; // raw numbers from gyro. At 50 readings/sec on 2000 degree/second range there are 819.2 counts per degree
    static int16_t gyroZero; // zero calibration value
};
#endif
