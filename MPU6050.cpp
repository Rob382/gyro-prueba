// extracted from my 'piagma' project (Feb 2014) but simplified for simple gyro operation
// ceptimus  July 2020

#include "MPU6050.h"
#include "config.h"

#ifdef DEBUG
  #if defined(__AVR_ATtiny85__)
    #include <SoftwareSerial.h>
    extern SoftwareSerial ss;
  #elif defined(__AVR_ATmega328P__)
    #define ss Serial    
  #endif
  extern char printBuff[];
#endif

int16_t MPU6050::gyro; // raw numbers from gyro. At 50 readings/sec on 2000 degree/second range there are 819.2 counts per degree
int16_t MPU6050::gyroZero; // calibration value
int32_t MPU6050::gyroPosn = 0L; // integrated readings - gives current angle (but can be reset when required)
int16_t MPU6050::gyroRate = 0;

uint16_t MPU6050::start(void) { // return 0 on success or error code
  uint8_t regValue;
  #ifdef DEBUG
    ss.print(F("MPU6050::start() (good result is zero): "));
  #endif
  utils::TWIstart();
  delay(1); // in case called as a result of error when updating
  uint16_t result = utils::TWIreadRegisters(MPU6050_TWI_ADDRESS, 0x75, 1, &regValue); // read WhoAmI register - should be 0x68
  if (result) {
    #ifdef DEBUG
      ss.println(result);
    #endif
    return result;
  }
  if (regValue != 0x68) {
    #ifdef DEBUG
      ss.println(0x68);
    #endif
    return 0x68;
  }

  regValue = 0x80;
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x6B, regValue); // reset chip
  while ((regValue & 0x80) || (result < 250)) { // chip is supposed to clear the reset bit only when reset is complete, but actually needs a little extra time - hence the 250 
    utils::TWIreadRegisters(MPU6050_TWI_ADDRESS, 0x6B, 1, &regValue);
    if (!++result) { // timeout waiting for reset
      #ifdef DEBUG
        ss.println(0x69);
      #endif
      return 0x69;
    }
  }

  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x6B, 1); // PLL with X axis gyroscope reference
  // set sample_rate_divider to give new gyro readings at the desired frequency - one update per servo pulse output 
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x19, SMPRT_DIV);
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x1A, 2); // 98 Hz low pass filter with Fs = 1 kHz
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x1B, 0x18); // 2000 degrees/second
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x1C, 0x10); // +/- 8 g
  // put only gyro rotations for the chosen axis in the FIFO - and not accelerations or temperatures
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x23, 0x40 >> GYRO_AXIS);
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x6A, 0x02); // reset FIFO    
  delay(1);
  utils::TWIwriteRegister(MPU6050_TWI_ADDRESS, 0x6A, 0x40); // enable FIFO
  
  #ifdef DEBUG
    ss.println(0);
  #endif
  return 0;
}

bool MPU6050::poll(void) { // process any queued data from gyro's FIFO.  Return true when new data was read
  bool newData = false;
  uint8_t buff[2];
  int samples;
  int32_t gyroSum;
    
  uint16_t result = utils::TWIreadRegisters(MPU6050_TWI_ADDRESS, 0x72, 2, buff); // read FIFO count from registers 0x72 and 0x73
  // the FIFO can hold up to 10.2 seconds of data (at 50Hz SERVO_FREQUENCY), but we're not interested in anything more than a fraction of a second old
  // also, the readings come as pairs of bytes, so we wait for a second byte if the count is 1
  if (result) { // error reading gyro - try reinitializing
    #ifdef DEBUG
      ss.print(F("\r\nError(A) "));
      ss.println(result);
    #endif
    start();
  } else if (buff[0] || (buff[1] & 0x80)) { // reset if more than 127 bytes in buffer
    #ifdef DEBUG
      ss.print(F("\r\nError(B) "));
      ss.print(buff[0], HEX);
      ss.print(F("  ")); 
      ss.println(buff[1], HEX);
    #endif
    start();
  } else {
    uint8_t count = buff[1] >> 1;
    samples = 0;
    gyroSum = 0L;
    while (count--) { // we have at least one rotation value
      newData = true;
      samples++;
      for (int i = 0; i < 2; i++) {
        result = utils::TWIreadRegisters(MPU6050_TWI_ADDRESS, 0x74, 1, buff + i); // read 2 bytes from fifo
        if (result) { // error reading from fifo - perhaps best to try re-initializing it?
          #ifdef DEBUG
            ss.print(F("\r\nError(C) "));
            ss.println(result);
          #endif
          start();
          count = 0;
          newData = false;
          break;
        }
      }
      gyro = ((buff[0] << 8) | buff[1]) - gyroZero;
      gyroSum += gyro;
    }
  }
  if (newData) { // limit gyroPosn to +/- 180 degrees
    gyroPosn += gyroSum;
    if (gyroPosn >= DEG_180) {
      gyroPosn -= DEG_360;
    } else if (gyroPosn < -DEG_180) {
      gyroPosn += DEG_360;
    }
    gyroRate = (int16_t)(gyroSum / samples);
  }
  return newData;
}

// calibrate gyro (not accelerometer) over five seconds of reasonable stability.  If LED >= 0, flash that output during calibration.
void MPU6050::calibrateGyro(int LED) {
  int16_t maxGyro;
  int16_t minGyro;
  int32_t totalGyro;
  uint8_t samples;

  #ifdef DEBUG
    ss.print(F("Calibrating gyro (0...250) keep it still!    "));
  #endif
  boolean stable = false;
  while (!stable) {
    maxGyro = -32768;
    minGyro = 32767;
    totalGyro = 0L;
    gyroZero = 0;
    
    poll(); // flush FIFO
    samples = 0;
    stable = true;
    while (stable && (samples < 250)) {  // wait for 250 stable samples (5 seconds at 50Hz SERVO_FREQUENCY) and totalize readings
      if (LED >= 0) {
        pinMode(LED, OUTPUT);
        digitalWrite(LED, samples % 50 < 25 ? HIGH : LOW);
      }
      if (poll()) {
        samples++;
        #ifdef DEBUG
          ss.print(F("\b\b\b"));
          sprintf(printBuff, "%03d", samples);
          ss.print(printBuff);
        #endif
        if (gyro > maxGyro) {
          maxGyro = gyro;
        }
        if (gyro < minGyro) {
          minGyro = gyro;
        }
        if ((maxGyro - minGyro) > 125) {
          stable = false;
          break;
        }
        totalGyro += (int32_t)gyro;
      }
    }
  }
  #ifdef DEBUG
    ss.println(F("\b\b\bcomplete"));
  #endif
  gyroZero = (int16_t)((totalGyro + 125L) / 250L);
  gyroPosn = 0L;
}
