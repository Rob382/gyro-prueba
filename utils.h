#ifndef utils_h
#define utils_h

#include <Arduino.h>

#if defined(__AVR_ATtiny85__)
  #include "TinyWireM.h"
#endif

class utils {
  public:
    static void TWIstart(void); // configure TWI registers (sets communication speed)
    static uint16_t TWIreadRegisters(uint8_t slaveAddr, uint8_t addr, int numBytes, uint8_t *buff); // returns 0 on success or error code
    static uint16_t TWIwriteRegister(uint8_t slaveAddr, uint8_t addr, uint8_t value); // returns 0 on success or error code
    #if defined(__AVR_ATmega328P__)
      private:
        static uint16_t TWIwaitCheckStatus(uint8_t expectedStatus);
        static uint16_t TWImasterTransmit(uint8_t slaveAddr, uint8_t buff[], int numBytes, bool sendRepeatedStart); // returns 0 on success or error code
        static uint16_t TWImasterReceive(uint8_t slaveAddr, uint8_t buff[], int numBtyes, bool sendStart); // returns 0 on success or error code
    #endif
};
#endif
