#include "utils.h"

void utils::TWIstart(void) { // configure for two wire interface operation
  #if defined(__AVR_ATmega328P__)
    // for 'standard' 400 kHz SCL frequency, set TWSR = 1 and TWBR = 3.  TWBR = 4 gives 200 kHz, etc.
    TWSR = 1; // set prescaler to 4
    TWBR = 3; // SCL = CPU clock / (16 + 2 * TWBR * prescaler)  
  #elif defined(__AVR_ATtiny85__)
    TinyWireM.begin();
  #endif
}

uint16_t utils::TWIreadRegisters(uint8_t slaveAddr, uint8_t addr, int numBytes, uint8_t *buff) { // returns 0 on success or error code
  #if defined(__AVR_ATmega328P__)
    uint16_t result = TWImasterTransmit(slaveAddr, &addr, 1, true);
    if (!result) {
      result = TWImasterReceive(slaveAddr, buff, numBytes, false);
    }
    return result;
  #elif defined(__AVR_ATtiny85__)
    uint8_t result;
    TinyWireM.beginTransmission(slaveAddr);
    TinyWireM.send(addr);
    result = TinyWireM.endTransmission();
    if (!result) {
      result = TinyWireM.requestFrom(slaveAddr, numBytes);
      if (!result) {
        while(numBytes--) {
          if (!TinyWireM.available()) {
            result = 0x0B;
            break;
          }
          *buff++ = TinyWireM.receive();
        }
      }
    }
    return (uint16_t)result;
  #endif
}

uint16_t utils::TWIwriteRegister(uint8_t slaveAddr, uint8_t addr, uint8_t value) { // returns 0 on success or error code
  #if defined(__AVR_ATmega328P__)
    uint8_t buff[2] = {addr, value};
    return TWImasterTransmit(slaveAddr, buff, 2, false);  
  #elif defined(__AVR_ATtiny85__)
    uint8_t result;
    TinyWireM.beginTransmission(slaveAddr);
    TinyWireM.send(addr);  
    TinyWireM.send(value);
    result = TinyWireM.endTransmission();
    return (uint16_t)result;
  #endif
}

#if defined(__AVR_ATmega328P__)
  // 328P twin wire interface utility routines - ceptimus March-2013
  uint16_t utils::TWIwaitCheckStatus(uint8_t expectedStatus) {
    uint16_t statusCode = 1000; // initially use statusCode for timeout check
    
    while (!(TWCR & (1 << TWINT))) { // wait for TWINT flag set
      if (!--statusCode) { // timeout
        TWCR = TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send STOP condition
        return 0xFF00 | expectedStatus; // 0xFFXX indicates timeout when waiting for status XX
      }
    }
    
    statusCode = TWSR & 0xF8;
    if (statusCode == expectedStatus) {
      return 0;
    }
    else {
      if (statusCode == 0x38) { // arbitration lost
        TWCR = (1 << TWINT) | (1 << TWEN); // release TW serial bus and enter slave mode
      }
      else { // other unexpected response
        TWCR = TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send STOP condition
      }
      return statusCode << 8 | expectedStatus; // 0xYYXX indicates received status YY when expecting status XX
    }  
  }
  
  // 328P twin wire interface utility routines - ceptimus March-2013
  uint16_t utils::TWImasterTransmit(uint8_t slaveAddr, uint8_t buff[], int numBytes, bool sendRepeatedStart) { // returns 0 on success or error code
    uint16_t result = 0; // continue while result remains zero, or terminate and return it as error code
    
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // send START condition
    if (!(result = TWIwaitCheckStatus(0x08))) {
      TWDR = slaveAddr << 1; // send slave address (and enter master transmit mode)
      TWCR = (1 << TWINT) | (1 << TWEN); // transmit address
      if (!(result = TWIwaitCheckStatus(0x18))) {
        for (int i = 0; i < numBytes; i++) {
          TWDR = buff[i];
          TWCR = (1 << TWINT) | (1 << TWEN); // send data bytes
          if (result = TWIwaitCheckStatus(0x28)) {
            break;
          }
        }
        if (!result && sendRepeatedStart) { // do this when expecting a response from slave
          TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // send repeated START condition
          result = result = TWIwaitCheckStatus(0x10); // check for correct response to repeated START
        }
        else { // no response expected from slave
          TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send STOP condition
        }
      }
    }
    return result;
  }
  
  // 328P twin wire interface utility routines - ceptimus March-2013
  uint16_t utils::TWImasterReceive(uint8_t slaveAddr, uint8_t buff[], int numBytes, bool sendStart) { // returns 0 on success or error code
    uint16_t result = 0; // continue while result remains zero, or terminate and return it as error code
    
    if (sendStart) {
      TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // send START condition
      result = TWIwaitCheckStatus(0x08);
    }
    if (!result) {
      TWDR = (slaveAddr << 1) | 0x01; // send slave address (and enter master receive mode)
      TWCR = (1 << TWINT) | (1 << TWEN); // transmit address
      if (!(result = TWIwaitCheckStatus(0x40))) {
        for (int i = 0; i < numBytes - 1; i++) {
          TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN); // receive data byte and return ACK for all but last expected byte
          if (result = TWIwaitCheckStatus(0x50)) {
            break;
          }
          buff[i] = TWDR; // receive data bytes
        }
        if (!result) {
          TWCR = (1 << TWINT) | (1 << TWEN); // receive data byte and return NAK for the last expected byte
          if (!(result = TWIwaitCheckStatus(0x58))) {
            buff[numBytes - 1] = TWDR;
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // send STOP condition
          }
        }
      }
    }
    return result;
  }
#endif
