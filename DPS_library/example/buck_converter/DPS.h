/*
  Library for communicating with DPS50xx converters over Serial1
  Jack Proudfoot
  March 19, 2019
*/

// ensure this library description is only included once
#ifndef DPS_h
#define DPS_h

#include <stdint.h>
#include "Arduino.h"

// library interface description
class DPS
{
  public:

    DPS(HardwareSerial *ser);

    // char sequence[8] = {0x01, 0x06, 0x00, 0x00, 0x01, 0xF4, 0x89, 0xdd};
    uint8_t writeBuf[16];

    bool update(); // returns true if a transmission as finished
    void set_voltage(float);
    void set_voltageCurrent(float v, float i);
    void set_on(bool on);

    bool transmitting;
    uint32_t transmit_time;

  private:

    HardwareSerial *DPShwSer;

    void set_register(uint16_t addr, uint16_t data);

    //crc function taken from https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16
    uint16_t crc(uint8_t *bufin, int len) {
      uint8_t* buf = (uint8_t*) bufin;
      uint16_t crc = 0xFFFF;
      for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];    // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {    // Loop over each bit
          if ((crc & 0x0001) != 0) {      // If the LSB is set
            crc >>= 1;                    // Shift right and XOR 0xA001
            crc ^= 0xA001;
          }
          else                            // Else LSB is not set
            crc >>= 1;                    // Just shift right
        }
      }

      return crc;
    }
};

#endif
