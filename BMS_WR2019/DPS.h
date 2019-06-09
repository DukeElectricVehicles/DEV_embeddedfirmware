/*
  Library for communicating with DPS50xx converters over Serial1
  Jack Proudfoot
  March 19, 2019

  modified Jun 8, 2019 by GDC
*/

// ensure this library description is only included once
#ifndef DPS_h
#define DPS_h

#include <stdint.h>
#include "Arduino.h"

#define DPS_ICAL 0.821
#define DPS_IOFF 0.030
#define DPS_VCAL 1
#define DPS_VOFF 0

#ifndef DPS_ICAL
  #define DPS_ICAL 1
#endif
#ifndef DPS_IOFF
  #define DPS_IOFF 0
#endif
#ifndef DPS_VCAL
  #define DPS_VCAL 1
#endif
#ifndef DPS_VOFF
  #define DPS_VOFF 0
#endif

// library interface description
typedef enum {
  POWER_OFF,
  POWER_ON_REQUESTED,
  POWER_ON,
  POWER_OFF_REQUESTED
} powerState_t;

class DPS
{
  public:

    DPS(HardwareSerial *ser);

    uint8_t writeBuf[16];

    bool update(); // returns true if a transmission as finished
    void set_voltage(float voltage);
    void set_voltageCurrent(float v, float i);
    void set_on(bool on);
    bool get_on() {return (powerState==POWER_ON) || (powerState==POWER_OFF_REQUESTED);};
    void readSerial();
    bool parseMessage(uint8_t *message, uint8_t len);

    bool transmitting;
    uint32_t transmit_time;
    bool msgInQueue;    // a queue of 1 is required because we must wait some time before the previous command goes through
    uint8_t queueData[30];
    uint8_t queueLen;

    uint8_t readBuffer[9];
    uint8_t readBufferInd;

  private:

    HardwareSerial *DPShwSer;

    powerState_t powerState = POWER_OFF;

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

    bool checkCRC(uint8_t *payload, int len) {
      uint16_t crcCalc = crc(payload, len-2);
      uint16_t crcRead = payload[len-1]<<8 | payload[len-2];
      return crcCalc == crcRead;
    }
};

#endif
