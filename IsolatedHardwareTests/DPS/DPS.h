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
      Serial.print("checking...\t");
      for (int i = 0; i<len; i++) {
        Serial.print(payload[i], HEX);
        Serial.print('\t');
      }
      uint16_t crcCalc = crc(payload, len-2);
      uint16_t crcRead = payload[len-1]<<8 | payload[len-2];
      Serial.print(crcCalc,HEX);
      Serial.print('\t');
      Serial.println(crcRead, HEX);
      return crcCalc == crcRead;
    }
};

#endif
