/*
  Library for communicating with DPS50xx converters over Serialx
  Jack Proudfoot
  March 19, 2019
*/

#include "DPS.h"
#include "Arduino.h"
#include "stdlib.h"

DPS::DPS(HardwareSerial *ser) {
  DPShwSer = ser;
  DPShwSer->begin(19200);
  DPShwSer->setTimeout(50);
  transmitting = false;
  transmit_time = millis()-500;
  msgInQueue = false;
  memset(readBuffer, 0, sizeof(readBuffer));
  readBufferInd = 0;
  powerState = POWER_OFF;
}

enum {
  REG_V_SET = 0x0000,
  REG_I_SET = 0x0001,
  REG_V_OUT = 0x0002,
  REG_I_OUT = 0x0003,
  REG_P_OUT = 0x0004,
  REG_V_IN = 0x0005,
  REG_LOCK = 0x0006,
  REG_PROTECT = 0x0007,
  REG_CVCCSTAT = 0x0008,
  REG_ONOFF = 0x0009,
  REG_B_LED = 0x000A,
  REG_MODEL = 0x000B,
  REG_VERSION = 0x000C,
  REG_EXTRACT_M = 0x0023
};

enum {
  FUNCTION_READ = 0x03,
  FUNCTION_WRITE = 0x06,
  FUNCTION_WRITEMULTIPLE = 0x10
};

bool DPS::update() {
  static int response_index = 0;

  readSerial();

  // Serial.print("test ");
  // Serial.println(get_on());

  if (transmitting && ((millis() - transmit_time) < 600)) // 750 is timeout
    return false;

  if (powerState == POWER_ON_REQUESTED) {
    set_on(true);
  }
  if (powerState == POWER_OFF_REQUESTED) {
    set_on(false);
  }

  if (msgInQueue) {
    // transmits the command to the DPS
    for (int i = 0; i < queueLen; i++) {
      DPShwSer->write(writeBuf[i]);
    }
    transmitting = true;
    transmit_time = millis();
    msgInQueue = false;
  }

  return false;
}

void DPS::readSerial() {
  while (DPShwSer->available()) {
    readBuffer[readBufferInd] = DPShwSer->read();
    readBufferInd++;

    // Serial.print(readBufferInd-1);
    // Serial.print('\t');
    // Serial.println(readBuffer[readBufferInd-1],HEX);

    // ok this is borked but i'm too lazy to fix it
    // this stuff relies on the fact that return messages can only be 8 or 9 bytes
    if (readBufferInd == 9) {
      if (parseMessage(readBuffer, 9)) {
        readBufferInd = 0;
      } else { // shift values over
        for (uint8_t i = 0; i<readBufferInd-1; i++ ){
          readBuffer[i] = readBuffer[i+1];
        }
        readBufferInd--;
      }
    }

    if (readBufferInd == 8) {
      if (parseMessage(readBuffer, 8)) {
        readBufferInd = 0;
      }
    }
  }
}
bool DPS::parseMessage(uint8_t *message, uint8_t len) {
  if (!checkCRC(message, len))
    return false;
  // Serial.println("got message");
  transmitting = false;
  uint8_t address = message[0];
  uint8_t functionCode = message[1];
  uint16_t reg, data;
  switch (functionCode) {
    case FUNCTION_READ:
      reg = (uint16_t)message[3] << 8 | message[4];
      data = 0; // todo
      break;
    case FUNCTION_WRITE:
      reg = (uint16_t) message[2] << 8 | message[3];
      data = (uint16_t) message[4] << 8 | message[5];
      break;
    case FUNCTION_WRITEMULTIPLE:
      reg = (uint16_t) message[2] << 8 | message[3];
      data = (uint16_t) message[4] << 8 | message[5];
      break;
  }
  // Serial.print(address, HEX);
  // Serial.print('\t');
  // Serial.print(reg, HEX);
  // Serial.print('\t');
  // Serial.println(data, HEX);
  // for (uint8_t i = 0; i<len; i++) {
  //   Serial.print(message[i], HEX);
  //   Serial.print('\t');
  // }
  // Serial.println();

  switch (reg) {
    case REG_ONOFF:
      powerState = data ? POWER_ON : POWER_OFF;
      break;
    default:
      break;
  }

  // Serial.println(powerState);
  // Serial.println((powerState==POWER_ON) || (powerState==POWER_OFF_REQUESTED));
  // Serial.println(get_on());

  return true;
}

void DPS::set_voltage(float voltage) {
  voltage = voltage*DPS_VCAL + DPS_ICAL;
  set_register(REG_V_SET, voltage*100);
}

void DPS::set_voltageCurrent(float v, float i) {
  v = v*DPS_VCAL + DPS_VOFF;
  i = i*DPS_ICAL + DPS_IOFF;
  uint16_t voltage = (uint16_t) (v*1e2);
  uint16_t current = (uint16_t) (i*1e2);

  writeBuf[0] = 0x01; // ID
  writeBuf[1] = FUNCTION_WRITEMULTIPLE; // multiple registers
  writeBuf[2] = 0x00; // starting register
  writeBuf[3] = 0x00;
  writeBuf[4] = 0x00; // # of registers
  writeBuf[5] = 0x02;
  writeBuf[6] = 0x04; // # of bytes
  writeBuf[7] = (voltage >> 8) & 0xFF;
  writeBuf[8] = (voltage >> 0) & 0xFF;
  writeBuf[9] = (current >> 8) & 0xFF;
  writeBuf[10] = (current >> 0) & 0xFF;

  uint16_t generated_crc = crc(writeBuf,11);
  writeBuf[11] = (generated_crc >> 0) & 0xFF;
  writeBuf[12] = (generated_crc >> 8) & 0xFF;

  msgInQueue = true;
  memcpy(queueData, writeBuf, 13);
  queueLen = 13;
}

void DPS::set_on(bool on) {
  set_register(REG_ONOFF, on ? 1 : 0);
  powerState = on ? POWER_ON_REQUESTED : POWER_OFF_REQUESTED;
}

void DPS::set_register(uint16_t addr, uint16_t data) {
  // while(DPShwSer->available()){
  //   DPShwSer->read(); // clear buffer
  // }

  writeBuf[0] = 0x01; // ID
  writeBuf[1] = FUNCTION_WRITE; // single data code
  writeBuf[2] = (addr >> 8) & 0xFF; // register
  writeBuf[3] = (addr >> 0) & 0xFF;
  writeBuf[4] = (data >> 8) & 0xFF; // data bytes
  writeBuf[5] = (data >> 0) & 0xFF; //

  //generate the corresponding crc seqeunce
  unsigned int generated_crc = crc(writeBuf, 6);

  //add the crc bytes little endian
  writeBuf[6] = generated_crc & 0xFF; //crc code for command
  writeBuf[7] = generated_crc >> 8 & 0xFF; //crc code for command

  msgInQueue = true;
  memcpy(queueData, writeBuf, 8);
  queueLen = 8;
  return;
}