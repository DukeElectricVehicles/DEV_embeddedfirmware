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
  DPShwSer->begin(9600);
  DPShwSer->setTimeout(50);
  transmitting = false;
  transmit_time = millis();
  msgInQueue = false;
  queueAddr = 0x00;
  queueData = 0x00;
}

bool DPS::update() {

  if (transmitting &&
      (millis() - transmit_time > 500 || DPShwSer->available() >= 8)) {
    
    while(DPShwSer->available()) {
      DPShwSer->read(); // discard useless response
    }
    
    transmitting = false;

    if (msgInQueue){
      set_register(queueAddr, queueData);
      msgInQueue = false;
    }

    return true;
  }
  return false;
}

void DPS::set_voltage(float voltage) {

  set_register(0x00, voltage*100);
  // Serial.print("setting voltage to ");
  // Serial.println(voltage);
  
}

void DPS::set_voltageCurrent(float v, float i) {
  while(DPShwSer->available()){
    DPShwSer->read(); // clear buffer
  }

  uint16_t voltage = (uint16_t) (v*1e2);
  uint16_t current = (uint16_t) (i*1e2);

  writeBuf[0] = 0x01; // ID
  writeBuf[1] = 0x10; // multiple registers
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

  for (int i = 0; i<13 ; i++) {
    DPShwSer->write(writeBuf[i]);
  }

  transmitting = true;
  transmit_time = millis();
}

void DPS::set_on(bool on) {
  set_register(0x09, on ? 1 : 0);
}

void DPS::set_register(uint16_t addr, uint16_t data) {
  if (transmitting){
    msgInQueue = true;
    queueAddr = addr;
    queueData = data;
    return;
  }
  while(DPShwSer->available()){
    DPShwSer->read(); // clear buffer
  }

  writeBuf[0] = 0x01; // ID
  writeBuf[1] = 0x06; // single data code
  writeBuf[2] = (addr >> 8) & 0xFF; // register
  writeBuf[3] = (addr >> 0) & 0xFF;
  writeBuf[4] = (data >> 8) & 0xFF; // data bytes
  writeBuf[5] = (data >> 0) & 0xFF; //

  //generate the corresponding crc seqeunce
  unsigned int generated_crc = crc(writeBuf, 6);

  //add the crc bytes little endian
  writeBuf[6] = generated_crc & 0xFF; //crc code for command
  writeBuf[7] = generated_crc >> 8 & 0xFF; //crc code for command


  // transmits the command to the DPS
  for (int i = 0; i < 8; i++) {
    DPShwSer->write(writeBuf[i]);
  }

  transmitting = true;
  transmit_time = millis();
}