/*
  Library for communicating with DPS50xx converters over Serial1
  Jack Proudfoot
  March 19, 2019
*/

#include "DPS.h"
#include "Arduino.h"
#include "stdlib.h"

DPS::DPS() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.setTimeout(330);
}

void DPS::set_voltage(float voltage) {

  voltage = voltage * 100;

  sequence[4] = ((int) voltage >> 8) & 0xFF; //first voltage byte
  sequence[5] = (int) voltage & 0xFF; //second voltage byte

  //generate the corresponding crc seqeunce
  unsigned int generated_crc = crc(sequence, 6);

  //add the crc bytes little endian
  sequence[6] = generated_crc & 0xFF; //crc code for command
  sequence[7] = generated_crc >> 8 & 0xFF; //crc code for command


  // transmits the command to the DPS
  for (int i = 0; i < 8; i++) {
    Serial1.write(sequence[i]);
  }
}
