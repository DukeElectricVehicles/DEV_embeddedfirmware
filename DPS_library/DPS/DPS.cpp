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

float* DPS::get() {

  //communication that reads current voltage and current from DPS
  char sequence[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB};

  char* response = transmit(sequence, 8);

  // voltage in 3rd and 4th byte
  float voltage = ((response[3] << 8) | response[4]) / 100.0;

  // current in 5th and 6th byte
  float current = ((response[5] << 8) | response[6]) / 100.0;

  free(response);

  float* res = (float*) malloc(sizeof(float) * 2);
  res[0] = voltage;
  res[1] = current;
  return res;
}

float DPS::get_voltage() {
  float* res = get();
  float voltage = res[0];
  free(res);
  return voltage;
}

float DPS::get_current() {
  float* res = get();
  float current = res[1];
  free(res);
  return current;
}

float DPS::setVoltage(float voltage) {

  voltage = voltage * 100;

  char* sequence = (char*) malloc(sizeof(char) * 8);
  sequence[0] = 0x01; //slave address
  sequence[1] = 0x06; //function code
  sequence[2] = 0x00; //register to set
  sequence[3] = 0x00; // "    "    "
  sequence[4] = ((int) voltage >> 8) & 0xFF; //first voltage byte
  sequence[5] = (int) voltage & 0xFF; //second voltage byte

  //generate the corresponding crc seqeunce
  unsigned int generated_crc = crc(sequence, 6);

  //add the crc bytes little endian
  sequence[6] = generated_crc & 0xFF; //crc code for command
  sequence[7] = generated_crc >> 8 & 0xFF; //crc code for command


  char* response = transmit(sequence, 8);

  // voltage in 3rd and 4th byte
  float voltage_res = ((response[4] << 8) | response[5]) / 100.0;

  free(response);

  return voltage_res;
}

char* DPS::transmit(char* data, int length)
{
  // transmits the command to the DPS
  for (int i = 0; i < length; i++) {
    Serial1.write(data[i]);
  }

  delay(4);

  char* response = receive_bytes();
  return response;
}

char* DPS::receive_bytes() {
  char* buffer = (char*) malloc(40);

  Serial1.readBytes(buffer, 40); //Reads chars into the buffer

  delay(40);

  return buffer;
}
