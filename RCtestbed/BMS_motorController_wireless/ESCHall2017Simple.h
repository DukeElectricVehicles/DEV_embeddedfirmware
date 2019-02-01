#include "TimerOne.h"
#include "config_ESC.h"

#ifndef ESCHALL2017SIMPLE_H
#define ESCHALL2017SIMPLE_H

uint8_t hallOrder[] = {255, 1, 3, 2, 5, 0, 4, 255}; //for ebike hub motor B
#define HALL_SHIFT 2

//uint8_t hallOrder[] = {255, 5, 3, 4, 1, 0, 2, 255}; //for gemini hub motor
//#define HALL_SHIFT 3

//uint8_t hallOrder[] = {255, 1, 3, 2, 5, 0, 4, 255}; //for maxwell motor
//#define HALL_SHIFT 2
#define HALL_SAMPLES 10

volatile uint16_t throttle = 0;

uint8_t getHalls(void);
void ESCupdate(uint16_t throttle);
void hallISR();
void writeLow(uint8_t phase);
void writeState(uint8_t pos);
void writeHigh(uint8_t phase);


void hallISR()
{
  uint8_t hall = getHalls();
  uint8_t pos = hallOrder[hall];
  if(pos > 6)
  {
    writeState(255);//error
    return;
  }

  pos = (pos + HALL_SHIFT) % 6;
  // Serial.println(pos);
  writeState(pos);
}

void ESCupdate(uint16_t throttlein)
{
  volatile uint16_t driverThrottle = throttlein;
  throttle = driverThrottle;
  if ((analogRead(A0)*215/10) > 10){ // UVLO
    hallISR();
  }
}

uint8_t getHalls()
{
  uint32_t hallCounts[] = {0, 0, 0};
  for(uint32_t i = 0; i < HALL_SAMPLES; i++)
  {
    hallCounts[0] += digitalReadFast(HALL1);
    hallCounts[1] += digitalReadFast(HALL2);
    hallCounts[2] += digitalReadFast(HALL3);
  }

  uint8_t hall = 0;
  if(hallCounts[0] > (HALL_SAMPLES/2))  hall |= 1<<0;
  if(hallCounts[1] > (HALL_SAMPLES/2))  hall |= 1<<1;
  if(hallCounts[2] > (HALL_SAMPLES/2))  hall |= 1<<2;
  
  if(hall == 7)
    digitalWrite(LED1, HIGH);
  else
    digitalWrite(LED1, LOW);
  
  return hall & 0x07;
}

// write the phase to the low side gates
// 1-hot encoding for the phase
// 001 = A, 010 = B, 100 = C
void writeLow(uint8_t phase){
  digitalWriteFast(INLA, (phase&(1<<0)));
  digitalWriteFast(INLB, (phase&(1<<1)));
  digitalWriteFast(INLC, (phase&(1<<2)));
}


void writeState(uint8_t pos)
{
  //Maybe this is necessary? Might solve some problems with bad handshaking?
  //writeHigh(0);
  //writeLow(0);

  switch(pos){
    case 0://LOW A, HIGH B
      writeLow( 0b001);
      writeHigh(0b010);
      break;
    case 1://LOW A, HIGH C
      writeLow( 0b001);
      writeHigh(0b100);
      break;
    case 2://LOW B, HIGH C
      writeLow( 0b010);
      writeHigh(0b100);
      break;
    case 3://LOW B, HIGH A
      writeLow( 0b010);
      writeHigh(0b001);
      break;
    case 4://LOW C, HIGH A
      writeLow( 0b100);
      writeHigh(0b001);
      break;
    case 5://LOW C, HIGH B
      writeLow( 0b100);
      writeHigh(0b010);
      break;
  }
}

// write the phase to the high side gates
// 1-hot encoding for the phase
// 001 = A, 010 = B, 100 = C
void writeHigh(uint8_t phase){
  switch(phase){
  case 0b001: // Phase A
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
    analogWrite(INHA, throttle);
    break;
  case 0b010: // Phase B
    analogWrite(INHA, 0);
    analogWrite(INHC, 0);
    analogWrite(INHB, throttle);
    break;
  case 0b100:// Phase C
    analogWrite(INHA, 0);
    analogWrite(INHB, 0);
    analogWrite(INHC, throttle);
    break;
  default://ALL OFF
    analogWrite(INHA, 0);
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
  }
}

#endif
