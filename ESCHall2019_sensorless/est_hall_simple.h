#ifndef EST_HALL_H
#define EST_HALL_H

#include "config.h"

extern void commutate_isr();

// uint8_t hallOrder[8] = {255, 25, 152, 195, 89, 57, 120, 255};
static const uint8_t hallOrder[8] = {255, 171, 39, 4, 104, 139, 71, 255}; // Mitsuba
static const uint8_t hallOrderDiscrete[8] = {255, 5, 1, 0, 3, 4, 2, 255}; // Mitsuba
// static const uint8_t hallOrder[8] = {255, 137, 70, 102, 2, 170, 37, 255}; // Koford
// static const uint8_t hallOrderDiscrete[8] = {255, 4, 2, 3, 0, 5, 1, 255}; // Koford
static const uint8_t hysteresis = 9;
#define HALL_SAMPLES 10

void setup_hall();
void hallISR();
uint8_t getHalls();

float curPos;
static float prevPos = -1;
static float realPos = -1;
static uint8_t trapPos = 0;

extern volatile commutateMode_t commutateMode;

volatile uint32_t period_hallsimple_usPerTick;

void setup_hall() {

  pinMode(HALLEN, OUTPUT);
  digitalWrite(HALLEN, HIGH);

	pinMode(HALLA, INPUT);
	pinMode(HALLB, INPUT);
	pinMode(HALLC, INPUT);
	attachInterrupt(HALLA, hallISR, CHANGE);
	attachInterrupt(HALLB, hallISR, CHANGE);
	attachInterrupt(HALLC, hallISR, CHANGE);

}

void hallnotISR() {
  cli();
  hallISR();
  sei();
}
void hallISR()
{
  static volatile uint32_t prevHallTransitionTime[6];
  static volatile uint8_t prevHallTransitionIndex = 0;
  volatile uint8_t hall = getHalls();
  volatile uint8_t pos = (hallOrder[hall]+(uint16_t)185) % 200;
  volatile uint32_t curMicros = micros();

  if (curMicros < prevHallTransitionTime[prevHallTransitionIndex]) { // this is happening but I can't figure ut why
    return;
  }
  if (((int32_t)(curMicros - prevHallTransitionTime[prevHallTransitionIndex])) > (1.1*period_hallsimple_usPerTick)){ // speed declines to 0 if motor stopped
    if (period_hallsimple_usPerTick > (uint32_t)(1<<31)) {
      Serial.println("***");
      for (uint8_t i = 0; i<6; i++){
        Serial.print(prevHallTransitionTime[i]);
        Serial.print('\t');
      }
      Serial.print(prevHallTransitionIndex);
      Serial.print('\t');
      Serial.print(curMicros);
      Serial.print('\n');
    } else {
      period_hallsimple_usPerTick = curMicros - prevHallTransitionTime[prevHallTransitionIndex];
    }
  }

  // curPos = updateHall(pos);
  curPos = pos * 360.0/200;
  if (prevPos > 360){
    prevPos = curPos;
    realPos = curPos;
  }
  // when the hall position changes, the actual sensed position is between the from/to positions
  if (curPos!=prevPos){
    // period_hallsimple_usPerTick += .3*((int32_t)(curMicros - prevHallTransitionTime) - period_hallsimple_usPerTick);
    
    // period_hallsimple_usPerTick = 0.7 * (period_hallsimple_usPerTick) + 0.3 * (curMicros - prevHallTransitionTime);
    // period_hallsimple_usPerTick = min(period_hallsimple_usPerTick, 100000);
    prevHallTransitionIndex = (prevHallTransitionIndex+1) % 6;
    period_hallsimple_usPerTick = (curMicros - prevHallTransitionTime[prevHallTransitionIndex]) / 6;
    if ((int32_t)period_hallsimple_usPerTick < 0) {
      Serial.println();
      for (uint8_t i = 0; i<6; i++){
        Serial.print(prevHallTransitionTime[i]);
        Serial.print('\t');
      }
      Serial.print(prevHallTransitionIndex);
      Serial.print('\t');
      Serial.print(curMicros);
      Serial.print('\n');
    }
    prevHallTransitionTime[prevHallTransitionIndex] = curMicros;
    // find the average position of curPos and prevPos
    float diffHall = curPos - prevPos;
    if (fabsf(diffHall) < 180){
      realPos = (curPos + prevPos) / 2;
    } else {
      realPos = (curPos + prevPos) / 2 + 180;
    }
    realPos = fmodf(realPos + 360, 360);

    prevPos = curPos;
  }
  // realPos = fmodf(realPos, 360);

  while(realPos < 0) {
    realPos += 360;
  }
  trapPos = int(realPos/60);
  trapPos += dir ? 2 : 4;
  while (trapPos >= 6){
    trapPos -= 6;
  }
	commutate_isr(trapPos, MODE_HALL);

}

uint8_t getHalls() {
  uint32_t hallCounts[] = {0, 0, 0};
  for(uint32_t i = 0; i < HALL_SAMPLES; i++)
  {
    hallCounts[0] += digitalReadFast(HALLA);
    hallCounts[1] += digitalReadFast(HALLB);
    hallCounts[2] += digitalReadFast(HALLC);
  }

  uint8_t hall = 0;
  if(hallCounts[0] > (HALL_SAMPLES/2))  hall |= 1<<0;
  if(hallCounts[1] > (HALL_SAMPLES/2))  hall |= 1<<1;
  if(hallCounts[2] > (HALL_SAMPLES/2))  hall |= 1<<2;

  return hall & 0x07;
}

#endif