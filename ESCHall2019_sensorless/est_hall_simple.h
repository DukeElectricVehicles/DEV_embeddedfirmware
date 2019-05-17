#ifndef EST_HALL_H
#define EST_HALL_H

#include "config.h"

extern void commutate_isr();

// uint8_t hallOrder[8] = {255, 25, 152, 195, 89, 57, 120, 255};
uint8_t hallOrder[8] = {255, 171, 39, 4, 104, 139, 71, 255};
uint8_t hallOrderDiscrete[8] = {255, 5, 1, 0, 3, 4, 2, 255};
uint8_t hysteresis = 9;
#define HALL_SAMPLES 10

void setup_hall();
void hallISR();
uint8_t getHalls();

float curPos;
static float prevPos = -1;
static float realPos = -1;
static uint8_t trapPos = 0;

extern volatile commutateMode_t commutateMode;

void setup_hall() {

	pinMode(HALLA, INPUT);
	pinMode(HALLB, INPUT);
	pinMode(HALLC, INPUT);
	attachInterrupt(HALLA, hallISR, CHANGE);
	attachInterrupt(HALLB, hallISR, CHANGE);
	attachInterrupt(HALLC, hallISR, CHANGE);

}

void hallISR()
{
  static uint32_t prevHallTransitionTime;
  uint8_t hall = getHalls();
  uint8_t pos = (hallOrder[hall]+(uint16_t)185) % 200;

  // curPos = updateHall(pos);
  curPos = pos * 360.0/200;
  if (prevPos > 360){
    prevPos = curPos;
    realPos = curPos;
  }
  // when the hall position changes, the actual sensed position is between the from/to positions
  if (curPos!=prevPos){
    if ((millis()-prevHallTransitionTime) < 100){
      commutateMode == MODE_SENSORLESS_DELAY;
    }
    prevHallTransitionTime = millis();
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

  if (commutateMode == MODE_HALL) {
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