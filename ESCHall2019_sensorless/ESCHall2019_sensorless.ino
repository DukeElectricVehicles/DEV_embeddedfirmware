#define useCANx
#define useI2Cx
#define useHallSpeedx
#define useWatchdogx
#define DRV8301x
#define PWMBODGE
#define COMPLEMENTARYPWMx
#define DEV
#define SENSORLESS
#define ADCBODGE

#if defined(useCAN) && !defined(__MK20DX256__)
  #error "Teensy 3.2 required for CAN"
#endif
#if defined(useCAN) && defined(useI2C)
  #error "strongly discourage using CAN and I2C at the same time"
#endif

#include <i2c_t3.h>
#include "TimerOne.h"
#include "config.h"
#include "MCpwm_2019sensorless.h"
#include "CANCommands.h"
#include "Metro.h"
#include "observer.h"
#include "BEMFestimator.h"
// #include "utils.h"

// uint8_t hallOrder[8] = {255, 25, 152, 195, 89, 57, 120, 255};
uint8_t hallOrder[8] = {255, 171, 39, 4, 104, 139, 71, 255};
uint8_t hallOrderDiscrete[8] = {255, 5, 1, 0, 3, 4, 2, 255};
uint8_t hysteresis = 9;
#define HALL_SAMPLES 10

#ifdef useHallSpeed
  #define DIST_PER_TICK 0.19948525 // (20 in) * pi / (8 ticks/rev) = (0.199 m/tick)
  uint32_t lastTime_hallSpeed_us = 0;
  float hallSpeed_alpha = .95;
  float hallSpeed_prev_mps = 0;
  float hallSpeed_LPF_mps = 0;
  void hallSpeedISR(){
    uint32_t newTime = micros();
    hallSpeed_prev_mps = DIST_PER_TICK * 1e6 / (newTime - lastTime_hallSpeed_us);
    lastTime_hallSpeed_us = newTime;
  }
#endif

uint32_t lastTime_throttle = 0;
uint32_t lastTime_I2C = 0;
uint32_t lastTime_CAN = 0;
Metro checkFaultTimer(100);
uint8_t recentWriteState;
float curPos;
static float prevPos = -1;
static float realPos = -1;
static uint8_t trapPos = 0;

volatile uint16_t throttle = 0;
bool dir = false;

typedef enum {
  MODE_HALL,
  MODE_SENSORLESS
} commutateMode_t;
commutateMode_t commutateMode = MODE_HALL;

void setup(){
  #ifdef COMPLEMENTARYPWM
    setupPWM();
    writePWM(0,0,0);
  #endif
  setupPins();
  #ifdef SENSORLESS
    // setupADC();
  #endif
  kickDog();
  #ifdef useCAN
    setupCAN();
  #endif

  kickDog();

  attachInterrupt(HALLA, hallISR, CHANGE);
  attachInterrupt(HALLB, hallISR, CHANGE);
  attachInterrupt(HALLC, hallISR, CHANGE);
  #ifdef useHallSpeed
    attachInterrupt(HALL_SPEED, hallSpeedISR, RISING);
  #endif
}

void loop(){
    
  uint32_t curTime = millis();
  
  #ifdef useCAN
    getThrottle_CAN();
  #endif

  if (curTime - lastTime_throttle > 5)
  {
    #ifdef useI2C
    if (curTime - lastTime_I2C < 300){
      throttle = getThrottle_I2C();
      if (throttle == 0)
        throttle = getThrottle_analog() * 4095;
    } else {
      throttle = getThrottle_analog() * 4095;
    }
    #elif defined(useCAN)
    throttle = 4096*pow(getThrottle_CAN()/4096.0,3);
    if ((curTime - lastTime_CAN > 300) || (throttle==0)){
      throttle = getThrottle_analog() * 4095;
    }
    #else
      #ifdef COMPLEMENTARYPWM
      throttle = getThrottle_analog() * MODULO;
      #else
      throttle = getThrottle_analog() * 4096;
      #endif
    #endif

    #ifdef useHallSpeed
      float hallSpeed_tmp = min(hallSpeed_prev_mps,
                                DIST_PER_TICK * 1e6 / (micros()-lastTime_hallSpeed_us)); // allows vel to approach 0
      hallSpeed_LPF_mps = (hallSpeed_alpha)*hallSpeed_LPF_mps + (1-hallSpeed_alpha)*hallSpeed_tmp;
    #endif

    hallISR();
    lastTime_throttle = curTime;
    Serial.print(analogRead(THROTTLE));
    Serial.print("\t");
    Serial.print(throttle);
    Serial.print("\t");
    Serial.print(recentWriteState);
    Serial.print("\t");
    Serial.print(realPos);
    Serial.print("\t");
    Serial.print(hallOrder[getHalls()]);
    Serial.print("\t");
    Serial.print(speed);
    Serial.print("\t");
    Serial.print(m_pll_speed / 360);
    Serial.print("\t");
    Serial.print(realPos);
    Serial.print("\t");
    Serial.print(m_pll_phase);
    // Serial.print('\t');
    // Serial.print(digitalRead(19));
    #ifdef useHallSpeed
    Serial.print('\t');
    Serial.print(digitalRead(HALL_SPEED));
    Serial.print('\t');
    Serial.print('\t');
    Serial.print(hallSpeed_LPF_mps,3);
    #endif

    // Serial.print('\t');
    // Serial.print(analogRead(VS_A));
    // Serial.print('\t');
    // Serial.print(analogRead(VS_B));
    // Serial.print('\t');
    // Serial.print(analogRead(VS_C));
    Serial.print('\n');

    kickDog();
  }

  if (checkFaultTimer.check()){
  }

  hallISR();

  delayMicroseconds(100);
}

void hallISR()
{
  static uint32_t prevHallTransitionTime;
  uint8_t hall = getHalls();
  uint8_t pos = (hallOrder[hall]+(uint16_t)185) % 200;

  curPos = updateHall(pos);
  if (prevPos > 360){
    prevPos = curPos;
    realPos = curPos;
  }
  // when the hall position changes, the actual sensed position is between the from/to positions
  if (curPos!=prevPos){
    if ((millis()-prevHallTransitionTime) < 100){
      commutateMode == MODE_SENSORLESS;
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
    while(realPos < 0){
      realPos += 360;
    }
    trapPos = int(realPos/60);
    trapPos += dir ? 2 : 4;
    while (trapPos >= 6){
      trapPos -= 6;
    }
    writeState(throttle, trapPos);

    // Serial.println(realPos);
    // Serial.println(pos);
    // writeState(pos);
  }
}
uint8_t getHalls()
{
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

void commutate_isr() {
  if (period_us_per_tick >= 100e3) {
    commutateMode == MODE_HALL;
  }
  if (commutateMode == MODE_SENSORLESS){
    trapPos = dir ? (trapPos+1)%6 : (trapPos+5)%6;

    trapPos += dir ? 2 : 4;
    while (trapPos >= 6){
      trapPos -= 6;
    }
    writeState(throttle, trapPos);
  }
}

#ifdef COMPLEMENTARYPWM
void writeState(uint16_t throttle, uint8_t phase) {
  if (throttle < (.01*MODULO)){
    writeTrap(0, -1); // writing -1 floats all phases
  } else {
    writeTrap(throttle, phase);
  }
  // if (recentWriteState != phase){
  //   Serial.print(hallOrder[getHalls()]);
  //   Serial.print("\t");
  //   Serial.println(phase);
  // }
  recentWriteState = phase;
  updatePhase(phase);
  updateDuty((float)throttle / MODULO);
}
#else
void writeState(uint16_t throttle, uint8_t pos)
{
  //Maybe this is necessary? Might solve some problems with bad handshaking?
  //writeHigh(0);
  //writeLow(0);
  recentWriteState = pos;

  switch(pos){
    case 0://HIGH A, LOW B
      writeHigh(0b001);
      writeLow( 0b010);
      break;
    case 1://HIGH C, LOW B
      writeHigh(0b100);
      writeLow( 0b010);
      break;
    case 2://HIGH C, LOW A
      writeHigh(0b100);
      writeLow( 0b001);
      break;
    case 3://HIGH B, LOW A
      writeHigh(0b010);
      writeLow( 0b001);
      break;
    case 4://HIGH B, LOW C
      writeHigh(0b010);
      writeLow( 0b100);
      break;
    case 5://HIGH A, LOW C
      writeHigh(0b001);
      writeLow( 0b100);
      break;
    default:
      writeHigh(0b000);
      writeLow (0b000);
      break;
  }
}
// write the phase to the low side gates
// 1-hot encoding for the phase
// 001 = A, 010 = B, 100 = C
void writeLow(uint8_t phase){
  if (throttle == 0){
    phase = 0;
  }
  digitalWriteFast(INLA, (phase&(1<<0)));
  digitalWriteFast(INLB, (phase&(1<<1)));
  digitalWriteFast(INLC, (phase&(1<<2)));
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