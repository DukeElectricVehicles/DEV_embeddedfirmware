#define useCANx
#define useI2Cx
#define useHallSpeedx
#define useWatchdogx
#define DRV8301
#define REVERSEDIR
#define PWMBODGE
#define DEV

#if defined(useCAN) && !defined(__MK20DX256__)
  #error "Teensy 3.2 required for CAN"
#endif
#if defined(useCAN) && defined(useI2C)
  #error "strongly discourage using CAN and I2C at the same time"
#endif

#include <i2c_t3.h>
#include "TimerOne.h"
#include "config.h"
#include "config_DRV.h"
#include "CANCommands.h"
#include "Metro.h"
#include "observer.h"
// #include "utils.h"

uint8_t hallOrder[8] = {255, 25, 152, 195, 89, 57, 120, 255};
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

volatile uint16_t throttle = 0;

void setup(){
  setupPins();
  kickDog();
  #ifdef useCAN
    setupCAN();
  #endif
  #ifdef DRV8301
    setupDRV();
  #endif

  kickDog();

  analogWrite(INHA, 0);
  analogWrite(INHB, 0);
  analogWrite(INHC, 0);

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
    throttle = getThrottle_analog() * 4095;
    #endif

    #ifdef useHallSpeed
      float hallSpeed_tmp = min(hallSpeed_prev_mps,
                                DIST_PER_TICK * 1e6 / (micros()-lastTime_hallSpeed_us)); // allows vel to approach 0
      hallSpeed_LPF_mps = (hallSpeed_alpha)*hallSpeed_LPF_mps + (1-hallSpeed_alpha)*hallSpeed_tmp;
    #endif

    hallISR();
    lastTime_throttle = curTime;
    // Serial.print(throttle);
    // Serial.print(" ");
    Serial.print(recentWriteState);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print("\t");
    Serial.print(m_pll_speed / 360);
    Serial.print("\t");
    Serial.print(updateHall(hallOrder[getHalls()]));
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
    Serial.print('\n');

    kickDog();
  }

  if (checkFaultTimer.check()){
    if (checkDRVfaults()){
      #ifndef KINETISL
      digitalWrite(LED2, HIGH);
      #endif
      Serial.println("DRV fault");
    }
    else{
      #ifndef KINETISL
      digitalWrite(LED2, LOW);
      #endif
    }
  }

  hallISR();

  delayMicroseconds(100);
}

void hallISR()
{
  uint8_t hall = getHalls();
  uint8_t pos = hallOrder[hall];

  float observerPos = updateHall(pos);

  if ((observerPos >= 330) || (observerPos < 30))
    writeState(0);
  else if ((observerPos >= 30) && (observerPos < 90))
    writeState(1);
  else if ((observerPos >= 90) && (observerPos < 150))
    writeState(2);
  else if ((observerPos >= 150) && (observerPos < 210))
    writeState(3);
  else if ((observerPos >= 210) && (observerPos < 270))
    writeState(4);
  else if ((observerPos >= 270) && (observerPos < 330))
    writeState(5);
  else
    writeState(255);
  // Serial.println(observerPos);
  // Serial.println(pos);
  // writeState(pos);
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
  
  #if !defined(KINETISL)
  if(hall == 7)
    digitalWrite(LED1, HIGH);
  else
    digitalWrite(LED1, LOW);
  #endif

  return hall & 0x07;
}

void writeState(uint8_t pos)
{
  //Maybe this is necessary? Might solve some problems with bad handshaking?
  //writeHigh(0);
  //writeLow(0);
  recentWriteState = pos;

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
