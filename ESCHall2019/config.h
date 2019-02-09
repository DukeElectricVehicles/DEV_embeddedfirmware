#ifndef CONFIG_H
#define CONFIG_H

#define INHA 9
#define INLA 6
#define INHB 20 
#define INLB 10
#define INHC 23
#define INLC 22

#ifdef KINETISL // teensy LC doesn't have interrupt on pin 1
  #error // haven't decided what pins to use for hall sensor replacements
#else
  #define HALLA 0
  #define HALLB 1
  #define HALLC 2
#endif

#define THROTTLE 15

#define DRV_EN_GATE 7
#define DRV_CS 21
#define DRV_MOSI 11
#define DRV_MISO 12
#define DRV_CLK 13

#define ISENSE1 17
#define ISENSE2 16

#define LED1 5
#define LED2 8

#define MAX_THROTTLE  1000
#define MIN_THROTTLE  300

void setupPins();
float getThrottle_analog();
void receiveEvent(size_t count);
void requestEvent(void);

#ifdef useI2C
volatile uint8_t reg = 0;
volatile uint16_t BMSThrottle = 0;
// volatile uint32_t BMSMillis = 0;
extern uint32_t lastTime_CAN;
void receiveEvent(size_t count)
{
  reg = Wire.readByte();

  if(reg == 0x40)//set throttle
  {
    BMSThrottle = Wire.readByte() << 8;
    BMSThrottle |= Wire.readByte();

    lastTime_I2C = millis();
  }
}
void requestEvent(void)
{
  uint8_t data = 0;
  
  if(reg == 0x12)
    data = 0x34;
    
  Wire.write(&data, 1);
}
uint16_t getThrottle_I2C(){
  return BMSThrottle;
}
#endif

#ifdef useCAN
#include "CANCommands.h"
uint16_t CANThrottle = 0;
extern uint32_t lastTime_CAN;
uint16_t getThrottle_CAN(){
  CAN_readBus();
  CAN_getThrottle(&CANThrottle, &lastTime_CAN);
  return CANThrottle;
}
#endif

float getThrottle_analog()
{
  uint16_t rawThrottle = analogRead(THROTTLE);
  float throttle = (rawThrottle - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE);
  
  if(throttle > 1)
    throttle = 1;
  if (throttle < 0)
    throttle = 0;

  return throttle;
}

void setupPins()
{
  Serial.println("Setting up pins");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  //pinMode(FAULT, INPUT);
  
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);
  
  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);
  
  pinMode(THROTTLE, INPUT);
  
  pinMode(ISENSE1, INPUT);
  pinMode(ISENSE2, INPUT);

  // change the analog write frequency to 8 kHz
  analogWriteFrequency(INHA, 8000);
  analogWriteFrequency(INHB, 8000);
  analogWriteFrequency(INHC, 8000);
  analogWriteResolution(12); // write from 0 to 2^12 = 4095

  Serial.begin(115200);
}

#endif
