#ifndef CONFIG_H
#define CONFIG_H

#define INHA 9
#define INLA 6
#define INHB 20 
#define INLB 10
#define INHC 23
#define INLC 22

#if defined(KINETISL) // teensy LC doesn't have interrupt on pin 1
  #define HALLA 8
  #define HALLB 5
  #define HALLC 2
  #warning hello
#else
  #define HALLA 0
  #define HALLB 1
  #define HALLC 2
  // #error
#endif

#define THROTTLE 15
#define HALL_SPEED 15 // select

#define DRV_EN_GATE 7
#define DRV_CS 21
#define DRV_MOSI 11
#define DRV_MISO 12
#define DRV_CLK 13

#define ISENSE1 17
#define ISENSE2 16

#if !defined(KINETISL) // if Teensy LC, use these for Halls
  #define LED1 5
  #define LED2 8
#endif

#define MAX_THROTTLE  1000
#define MIN_THROTTLE  300

void setupPins();
float getThrottle_analog();
void receiveEvent(size_t count);
void requestEvent(void);

void kickDog();
void setupWatchdog();

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
  #ifdef useHallSpeed
    return 0;
  #endif
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
  setupWatchdog();

  Serial.println("Setting up pins");
  #if !defined(KINETISL)
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  #endif
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
  
  #ifdef useHallSpeed
    pinMode(HALL_SPEED, INPUT);
  #else
    pinMode(THROTTLE, INPUT_PULLDOWN);
  #endif
  
  pinMode(ISENSE1, INPUT);
  pinMode(ISENSE2, INPUT);

  // change the analog write frequency to 1 kHz
  /* Note that patrick originally had this at 8kHz.  When I changed to ESC2019, I switched to beefier
      FETs with higher gate capacitance which requires more steady state gate current according to:
        I = f*C*V
      I was getting FETHx_OC errors otherwise
      Now there's more audible noise
      According to section 8.2.2.1 (page 26) of the DRV8301 datasheet, GVDD can deliver around 30mA.
      Also note that I couldn't find a 2.2uF cap for GVDD and put a 4.7uF instead - this might be causing issues too
  */
  analogWriteFrequency(INHA, 1000);
  analogWriteFrequency(INHB, 1000);
  analogWriteFrequency(INHC, 1000);
  analogWriteResolution(12); // write from 0 to 2^12 = 4095

  Serial.begin(115200);
}

#ifdef useWatchdog
  void kickDog()
  {
    #if defined(__MKL26Z64__)
      // Teensy LC
      __disable_irq();
      SIM_SRVCOP = 0x55;
      SIM_SRVCOP = 0xAA;
      __enable_irq();
    #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
      // Teensy 3.x
      noInterrupts();
      WDOG_REFRESH = 0xA602;
      WDOG_REFRESH = 0xB480;
      interrupts();
    #else
      #error // watchdog not configured - comment out this line if you are ok with no watchdog
    #endif
  }

  #if defined(__MKL26Z64__)
  extern "C" void startup_early_hook(void) {}
  #endif

  void setupWatchdog()
  {
    #if defined(__MKL26Z64__)
      // Teensy LC
      SIM_COPC = 12; // 1024ms watchdog
      SIM_COPC = 8; // 256ms watchdog
      SIM_COPC = 4; // 32ms watchdog
    #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
      // Teensy 3.x
      // kickDog();
      noInterrupts();                                         // don't allow interrupts while setting up WDOG
      WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
      WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
      delayMicroseconds(1);                                   // Need to wait a bit..
      
      // about 0.25 second timeout
      WDOG_TOVALH = 0x001B;
      WDOG_TOVALL = 0x7740;
      
      // This sets prescale clock so that the watchdog timer ticks at 7.2MHz
      WDOG_PRESC  = 0x400;
      
      // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
      WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
          WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
          WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
      interrupts();
    #else
      #error // watchdog not configured
    #endif

    kickDog();
  }
#else
  void kickDog() {}
  void setupWatchdog() {}
#endif

#endif
