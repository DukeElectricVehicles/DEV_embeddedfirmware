#ifndef CONFIG_H
#define CONFIG_H

#define TPM_C F_BUS            // core clock, for calculation only
#define PWM_FREQ 5000            //  PWM frequency [Hz]
#define MODULO (TPM_C / PWM_FREQ) // calculation the modulo for FTM0

#ifdef ADCBODGE
  #define VS_A A2 // A7 is only capable of using ADC0
  #define VS_B A10
  #define VS_C A3 // A11 is only capable of using ADC0
  #define VS_AOLD A7
  #define VS_COLD A11
#else
  #define VS_A A7
  #define VS_B A10
  #define VS_C A11
#endif

#ifndef COMPLEMENTARYPWM
  #define INLA 22
  #define INHA 23
  #define INLB 9
  #define INHB 10
  #define INLC 6
  #define INHC 20
#endif

#define HALLA 5
#define HALLB 7
#define HALLC 8
#define HALLEN 15

#define THROTTLE 14 // IMPORTANT: BODGE WIRE TO PIN 29/A18
#define REGENBUTTON 16
#define ALERT_PIN 12 // INA alert

// #define MAX_THROTTLE  1000
// #define MIN_THROTTLE  300
#define MAX_THROTTLE 750
#define MIN_THROTTLE 300

typedef enum {
  MODE_HALL,
  MODE_HALL_PLL,
  MODE_SENSORLESS_DELAY
} commutateMode_t;

typedef enum {
  INPUT_THROTTLE,
  INPUT_UART,
  INPUT_I2C,
  INPUT_CAN
} inputMode_t;

typedef enum {
  CONTROL_DUTY,
  CONTROL_CURRENT_OPENLOOP,
  CONTROL_CURRENT_CLOSEDLOOP
} controlMode_t;

typedef enum {
  PWM_COMPLEMENTARY,
  PWM_NONSYNCHRONOUS,
  PWM_BIPOLAR // not yet implemented
} pwmMode_t;

void setupPins();
float getThrottle_analog();
void receiveEvent(size_t count);
void requestEvent(void);

void kickDog();
void setupWatchdog();

void setupPins()
{
  Serial.println("Setting up pins");
  
  #ifdef useHallSpeed
    pinMode(HALL_SPEED, INPUT);
  #else
    pinMode(THROTTLE, INPUT_PULLDOWN);
  #endif
  
  Serial.begin(115200);
}

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

#include "ADC_2019sensorless.h" // for throttle
float getThrottle_analog() // plz don't call this too fast
{
  #ifdef useHallSpeed
    return 0;
  #endif
  #ifdef SENSORLESS
    uint16_t rawThrottle = getThrottle_ADC();
  #else
    uint16_t rawThrottle = analogRead(THROTTLE);
  #endif
  float throttle = (rawThrottle - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE);
  
  if(throttle > 1)
    throttle = 1;
  if (throttle < 0)
    throttle = 0;

  return throttle;
}

float getBusVoltage() {
  pinMode(INLA, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INLC, OUTPUT);
  pinMode(INHA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(VS_A, INPUT);
  analogReadRes(12);

  digitalWrite(INLA, LOW);
  digitalWrite(INLB, LOW);
  digitalWrite(INLC, LOW);
  digitalWrite(INHA, HIGH);
  digitalWrite(INHB, LOW);
  digitalWrite(INHC, LOW);
  delayMicroseconds(10);
  uint16_t v = analogRead(VS_A);
  digitalWrite(INHA, LOW);
  return 3.3 * v / (1<<ADC_RES_BITS) * (39.2e3 / 3.32e3) * 16.065/14.77;
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
