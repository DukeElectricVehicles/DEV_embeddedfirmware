#define useCAN
#define useI2Cx
#define useINA226x
#define useINA332
#define useHallSpeedx
#define useWatchdogx
#define COMPLEMENTARYPWMx
#define OC_LIMIT 1.0 // current limit
#define useTRIGDELAYCOMPENSATION
#define BUCK

#include "config.h"
#include "MCpwm_2019sensorless.h"
#include "ADC_2019sensorless.h"
#include "Metro.h"
#include "infinityPV_INA233.h"
#include "DEVCAN.h"

#define LED 13

INA233 Ina(0x40);
Metro printTimer(100);
Metro controlTimer(5);

float INA_V = 0;
float INA_I = 0;
float INA_E = 0;
float Vout = 0;
float Iout = 0;

float D = 0;
float Isetpoint = 7.5;
float kI = 3; // 30;

void setup() {
  setupWatchdog();
  setupPWM();
  setupPins();
	setupADC();
  kickDog();
  #ifdef useCAN
    setupCAN();
  #endif
  #ifdef useINA226
    INAinit();
  #endif

  kickDog();

  Serial.println("test");

  pinMode(INA_ALERT, INPUT_PULLUP);
  // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  // Wire.setDefaultTimeout(50000);
  Ina.begin();
  Ina.wireWriteByte(MFR_ALERT_MASK, 0x7F); //only turn on conversion complete ALERT
  Ina.wireSendCmd(CLEAR_FAULTS);
  // printHelp();
}

void loop() {
  calcIVout();
  parseCAN();
  /*if (Vout < 8.5) {
    D += 0.05;
    writeDC(D*MODULO);
  } else */if (controlTimer.check()) {
    calcD();
    D = constrain(D, 0.5, 0.99);
    writeDC(D*MODULO);
  }
	
  if (printTimer.check()) {
    printDebug();
    readSerial();
  }

  readIna();
}

void calcIVout() {
  // if (vsx_cnts[0] < 1100) {
    Vout = vsx_cnts[1] * (3.0/(1<<ADC_RES_BITS)) / (3.3/(39.2+3.3));
  // } else {
  //   Vout = INA_V / vsx_cnts[0] * vsx_cnts[1];
  // }
  if (Vout == 0) {
    Iout = 0;
  } else {
    Iout = INA_V / Vout * INA_I;
  }
}

void calcD() {
  static uint32_t prevUpdateTime_us = micros();
  uint32_t curTime = micros();
  D += (Isetpoint - Iout) * kI * (curTime - prevUpdateTime_us) / 1000000;
  prevUpdateTime_us = curTime;
}

void readIna() {
  static uint32_t INA_Poll_Count = 0;
  static uint32_t INA_Poll_Time = micros();
  // if(digitalRead(INA_ALERT) == 0)
  // {
    INA_Poll_Count++;
    Ina.wireSendCmd(CLEAR_FAULTS); //clear ALERT pin
    if(INA_Poll_Count % 2 == 0) //only take every other cycle, as both I and V conversions trigger ALERT
    {
      uint32_t curTime = micros();
      uint32_t dt = curTime - INA_Poll_Time;
      INA_V = Ina.getBusVoltage_raw() / 800.0;
      INA_I = Ina.getShuntVoltage_raw() * (2.5e-6 / 1e-3) * 0.6804245283; // fudge factor
      INA_E += INA_V * INA_I * dt / 1e6;  
    }    
  // }
}

void printDebug() {
  Serial.print(D);
  Serial.print('\t');
  Serial.print(vsx_cnts[0]);
  Serial.print('\t');
  Serial.print(vsx_cnts[1]);
  Serial.print('\t');
  Serial.print(vsx_cnts[0] * (3.0/(1<<ADC_RES_BITS)) / (3.3/(39.2+3.3)));
  Serial.print('\t');
  Serial.print(vsx_cnts[1] * (3.0/(1<<ADC_RES_BITS)) / (3.3/(39.2+3.3)));
  Serial.print('\t');
  Serial.print(INA_V);
  Serial.print('\t');
  Serial.print(INA_I);
  Serial.print('\t');
  Serial.print(Vout);
  Serial.print('\t');
  Serial.print(Iout);
  Serial.print('\t');
  Serial.print(Isetpoint);
  Serial.print('\n');
}

void readSerial() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'A':
        Isetpoint = Serial.parseFloat();
        break;
      case 'k':
        kI = Serial.parseFloat();
        break;
    }
  }
}