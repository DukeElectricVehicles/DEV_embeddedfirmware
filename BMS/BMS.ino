#include <i2c_t3.h>
#include <SD.h>
#include "Adafruit_GPS.h"
#include "INA.h"
#include "H2.h"
#include "ms5611.h"
#include "DPS.h"
#include "Metro.h"

#define USE_GPSx
#define USE_BTx
#define USE_SD
#define USE_BAROMETERx
#define USE_DPSBUCKx
#define USE_WDOG
#define USE_H2x

#define LED1 7
#define LED2 8

#define S0 15
#define S1 16
#define S2 17
#define S3 20

#define RELAY 2
// #define SOLENOID 7
#define HALL 23
#define SD_CS 6
#define TEMP 22

#define WHEEL_CIRC 1.492
#define WHEEL_TICKS 16
#define TICK_DIST (WHEEL_CIRC / WHEEL_TICKS)

#define TARGET_CURRENT 5
#define UV_BAT 10

volatile uint32_t tickTimes[WHEEL_TICKS];
volatile uint32_t tickPos;

volatile uint32_t loopTime = 0;
volatile uint32_t lastHallPulse = 0;
volatile uint32_t lastInaMeasurement = 0;
volatile uint32_t countIntervals = 0;
volatile int32_t avgdT = 1000000;
volatile uint32_t distTicks = 0;

uint32_t shortTime = 0;

double energyUsed = 0.0;
double distance = 0.0;
double currentSpeed = 0.0;
double temperature = 0.0;
double InaVoltage = 0.0;
double InaCurrent = 0.0;
double InaPower = 0;
double batteryVoltage = 0.0;
double startingAlt = 0;
double currentAlt = 0;
double throttle = 0;
double FCV = 0;
double FCI = 0;
double FCE = 0;
double FCTemp = 0;
double H2Press = 0;
double H2Flow = 0;
double H2Tot = 0;
double H2Eff = 0;
double H2AvgEff = 0;

bool batteryOK = true;
uint8_t powerSaveVote = 0;
uint32_t h2Detected = 0;

Metro UndervoltageTimeout(15000);
bool UVlatch = false;

File myFile;
#ifdef USE_GPS
Adafruit_GPS GPS(&Serial1);
#endif
//MS5611 baro;
#ifdef USE_DPSBUCK
DPS dps(&Serial3);
#endif

void setup() {
  setupWatchdog();

  Serial.println("BEGINNING");
  
  kickDog();

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  // Wire.setDefaultTimeout(100);//this makes i2c not work?
  INAinit();
  Serial.print("setup INA...");

  kickDog();
  Serial.begin(115200);
  Serial.print(" Serial...");
  kickDog();
  #ifdef USE_BT
  Serial2.begin(115200);
  Serial.print(" bluetooth...");
  #endif
  kickDog();
  SD.begin(SD_CS);
  Serial.print(" SD...");

  kickDog();

  // pinMode(LED1, OUTPUT);
  // pinMode(LED2, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(RELAY, OUTPUT);
  // pinMode(SOLENOID, OUTPUT);
  pinMode(TEMP, INPUT);
  digitalWrite(RELAY, LOW);
  // digitalWrite(SOLENOID, HIGH);

  // digitalWrite(LED1, HIGH);
  // digitalWrite(LED2, HIGH);

  pinMode(HALL, INPUT_PULLUP);
  attachInterrupt(HALL, countHallPulse, FALLING);

  #ifdef USE_SD
  myFile = SD.open("data.txt", FILE_WRITE);
  Serial.print(" SD (part 2)...");
  #endif

  kickDog();

  GPSInit();
  Serial.print(" GPS...");

  #ifdef USE_DPSBUCK
  dps.set_on(true);
  Serial.print(" DPS...");
  #endif

  kickDog();

  UndervoltageTimeout.reset();

  Serial.println("FINISHED SETUP");
}


void loop() {

  GPSPoll();//must be called rapidly
  
  uint32_t curTime = millis();
  kickDog();//reset watchdog. sometimes i2c causes the processor to hang
  if(curTime < loopTime + 100)//if less than 100ms, start over
    return;

  // digitalWrite(LED2, !digitalRead(LED2));
  loopTime = curTime;

  updateINA();
  updateSpeed();
  // pollH2();
  
  uint8_t btn = readBtn();
  updateThrottle(btn);
  // updateH2Btn(btn);
  #ifdef USE_DPSBUCK
  dps.update();
  GPSPoll(); // may be needed because I haven't tested how slow the dps.update is
  #endif

  writeToBtSd();

  if (InaVoltage > UV_BAT) {
    UndervoltageTimeout.reset();
    UVlatch = false;
  }
  if (UndervoltageTimeout.check()){
    UVlatch = true;
  }
  if((InaCurrent > 20) || UVlatch) //|| powerSaveVote == 0)
  {
    digitalWrite(RELAY, LOW);
    // shortTime = curTime;
  }
  else {//if(curTime - shortTime > 2000 && powerSaveVote > 0)
    digitalWrite(RELAY, HIGH);
  }
}

void pollH2()
{
  #ifdef USE_H2
  FCV = readH2(I2C_READ_FCV) / 1000.0;
  FCI = readH2(I2C_READ_FCI) / 1000.0;
  FCE = readH2(I2C_READ_FCE) / 1000.0;
  FCTemp = readH2(I2C_READ_FCTEMP) / 1000.0;
  H2Press = readH2(I2C_READ_H2PRESS) / 1000.0;
  H2Flow = readH2(I2C_READ_H2FLOW) / 1000.0;
  H2Tot = readH2(I2C_READ_H2TOT) / 10000.0;//10 thousand
  H2AvgEff = readH2(I2C_READ_H2AVGEFF) / 1000.0;
  
  H2Eff = FCV * FCI / mgtoJ(H2Flow);
  #endif
}

void updateThrottle(uint8_t btn)
{
  static uint32_t btnDuration = 0;
  static uint32_t btnSelected = 0;

  if(btn != btnSelected)
  {
    btnSelected = btn;
    btnDuration = 0; 
  }
  else
    btnDuration++;

  #ifdef USE_DPSBUCK
  if (btnDuration == 5){
    switch(btnSelected) {
      case 1:
        dps.set_voltage(8);
        break;
      case 2:
        dps.set_voltage(12);
        break;
      case 3:
        dps.set_voltage(16);
        break;
      case 4:
        dps.set_voltage(20);
        break;
      case 5:
        dps.set_voltage(24);
        break;
    }
  }
  #endif

  // //Write over I2C
  // uint16_t rawThrottle = throttle * 65535;
  // Wire.beginTransmission(0x66);
  // Wire.write(0x40);//throttle register
  // Wire.write((rawThrottle >> 8) & 0xFF);
  // Wire.write(rawThrottle & 0xFF);
  // Wire.endTransmission();
}

void updateH2Btn(uint8_t btn)
{
  static uint32_t btnDuration = 0;
  static uint32_t btnSelected = 0;

  if(btn != btnSelected)
  {
    btnSelected = btn;
    btnDuration = 0; 
  }
  else
    btnDuration++;

  if(btnSelected == 1 && btnDuration == 5)
    writeH2(I2C_WRITE_PURGE, 0);

  if(btnSelected == 2 && btnDuration == 5)
    writeH2(I2C_WRITE_SHORT, 0);

  if(btnSelected == 3 && btnDuration == 5)
    writeH2(I2C_WRITE_LOADSHORT, 0);

  if(btnSelected == 4 && btnDuration == 5)
    writeH2(I2C_WRITE_TIMESHORT, 0);


  float clampedCurrent = InaCurrent * 10.0;

  if(clampedCurrent > 200)
    clampedCurrent = 200;

  if(clampedCurrent < 0)
    clampedCurrent = 0;
    
  writeH2(I2C_WRITE_REPORTCURRENT, (uint8_t)clampedCurrent);
}

uint8_t readBtn()
{
  uint16_t btnAnalog = analogRead(TEMP);
  //Serial.println(btnAnalog);

  uint8_t btn = 0;
  if(btnAnalog < 10)  btn = 1;
  if(btnAnalog < 320 && btnAnalog > 300)  btn = 2;
  if(btnAnalog < 145 && btnAnalog > 125)  btn = 3;
  if(btnAnalog < 495 && btnAnalog > 470)  btn = 4;
  if(btnAnalog < 735 && btnAnalog > 710)  btn = 5;
  
  return btn;
}

void updateINA()
{
  InaVoltage = INAvoltage() * 24.67/24.62;
  InaCurrent = INAcurrent() * .999/.989;
  InaPower = InaVoltage * InaCurrent;
  
  uint32_t currentInaTime = micros();
  energyUsed += InaPower * (currentInaTime - lastInaMeasurement) / 1000000.0;
  lastInaMeasurement = currentInaTime;  
}

void updateSpeed()
{
  currentSpeed = 1000000.0 / avgdT * WHEEL_CIRC; 
  if(micros() - lastHallPulse > 2000000)
    currentSpeed = 0;
  
  distance = distTicks * TICK_DIST;
}

void countHallPulse() {
  uint32_t current = micros();

  uint32_t prevTime = tickTimes[tickPos];// time 1 rev ago. fixed 7/5/2018 :(

  tickTimes[tickPos++] = current;
  tickPos %= WHEEL_TICKS;

  avgdT = current - prevTime;

  distTicks++;
  
  lastHallPulse = current;

  // digitalWrite(LED1, (distTicks) & 1);
}

void GPSInit()
{
  #ifdef USE_GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_57600);
  delay(500);
  Serial1.end();
  kickDog();
  delay(500);
  GPS.begin(57600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
  #endif
}

void GPSPoll()
{
  #ifdef USE_GPS
    while(GPS.read());
    
    if (GPS.newNMEAreceived())
      GPS.parse(GPS.lastNMEA());
  #endif
}

void writeToBtSd() {
  //uint32_t startMicros = micros();
  
  // TODO: on Teensy LC, this stops working when the string is too many characters long due to idk why dynamic memory or something
  String outputStr = String(InaVoltage, 3) + " " + String(InaCurrent, 3) + " " + String(InaPower) + " "+ String(currentSpeed) + " " +
                     String(energyUsed) + " " + String(distance) + " " +
                     #ifdef USE_H2
                      String(FCV, 3) + " " + String(FCI, 3) + " "+ String(FCE, 1) +" " + 
                     #else
                      "0 0 0 " +
                     #endif
                      String(millis()) + " " + 
                     #ifdef USE_GPS
                      String(GPS.latitudeDegrees, 7) + " " + String(GPS.longitudeDegrees, 7) + " " +
                     #else
                      "0 0 " + 
                     #endif
                     #ifdef USE_H2
                      String(FCTemp, 1) + " " + String(H2Press, 1) + " " + String(H2Flow, 3) + " " + String(H2Tot, 4) + " " + String(H2Eff, 4) + " " + String(H2AvgEff, 4)
                     #else
                      "0 0 0 0 0 0"
                     #endif
                     ;
  
  Serial.println(outputStr);//usb  
  GPSPoll();//super hacky bc short GPS buffer
  #ifdef USE_BT
  Serial2.println(outputStr);//bluetooth
  #endif
  GPSPoll();
  myFile.println(outputStr);
  myFile.flush();

  //Serial.println(micros() - startMicros);
}

#ifdef USE_WDOG
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
      // SIM_COPC = 8; // 256ms watchdog
      // SIM_COPC = 4; // 32ms watchdog
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