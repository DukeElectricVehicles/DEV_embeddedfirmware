#define LED1 -1
#define LED2 -1
// #define LED1 3
// #define LED2 21

#define RELAY 2
#define HALL 23
#define SD_CS 6
#define TEMP 22
#define BMS

#define INA_ID 3
#define CAN_MODE_PIN 5

#include <i2c_t3.h>
#include <SD.h>
#include "Adafruit_GPS.h"
#include "INA.h"
#include "DPS.h"
#include "analogButtonMatrix.h"
#include "DEVCAN.h"

// #define WHEEL_CIRC 1.492
// #define WHEEL_TICKS 16
#define WHEEL_CIRC (1.492 * 1.034328525)
#define WHEEL_TICKS 8
#define TICK_DIST (WHEEL_CIRC / WHEEL_TICKS)

volatile uint32_t tickTimes[WHEEL_TICKS];
volatile uint32_t tickPos;

volatile uint32_t loopTime = 0;
volatile uint32_t lastHallPulse = 0;
volatile uint32_t lastInaMeasurement = 0;
volatile uint32_t countIntervals = 0;
volatile int32_t avgdT = 1000000;
volatile uint32_t distTicks = 0;

uint32_t shortTime = 0;

float setpointV = 13;
float setpointI = 7.5;

double energyUsed = 0.0;
double distance = 0.0;
double currentSpeed = 0.0;
double temperature = 0.0;
double batteryVoltage = 0.0;
double startingAlt = 0;
double currentAlt = 0;
double throttle = 0;
uint32_t sdOk = 0;
uint32_t statusReg = 0;

File myFile;
Adafruit_GPS GPS(&Serial1);
DPS dps(&Serial3);

void setup() {
  setupWatchdog();

  // for (uint8_t i = 0; i<30; i++) {
  //   dps.set_on(true);
  //   delay(100);
  //   dps.update();
  //   kickDog();
  // }
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.setDefaultTimeout(100);//this makes i2c not work?
  INAinit();

  setBtnCallback(&updateSetpointCurrent);
  setupCAN();
  sendSetpointCurrentCAN(setpointI);

  Serial.begin(115200);
  //Serial2.begin(115200);
  Serial2.begin(38400); // Bluetooth
  SD.begin(SD_CS);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(RELAY, OUTPUT);
  pinMode(TEMP, INPUT);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  pinMode(HALL, INPUT_PULLUP);
  attachInterrupt(HALL, countHallPulse, FALLING);

  myFile = SD.open("data.txt", FILE_WRITE);

  kickDog();
  delay(100); // allow voltage to stabilize before providing power to motor controller
  digitalWrite(RELAY, HIGH);
  kickDog();

  GPSInit();

  kickDog();
}


void loop() {  
  GPSPoll();//must be called rapidly
  
  dps.update();
  updateBtn();
  parseCAN();

  uint32_t curTime = millis();
  if(curTime < loopTime + 100)//if less than 100ms, start over
    return;

  kickDog();//reset watchdog. sometimes i2c causes the processor to hang
  digitalWrite(LED2, !digitalRead(LED2));
  loopTime = curTime;

  updateINA();
  updateSpeed();
  // dps.set_voltageCurrent(setpointV, setpointI);
  sendSetpointCurrentCAN(setpointI);
  statusReg = (sdOk << 1) + (uint32_t)GPS.fix;

  writeToBtSd();
}

void updateSetpointCurrent(uint8_t btn) { // callback
  Serial.print("BTN PRESS ");
  Serial.println(btn);
  switch (btn) {
    case 1:
      setpointI -= 0.5;
      // dps.set_voltageCurrent(setpointV, setpointI);
      sendSetpointCurrentCAN(setpointI);
      break;
    case 2:
      // setpointI = 0;
      break;
    case 3:
      // dps.set_voltageCurrent(setpointV, setpointI);
      break;
    case 4:
      setpointI += 0.5;
      // dps.set_voltageCurrent(setpointV, setpointI);
      sendSetpointCurrentCAN(setpointI);
      break;
    case 5:
      // digitalWrite(RELAY, !digitalRead(RELAY));
      // dps.set_on(!dps.get_on());
      break;
  }
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

  digitalWrite(LED1, (distTicks) & 1);
}

void GPSInit()
{
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_57600);
  delay(500);
  Serial1.end();
  
  delay(500);
  GPS.begin(57600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
}

void GPSPoll()
{
  while(GPS.read());
  
  if (GPS.newNMEAreceived())
  {
    //Serial.println("GPS RX");
    GPS.parse(GPS.lastNMEA());
  }
}

void writeToBtSd() {
  //uint32_t startMicros = micros();
  
  String outputStr = String(InaVoltage_V, 3) + " " + String(InaCurrent_A, 3) + " " + String(InaPower_W) + " "+ String(currentSpeed) + " " +
                     String(InaEnergy_J) + " " + String(distance) + " " + String(setpointV,2) + " " + 
                     String(setpointI,2) + " "+ String(statusReg) + " " + String(millis()) + " " + String(GPS.latitudeDegrees, 7) + 
                     " " + String(GPS.longitudeDegrees, 7);
  
  
  Serial.println(outputStr);//usb  
  GPSPoll();//super hacky bc short GPS buffer
  Serial2.println(outputStr);//bluetooth
  GPSPoll();
  int sdWritten = myFile.println(outputStr);
  myFile.flush();
  sdOk = sdWritten > 10;//if we havent written any bytes to SD card, assume fault

  //Serial.println(micros() - startMicros);
  //Serial.println(distTicks);
}

void kickDog()
{
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

void setupWatchdog()
{
  kickDog();
  
  noInterrupts();                                         // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);                                   // Need to wait a bit..
  
  // about 2-3 second timeout
  WDOG_TOVALH = 0x0550;
  WDOG_TOVALL = 0x0000;
  
  // This sets prescale clock so that the watchdog timer ticks at 7.2MHz
  WDOG_PRESC  = 0x400;
  
  // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
      WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
      WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();

  kickDog();
}

