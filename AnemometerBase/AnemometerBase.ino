#define LED1 7
#define LED2 8

#define WIND_TICK 23
#define WIND_ANGLE 22
#define SD_CS 6

#include <i2c_t3.h>
#include <SD.h>
//#include "DEVCAN.h"

#define WHEEL_CIRC 1.492
#define WHEEL_TICKS 16
#define TICK_DIST (WHEEL_CIRC / WHEEL_TICKS)

volatile uint32_t tickTimes[WHEEL_TICKS];
volatile uint32_t tickPos;

volatile uint32_t loopTime = 0;
volatile uint32_t lastHallPulse = 0;
volatile uint32_t lastInaMeasurement = 0;
volatile uint32_t countIntervals = 0;
volatile int32_t avgdT = 1000000;
volatile uint32_t distTicks = 0;

double distance = 0.0;
double currentSpeed = 0.0;
double windAngle = 0.0;
uint32_t sdOk = 0;
uint32_t statusReg = 0;

File myFile;

void setup() {  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);

  Serial.begin(115200);
  Serial1.begin(19200); //Ublox RTK
  SD.begin(SD_CS);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  pinMode(WIND_TICK, INPUT_PULLUP);
  attachInterrupt(WIND_TICK, countHallPulse, FALLING);

  myFile = SD.open("data.txt", FILE_WRITE);
}


void loop() {

  uint32_t curTime = millis();
  if(curTime < loopTime + 100)//if less than 100ms, start over
    return;

  digitalWrite(LED2, !digitalRead(LED2));
  loopTime = curTime;

  windAngle = analogRead(WIND_ANGLE);

  updateSpeed();

  readGPS();
  writeToBtSd();
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

void readGPS() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }
  Serial.println();
}

void writeToBtSd() {
  String outputStr = String(windAngle) + " " + String(currentSpeed) + " " + String(distance) + " " + String(statusReg) + " " + String(millis());
  
  Serial.println(outputStr);//usb
  
  int sdWritten = myFile.println(outputStr);
  myFile.flush();
  sdOk = sdWritten;
}
