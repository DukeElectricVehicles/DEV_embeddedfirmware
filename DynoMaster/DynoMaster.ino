#define LED1 7
#define LED2 8

#define RELAY 2
#define HALL 23
#define MA_WINDOW 1
#define SPROCKET_TICKS 54
#define MOTOR_TICKS 72

#define PRINTEVERYTICK

#define INA_ID 2
#include <i2c_t3.h>
#include "INA.h"

volatile uint32_t tickTimes[MA_WINDOW];
volatile uint32_t tickPos;

volatile uint32_t loopTicker = 0;
volatile uint32_t lastHallPulse = 0;
volatile int32_t avgdT = 1000000;
volatile uint32_t distTicks = 0;

uint32_t testBeginTime = 0;
uint32_t testActive = 0;

float currentRPM = 0;
float targetThrottle = 0;
float targetCurrent = 4;
float integralTerm = 0;


void setup() {
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(RELAY, OUTPUT);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  digitalWrite(RELAY, HIGH);

  pinMode(HALL, INPUT_PULLUP);

  analogWriteResolution(12);

  attachInterrupt(HALL, countHallPulse, FALLING);
}

void loop() {

  //Main loop delay-----------------------------------------
  uint32_t currentMillis = millis();
  updateINA();
  if(currentMillis - loopTicker < 10)
    return;

  loopTicker = currentMillis;

  //Read sensors--------------------------------------------
  currentRPM = 1000000.0 / avgdT * 60 / SPROCKET_TICKS;
  if(micros() - lastHallPulse > 2000000)
    currentRPM = 0;

  //Read serial port----------------------------------------
  while(Serial.available())
  {
    uint8_t c = Serial.read();

    if(c == 'b')
    {
      testBeginTime = currentMillis;
      testActive = 1;
      initTest();
    }
    
    if(c == 's')
      testActive = 0;

    if(c >= 'A' && c <='Z')//the jankiest shit ever. use letters as current targets
      targetCurrent = c - 'A' + 1;
  }

  //Begin state machine------------------------------------

  if(currentMillis - testBeginTime > 120000)
    testActive = 0;
    
  if(testActive)
    runTest(currentMillis - testBeginTime);
  else
    writeThrottle(0);

  #ifndef PRINTEVERYTICK
    printData(currentMillis);
  #endif
}

void printData(uint32_t currentMillis) {

  //Print---------------------------------------------------
  currentRPM = 1000000.0 / avgdT * 60 / SPROCKET_TICKS;
  float velo = currentRPM / 9.5492;//to rad/sec
  float flywheelEnergy = 0.5 * velo * velo * 0.8489;
  
  Serial.print(InaVoltage_V);
  Serial.print(' ');
  Serial.print(InaCurrent_A);
  Serial.print(' ');
  Serial.print(InaPower_W);
  Serial.print(' ');
  Serial.print(currentRPM);
  Serial.print(' ');
  Serial.print(targetCurrent);
  Serial.print(' ');
  Serial.print(currentMillis);
  Serial.print(' ');
  Serial.print(targetThrottle);
  Serial.print(' ');
  Serial.print(flywheelEnergy);
  Serial.print(' ');
  Serial.print(InaEnergy_J);
  Serial.println();
}

void initTest()
{
  integralTerm = 0;
}

void runTest(uint32_t msElapsed)
{
  float errorCurrent = targetCurrent - InaCurrent_A;
  integralTerm += errorCurrent * 0.001;
  
  if(currentRPM < 100)    integralTerm = 0;

  //float targetThrottle = 0.47 + currentRPM * 0.00065;//5A at 20v
  targetThrottle = 0.47 + integralTerm;//5A at 20v
  
  //targetThrottle = 0.51 + currentRPM * 0.00068;//10A at 20v

  //Serial.println(integralTerm);
  writeThrottle(targetThrottle);
}

void writeThrottle(float pct)
{
  pct = constrain(pct, 0.0, 1.0);
  uint16_t dacVal = pct * 4095.0;
  analogWrite(A14, dacVal);

  digitalWrite(LED2, dacVal > 0);
}

void countHallPulse() {
  uint32_t currentMicros = micros();
  uint32_t oldTime = tickTimes[tickPos];
  
  tickTimes[tickPos++] = currentMicros;
  tickPos %= MA_WINDOW;

  avgdT = (currentMicros - oldTime) / MA_WINDOW;

  distTicks++;
  
  lastHallPulse = currentMicros;

  #ifdef PRINTEVERYTICK
    printData(currentMicros/1e3);
  #endif

  digitalWrite(LED1, (distTicks) & 1);
}
