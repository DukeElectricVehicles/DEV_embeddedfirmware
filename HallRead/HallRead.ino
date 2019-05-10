/*

INSTRUCTIONS:

Upload this program to read the order of the hall sensors
To figure out the "hallOrder" array using this, see this example:

numbers printed to serial monitor in this order:
  1 3 2 6 4 5
to figure out hallOrder:
  value: X 1 3 2 6 4 5 X
  count: 0 1 2 3 4 5 6 7
so the 1st element should be 1, the 3rd element should be 2, the 2nd element should be 3, etc
in other words,
  hallOrder[value] = count
so,
  hallValue = [X, 1, 3, 2, 5, 6, 4, X]

figure out HALL_SHIFT by trial and error.

*/

#define ESC2019_SENSORLESS

#define AUTODETECT
#define PWMBODGE
#define NUMPOLES 22

#ifdef ESC2019_SENSORLESS
  #define HALLA 5
  #define HALLB 7
  #define HALLC 8
#elif defined(ESC2019)
  // For 2019 ESC
  #if defined(KINETISL) // teensy LC doesn't have interrupt on pin 1
    #define HALLA 8
    #define HALLB 5
    #define HALLC 2
    #warning hello
  #else
    #define HALLA 0
    #define HALLB 1
    #define HALLC 2
  #endif
#endif
#define HALL1 HALLA
#define HALL2 HALLB
#define HALL3 HALLC

//For 2017 ESC
/*#ifdef KINETISL // teensy LC doesn't have interrupt on pin 1
  #define HALL1 20
#else
  #define HALL1 1
#endif
#define HALL2 2
#define HALL3 3*/


//For 2016 ESC
/*#define HALL1 16
#define HALL2 17
#define HALL3 20*/

long last;
long buf[6];
int buf_index = 0;
volatile bool hallISRflag;
volatile uint8_t hallPos = 0;

#ifdef AUTODETECT
extern bool detectingHalls;
#endif
#ifdef AUTODETECT
  // 2019 ESC
  #ifdef ESC2019_SENSORLESS
    #define INHA 23
    #define INLA 22
    #define INHB 10
    #define INLB 9
    #define INHC 20
    #define INLC 6
  #elif defined(ESC2019)
    #define INHA 9
    #define INLA 6
    #define INHB 20 
    #define INLB 10
    #define INHC 23
    #define INLC 22
    #define DRV_EN_GATE 7
    #include "config_DRV.h"
    #include "autodetectHalls.h"
  #endif
#endif

void readSerial();

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);

  attachInterrupt(HALL1, hallISR, CHANGE);
  attachInterrupt(HALL2, hallISR, CHANGE);
  attachInterrupt(HALL3, hallISR, CHANGE);

  #ifdef AUTODETECT
    pinMode(INHA, OUTPUT);
    pinMode(INLA, OUTPUT);
    pinMode(INHB, OUTPUT);
    pinMode(INLB, OUTPUT);
    pinMode(INHC, OUTPUT);
    pinMode(INLC, OUTPUT);
    analogWriteFrequency(INHA, 8000);
    analogWriteFrequency(INHB, 8000);
    analogWriteFrequency(INHC, 8000);
    analogWriteResolution(12); // write from 0 to 2^12 = 4095

    #ifndef ESC2019_SENSORLESS
    setupDRV();
    #endif
    analogWrite(INHA, 0);
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
  #endif

  last = micros();
}

void loop() {
  
  readSerial();

  // int out1 = digitalRead(HALL1);
  // int out2 = digitalRead(HALL2);
  // int out3 = digitalRead(HALL3);
  
  // Serial.print(out3);
  // Serial.print("\t");
  // Serial.print(out2);
  // Serial.print("\t");
  // Serial.print(out1);
  // Serial.print("\t");
  // Serial.print(out3 << 2 | out2 << 1 | out1);
  // Serial.print("\n");

  // delay(10);

  /*int out1 = digitalRead(HALL1);
  int out2 = digitalRead(HALL2);
  int out3 = digitalRead(HALL3);

  Serial.print(out3);
  Serial.print(out2);
  Serial.print(out1);
  Serial.print('\t');
  Serial.println((out3 << 2) | (out2 << 1) | (out1));*/

  delay(50);
}

void hallISR()
{
  hallISRflag = true;
  
  int out1 = digitalRead(HALL1);
  int out2 = digitalRead(HALL2);
  int out3 = digitalRead(HALL3);

  hallPos = (out3 << 2) | (out2 << 1) | (out1);
  /*Serial.print(out3);
  Serial.print(out2);
  Serial.print(out1);
  Serial.print('\t');
  Serial.println((out3 << 2) | (out2 << 1) | (out1));*/

  #ifdef AUTODETECT
  if (detectingHalls)
    return;
  #endif
  
  buf[((out3 << 2) | (out2 << 1) | (out1)) - 1] = micros() - last;
  
  buf_index++;
  
  if (buf_index == 6) {
    for (int i = 0; i < 6; i++) {
      Serial.print(buf[i]);
      Serial.print('\t');
    }
    Serial.println();
    buf_index = 0;
  }

  last = micros();
}

void readSerial(){
  if (Serial.available()){
    uint8_t data = Serial.read();
    switch(data){
      case 'd':
        #ifdef AUTODETECT
          Serial.println("Testing position 1 at speeds of 1, 0.5, then 0.25");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(.5, 1000));
          printHallTransitions(runOpenLoop(-.5, 1000));
          printHallTransitions(runOpenLoop(.25, 1000));
          printHallTransitions(runOpenLoop(-.25, 1000));

          Serial.println("Advancing to position 2");
          printHallTransitions(runOpenLoop(5, 1000));

          Serial.println("Testing position 2 at speed of 1");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));

          Serial.println("Reversing to position 0");
          printHallTransitions(runOpenLoop(-5, 1000));
          printHallTransitions(runOpenLoop(-5, 1000));

          Serial.println("Testing position 0 at speed of 1");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));

          Serial.println("Advancing to position 1");
          printHallTransitions(runOpenLoop(5, 1000));

          Serial.println("Testing position 1 at speed of 1");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
        #endif
        break;
      case 'r':
        #ifdef AUTODETECT
          Serial.println("baseline");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));

          Serial.println("Testing full revolution forward");
          for (uint8_t i = 0; i<NUMPOLES; i++)
            printHallTransitions(runOpenLoop(3, 1000));

          Serial.println("Testing full revolution backward");
          for (uint8_t i = 0; i<NUMPOLES; i++)
            printHallTransitions(runOpenLoop(-3, 1000));

          Serial.println("recheck baseline");
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
          printHallTransitions(runOpenLoop(1, 1000));
          printHallTransitions(runOpenLoop(-1, 1000));
        #endif
        break;
      case 'c':
        #ifdef AUTODETECT
          double sumthetasForward[6];
          double sumthetasBackward[6];
          memset(sumthetasForward, 0, sizeof(sumthetasForward));
          memset(sumthetasBackward, 0, sizeof(sumthetasBackward));
          hallTransitions_t tmp;

          runOpenLoop(1,1000);
          for (uint8_t i = 0; i<NUMPOLES; i++){
            tmp = runOpenLoop(.5,1000);
            for (uint8_t j=0; j<6; j++){
              sumthetasForward[j] += tmp.positionTheta[j+1] / M_PI * 100;
              Serial.print(tmp.positionTheta[j+1] / M_PI * 100); Serial.print('\t');
            }
            Serial.println();
          }
          runOpenLoop(1,1000);
          runOpenLoop(-1,1000);
          for (uint8_t i = 0; i<NUMPOLES; i++){
            tmp = runOpenLoop(-.5,1000);
            for (uint8_t j=0; j<6; j++){
              sumthetasBackward[j] += tmp.positionTheta[j+1] / M_PI * 100;
              Serial.print(tmp.positionTheta[j+1] / M_PI * 100); Serial.print('\t');
            }
            Serial.println();
          }

          for (uint8_t i=0; i<6; i++){
            sumthetasForward[i] /= NUMPOLES;
            sumthetasBackward[i] /= NUMPOLES;
            Serial.print(sumthetasForward[i]); Serial.print('\t');
            Serial.print(sumthetasBackward[i]); Serial.print('\t');
            Serial.print(avgMod2pi(sumthetasForward[i],sumthetasBackward[i])); Serial.print('\n');
          }

        #endif
        break;
    }
  }
}

float avgMod2pi(float a1, float a2){
    float diffHall = a1 - a2;
    float hallAngle;
    if (fabsf(diffHall) < 100){
      hallAngle = (a1 + a2) / 2;
    } else {
      hallAngle = (a1 + a2) / 2 + 100;
    }
    hallAngle = fmodf(hallAngle + 200, 200);
    return hallAngle;
}
