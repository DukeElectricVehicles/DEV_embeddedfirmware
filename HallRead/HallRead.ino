//For 2017 ESC
#ifdef KINETISL // teensy LC doesn't have interrupt on pin 1
  #define HALL1 20
#else
  #define HALL1 1
#endif
#define HALL2 2
#define HALL3 3


//For 2016 ESC
/*#define HALL1 16
#define HALL2 17
#define HALL3 20*/

void setup() {
  Serial.begin(9600);
  
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);

  attachInterrupt(HALL1, hallISR, CHANGE);
  attachInterrupt(HALL2, hallISR, CHANGE);
  attachInterrupt(HALL3, hallISR, CHANGE);
}

void loop() {
  
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
}

void hallISR()
{
  
  int out1 = digitalRead(HALL1);
  int out2 = digitalRead(HALL2);
  int out3 = digitalRead(HALL3);

  Serial.println((out3 << 2) | (out2 << 1) | (out1));
}