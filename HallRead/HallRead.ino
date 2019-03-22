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


// For 2019 ESC
#if defined(__MK20DX256__) // teensy LC doesn't have interrupt on pin 1
  #define HALLA 8
  #define HALLB 5
  #define HALLC 2
  #warning hello
#else
  #define HALLA 0
  #define HALLB 1
  #define HALLC 2
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

void setup() {
  Serial.begin(115200);
  
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

  int out1 = digitalRead(HALL1);
  int out2 = digitalRead(HALL2);
  int out3 = digitalRead(HALL3);

  Serial.print(out3);
  Serial.print(out2);
  Serial.print(out1);
  Serial.print('\t');
  Serial.println((out3 << 2) | (out2 << 1) | (out1));

  delay(50);
}

void hallISR()
{
  
  int out1 = digitalRead(HALL1);
  int out2 = digitalRead(HALL2);
  int out3 = digitalRead(HALL3);

  Serial.print(out3);
  Serial.print(out2);
  Serial.print(out1);
  Serial.print('\t');
  Serial.println((out3 << 2) | (out2 << 1) | (out1));
}