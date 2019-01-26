#include "RF24.h"
#include <Metro.h>

RF24 radio(7,8);

#define PIN_RSW A9
#define PIN_RJY A8
#define PIN_RJX A7
#define PIN_LSW A6
#define PIN_LJY A5
#define PIN_LJX A4

uint8_t sendaddress[6] = "car00";
uint8_t recvaddress[6] = "contr";

Metro writeTimer(10);
uint8_t data;

bool writeRadio();
void readRadio();

int8_t commands[6];
int8_t zeroRJY, zeroRJX, zeroLJY, zeroLJX;

void setup() {
  Serial.begin(115200);
  Serial.println(F("DEV wireless controller"));
  Serial.print(F("Initializing..."));
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.openWritingPipe(sendaddress);
  radio.openReadingPipe(1, recvaddress);
  radio.printDetails();
  radio.startListening();

  pinMode(PIN_RSW, INPUT_PULLUP);
  pinMode(PIN_RJY, INPUT);
  pinMode(PIN_RJX, INPUT);
  pinMode(PIN_LSW, INPUT_PULLUP);
  pinMode(PIN_LJY, INPUT);
  pinMode(PIN_LJX, INPUT);
  delay(10);

  zeroRJY = analogRead(PIN_RJY)>>3;
  zeroRJX = analogRead(PIN_RJX)>>3;
  zeroLJY = analogRead(PIN_LJY)>>3;
  zeroLJX = analogRead(PIN_LJX)>>3;

  delay(10);

  Serial.println(F(" complete"));
}

void loop() {
  if (writeTimer.check()){
    writeRadio();
  }
  
  readRadio();
}

bool writeRadio() {

  commands[0] = digitalRead(PIN_RSW);
  commands[1] = (analogRead(PIN_RJY)>>3)-zeroRJY;
  commands[2] = (analogRead(PIN_RJX)>>3)-zeroRJX;
  commands[3] = digitalRead(PIN_LSW);
  commands[4] = (analogRead(PIN_LJY)>>3)-zeroLJY;
  commands[5] = (analogRead(PIN_LJX)>>3)-zeroLJX;

  radio.stopListening();
  bool success = radio.write(&commands, sizeof(commands));
  radio.startListening();
  return success;
}
void readRadio() {
  while (radio.available()) {
    radio.read(&data, sizeof(byte));
    Serial.print("Got ");
    Serial.println(data);
  }
}

