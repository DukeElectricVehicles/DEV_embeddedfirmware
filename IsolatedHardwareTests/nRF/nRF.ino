#include "RF24.h"
#include <Metro.h>

RF24 radio(7,8);

#define ROLE 0

byte address[6] = "car00";

Metro writeTimer(50);
byte data;

bool writeRadio(byte data);
void readRadio();

void setup() {
  Serial.begin(115200);
  Serial.println(F("DEV radio"));
  Serial.print(F("Initializing..."));
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.printDetails();
  radio.startListening();

  Serial.println(F(" complete"));
}

void loop() {
  #if ROLE==1
    if (writeTimer.check()){
      writeRadio(millis());
    }
  #endif
  
  readRadio();
}

bool writeRadio(byte data) {
  radio.stopListening();
  bool success = radio.write(&data, sizeof(byte));
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

