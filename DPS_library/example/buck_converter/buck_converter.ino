#include "DPS.h"

DPS dps(&Serial3);

int cycles = 5;
uint32_t prevTime;

void setup() {
  prevTime = millis();
  dps.set_on(true);
}

void loop() {
  if ((millis()-prevTime) > 1000){
    prevTime = millis();
    Serial.print("Setting voltage to ");
    Serial.print(cycles);
    Serial.print("V... ");
    dps.set_voltage(cycles);
    if (cycles > 9){
      cycles = 5;
    } else {
      cycles ++;
    }
  }
  if (dps.update()){
    Serial.println("successful");
  }
}
