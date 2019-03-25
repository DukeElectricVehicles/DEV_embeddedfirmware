#include <DPS.h>

bool transmitting = false;

int voltage = 100;
bool voltage_changed = true;

long transmit_time = millis();

DPS dps;

int cycles = 0;

void setup() {
  
}

void loop() {
  if (millis() - transmit_time > 500 || Serial1.available() >= 8) {
    
    while(Serial1.available()) {
      Serial1.read();
    }
    

    transmitting = false;
  }

  if (!transmitting && voltage_changed) {
    dps.set_voltage(max(5, voltage / 10.0));
    transmitting = true;
    transmit_time = millis();
  }

  voltage = (voltage + 5) % 150;
  voltage_changed = true;

}
