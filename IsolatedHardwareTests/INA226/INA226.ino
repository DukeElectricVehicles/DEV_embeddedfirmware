#include "INA.h"

uint32_t lastLoopTime = 0;

char buffer[100];

void setup() {
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  INAinit();

  Serial.begin(115200);
  memset(buffer, 0, sizeof(buffer));

  Serial.println("INA226 test");

  lastLoopTime = millis();
}

void loop() {
  static uint32_t thisLoopTime = 0;

  thisLoopTime = millis();

  InaVoltage = INAvoltage();
  InaCurrent = INAcurrent();

  if ((thisLoopTime - lastLoopTime) > 100){
    Serial.print(InaVoltage,4);
    Serial.print(" ");
    Serial.print(InaCurrent,4);
    Serial.print(" ");
    Serial.print(InaPower,4);
    Serial.print("\n");
  }

  if (Serial.available()){
    uu
  }

}