#include "DPS.h"

DPS dps(&Serial3);

uint32_t randomTimer;

void setup() {

	Serial.println("beginning");
	dps.set_on(true);
	randomTimer = millis();

}

void loop() {

	dps.update();

	dps.set_voltageCurrent((millis() - randomTimer) / 1000, 10 - (millis() - randomTimer) / 1000);

	if ((millis() - randomTimer) > 10000) {
		Serial.println("resetting");
		randomTimer = millis();
	}
}