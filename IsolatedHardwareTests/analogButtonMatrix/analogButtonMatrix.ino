#define TEMP 22

#include "analogButtonMatrix.h"

uint32_t printtimer;

void setup() {
	printtimer = millis();
	setBtnCallback(&btnDown);
}

void loop() {
	updateBtn();

	if ((millis() - printtimer) > 100) {
		Serial.println(getBtn());
		printtimer += 100;
	}
}

void btnDown(uint8_t btn) {
	Serial.print("Pressed ");
	Serial.print(btn);
	Serial.println("!");
}