#ifndef ANALOGBUTTONMATRIX_H
#define ANALOGBUTTONMATRIX_H

uint8_t curButton = 0;
void (*btnPressCallback)(uint8_t);

uint8_t readBtn();

void updateBtn() {
	static uint32_t debounceTime = 0;
	static uint8_t btnUnconfirmed = 0;

	uint8_t tmpBtn = readBtn();

	if (btnUnconfirmed != tmpBtn) {
		btnUnconfirmed = tmpBtn;
		debounceTime = millis();
	}

	if ((millis() - debounceTime) > 10) {
		if (curButton!=tmpBtn){
			btnPressCallback(tmpBtn);
		}
		curButton = tmpBtn;
	}
}

void setBtnCallback(void (*callback)(uint8_t)) {
	btnPressCallback = callback;
}

uint8_t getBtn() {
	return curButton;
}

uint8_t readBtn()
{
  uint16_t btnAnalog = analogRead(TEMP);
  //Serial.println(btnAnalog);

  uint8_t btn = 0;
  if(btnAnalog < 10)  btn = 1;
  if(btnAnalog < 320 && btnAnalog > 300)  btn = 2;
  if(btnAnalog < 145 && btnAnalog > 125)  btn = 3;
  if(btnAnalog < 495 && btnAnalog > 470)  btn = 4;
  if(btnAnalog < 735 && btnAnalog > 710)  btn = 5;
  
  return btn;
}

#endif