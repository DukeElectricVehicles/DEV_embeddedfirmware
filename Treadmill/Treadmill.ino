// THIS CODE IS PLACEHOLDER CODE - Quinn has more up to date code but hasn't PR'd yet

#include <Metro.h>

Metro printTimer(10);

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (printTimer.check()){
    Serial.print(millis());
    Serial.print('\t');
    Serial.print(analogRead(A0));
    Serial.print('\n');
  }
}
