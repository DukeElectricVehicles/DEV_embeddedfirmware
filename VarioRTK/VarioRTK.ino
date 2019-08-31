#include <SparkFun_Ublox_Arduino_Library.h>

#define LED1 7
#define LED2 8

#define SD_CS 6

#include <SD.h>

volatile uint32_t loopTime = 0;
uint32_t sdOk = 0;
uint32_t statusReg = 0;

File myFile;
SFE_UBLOX_GPS myGPS;


long latitude;
long longitude;
long altitude;
byte fixType;
byte RTK;
long speed;
long heading;


void setup() {  
  Serial.begin(115200);
  Serial1.begin(19200); //Ublox RTK
  SD.begin(SD_CS);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  myFile = SD.open("data.txt", FILE_WRITE);

  delay(1000);

  while(myGPS.begin(Serial1) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected. Fail."));
    delay(100);
  }

  //myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(10); //Set output to 10 times a second
}

void loop() {
  uint32_t curTime = millis();
  if(curTime < loopTime + 200)//if less than 100ms, start over
    return;
      
  digitalWrite(LED2, !digitalRead(LED2));
  loopTime = curTime;


  latitude = myGPS.getLatitude();
  longitude = myGPS.getLongitude();
  altitude = myGPS.getAltitude();
  fixType = myGPS.getFixType();
  RTK = myGPS.getCarrierSolutionType();
  speed = myGPS.getGroundSpeed();
  heading = myGPS.getHeading();

  /*Serial.print(millis() - curTime);
  Serial.println(" Read time");
  
  Serial.print(F("Lat: "));
  Serial.print(latitude);

  Serial.print(F(" Long: "));
  Serial.print(longitude);

  Serial.print(F(" Alt: "));
  Serial.print(altitude);

  Serial.print(F(" Fix: "));
  Serial.print(fixType);
  if(fixType == 0) Serial.print(F("No fix"));
  else if(fixType == 1) Serial.print(F("Dead reckoning"));
  else if(fixType == 2) Serial.print(F("2D"));
  else if(fixType == 3) Serial.print(F("3D"));
  else if(fixType == 4) Serial.print(F("GNSS+Dead reckoning"));

  Serial.print(" RTK: ");
  Serial.print(RTK);
  //if (RTK == 1) Serial.print(F("High precision float fix!"));
  //if (RTK == 2) Serial.print(F("High precision fix!"));

  Serial.println();*/
  
  writeToBtSd();
}

void writeToBtSd() {
  String outputStr = String(latitude) + " " + String(longitude) + " " + String(altitude) + " " + 
                     String(fixType) + " " + String(RTK) + " " + String(speed) + " " + 
                     String(heading) + " " + String(millis());
  
  Serial.println(outputStr);//usb
  
  int sdWritten = myFile.println(outputStr);
  myFile.flush();
  sdOk = sdWritten;
}
