#include <Metro.h>
#include <RF24.h>
#include "MPU6050_tockn_DEV.h"
#include <Wire.h>

#define SENDERx
#define RECEIVER

byte address[6] = "DEV:)";
typedef struct packet
{
  uint32_t time_ms;
  float AngleX, AngleY, AngleZ;
  float AccX, AccY, AccZ;
};

#ifdef SENDER
  MPU6050 mpu6050(Wire);
  RF24 radio(7,8);

  packet toSend;
  
  void setup() {
    Serial.begin(115200);
    Serial.println("DEV Pendulum Data collection rig");
    Serial.println("Attach this to the pendulum");
    Serial.println("Setting up...");
    delay(1000);
    digitalWrite(13, LOW);
    
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);
    radio.printDetails();
    radio.startListening();
    radio.stopListening();

    Serial.println("Setup complete");
    digitalWrite(13, HIGH);
  }
  
  void loop() {
    mpu6050.update();
    toSend.time_ms = millis();
    toSend.AngleX = mpu6050.getAngleX();
    toSend.AngleY = mpu6050.getAngleY();
    toSend.AngleZ = mpu6050.getAngleZ();
    toSend.AccX = mpu6050.getAccX();
    toSend.AccY = mpu6050.getAccY();
    toSend.AccZ = mpu6050.getAccZ();
    radio.write(&toSend, sizeof(packet));
  }
  
#elif defined RECEIVER
  RF24 radio(7,8);
  
  void readRadio();
  void printData(packet data);

  packet data;
  
  void setup() {
    Serial.begin(115200);
    Serial.println("DEV Pendulum Data collection rig");
    Serial.println("Keep this connected to computer");
    Serial.println("Setting up...");
    
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);
    radio.printDetails();
    radio.startListening();

    Serial.println("Setup complete");
  }
  
  void loop() {
    readRadio();
  }
  
  void readRadio() {
    while (radio.available()) {
      radio.read(&data, sizeof(packet));
      printData(data);
    }
  }

  #define PLOTTERMODEx
  void printData(packet data) {
    #ifdef PLOTTERMODE
      Serial.print(50); Serial.print('\t'); // faux y-axes
      Serial.print(-50); Serial.print('\t');
    #else
      Serial.print(data.time_ms); Serial.print('\t');
    #endif
    Serial.print(data.AngleX);  Serial.print('\t');
    Serial.print(data.AngleY);  Serial.print('\t');
    Serial.print(data.AngleZ);  Serial.print('\t');
    Serial.print(data.AccX);    Serial.print('\t');
    Serial.print(data.AccY);    Serial.print('\t');
    Serial.print(data.AccZ);    Serial.print('\n');
  }
  
#else
  #error
#endif
