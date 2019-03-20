#include <Metro.h>
#include <FlexCAN.h>
#include "RF24.h"
#include "vESC_datatypes.h"
#include "ESCHall2017Simple.h"
#include "config_ESC.h"

#define useNRFx

#define NSIL 3
#define STANDBY 4

#define RmotorCAN 55
#define LmotorCAN 56
#define MmotorCAN 100 // main motor (throttle)

typedef struct {
  int32_t LSteeringMotor;
  int32_t RSteeringMotor;
  uint16_t throttle;
} driveCommands_t;

#ifdef useNRF
  RF24 radio(7,8);
  uint8_t recvaddress[6] = "car00";
  uint8_t sendaddress[6] = "contr";
  int8_t readBuffer[6];
  driveCommands_t mostRecentCommands;
#else 
  uint16_t throttleCAN;
#endif

FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;

int txCount,rxCount;
unsigned int txTimer,rxTimer;

Metro sysTimer = Metro(1);// milliseconds
Metro CANtimer = Metro(10);
Metro commandTimeout = Metro(100);
Metro ESCupdateTimer = Metro(50);
Metro radioReadTimer = Metro(20);

#ifdef useNRF
  void readRadio(void);
  void sendSteering(void);
  void sendThrottle(void);
#else
  void readCAN(void);
#endif

// -------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("DEV wireless car testbed"));
  Serial.print(F("Initializing..."));
  
  #ifdef useNRF
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.enableDynamicAck();
    radio.openWritingPipe(sendaddress);
    radio.openReadingPipe(1, recvaddress);
    radio.printDetails();
    radio.startListening();
  #endif

  CANbus.begin();

  // ESC stuff
  setupPins();
  analogWrite(INHA, 0);
  analogWrite(INHB, 0);
  analogWrite(INHC, 0);

  attachInterrupt(HALL1, hallISR, CHANGE);
  attachInterrupt(HALL2, hallISR, CHANGE);
  attachInterrupt(HALL3, hallISR, CHANGE);

  pinMode(A0, INPUT);
  // end ESC stuff

  // pinMode(NSIL, HIGH);
  // pinMode(STANDBY, LOW);

  sysTimer.reset();

  Serial.println(F(" complete"));
}

// -------------------------------------------------------------
void loop(void)
{
  if (commandTimeout.check()){
    #ifdef useNRF
      mostRecentCommands.LSteeringMotor = 0;
      mostRecentCommands.RSteeringMotor = 0;
      mostRecentCommands.throttle = 0;
    #else
      throttleCAN = 0;
    #endif
  }

  if (ESCupdateTimer.check()){
    #ifdef useNRF
      ESCupdate(mostRecentCommands.throttle);
    #else
      ESCupdate(throttleCAN);
    #endif
  }

  #ifdef useNRF
    if (CANtimer.check()){
      sendSteering();
      sendThrottle();
    }
    if (radioReadTimer.check()){
      readRadio();
    }
  #else
    readCAN();
  #endif
}

#ifdef useNRF
void readRadio() {
  if (radio.available()) {
    radio.read(&readBuffer, sizeof(readBuffer));
    Serial.print("Got ");
    for (int i = 0; i<6; i++){
      Serial.print('\t');
      Serial.print(readBuffer[i]);
    }
    mostRecentCommands.LSteeringMotor = ((int32_t)readBuffer[4])*600; // scale from 64 to 0.40*1e5
    mostRecentCommands.RSteeringMotor = ((int32_t)readBuffer[4])*600;
    mostRecentCommands.throttle = max(0, readBuffer[2]) << 6; // scale from 64 to 4096
    Serial.print('\t');
    Serial.print(mostRecentCommands.LSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.RSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.throttle);
    Serial.println();
    commandTimeout.reset();
  }
}
#endif

#ifdef useNRF // send over CAN
void sendSteering(void){
  // Left steering motor command
  msg.id = (CAN_PACKET_SET_DUTY<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = (mostRecentCommands.LSteeringMotor >> 24) & 0xFF;
  msg.buf[1] = (mostRecentCommands.LSteeringMotor >> 16) & 0xFF;
  msg.buf[2] = (mostRecentCommands.LSteeringMotor >> 8) & 0xFF;
  msg.buf[3] = (mostRecentCommands.LSteeringMotor >> 0) & 0xFF;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);

  // right steering motor command
  msg.id = (CAN_PACKET_SET_DUTY<<8) | (RmotorCAN & 0xFF);
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = (mostRecentCommands.RSteeringMotor >> 24) & 0xFF;
  msg.buf[1] = (mostRecentCommands.RSteeringMotor >> 16) & 0xFF;
  msg.buf[2] = (mostRecentCommands.RSteeringMotor >> 8) & 0xFF;
  msg.buf[3] = (mostRecentCommands.RSteeringMotor >> 0) & 0xFF;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
void sendThrottle(void){
  // main motor command
  msg.id = MmotorCAN & 0xFF;
  msg.ext = 0;
  msg.len = 8;
  msg.buf[0] = (mostRecentCommands.throttle >> 8) & 0xFF;
  msg.buf[1] = (mostRecentCommands.throttle >> 0) & 0xFF;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
#else // read over CAN
void readCAN(void){
  if (CANbus.available()){
    int suc = CANbus.read(rxmsg);
    if (suc){
      Serial.print(rxmsg.id);
      Serial.print('\t');
      Serial.print(rxmsg.buf[0]);
      Serial.print('\t');
      Serial.print(rxmsg.buf[1]);
      Serial.print('\t');
      Serial.print(rxmsg.buf[2]);
      Serial.print('\t');
      Serial.print(rxmsg.buf[3]);
      Serial.print('\t');
      if (rxmsg.id == MmotorCAN){
        commandTimeout.reset();
        throttleCAN = (rxmsg.buf[0] << 8) | rxmsg.buf[1];
        Serial.print("got throttle: ");
        Serial.println(throttleCAN);
      } else {
        Serial.println();
      }
    }
  }
}
#endif