#include <Metro.h>
#include <FlexCAN.h>
#include "RF24.h"
#include "vESC_datatypes.h"

#define NSIL 3
#define STANDBY 4

#define RmotorCAN 55
#define LmotorCAN 56
#define MmotorCAN 100 // main motor (throttle)

typedef struct {
  int32_t LSteeringMotor;
  int32_t RSteeringMotor;
  uint8_t throttle;
} driveCommands_t;

RF24 radio(7,8);

uint8_t recvaddress[6] = "car00";
uint8_t sendaddress[6] = "contr";

int8_t readBuffer[6];
driveCommands_t mostRecentCommands;

FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;

int txCount,rxCount;
unsigned int txTimer,rxTimer;

Metro sysTimer = Metro(1);// milliseconds
Metro CANtimer = Metro(10);
Metro wirelessTimeout = Metro(100);

void readRadio(void);
void sendSteering(void);
void sendThrottle(void);

// -------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("DEV wireless car testbed"));
  Serial.print(F("Initializing..."));
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.openWritingPipe(sendaddress);
  radio.openReadingPipe(1, recvaddress);
  radio.printDetails();
  radio.startListening();

  CANbus.begin();

  // pinMode(NSIL, HIGH);
  // pinMode(STANDBY, LOW);

  sysTimer.reset();

  Serial.println(F(" complete"));
}

// -------------------------------------------------------------
void loop(void)
{
  if (CANtimer.check()){
    sendSteering();
    sendThrottle();
  }

  if (CANbus.available()) {
    int test = CANbus.read(rxmsg);
    Serial.write(sizeof(test));
    Serial.write("=");
    Serial.println(rxmsg.id);
  }

  if (wirelessTimeout.check()){
    mostRecentCommands.LSteeringMotor = 0;
    mostRecentCommands.RSteeringMotor = 0;
    mostRecentCommands.throttle = 0;
  }

  readRadio();
}

void readRadio() {
  if (radio.available()) {
    radio.read(&readBuffer, sizeof(readBuffer));
    Serial.print("Got ");
    for (int i = 0; i<6; i++){
      Serial.print('\t');
      Serial.print(readBuffer[i]);
    }
    mostRecentCommands.LSteeringMotor = ((int32_t)readBuffer[4])*50;
    mostRecentCommands.RSteeringMotor = ((int32_t)readBuffer[4])*50;
    mostRecentCommands.throttle = max(0, readBuffer[2]);
    Serial.print('\t');
    Serial.print(mostRecentCommands.LSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.RSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.throttle);
    Serial.println();
    wirelessTimeout.reset();
  }
}
void sendSteering(void){
  // Left steering motor command
  msg.id = (CAN_PACKET_SET_RPM<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
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
  msg.id = (CAN_PACKET_SET_RPM<<8) | (RmotorCAN & 0xFF);
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
  msg.buf[0] = mostRecentCommands.throttle & 0xFF;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}