#ifndef VESCSTEERING_H
#define VESCSTEERING_H

#include "config.h"
#include "vESC_datatypes.h"
#include <FlexCAN.h>

#define MAXANGLE 20
#define MIN_D 3.909
#define L_PIVOT 6.014
// #define RAD_TO_DEG 57.2957795131 // miraculously this was already defined in Arduino somewhere
#define TPI 28
#define HOMEANGLE_DEG -MAXANGLE

void initSteering();
void updateSteering();
void readVESCCAN();
void pollAngle();
void homeSteering();
void sendSteeringPos(float deg);
void sendSteeringDuty(float duty);
void sendSteeringVel(int32_t rpm);

typedef enum {
  STEERINGMODE_INIT,
	STEERINGMODE_HOME,
	STEERINGMODE_NORMAL,
	STEERINGMODE_FAULT
} steeringMode_t;

steeringMode_t steeringMode;
float homeOffset, lastKnownPos;

extern driveCommands_t mostRecentCommands;
extern FlexCAN CANbus;
static CAN_message_t msg_steer;
static CAN_message_t* rxmsg_steer;

void initSteering(CAN_message_t* rxmsg_steer_){
  lastKnownPos = -1.2345;
  pinMode(STEER_HOMEPIN, INPUT);
  steeringMode = STEERINGMODE_INIT;
  lastKnownPos = -1.23456; // will be updated
  homeOffset = 0; // will be updated
  rxmsg_steer = rxmsg_steer_;
  pollAngle();
}

void updateSteering(){
  // Serial.print("state: ");
  // Serial.print(steeringMode);
  switch (steeringMode){
    case STEERINGMODE_INIT:
      if (lastKnownPos != -1.23456){
        homeOffset = lastKnownPos;
        steeringMode = STEERINGMODE_NORMAL;
      }
  	case STEERINGMODE_NORMAL:
      sendSteeringPos(map(mostRecentCommands.LSteeringMotor,-64,64,-MAXANGLE,MAXANGLE));
			pollAngle();
		  break;
		case STEERINGMODE_HOME:
      // Serial.print("\thome pin:");
      // Serial.print(digitalRead(STEER_HOMEPIN));
      // Serial.println();
			if (digitalRead(STEER_HOMEPIN)){
				sendSteeringDuty(0);
				homeOffset = lastKnownPos + HOMEANGLE_DEG;
				steeringMode = STEERINGMODE_NORMAL;
			}
			else {
				sendSteeringDuty(.1);
				pollAngle();
			}
			break;
		case STEERINGMODE_FAULT:
			break;
  }
  // Serial.println();
}

void readVESCCAN(){
	if (((rxmsg_steer->id) & 0xFF) == LmotorCAN){
		switch ((rxmsg_steer->id) >> 8){
			case CAN_PACKET_POSVAL:
				int32_t pos = 0;
				pos |= ((uint32_t)(rxmsg_steer->buf[0])) << 24;
				pos |= ((uint32_t)(rxmsg_steer->buf[1])) << 16;
				pos |= ((uint32_t)(rxmsg_steer->buf[2])) << 8;
				pos |= ((uint32_t)(rxmsg_steer->buf[3])) << 0;
				// Serial.print("got vESC position: ");
				// Serial.print(pos/1e6);
				// Serial.print("°\n");
				lastKnownPos = pos / 1000000.0;
				break;
		}
	}
}
void pollAngle(){
	msg_steer.id = ((CAN_PACKET_GET_POS<<8) | (LmotorCAN & 0xFF));
	msg_steer.ext = 1;
	msg_steer.len = 0;
	CANbus.write(msg_steer);
}

void homeSteering(){
	steeringMode = STEERINGMODE_HOME;
}
void sendSteeringPos(float deg){ // angle in deg
  if (deg > MAXANGLE){
    deg = MAXANGLE;
  }
  if (deg < -MAXANGLE){
    deg = -MAXANGLE;
  }

  deg = deg+homeOffset;

  while (deg < 0){
    deg = 360+deg;
  }
  while (deg > 360){
  	deg = deg-360;
  }

  // Serial.println(deg);

  int32_t toSend = deg*1e6;

  // steering motor command
  msg_steer.id = (CAN_PACKET_SET_POS<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
  msg_steer.ext = 1;
  msg_steer.len = 8;
  msg_steer.buf[0] = (toSend >> 24) & 0xFF;
  msg_steer.buf[1] = (toSend >> 16) & 0xFF;
  msg_steer.buf[2] = (toSend >> 8) & 0xFF;
  msg_steer.buf[3] = (toSend >> 0) & 0xFF;
  msg_steer.buf[4] = 0x00;
  msg_steer.buf[5] = 0x00;
  msg_steer.buf[6] = 0x00;
  msg_steer.buf[7] = 0x00;
  CANbus.write(msg_steer);
}
void sendSteeringDuty(float duty){ // duty cycle from -1 to 1
  if (duty > 1){
    duty = 1;
  }
  if (duty < -1){
    duty = -1;
  }

  int32_t toSend = duty * 1e5;
  // steering motor command
  msg_steer.id = (CAN_PACKET_SET_DUTY<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
  msg_steer.ext = 1;
  msg_steer.len = 8;
  msg_steer.buf[0] = (toSend >> 24) & 0xFF;
  msg_steer.buf[1] = (toSend >> 16) & 0xFF;
  msg_steer.buf[2] = (toSend >> 8) & 0xFF;
  msg_steer.buf[3] = (toSend >> 0) & 0xFF;
  msg_steer.buf[4] = 0x00;
  msg_steer.buf[5] = 0x00;
  msg_steer.buf[6] = 0x00;
  msg_steer.buf[7] = 0x00;
  CANbus.write(msg_steer);
}
void sendSteeringVel(int32_t rpm){ // duty cycle from -1 to 1
  if (rpm > 30000){
    rpm = 30000;
  }
  if (rpm < -30000){
    rpm = -30000;
  }

  int32_t toSend = rpm * 1e0;
  // steering motor command
  msg_steer.id = (CAN_PACKET_SET_RPM<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
  msg_steer.ext = 1;
  msg_steer.len = 8;
  msg_steer.buf[0] = (toSend >> 24) & 0xFF;
  msg_steer.buf[1] = (toSend >> 16) & 0xFF;
  msg_steer.buf[2] = (toSend >> 8) & 0xFF;
  msg_steer.buf[3] = (toSend >> 0) & 0xFF;
  msg_steer.buf[4] = 0x00;
  msg_steer.buf[5] = 0x00;
  msg_steer.buf[6] = 0x00;
  msg_steer.buf[7] = 0x00;
  CANbus.write(msg_steer);
}

#endif