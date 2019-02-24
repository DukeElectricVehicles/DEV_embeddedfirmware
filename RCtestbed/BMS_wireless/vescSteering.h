#ifndef VESCSTEERING_H
#define VESCSTEERING_H

#include "config.h"
#include "vESC_datatypes.h"
#include <FlexCAN.h>

void initSteering();
void updateSteering();
void readVESCCAN();
void pollAngle();
void homeSteering();
void sendSteeringPos(float deg);
void sendSteeringDuty(float duty);

typedef enum {
	STEERINGMODE_HOME,
	STEERINGMODE_NORMAL,
	STEERINGMODE_FAULT
} steeringMode_t;

#define HOMEANGLE_DEG -25
steeringMode_t steeringMode;
float homeOffset, lastKnownPos;

extern driveCommands_t mostRecentCommands;
extern FlexCAN CANbus;
static CAN_message_t msg_steer;
static CAN_message_t* rxmsg_steer;

void initSteering(CAN_message_t* rxmsg_steer_){
  pinMode(STEER_HOMEPIN, OUTPUT);
  steeringMode = STEERINGMODE_NORMAL;
  lastKnownPos = 0;
  homeOffset = 0;
  rxmsg_steer = rxmsg_steer_;
}

void updateSteering(){
  switch (steeringMode){
  	case STEERINGMODE_NORMAL:
		  // sendSteeringPos(mostRecentCommands.LSteeringMotor);
			pollAngle();
		  break;
		case STEERINGMODE_HOME:
			if (digitalRead(STEER_HOMEPIN)){
				sendSteeringDuty(0);
				homeOffset = lastKnownPos;
				steeringMode = STEERINGMODE_NORMAL;
			}
			else {
				sendSteeringDuty(.01);
				pollAngle();
			}
			break;
		case STEERINGMODE_FAULT:
			break;
  }
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
				Serial.print("got vESC position: ");
				Serial.print(pos/1e6);
				Serial.print("°\n");
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
  if (deg > 10){
    deg = 10;
  }
  if (deg < -10){
    deg = -10;
  }

  deg = deg-homeOffset-HOMEANGLE_DEG;

  while (deg < 0){
    deg = 360+deg;
  }
  while (deg > 360){
  	deg = deg-360;
  }

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

#endif