#ifndef VESCTHROTTLE_H
#define VESCTHROTTLE_H

#include "config.h"
#include "vESC_datatypes.h"
#include <FlexCAN.h>

void initThrottle();
void updateThrottle();
// void readVESCCAN();
// void sendThrottlePos(float deg);
void sendThrottleDuty(float duty);
void sendThrottleVel(int32_t rpm);

extern driveCommands_t mostRecentCommands;
extern FlexCAN CANbus;
static CAN_message_t msg_throttle;
static CAN_message_t* rxmsg_throttle;

void initThrottle(CAN_message_t* rxmsg_throttle_){
  rxmsg_throttle = rxmsg_throttle_;
  pollAngle();
}

void updateThrottle(){
  // Serial.print("state: ");
  // Serial.print(ThrottleMode);
  sendThrottleDuty(mostRecentCommands.throttle/4096.0);
}

// void readVESCCAN(){
// 	if (((rxmsg_throttle->id) & 0xFF) == MmotorCANVESC){
// 		switch ((rxmsg_throttle->id) >> 8){
// 			case CAN_PACKET_POSVAL:
// 				int32_t pos = 0;
// 				pos |= ((uint32_t)(rxmsg_throttle->buf[0])) << 24;
// 				pos |= ((uint32_t)(rxmsg_throttle->buf[1])) << 16;
// 				pos |= ((uint32_t)(rxmsg_throttle->buf[2])) << 8;
// 				pos |= ((uint32_t)(rxmsg_throttle->buf[3])) << 0;
// 				// Serial.print("got vESC position: ");
// 				// Serial.print(pos/1e6);
// 				// Serial.print("°\n");
// 				lastKnownPos = pos / 1000000.0;
// 				break;
// 		}
// 	}
// }

// void sendThrottlePos(float deg){ // angle in deg
//   if (deg > MAXANGLE){
//     deg = MAXANGLE;
//   }
//   if (deg < -MAXANGLE){
//     deg = -MAXANGLE;
//   }

//   deg = deg+homeOffset;

//   while (deg < 0){
//     deg = 360+deg;
//   }
//   while (deg > 360){
//   	deg = deg-360;
//   }

//   // Serial.println(deg);

//   int32_t toSend = deg*1e6;

//   // Throttle motor command
//   msg_throttle.id = (CAN_PACKET_SET_POS<<8) | (MmotorCANVESC & 0xFF); // 0x3 is RPM
//   msg_throttle.ext = 1;
//   msg_throttle.len = 8;
//   msg_throttle.buf[0] = (toSend >> 24) & 0xFF;
//   msg_throttle.buf[1] = (toSend >> 16) & 0xFF;
//   msg_throttle.buf[2] = (toSend >> 8) & 0xFF;
//   msg_throttle.buf[3] = (toSend >> 0) & 0xFF;
//   msg_throttle.buf[4] = 0x00;
//   msg_throttle.buf[5] = 0x00;
//   msg_throttle.buf[6] = 0x00;
//   msg_throttle.buf[7] = 0x00;
//   CANbus.write(msg_throttle);
// }
void sendThrottleDuty(float duty){ // duty cycle from -1 to 1
  if (duty > 1){
    duty = 1;
  }
  if (duty < -1){
    duty = -1;
  }

  int32_t toSend = duty * 1e5;
  // Throttle motor command
  msg_throttle.id = (CAN_PACKET_SET_DUTY<<8) | (MmotorCANVESC & 0xFF); // 0x3 is RPM
  msg_throttle.ext = 1;
  msg_throttle.len = 8;
  msg_throttle.buf[0] = (toSend >> 24) & 0xFF;
  msg_throttle.buf[1] = (toSend >> 16) & 0xFF;
  msg_throttle.buf[2] = (toSend >> 8) & 0xFF;
  msg_throttle.buf[3] = (toSend >> 0) & 0xFF;
  msg_throttle.buf[4] = 0x00;
  msg_throttle.buf[5] = 0x00;
  msg_throttle.buf[6] = 0x00;
  msg_throttle.buf[7] = 0x00;
  CANbus.write(msg_throttle);
}
void sendThrottleVel(int32_t rpm){ // duty cycle from -1 to 1
  // I noticed that, even though the VESC seems to indicate it's providing power
  //  to the motor on the VESC tool when commanded 0 speed, I don't actually think it is
  // if (rpm < 100){ // to avoid static 1A current draw at brake
  //   sendThrottleDuty(0);
  //   return;
  // }
  if (rpm > 30000){
    rpm = 30000;
  }
  if (rpm < -30000){
    rpm = -30000;
  }

  int32_t toSend = rpm * 1e0;
  // Throttle motor command
  msg_throttle.id = (CAN_PACKET_SET_RPM<<8) | (MmotorCANVESC & 0xFF); // 0x3 is RPM
  msg_throttle.ext = 1;
  msg_throttle.len = 8;
  msg_throttle.buf[0] = (toSend >> 24) & 0xFF;
  msg_throttle.buf[1] = (toSend >> 16) & 0xFF;
  msg_throttle.buf[2] = (toSend >> 8) & 0xFF;
  msg_throttle.buf[3] = (toSend >> 0) & 0xFF;
  msg_throttle.buf[4] = 0x00;
  msg_throttle.buf[5] = 0x00;
  msg_throttle.buf[6] = 0x00;
  msg_throttle.buf[7] = 0x00;
  CANbus.write(msg_throttle);
}

#endif