/*
	Gerry Chen - Apr. 21, 2019

	Communicate with the Swift Piksi Multi RTK GPS over UART

	This code is just an Arduino version of the code from this github repo:
		https://github.com/swift-nav/libsbp

	For details on the protocol specification, please refer to:
		https://support.swiftnav.com/customer/en/portal/articles/2492810-swift-binary-protocol

*/

#include "libsbp/common.h"
#include "libsbp/sbp.h"
#include "libsbp/system.h"
#include "libsbp/navigation.h"
#include "Metro.h"

static sbp_state_t sbp_state;
static s8 retStatus;

s32 RTK_readSerial(u8 *buff, u32 n, void *context);

static sbp_msg_callbacks_node_t
	RTK_heartbeat_cbnode,
	RTK_posLLH_cbnode,
	RTK_velNED_cbnode;
void RTK_heartbeat_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void RTK_posLLH_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void RTK_velNED_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void RTK_printmsg(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);

Metro RTKaliveTimer(1100);
Metro GPSvalidTimer(200);
Metro debugPrintTimer(100);
bool RTKisAlive, GPSisValid;

msg_pos_llh_t curPos;
msg_vel_ned_t curVel;

void setup() {
	Serial.begin(115200);
	Serial1.begin(230400);

	Serial.println("Duke Electric Vehicles - Piksi Multi RTK GPS interface example");

  sbp_state_init(&sbp_state);

	sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &RTK_heartbeat_cb, NULL, &RTK_heartbeat_cbnode);
	sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &RTK_posLLH_cb, NULL, &RTK_posLLH_cbnode);
	sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &RTK_velNED_cb, NULL, &RTK_velNED_cbnode);

}

void loop() {
  retStatus = sbp_process(&sbp_state, &RTK_readSerial);

  if(RTKaliveTimer.check()){
  	Serial.println("Missed RTK heartbeat signal!");
  	RTKisAlive = false;
  }
  if(GPSvalidTimer.check()){
  	Serial.println("GPS position lost");
  	GPSisValid = false;
  }
  if(debugPrintTimer.check()){
  	Serial.print('\t'); Serial.print(curPos.tow);
  	Serial.print('\t'); Serial.print(curPos.lat, 6);
  	Serial.print('\t'); Serial.print(curPos.lon, 6);
  	Serial.print('\t'); Serial.print(curPos.height,2);
  	Serial.print('\t'); Serial.print(curPos.h_accuracy);
  	Serial.print('\t'); Serial.print(curPos.v_accuracy);
  	Serial.print('\t'); Serial.print(curPos.n_sats);
  	Serial.print('\t'); Serial.print(curPos.flags, HEX);
  	Serial.print('\t');
  	Serial.print('\t'); Serial.print(curVel.n);
  	Serial.print('\t'); Serial.print(curVel.e);
  	Serial.print('\t'); Serial.print(curVel.d);
  	Serial.print('\t'); Serial.print(curVel.h_accuracy);
  	Serial.print('\t'); Serial.print(curVel.v_accuracy);
  	Serial.print('\t'); Serial.print(curVel.n_sats);
  	Serial.print('\t'); Serial.print(curVel.flags, HEX);
  	Serial.print('\n');
  	// Serial.println("teensy is alive");
  }

}

void RTK_heartbeat_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
	msg_heartbeat_t* hbMsg = (msg_heartbeat_t*) msg;
	if ((hbMsg->flags>>0) & 1) 
		Serial.println("RTK: system error");
	if ((hbMsg->flags>>1) & 1) 
		Serial.println("RTK: IO error");
	if ((hbMsg->flags>>2) & 1) 
		Serial.println("RTK: swiftNAP error");
	if (!((hbMsg->flags>>31) & 1))
		Serial.println("RTK: no external antenna");
	RTKaliveTimer.reset();
	RTKisAlive = true;
}
void RTK_posLLH_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
	msg_pos_llh_t* posMsg = (msg_pos_llh_t*) msg;
	curPos = *posMsg;
	GPSvalidTimer.reset();
	GPSisValid = true;
}
void RTK_velNED_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
	msg_vel_ned_t* velMsg = (msg_vel_ned_t*) msg;
	curVel = *velMsg;
}
void RTK_printmsg(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
	Serial.print(sender_id);
	Serial.print('\t');	Serial.print(len);
	Serial.print('\t');
	for (uint8_t i = 0; i<len; i++){
		Serial.print('\t');	Serial.print(msg[i], HEX);
	}
	Serial.print('\n');
}

s32 RTK_readSerial(u8 *buff, u32 n, void *context){
	static s32 i;
  for (i=0; i<n; i++) {
    if (Serial1.available()){
      buff[i] = Serial1.read();
    }
    else
      break;
  }
  return i;
}