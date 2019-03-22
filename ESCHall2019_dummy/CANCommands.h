#ifndef CANCOMMANDS_H
#define CANCOMMANDS_H

#define RmotorCAN 55
#define LmotorCAN 56
#define MmotorCAN 100 // main motor (throttle)
#define accCAN 200
#define gyroCAN 201
#define magCAN 202

#ifdef useCAN

#include <FlexCAN.h>
#include "vESC_datatypes.h"

FlexCAN CANbus(500000);
#define CAN_RXBUFFERSIZE 3
static CAN_message_t msg,rxmsg;
bool rxmsgValid;
// CAN_message_t CAN_rxBuffer[CAN_RXBUFFERSIZE];
// uint8_t CAN_rxBufferWriteInd, CAN_rxBufferReadInd;

int txCount,rxCount;
unsigned int txTimer,rxTimer;

void setupCAN();
void CAN_readBus();
bool CAN_getThrottle(uint16_t* pThrottle, uint32_t* lastTime_CAN);
// TODO: the rest

void setupCAN(){
  CANbus.begin();
}
void CAN_readBus(){
  if (CANbus.available()) {
  	rxmsgValid = CANbus.read(rxmsg);
  	// if (CAN_rxBufferWriteInd>CAN_RXBUFFERSIZE){
  	// 	CAN_rxBufferWriteInd = 0;
  	// }
  	// if (CANbus.read(rxmsg)){
  	// 	if (CAN_rxBufferWriteInd == CAN_rxBufferReadInd){
  	// 		CAN_rxBufferReadInd++;
  	// 		if (CAN_rxBufferReadInd>CAN_RXBUFFERSIZE)
  	// 		{
  	// 			CAN_rxBufferReadInd = 0;
  	// 		}
  	// 	}
  	// 	memcpy(CAN_rxBuffer+(CAN_rxBufferWriteInd*sizeof(CAN_message_t)), rxmsg, sizeof(rxmsg));
  	// 	CAN_rxBufferWriteInd++;
  	// }
    #ifdef DEBUG_PRINT
	if (rxmsgValid){
    	Serial.print("Got message with ID ");
	    Serial.println(rxmsg.id);
	}
    #endif
  }
}
// updates by pointer reference
// returns true if updated, false otherwise
bool CAN_getThrottle(uint16_t* pThrottle, uint32_t* lastTime_CAN){
	if (rxmsgValid){
		if (rxmsg.id==MmotorCAN){
			*pThrottle  = (uint16_t)(rxmsg.buf[0]) << 8;
			*pThrottle |= rxmsg.buf[1];
			rxmsgValid = false;
			*lastTime_CAN = millis();
			// Serial.println(*pThrottle);
			return true;
		}
	}
	return false;
}

void CAN_sendSteering(int32_t LSteeringMotor, int32_t RSteeringMotor){
  // Left steering motor command
  msg.id = (CAN_PACKET_SET_DUTY<<8) | (LmotorCAN & 0xFF); // 0x3 is RPM
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = (LSteeringMotor >> 24) & 0xFF;
  msg.buf[1] = (LSteeringMotor >> 16) & 0xFF;
  msg.buf[2] = (LSteeringMotor >> 8) & 0xFF;
  msg.buf[3] = (LSteeringMotor >> 0) & 0xFF;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);

  // right steering motor command
  msg.id = (CAN_PACKET_SET_DUTY<<8) | (RmotorCAN & 0xFF);
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = (RSteeringMotor >> 24) & 0xFF;
  msg.buf[1] = (RSteeringMotor >> 16) & 0xFF;
  msg.buf[2] = (RSteeringMotor >> 8) & 0xFF;
  msg.buf[3] = (RSteeringMotor >> 0) & 0xFF;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
void CAN_sendThrottle(uint16_t throttle){
  // main motor command
  msg.id = MmotorCAN & 0xFF;
  msg.ext = 0;
  msg.len = 8;
  msg.buf[0] = (throttle >> 8) & 0xFF;
  msg.buf[1] = (throttle >> 0) & 0xFF;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
void CAN_sendTachReq(void){ // not working yet
  // left steering get tach
  msg.id = (CAN_PACKET_GET_TACH<<8) | (LmotorCAN & 0xFF);
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
  // right steering get tach
  msg.id = (CAN_PACKET_GET_TACH<<8) | (RmotorCAN & 0xFF);
  msg.ext = 1;
  msg.len = 8;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
void CAN_sendIMU(float accX, float accY, float accZ,
				 float angX, float angY, float angZ){
  #ifdef useIMU
	#ifdef DEBUG_PRINT
    Serial.print("accX : ");Serial.print(accX);
    Serial.print("\taccY : ");Serial.print(accY);
    Serial.print("\taccZ : ");Serial.print(accZ);
    Serial.print("\tangleX : ");Serial.print(angX);
    Serial.print("\tangleY : ");Serial.print(angY);
    Serial.print("\tangleZ : ");Serial.println(angZ);
    Serial.println("=======================================================\n");
    #endif
    // accelerometer
    msg.id = accCAN & 0xFF;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = (((int16_t)(accX*1e3)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(accX*1e3)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(accY*1e3)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(accY*1e3)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(accZ*1e3)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(accZ*1e3)) >> 0) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CANbus.write(msg);
    // gyro
    msg.id = gyroCAN & 0xFF;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = (((int16_t)(angX*1e2)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(angX*1e2)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(angY*1e2)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(angY*1e2)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(angZ*1e2)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(angZ*1e2)) >> 0) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CANbus.write(msg);
  #endif
}
#endif // useCAN

#endif