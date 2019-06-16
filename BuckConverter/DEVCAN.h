#ifndef DEVCAN_H
#define DEVCAN_H

#include <FlexCAN.h>

// board specific declarations
extern float Isetpoint;

// CAN stuff
FlexCAN CANbus(500000);
static CAN_message_t txmsg,rxmsg;

typedef enum {
  CAN_BMS2BUCK_CURRENT = 0X101
};

// function declarations
void setupCAN(void);
void parseCAN(void);
int32_t readFromBuffer_int32(uint8_t *buf);
int16_t readFromBuffer_int16(uint8_t *buf);
void sendSetpointCurrentCAN(float Isetpoint);

// function implementations
void setupCAN(void) {
  CANbus.begin();
  pinMode(CAN_MODE_PIN, OUTPUT);
  digitalWrite(CAN_MODE_PIN, LOW);//put CAN transciever to low power mode
}

void parseCAN(void) {
  while(CANbus.available()) {
    CANbus.read(rxmsg);
    // Serial.print(rxmsg.id, HEX);
    // for (uint8_t i = 0; i<rxmsg.len; i++) {
    //   Serial.print('\t');
    //   Serial.print(rxmsg.buf[i]);
    // }
    // Serial.println();
    // BMS
    #ifdef BMS
      switch (rxmsg.id) {
        break;
      }

    // BUCK CONVERTER
    #elif defined(BUCK)
      switch (rxmsg.id) {
        case CAN_BMS2BUCK_CURRENT:
          Isetpoint = readFromBuffer_int16(rxmsg.buf) / 1000.0;
          break;
      }
    #else
      #error "No board defined"
    #endif
  }
}

int32_t readFromBuffer_int32(uint8_t *buf) {
  int32_t retVal = buf[0];
  retVal |= buf[1] << 8;
  retVal |= buf[2] << 16;
  retVal |= buf[3] << 24;
  return retVal;
}
int16_t readFromBuffer_int16(uint8_t *buf) {
  int32_t retVal = buf[0];
  retVal |= buf[1] << 8;
  return retVal;
}

void sendSetpointCurrentCAN(float Isetpoint) {
  int16_t toSend = constrain(Isetpoint * 1000, -(1<<15), (1<<15)-1);
  txmsg.len = 8;
  txmsg.id = CAN_BMS2BUCK_CURRENT;
  txmsg.buf[0] = (toSend >> 0) & 0xFF;
  txmsg.buf[1] = (toSend >> 8) & 0xFF;
  txmsg.buf[2] = 0;
  txmsg.buf[3] = 0;
  txmsg.buf[4] = 0;
  txmsg.buf[5] = 0;
  txmsg.buf[6] = 0;
  txmsg.buf[7] = 0;
  CANbus.write(txmsg);
}

#endif