#include <i2c_t3.h>
#include <FlexCAN.h>
#include <Chrono.h>
#include "infinityPV_INA233.h"

#define LED 13

INA233 IC1(0x40);
#define INA_ALERT 17

#define CAN_MODE_PIN 5
FlexCAN CANbus(500000);
static CAN_message_t txmsg,rxmsg;

#define ENERGY_WINDOW 100
float energyBuf[ENERGY_WINDOW];
uint32_t energyBufPos = 0;
float averagePower = 0;

Chrono printChrono;
Chrono LCDChrono;

float IVT_V = 0;
float IVT_I = 0;
float IVT_E = 0;
float INA_V = 0;
float INA_I = 0;
float INA_E = 0;

uint32_t INA_Poll_Count = 0;
uint32_t INA_Poll_Time = 0;
uint32_t IVT_Poll_Time = 0;

#define DISPLAY_ADDRESS 0x72 //default address of the OpenLCD

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  pinMode(INA_ALERT, INPUT_PULLUP);
  IC1.begin();
  IC1.wireWriteByte(MFR_ALERT_MASK, 0x7F); //only turn on conversion complete ALERT
  IC1.wireSendCmd(CLEAR_FAULTS);
  //IC1.wireWriteWord(MFR_ADC_CONFIG, 0x00DF);//588us per conversion, no averaging
  //IC1.wireWriteWord(MFR_ADC_CONFIG, 0x0097);//332us per conversion, no averaging
  
  CANbus.begin();
  delay(1000);
  pinMode(CAN_MODE_PIN, OUTPUT);
  digitalWrite(CAN_MODE_PIN, LOW);//put CAN transciever to low power mode
  
  setupIVT();

  
  /*Wire.beginTransmission(DISPLAY_ADDRESS); // transmit to device #1
  Wire.write('|'); //Put LCD into setting mode
  Wire.write(0x08); //reset LCD
  Wire.endTransmission(); //Stop I2C transmission*/

  INA_Poll_Time = IVT_Poll_Time = micros();
}

void loop() {
  
  if(digitalRead(INA_ALERT) == 0)
  {
    INA_Poll_Count++;
    IC1.wireSendCmd(CLEAR_FAULTS); //clear ALERT pin

    if(INA_Poll_Count % 2 == 0) //only take every other cycle, as both I and V conversions trigger ALERT
    {
      uint32_t curTime = micros();
      uint32_t dt = curTime - INA_Poll_Time;
      INA_V = IC1.getBusVoltage_raw() / 800.0;
      INA_I = IC1.getShuntVoltage_raw() / 2000.0;
      INA_E += INA_V * INA_I * dt / 1e6;   
      INA_Poll_Time = curTime;
    }    
  }
  
  if(printChrono.hasPassed(100, true))
  {     
    averagePower = (INA_E - energyBuf[energyBufPos]) / (0.1 * ENERGY_WINDOW);
    energyBuf[energyBufPos++] = INA_E;
    energyBufPos %= ENERGY_WINDOW;
    
    Serial.print(IVT_V, 3);
    Serial.print('\t');
    Serial.print(IVT_I, 3);
    Serial.print('\t');
    Serial.print(INA_V, 3);
    Serial.print('\t');
    Serial.print(INA_I, 3);
    Serial.print('\t');
    Serial.print(averagePower);
    Serial.print('\t');
    Serial.print(IVT_E);
    Serial.print('\t');
    Serial.print(INA_E);
    Serial.println('\t');


    digitalWrite(LED, !digitalRead(LED));
  }

  if(LCDChrono.hasPassed(250, true))
  {
    writeLCD();
  }
  

  parseCAN();

  if(Serial.available())
    if(Serial.read() == '0')
    {
      IVT_E = 0;
      INA_E = 0;
    }
}

void parseCAN(void)
{
  while(CANbus.available()) {
    CANbus.read(rxmsg);

    uint8_t msg_id = rxmsg.buf[0];
    //uint8_t error = rxmsg.buf[1] >> 4;
    //uint8_t counter = rxmsg.buf[1] & 0xF;
    int32_t payload = rxmsg.buf[5];
    payload |= rxmsg.buf[4] << 8;
    payload |= rxmsg.buf[3] << 16;
    payload |= rxmsg.buf[2] << 24;

    if(msg_id == 0x00)
      IVT_I = payload / 1000.0;

    if(msg_id == 0x01)
    {
      IVT_V = payload / 1000.0;
      uint32_t curTime = micros();
      
      IVT_E += IVT_I * IVT_V * (curTime - IVT_Poll_Time) / 1e6;
      IVT_Poll_Time = curTime;
    }

    //if(msg_id == 0x07)
    //  IVT_E = payload * 3600.0;
  }
}

void setupIVT(void)
{
  IVTConfig(0x34, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00); //put sensor in STOP mode so we can change settings
  IVTConfig(0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); //Reset sensor
  delay(1000);
  IVTConfig(0x34, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00); //put sensor in STOP mode so we can change settings
  IVTConfig(0x20, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00); //Set current to 5ms
  IVTConfig(0x21, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00); //Set U1 to 5ms
  IVTConfig(0x22, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00); //Turn off U2
  IVTConfig(0x23, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00); //Turn off U3
  IVTConfig(0x27, 0x02, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00); //Set energy reporting to 100ms
  IVTConfig(0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00); //put back in RUN mode
}

void IVTConfig(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  txmsg.len = 8;
  txmsg.id = 0x411;
  txmsg.buf[0] = b0;
  txmsg.buf[1] = b1;
  txmsg.buf[2] = b2;
  txmsg.buf[3] = b3;
  txmsg.buf[4] = b4;
  txmsg.buf[5] = b5;
  txmsg.buf[6] = b6;
  txmsg.buf[7] = b7;
  CANbus.write(txmsg);
  delay(5);
}

void writeLCD(void)
{
  char buf[30];
  int32_t sec = millis() / 1000;
  
  Wire.beginTransmission(DISPLAY_ADDRESS); // transmit to device #1
  Wire.write('|'); //Put LCD into setting mode
  Wire.write('-'); //Send clear display command

  //sprintf(buf, "%5.2fV %5.2fA", INA_V, INA_I);
  sprintf(buf, "%7.1fJ", INA_E);
  Wire.print(buf);

  Wire.write(254);//command char
  Wire.write(128 + 64 + 0);//cursor to line 1 col 0

  //sprintf(buf, "%6.1fJ %6.1fJ", INA_E, IVT_E);
  //sprintf(buf, "%6.0fJ %5ds", INA_E, sec);
  sprintf(buf, "%7ds", sec);
  Wire.print(buf);
  
  Wire.endTransmission(); //Stop I2C transmission
}
