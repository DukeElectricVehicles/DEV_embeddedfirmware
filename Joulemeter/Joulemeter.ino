#include <i2c_t3.h>
#include <FlexCAN.h>
#include "infinityPV_INA233.h"

INA233 IC1(0x40);
FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;


uint16_t CAL=0;
int16_t m_c=0;
int16_t m_p=0;
int8_t R_c=0;
int8_t R_p=0;
uint8_t Set_ERROR=0;
//uint8_t r_data8=0;
float Current_LSB=0;
float Power_LSB=0;

void setup() {
  float R_shunt_IC1=0.005;
  float I_max_IC1=16.38;
  
  Serial.begin(115200);
  IC1.begin();
  CAL=IC1.setCalibration(R_shunt_IC1,I_max_IC1,&Current_LSB,&Power_LSB,&m_c,&R_c,&m_p,&R_p,&Set_ERROR);

  //CANbus.begin();
  //Configuring the devices for clearing the accumulator after every reading
  //Check INA233 Datasheet section 7.6.2.14 for more info.
  //IC1.wireWriteByte (MFR_DEVICE_CONFIG, 0x06);
 
}

void loop() {
  float av_power_IC1=0;
  static float Energy_IC1_Wh=0;
  
  av_power_IC1=IC1.getAv_Power_mW();
  Energy_IC1_Wh=Energy_IC1_Wh+av_power_IC1/(3600L*1000L);
  Serial.print(" AvPower IC1:   "); Serial.print(av_power_IC1);Serial.println(" mW");
  Serial.print(" Accumulated Energy IC1:   "); Serial.print(Energy_IC1_Wh);Serial.println(" Wh");
  Serial.println(IC1.getBusVoltage_V());
  Serial.println(IC1.getCurrent_mA());
  //Serial.println(CAL);

  
  Serial.print("CAN msgs ");
  Serial.println(CANbus.available());
  
  /*while ( CANbus.read(rxmsg) ) {
    Serial.println("Reading from CAN!!!");
    //hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
    Serial.write(rxmsg.buf[0]);
  }*/


  msg.len = 8;
  msg.id = 0x222;
  for( int idx=0; idx<8; ++idx ) {
    msg.buf[idx] = '0'+idx;
  }
  CANbus.write(msg);
    
  delay(100);
}
