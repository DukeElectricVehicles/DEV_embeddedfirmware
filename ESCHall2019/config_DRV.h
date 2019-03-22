#ifndef CONFIG_DRV_H
#define CONFIG_DRV_H

#ifndef DRV8301
	// #error "only DRV8301 supported, please pound define DRV8301"
#endif

#include <SPI.h>
#include "config.h"

SPISettings settingsDRV(2000000, MSBFIRST, SPI_MODE1);

// function declarations
void setupDRV();
bool checkDRVfaults();
void printDRVfaults(uint16_t status1, uint16_t status2);
uint16_t DRV_SPIread(uint8_t addr);
void DRV_SPIwrite(uint8_t addr, uint16_t data);

// this function just makes sure SPI communications are working and no fault conditions exist
void setupDRV(){
  pinMode(DRV_EN_GATE, OUTPUT);

  pinMode(DRV_CLK, OUTPUT);
  pinMode(DRV_MOSI, OUTPUT);
  pinMode(DRV_MISO, INPUT);
  pinMode(DRV_CS, OUTPUT);
  digitalWriteFast(DRV_CS, HIGH);

  kickDog();
  digitalWriteFast(DRV_EN_GATE, LOW);
  delay(50);
  kickDog(); // WDOG set to 100ms
  delay(50);
  kickDog();
  digitalWriteFast(DRV_EN_GATE, HIGH);
  delay(50);
  kickDog();
  delay(50);
  kickDog();

  SPI.begin();
  Serial.println("Setting up DRV SPI");
  SPI.beginTransaction(settingsDRV);

  // Refer to section 7.6 of the DRV8301 datasheet
  // 0x00 - Status register 1: any faults show as 1's
  // 0x01 - Status register 2: device ID (default 0x1)
  while((DRV_SPIread(0x00) != 0x00) || (DRV_SPIread(0x01) != 0x01))
  {
    kickDog();
    Serial.println("DRV init fail");
    DRV_SPIwrite(0x02, 0x00);
    uint16_t regs [4];
    for(uint32_t i = 0; i < 4; i++)
    {
      Serial.print("0x");
      Serial.println(DRV_SPIread(i),HEX);
      regs[i] = DRV_SPIread(i);
      Serial.print("0x");
      Serial.println(regs[i],HEX);
    }
    printDRVfaults(regs[0], regs[1]);
    
    digitalWriteFast(DRV_EN_GATE, LOW);
    delay(10);
    digitalWriteFast(DRV_EN_GATE, HIGH);
    kickDog();
    delay(100);
    kickDog();
    delay(100);
    kickDog();
    delay(100);
    kickDog();
    delay(100);
    kickDog();
  }
  DRV_SPIwrite(0x02, 0x20);

  SPI.endTransaction();
  Serial.println("Finished DRV SPI setup");
  // SPI.end();
}

// returns true if there is a fault
bool checkDRVfaults(){
  #ifdef DRV8301
    uint16_t status1 = DRV_SPIread(0x00);
    uint16_t status2 = DRV_SPIread(0x01);
    if ((status1 != 0x00) || (status2 != 0x01)){
      Serial.print("DRV fault codes: ");
      Serial.print(status1, BIN);
      Serial.print('\t');
      Serial.print(status2, BIN);
      Serial.print('\t');
      printDRVfaults(status1, status2);
      Serial.print('\n');
    	return true;
    }
    return false;
  #else
    return false;
  #endif
}
void printDRVfaults(uint16_t status1, uint16_t status2){
  if ((status1 >> 10) & 1){
    // Serial.print("fault\t");
  }
  if ((status1 >> 9) & 1){
    Serial.print("GVDD_UV\t");
  }
  if ((status1 >> 8) & 1){
    Serial.print("PVDD_UV\t");
  }
  if ((status1 >> 7) & 1){
    Serial.print("TEMP SD\t");
  }
  if ((status1 >> 6) & 1){
    Serial.print("TEMP WARN\t");
  }
  if ((status1 >> 5) & 1){
    Serial.print("FETHA_OC\t");
  }
  if ((status1 >> 4) & 1){
    Serial.print("FETLA_OC\t");
  }
  if ((status1 >> 3) & 1){
    Serial.print("FETHB_OC\t");
  }
  if ((status1 >> 2) & 1){
    Serial.print("FETLB_OC\t");
  }
  if ((status1 >> 1) & 1){
    Serial.print("FETHC_OC\t");
  }
  if ((status1 >> 0) & 1){
    Serial.print("FETLC_OC\t");
  }
  if ((status2 >> 7) & 1){
    Serial.print("GVDD_OV\t");
  }
  Serial.print("ID: ");
  Serial.println(status2 & 0xF);
}

// these just do the SPI stuff
uint16_t DRV_SPIread(uint8_t addr)
{
  delayMicroseconds(50);
  digitalWrite(DRV_CS, LOW);

  delayMicroseconds(50);
  uint8_t d = 1 << 7;
  d |= addr << 3;
  SPI.transfer(d);
  SPI.transfer(0);

  digitalWrite(DRV_CS, HIGH);
  delayMicroseconds(30);
  digitalWrite(DRV_CS, LOW);
  
  d = SPI.transfer(1<<7);
  uint16_t resp = (uint16_t) d << 8;
  resp |= SPI.transfer(0);

  digitalWrite(DRV_CS, HIGH);

  return resp & 0x7FF;
}
void DRV_SPIwrite(uint8_t addr, uint16_t data)
{
  digitalWriteFast(DRV_CS, LOW);

  delayMicroseconds(50);
  uint8_t d = data >> 8;
  d |= addr << 3;
  SPI.transfer(d);
  d = data & 0xFF;
  SPI.transfer(d);
    
  digitalWriteFast(DRV_CS, HIGH);
}

#endif