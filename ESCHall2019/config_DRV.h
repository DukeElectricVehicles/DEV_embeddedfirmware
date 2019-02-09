#ifndef CONFIG_DRV_H
#define CONFIG_DRV_H

#ifndef DRV8301
	#error "only DRV8301 supported, please pound define DRV8301"
#endif

#include <SPI.h>
#include "config.h"

SPISettings settingsDRV(2000000, MSBFIRST, SPI_MODE1);

// function declarations
void setupDRV();
bool checkDRVfaults();
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


  digitalWriteFast(DRV_EN_GATE, LOW);
  delay(100);
  digitalWriteFast(DRV_EN_GATE, HIGH);
  delay(100);

  SPI.begin();
  Serial.println("Setting up DRV SPI");
  SPI.beginTransaction(settingsDRV);

  // Refer to section 7.6 of the DRV8301 datasheet
  // 0x00 - Status register 1: any faults show as 1's
  // 0x01 - Status register 2: device ID (default 0x1)
  while((DRV_SPIread(0x00) != 0x00) || (DRV_SPIread(0x01) != 0x01))
  {
    Serial.println("DRV init fail");
    DRV_SPIwrite(0x02, 0x00);
    for(uint32_t i = 0; i < 4; i++)
    {
      Serial.print("0x");
      Serial.println(DRV_SPIread(i),HEX);
      Serial.print("0x");
      Serial.println(DRV_SPIread(i),HEX);
    }
    
    digitalWriteFast(DRV_EN_GATE, LOW);
    delay(10);
    digitalWriteFast(DRV_EN_GATE, HIGH);
    delay(500);
  }

  SPI.endTransaction();
  Serial.println("Finished DRV SPI setup");
  // SPI.end();
}

// returns true if there is a fault
bool checkDRVfaults(){
	return (DRV_SPIread(0x00) != 0x00) || (DRV_SPIread(0x01) != 0x01);
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