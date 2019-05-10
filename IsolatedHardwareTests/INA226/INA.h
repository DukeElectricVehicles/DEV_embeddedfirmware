#ifndef INA_H
#define INA_H

#define CURRENT_CAL 0.9795 //H2 BMS #1
#define CURRENT_OFFSET 0.002;

#warning "Make sure CURRENT_CAL is set!"

static uint16_t CURRENT_DID_STORE [] = {0, 0, 0};
static double CURRENT_CAL_STORE [] = {0, 0, 0};
static double CURRENT_OFF_STORE [] = {0, 0, 0};

static uint16_t DIE_ID = 0;
static double CURRENT_CAL = 0;
static double CURRENT_OFFSET = 0;

uint16_t INAreadReg(uint8_t reg);
void INAinit();
double INAvoltage();
double INAcurrent();

double INAcurrent()
{
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return (float)raw / 400.0 * CURRENT_CAL + CURRENT_OFFSET; //2.5uV lsb and 1mOhm resistor
}

double INAvoltage()
{
  uint16_t raw = INAreadReg(0x02);
  return (float)raw / 800.0; //multiply by 1.25mV LSB
}

void INAinit()
{
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  Wire.write(0b0111);//64 averages, 1ms voltage sampling
  Wire.write(0b100111);//1ms current sampling, free running
  Wire.endTransmission();
  DIE_ID = Wire.read(0xFF);
  for (uint8_t i = 0; i<(sizeof(CURRENT_DID_STORE)/sizeof(CURRENT_DID_STORE[0])); i++){
    if (DIE_ID == CURRENT_DID_STORE[i]){
      CURRENT_CAL = CURRENT_CAL_STORE[i];
      CURRENT_OFFSET = CURRENT_OFF_STORE[i];
      return;
    }
  }
  Serial.println("INA chip not calibrated!!!");
  CURRENT_CAL = 2048;
  CURRENT_OFFSET = 0;
}

uint16_t INAreadReg(uint8_t reg)
{
  Wire.beginTransmission(0x40);
  Wire.write(reg);//read from the bus voltage
  Wire.endTransmission();

  Wire.requestFrom(0x40, 2);

  delayMicroseconds(100);
  if (Wire.available() < 2)
    return 0;

  uint16_t resp = (uint16_t)Wire.read() << 8;
  resp |= Wire.read();

  return resp;
}

#endif