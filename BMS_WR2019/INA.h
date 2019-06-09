#define CURRENT_CAL 0.9911 //H2 BMS #1
#define CURRENT_OFFSET 0.005;

#warning "Make sure CURRENT_CAL is set!"


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
  uint16_t confReg = ((0b0100) << 12) | // junk unused
                     ((0b000) << 9) |   // averages (1)
                     ((0b000) << 6) |   // voltage conversion time (140us)
                     ((0b101) << 3) |   // current conversion time (2.116ms)
                     ((0b111) << 0);    // continuous voltage and current measurements
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  // Wire.write(0b00000111);//64 averages, 1ms voltage sampling
  // Wire.write(0b00100111);//1ms current sampling, free running
  Wire.write((uint8_t)(confReg >> 8));
  Wire.write((uint8_t)(confReg & 0xFF));
  Wire.endTransmission();
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
