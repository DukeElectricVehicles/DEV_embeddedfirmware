#ifndef INA_H
#define INA_H

#include <i2c_t3.h>

#warning "Make sure INA ID is set!"
#ifndef INA_ID
  #error "PLEASE DEFINE INA ID"
#endif

static const double CURRENT_CAL_STORE [] = {1,    0.653643,   0.903559,      0}; // resistance in mOhms
static const double CURRENT_OFF_STORE [] = {0,    0.001551,   0.007352,      0}; // offset in A
static const double VOLTAGE_CAL_STORE [] = {1,    1.005079,   1.006994,      0}; // voltage scaling factor
static const double VOLTAGE_OFF_STORE [] = {0,    -0.025144,  -0.014234,      0}; // offset voltage in V

static const double CURRENT_CAL = CURRENT_CAL_STORE[INA_ID];
static const double CURRENT_OFFSET = CURRENT_OFF_STORE[INA_ID];
static const double VOLTAGE_CAL = VOLTAGE_CAL_STORE[INA_ID];
static const double VOLTAGE_OFFSET = VOLTAGE_OFF_STORE[INA_ID];

uint16_t INAreadReg(uint8_t reg);
void INAinit();
double INAvoltage_V();
double INAcurrent_A();

double InaVoltage_V = 0.0;
double InaCurrent_A = 0.0;
double InaPower_W = 0;
double InaEnergy_J = 0;

#ifdef OC_LIMIT
  extern void INAOC_isr();
#endif

void updateINA()
{
  static uint32_t lastInaMeasurement = micros();
  InaVoltage_V = INAvoltage_V();
  InaCurrent_A = INAcurrent_A();
  InaPower_W = InaVoltage_V * InaCurrent_A;

  #ifdef OC_LIMIT
    if (InaCurrent_A > OC_LIMIT) {
      INAOC_isr();
    }
  #endif
  
  uint32_t currentInaTime = micros();
  InaEnergy_J += InaPower_W * (currentInaTime - lastInaMeasurement) / 1000000.0;
  lastInaMeasurement = currentInaTime;  
}

double INAcurrent_A()
{
  int16_t raw = INAreadReg(0x01); //deliberate bad cast! the register is stored as two's complement
  return (float)raw / 400.0 * CURRENT_CAL + CURRENT_OFFSET; //2.5uV lsb and 1mOhm resistor
}

double INAvoltage_V()
{
  uint16_t raw = INAreadReg(0x02);
  return (float)raw / 800.0 * VOLTAGE_CAL + VOLTAGE_OFFSET; //multiply by 1.25mV LSB
}

void INAinit()
{
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.beginTransmission(0x40);
  Wire.write(0x00);//reg select = 0x00
  Wire.write(0b0111);//64 averages, 1ms voltage sampling
  Wire.write(0b100111);//1ms current sampling, free running
  Wire.endTransmission();

  #ifdef OC_LIMIT
    Wire.beginTransmission(0x40);
    Wire.write(0x06); // alert selectior
    Wire.write(0x80); // shunt over voltage
    Wire.write(0x00); // no other alerts
    Wire.endTransmission();
    uint16_t threshold = (OC_LIMIT - CURRENT_OFFSET) * 400 / CURRENT_CAL;
    Wire.beginTransmission(0x40);
    Wire.write(0x07); // alert threshold
    Wire.write(threshold >> 8);
    Wire.write(threshold & 0xFF);
    Wire.endTransmission();
    pinMode(ALERT_PIN, INPUT_PULLUP);
    attachInterrupt(ALERT_PIN, INAOC_isr, FALLING);
  #endif
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