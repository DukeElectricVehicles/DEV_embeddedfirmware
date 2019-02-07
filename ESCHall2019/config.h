#define INHA 9
#define INLA 6
#define INHB 20 
#define INLB 10
#define INHC 23
#define INLC 22

#ifdef KINETISL // teensy LC doesn't have interrupt on pin 1
  #error // haven't decided what pins to use for hall sensor replacements
#else
  #define HALLA 0
  #define HALLB 1
  #define HALLC 2
#endif

#define THROTTLE 15

#define DRV_EN_GATE 7
#define DRV_CS 21
#define DRV_MOSI 11
#define DRV_MISO 12
#define DRV_CLK 13

#define ISENSE1 17
#define ISENSE2 16

#define LED1 5
#define LED2 8

#define MAX_THROTTLE  1000
#define MIN_THROTTLE  300

volatile uint8_t reg = 0;
volatile uint16_t BMSThrottle = 0;
volatile uint32_t BMSMillis = 0;

uint16_t SPIread(uint8_t addr);
void SPIwrite(uint8_t addr, uint16_t data);
void setupPins();
float getThrottle();
void receiveEvent(size_t count);
void requestEvent(void);

SPISettings settingsDRV(2000000, MSBFIRST, SPI_MODE1);

void receiveEvent(size_t count)
{
  reg = Wire.readByte();

  if(reg == 0x40)//set throttle
  {
    BMSThrottle = Wire.readByte() << 8;
    BMSThrottle |= Wire.readByte();
    
    BMSMillis = millis();
  }
}


void requestEvent(void)
{
  uint8_t data = 0;
  
  if(reg == 0x12)
    data = 0x34;
    
  Wire.write(&data, 1);
}

float getThrottle()
{
  uint16_t rawThrottle = analogRead(THROTTLE);
  float throttle = (rawThrottle - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE);
  
  if(throttle > 1)
    throttle = 1;
  if (throttle < 0)
    throttle = 0;

  return throttle;
}

void setupPins()
{
  Serial.println("Setting up pins");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  //pinMode(FAULT, INPUT);
  
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);
  
  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);
  
  pinMode(THROTTLE, INPUT);

  pinMode(DRV_EN_GATE, OUTPUT);

  pinMode(DRV_CLK, OUTPUT);
  pinMode(DRV_MOSI, OUTPUT);
  pinMode(DRV_MISO, INPUT);
  pinMode(DRV_CS, OUTPUT);
  digitalWriteFast(DRV_CS, HIGH);
  
  pinMode(ISENSE1, INPUT);
  pinMode(ISENSE2, INPUT);

  // change the analog write frequency to 8 kHz
  analogWriteFrequency(INHA, 8000);
  analogWriteFrequency(INHB, 8000);
  analogWriteFrequency(INHC, 8000);
  analogWriteResolution(12); // write from 0 to 2^12 = 4095

  Serial.begin(115200);

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
  while((SPIread(0x00) != 0x00) || (SPIread(0x01) != 0x01))
  {
    Serial.println("DRV init fail");
    SPIwrite(0x02, 0x00);
    for(uint32_t i = 0; i < 4; i++)
    {
      Serial.print("0x");
      Serial.println(SPIread(i),HEX);
      Serial.print("0x");
      Serial.println(SPIread(i),HEX);
    }
    
    digitalWriteFast(DRV_EN_GATE, LOW);
    delay(10);
    digitalWriteFast(DRV_EN_GATE, HIGH);
    delay(500);
  }

  SPI.endTransaction();
  Serial.println("Finished DRV SPI setup");
  SPI.end();
}

uint16_t SPIread(uint8_t addr)
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

void SPIwrite(uint8_t addr, uint16_t data)
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
