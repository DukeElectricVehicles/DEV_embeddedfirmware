#ifndef CONFIG_H
#define CONFIG_H

#define INHA 23
#define INLA 4
#define INHB 22 
#define INLB 5
#define INHC 21
#define INLC 6

#ifdef KINETISL // teensy LC doesn't have interrupt on pin 1
  #define HALL1 20
#else
  #define HALL1 1
#endif
#define HALL2 2
#define HALL3 3

#define THROTTLE 15

#define DRV_EN_GATE 7
#define DRV_CS 10
#define DRV_MOSI 11
#define DRV_MISO 12
#define DRV_CLK 13

#define ISENSE1 17
#define ISENSE2 16

#define LED1 9
#define LED2 8


#define MAX_THROTTLE  1000
#define MIN_THROTTLE  300

volatile uint8_t reg = 0;
volatile uint16_t BMSThrottle = 0;
volatile uint32_t BMSMillis = 0;

void setupPins();
float getThrottle();
void receiveEvent(size_t count);
void requestEvent(void);

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
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  //pinMode(FAULT, INPUT);
  
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);
  
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  
  pinMode(THROTTLE, INPUT);

  pinMode(ISENSE1, INPUT);
  pinMode(ISENSE2, INPUT);

  // change the analog write frequency to 8 kHz
  analogWriteFrequency(INHA, 8000);
  analogWriteFrequency(INHB, 8000);
  analogWriteFrequency(INHC, 8000);
  analogWriteResolution(12); // write from 0 to 2^12 = 4095

  Serial.begin(115200);
}

#endif
