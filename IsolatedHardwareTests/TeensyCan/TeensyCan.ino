// -------------------------------------------------------------
// CANtest for Teensy 3.1
// by teachop
//
// This test is talking to a single other echo-node on the bus.
// 6 frames are transmitted and rx frames are counted.
// Tx and rx are done in a way to force some driver buffering.
// Serial is used to print the ongoing status.
//

#include <Metro.h>
#include <FlexCAN.h>

Metro sysTimer = Metro(1);// milliseconds
Metro CANtimer = Metro(10);

#define NSIL 5
#define STANDBY 12

int led = 13;
FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;
static uint8_t hex[17] = "hellohellohello.";

int txCount,rxCount;
unsigned int txTimer,rxTimer;


// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}


// -------------------------------------------------------------
void setup(void)
{
  CANbus.begin();

  Serial.begin(115200);

  pinMode(led, OUTPUT);
  digitalWrite(led, 1);

  delay(1000);
  Serial.println(F("Hello Teensy 3.1 CAN Test."));

  pinMode(NSIL, HIGH);
  pinMode(STANDBY, LOW);

  sysTimer.reset();

 
}


// -------------------------------------------------------------
void loop(void)
{
  if (CANtimer.check()){
    msg.id = 0x220;
    msg.len = 8;
    msg.buf[0] = 1;
    msg.buf[1] = 2;
    msg.buf[2] = 3;
    msg.buf[3] = 4;
    msg.buf[4] = 5;
    msg.buf[5] = 6;
    msg.buf[6] = 7;
    msg.buf[7] = 8;

    // msg.id = (0x3<<8) | (0xFF);
    // msg.ext = 1;
    // msg.len = 8;
    // msg.buf[0] = 0x3E;
    // msg.buf[1] = 0x03;
    // msg.buf[2] = 0x00;
    // msg.buf[3] = 0x00;
    // msg.buf[4] = 0x00;
    // msg.buf[5] = 0x00;
    // msg.buf[6] = 0x00;
    // msg.buf[7] = 0x00;

    CANbus.write(msg);
  }

  if (CANbus.available()) {
    int test = CANbus.read(rxmsg);
    Serial.write(sizeof(test));
    Serial.write("=");
    Serial.println(rxmsg.id);
  }

   
  // service software timers based on Metro tick
  /*if ( sysTimer.check() ) {
    if ( txTimer ) {
      --txTimer;
    }
    if ( rxTimer ) {
      --rxTimer;
    }
  }

  // if not time-delayed, read CAN messages and print 1st byte
  if ( !rxTimer ) {
    while ( CANbus.read(rxmsg) ) {
      //hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
      Serial.write(rxmsg.buf[2]);
      rxCount++;
    }
  }

  // insert a time delay between transmissions
  if ( !txTimer ) {
    // if frames were received, print the count
    if ( rxCount ) {
      Serial.write('=');
      Serial.print(rxmsg.id);
      rxCount = 0;
    }
    txTimer = 100;//milliseconds
    msg.len = 8;
    msg.id = 0x222;
    for( int idx=0; idx<8; ++idx ) {
      msg.buf[idx] = '0'+idx;
    }
    // send 6 at a time to force tx buffering
    txCount = 6;
    digitalWrite(led, 1);
    Serial.println(".");
    while ( txCount-- ) {
      CANbus.write(msg);
      msg.buf[0]++;
    }
    digitalWrite(led, 0);
    // time delay to force some rx data queue use
    rxTimer = 3;//milliseconds
  }*/

}
