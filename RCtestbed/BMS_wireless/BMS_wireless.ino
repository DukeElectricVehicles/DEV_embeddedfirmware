#define useNRF
#define useCAN
#define useGPSx
#define useIMUx
#define useMAGx
#define useHall
#define useBrake
#define DEV

#include "config.h"
#include <Metro.h>
#include <FlexCAN.h>
#include "RF24.h"
#include "vESC_datatypes.h"
#include "vescSteering.h"

#ifdef useIMU
  #include <Wire.h>
  #include "MPU6050_tockn_DEV.h"
  MPU6050 imu(Wire);
#endif

#ifdef useMAG
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_HMC5883_U.h>
  Adafruit_HMC5883_Unified mag(12345); // sensor ID (dummy)
  sensors_event_t mag_data;
#endif

#ifdef useBrake
  #define BrakeServoPin 5
  #include <Servo.h>
  Servo brakeServo;
  Metro servoUpdateTimer(10);
#endif

RF24 radio(7,8);

uint8_t recvaddress[6] = "car00";
uint8_t sendaddress[6] = "contr";

int8_t readBuffer[6];
driveCommands_t mostRecentCommands;

FlexCAN CANbus(500000);
static CAN_message_t msg,rxmsg;

int txCount,rxCount;
unsigned int txTimer,rxTimer;

Metro sysTimer = Metro(1);// milliseconds
Metro CANtimer = Metro(10);
Metro wirelessTimeout = Metro(100);

void readRadio(void);
void sendThrottle(void);
void sendSteering(void);
// void sendTachReq(void);
void sendIMU(void);
void sendMAG(void);

// -------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("DEV wireless car testbed"));
  Serial.println(F("Initializing..."));
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.openWritingPipe(sendaddress);
  radio.openReadingPipe(1, recvaddress);
  radio.printDetails();
  radio.startListening();

  CANbus.begin();

  #ifdef useIMU
    Wire.begin();
    imu.begin();
    imu.calcGyroOffsets(true);
  #endif
  #ifdef useMAG
  if(!mag.begin())
  {
    Serial.println("No HMC5883 detected ...");
    while(1);
  }
  #endif

  #ifdef useBrake
    brakeServo.attach(BrakeServoPin);
  #endif

  // pinMode(NSIL, HIGH);
  // pinMode(STANDBY, LOW);

  pinMode(2, OUTPUT);

  initSteering(&rxmsg);

  sysTimer.reset();

  Serial.println(F("setup complete"));
}

// -------------------------------------------------------------
void loop(void)
{
  digitalWrite(2, mostRecentCommands.throttle>0); // RELAY
  if (CANtimer.check()){
    updateSteering();
    sendThrottle();
    // sendTachReq();
    sendIMU();
    sendMAG();
  }

  if (CANbus.available()) {
    int test = CANbus.read(rxmsg);
    Serial.write(sizeof(test));
    Serial.write("=");
    Serial.println(rxmsg.id, HEX);
    readVESCCAN();
  }

  if (wirelessTimeout.check()){
    mostRecentCommands.LSteeringMotor = 0;
    mostRecentCommands.RSteeringMotor = 0;
    mostRecentCommands.throttle = 0;
    mostRecentCommands.brake = 64;
  }

  #ifdef useIMU
    imu.update();
  #endif
  #ifdef useMAG
    mag.getEvent(&mag_data);
  #endif

  #ifdef useBrake
    if (servoUpdateTimer.check()){
      brakeServo.write(map(mostRecentCommands.brake,0,64,0,60));
    }
  #endif

  readRadio();
}

void readRadio() {
  if (radio.available()) {
    radio.read(&readBuffer, sizeof(readBuffer));
    Serial.print("Got ");
    for (int i = 0; i<6; i++){
      Serial.print('\t');
      Serial.print(readBuffer[i]);
    }
    mostRecentCommands.LSteeringMotor = STEER_LPF*mostRecentCommands.LSteeringMotor + 
                                    (1-STEER_LPF)*((int32_t)readBuffer[4]);//*600;
    // mostRecentCommands.RSteeringMotor = ((int32_t)readBuffer[4]);//*600;
    mostRecentCommands.throttle = max(0, readBuffer[2]) << 6; // scale from 64 to 4096
    mostRecentCommands.brake = -min(0, readBuffer[2]);

    if (!readBuffer[3]){
      homeSteering();
    }

    Serial.print('\t');
    Serial.print(mostRecentCommands.LSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.RSteeringMotor);
    Serial.print('\t');
    Serial.print(mostRecentCommands.throttle);
    Serial.println();
    wirelessTimeout.reset();
  }
}
void sendThrottle(void){
  // main motor command
  msg.id = MmotorCAN & 0xFF;
  msg.ext = 0;
  msg.len = 8;
  msg.buf[0] = (mostRecentCommands.throttle >> 8) & 0xFF;
  msg.buf[1] = (mostRecentCommands.throttle >> 0) & 0xFF;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CANbus.write(msg);
}
// void sendTachReq(void){
//   // left steering get tach
//   msg.id = (CAN_PACKET_GET_TACH<<8) | (LmotorCAN & 0xFF);
//   msg.ext = 1;
//   msg.len = 8;
//   msg.buf[0] = 0x00;
//   msg.buf[1] = 0x00;
//   msg.buf[2] = 0x00;
//   msg.buf[3] = 0x00;
//   msg.buf[4] = 0x00;
//   msg.buf[5] = 0x00;
//   msg.buf[6] = 0x00;
//   msg.buf[7] = 0x00;
//   CANbus.write(msg);
//   // right steering get tach
//   msg.id = (CAN_PACKET_GET_TACH<<8) | (RmotorCAN & 0xFF);
//   msg.ext = 1;
//   msg.len = 8;
//   msg.buf[0] = 0x00;
//   msg.buf[1] = 0x00;
//   msg.buf[2] = 0x00;
//   msg.buf[3] = 0x00;
//   msg.buf[4] = 0x00;
//   msg.buf[5] = 0x00;
//   msg.buf[6] = 0x00;
//   msg.buf[7] = 0x00;
//   CANbus.write(msg);
// }
void sendIMU(void){
  #ifdef useIMU
    Serial.print("accX : ");Serial.print(imu.getAccX());
    Serial.print("\taccY : ");Serial.print(imu.getAccY());
    Serial.print("\taccZ : ");Serial.print(imu.getAccZ());
    Serial.print("\tangleX : ");Serial.print(imu.getAngleX());
    Serial.print("\tangleY : ");Serial.print(imu.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(imu.getAngleZ());
    Serial.println("=======================================================\n");
    // accelerometer
    msg.id = accCAN & 0xFF;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = (((int16_t)(imu.getAccX()*1e3)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(imu.getAccX()*1e3)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(imu.getAccY()*1e3)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(imu.getAccY()*1e3)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(imu.getAccZ()*1e3)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(imu.getAccZ()*1e3)) >> 0) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CANbus.write(msg);
    // gyro
    msg.id = gyroCAN & 0xFF;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = (((int16_t)(imu.getAngleX()*1e2)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(imu.getAngleX()*1e2)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(imu.getAngleY()*1e2)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(imu.getAngleY()*1e2)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(imu.getAngleZ()*1e2)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(imu.getAngleZ()*1e2)) >> 0) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CANbus.write(msg);
  #endif
}
void sendMAG(void){
  #ifdef useMAG
    Serial.print("magX : ");Serial.print(mag_data.magnetic.x);
    Serial.print("\tmagY : ");Serial.print(mag_data.magnetic.y);
    Serial.print("\tmagZ : ");Serial.print(mag_data.magnetic.z);
    Serial.print('\n');
    Serial.println("=======================================================\n");
    // magnetometer
    msg.id = magCAN & 0xFF;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = (((int16_t)(mag_data.magnetic.x*1e3)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(mag_data.magnetic.x*1e3)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(mag_data.magnetic.y*1e3)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(mag_data.magnetic.y*1e3)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(mag_data.magnetic.z*1e3)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(mag_data.magnetic.z*1e3)) >> 0) & 0xFF;
    msg.buf[6] = 0x00; // send compass heading here
    msg.buf[7] = 0x00;
    CANbus.write(msg);
  #endif
}