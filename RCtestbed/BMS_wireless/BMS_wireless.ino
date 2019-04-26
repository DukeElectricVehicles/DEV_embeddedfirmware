#define useNRF
#define useCAN
#define useGPSx
#define useIMUx
#define useMAGx
#define useHall
#define useBrake
#define useRTK
#define usePathFollow
#define useSpeedControlx
#define useDistanceControlx
#define useAngleControlx
#define useVESCthrottle
#define DEV

// #include <i2c_t3.h>
#include "config.h"
#include <Metro.h>
#include <FlexCAN.h>
#include "RF24.h"
#include "vESC_datatypes.h"
#include "vescSteering.h"
#include "PID_gerry.h"
#include "vescThrottle.h"

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
  #define BrakeServoPin 21
  #include <Servo.h>
  Servo brakeServo;
  Metro servoUpdateTimer(10);
#endif
#ifdef useHall
  #define HALL 23
  #define WHEEL_CIRC 1.492
  #define WHEEL_TICKS 8
  #define TICK_DIST (WHEEL_CIRC / WHEEL_TICKS)
  uint32_t tickTimes[WHEEL_TICKS];
  uint8_t tickPos = 0;
  uint32_t avgHalldT = 1000000;
  uint32_t lastHallPulse = 0;
  uint32_t odometer_ticks = 0;
  double odometer_m = 0.0;
  double currentSpeed_m_per_s = 0.0;
#endif
#ifdef useRTK
  #include "libsbp/common.h"
  #include "libsbp/sbp.h"
  #include "libsbp/system.h"
  #include "libsbp/navigation.h"

  static sbp_state_t sbp_state;

  s32 RTK_readSerial(u8 *buff, u32 n, void *context);
  void RTK_heartbeat_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void RTK_posLLH_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void RTK_velNED_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void RTK_printmsg(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  static sbp_msg_callbacks_node_t
    RTK_heartbeat_cbnode,
    RTK_posLLH_cbnode,
    RTK_velNED_cbnode;

  Metro RTKaliveTimer(1250);
  Metro GPSvalidTimer(250);
  bool RTKisAlive, GPSisValid;

  msg_pos_llh_t curPosLLH_RTK;
  msg_vel_ned_t curVelNED_RTK;
#endif
#ifdef usePathFollow
  #include "pathFollowRTK.h"
#endif

#ifdef useSpeedControl
  #ifndef useHall
    #error "cannot do speed control without hall sensor"
  #endif
  #ifdef useDistanceControl
    #error "cannot use distance and speed control at the same time"
  #endif
  float speedError_PID_m_per_s = 0, setpointSpeed_m_per_s = 0;
  float throttleControl_PID = 0;
  PID PIDthrottle(.15, 1, 0, &speedError_PID_m_per_s, &throttleControl_PID);
  float brakeControl_PID = 0;
  PID PIDbrake(.25, 0, .2, &speedError_PID_m_per_s, &brakeControl_PID);
  Metro speedPIDtimer = Metro(50);
  bool speedChanged = false;
#endif
#ifdef useDistanceControl
  #ifndef useHall
    #error "cannot do distance control without hall sensor"
  #endif
  float distError_PID_m = 0, setpointDist_m = 0;
  float throttleControl_PID = 0;
  float brakeControl_PID = 0;
  #ifdef useVESCthrottle
  PID PIDthrottle(.2, 0, 0, &distError_PID_m, &throttleControl_PID);
  PID PIDbrake(.5, 0, 10, &distError_PID_m, &brakeControl_PID);
  #else
  PID PIDthrottle(.2, .005, .05, &distError_PID_m, &throttleControl_PID);
  PID PIDbrake(.5, 0, 10, &distError_PID_m, &brakeControl_PID);
  #endif
  Metro distPIDtimer = Metro(50);
  bool distChanged = false;
#endif
#ifdef useAngleControl
  float LPFAngle_rad = 0, setpointAngle_rad = 0;
  float errorAngle_rad = 0, angleControl_PID = 0;
  PID PIDangle(1, 0, 1, &errorAngle_rad, &angleControl_PID);
  Metro anglePIDtimer = Metro(10);
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
Metro debugPrintTimer = Metro(50);

bool wirelessTimedOut = true;
bool manualPrint = false;

float storedAngles[AUTOSTORESIZE]; // 1 angle for each meter, stored for playback
uint16_t lastSavedBufferInd = 0;
bool isPlayingBack = false;
uint32_t leftButtonDoubleClickTimer;

void readRadio(void);
void sendThrottle(void);
void sendSteering(void);
// void sendTachReq(void);
void sendIMU(void);
void sendMAG(void);

bool isPathComplete = false;

// -------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println(F("DEV wireless car testbed"));
  Serial.println(F("Initializing..."));

  // Wire.setDefaultTimeout(1000);
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicAck();
  radio.openWritingPipe(sendaddress);
  radio.openReadingPipe(1, recvaddress);
  radio.printDetails();
  radio.startListening();

  memset(storedAngles, 0, sizeof(storedAngles));

  CANbus.begin();

  #if defined(useIMU) || defined(useMAG)
  Wire.begin();
  #endif
  #ifdef useIMU
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
  #ifdef useHall
    pinMode(HALL, INPUT_PULLUP);
    attachInterrupt(HALL, countHallPulse, FALLING);
  #endif
  #ifdef useSpeedControl
    PIDthrottle.setLim(0, 1);
    PIDthrottle.setPLim(0, 1);
    PIDthrottle.setILim(0, .5);
    PIDthrottle.setDLim(-1, 1);
    PIDthrottle.setSlewLim(-1e9, 4);
    PIDbrake.setLim(-1, 0);
    PIDbrake.setPLim(-1, 0);
    PIDbrake.setILim(-.2, 0);
    PIDbrake.setDLim(-1, 1);
    PIDbrake.setSlewLim(-1e9, 1e9);
  #endif
  #ifdef useDistanceControl
    PIDthrottle.setLim(0, 1);
    PIDthrottle.setPLim(0, 1);
    PIDthrottle.setILim(0, 1);
    PIDthrottle.setDLim(-1, 1);
    PIDthrottle.setSlewLim(-1e9, 4);
    PIDbrake.setLim(-1, 0);
    PIDbrake.setPLim(-1, 1);
    PIDbrake.setILim(-.2, 0);
    PIDbrake.setDLim(-2, 1);
    PIDbrake.setSlewLim(-1e9, 1e9);
  #endif
  #ifdef useAngleControl
    PIDangle.setLim(-1, 1);
    PIDangle.setPLim(-1, 1);
    PIDangle.setILim(-1, 1);
    PIDangle.setDLim(-1, 1);
    PIDangle.setSlewLim(-4, 4);
  #endif

  #ifdef useRTK
    Serial1.begin(230400);

    sbp_state_init(&sbp_state);
    sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &RTK_heartbeat_cb, NULL, &RTK_heartbeat_cbnode);
    sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &RTK_posLLH_cb, NULL, &RTK_posLLH_cbnode);
    sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &RTK_velNED_cb, NULL, &RTK_velNED_cbnode);
  #endif

  #ifdef usePathFollow
    initPathFollow();
  #endif

  isPathComplete = false;

  // pinMode(NSIL, HIGH);
  // pinMode(STANDBY, LOW);

  pinMode(2, OUTPUT);

  initSteering(&rxmsg);
  #ifdef useVESCthrottle
  initThrottle(&rxmsg);
  #endif

  sysTimer.reset();

  Serial.println(F("setup complete"));
}

// -------------------------------------------------------------
void loop(void)
{
  if (CANtimer.check()){
    updateSteering();
    #ifdef useVESCthrottle
      #ifdef useSpeedControl
        sendThrottleVel(max(0,mostRecentCommands.throttle));
      #else
        updateThrottle();
      #endif
    #else
      sendThrottle();
    #endif
    // sendTachReq();
    sendIMU();
    sendMAG();
  }

  if (CANbus.available()) {
    int test = CANbus.read(rxmsg);
    // Serial.write(sizeof(test));
    // Serial.write("=");
    // Serial.println(rxmsg.id, HEX);
    readVESCCAN();
  }

  if (wirelessTimeout.check()){
    // mostRecentCommands.LSteeringMotor = 0;
    // mostRecentCommands.RSteeringMotor = 0;
    mostRecentCommands.throttle = 0;
    mostRecentCommands.brake = 64;
    wirelessTimedOut = true;
  }
  if (!wirelessTimedOut) {
    #ifdef useSpeedControl
      #ifdef useVESCthrottle
        mostRecentCommands.throttle = 1600*setpointSpeed_m_per_s;
        mostRecentCommands.brake = constrain((currentSpeed_m_per_s - 1.1*setpointSpeed_m_per_s - 0.5)*64, 0, 64);
      #else
        if (speedPIDtimer.check() || speedChanged){
          speedChanged = false;
          speedError_PID_m_per_s = (setpointSpeed_m_per_s - currentSpeed_m_per_s);
          PIDthrottle.update();
          PIDbrake.update();
          mostRecentCommands.throttle = pow(throttleControl_PID,.7) * 4096; // already constrained by PID lims
          mostRecentCommands.brake = -brakeControl_PID * 64;
        }
      #endif
    #elif defined (useDistanceControl)
      if (distPIDtimer.check() || distChanged){
        distChanged = false;
        distError_PID_m = (setpointDist_m - odometer_m);
        PIDthrottle.update();
        PIDbrake.update();
        mostRecentCommands.throttle = pow(throttleControl_PID,.7) * 4096; // already constrained by PID lims
        mostRecentCommands.brake = -brakeControl_PID * 64;
      }
    #endif
    #ifdef useAngleControl
      if (anglePIDtimer.check()){
        LPFAngle_rad = ANGLE_LPF*LPFAngle_rad + (1-ANGLE_LPF)*atan2(mag_data.magnetic.y, -mag_data.magnetic.z);
        if (isPlayingBack){
          errorAngle_rad = LPFAngle_rad - setpointAngle_rad;
          errorAngle_rad = fmod(errorAngle_rad - 3.1415, 6.2830) + 3.1415; // difference in mod 2pi
          PIDangle.update();
          #ifdef useHall
            // dtheta/dt = v sin(phi)
            // phi = dtheta/dt / v
            if (currentSpeed_m_per_s <= 0){
              mostRecentCommands.LSteeringMotor = STEER_LPF*mostRecentCommands.LSteeringMotor + 
                                                  (1-STEER_LPF)*constrain(angleControl_PID*100 / .15, -64, 64);
            } else {
              mostRecentCommands.LSteeringMotor = STEER_LPF*mostRecentCommands.LSteeringMotor + 
                                                  (1-STEER_LPF)*constrain(angleControl_PID*100 / currentSpeed_m_per_s*10, -64, 64);
            }
          #else
            mostRecentCommands.LSteeringMotor = angleControl_PID*100;
          #endif
        }
      }
    #endif
    #ifdef usePathFollow
      if (isPlayingBack) {
        mostRecentCommands.LSteeringMotor = constrain(-getWaypointDir() * 64,-64,64);
      }
    #endif
  }
  digitalWrite(2, mostRecentCommands.brake>-40); // RELAY, usually on for faster response time


  #ifdef useRTK // poll 1/6
    sbp_process(&sbp_state, &RTK_readSerial);
  #endif

  if (debugPrintTimer.check() || manualPrint){
    manualPrint = false;
    #ifdef usePathFollow
      getWaypointDir();
    #endif
    String outputStr = 
      String(odometer_m)
      + "\t"
      + "\t"
      #ifdef useSpeedControl
      + String(setpointSpeed_m_per_s)
      + "\t"
      + String(currentSpeed_m_per_s)
      + "\t"
      + String(speedError_PID_m_per_s)
      + "\t"
      + String(throttleControl_PID)
      + "\t"
      + String(brakeControl_PID)
      + "\t"
      + "\t"
      #elif defined(useDistanceControl)
      + String(setpointDist_m)
      + "\t"
      + String(odometer_m)
      + "\t"
      + String(distError_PID_m)
      + "\t"
      + String(throttleControl_PID)
      + "\t"
      + String(brakeControl_PID)
      + "\t"
      + "\t"
      #endif
      #ifdef useAngleControl
      + String(setpointAngle_rad)
      + "\t"
      + String(LPFAngle_rad)
      + "\t"
      + String(angleControl_PID, 4)
      + "\t"
      + String(mostRecentCommands.LSteeringMotor)
      + "\t"
      + "\t"
      #endif
      + String(mostRecentCommands.LSteeringMotor)
      + "\t"
      + String(mostRecentCommands.throttle)
      + "\t"
      + String(mostRecentCommands.brake)
      + "\t"
      + "\t"
      #ifdef useMAG
      + String(mag_data.magnetic.y)
      + "\t"
      + "\t"
      #endif
      #ifdef useRTK
      + String(fmod(curPosLLH_RTK.lat*1e6,1000))
      + "\t"
      + String(fmod(curPosLLH_RTK.lon*1e6,1000))
      + "\t"
      + String(fmod(waypoints_RTK[curWaypointInd][0]*1e6,1000))
      + "\t"
      + String(fmod(waypoints_RTK[curWaypointInd][1]*1e6,1000))
      + "\t"
      + "\t"
      + String(fmod(delLat*1e6,1000))
      + "\t"
      + String(fmod(delLon*1e6,1000))
      + "\t"
      + String(fmod(setLat*1e6,1000))
      + "\t"
      + String(fmod(setLon*1e6,1000))
      + "\t"
      #endif
      #ifdef usePathFollow
      + String(nLPF)
      + "\t"
      + String(eLPF)
      + "\t"
      + "\t"
      + String(curHeading)
      + "\t"
      + String(desHeading)
      + "\t"
      + String(isPathComplete)
      + "\t"
      + String(getProgress())
      + "\t"
      + "\t"
      #endif
      ;
    Serial.println(outputStr);
    Serial2.println(outputStr);//bluetooth
  }

  #ifdef useRTK // poll 2/6
    sbp_process(&sbp_state, &RTK_readSerial);
  #endif

  if (Serial.available()){
    switch(Serial.read()){
      case 'r':
        lastSavedBufferInd = 0;
        break;
    }
  }

  #ifdef useRTK // poll 3/6
    sbp_process(&sbp_state, &RTK_readSerial);
  #endif

  #ifdef useIMU
    imu.update();
  #endif
  #ifdef useMAG
    mag.getEvent(&mag_data);
  #endif

  #ifdef useRTK // poll 4/6
    sbp_process(&sbp_state, &RTK_readSerial);
  #endif

  #ifdef useBrake
    if (servoUpdateTimer.check()){
      brakeServo.write(map(mostRecentCommands.brake,0,64,0,60));
    }
  #endif
  #ifdef useHall
    updateHallSpeed();
  #endif

  #ifdef useRTK // poll 5/6
    sbp_process(&sbp_state, &RTK_readSerial);
  #endif

  readRadio();

  #ifdef useRTK // poll 6/6
    sbp_process(&sbp_state, &RTK_readSerial);
    if(RTKaliveTimer.check()){
      Serial.println("Missed RTK heartbeat signal!");
      RTKisAlive = false;
    }
    if(GPSvalidTimer.check()){
      Serial.println("GPS position lost");
      GPSisValid = false;
    }
  #endif
}

void readRadio() {
  #ifdef useDistanceControl
  static uint32_t lastUpdateTime = 0;
  #endif
  if (radio.available()) {
    radio.read(&readBuffer, sizeof(readBuffer));
    // Serial.print("Got ");
    // for (int i = 0; i<6; i++){
    //   Serial.print('\t');
    //   Serial.print(readBuffer[i]);
    // }

    // #ifdef useAngleControl
    //   setpointAngle_rad = readBuffer[4] / 127.0 * 3.1415;
    // #endif
    if (!isPlayingBack){
        mostRecentCommands.LSteeringMotor = STEER_LPF*mostRecentCommands.LSteeringMotor + 
                                          (1-STEER_LPF)*((int32_t)readBuffer[4]);//*600;
          // mostRecentCommands.RSteeringMotor = ((int32_t)readBuffer[4]);//*600;
    }

    #ifdef useSpeedControl
      setpointSpeed_m_per_s = readBuffer[2] / 64.0 * 5.0; // 5m/s top speed
    #elif defined(useDistanceControl)
      if (lastUpdateTime == 0)
        lastUpdateTime = millis();
      uint32_t tNow = millis();
      setpointDist_m += readBuffer[2] / 64.0 * 3.0 * ((tNow - lastUpdateTime) / 1000.0);
      lastUpdateTime = tNow;
    #else
      mostRecentCommands.throttle = max(0, readBuffer[2]) << 6; // scale from 64 to 4096
      mostRecentCommands.brake = -min(0, readBuffer[2]);
    #endif

    if (!readBuffer[0]){ // right button
      homeSteering();
    }
    if (!readBuffer[3]){ // left button
      if ((millis()-leftButtonDoubleClickTimer)<500){
        odometer_ticks = 0;
        #ifdef useDistanceControl
        setpointDist_m = 0;
        #endif
        #ifdef usePathFollow
        isPathComplete = false;
        curWaypointInd = 0;
        #endif
      }
      isPlayingBack = true;
      #ifdef useAngleControl
        setpointAngle_rad = storedAngles[(uint16_t)odometer_m];
      #endif
    } else {
      if (isPlayingBack){
        leftButtonDoubleClickTimer = millis();
      }
      isPlayingBack = false;
      #ifdef useAngleControl
        if (odometer_m < AUTOSTORESIZE){
          uint16_t loc = (uint16_t) odometer_m;
          if (lastSavedBufferInd <= loc){ // LTE to get the last entry in the meter
            storedAngles[loc] = LPFAngle_rad;
            lastSavedBufferInd = loc;
          }
        }
      #endif
    }
    wirelessTimeout.reset();
    wirelessTimedOut = false;
  }
}
void updateHallSpeed()
{
  currentSpeed_m_per_s = 1000000.0 / avgHalldT * WHEEL_CIRC; 
  if(micros() - lastHallPulse > 250000)
    currentSpeed_m_per_s = 0;
  
  odometer_m = odometer_ticks * TICK_DIST;

}

void countHallPulse() {

  // uint16_t analogVal = analogRead(HALL);
  // if ((analogVal < 55) || (analogVal > 70)){
  //   Serial.println(analogVal);
  //   return;
  // }
  // uint16_t analogVal2 = analogRead(HALL);

  uint32_t current = micros();

  uint32_t prevTime = tickTimes[tickPos];// time 1 rev ago. fixed 7/5/2018 :(

  tickTimes[tickPos++] = current;
  tickPos %= WHEEL_TICKS;

  avgHalldT = current - prevTime;

  odometer_ticks++;
  
  lastHallPulse = current;

  updateHallSpeed();

  #ifdef useSpeedControl
  speedChanged = true;
  #endif
  #ifdef useDistanceControl
  distChanged = true;
  #endif

  // Serial.print("\t\t\t");
  // Serial.print(analogVal);
  // Serial.print('\t');
  // Serial.print(analogVal2);
  // Serial.print('\t');
  // Serial.print(current-tickTimes[(tickPos-2) % WHEEL_TICKS]);
  // Serial.print('\n');
  // digitalWrite(LED1, (odometer_ticks) & 1);
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
    msg.buf[0] = (((int16_t)(imu.getGyroX()*1e2)) >> 8) & 0xFF;
    msg.buf[1] = (((int16_t)(imu.getGyroX()*1e2)) >> 0) & 0xFF;
    msg.buf[2] = (((int16_t)(imu.getGyroY()*1e2)) >> 8) & 0xFF;
    msg.buf[3] = (((int16_t)(imu.getGyroY()*1e2)) >> 0) & 0xFF;
    msg.buf[4] = (((int16_t)(imu.getGyroZ()*1e2)) >> 8) & 0xFF;
    msg.buf[5] = (((int16_t)(imu.getGyroZ()*1e2)) >> 0) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    CANbus.write(msg);
  #endif
}
void sendMAG(void){
  #ifdef useMAG
    // Serial.print("magX : ");Serial.print(mag_data.magnetic.x);
    // Serial.print("\tmagY : ");Serial.print(mag_data.magnetic.y);
    // Serial.print("\tmagZ : ");Serial.print(mag_data.magnetic.z);
    // Serial.print('\n');
    // Serial.println("=======================================================\n");
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

#ifdef useRTK
void RTK_heartbeat_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
  msg_heartbeat_t* hbMsg = (msg_heartbeat_t*) msg;
  if ((hbMsg->flags>>0) & 1) 
    Serial.print("RTK: system error");
  if ((hbMsg->flags>>1) & 1) 
    Serial.print("RTK: IO error");
  if ((hbMsg->flags>>2) & 1) 
    Serial.print("RTK: swiftNAP error");
  if (!((hbMsg->flags>>31) & 1))
    Serial.print("RTK: no external antenna");
  RTKaliveTimer.reset();
  RTKisAlive = true;
}
void RTK_posLLH_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
  msg_pos_llh_t* posMsg = (msg_pos_llh_t*) msg;
  curPosLLH_RTK = *posMsg;
  GPSvalidTimer.reset();
  GPSisValid = true;
}
void RTK_velNED_cb(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
  msg_vel_ned_t* velMsg = (msg_vel_ned_t*) msg;
  curVelNED_RTK = *velMsg;
}
void RTK_printmsg(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context){
  Serial.print(sender_id);
  Serial.print('\t'); Serial.print(len);
  Serial.print('\t');
  for (uint8_t i = 0; i<len; i++){
    Serial.print('\t'); Serial.print(msg[i], HEX);
  }
  Serial.print('\n');
}

s32 RTK_readSerial(u8 *buff, u32 n, void *context){
  static s32 i;
  for (i=0; i<n; i++) {
    if (Serial1.available()){
      buff[i] = Serial1.read();
    }
    else
      break;
  }
  return i;
}
#endif