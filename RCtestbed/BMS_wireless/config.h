#ifndef CONFIG_H
#define CONFIG_H

typedef struct {
  int32_t LSteeringMotor;
  int32_t RSteeringMotor;
  uint16_t throttle;
  uint8_t brake;
} driveCommands_t;
#define STEER_HOMEPIN 22

#define NSIL 3
#define STANDBY 4

#define RmotorCAN 55
#define LmotorCAN 56
#define MmotorCAN 100 // main motor (throttle)
#define accCAN 200
#define gyroCAN 201
#define magCAN 202

#endif