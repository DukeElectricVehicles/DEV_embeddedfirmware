#ifndef CONFIG_H
#define CONFIG_H

typedef struct {
  float LSteeringMotor;
  float RSteeringMotor;
  uint16_t throttle;
  uint8_t brake;
} driveCommands_t;

#define STEER_HOMEPIN 22
#define STEER_LPF .9
#define ANGLE_LPF .99

#define AUTOSTORESIZE 500

#define NSIL 3
#define STANDBY 4

#define RmotorCAN 55
#define LmotorCAN 56
#define MmotorCAN 100 // main motor (throttle)
#define MmotorCANVESC 57
#define accCAN 200
#define gyroCAN 201
#define magCAN 202

#endif