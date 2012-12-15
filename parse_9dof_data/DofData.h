#include "Arduino.h"

#ifndef DofData_h
#define DofData_h

/**
 * Structure of the sensor data.
 */
struct DofData {
  double accelX;
  double accelY;
  double accelZ;
  double magX;
  double magY;
  double magZ;
  double gyroX;
  double gyroY;
  double gyroZ;
};

/**
 * Structure for Euler angle data (yaw, pitch, roll).
 * (not technically Euler angles, but whatever).
 */
struct EulerData {
  double roll;
  double pitch;
  double yaw;
};

/**
 * Structure for Gyroscope angle data. Data has been scaled up 100 times.
 */
struct GyroData {
  int x, y, z;
  char checkSum;
};

#endif
