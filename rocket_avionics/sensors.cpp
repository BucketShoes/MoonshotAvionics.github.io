// sensors.cpp — Global variable definitions for sensors.h
// Include this file in your Arduino project alongside sensors.h.

#include "sensors.h"

AccelData accelData  = {0, 0, 0, false, 0};
GyroData  gyroData   = {0, 0, 0, false, 0};
MagData   magData    = {0, 0, 0, false, 0};
BaroData  baroData   = {0, 0, 0, 0, false, false, 0, 0, 0};

SensorState sensorState = {
  false, false, MAG_NONE, false,  // accelOk, gyroOk, magType, baroOk
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false},  // bmpCal (BMP280Cal: 3 temp + 9 pressure + valid)
  BMP_IDLE, 0,                     // bmpPhase, bmpPhaseStartUs
  0, 0, 0, 0,                     // next poll timestamps
  0.0f, 0, 0.0f, false            // vvel state
};
